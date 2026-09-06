from __future__ import annotations

import ipaddress
import secrets
import time
from dataclasses import dataclass
from urllib.parse import urlsplit

from aiohttp import web


WEB_CONSENT_SESSIONS_KEY = "web_consent_sessions"
WEB_CONSENT_TOKEN_HEADER = "X-Carrot-Web-Consent"
WEB_REQUEST_MARKER_HEADER = "X-Carrot-Web-Request"
WEB_CONSENT_SESSION_TTL_SECONDS = 5 * 60
WEB_CONSENT_SESSION_LIMIT = 32

_LOCAL_IPV4_NETWORKS = tuple(ipaddress.ip_network(cidr) for cidr in (
  "10.0.0.0/8",
  "127.0.0.0/8",
  "169.254.0.0/16",
  "172.16.0.0/12",
  "192.168.0.0/16",
))
_LOCAL_IPV6_NETWORKS = tuple(ipaddress.ip_network(cidr) for cidr in (
  "::1/128",
  "fc00::/7",
  "fe80::/10",
))


@dataclass(frozen=True)
class _ConsentSession:
  scope: str
  peer: str
  expires_at: float


def initialize_web_consent_sessions(app: web.Application) -> None:
  """Create boot-local consent state; never persist or reuse it across boots."""
  app[WEB_CONSENT_SESSIONS_KEY] = {}


def _parse_authority(authority: str) -> tuple[str, int | None] | None:
  value = str(authority or "").strip()
  if not value or len(value) > 255 or any(char in value for char in "@/?#\\,"):
    return None
  if any(char.isspace() for char in value):
    return None
  try:
    parsed = urlsplit(f"//{value}")
    hostname = parsed.hostname
    port = parsed.port
  except (TypeError, ValueError):
    return None
  if not hostname or parsed.username is not None or parsed.password is not None:
    return None
  return hostname.rstrip(".").lower(), port


def _is_private_web_host(hostname: str) -> bool:
  """Carrot Web documents IP-literal access on the private vehicle network."""
  try:
    address = ipaddress.ip_address(hostname)
  except ValueError:
    return hostname == "localhost"
  networks = _LOCAL_IPV4_NETWORKS if address.version == 4 else _LOCAL_IPV6_NETWORKS
  return any(address in network for network in networks)


def _origin_scope(scheme: str, authority: str) -> str | None:
  normalized_scheme = str(scheme or "").strip().lower()
  if normalized_scheme not in {"http", "https"}:
    return None
  parsed = _parse_authority(authority)
  if parsed is None:
    return None
  hostname, port = parsed
  if not _is_private_web_host(hostname):
    return None
  default_port = 443 if normalized_scheme == "https" else 80
  effective_port = port if port is not None else default_port
  bracketed_host = f"[{hostname}]" if ":" in hostname else hostname
  return f"{normalized_scheme}://{bracketed_host}:{effective_port}"


def _request_scope(request: web.Request) -> str | None:
  return _origin_scope(request.scheme, request.headers.get("Host", ""))


def _normalized_ip(value: object) -> str | None:
  raw = str(value or "").strip()
  if not raw:
    return None
  # aiohttp can expose a scoped IPv6 address (for example fe80::1%wlan0).
  # The interface is already constrained by the kernel route table below.
  raw = raw.split("%", 1)[0]
  try:
    address = ipaddress.ip_address(raw)
  except ValueError:
    return None
  if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped is not None:
    address = address.ipv4_mapped
  return str(address)


def _request_peer(request: web.Request) -> str | None:
  """Return the TCP peer only; never trust forwarding headers on this server."""
  peer: object = None
  try:
    transport = request.transport
    if transport is not None:
      peer = transport.get_extra_info("peername")
  except Exception:
    peer = None
  if isinstance(peer, (tuple, list)) and peer:
    peer = peer[0]
  if peer is None:
    # Real aiohttp requests resolve remote from the transport. This fallback
    # also keeps the policy independently testable with a minimal request.
    try:
      peer = request.remote
    except Exception:
      peer = None
  return _normalized_ip(peer)


def _default_tether_gateway_addresses(
  ipv4_path: str = "/proc/net/route",
  ipv6_path: str = "/proc/net/ipv6_route",
) -> frozenset[str]:
  """Read active Wi-Fi default gateways without invoking a shell command."""
  candidates: dict[int, list[tuple[int, str]]] = {4: [], 6: []}
  try:
    with open(ipv4_path, encoding="ascii") as route_file:
      for line in route_file.readlines()[1:]:
        fields = line.split()
        if len(fields) < 8 or fields[1] != "00000000" or not fields[0].startswith("wlan"):
          continue
        flags = int(fields[3], 16)
        gateway_hex = fields[2]
        if (flags & 0x3) != 0x3 or gateway_hex == "00000000":
          continue
        gateway = _normalized_ip(ipaddress.IPv4Address(bytes.fromhex(gateway_hex)[::-1]))
        if gateway is not None:
          candidates[4].append((int(fields[6]), gateway))
  except (OSError, ValueError):
    pass

  try:
    with open(ipv6_path, encoding="ascii") as route_file:
      for line in route_file:
        fields = line.split()
        if (
          len(fields) < 10
          or fields[0] != "0" * 32
          or fields[1] != "00"
          or not fields[9].startswith("wlan")
        ):
          continue
        flags = int(fields[8], 16)
        gateway_hex = fields[4]
        if (flags & 0x3) != 0x3 or gateway_hex == "0" * 32:
          continue
        gateway = _normalized_ip(ipaddress.IPv6Address(int(gateway_hex, 16)))
        if gateway is not None:
          candidates[6].append((int(fields[5], 16), gateway))
  except (OSError, ValueError):
    pass

  # IPv4 and IPv6 metrics are not comparable. Accept only the lowest-metric
  # active default gateway in each family, not gateways from dormant routes.
  gateways: set[str] = set()
  for family_candidates in candidates.values():
    if family_candidates:
      lowest_metric = min(metric for metric, _gateway in family_candidates)
      gateways.update(gateway for metric, gateway in family_candidates if metric == lowest_metric)
  return frozenset(gateways)


def request_is_tether_owner(request: web.Request) -> bool:
  """Authenticate consent issuance to the device-local or tether-host peer."""
  peer = _request_peer(request)
  if peer is None:
    return False
  try:
    if ipaddress.ip_address(peer).is_loopback:
      return True
  except ValueError:
    return False
  return peer in _default_tether_gateway_addresses()


def _header_origin_scope(request: web.Request, header_name: str) -> str | None:
  value = str(request.headers.get(header_name, "") or "").strip()
  if not value or value == "null":
    return None
  try:
    parsed = urlsplit(value)
  except ValueError:
    return None
  if (
    parsed.scheme not in {"http", "https"}
    or not parsed.netloc
    or parsed.username is not None
    or parsed.password is not None
  ):
    return None
  return _origin_scope(parsed.scheme, parsed.netloc)


def request_has_same_origin(request: web.Request, *, require_origin: bool) -> bool:
  request_scope = _request_scope(request)
  if request_scope is None:
    return False

  fetch_site = str(request.headers.get("Sec-Fetch-Site", "") or "").strip().lower()
  if fetch_site and fetch_site != "same-origin":
    return False

  origin = str(request.headers.get("Origin", "") or "").strip()
  if origin:
    return _header_origin_scope(request, "Origin") == request_scope
  if require_origin:
    return False

  # Same-origin GET fetches are allowed to omit Origin. If a Referer is
  # supplied, it must still name the exact local origin that issued the token.
  referer = str(request.headers.get("Referer", "") or "").strip()
  return not referer or _header_origin_scope(request, "Referer") == request_scope


def _prune_sessions(
  sessions: dict[str, _ConsentSession],
  now: float,
  *,
  max_entries: int = WEB_CONSENT_SESSION_LIMIT,
) -> None:
  for token, session in list(sessions.items()):
    if session.expires_at <= now:
      sessions.pop(token, None)
  overflow = len(sessions) - max(0, max_entries)
  if overflow > 0:
    oldest = sorted(sessions.items(), key=lambda item: item[1].expires_at)[:overflow]
    for token, _session in oldest:
      sessions.pop(token, None)


def issue_web_consent_session(request: web.Request, *, now: float | None = None) -> str | None:
  if (
    not request_has_same_origin(request, require_origin=False)
    or not request_is_tether_owner(request)
  ):
    return None
  sessions = request.app.get(WEB_CONSENT_SESSIONS_KEY)
  if not isinstance(sessions, dict):
    return None
  scope = _request_scope(request)
  peer = _request_peer(request)
  if scope is None or peer is None:
    return None
  issued_at = time.monotonic() if now is None else float(now)
  _prune_sessions(sessions, issued_at, max_entries=WEB_CONSENT_SESSION_LIMIT - 1)
  token = secrets.token_urlsafe(32)
  sessions[token] = _ConsentSession(
    scope=scope,
    peer=peer,
    expires_at=issued_at + WEB_CONSENT_SESSION_TTL_SECONDS,
  )
  return token


def consume_web_consent_session(request: web.Request, *, now: float | None = None) -> bool:
  """Consume one same-origin consent token; every positive write needs a new one."""
  try:
    if request.content_type != "application/json":
      return False
    if request.headers.get(WEB_REQUEST_MARKER_HEADER) != "1":
      return False
    if not request_has_same_origin(request, require_origin=True):
      return False
    token = str(request.headers.get(WEB_CONSENT_TOKEN_HEADER, "") or "").strip()
    if not token or len(token) > 128:
      return False
    sessions = request.app.get(WEB_CONSENT_SESSIONS_KEY)
    if not isinstance(sessions, dict):
      return False
    checked_at = time.monotonic() if now is None else float(now)
    _prune_sessions(sessions, checked_at)
    session = sessions.pop(token, None)
    return bool(
      isinstance(session, _ConsentSession)
      and session.expires_at > checked_at
      and session.scope == _request_scope(request)
      and session.peer == _request_peer(request)
    )
  except Exception:
    return False
