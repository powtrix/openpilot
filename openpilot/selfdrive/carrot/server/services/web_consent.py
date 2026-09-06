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
  if not request_has_same_origin(request, require_origin=False):
    return None
  sessions = request.app.get(WEB_CONSENT_SESSIONS_KEY)
  if not isinstance(sessions, dict):
    return None
  scope = _request_scope(request)
  if scope is None:
    return None
  issued_at = time.monotonic() if now is None else float(now)
  _prune_sessions(sessions, issued_at, max_entries=WEB_CONSENT_SESSION_LIMIT - 1)
  token = secrets.token_urlsafe(32)
  sessions[token] = _ConsentSession(
    scope=scope,
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
    )
  except Exception:
    return False
