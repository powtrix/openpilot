from __future__ import annotations

import asyncio
import base64
import hashlib
import hmac
import ipaddress
import json
import logging
import math
import os
import re
import secrets
import shutil
import sqlite3
import time
from collections import OrderedDict, defaultdict, deque
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import UTC, datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Protocol

from aiohttp import web
from cryptography.exceptions import InvalidSignature, UnsupportedAlgorithm
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, padding, rsa


GIB = 1024 * 1024 * 1024
MIB = 1024 * 1024
DEVICE_RE = re.compile(r"^[A-Za-z0-9_-]{8,64}$")
SEGMENT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.|-]{0,127}$")
FILENAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
CAPTURE_RE = re.compile(r"^[0-9a-f]{32}$")
CHALLENGE_RE = re.compile(r"^[A-Za-z0-9_-]{20,96}$")
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
DEVICE_KEY_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
DEVICE_PROOF_RE = re.compile(r"^[A-Za-z0-9_-]{1,2048}$")
RECEIVER_PART_RE = re.compile(r"^\..+\.(?:[0-9a-f]{16}|[0-9a-f]{32})\.part$")
MANIFEST_PART_RE = re.compile(r"^\d{8}T\d{6}\.\d{6}Z\.json\.part$")
VALIDATION_RLOG_NAMES = frozenset({"rlog", "rlog.bz2", "rlog.zst"})
VALIDATION_NAMESPACE = ".carrot-validation-v1"
VALIDATION_PURPOSE = "validation"
LEGACY_QUOTA_NAMESPACE = "legacy"
DEVICE_AUTH_VERSION = 2
DEVICE_PROOF_DOMAIN = b"dk-carrot-validation-device-proof-v2\0"
RECEIPT_VERSION = 1
VALIDATION_COMPLETION_BODY_MAX = 32 * 1024
VALIDATION_MANIFEST_MAX = 16 * 1024
VALIDATION_MAX_FILES = 3
VALIDATION_CONDITIONS = frozenset({
  "standstill_off",
  "standstill_off_physical_res",
  "standstill_on",
  "lane_offset_0",
  "lane_offset_10",
  "standstill_on_no_request",
  "stock_scc_close_accel",
})
VALIDATION_TRIGGERS = frozenset({
  "keepalive_requested",
  "duration_no_keepalive_request",
  "duration",
  "stop_ended_before_keepalive_request",
  "stop_ended_early",
  "stable_lane_control",
  "accelerating_while_closing",
})
# Protocol-v1 automatic validation is intentionally scoped to one exact KA4
# stock-SCC topology. Keep these wire values explicit here: the receiver is a
# standalone service and must not gain the vehicle stack's runtime dependency
# closure merely to validate signed capture metadata.
KA4_STOCK_SCC_CAR_FINGERPRINT = "KIA_CARNIVAL_4TH_GEN"
HYUNDAI_FLAG_CANFD_HDA2 = 1 << 0
HYUNDAI_FLAG_CAMERA_SCC = 1 << 3
HYUNDAI_FLAG_CANFD = 1 << 13
HYUNDAI_FLAG_RADAR_SCC = 1 << 14
HYUNDAI_CANFD_SAFETY_MODEL = "hyundaiCanfd"
VALIDATION_CAPTURE_KEYS = frozenset({
  "schemaVersion",
  "campaignId",
  "captureId",
  "condition",
  "route",
  "segments",
  "anchorSegment",
  "duration",
  "trigger",
  "keepaliveRequestDelta",
  "qualified",
  "controllerStoppedSec",
  "vEgo",
  "aEgo",
  "leadDRel",
  "leadVRel",
  "timeGap",
  "ttc",
  "detectedAt",
  "settingsEpoch",
  "routeSettings",
  "git",
})
BLOCKED_EXTENSIONS = {
  ".apk", ".bat", ".cgi", ".cmd", ".com", ".dll", ".exe", ".html",
  ".htm", ".jar", ".js", ".php", ".ps1", ".py", ".sh", ".so",
}
KST = timezone(timedelta(hours=9))


def _env_int(name: str, default: int, minimum: int = 0) -> int:
  try:
    return max(minimum, int(os.environ.get(name, str(default))))
  except Exception:
    return default


def _env_float(name: str, default: float, minimum: float = 0.0) -> float:
  try:
    value = float(os.environ.get(name, str(default)))
    return max(minimum, value) if math.isfinite(value) else default
  except Exception:
    return default


def _env_bool(name: str, default: bool) -> bool:
  raw = os.environ.get(name)
  if raw is None:
    return default
  value = raw.strip().lower()
  if value in {"1", "true", "yes", "on"}:
    return True
  if value in {"0", "false", "no", "off"}:
    return False
  raise ValueError(f"{name} must be a boolean value")


def _env_allowed_device_ids(name: str = "CARROT_ALLOWED_DEVICE_IDS") -> frozenset[str]:
  raw = os.environ.get(name, "")
  values = frozenset(value for value in re.split(r"[,\s]+", raw.strip()) if value)
  invalid = sorted(
    value for value in values
    if not DEVICE_RE.fullmatch(value) or value.lower() in {"unknown", "none"}
  )
  if invalid:
    raise ValueError(f"{name} contains an invalid device ID")
  return values


def _env_device_public_key_pins(
  name: str = "CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256",
) -> dict[str, str]:
  raw = os.environ.get(name, "")
  pins: dict[str, str] = {}
  for entry in (value for value in re.split(r"[,\s]+", raw.strip()) if value):
    if entry.count("=") != 1:
      raise ValueError(f"{name} must contain deviceId=lowercase-sha256 entries")
    device, fingerprint = entry.split("=", 1)
    if (
      not DEVICE_RE.fullmatch(device)
      or device.lower() in {"unknown", "none"}
      or not DEVICE_KEY_SHA256_RE.fullmatch(fingerprint)
      or device in pins
    ):
      raise ValueError(f"{name} contains an invalid or duplicate device key pin")
    pins[device] = fingerprint
  return pins


@dataclass(frozen=True)
class Config:
  storage_root: Path
  db_path: Path
  allowed_device_ids: frozenset[str] = frozenset()
  allowed_device_public_key_sha256: Mapping[str, str] = field(default_factory=dict)
  legacy_uploads_enabled: bool = False
  daily_device_quota: int = GIB
  daily_ip_quota: int = 8 * GIB
  max_file_bytes: int = 512 * MIB
  max_tmux_bytes: int = 16 * MIB
  min_free_bytes: int = 10 * GIB
  validation_free_space_reserve_bytes: int = GIB
  session_ttl_seconds: int = 4 * 60 * 60
  concurrent_per_device: int = 3
  concurrent_global: int = 16
  validation_concurrent_per_device: int = 2
  validation_concurrent_global: int = 8
  upload_idle_timeout_seconds: float = 60.0
  upload_total_timeout_seconds: float = 6 * 60 * 60
  json_body_idle_timeout_seconds: float = 10.0
  json_body_total_timeout_seconds: float = 30.0
  request_concurrent_per_ip: int = 4
  request_concurrent_global: int = 16
  validation_request_concurrent_per_ip: int = 4
  validation_request_concurrent_global: int = 16
  progress_commit_bytes: int = MIB
  progress_commit_interval_seconds: float = 1.0
  session_issue_limit: int = 30
  session_issue_window_seconds: int = 10 * 60
  session_rate_bucket_limit: int = 4096
  trusted_proxy_networks: tuple[str, ...] = ("127.0.0.0/8", "::1/128", "172.16.0.0/12")
  validation_challenge_ttl_seconds: int = 5 * 60
  validation_session_ttl_seconds: int = 30 * 60
  validation_audience: str = "carrot-validation-upload-v1"
  validation_verify_timeout_seconds: float = 5.0
  validation_verify_concurrent: int = 4
  validation_verify_attempt_limit: int = 5
  validation_verify_cooldown_seconds: int = 2
  reservation_lease_seconds: int = 2 * 60 * 60
  stale_part_seconds: int = 2 * 60 * 60
  validation_hash_concurrent: int = 2
  cleanup_interval_seconds: int = 15 * 60

  @classmethod
  def from_env(cls) -> Config:
    root = Path(os.environ.get("CARROT_UPLOAD_ROOT", "/data/uploads"))
    return cls(
      storage_root=root,
      db_path=Path(os.environ.get("CARROT_UPLOAD_DB", "/data/state/uploads.sqlite3")),
      allowed_device_ids=_env_allowed_device_ids(),
      allowed_device_public_key_sha256=_env_device_public_key_pins(),
      legacy_uploads_enabled=_env_bool("CARROT_LEGACY_UPLOADS_ENABLED", False),
      daily_device_quota=_env_int("CARROT_DAILY_DEVICE_QUOTA_BYTES", GIB, MIB),
      daily_ip_quota=_env_int("CARROT_DAILY_IP_QUOTA_BYTES", 8 * GIB, MIB),
      max_file_bytes=_env_int("CARROT_MAX_FILE_BYTES", 512 * MIB, MIB),
      max_tmux_bytes=_env_int("CARROT_MAX_TMUX_BYTES", 16 * MIB, MIB),
      min_free_bytes=_env_int("CARROT_MIN_FREE_BYTES", 10 * GIB),
      validation_free_space_reserve_bytes=_env_int(
        "CARROT_VALIDATION_FREE_SPACE_RESERVE_BYTES", GIB,
      ),
      session_ttl_seconds=_env_int("CARROT_SESSION_TTL_SECONDS", 4 * 60 * 60, 60),
      concurrent_per_device=_env_int("CARROT_CONCURRENT_PER_DEVICE", 3, 1),
      concurrent_global=_env_int("CARROT_CONCURRENT_GLOBAL", 16, 1),
      validation_concurrent_per_device=_env_int("CARROT_VALIDATION_CONCURRENT_PER_DEVICE", 2, 1),
      validation_concurrent_global=_env_int("CARROT_VALIDATION_CONCURRENT_GLOBAL", 8, 1),
      upload_idle_timeout_seconds=_env_float("CARROT_UPLOAD_IDLE_TIMEOUT_SECONDS", 60.0, 1.0),
      upload_total_timeout_seconds=_env_float("CARROT_UPLOAD_TOTAL_TIMEOUT_SECONDS", 6 * 60 * 60, 60.0),
      json_body_idle_timeout_seconds=_env_float("CARROT_JSON_BODY_IDLE_TIMEOUT_SECONDS", 10.0, 1.0),
      json_body_total_timeout_seconds=_env_float("CARROT_JSON_BODY_TOTAL_TIMEOUT_SECONDS", 30.0, 1.0),
      request_concurrent_per_ip=_env_int("CARROT_REQUEST_CONCURRENT_PER_IP", 4, 1),
      request_concurrent_global=_env_int("CARROT_REQUEST_CONCURRENT_GLOBAL", 16, 1),
      validation_request_concurrent_per_ip=_env_int(
        "CARROT_VALIDATION_REQUEST_CONCURRENT_PER_IP", 4, 1,
      ),
      validation_request_concurrent_global=_env_int(
        "CARROT_VALIDATION_REQUEST_CONCURRENT_GLOBAL", 16, 1,
      ),
      progress_commit_bytes=_env_int("CARROT_PROGRESS_COMMIT_BYTES", MIB, 4096),
      progress_commit_interval_seconds=_env_float(
        "CARROT_PROGRESS_COMMIT_INTERVAL_SECONDS", 1.0, 0.1,
      ),
      session_rate_bucket_limit=_env_int("CARROT_SESSION_RATE_BUCKET_LIMIT", 4096, 16),
      validation_challenge_ttl_seconds=_env_int("CARROT_VALIDATION_CHALLENGE_TTL_SECONDS", 5 * 60, 30),
      validation_session_ttl_seconds=_env_int("CARROT_VALIDATION_SESSION_TTL_SECONDS", 30 * 60, 60),
      validation_audience=os.environ.get("CARROT_VALIDATION_AUDIENCE", "carrot-validation-upload-v1"),
      validation_verify_timeout_seconds=_env_float("CARROT_VALIDATION_VERIFY_TIMEOUT_SECONDS", 5.0, 0.5),
      validation_verify_concurrent=_env_int("CARROT_VALIDATION_VERIFY_CONCURRENT", 4, 1),
      validation_verify_attempt_limit=_env_int("CARROT_VALIDATION_VERIFY_ATTEMPT_LIMIT", 5, 1),
      validation_verify_cooldown_seconds=_env_int("CARROT_VALIDATION_VERIFY_COOLDOWN_SECONDS", 2, 1),
      reservation_lease_seconds=_env_int("CARROT_RESERVATION_LEASE_SECONDS", 2 * 60 * 60, 60),
      stale_part_seconds=_env_int("CARROT_STALE_PART_SECONDS", 2 * 60 * 60, 60),
      validation_hash_concurrent=_env_int("CARROT_VALIDATION_HASH_CONCURRENT", 2, 1),
      cleanup_interval_seconds=_env_int("CARROT_CLEANUP_INTERVAL_SECONDS", 15 * 60, 60),
    )


@dataclass(frozen=True)
class UploadReservation:
  reservation_id: str
  usage_day: str
  device_id: str
  source_ip: str
  reserved_bytes: int
  reserved_storage_bytes: int
  quota_namespace: str


@dataclass
class StreamProgress:
  received_bytes: int = 0
  disk_written_bytes: int = 0
  stored_bytes: int = 0
  persisted_received_bytes: int = 0
  persisted_disk_written_bytes: int = 0
  persisted_stored_bytes: int = 0
  last_persisted_at: float = 0.0
  persisted_once: bool = False


class DeviceProofVerificationRejected(Exception):
  """The supplied key or signature did not match the receiver's device pin."""


class DeviceProofVerificationUnavailable(Exception):
  """The local cryptographic verifier failed without an authentication result."""


class DeviceProofVerifier(Protocol):
  async def verify(
    self,
    *,
    algorithm: str,
    public_key_pem: str,
    proof: str,
    message: bytes,
    expected_fingerprint: str,
    expected_device_id: str,
  ) -> str:
    """Return the authenticated device ID or raise a verification exception."""


class PinnedDeviceProofVerifier:
  """Verify a challenge signature against an exact SHA-256 SPKI key pin."""

  MAX_PUBLIC_KEY_PEM_BYTES = 8 * 1024

  @staticmethod
  def _decode_proof(value: str) -> bytes:
    if not DEVICE_PROOF_RE.fullmatch(value):
      raise DeviceProofVerificationRejected("device proof is not unpadded base64url")
    try:
      return base64.b64decode(
        value + "=" * (-len(value) % 4),
        altchars=b"-_",
        validate=True,
      )
    except (ValueError, TypeError) as exc:
      raise DeviceProofVerificationRejected("device proof is not unpadded base64url") from exc

  @classmethod
  def _verify_sync(
    cls,
    *,
    algorithm: str,
    public_key_pem: str,
    proof: str,
    message: bytes,
    expected_fingerprint: str,
    expected_device_id: str,
  ) -> str:
    try:
      public_key_bytes = public_key_pem.encode("ascii")
    except UnicodeEncodeError as exc:
      raise DeviceProofVerificationRejected("device public key must be ASCII PEM") from exc
    public_key_block = public_key_bytes.strip()
    if (
      not public_key_bytes
      or len(public_key_bytes) > cls.MAX_PUBLIC_KEY_PEM_BYTES
      or b"\0" in public_key_bytes
      or not public_key_block.startswith((b"-----BEGIN PUBLIC KEY-----\n", b"-----BEGIN PUBLIC KEY-----\r\n"))
      or not public_key_block.endswith(b"-----END PUBLIC KEY-----")
    ):
      raise DeviceProofVerificationRejected("device public key PEM is invalid")
    try:
      public_key = serialization.load_pem_public_key(public_key_bytes)
    except (TypeError, ValueError, UnsupportedAlgorithm) as exc:
      raise DeviceProofVerificationRejected("device public key PEM is invalid") from exc

    public_der = public_key.public_bytes(
      serialization.Encoding.DER,
      serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    fingerprint = hashlib.sha256(public_der).hexdigest()
    if not hmac.compare_digest(fingerprint, expected_fingerprint):
      raise DeviceProofVerificationRejected("device public key does not match its receiver pin")

    signature = cls._decode_proof(proof)
    try:
      if algorithm == "RS256":
        if not isinstance(public_key, rsa.RSAPublicKey) or public_key.key_size < 2048:
          raise DeviceProofVerificationRejected("RS256 requires an RSA key of at least 2048 bits")
        public_key.verify(signature, message, padding.PKCS1v15(), hashes.SHA256())
      elif algorithm == "ES256":
        if not isinstance(public_key, ec.EllipticCurvePublicKey) or not isinstance(public_key.curve, ec.SECP256R1):
          raise DeviceProofVerificationRejected("ES256 requires a P-256 EC key")
        # cryptography's ECDSA API and the device client exchange the standard
        # ASN.1 DER-encoded (r, s) signature, not the JOSE 64-byte form.
        public_key.verify(signature, message, ec.ECDSA(hashes.SHA256()))
      else:
        raise DeviceProofVerificationRejected("unsupported device key algorithm")
    except InvalidSignature as exc:
      raise DeviceProofVerificationRejected("device proof signature is invalid") from exc
    return expected_device_id

  async def verify(self, **kwargs: Any) -> str:
    try:
      return await asyncio.to_thread(self._verify_sync, **kwargs)
    except DeviceProofVerificationRejected:
      raise
    except Exception as exc:
      raise DeviceProofVerificationUnavailable("local device proof verification failed closed") from exc


class UploadService:
  def __init__(self, config: Config, *, validation_verifier: DeviceProofVerifier | None = None):
    self.config = config
    # Snapshot deployment policy so a mutable mapping supplied by an embedding
    # process cannot change the authentication boundary behind active sessions.
    self._device_key_pins = dict(config.allowed_device_public_key_sha256)
    self._mkdir_parents_durable(self.config.storage_root)
    self._mkdir_parents_durable(self._path(VALIDATION_NAMESPACE))
    self._mkdir_parents_durable(self.config.db_path.parent)
    self._db_lock = asyncio.Lock()
    self._active_lock = asyncio.Lock()
    self._active_global = 0
    self._active_by_device: dict[str, int] = defaultdict(int)
    self._validation_active_global = 0
    self._validation_active_by_device: dict[str, int] = defaultdict(int)
    self._request_active_global = 0
    self._validation_request_active_global = 0
    self._request_active_by_ip: dict[str, int] = defaultdict(int)
    self._validation_request_active_by_ip: dict[str, int] = defaultdict(int)
    self._session_issues: dict[str, OrderedDict[str, deque[float]]] = {
      LEGACY_QUOTA_NAMESPACE: OrderedDict(),
      VALIDATION_PURPOSE: OrderedDict(),
    }
    self._validation_verifier_semaphore = asyncio.Semaphore(config.validation_verify_concurrent)
    self._validation_hash_semaphore = asyncio.Semaphore(config.validation_hash_concurrent)
    self._validation_storage_probe_at = 0.0
    self._validation_storage_probe_ok = False
    self._trusted_proxies = tuple(ipaddress.ip_network(value) for value in config.trusted_proxy_networks)
    self._validation_verifier = validation_verifier or PinnedDeviceProofVerifier()
    self._init_db()
    self._fsync_directory(self.config.db_path.parent)
    self._reconcile_startup()

  def _connect(self) -> sqlite3.Connection:
    connection = sqlite3.connect(self.config.db_path, timeout=30)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA foreign_keys=ON")
    connection.execute("PRAGMA synchronous=FULL")
    return connection

  def _init_db(self) -> None:
    with self._connect() as connection:
      connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS sessions (
          token_hash TEXT PRIMARY KEY,
          device_id TEXT NOT NULL,
          source_ip TEXT NOT NULL,
          purpose TEXT NOT NULL,
          car_name TEXT NOT NULL DEFAULT 'none',
          git_branch TEXT NOT NULL DEFAULT 'unknown',
          tmux_reason TEXT NOT NULL DEFAULT 'tmux',
          device_key_sha256 TEXT NOT NULL DEFAULT '',
          created_at INTEGER NOT NULL,
          expires_at INTEGER NOT NULL
        );
        CREATE INDEX IF NOT EXISTS sessions_expiry ON sessions(expires_at);
        CREATE TABLE IF NOT EXISTS daily_usage (
          usage_day TEXT NOT NULL,
          scope TEXT NOT NULL,
          scope_id TEXT NOT NULL,
          committed_bytes INTEGER NOT NULL DEFAULT 0,
          reserved_bytes INTEGER NOT NULL DEFAULT 0,
          stored_bytes INTEGER NOT NULL DEFAULT 0,
          PRIMARY KEY (usage_day, scope, scope_id)
        );
        CREATE TABLE IF NOT EXISTS upload_reservations (
          reservation_id TEXT PRIMARY KEY,
          usage_day TEXT NOT NULL,
          device_id TEXT NOT NULL,
          source_ip TEXT NOT NULL,
          reserved_bytes INTEGER NOT NULL,
          reserved_storage_bytes INTEGER NOT NULL DEFAULT 0,
          quota_namespace TEXT NOT NULL DEFAULT 'legacy',
          received_bytes INTEGER NOT NULL DEFAULT 0,
          disk_written_bytes INTEGER NOT NULL DEFAULT 0,
          stored_bytes INTEGER NOT NULL DEFAULT 0,
          created_at INTEGER NOT NULL,
          touched_at INTEGER NOT NULL
        );
        CREATE INDEX IF NOT EXISTS upload_reservations_touched
          ON upload_reservations(touched_at);
        CREATE TABLE IF NOT EXISTS validation_challenges (
          challenge_id TEXT PRIMARY KEY,
          nonce TEXT NOT NULL,
          device_id TEXT NOT NULL,
          audience TEXT NOT NULL,
          source_ip TEXT NOT NULL,
          created_at INTEGER NOT NULL,
          expires_at INTEGER NOT NULL,
          consumed_at INTEGER,
          session_token_hash TEXT,
          verification_started_at INTEGER,
          verification_failures INTEGER NOT NULL DEFAULT 0,
          retry_after INTEGER NOT NULL DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS validation_challenges_expiry
          ON validation_challenges(expires_at);
        CREATE TABLE IF NOT EXISTS validation_files (
          device_id TEXT NOT NULL,
          capture_id TEXT NOT NULL,
          segment TEXT NOT NULL,
          filename TEXT NOT NULL,
          size INTEGER NOT NULL,
          sha256 TEXT NOT NULL,
          relative_path TEXT NOT NULL,
          received_at INTEGER NOT NULL,
          PRIMARY KEY (device_id, capture_id, segment, filename)
        );
        CREATE TABLE IF NOT EXISTS validation_completions (
          device_id TEXT NOT NULL,
          capture_id TEXT NOT NULL,
          receipt_id TEXT NOT NULL UNIQUE,
          manifest_sha256 TEXT NOT NULL,
          manifest_json BLOB NOT NULL,
          relative_path TEXT NOT NULL,
          completed_at INTEGER NOT NULL,
          PRIMARY KEY (device_id, capture_id)
        );
        """
      )
      columns = {str(row[1]) for row in connection.execute("PRAGMA table_info(sessions)")}
      for name, definition in (
        ("car_name", "TEXT NOT NULL DEFAULT 'none'"),
        ("git_branch", "TEXT NOT NULL DEFAULT 'unknown'"),
        ("tmux_reason", "TEXT NOT NULL DEFAULT 'tmux'"),
        ("device_key_sha256", "TEXT NOT NULL DEFAULT ''"),
      ):
        if name not in columns:
          connection.execute(f"ALTER TABLE sessions ADD COLUMN {name} {definition}")
      usage_columns = {str(row[1]) for row in connection.execute("PRAGMA table_info(daily_usage)")}
      if "stored_bytes" not in usage_columns:
        connection.execute("ALTER TABLE daily_usage ADD COLUMN stored_bytes INTEGER NOT NULL DEFAULT 0")
      reservation_columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(upload_reservations)")
      }
      if "reserved_storage_bytes" not in reservation_columns:
        connection.execute(
          "ALTER TABLE upload_reservations ADD COLUMN reserved_storage_bytes INTEGER NOT NULL DEFAULT 0",
        )
        connection.execute(
          "UPDATE upload_reservations SET reserved_storage_bytes=reserved_bytes",
        )
      if "disk_written_bytes" not in reservation_columns:
        connection.execute(
          "ALTER TABLE upload_reservations ADD COLUMN disk_written_bytes INTEGER NOT NULL DEFAULT 0",
        )
        connection.execute(
          "UPDATE upload_reservations SET disk_written_bytes=received_bytes",
        )
      if "quota_namespace" not in reservation_columns:
        connection.execute(
          "ALTER TABLE upload_reservations ADD COLUMN quota_namespace TEXT NOT NULL DEFAULT 'legacy'",
        )
      challenge_columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(validation_challenges)")
      }
      for name, definition in (
        ("verification_started_at", "INTEGER"),
        ("verification_failures", "INTEGER NOT NULL DEFAULT 0"),
        ("retry_after", "INTEGER NOT NULL DEFAULT 0"),
      ):
        if name not in challenge_columns:
          connection.execute(f"ALTER TABLE validation_challenges ADD COLUMN {name} {definition}")

  @staticmethod
  def _token_hash(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()

  @staticmethod
  def _usage_day() -> str:
    return datetime.now(UTC).strftime("%Y-%m-%d")

  @staticmethod
  def _clean_metadata(value: Any) -> str:
    text = str(value or "").strip().replace("\r", " ").replace("\n", " ")
    return text[:160]

  @classmethod
  def _storage_component(cls, value: Any, default: str, *, branch: bool = False) -> str:
    text = cls._clean_metadata(value)
    if branch:
      text = text.replace("/", "__").replace("\\", "__")
    text = re.sub(r"[^A-Za-z0-9_. -]+", "_", text).strip(" .")[:96]
    return text if text and text not in {".", ".."} else default

  @staticmethod
  def _storage_directory(session: sqlite3.Row) -> str:
    return f"{session['car_name']} {session['device_id']}"

  def source_ip(self, request: web.Request) -> str:
    remote = str(request.remote or "unknown")
    try:
      remote_ip = ipaddress.ip_address(remote)
      trusted = any(remote_ip in network for network in self._trusted_proxies)
    except ValueError:
      trusted = False
    if trusted:
      # DSM/nginx appends the real client to any inbound X-Forwarded-For
      # values. Walk from the proxy-facing end so a caller cannot spoof the
      # first value to evade IP quotas or session binding.
      forwarded = request.headers.get("X-Forwarded-For", "")
      for value in reversed(forwarded.split(",")):
        try:
          candidate = ipaddress.ip_address(value.strip())
        except ValueError:
          continue
        if not any(candidate in network for network in self._trusted_proxies):
          return str(candidate)
    return remote[:64]

  def _validate_device(self, value: Any) -> str:
    device = str(value or "").strip()
    if not DEVICE_RE.fullmatch(device) or device.lower() in {"unknown", "none"}:
      raise web.HTTPBadRequest(text="invalid device id")
    return device

  def _require_allowed_device(self, device: str) -> str:
    if device not in self.config.allowed_device_ids:
      raise web.HTTPForbidden(text="device is not allowed by receiver policy")
    return device

  def _require_validation_device_key_pin(self, device: str) -> str:
    self._require_allowed_device(device)
    fingerprint = self._device_key_pins.get(device)
    if not isinstance(fingerprint, str) or not DEVICE_KEY_SHA256_RE.fullmatch(fingerprint):
      raise web.HTTPServiceUnavailable(text="device public key pin is not configured")
    return fingerprint

  def _require_legacy_uploads_enabled(self) -> None:
    if not self.config.legacy_uploads_enabled:
      raise web.HTTPForbidden(text="legacy uploads are disabled")

  @staticmethod
  def _validate_segment(value: Any) -> str:
    segment = str(value or "").strip()
    if not SEGMENT_RE.fullmatch(segment) or segment in {".", ".."}:
      raise web.HTTPBadRequest(text="invalid segment")
    return segment

  @staticmethod
  def _validate_filename(value: Any) -> str:
    filename = str(value or "").strip()
    if not FILENAME_RE.fullmatch(filename) or filename in {".", ".."}:
      raise web.HTTPBadRequest(text="invalid filename")
    if Path(filename).suffix.lower() in BLOCKED_EXTENSIONS:
      raise web.HTTPBadRequest(text="file type is not allowed")
    return filename

  @staticmethod
  def _validate_capture(value: Any) -> str:
    capture = str(value or "").strip()
    if not CAPTURE_RE.fullmatch(capture) or capture in {".", ".."}:
      raise web.HTTPBadRequest(text="invalid capture id")
    return capture

  @staticmethod
  def _validate_challenge(value: Any) -> str:
    challenge = str(value or "").strip()
    if not CHALLENGE_RE.fullmatch(challenge):
      raise web.HTTPBadRequest(text="invalid challenge id")
    return challenge

  @staticmethod
  def _validate_sha256(value: Any) -> str:
    digest = str(value or "").strip().lower()
    if not SHA256_RE.fullmatch(digest):
      raise web.HTTPBadRequest(text="invalid SHA-256")
    return digest

  @classmethod
  def _validate_validation_filename(cls, value: Any) -> str:
    filename = cls._validate_filename(value)
    if filename not in VALIDATION_RLOG_NAMES:
      raise web.HTTPBadRequest(text="validation uploads accept rlog files only")
    return filename

  @staticmethod
  def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
      os.fsync(descriptor)
    finally:
      os.close(descriptor)

  @classmethod
  def _mkdir_parents_durable(cls, path: Path) -> None:
    """Create every missing directory and durably record each parent entry."""
    path = path.absolute()
    missing: list[Path] = []
    cursor = path
    while not cursor.exists():
      missing.append(cursor)
      if cursor.parent == cursor:
        break
      cursor = cursor.parent
    for directory in reversed(missing):
      directory.mkdir(exist_ok=True)
      cls._fsync_directory(directory.parent)

  @classmethod
  def _unlink_durable(cls, path: Path) -> None:
    try:
      path.unlink()
    except FileNotFoundError:
      return
    cls._fsync_directory(path.parent)

  @staticmethod
  def _validation_device_proof_message(
    device: str,
    challenge_id: str,
    nonce: str,
    audience: str,
  ) -> bytes:
    # Every field has already passed its endpoint-specific length/alphabet
    # validation, and the NUL separators make the tuple unambiguous.
    return DEVICE_PROOF_DOMAIN + b"\0".join(
      value.encode("utf-8") for value in (device, challenge_id, nonce, audience)
    )

  @staticmethod
  def _canonical_json(value: Any) -> bytes:
    try:
      return (json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
      ) + "\n").encode("utf-8")
    except (TypeError, ValueError) as exc:
      raise web.HTTPBadRequest(text="manifest contains invalid JSON values") from exc

  @staticmethod
  def _hash_file(path: Path) -> tuple[int, str]:
    before = path.stat()
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as source:
      while chunk := source.read(MIB):
        size += len(chunk)
        digest.update(chunk)
    after = path.stat()
    before_identity = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    after_identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if before_identity != after_identity or size != after.st_size:
      raise OSError("validation file changed while hashing")
    return size, digest.hexdigest()

  def _retain_validation_operation(self, device: str) -> None:
    # Called only from the event-loop thread. There is no await between these
    # mutations, so they remain atomic with the lock-protected enter/leave
    # methods and keep a cancelled handler's background hash represented.
    self._validation_active_global += 1
    self._validation_active_by_device[device] += 1

  def _release_retained_validation_operation(self, device: str) -> None:
    self._validation_active_global = max(0, self._validation_active_global - 1)
    self._validation_active_by_device[device] = max(
      0, self._validation_active_by_device[device] - 1,
    )
    if not self._validation_active_by_device[device]:
      self._validation_active_by_device.pop(device, None)

  async def _hash_file_bounded(
    self,
    path: Path,
    *,
    device: str | None = None,
  ) -> tuple[int, str]:
    await self._validation_hash_semaphore.acquire()
    worker = asyncio.create_task(asyncio.to_thread(self._hash_file, path))
    deferred_release = False
    try:
      return await asyncio.shield(worker)
    except asyncio.CancelledError:
      # Cancelling asyncio.to_thread cannot stop its synchronous filesystem
      # read. Keep both the hash semaphore and, for request paths, one
      # validation operation slot until that thread really exits. The request
      # task itself remains responsive and receives cancellation immediately.
      if device is not None:
        self._retain_validation_operation(device)

      def release_after_worker(done: asyncio.Task[tuple[int, str]]) -> None:
        try:
          done.result()
        except BaseException:
          # Retrieve any eventual worker exception so the detached task never
          # emits an unhandled-exception warning.
          pass
        self._validation_hash_semaphore.release()
        if device is not None:
          self._release_retained_validation_operation(device)

      worker.add_done_callback(release_after_worker)
      deferred_release = True
      raise
    finally:
      if not deferred_release:
        self._validation_hash_semaphore.release()

  def _path(self, *parts: str) -> Path:
    root = self.config.storage_root.resolve()
    path = root.joinpath(*parts).resolve()
    if path != root and root not in path.parents:
      raise web.HTTPBadRequest(text="invalid upload path")
    return path

  @staticmethod
  def _quota_scope_pairs(
    namespace: str,
    device: str,
    source_ip: str,
  ) -> tuple[tuple[str, str], tuple[str, str]]:
    if namespace == VALIDATION_PURPOSE:
      return (("validation_device", device), ("validation_ip", source_ip))
    return (("device", device), ("ip", source_ip))

  @staticmethod
  def _quota_scope_reservation(scope: str) -> tuple[str, str] | None:
    return {
      "device": ("device_id", LEGACY_QUOTA_NAMESPACE),
      "ip": ("source_ip", LEGACY_QUOTA_NAMESPACE),
      "validation_device": ("device_id", VALIDATION_PURPOSE),
      "validation_ip": ("source_ip", VALIDATION_PURPOSE),
    }.get(scope)

  def _has_disk_space(
    self,
    expected: int = 0,
    reserved: int = 0,
    *,
    validation: bool = True,
  ) -> bool:
    disk_path = self._path(VALIDATION_NAMESPACE) if validation else self.config.storage_root
    free = shutil.disk_usage(disk_path).free
    protected = 0 if validation else self.config.validation_free_space_reserve_bytes
    return free - max(0, reserved) - max(0, expected) >= self.config.min_free_bytes + protected

  def _validation_storage_writable(self, *, cache_seconds: float = 10.0) -> bool:
    """Probe the only writable upload namespace without exposing its contents."""
    now = time.monotonic()
    if self._validation_storage_probe_at > 0.0 and now - self._validation_storage_probe_at < cache_seconds:
      return self._validation_storage_probe_ok

    root = self._path(VALIDATION_NAMESPACE)
    probe = root / f".health.{secrets.token_hex(16)}.part"
    ready = False
    try:
      self._mkdir_parents_durable(root)
      with probe.open("xb") as output:
        output.write(b"dk-upload-health-v1\n")
        output.flush()
        os.fsync(output.fileno())
      self._unlink_durable(probe)
      ready = True
    except OSError:
      logging.exception("validation storage write probe failed")
      try:
        self._unlink_durable(probe)
      except OSError:
        pass

    self._validation_storage_probe_ok = ready
    self._validation_storage_probe_at = now
    return ready

  @staticmethod
  def _reconcile_stale_reservations_connection(
    connection: sqlite3.Connection, *, stale_before: int | None,
  ) -> int:
    rows = connection.execute(
      "SELECT * FROM upload_reservations"
      if stale_before is None
      else "SELECT * FROM upload_reservations WHERE touched_at < ?",
      () if stale_before is None else (stale_before,),
    ).fetchall()
    for row in rows:
      network_bytes = max(0, int(row["received_bytes"]))
      stored_bytes = max(0, int(row["stored_bytes"]))
      reserved_bytes = max(0, int(row["reserved_bytes"]))
      namespace = (
        VALIDATION_PURPOSE
        if str(row["quota_namespace"]) == VALIDATION_PURPOSE
        else LEGACY_QUOTA_NAMESPACE
      )
      for scope, scope_id in UploadService._quota_scope_pairs(
        namespace,
        str(row["device_id"]),
        str(row["source_ip"]),
      ):
        connection.execute(
          """UPDATE daily_usage
             SET reserved_bytes=MAX(0, reserved_bytes-?),
                 committed_bytes=MAX(0, committed_bytes+?),
                 stored_bytes=MAX(0, stored_bytes+?)
             WHERE usage_day=? AND scope=? AND scope_id=?""",
          (reserved_bytes, network_bytes, stored_bytes, row["usage_day"], scope, scope_id),
        )
      connection.execute(
        "DELETE FROM upload_reservations WHERE reservation_id=?", (row["reservation_id"],),
      )
    usage_rows = connection.execute(
      "SELECT usage_day, scope, scope_id FROM daily_usage",
    ).fetchall()
    for usage in usage_rows:
      reservation_scope = UploadService._quota_scope_reservation(str(usage["scope"]))
      if reservation_scope is None:
        continue
      identity_column, namespace = reservation_scope
      actual_reserved = int(connection.execute(
        f"""SELECT COALESCE(SUM(reserved_bytes), 0) FROM upload_reservations
            WHERE usage_day=? AND {identity_column}=? AND quota_namespace=?""",
        (usage["usage_day"], usage["scope_id"], namespace),
      ).fetchone()[0])
      connection.execute(
        """UPDATE daily_usage SET reserved_bytes=?
           WHERE usage_day=? AND scope=? AND scope_id=?""",
        (actual_reserved, usage["usage_day"], usage["scope"], usage["scope_id"]),
      )
    return len(rows)

  def _reconcile_stale_reservations_sync(self, now: int) -> int:
    with self._connect() as connection:
      connection.execute("BEGIN IMMEDIATE")
      return self._reconcile_stale_reservations_connection(
        connection, stale_before=now - self.config.reservation_lease_seconds,
      )

  def _reconcile_startup_reservations_sync(self) -> int:
    with self._connect() as connection:
      connection.execute("BEGIN IMMEDIATE")
      # This service intentionally runs as one process. Any persisted lease at
      # construction time therefore belongs to the previous crashed process,
      # regardless of how recently it was touched.
      return self._reconcile_stale_reservations_connection(connection, stale_before=None)

  def _cleanup_stale_parts_sync(self, now: int, *, startup: bool = False) -> int:
    removed = 0
    roots = [self.config.storage_root / VALIDATION_NAMESPACE]
    if self.config.legacy_uploads_enabled:
      roots.extend((
        self.config.storage_root / "routes",
        self.config.db_path.parent / "manifests",
      ))
    stale_before = now - self.config.stale_part_seconds
    for root in roots:
      if not root.is_dir():
        continue
      for part in root.rglob("*.part"):
        try:
          stat = part.lstat()
          receiver_owned = RECEIVER_PART_RE.fullmatch(part.name) or (
            root.name == "manifests" and MANIFEST_PART_RE.fullmatch(part.name)
          )
          if (
            not receiver_owned
            or not part.is_file()
            or (not startup and stat.st_mtime >= stale_before)
          ):
            continue
          self._unlink_durable(part)
          removed += 1
        except FileNotFoundError:
          continue
        except OSError:
          logging.exception("could not remove stale upload part %s", part)
    return removed

  def _repair_completion_manifests_sync(self) -> tuple[int, int]:
    repaired = 0
    unavailable = 0
    with self._connect() as connection:
      rows = connection.execute("SELECT * FROM validation_completions").fetchall()
    for row in rows:
      device = str(row["device_id"])
      capture = str(row["capture_id"])
      encoded = bytes(row["manifest_json"])
      if (
        not DEVICE_RE.fullmatch(device)
        or not CAPTURE_RE.fullmatch(capture)
        or hashlib.sha256(encoded).hexdigest() != str(row["manifest_sha256"])
      ):
        unavailable += 1
        continue
      path = self._path(VALIDATION_NAMESPACE, device, capture, "manifest.json")
      relative_path = path.relative_to(self.config.storage_root.resolve()).as_posix()
      if str(row["relative_path"]) != relative_path:
        unavailable += 1
        continue
      try:
        if path.is_file():
          if path.read_bytes() != encoded:
            unavailable += 1
          continue
        self._mkdir_parents_durable(path.parent)
        temp = path.with_name(f".manifest.reconcile.{secrets.token_hex(16)}.part")
        try:
          with temp.open("xb") as output:
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
          os.link(temp, path)
          self._fsync_directory(path.parent)
          repaired += 1
        except FileExistsError:
          if path.read_bytes() != encoded:
            unavailable += 1
        finally:
          self._unlink_durable(temp)
      except OSError:
        unavailable += 1
        logging.exception("could not reconcile validation manifest %s", path)
    return repaired, unavailable

  def _audit_validation_files_sync(self) -> int:
    unavailable = 0
    with self._connect() as connection:
      rows = connection.execute(
        "SELECT device_id, capture_id, segment, filename, relative_path FROM validation_files",
      ).fetchall()
    for row in rows:
      values = tuple(str(row[name]) for name in ("device_id", "capture_id", "segment", "filename"))
      device, capture, segment, filename = values
      if (
        not DEVICE_RE.fullmatch(device)
        or not CAPTURE_RE.fullmatch(capture)
        or not SEGMENT_RE.fullmatch(segment)
        or filename not in VALIDATION_RLOG_NAMES
      ):
        unavailable += 1
        continue
      path = self._path(VALIDATION_NAMESPACE, device, capture, segment, filename)
      expected_relative = path.relative_to(self.config.storage_root.resolve()).as_posix()
      if str(row["relative_path"]) != expected_relative or not path.is_file():
        unavailable += 1
    return unavailable

  def _reconcile_missing_incomplete_files_sync(self) -> int:
    removed = 0
    with self._connect() as connection:
      rows = connection.execute(
        """SELECT f.device_id, f.capture_id, f.segment, f.filename, f.relative_path
           FROM validation_files AS f
           LEFT JOIN validation_completions AS c
             ON c.device_id=f.device_id AND c.capture_id=f.capture_id
           WHERE c.capture_id IS NULL""",
      ).fetchall()
      for row in rows:
        device = str(row["device_id"])
        capture = str(row["capture_id"])
        segment = str(row["segment"])
        filename = str(row["filename"])
        if (
          not DEVICE_RE.fullmatch(device)
          or not CAPTURE_RE.fullmatch(capture)
          or not SEGMENT_RE.fullmatch(segment)
          or filename not in VALIDATION_RLOG_NAMES
        ):
          continue
        path = self._path(VALIDATION_NAMESPACE, device, capture, segment, filename)
        expected_relative = path.relative_to(self.config.storage_root.resolve()).as_posix()
        if str(row["relative_path"]) != expected_relative or path.exists():
          continue
        removed += connection.execute(
          """DELETE FROM validation_files
             WHERE device_id=? AND capture_id=? AND segment=? AND filename=?
               AND relative_path=?""",
          (device, capture, segment, filename, expected_relative),
        ).rowcount
    return removed

  def _reconcile_startup(self) -> None:
    now = int(datetime.now(UTC).timestamp())
    try:
      self._reconcile_startup_reservations_sync()
      self._cleanup_stale_parts_sync(now, startup=True)
      self._repair_completion_manifests_sync()
      self._reconcile_missing_incomplete_files_sync()
      self._audit_validation_files_sync()
    except Exception:
      # Startup remains fail-closed for uploads through the normal free-space,
      # quota, and integrity checks even if best-effort recovery is unavailable.
      logging.exception("upload receiver startup reconciliation failed")

  def _prune_session_issue_buckets(self, now: float | None = None) -> int:
    current = time.monotonic() if now is None else now
    cutoff = current - self.config.session_issue_window_seconds
    removed = 0
    for buckets in self._session_issues.values():
      for source_ip, bucket in list(buckets.items()):
        while bucket and bucket[0] < cutoff:
          bucket.popleft()
        if not bucket:
          buckets.pop(source_ip, None)
          removed += 1
    return removed

  def _check_session_rate(self, source_ip: str, *, validation: bool) -> None:
    now = time.monotonic()
    cutoff = now - self.config.session_issue_window_seconds
    namespace = VALIDATION_PURPOSE if validation else LEGACY_QUOTA_NAMESPACE
    buckets = self._session_issues[namespace]
    bucket = buckets.get(source_ip)
    if bucket is None:
      self._prune_session_issue_buckets(now)
      if len(buckets) >= self.config.session_rate_bucket_limit:
        raise web.HTTPTooManyRequests(text="session rate bucket capacity reached")
      bucket = deque()
      buckets[source_ip] = bucket
    else:
      buckets.move_to_end(source_ip)
    while bucket and bucket[0] < cutoff:
      bucket.popleft()
    if len(bucket) >= self.config.session_issue_limit:
      raise web.HTTPTooManyRequests(text="too many session requests")
    bucket.append(now)

  @staticmethod
  def _body_wait_timeout(deadline: float, idle_timeout: float) -> float:
    remaining = deadline - asyncio.get_running_loop().time()
    if remaining <= 0:
      raise web.HTTPRequestTimeout(text="upload body total deadline exceeded")
    return min(max(0.001, idle_timeout), remaining)

  async def _await_body_operation(
    self,
    operation: Any,
    *,
    deadline: float,
    idle_timeout: float,
  ) -> Any:
    timeout = self._body_wait_timeout(deadline, idle_timeout)
    try:
      return await asyncio.wait_for(operation(), timeout=timeout)
    except TimeoutError as exc:
      if asyncio.get_running_loop().time() >= deadline:
        raise web.HTTPRequestTimeout(text="upload body total deadline exceeded") from exc
      raise web.HTTPRequestTimeout(text="upload body idle deadline exceeded") from exc

  async def _body_chunks(
    self,
    content: Any,
    chunk_size: int,
    *,
    deadline: float,
    idle_timeout: float | None = None,
  ):
    iterator = content.iter_chunked(chunk_size).__aiter__()
    idle = self.config.upload_idle_timeout_seconds if idle_timeout is None else idle_timeout
    while True:
      try:
        chunk = await self._await_body_operation(
          lambda: anext(iterator),
          deadline=deadline,
          idle_timeout=idle,
        )
      except StopAsyncIteration:
        return
      yield chunk

  async def enter_request(self, source_ip: str, *, validation: bool) -> None:
    async with self._active_lock:
      if validation:
        if self._validation_request_active_global >= self.config.validation_request_concurrent_global:
          raise web.HTTPServiceUnavailable(text="validation request receiver is busy")
        if (
          self._validation_request_active_by_ip[source_ip]
          >= self.config.validation_request_concurrent_per_ip
        ):
          raise web.HTTPTooManyRequests(text="too many concurrent validation requests")
        self._validation_request_active_global += 1
        self._validation_request_active_by_ip[source_ip] += 1
      else:
        if self._request_active_global >= self.config.request_concurrent_global:
          raise web.HTTPServiceUnavailable(text="legacy request receiver is busy")
        if self._request_active_by_ip[source_ip] >= self.config.request_concurrent_per_ip:
          raise web.HTTPTooManyRequests(text="too many concurrent legacy requests")
        self._request_active_global += 1
        self._request_active_by_ip[source_ip] += 1

  async def leave_request(self, source_ip: str, *, validation: bool) -> None:
    async with self._active_lock:
      if validation:
        self._validation_request_active_global = max(
          0, self._validation_request_active_global - 1,
        )
        self._validation_request_active_by_ip[source_ip] = max(
          0, self._validation_request_active_by_ip[source_ip] - 1,
        )
        if not self._validation_request_active_by_ip[source_ip]:
          self._validation_request_active_by_ip.pop(source_ip, None)
      else:
        self._request_active_global = max(0, self._request_active_global - 1)
        self._request_active_by_ip[source_ip] = max(
          0, self._request_active_by_ip[source_ip] - 1,
        )
        if not self._request_active_by_ip[source_ip]:
          self._request_active_by_ip.pop(source_ip, None)

  async def _json_body(
    self,
    request: web.Request,
    limit: int,
    *,
    on_progress: Any = None,
    include_size: bool = False,
  ) -> Any:
    content_length = request.content_length
    if content_length is not None and content_length > limit:
      raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=content_length)
    deadline = asyncio.get_running_loop().time() + self.config.json_body_total_timeout_seconds
    data = bytearray()
    while True:
      read_size = min(64 * 1024, limit + 1 - len(data))
      chunk = await self._await_body_operation(
        lambda read_size=read_size: request.content.read(read_size),
        deadline=deadline,
        idle_timeout=self.config.json_body_idle_timeout_seconds,
      )
      if not chunk:
        break
      data.extend(chunk)
      if on_progress is not None:
        await on_progress(len(data))
      if len(data) > limit:
        raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=len(data))
    try:
      body = json.loads(data.decode("utf-8"))
    except Exception as exc:
      raise web.HTTPBadRequest(text="invalid JSON") from exc
    return (body, len(data)) if include_size else body

  async def create_session(self, request: web.Request) -> web.Response:
    self._require_legacy_uploads_enabled()
    body = await self._json_body(request, 16 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    device = self._require_allowed_device(
      self._validate_device(body.get("deviceId") or body.get("dongleId")),
    )
    purpose = str(body.get("purpose") or "upload").strip().lower()
    if purpose not in {"dashcam", "tmux", "test"}:
      raise web.HTTPBadRequest(text="invalid purpose")
    car_name = self._storage_component(body.get("carName") or body.get("car_name"), "none")
    git_branch = self._storage_component(
      body.get("branch") or body.get("gitBranch") or body.get("git_branch"), "unknown", branch=True,
    )
    tmux_reason = self._storage_component(body.get("tmux_why") or body.get("reason"), "tmux")
    source_ip = self.source_ip(request)
    self._check_session_rate(source_ip, validation=False)
    token = secrets.token_urlsafe(32)
    now = int(datetime.now(UTC).timestamp())
    expires = now + self.config.session_ttl_seconds
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("DELETE FROM sessions WHERE expires_at < ?", (now,))
        connection.execute(
          """INSERT INTO sessions(
               token_hash, device_id, source_ip, purpose, car_name, git_branch, tmux_reason, created_at, expires_at
             ) VALUES(?,?,?,?,?,?,?,?,?)""",
          (self._token_hash(token), device, source_ip, purpose, car_name, git_branch, tmux_reason, now, expires),
        )
    return web.json_response({
      "ok": True,
      "token": token,
      "expiresAt": expires,
      "expiresIn": self.config.session_ttl_seconds,
      "dailyQuotaBytes": self.config.daily_device_quota,
      "maxFileBytes": self.config.max_file_bytes,
    })

  async def create_validation_challenge(self, request: web.Request) -> web.Response:
    body = await self._json_body(request, 8 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    device = self._require_allowed_device(self._validate_device(body.get("deviceId")))
    self._require_validation_device_key_pin(device)
    source_ip = self.source_ip(request)
    self._check_session_rate(source_ip, validation=True)
    audience = str(self.config.validation_audience).strip()
    try:
      audience_bytes = audience.encode("utf-8")
    except UnicodeEncodeError as exc:
      raise web.HTTPServiceUnavailable(text="validation audience is not configured") from exc
    if not audience or len(audience_bytes) > 256 or "\0" in audience:
      raise web.HTTPServiceUnavailable(text="validation audience is not configured")
    now = int(datetime.now(UTC).timestamp())
    expires_at = now + self.config.validation_challenge_ttl_seconds
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        connection.execute("DELETE FROM validation_challenges WHERE expires_at < ?", (now - 3600,))
        existing = connection.execute(
          """SELECT * FROM validation_challenges
             WHERE device_id=? AND source_ip=? AND audience=?
               AND consumed_at IS NULL AND expires_at>=?
             ORDER BY created_at DESC LIMIT 1""",
          (device, source_ip, audience, now),
        ).fetchone()
        if existing is not None:
          challenge_id = str(existing["challenge_id"])
          nonce = str(existing["nonce"])
          expires_at = int(existing["expires_at"])
        else:
          challenge_id = secrets.token_urlsafe(24)
          nonce = secrets.token_urlsafe(24)
          connection.execute(
            """INSERT INTO validation_challenges(
                 challenge_id, nonce, device_id, audience, source_ip, created_at, expires_at
               ) VALUES(?,?,?,?,?,?,?)""",
            (challenge_id, nonce, device, audience, source_ip, now, expires_at),
          )
    return web.json_response({
      "ok": True,
      "challengeId": challenge_id,
      "nonce": nonce,
      "audience": audience,
      "expiresAt": expires_at,
      "deviceAuthVersion": DEVICE_AUTH_VERSION,
      "receiptVersion": RECEIPT_VERSION,
    })

  async def _claim_validation_verification(
    self, challenge_id: str, device: str, source_ip: str,
  ) -> tuple[sqlite3.Row, int]:
    now = int(datetime.now(UTC).timestamp())
    verifier_lease = max(2, math.ceil(self.config.validation_verify_timeout_seconds) + 2)
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        connection.execute(
          """UPDATE validation_challenges SET verification_started_at=NULL
             WHERE verification_started_at IS NOT NULL AND verification_started_at < ?""",
          (now - verifier_lease,),
        )
        challenge = connection.execute(
          "SELECT * FROM validation_challenges WHERE challenge_id=?", (challenge_id,),
        ).fetchone()
        if challenge is None or challenge["device_id"] != device:
          raise web.HTTPUnauthorized(text="unknown validation challenge")
        if challenge["source_ip"] != source_ip:
          raise web.HTTPForbidden(text="validation challenge IP mismatch")
        if challenge["consumed_at"] is not None:
          raise web.HTTPConflict(text="validation challenge was already consumed")
        if int(challenge["expires_at"]) < now:
          raise web.HTTPUnauthorized(text="validation challenge expired")
        if int(challenge["verification_failures"]) >= self.config.validation_verify_attempt_limit:
          retry_seconds = max(1, int(challenge["expires_at"]) - now)
          raise web.HTTPTooManyRequests(
            text="validation challenge authentication attempt limit reached",
            headers={"Retry-After": str(retry_seconds)},
          )
        if int(challenge["retry_after"]) > now:
          raise web.HTTPTooManyRequests(
            text="validation challenge authentication is cooling down",
            headers={"Retry-After": str(max(1, int(challenge["retry_after"]) - now))},
          )
        if challenge["verification_started_at"] is not None:
          raise web.HTTPConflict(
            text="validation challenge verification is already in progress",
            headers={"Retry-After": "1"},
          )
        other_in_flight = connection.execute(
          """SELECT 1 FROM validation_challenges
             WHERE device_id=? AND source_ip=? AND challenge_id<>?
               AND verification_started_at IS NOT NULL
             LIMIT 1""",
          (device, source_ip, challenge_id),
        ).fetchone()
        if other_in_flight is not None:
          raise web.HTTPConflict(
            text="device identity verification is already in progress",
            headers={"Retry-After": "1"},
          )
        updated = connection.execute(
          """UPDATE validation_challenges SET verification_started_at=?
             WHERE challenge_id=? AND verification_started_at IS NULL AND consumed_at IS NULL""",
          (now, challenge_id),
        ).rowcount
        if updated != 1:
          raise web.HTTPConflict(text="validation challenge verification is already in progress")
        claimed = connection.execute(
          "SELECT * FROM validation_challenges WHERE challenge_id=?", (challenge_id,),
        ).fetchone()
    return claimed, now

  async def _release_validation_verification(
    self, challenge_id: str, marker: int, *, rejected: bool,
  ) -> None:
    now = int(datetime.now(UTC).timestamp())
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute(
          """UPDATE validation_challenges
             SET verification_started_at=NULL,
                 verification_failures=verification_failures+?,
                 retry_after=?
             WHERE challenge_id=? AND verification_started_at=? AND consumed_at IS NULL""",
          (
            1 if rejected else 0,
            now + self.config.validation_verify_cooldown_seconds,
            challenge_id,
            marker,
          ),
        )

  async def create_validation_session(self, request: web.Request) -> web.Response:
    body = await self._json_body(request, 32 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    device = self._require_allowed_device(self._validate_device(body.get("deviceId")))
    expected_fingerprint = self._require_validation_device_key_pin(device)
    challenge_id = self._validate_challenge(body.get("challengeId"))
    if "identityToken" in body:
      raise web.HTTPBadRequest(text="generic device identity tokens are not accepted")
    algorithm = body.get("deviceKeyAlgorithm")
    public_key_pem = body.get("devicePublicKey")
    proof = body.get("deviceProof")
    if (
      not isinstance(algorithm, str)
      or algorithm not in {"RS256", "ES256"}
      or not isinstance(public_key_pem, str)
      or not public_key_pem
      or not isinstance(proof, str)
      or not proof
    ):
      raise web.HTTPUnauthorized(text="device key algorithm, public key, and proof are required")
    source_ip = self.source_ip(request)
    async with self._db_lock:
      with self._connect() as connection:
        challenge = connection.execute(
          "SELECT * FROM validation_challenges WHERE challenge_id = ?", (challenge_id,),
        ).fetchone()
    if challenge is None or challenge["device_id"] != device:
      raise web.HTTPUnauthorized(text="unknown validation challenge")
    if challenge["source_ip"] != source_ip:
      raise web.HTTPForbidden(text="validation challenge IP mismatch")
    if challenge["consumed_at"] is not None:
      raise web.HTTPConflict(text="validation challenge was already consumed")

    challenge, verification_marker = await self._claim_validation_verification(
      challenge_id, device, source_ip,
    )

    acquired_verifier = False
    try:
      try:
        await asyncio.wait_for(self._validation_verifier_semaphore.acquire(), timeout=0.1)
        acquired_verifier = True
      except TimeoutError as exc:
        await self._release_validation_verification(challenge_id, verification_marker, rejected=False)
        raise web.HTTPServiceUnavailable(
          text="device proof verifier is busy",
          headers={"Retry-After": str(self.config.validation_verify_cooldown_seconds)},
        ) from exc
      verified_device = await asyncio.wait_for(
        self._validation_verifier.verify(
          algorithm=algorithm,
          public_key_pem=public_key_pem,
          proof=proof,
          message=self._validation_device_proof_message(
            device,
            challenge_id,
            str(challenge["nonce"]),
            str(challenge["audience"]),
          ),
          expected_fingerprint=expected_fingerprint,
          expected_device_id=device,
        ),
        timeout=self.config.validation_verify_timeout_seconds + 1.0,
      )
    except DeviceProofVerificationUnavailable as exc:
      await self._release_validation_verification(challenge_id, verification_marker, rejected=False)
      raise web.HTTPServiceUnavailable(text="device proof verification is temporarily unavailable") from exc
    except DeviceProofVerificationRejected as exc:
      await self._release_validation_verification(challenge_id, verification_marker, rejected=True)
      raise web.HTTPUnauthorized(text="device proof verification rejected the signature") from exc
    except TimeoutError as exc:
      await self._release_validation_verification(challenge_id, verification_marker, rejected=False)
      raise web.HTTPServiceUnavailable(text="device proof verification timed out") from exc
    except asyncio.CancelledError:
      await asyncio.shield(
        self._release_validation_verification(challenge_id, verification_marker, rejected=False),
      )
      raise
    except web.HTTPException:
      raise
    except Exception as exc:
      # An injected verifier is still an authentication boundary. Unexpected
      # behavior must never degrade into accepting an unverified token.
      await self._release_validation_verification(challenge_id, verification_marker, rejected=False)
      raise web.HTTPServiceUnavailable(text="device proof verification failed closed") from exc
    finally:
      if acquired_verifier:
        self._validation_verifier_semaphore.release()
    if verified_device != device:
      await self._release_validation_verification(challenge_id, verification_marker, rejected=True)
      raise web.HTTPUnauthorized(text="verified device identity mismatch")

    token = secrets.token_urlsafe(32)
    token_hash = self._token_hash(token)
    # Exchange the one-time, receiver-bound proof for a longer, narrowly
    # purpose-scoped session so several large rlogs can finish after the
    # challenge expires without sending the device's general comma JWT.
    now = int(datetime.now(UTC).timestamp())
    expires_at = now + self.config.validation_session_ttl_seconds
    car_name = self._storage_component(body.get("carName"), "none")
    git_branch = self._storage_component(body.get("branch"), "unknown", branch=True)

    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        current = connection.execute(
          "SELECT * FROM validation_challenges WHERE challenge_id = ?", (challenge_id,),
        ).fetchone()
        if current is None or current["device_id"] != device:
          raise web.HTTPUnauthorized(text="unknown validation challenge")
        if current["source_ip"] != source_ip:
          raise web.HTTPForbidden(text="validation challenge IP mismatch")
        if current["consumed_at"] is not None:
          raise web.HTTPConflict(text="validation challenge was already consumed")
        if int(current["verification_started_at"] or -1) != verification_marker:
          raise web.HTTPConflict(text="validation challenge verification lease was lost")
        if int(current["expires_at"]) < int(datetime.now(UTC).timestamp()):
          raise web.HTTPUnauthorized(text="validation challenge expired")
        updated = connection.execute(
          """UPDATE validation_challenges
             SET consumed_at = ?, session_token_hash = ?, verification_started_at=NULL
             WHERE challenge_id = ? AND consumed_at IS NULL AND verification_started_at=?""",
          (now, token_hash, challenge_id, verification_marker),
        ).rowcount
        if updated != 1:
          raise web.HTTPConflict(text="validation challenge was already consumed")
        connection.execute(
          """INSERT INTO sessions(
               token_hash, device_id, source_ip, purpose, car_name, git_branch, tmux_reason,
               device_key_sha256, created_at, expires_at
             ) VALUES(?,?,?,?,?,?,?,?,?,?)""",
          (
            token_hash, device, source_ip, VALIDATION_PURPOSE, car_name, git_branch,
            "validation", expected_fingerprint, now, expires_at,
          ),
        )

    return web.json_response({
      "ok": True,
      "token": token,
      "verifiedDeviceId": verified_device,
      "expiresAt": expires_at,
      "expiresIn": max(0, expires_at - now),
      "dailyQuotaBytes": self.config.daily_device_quota,
      "maxFileBytes": self.config.max_file_bytes,
      "deviceAuthVersion": DEVICE_AUTH_VERSION,
      "receiptVersion": RECEIPT_VERSION,
    })

  async def authenticate(
    self,
    request: web.Request,
    *,
    device: str | None = None,
    purposes: set[str] | None = None,
  ) -> sqlite3.Row:
    authorization = request.headers.get("Authorization", "")
    if not authorization.startswith("Bearer "):
      raise web.HTTPUnauthorized(text="missing upload session")
    token = authorization[7:].strip()
    if not token:
      raise web.HTTPUnauthorized(text="missing upload session")
    async with self._db_lock:
      with self._connect() as connection:
        row = connection.execute(
          "SELECT * FROM sessions WHERE token_hash = ?", (self._token_hash(token),),
        ).fetchone()
    if row is None or int(row["expires_at"]) < int(datetime.now(UTC).timestamp()):
      raise web.HTTPUnauthorized(text="expired upload session")
    self._require_allowed_device(str(row["device_id"]))
    if row["purpose"] == VALIDATION_PURPOSE:
      current_pin = self._require_validation_device_key_pin(str(row["device_id"]))
      if not hmac.compare_digest(str(row["device_key_sha256"]), current_pin):
        raise web.HTTPForbidden(text="upload session device key pin changed")
    if row["source_ip"] != self.source_ip(request):
      raise web.HTTPForbidden(text="upload session IP mismatch")
    if device is not None and row["device_id"] != device:
      raise web.HTTPForbidden(text="upload session device mismatch")
    if purposes is not None and row["purpose"] not in purposes:
      raise web.HTTPForbidden(text="upload session purpose mismatch")
    return row

  async def _reserve(
    self,
    device: str,
    source_ip: str,
    amount: int,
    *,
    storage_amount: int | None = None,
    validation: bool = False,
  ) -> UploadReservation:
    day = self._usage_day()
    amount = max(0, int(amount))
    reserved_storage = amount if storage_amount is None else max(0, int(storage_amount))
    quota_namespace = VALIDATION_PURPOSE if validation else LEGACY_QUOTA_NAMESPACE
    quota_scopes = self._quota_scope_pairs(quota_namespace, device, source_ip)
    reservation_id = secrets.token_urlsafe(24)
    now = int(datetime.now(UTC).timestamp())
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        self._reconcile_stale_reservations_connection(
          connection, stale_before=now - self.config.reservation_lease_seconds,
        )
        for (scope, scope_id), quota in zip(
          quota_scopes,
          (self.config.daily_device_quota, self.config.daily_ip_quota),
          strict=True,
        ):
          connection.execute(
            "INSERT OR IGNORE INTO daily_usage(usage_day, scope, scope_id) VALUES(?,?,?)",
            (day, scope, scope_id),
          )
          row = connection.execute(
            "SELECT committed_bytes, reserved_bytes FROM daily_usage WHERE usage_day=? AND scope=? AND scope_id=?",
            (day, scope, scope_id),
          ).fetchone()
          if int(row["committed_bytes"]) + int(row["reserved_bytes"]) + amount > quota:
            raise web.HTTPRequestEntityTooLarge(max_size=quota, actual_size=int(row["committed_bytes"]) + amount)
        in_flight = int(connection.execute(
          """SELECT COALESCE(SUM(MAX(0, reserved_storage_bytes-disk_written_bytes)), 0)
             FROM upload_reservations""",
        ).fetchone()[0])
        if not self._has_disk_space(
          reserved_storage,
          in_flight,
          validation=validation,
        ):
          raise web.HTTPInsufficientStorage(text="not enough unreserved free storage")
        for scope, scope_id in quota_scopes:
          connection.execute(
            "UPDATE daily_usage SET reserved_bytes=reserved_bytes+? WHERE usage_day=? AND scope=? AND scope_id=?",
            (amount, day, scope, scope_id),
          )
        connection.execute(
          """INSERT INTO upload_reservations(
               reservation_id, usage_day, device_id, source_ip, reserved_bytes, reserved_storage_bytes,
               quota_namespace, received_bytes, disk_written_bytes, stored_bytes, created_at, touched_at
             ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)""",
          (
            reservation_id, day, device, source_ip, amount, reserved_storage,
            quota_namespace, 0, 0, 0, now, now,
          ),
        )
    return UploadReservation(
      reservation_id,
      day,
      device,
      source_ip,
      amount,
      reserved_storage,
      quota_namespace,
    )

  async def _grow_storage_reservation(
    self,
    reservation: UploadReservation,
    storage_amount: int,
  ) -> None:
    requested = max(0, int(storage_amount))
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
          "SELECT * FROM upload_reservations WHERE reservation_id=?", (reservation.reservation_id,),
        ).fetchone()
        if row is None:
          raise web.HTTPServiceUnavailable(text="upload reservation expired")
        current = max(0, int(row["reserved_storage_bytes"]))
        additional = max(0, requested - current)
        in_flight = int(connection.execute(
          """SELECT COALESCE(SUM(MAX(0, reserved_storage_bytes-disk_written_bytes)), 0)
             FROM upload_reservations""",
        ).fetchone()[0])
        if additional and not self._has_disk_space(
          additional,
          in_flight,
          validation=reservation.quota_namespace == VALIDATION_PURPOSE,
        ):
          raise web.HTTPInsufficientStorage(text="not enough unreserved free storage")
        connection.execute(
          """UPDATE upload_reservations
             SET reserved_storage_bytes=MAX(reserved_storage_bytes, ?), touched_at=?
             WHERE reservation_id=?""",
          (requested, int(datetime.now(UTC).timestamp()), reservation.reservation_id),
        )

  async def _record_reservation_progress(
    self,
    reservation: UploadReservation,
    received_bytes: int,
    stored_bytes: int = 0,
    *,
    disk_written_bytes: int,
  ) -> None:
    received = max(0, int(received_bytes))
    disk_written = max(0, int(disk_written_bytes))
    async with self._db_lock:
      with self._connect() as connection:
        updated = connection.execute(
          """UPDATE upload_reservations
             SET received_bytes=MAX(received_bytes, ?),
                 disk_written_bytes=MAX(disk_written_bytes, ?),
                 stored_bytes=?,
                 touched_at=?
             WHERE reservation_id=?""",
          (
            received, disk_written, max(0, int(stored_bytes)),
            int(datetime.now(UTC).timestamp()), reservation.reservation_id,
          ),
        ).rowcount
    if updated != 1:
      raise web.HTTPServiceUnavailable(text="upload reservation expired")

  async def _record_stream_progress(
    self,
    reservation: UploadReservation,
    progress: StreamProgress,
    *,
    received_bytes: int,
    disk_written_bytes: int,
    stored_bytes: int = 0,
    force: bool = False,
  ) -> None:
    # Update the event-loop-local view immediately. Durable progress is
    # coalesced to bound SQLite FULL-sync churn from arbitrarily fragmented
    # request bodies. Lagging disk progress keeps more free space reserved,
    # while lagging network progress is bounded by the byte/time thresholds.
    progress.received_bytes = max(progress.received_bytes, max(0, int(received_bytes)))
    progress.disk_written_bytes = max(
      progress.disk_written_bytes,
      max(0, int(disk_written_bytes)),
    )
    progress.stored_bytes = max(0, int(stored_bytes))
    now = asyncio.get_running_loop().time()
    changed = (
      progress.received_bytes != progress.persisted_received_bytes
      or progress.disk_written_bytes != progress.persisted_disk_written_bytes
      or progress.stored_bytes != progress.persisted_stored_bytes
    )
    should_persist = changed and (
      not progress.persisted_once
      or force
      or progress.stored_bytes != progress.persisted_stored_bytes
      or progress.received_bytes - progress.persisted_received_bytes
      >= self.config.progress_commit_bytes
      or now - progress.last_persisted_at >= self.config.progress_commit_interval_seconds
    )
    if not should_persist:
      return
    await self._record_reservation_progress(
      reservation,
      progress.received_bytes,
      progress.stored_bytes,
      disk_written_bytes=progress.disk_written_bytes,
    )
    progress.persisted_received_bytes = progress.received_bytes
    progress.persisted_disk_written_bytes = progress.disk_written_bytes
    progress.persisted_stored_bytes = progress.stored_bytes
    progress.last_persisted_at = now
    progress.persisted_once = True

  async def _write_reserved_chunk(
    self,
    output: Any,
    chunk: bytes,
    reservation: UploadReservation,
    progress: StreamProgress,
    *,
    network_received: int,
    disk_written: int,
    stored_bytes: int = 0,
  ) -> int:
    """Write a received chunk without releasing free-space reserve early.

    The caller durably records the network byte count before entering here.
    Each successful file write is then recorded separately, so an await may
    interleave another reservation only while the unwritten bytes remain
    covered by this reservation. A short write is accounted before failing.
    """
    offset = 0
    while offset < len(chunk):
      count = output.write(memoryview(chunk)[offset:])
      if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
        raise OSError("upload file write made no progress")
      count = min(count, len(chunk) - offset)
      offset += count
      disk_written += count
      await self._record_stream_progress(
        reservation,
        progress,
        received_bytes=network_received,
        disk_written_bytes=disk_written,
        stored_bytes=stored_bytes,
      )
    return disk_written

  async def _finish_reservation(
    self,
    reservation: UploadReservation,
    network_bytes: int,
    stored_bytes: int = 0,
  ) -> None:
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        row = connection.execute(
          "SELECT * FROM upload_reservations WHERE reservation_id=?", (reservation.reservation_id,),
        ).fetchone()
        if row is None:
          return
        charged_network = max(max(0, int(network_bytes)), int(row["received_bytes"]))
        charged_storage = max(max(0, int(stored_bytes)), int(row["stored_bytes"]))
        namespace = (
          VALIDATION_PURPOSE
          if str(row["quota_namespace"]) == VALIDATION_PURPOSE
          else LEGACY_QUOTA_NAMESPACE
        )
        for scope, scope_id in self._quota_scope_pairs(
          namespace,
          str(row["device_id"]),
          str(row["source_ip"]),
        ):
          connection.execute(
            """UPDATE daily_usage
               SET reserved_bytes=MAX(0, reserved_bytes-?),
                   committed_bytes=MAX(0, committed_bytes+?),
                   stored_bytes=MAX(0, stored_bytes+?)
               WHERE usage_day=? AND scope=? AND scope_id=?""",
            (
              int(row["reserved_bytes"]), charged_network, charged_storage,
              row["usage_day"], scope, scope_id,
            ),
          )
        connection.execute(
          "DELETE FROM upload_reservations WHERE reservation_id=?", (reservation.reservation_id,),
        )

  async def _enter_upload(self, device: str, *, validation: bool = False) -> None:
    async with self._active_lock:
      if validation:
        if self._validation_active_global >= self.config.validation_concurrent_global:
          raise web.HTTPServiceUnavailable(text="validation upload server is busy")
        if self._validation_active_by_device[device] >= self.config.validation_concurrent_per_device:
          raise web.HTTPTooManyRequests(text="too many concurrent validation uploads for device")
        self._validation_active_global += 1
        self._validation_active_by_device[device] += 1
      else:
        if self._active_global >= self.config.concurrent_global:
          raise web.HTTPServiceUnavailable(text="upload server is busy")
        if self._active_by_device[device] >= self.config.concurrent_per_device:
          raise web.HTTPTooManyRequests(text="too many concurrent uploads for device")
        self._active_global += 1
        self._active_by_device[device] += 1

  async def _leave_upload(self, device: str, *, validation: bool = False) -> None:
    async with self._active_lock:
      if validation:
        self._validation_active_global = max(0, self._validation_active_global - 1)
        self._validation_active_by_device[device] = max(
          0, self._validation_active_by_device[device] - 1,
        )
        if not self._validation_active_by_device[device]:
          self._validation_active_by_device.pop(device, None)
      else:
        self._active_global = max(0, self._active_global - 1)
        self._active_by_device[device] = max(0, self._active_by_device[device] - 1)
        if not self._active_by_device[device]:
          self._active_by_device.pop(device, None)

  async def health(self, _request: web.Request) -> web.Response:
    allowlist_configured = bool(self.config.allowed_device_ids)
    pins_configured = (
      allowlist_configured
      and set(self._device_key_pins) == set(self.config.allowed_device_ids)
      and all(
        isinstance(value, str) and DEVICE_KEY_SHA256_RE.fullmatch(value)
        for value in self._device_key_pins.values()
      )
    )
    storage_writable = self._validation_storage_writable()
    try:
      disk_ready = self._has_disk_space()
    except OSError:
      disk_ready = False
    ready = allowlist_configured and pins_configured and storage_writable and disk_ready
    status = 200 if ready else 503 if not allowlist_configured or not pins_configured else 507
    return web.json_response({
      "ok": ready,
      "service": "dk-upload",
      "deviceAllowlistConfigured": allowlist_configured,
      "deviceKeyPinsConfigured": pins_configured,
      "legacyUploadsEnabled": self.config.legacy_uploads_enabled,
      "storageWritable": storage_writable,
      "dailyQuotaBytes": self.config.daily_device_quota,
      "maxFileBytes": self.config.max_file_bytes,
      "bandwidthLimit": None,
    }, status=status)

  async def upload_file(self, request: web.Request) -> web.Response:
    self._require_legacy_uploads_enabled()
    device = self._validate_device(request.match_info.get("device"))
    segment = self._validate_segment(request.match_info.get("segment"))
    filename = self._validate_filename(request.match_info.get("filename"))
    session = await self.authenticate(request, device=device, purposes={"dashcam"})
    try:
      expected = int(request.headers.get("X-File-Size", ""))
    except ValueError as exc:
      raise web.HTTPLengthRequired(text="X-File-Size is required") from exc
    if expected <= 0:
      raise web.HTTPBadRequest(text="upload file must not be empty")
    if expected > self.config.max_file_bytes:
      raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_file_bytes, actual_size=expected)
    target = self._path("routes", self._storage_directory(session), segment, filename)
    # The daily quota measures network usage, not the final storage delta.
    # Re-uploading the same path must therefore consume the full request size
    # again instead of allowing repeated overwrites to bypass the limit.
    reserved = expected
    await self._enter_upload(device)
    reservation: UploadReservation | None = None
    temp = target.with_name(f".{filename}.{secrets.token_hex(8)}.part")
    received = 0
    disk_written = 0
    stored = 0
    progress = StreamProgress()
    try:
      reservation = await self._reserve(device, session["source_ip"], reserved)
      self._mkdir_parents_durable(target.parent)
      deadline = asyncio.get_running_loop().time() + self.config.upload_total_timeout_seconds
      with temp.open("xb", buffering=0) as output:
        async for chunk in self._body_chunks(request.content, MIB, deadline=deadline):
          received += len(chunk)
          await self._record_stream_progress(
            reservation,
            progress,
            received_bytes=received,
            disk_written_bytes=disk_written,
          )
          if received > expected or received > self.config.max_file_bytes:
            raise web.HTTPRequestEntityTooLarge(max_size=expected, actual_size=received)
          disk_written = await self._write_reserved_chunk(
            output,
            chunk,
            reservation,
            progress,
            network_received=received,
            disk_written=disk_written,
          )
        output.flush()
        os.fsync(output.fileno())
      if received != expected:
        raise web.HTTPBadRequest(text=f"file size mismatch: expected {expected}, received {received}")
      os.replace(temp, target)
      self._fsync_directory(target.parent)
      stored = disk_written
      await self._record_stream_progress(
        reservation,
        progress,
        received_bytes=received,
        stored_bytes=stored,
        disk_written_bytes=disk_written,
        force=True,
      )
      return web.json_response({"ok": True, "size": received})
    finally:
      self._unlink_durable(temp)
      if reservation is not None:
        try:
          await self._record_stream_progress(
            reservation,
            progress,
            received_bytes=received,
            stored_bytes=stored,
            disk_written_bytes=disk_written,
            force=True,
          )
        finally:
          await self._finish_reservation(reservation, received, stored)
      await self._leave_upload(device)

  async def validation_upload_file(self, request: web.Request) -> web.Response:
    capture = self._validate_capture(request.match_info.get("captureId"))
    segment = self._validate_segment(request.match_info.get("segment"))
    self._validation_segment_route(segment)
    filename = self._validate_validation_filename(request.match_info.get("filename"))
    session = await self.authenticate(request, purposes={VALIDATION_PURPOSE})
    device = str(session["device_id"])
    await self._enter_upload(device, validation=True)
    try:
      return await self._validation_upload_file_active(
        request, capture=capture, segment=segment, filename=filename, session=session, device=device,
      )
    finally:
      await self._leave_upload(device, validation=True)

  async def _validation_upload_file_active(
    self,
    request: web.Request,
    *,
    capture: str,
    segment: str,
    filename: str,
    session: dict[str, Any],
    device: str,
  ) -> web.Response:
    try:
      expected = int(request.headers.get("X-File-Size", ""))
    except ValueError as exc:
      raise web.HTTPLengthRequired(text="X-File-Size is required") from exc
    if expected <= 0:
      raise web.HTTPBadRequest(text="validation rlog must not be empty")
    if expected > self.config.max_file_bytes:
      raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_file_bytes, actual_size=expected)
    expected_sha256 = self._validate_sha256(request.headers.get("X-Content-SHA256"))
    if request.content_length is not None and request.content_length != expected:
      raise web.HTTPBadRequest(text="Content-Length does not match X-File-Size")
    target = self._path(VALIDATION_NAMESPACE, device, capture, segment, filename)
    relative_path = target.relative_to(self.config.storage_root.resolve()).as_posix()

    async with self._db_lock:
      with self._connect() as connection:
        existing = connection.execute(
          """SELECT * FROM validation_files
             WHERE device_id=? AND capture_id=? AND segment=? AND filename=?""",
          (device, capture, segment, filename),
        ).fetchone()
        completed = connection.execute(
          "SELECT 1 FROM validation_completions WHERE device_id=? AND capture_id=?",
          (device, capture),
        ).fetchone()
        capture_rows = connection.execute(
          """SELECT segment, filename FROM validation_files
             WHERE device_id=? AND capture_id=?""",
          (device, capture),
        ).fetchall()
    if existing is not None:
      identical = int(existing["size"]) == expected and str(existing["sha256"]) == expected_sha256
      if not identical:
        raise web.HTTPConflict(text="validation file path is immutable")
      if str(existing["relative_path"]) != relative_path or not target.is_file():
        raise web.HTTPInternalServerError(text="recorded validation file is unavailable")
      actual_size, actual_sha256 = await self._hash_file_bounded(target, device=device)
      if actual_size != expected or actual_sha256 != expected_sha256:
        raise web.HTTPInternalServerError(text="recorded validation file failed integrity verification")
    if completed is not None:
      if existing is None:
        raise web.HTTPConflict(text="validation capture is already complete")
    if existing is None and (
      len(capture_rows) >= VALIDATION_MAX_FILES
      or any(str(row["segment"]) == segment for row in capture_rows)
    ):
      raise web.HTTPConflict(text="validation capture already has its bounded rlog set")

    reservation: UploadReservation | None = None
    temp = target.with_name(f".{filename}.{secrets.token_hex(16)}.part")
    linked_new_file = False
    received = 0
    disk_written = 0
    stored = 0
    progress = StreamProgress()
    try:
      reservation = await self._reserve(
        device,
        session["source_ip"],
        expected,
        validation=True,
      )
      self._mkdir_parents_durable(target.parent)
      digest = hashlib.sha256()
      deadline = asyncio.get_running_loop().time() + self.config.upload_total_timeout_seconds
      with temp.open("xb", buffering=0) as output:
        async for chunk in self._body_chunks(request.content, MIB, deadline=deadline):
          received += len(chunk)
          await self._record_stream_progress(
            reservation,
            progress,
            received_bytes=received,
            disk_written_bytes=disk_written,
          )
          if received > expected or received > self.config.max_file_bytes:
            raise web.HTTPRequestEntityTooLarge(max_size=expected, actual_size=received)
          disk_written = await self._write_reserved_chunk(
            output,
            chunk,
            reservation,
            progress,
            network_received=received,
            disk_written=disk_written,
          )
          digest.update(chunk)
        output.flush()
        os.fsync(output.fileno())
      if received != expected:
        raise web.HTTPBadRequest(text=f"file size mismatch: expected {expected}, received {received}")
      actual_sha256 = digest.hexdigest()
      if actual_sha256 != expected_sha256:
        raise web.HTTPBadRequest(text="file SHA-256 mismatch")

      try:
        # A hard-link publication is atomic and, unlike os.replace(), can
        # never overwrite an already-published validation artifact.
        os.link(temp, target)
        linked_new_file = True
        self._fsync_directory(target.parent)
      except FileExistsError:
        actual_size, published_sha256 = await self._hash_file_bounded(target, device=device)
        if actual_size != expected or published_sha256 != expected_sha256:
          raise web.HTTPConflict(text="validation file path is immutable") from None

      inserted_receipt = False
      async with self._db_lock:
        with self._connect() as connection:
          connection.execute("BEGIN IMMEDIATE")
          completed = connection.execute(
            "SELECT 1 FROM validation_completions WHERE device_id=? AND capture_id=?",
            (device, capture),
          ).fetchone()
          existing = connection.execute(
            """SELECT * FROM validation_files
               WHERE device_id=? AND capture_id=? AND segment=? AND filename=?""",
            (device, capture, segment, filename),
          ).fetchone()
          if completed is not None and existing is None:
            if linked_new_file:
              self._unlink_durable(target)
              linked_new_file = False
              stored = 0
            raise web.HTTPConflict(text="validation capture is already complete")
          if existing is not None:
            if int(existing["size"]) != expected or str(existing["sha256"]) != expected_sha256:
              if linked_new_file:
                self._unlink_durable(target)
                linked_new_file = False
                stored = 0
              raise web.HTTPConflict(text="validation file path is immutable")
          else:
            capture_rows = connection.execute(
              """SELECT segment FROM validation_files
                 WHERE device_id=? AND capture_id=?""",
              (device, capture),
            ).fetchall()
            if (
              len(capture_rows) >= VALIDATION_MAX_FILES
              or any(str(row["segment"]) == segment for row in capture_rows)
            ):
              if linked_new_file:
                self._unlink_durable(target)
                linked_new_file = False
                stored = 0
              raise web.HTTPConflict(text="validation capture already has its bounded rlog set")
            connection.execute(
              """INSERT INTO validation_files(
                   device_id, capture_id, segment, filename, size, sha256, relative_path, received_at
                 ) VALUES(?,?,?,?,?,?,?,?)""",
              (
                device, capture, segment, filename, expected, expected_sha256, relative_path,
                int(datetime.now(UTC).timestamp()),
              ),
            )
            reservation_updated = connection.execute(
              """UPDATE upload_reservations
                 SET received_bytes=MAX(received_bytes, ?),
                     disk_written_bytes=MAX(disk_written_bytes, ?),
                     stored_bytes=?,
                     touched_at=?
                 WHERE reservation_id=?""",
              (
                received,
                disk_written,
                expected,
                int(datetime.now(UTC).timestamp()),
                reservation.reservation_id,
              ),
            ).rowcount
            if reservation_updated != 1:
              raise web.HTTPServiceUnavailable(text="upload reservation expired")
            inserted_receipt = True

      # Whichever request inserts the immutable receipt owns the one storage
      # charge, even when it recovered a link left by a crashed request. The
      # receipt and reservation accounting commit atomically above.
      stored = expected if inserted_receipt else 0
      await self._record_stream_progress(
        reservation,
        progress,
        received_bytes=received,
        stored_bytes=stored,
        disk_written_bytes=disk_written,
        force=True,
      )
      return web.json_response({
        "ok": True,
        "size": received,
        "sha256": actual_sha256,
        "alreadyReceived": not linked_new_file,
      })
    finally:
      self._unlink_durable(temp)
      if reservation is not None:
        try:
          await self._record_stream_progress(
            reservation,
            progress,
            received_bytes=received,
            stored_bytes=stored,
            disk_written_bytes=disk_written,
            force=True,
          )
        finally:
          await self._finish_reservation(reservation, received, stored)

  @staticmethod
  def _protocol_text(value: Any, name: str, *, maximum: int, allow_empty: bool = False) -> str:
    if not isinstance(value, str) or len(value) > maximum or (not allow_empty and not value):
      raise web.HTTPBadRequest(text=f"invalid validationCapture {name}")
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in value):
      raise web.HTTPBadRequest(text=f"invalid validationCapture {name}")
    return value

  @staticmethod
  def _protocol_int(value: Any, name: str, *, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
      raise web.HTTPBadRequest(text=f"invalid validationCapture {name}")
    return value

  @staticmethod
  def _protocol_number_or_none(value: Any, name: str, *, low: float, high: float) -> float | int | None:
    if value is None:
      return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
      raise web.HTTPBadRequest(text=f"invalid validationCapture {name}")
    number = float(value)
    if not math.isfinite(number) or not low <= number <= high:
      raise web.HTTPBadRequest(text=f"invalid validationCapture {name}")
    return value

  @classmethod
  def _validation_segment_route(cls, value: Any) -> tuple[str, int]:
    segment = cls._validate_segment(value)
    route, separator, index = segment.rpartition("--")
    if not separator or not route or not index.isdigit() or len(index) > 9:
      raise web.HTTPBadRequest(text="invalid validation segment")
    return route, int(index)

  def _validation_capture_metadata(
    self,
    value: Any,
    *,
    capture: str,
    files: list[dict[str, Any]],
  ) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != VALIDATION_CAPTURE_KEYS:
      raise web.HTTPBadRequest(text="validationCapture does not match protocol v1")
    if self._protocol_int(value.get("schemaVersion"), "schemaVersion", low=1, high=1) != 1:
      raise web.HTTPBadRequest(text="unsupported validationCapture schema")

    campaign_id = self._protocol_text(value.get("campaignId"), "campaignId", maximum=24)
    if not re.fullmatch(r"[0-9a-f]{24}", campaign_id):
      raise web.HTTPBadRequest(text="invalid validationCapture campaignId")
    metadata_capture = self._protocol_text(value.get("captureId"), "captureId", maximum=32)
    if metadata_capture != capture or not re.fullmatch(r"[0-9a-f]{32}", metadata_capture):
      raise web.HTTPBadRequest(text="validationCapture captureId mismatch")
    condition = self._protocol_text(value.get("condition"), "condition", maximum=40)
    if condition not in VALIDATION_CONDITIONS:
      raise web.HTTPBadRequest(text="invalid validationCapture condition")
    route = self._protocol_text(value.get("route"), "route", maximum=120)

    raw_segments = value.get("segments")
    if not isinstance(raw_segments, list) or not 1 <= len(raw_segments) <= VALIDATION_MAX_FILES:
      raise web.HTTPBadRequest(text="invalid validationCapture segments")
    segments: list[str] = []
    segment_indexes: set[int] = set()
    for raw_segment in raw_segments:
      segment_route, segment_index = self._validation_segment_route(raw_segment)
      segment = str(raw_segment)
      if segment_route != route or segment in segments or segment_index in segment_indexes:
        raise web.HTTPBadRequest(text="invalid validationCapture segments")
      segments.append(segment)
      segment_indexes.add(segment_index)
    if set(segments) != {str(item["segment"]) for item in files}:
      raise web.HTTPBadRequest(text="validationCapture segments do not match files")
    anchor = self._protocol_int(value.get("anchorSegment"), "anchorSegment", low=0, high=999_999_999)
    if anchor not in segment_indexes:
      raise web.HTTPBadRequest(text="validationCapture anchorSegment is outside the capture")

    duration = self._protocol_number_or_none(
      value.get("duration"), "duration", low=0.0, high=7 * 24 * 60 * 60,
    )
    if duration is None:
      raise web.HTTPBadRequest(text="validationCapture duration is required")
    trigger = self._protocol_text(value.get("trigger"), "trigger", maximum=64)
    if trigger not in VALIDATION_TRIGGERS:
      raise web.HTTPBadRequest(text="invalid validationCapture trigger")
    keepalive_delta = value.get("keepaliveRequestDelta")
    if keepalive_delta is not None:
      self._protocol_int(keepalive_delta, "keepaliveRequestDelta", low=0, high=10)
    qualified = value.get("qualified")
    if qualified is not None and not isinstance(qualified, bool):
      raise web.HTTPBadRequest(text="invalid validationCapture qualified")
    for name, low, high in (
      ("controllerStoppedSec", 0.0, 7 * 24 * 60 * 60),
      ("vEgo", 0.0, 100.0),
      ("aEgo", -20.0, 20.0),
      ("leadDRel", 0.0, 500.0),
      ("leadVRel", -100.0, 100.0),
      ("timeGap", 0.0, 100.0),
      ("ttc", 0.0, 100.0),
    ):
      self._protocol_number_or_none(value.get(name), name, low=low, high=high)
    self._protocol_int(value.get("detectedAt"), "detectedAt", low=1, high=4_102_444_800)
    self._protocol_int(value.get("settingsEpoch"), "settingsEpoch", low=0, high=1_000_000)

    route_settings = value.get("routeSettings")
    if not isinstance(route_settings, dict) or set(route_settings) != {
      "Ka4StockSccStandstillRearm", "PathOffset", "AdjustLaneOffset",
    }:
      raise web.HTTPBadRequest(text="invalid validationCapture routeSettings")
    rearm_enabled = self._protocol_int(
      route_settings.get("Ka4StockSccStandstillRearm"),
      "routeSettings.Ka4StockSccStandstillRearm",
      low=0,
      high=1,
    )
    if rearm_enabled != 1:
      raise web.HTTPBadRequest(text="validationCapture KA4 stock-SCC rearm gate did not pass")
    self._protocol_int(route_settings.get("PathOffset"), "routeSettings.PathOffset", low=-150, high=150)
    self._protocol_int(
      route_settings.get("AdjustLaneOffset"), "routeSettings.AdjustLaneOffset", low=0, high=500,
    )

    git = value.get("git")
    if not isinstance(git, dict) or set(git) != {"branch", "commit", "dirty", "topology"}:
      raise web.HTTPBadRequest(text="invalid validationCapture git identity")
    self._protocol_text(git.get("branch"), "git.branch", maximum=128)
    self._protocol_text(git.get("commit"), "git.commit", maximum=64)
    if not isinstance(git.get("dirty"), bool):
      raise web.HTTPBadRequest(text="invalid validationCapture git.dirty")
    topology = git.get("topology")
    topology_keys = {
      "carFingerprint", "pcmCruise", "openpilotLongitudinalControl", "flags",
      "alternativeExperience", "safetyConfigs", "gatePassed",
    }
    if not isinstance(topology, dict) or set(topology) != topology_keys:
      raise web.HTTPBadRequest(text="invalid validationCapture git.topology")
    car_fingerprint = self._protocol_text(
      topology.get("carFingerprint"), "git.topology.carFingerprint", maximum=128,
    )
    if car_fingerprint != KA4_STOCK_SCC_CAR_FINGERPRINT:
      raise web.HTTPBadRequest(text="validationCapture topology is outside the KA4 gate")
    if topology.get("pcmCruise") is not True or topology.get("openpilotLongitudinalControl") is not False:
      raise web.HTTPBadRequest(text="validationCapture topology is outside the stock-SCC gate")
    flags = self._protocol_int(
      topology.get("flags"), "git.topology.flags", low=0, high=0xFFFFFFFFFFFFFFFF,
    )
    required_flags = HYUNDAI_FLAG_CANFD | HYUNDAI_FLAG_RADAR_SCC
    if (
      flags & required_flags != required_flags
      or flags & HYUNDAI_FLAG_CAMERA_SCC
      or flags & HYUNDAI_FLAG_CANFD_HDA2
    ):
      raise web.HTTPBadRequest(text="validationCapture topology is outside the KA4 stock-SCC flag gate")
    self._protocol_int(
      topology.get("alternativeExperience"), "git.topology.alternativeExperience", low=0, high=0xFFFFFFFF,
    )
    if topology.get("gatePassed") is not True:
      raise web.HTTPBadRequest(text="validationCapture topology gate did not pass")
    safety_configs = topology.get("safetyConfigs")
    if not isinstance(safety_configs, list) or not 1 <= len(safety_configs) <= 8:
      raise web.HTTPBadRequest(text="invalid validationCapture safetyConfigs")
    safety_models: set[str] = set()
    for safety_config in safety_configs:
      if not isinstance(safety_config, dict) or set(safety_config) != {"model", "param"}:
        raise web.HTTPBadRequest(text="invalid validationCapture safetyConfig")
      safety_models.add(self._protocol_text(
        safety_config.get("model"), "git.topology.safetyConfig.model", maximum=64,
      ))
      self._protocol_int(
        safety_config.get("param"), "git.topology.safetyConfig.param", low=0, high=0xFFFFFFFF,
      )
    if HYUNDAI_CANFD_SAFETY_MODEL not in safety_models:
      raise web.HTTPBadRequest(text="validationCapture topology lacks Hyundai CAN-FD safety")
    return value

  def _validation_manifest_files(self, value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or not value or len(value) > VALIDATION_MAX_FILES:
      raise web.HTTPBadRequest(text="validation manifest must contain 1 to 3 files")
    files: list[dict[str, Any]] = []
    segments: set[str] = set()
    for item in value:
      if not isinstance(item, dict) or set(item) != {"segment", "name", "size", "sha256"}:
        raise web.HTTPBadRequest(text="invalid validation manifest file entry")
      if not all(isinstance(item.get(name), str) for name in ("segment", "name", "sha256")):
        raise web.HTTPBadRequest(text="invalid validation manifest file entry types")
      segment = self._validate_segment(item.get("segment"))
      self._validation_segment_route(segment)
      filename = self._validate_validation_filename(item.get("name"))
      size = item.get("size")
      if isinstance(size, bool) or not isinstance(size, int) or size <= 0 or size > self.config.max_file_bytes:
        raise web.HTTPBadRequest(text="invalid validation manifest file size")
      sha256 = self._validate_sha256(item.get("sha256"))
      if segment in segments:
        raise web.HTTPBadRequest(text="duplicate validation manifest segment")
      segments.add(segment)
      files.append({"segment": segment, "name": filename, "size": size, "sha256": sha256})
    return sorted(files, key=lambda item: (item["segment"], item["name"]))

  async def _verify_validation_file_set(
    self,
    device: str,
    capture: str,
    files: list[dict[str, Any]],
    recorded_rows: list[sqlite3.Row],
  ) -> None:
    recorded_files = [{
      "segment": str(row["segment"]),
      "name": str(row["filename"]),
      "size": int(row["size"]),
      "sha256": str(row["sha256"]),
    } for row in recorded_rows]
    if recorded_files != files:
      raise web.HTTPConflict(text="validation manifest does not exactly match received files")

    for row in recorded_rows:
      expected_path = self._path(
        VALIDATION_NAMESPACE,
        device,
        capture,
        str(row["segment"]),
        str(row["filename"]),
      )
      expected_relative_path = expected_path.relative_to(self.config.storage_root.resolve()).as_posix()
      if str(row["relative_path"]) != expected_relative_path or not expected_path.is_file():
        raise web.HTTPConflict(text="a recorded validation file is unavailable")
      try:
        actual_size, actual_sha256 = await self._hash_file_bounded(expected_path, device=device)
      except OSError as exc:
        raise web.HTTPConflict(text="a recorded validation file could not be verified") from exc
      if actual_size != int(row["size"]) or actual_sha256 != str(row["sha256"]):
        raise web.HTTPConflict(text="a recorded validation file failed integrity verification")

  async def validation_complete(self, request: web.Request) -> web.Response:
    session = await self.authenticate(request, purposes={VALIDATION_PURPOSE})
    device = str(session["device_id"])
    await self._enter_upload(device, validation=True)
    try:
      return await self._validation_complete_active(
        request,
        device=device,
        source_ip=str(session["source_ip"]),
      )
    finally:
      await self._leave_upload(device, validation=True)

  async def _validation_complete_active(
    self,
    request: web.Request,
    *,
    device: str,
    source_ip: str,
  ) -> web.Response:
    declared = request.content_length
    if declared is not None and declared > VALIDATION_COMPLETION_BODY_MAX:
      raise web.HTTPRequestEntityTooLarge(max_size=VALIDATION_COMPLETION_BODY_MAX, actual_size=declared)
    reserve_bytes = VALIDATION_COMPLETION_BODY_MAX + 1 if declared is None else min(
      max(0, declared), VALIDATION_COMPLETION_BODY_MAX,
    )
    reservation = await self._reserve(
      device,
      source_ip,
      reserve_bytes,
      storage_amount=0,
      validation=True,
    )
    received = 0
    stored = 0

    async def record_body_progress(amount: int) -> None:
      nonlocal received
      received = max(received, amount)
      await self._record_reservation_progress(
        reservation,
        received,
        disk_written_bytes=0,
      )

    try:
      body, received = await self._json_body(
        request,
        VALIDATION_COMPLETION_BODY_MAX,
        on_progress=record_body_progress,
        include_size=True,
      )
      if not isinstance(body, dict) or set(body) != {
        "deviceId", "captureId", "files", "validationCapture",
      }:
        raise web.HTTPBadRequest(text="validation completion does not match protocol v1")
      body_device = self._validate_device(body.get("deviceId"))
      if body_device != device:
        raise web.HTTPForbidden(text="completion device mismatch")
      capture = self._validate_capture(body.get("captureId"))
      files = self._validation_manifest_files(body.get("files"))
      validation_capture = self._validation_capture_metadata(
        body.get("validationCapture"),
        capture=capture,
        files=files,
      )

      canonical_manifest = {
        "captureId": capture,
        "deviceAuthVersion": DEVICE_AUTH_VERSION,
        "deviceId": device,
        "files": files,
        "receiptVersion": RECEIPT_VERSION,
        "validationCapture": validation_capture,
      }
      encoded = self._canonical_json(canonical_manifest)
      if len(encoded) > VALIDATION_MANIFEST_MAX:
        raise web.HTTPRequestEntityTooLarge(max_size=VALIDATION_MANIFEST_MAX, actual_size=len(encoded))
      manifest_sha256 = hashlib.sha256(encoded).hexdigest()
      receipt_id = hashlib.sha256(
        f"carrot-validation-receipt-v1\0{manifest_sha256}".encode("ascii"),
      ).hexdigest()
      manifest_path = self._path(VALIDATION_NAMESPACE, device, capture, "manifest.json")
      relative_path = manifest_path.relative_to(self.config.storage_root.resolve()).as_posix()
      temp = manifest_path.with_name(f".manifest.{secrets.token_hex(16)}.part")

      response_body = {
        "ok": True,
        "receiptVersion": RECEIPT_VERSION,
        "receiptId": receipt_id,
        "manifestSha256": manifest_sha256,
        "files": files,
        "verifiedDeviceId": device,
        "deviceId": device,
        "captureId": capture,
      }

      async with self._db_lock:
        with self._connect() as connection:
          existing_completion = connection.execute(
            "SELECT * FROM validation_completions WHERE device_id=? AND capture_id=?",
            (device, capture),
          ).fetchone()
          recorded_rows = connection.execute(
            """SELECT segment, filename, size, sha256, relative_path
               FROM validation_files WHERE device_id=? AND capture_id=?
               ORDER BY segment, filename""",
            (device, capture),
          ).fetchall()

      if existing_completion is not None:
        if (
          str(existing_completion["manifest_sha256"]) != manifest_sha256
          or bytes(existing_completion["manifest_json"]) != encoded
        ):
          raise web.HTTPConflict(text="validation completion manifest is immutable")
        if str(existing_completion["relative_path"]) != relative_path or not manifest_path.is_file():
          raise web.HTTPInternalServerError(text="recorded validation manifest is unavailable")
        if await asyncio.to_thread(manifest_path.read_bytes) != encoded:
          raise web.HTTPInternalServerError(text="recorded validation manifest failed integrity verification")
        await self._verify_validation_file_set(device, capture, files, recorded_rows)
        return web.json_response(response_body)

      # A new completion persists the canonical bytes twice: the immutable
      # manifest file and the SQLite recovery BLOB. Reserve both copies before
      # expensive verification or any durable publication.
      await self._grow_storage_reservation(reservation, 2 * len(encoded))

      # Full rlogs can be hundreds of MiB. Re-hash outside both the asyncio DB
      # mutex and a SQLite transaction, then re-check the immutable receipt set
      # under BEGIN IMMEDIATE immediately before publishing the receipt.
      await self._verify_validation_file_set(device, capture, files, recorded_rows)
      await self._record_reservation_progress(
        reservation,
        received,
        disk_written_bytes=0,
      )

      manifest_file_added = False
      try:
        async with self._db_lock:
          with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            existing_completion = connection.execute(
              "SELECT * FROM validation_completions WHERE device_id=? AND capture_id=?",
              (device, capture),
            ).fetchone()
            if existing_completion is not None:
              if (
                str(existing_completion["manifest_sha256"]) != manifest_sha256
                or bytes(existing_completion["manifest_json"]) != encoded
              ):
                raise web.HTTPConflict(text="validation completion manifest is immutable")
              if str(existing_completion["relative_path"]) != relative_path or not manifest_path.is_file():
                raise web.HTTPInternalServerError(text="recorded validation manifest is unavailable")
              if manifest_path.read_bytes() != encoded:
                raise web.HTTPInternalServerError(text="recorded validation manifest failed integrity verification")
              return web.json_response(response_body)

            current_rows = connection.execute(
              """SELECT segment, filename, size, sha256, relative_path
                 FROM validation_files WHERE device_id=? AND capture_id=?
                 ORDER BY segment, filename""",
              (device, capture),
            ).fetchall()
            current_snapshot = [tuple(row) for row in current_rows]
            if current_snapshot != [tuple(row) for row in recorded_rows]:
              raise web.HTTPConflict(text="validation receipts changed during completion")

            self._mkdir_parents_durable(manifest_path.parent)
            with temp.open("xb") as output:
              output.write(encoded)
              output.flush()
              os.fsync(output.fileno())
            try:
              os.link(temp, manifest_path)
              manifest_file_added = True
              self._fsync_directory(manifest_path.parent)
            except FileExistsError:
              if manifest_path.read_bytes() != encoded:
                raise web.HTTPConflict(text="validation completion manifest is immutable") from None
            connection.execute(
              """INSERT INTO validation_completions(
                   device_id, capture_id, receipt_id, manifest_sha256, manifest_json, relative_path, completed_at
                 ) VALUES(?,?,?,?,?,?,?)""",
              (
                device, capture, receipt_id, manifest_sha256, encoded, relative_path,
                int(datetime.now(UTC).timestamp()),
              ),
            )
            # A manifest without its completion row is the crash residue from
            # the prior link+fsync-before-commit attempt. That file was never
            # charged, so completing recovery accounts the final file and BLOB
            # copies even when this retry did not create the hard link itself.
            stored = 2 * len(encoded)
            reservation_updated = connection.execute(
              """UPDATE upload_reservations
                 SET stored_bytes=?, disk_written_bytes=?, touched_at=?
                 WHERE reservation_id=?""",
              (
                stored,
                stored,
                int(datetime.now(UTC).timestamp()),
                reservation.reservation_id,
              ),
            ).rowcount
            if reservation_updated != 1:
              raise web.HTTPServiceUnavailable(text="upload reservation expired")
      except Exception:
        if manifest_file_added:
          self._unlink_durable(manifest_path)
          manifest_file_added = False
        stored = 0
        raise
      finally:
        self._unlink_durable(temp)

      await self._record_reservation_progress(
        reservation,
        received,
        stored,
        disk_written_bytes=stored,
      )
      return web.json_response(response_body)
    finally:
      await self._finish_reservation(reservation, received, stored)

  async def complete(self, request: web.Request) -> web.Response:
    self._require_legacy_uploads_enabled()
    session = await self.authenticate(request, purposes={"dashcam"})
    body = await self._json_body(request, 512 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    meta = body.get("meta") if isinstance(body, dict) else {}
    meta_device = str((meta or {}).get("dongleId") or "").strip()
    if meta_device.lower() in {"", "unknown", "none"}:
      meta_device = ""
    body_device = meta_device or str(body.get("deviceId") or "").strip()
    if body_device and body_device != session["device_id"]:
      raise web.HTTPForbidden(text="completion device mismatch")
    state_root = self.config.db_path.parent.resolve()
    directory = state_root / "manifests" / session["device_id"]
    if state_root not in directory.resolve().parents:
      raise web.HTTPBadRequest(text="invalid manifest path")
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    path = directory / f"{stamp}.json"
    safe_body = dict(body) if isinstance(body, dict) else {}
    safe_body["receivedAt"] = datetime.now(UTC).isoformat()
    encoded = json.dumps(safe_body, ensure_ascii=False, indent=2).encode("utf-8")
    reservation = await self._reserve(session["device_id"], session["source_ip"], len(encoded))
    temp = path.with_suffix(".json.part")
    stored = 0
    try:
      self._mkdir_parents_durable(directory)
      with temp.open("xb") as output:
        output.write(encoded)
        output.flush()
        os.fsync(output.fileno())
      os.replace(temp, path)
      self._fsync_directory(path.parent)
      stored = len(encoded)
      await self._record_reservation_progress(
        reservation,
        len(encoded),
        stored,
        disk_written_bytes=len(encoded),
      )
      return web.json_response({"ok": True})
    finally:
      self._unlink_durable(temp)
      await self._finish_reservation(reservation, len(encoded), stored)

  async def tmux_upload(self, request: web.Request) -> web.Response:
    self._require_legacy_uploads_enabled()
    session = await self.authenticate(request, purposes={"tmux"})
    device = session["device_id"]
    await self._enter_upload(device)
    # Keep the original FTP-era remote layout so existing DSM workflows and
    # operators continue to find diagnostics at openpilot/<GitBranch>/....
    destination = self._path(session["git_branch"], self._storage_directory(session))
    temp_dir = destination / f".carrot-incoming-{secrets.token_hex(12)}"
    total = 0
    metadata: dict[str, str] = {}
    files: list[tuple[str, Path, int]] = []
    disk_written = 0
    stored = 0
    progress = StreamProgress()
    reservation: UploadReservation | None = None
    try:
      reservation = await self._reserve(device, session["source_ip"], self.config.max_tmux_bytes)
      self._mkdir_parents_durable(destination)
      temp_dir.mkdir(exist_ok=False)
      self._fsync_directory(temp_dir.parent)
      deadline = asyncio.get_running_loop().time() + self.config.upload_total_timeout_seconds
      reader = await self._await_body_operation(
        request.multipart,
        deadline=deadline,
        idle_timeout=self.config.upload_idle_timeout_seconds,
      )
      while True:
        part = await self._await_body_operation(
          reader.next,
          deadline=deadline,
          idle_timeout=self.config.upload_idle_timeout_seconds,
        )
        if part is None:
          break
        if part.filename:
          filename = "tmux.log" if part.name == "files[0]" else "toggle_values.json" if part.name == "files[1]" else ""
          if not filename:
            raise web.HTTPBadRequest(text="unexpected upload file")
          path = temp_dir / filename
          size = 0
          with path.open("xb", buffering=0) as output:
            while chunk := await self._await_body_operation(
              lambda part=part: part.read_chunk(MIB),
              deadline=deadline,
              idle_timeout=self.config.upload_idle_timeout_seconds,
            ):
              size += len(chunk)
              total += len(chunk)
              await self._record_stream_progress(
                reservation,
                progress,
                received_bytes=total,
                disk_written_bytes=disk_written,
              )
              if total > self.config.max_tmux_bytes:
                raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_tmux_bytes, actual_size=total)
              disk_written = await self._write_reserved_chunk(
                output,
                chunk,
                reservation,
                progress,
                network_received=total,
                disk_written=disk_written,
              )
            output.flush()
            os.fsync(output.fileno())
          files.append((filename, path, size))
        else:
          value = bytearray()
          while chunk := await self._await_body_operation(
            lambda part=part: part.read_chunk(4096),
            deadline=deadline,
            idle_timeout=self.config.upload_idle_timeout_seconds,
          ):
            value.extend(chunk)
            total += len(chunk)
            await self._record_stream_progress(
              reservation,
              progress,
              received_bytes=total,
              disk_written_bytes=disk_written,
            )
            if len(value) > 4096 or total > self.config.max_tmux_bytes:
              raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_tmux_bytes, actual_size=total)
          metadata[str(part.name or "")[:64]] = self._clean_metadata(value.decode("utf-8", errors="replace"))
      if not files or files[0][0] != "tmux.log":
        raise web.HTTPBadRequest(text="tmux.log is required")
      stamp = datetime.now(KST).strftime("%Y%m%d-%H%M%S")
      branch = session["git_branch"]
      reason = session["tmux_reason"]
      for filename, path, _size in files:
        target_name = (
          f"{reason}-{stamp}-{branch}.txt" if filename == "tmux.log" else f"toggles-{stamp}.json"
        )
        os.replace(path, destination / target_name)
        stored += _size
        await self._record_stream_progress(
          reservation,
          progress,
          received_bytes=total,
          stored_bytes=stored,
          disk_written_bytes=disk_written,
          force=True,
        )
      self._fsync_directory(destination)
      await self._record_stream_progress(
        reservation,
        progress,
        received_bytes=total,
        stored_bytes=stored,
        disk_written_bytes=disk_written,
        force=True,
      )
      return web.json_response({"ok": True, "size": total, "files": len(files)})
    finally:
      if reservation is not None:
        try:
          await self._record_stream_progress(
            reservation,
            progress,
            received_bytes=total,
            stored_bytes=stored,
            disk_written_bytes=disk_written,
            force=True,
          )
        finally:
          await self._finish_reservation(reservation, total, stored)
      if temp_dir.exists():
        shutil.rmtree(temp_dir, ignore_errors=True)
        self._fsync_directory(temp_dir.parent)
      await self._leave_upload(device)

  async def cleanup(self) -> dict[str, int]:
    # The DSM root already contains historical FTP uploads and source folders.
    # Recovery scans are limited to receiver-managed subtrees and names.
    now = int(datetime.now(UTC).timestamp())
    session_rate_buckets = self._prune_session_issue_buckets()
    oldest_usage_day = (datetime.now(UTC) - timedelta(days=2)).strftime("%Y-%m-%d")
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        stale_reservations = self._reconcile_stale_reservations_connection(
          connection, stale_before=now - self.config.reservation_lease_seconds,
        )
        expired = connection.execute("DELETE FROM sessions WHERE expires_at < ?", (now,)).rowcount
        usage = connection.execute("DELETE FROM daily_usage WHERE usage_day < ?", (oldest_usage_day,)).rowcount
        challenges = connection.execute(
          "DELETE FROM validation_challenges WHERE expires_at < ?", (now - 24 * 60 * 60,),
        ).rowcount
        connection.execute(
          """UPDATE validation_challenges SET verification_started_at=NULL
             WHERE verification_started_at IS NOT NULL AND verification_started_at < ?""",
          (now - max(2, math.ceil(self.config.validation_verify_timeout_seconds) + 2),),
        )
    stale_parts = await asyncio.to_thread(self._cleanup_stale_parts_sync, now)
    repaired_manifests, unavailable_manifests = await asyncio.to_thread(
      self._repair_completion_manifests_sync,
    )
    removed_missing_files = await asyncio.to_thread(self._reconcile_missing_incomplete_files_sync)
    unavailable_files = await asyncio.to_thread(self._audit_validation_files_sync)
    return {
      "sessions": expired,
      "usageRows": usage,
      "validationChallenges": challenges,
      "sessionRateBuckets": session_rate_buckets,
      "staleReservations": stale_reservations,
      "staleParts": stale_parts,
      "repairedManifests": repaired_manifests,
      "unavailableManifests": unavailable_manifests,
      "removedMissingIncompleteFiles": removed_missing_files,
      "unavailableValidationFiles": unavailable_files,
    }


UPLOAD_SERVICE_KEY = web.AppKey("upload_service", UploadService)
CLEANUP_TASK_KEY = web.AppKey("cleanup_task", asyncio.Task[None])


async def _cleanup_loop(app: web.Application) -> None:
  service = app[UPLOAD_SERVICE_KEY]
  while True:
    try:
      await service.cleanup()
    except Exception:
      pass
    await asyncio.sleep(service.config.cleanup_interval_seconds)


async def _start_cleanup(app: web.Application) -> None:
  app[CLEANUP_TASK_KEY] = asyncio.create_task(_cleanup_loop(app))


async def _stop_cleanup(app: web.Application) -> None:
  task = app.get(CLEANUP_TASK_KEY)
  if task:
    task.cancel()
    try:
      await task
    except asyncio.CancelledError:
      pass


@web.middleware
async def security_headers(request: web.Request, handler):
  try:
    response = await handler(request)
  except web.HTTPException as exc:
    response = web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
    for name, value in exc.headers.items():
      if name.lower() not in {"content-length", "content-type"}:
        response.headers[name] = value
  except Exception:
    logging.exception("unhandled upload request error")
    response = web.json_response({"ok": False, "error": "internal server error"}, status=500)
  response.headers["Cache-Control"] = "no-store"
  response.headers["X-Content-Type-Options"] = "nosniff"
  response.headers["Referrer-Policy"] = "no-referrer"
  return response


@web.middleware
async def request_admission(request: web.Request, handler):
  path = request.path
  validation = path.startswith("/api/v1/validation/")
  legacy = (
    path in {"/api/v1/session", "/api/v1/complete", "/api/v1/tmux/upload"}
    or path.startswith("/api/v1/upload/")
  )
  if not validation and not legacy:
    return await handler(request)
  service = request.app[UPLOAD_SERVICE_KEY]
  source_ip = service.source_ip(request)
  await service.enter_request(source_ip, validation=validation)
  try:
    return await handler(request)
  finally:
    await service.leave_request(source_ip, validation=validation)


def create_app(
  config: Config | None = None,
  *,
  start_cleanup: bool = True,
  validation_verifier: DeviceProofVerifier | None = None,
) -> web.Application:
  service = UploadService(config or Config.from_env(), validation_verifier=validation_verifier)
  app = web.Application(
    client_max_size=service.config.max_file_bytes + MIB,
    middlewares=[security_headers, request_admission],
  )
  app[UPLOAD_SERVICE_KEY] = service
  app.router.add_get("/api/v1/health", service.health)
  app.router.add_post("/api/v1/session", service.create_session)
  app.router.add_put("/api/v1/upload/{device}/{segment}/{filename}", service.upload_file)
  app.router.add_post("/api/v1/complete", service.complete)
  app.router.add_post("/api/v1/tmux/upload", service.tmux_upload)
  app.router.add_post("/api/v1/validation/challenge", service.create_validation_challenge)
  app.router.add_post("/api/v1/validation/session", service.create_validation_session)
  app.router.add_put(
    "/api/v1/validation/upload/{captureId}/{segment}/{filename}", service.validation_upload_file,
  )
  app.router.add_post("/api/v1/validation/complete", service.validation_complete)
  if start_cleanup:
    app.on_startup.append(_start_cleanup)
    app.on_cleanup.append(_stop_cleanup)
  return app


def main() -> None:
  web.run_app(
    create_app(),
    host=os.environ.get("CARROT_UPLOAD_HOST", "0.0.0.0"),
    port=_env_int("CARROT_UPLOAD_PORT", 8080, 1),
    access_log_format='%a %t "%r" %s %b %Tf',
  )


if __name__ == "__main__":
  main()
