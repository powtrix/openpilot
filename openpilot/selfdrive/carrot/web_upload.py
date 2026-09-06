from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import os
import time
import urllib.parse
from collections.abc import Callable, Mapping, Sequence
from contextlib import ExitStack
from typing import Any

from aiohttp import ClientSession, ClientTimeout
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, padding, rsa

from openpilot.common.api import get_key_pair


DEFAULT_WEB_UPLOAD_URL = "https://adot.synology.me"
DEFAULT_TMUX_WEB_UPLOAD_URL = "https://tmux.carrotpilot.app/upload"
VALIDATION_DEVICE_AUTH_VERSION = 2
VALIDATION_DEVICE_PROOF_DOMAIN = b"dk-carrot-validation-device-proof-v2\0"
VALIDATION_COMPLETION_CONNECT_TIMEOUT_SECONDS = 20
# Completion makes the receiver re-hash as many as three rlogs (up to 750 MiB).
# Bound connection establishment, but allow the authenticated integrity check
# enough read time on a slow NAS instead of imposing the generic 15 s timeout.
VALIDATION_COMPLETION_READ_TIMEOUT_SECONDS = 30 * 60


def normalize_base_url(value: Any, default: str = "") -> str:
  url = str(value or "").strip().rstrip("/")
  if not url:
    url = str(default or "").strip().rstrip("/")
  if url and not url.startswith(("http://", "https://")):
    raise ValueError("web upload URL must start with http:// or https://")
  return url


def validation_device_proof_message(
  device_id: str,
  challenge_id: str,
  nonce: str,
  audience: str,
) -> bytes:
  values = (device_id, challenge_id, nonce, audience)
  if any(not value or len(value) > 256 or "\0" in value for value in values):
    raise RuntimeError("validation challenge contains invalid proof fields")
  return VALIDATION_DEVICE_PROOF_DOMAIN + b"\0".join(value.encode("utf-8") for value in values)


def _validation_device_key_material() -> tuple[str, Any, str, str]:
  algorithm, private_pem, _public_pem = get_key_pair()
  if algorithm not in {"RS256", "ES256"} or not private_pem:
    raise RuntimeError("registered device signing key is unavailable")
  try:
    private_key = serialization.load_pem_private_key(private_pem.encode("utf-8"), password=None)
  except Exception as exc:
    raise RuntimeError("registered device signing key is invalid") from exc

  if algorithm == "RS256":
    if not isinstance(private_key, rsa.RSAPrivateKey) or private_key.key_size < 2048:
      raise RuntimeError("registered RSA device key is invalid")
  elif not isinstance(private_key, ec.EllipticCurvePrivateKey) or not isinstance(private_key.curve, ec.SECP256R1):
    raise RuntimeError("registered EC device key is invalid")

  public_key = private_key.public_key()
  public_der = public_key.public_bytes(
    serialization.Encoding.DER,
    serialization.PublicFormat.SubjectPublicKeyInfo,
  )
  public_pem = public_key.public_bytes(
    serialization.Encoding.PEM,
    serialization.PublicFormat.SubjectPublicKeyInfo,
  ).decode("ascii")
  return algorithm, private_key, public_pem, hashlib.sha256(public_der).hexdigest()


def validation_device_key_fingerprint() -> dict[str, str]:
  algorithm, _private_key, _public_pem, fingerprint = _validation_device_key_material()
  return {"algorithm": algorithm, "sha256": fingerprint}


def create_validation_device_proof(
  device_id: str,
  challenge_id: str,
  nonce: str,
  audience: str,
) -> dict[str, str]:
  algorithm, private_key, public_pem, _fingerprint = _validation_device_key_material()
  message = validation_device_proof_message(device_id, challenge_id, nonce, audience)
  if algorithm == "RS256":
    signature = private_key.sign(message, padding.PKCS1v15(), hashes.SHA256())
  else:
    signature = private_key.sign(message, ec.ECDSA(hashes.SHA256()))
  return {
    "deviceKeyAlgorithm": algorithm,
    "devicePublicKey": public_pem,
    "deviceProof": base64.urlsafe_b64encode(signature).rstrip(b"=").decode("ascii"),
  }


def web_upload_settings(settings: Mapping[str, Any] | None = None) -> tuple[str, str]:
  settings = settings or {}
  base_url = (
    os.environ.get("CARROT_WEB_UPLOAD_URL", "").strip()
    or str(settings.get("web_upload_url") or "").strip()
    or str(settings.get("toss_upload_url") or "").strip()
    or DEFAULT_WEB_UPLOAD_URL
  )
  # Normal users never configure a token. The server issues a short-lived
  # session automatically. Keep only an environment override for private
  # deployments that deliberately use a static service token.
  token = os.environ.get("CARROT_WEB_UPLOAD_TOKEN", "").strip()
  return normalize_base_url(base_url), token


def api_url(base_url: str, *parts: str) -> str:
  base_url = normalize_base_url(base_url)
  if not base_url:
    raise ValueError("web upload URL is not configured")
  quoted = "/".join(urllib.parse.quote(str(part), safe="") for part in parts)
  return f"{base_url}/api/v1/{quoted}"


def tmux_web_target(
  settings: Mapping[str, Any] | None = None,
  session_token: str = "",
) -> tuple[str, dict[str, str]]:
  base_url, token = web_upload_settings(settings)
  token = str(session_token or token).strip()
  if token:
    return api_url(base_url, "tmux", "upload"), {"Authorization": f"Bearer {token}"}

  direct_url = normalize_base_url(
    os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", ""),
    DEFAULT_TMUX_WEB_UPLOAD_URL,
  )
  return direct_url, {}


def carrot_logs_web_target() -> tuple[str, dict[str, str]]:
  """Return the independent Carrot Logs receiver used by the Discord forum.

  This target must not depend on the DSM upload token. Diagnostics are sent to
  both destinations, so a configured DSM token must never redirect this copy
  away from the Carrot Logs service.
  """
  direct_url = normalize_base_url(
    os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", ""),
    DEFAULT_TMUX_WEB_UPLOAD_URL,
  )
  return direct_url, {}


def upload_device_id(metadata: Mapping[str, Any]) -> str:
  for key in ("dongleId", "dongle_id", "deviceId", "device_id", "serial", "device_serial"):
    value = str(metadata.get(key) or "").strip()
    if value and value.lower() not in {"unknown", "none"}:
      return value
  return "unknown"


def validation_manifest_sha256(
  device_id: str,
  capture_id: str,
  files: Sequence[Mapping[str, Any]],
  validation_capture: Mapping[str, Any] | None = None,
) -> str:
  """Hash the exact canonical manifest persisted by the validation receiver."""
  normalized_files = sorted(({
    "segment": str(item.get("segment") or ""),
    "name": str(item.get("name") or ""),
    "size": int(item.get("size")),
    "sha256": str(item.get("sha256") or "").lower(),
  } for item in files), key=lambda item: (item["segment"], item["name"]))
  manifest = {
    "captureId": str(capture_id),
    "deviceAuthVersion": VALIDATION_DEVICE_AUTH_VERSION,
    "deviceId": str(device_id),
    "files": normalized_files,
    "receiptVersion": 1,
    "validationCapture": dict(validation_capture or {}),
  }
  encoded = (json.dumps(
    manifest,
    ensure_ascii=False,
    allow_nan=False,
    separators=(",", ":"),
    sort_keys=True,
  ) + "\n").encode("utf-8")
  return hashlib.sha256(encoded).hexdigest()


def validation_receipt_id(manifest_sha256: Any) -> str:
  """Derive the exact receiver v1 receipt from a canonical manifest hash."""
  manifest = str(manifest_sha256 or "")
  if (
    len(manifest) != 64
    or any(character not in "0123456789abcdef" for character in manifest)
  ):
    return ""
  return hashlib.sha256(
    b"carrot-validation-receipt-v1\0" + manifest.encode("ascii"),
  ).hexdigest()


def _session_payload(metadata: Mapping[str, Any], purpose: str) -> dict[str, str]:
  payload = {str(key): str(value or "")[:160] for key, value in metadata.items()}
  payload["deviceId"] = upload_device_id(metadata)
  payload["purpose"] = purpose
  return payload


def _session_token(body: Any) -> str:
  token = str((body or {}).get("token") or "").strip() if isinstance(body, Mapping) else ""
  if not token:
    raise RuntimeError("upload server did not issue a session")
  return token


def _ensure_validation_request_safe(should_continue: Callable[[], bool] | None) -> None:
  if should_continue is None:
    return
  try:
    allowed = bool(should_continue())
  except Exception:
    allowed = False
  if not allowed:
    raise RuntimeError("validation upload safety policy changed")


async def create_web_upload_session(
  base_url: str,
  metadata: Mapping[str, Any],
  purpose: str = "dashcam",
) -> str:
  timeout = ClientTimeout(total=12)
  async with ClientSession(timeout=timeout) as session:
    async with session.post(api_url(base_url, "session"), json=_session_payload(metadata, purpose)) as resp:
      text = await resp.text()
      try:
        body = json.loads(text)
      except Exception:
        body = None
      if not 200 <= resp.status < 300 or not (body or {}).get("ok"):
        error = str((body or {}).get("error") or text or "")[:300]
        raise RuntimeError(f"upload session HTTP {resp.status}: {error}")
      return _session_token(body)


async def create_validation_upload_session(
  base_url: str,
  metadata: Mapping[str, Any],
  should_continue: Callable[[], bool] | None = None,
) -> str:
  """Authenticate with a receiver-pinned proof from the device registration key."""
  device_id = upload_device_id(metadata)
  if device_id == "unknown":
    raise RuntimeError("registered device id is required for validation upload")
  timeout = ClientTimeout(total=15)
  async with ClientSession(timeout=timeout) as session:
    _ensure_validation_request_safe(should_continue)
    async with session.post(
      api_url(base_url, "validation", "challenge"),
      json={"deviceId": device_id},
      allow_redirects=False,
    ) as response:
      text = await response.text()
      try:
        challenge = json.loads(text)
      except Exception:
        challenge = None
      if (
        not 200 <= response.status < 300
        or not isinstance(challenge, dict)
        or challenge.get("ok") is not True
        or challenge.get("deviceAuthVersion") != VALIDATION_DEVICE_AUTH_VERSION
        or challenge.get("receiptVersion") != 1
      ):
        error = str((challenge or {}).get("error") or text or "validation authentication unavailable")[:300]
        raise RuntimeError(f"validation challenge HTTP {response.status}: {error}")

    _ensure_validation_request_safe(should_continue)
    challenge_id = str(challenge.get("challengeId") or "")
    nonce = str(challenge.get("nonce") or "")
    audience = str(challenge.get("audience") or "")
    if not challenge_id or not nonce or not audience:
      raise RuntimeError("validation challenge is incomplete")
    proof = await asyncio.to_thread(
      create_validation_device_proof,
      device_id,
      challenge_id,
      nonce,
      audience,
    )
    session_payload = _session_payload(metadata, "validation")
    session_payload.update({
      "challengeId": challenge_id,
      **proof,
    })
    _ensure_validation_request_safe(should_continue)
    async with session.post(
      api_url(base_url, "validation", "session"),
      json=session_payload,
      allow_redirects=False,
    ) as response:
      text = await response.text()
      try:
        body = json.loads(text)
      except Exception:
        body = None
      if (
        not 200 <= response.status < 300
        or not isinstance(body, dict)
        or body.get("ok") is not True
        or body.get("deviceAuthVersion") != VALIDATION_DEVICE_AUTH_VERSION
        or body.get("receiptVersion") != 1
        or body.get("verifiedDeviceId") != device_id
      ):
        error = str((body or {}).get("error") or text or "validation authentication failed")[:300]
        raise RuntimeError(f"validation session HTTP {response.status}: {error}")
      return _session_token(body)


def create_web_upload_session_sync(
  base_url: str,
  metadata: Mapping[str, Any],
  post: Callable[..., Any],
  purpose: str = "tmux",
  request_allowed: Callable[[], bool] | None = None,
) -> str:
  if request_allowed is not None and not request_allowed():
    raise PermissionError("upload request is no longer allowed")
  response = post(
    api_url(base_url, "session"),
    json=_session_payload(metadata, purpose),
    timeout=12,
  )
  try:
    body = response.json()
  except Exception:
    body = None
  status = int(getattr(response, "status_code", 0) or 0)
  if not 200 <= status < 300 or not (body or {}).get("ok"):
    text = str(getattr(response, "text", "") or "")[:300]
    error = str((body or {}).get("error") or text)[:300]
    raise RuntimeError(f"upload session HTTP {status}: {error}")
  return _session_token(body)


def post_tmux_web(
  url: str,
  headers: Mapping[str, str],
  payload: Mapping[str, Any],
  tmux_path: str,
  settings_path: str | None = None,
  post: Callable[..., Any] | None = None,
  request_allowed: Callable[[], bool] | None = None,
):
  if post is None:
    raise ValueError("web POST function is required")
  with ExitStack() as stack:
    tmux_file = stack.enter_context(open(tmux_path, "rb"))
    files = [("files[0]", ("tmux.log", tmux_file, "text/plain"))]
    if settings_path and os.path.isfile(settings_path):
      settings_file = stack.enter_context(open(settings_path, "rb"))
      files.append(("files[1]", ("toggle_values.json", settings_file, "application/json")))
    if request_allowed is not None and not request_allowed():
      raise PermissionError("upload request is no longer allowed")
    return post(
      url,
      headers=dict(headers),
      data=dict(payload),
      files=files,
      timeout=30,
    )


async def check_web_upload_health(base_url: str, token: str) -> dict[str, Any]:
  started = time.monotonic()

  def elapsed_ms() -> int:
    return int((time.monotonic() - started) * 1000)

  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      headers = {"Authorization": f"Bearer {token}"} if token else {}
      async with session.get(
        api_url(base_url, "health"),
        headers=headers,
      ) as resp:
        text = await resp.text()
        if resp.status == 200:
          result: dict[str, Any] = {"ok": True, "status": resp.status, "elapsed_ms": elapsed_ms()}
          try:
            payload = json.loads(text)
          except (TypeError, ValueError):
            payload = None
          if isinstance(payload, dict):
            if payload.get("ok") is False:
              return {
                "ok": False,
                "status": resp.status,
                "error": str(payload.get("error") or "receiver is not ready")[:300],
                "elapsed_ms": elapsed_ms(),
              }
            service = payload.get("service")
            if isinstance(service, str) and len(service) <= 64:
              result["service"] = service
            for key in ("deviceAllowlistConfigured", "legacyUploadsEnabled"):
              if isinstance(payload.get(key), bool):
                result[key] = payload[key]
          return result
        return {"ok": False, "status": resp.status, "error": text[:300], "elapsed_ms": elapsed_ms()}
  except Exception as e:
    return {"ok": False, "error": str(e), "elapsed_ms": elapsed_ms()}


async def upload_folder_to_web(
  local_folder: str,
  directory: str,
  remote_path: str,
  base_url: str,
  token: str,
  should_cancel: Callable[[], bool] | None = None,
  filenames: Sequence[str] | None = None,
  on_progress: Callable[[str, int, int, int], None] | None = None,
) -> bool:
  def check_cancel() -> None:
    if should_cancel and should_cancel():
      raise RuntimeError("upload canceled")

  check_cancel()
  if filenames is None:
    try:
      entries = sorted(entry.name for entry in os.scandir(local_folder) if entry.is_file(follow_symlinks=False))
    except OSError as e:
      raise RuntimeError(f"cannot read segment folder: {e}") from e
  else:
    entries = []
    seen: set[str] = set()
    for raw_name in filenames:
      filename = str(raw_name or "")
      if not filename or filename in (".", "..") or "/" in filename or "\\" in filename:
        raise RuntimeError("invalid upload filename")
      if filename in seen:
        continue
      local_path = os.path.join(local_folder, filename)
      if not os.path.isfile(local_path):
        raise RuntimeError(f"upload file not found: {filename}")
      seen.add(filename)
      entries.append(filename)
  if not token:
    raise RuntimeError("upload session is not configured")

  headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/octet-stream"}
  timeout = ClientTimeout(total=None, connect=20, sock_read=180)
  async with ClientSession(timeout=timeout, headers=headers) as session:
    for filename in entries:
      local_path = os.path.join(local_folder, filename)
      url = api_url(base_url, "upload", directory, remote_path, filename)
      file_size = os.path.getsize(local_path)
      sent = 0

      async def send_file(
        path: str = local_path,
        upload_name: str = filename,
        upload_size: int = file_size,
      ):
        nonlocal sent
        sent = 0
        if on_progress:
          on_progress(upload_name, sent, upload_size, 0)
        check_cancel()
        with open(path, "rb") as f:
          while True:
            check_cancel()
            chunk = f.read(1024 * 1024)
            if not chunk:
              break
            sent += len(chunk)
            if on_progress:
              on_progress(upload_name, sent, upload_size, len(chunk))
            yield chunk

      last_error: Exception | None = None
      for _attempt in range(2):
        check_cancel()
        try:
          async with session.put(url, data=send_file(), headers={"X-File-Size": str(file_size)}) as resp:
            text = await resp.text()
            try:
              body = json.loads(text)
            except Exception:
              body = None
            if not 200 <= resp.status < 300 or not (body or {}).get("ok"):
              error = str((body or {}).get("error") or text or "")[:200]
              raise RuntimeError(f"HTTP {resp.status}: {error}")
            raw_size = (body or {}).get("size")
            remote_size = int(raw_size) if raw_size is not None else -1
            if remote_size != sent:
              raise RuntimeError(f"size mismatch for {filename}: sent {sent}, remote {remote_size}")
          last_error = None
          break
        except Exception as e:
          check_cancel()
          last_error = e
      if last_error is not None:
        raise RuntimeError(f"{filename}: {last_error}") from last_error
      check_cancel()
  return True


async def upload_validation_folder_to_web(
  local_folder: str,
  segment: str,
  capture_id: str,
  base_url: str,
  token: str,
  expected_files: Sequence[Mapping[str, Any]],
  should_cancel: Callable[[], bool] | None = None,
  on_progress: Callable[[str, int, int, int], None] | None = None,
) -> bool:
  """Stream an immutable, hash-verified validation capture segment."""
  def check_cancel() -> None:
    if should_cancel and should_cancel():
      raise RuntimeError("upload canceled")

  expected_by_name = {
    str(item.get("name") or ""): item
    for item in expected_files
    if str(item.get("segment") or "") == segment
  }
  if not expected_by_name:
    raise RuntimeError(f"validation manifest has no files for {segment}")
  if not token:
    raise RuntimeError("authenticated validation upload session is not configured")

  headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/octet-stream"}
  timeout = ClientTimeout(total=None, connect=20, sock_read=180)
  async with ClientSession(timeout=timeout, headers=headers) as session:
    for filename, expected in sorted(expected_by_name.items()):
      if not filename or filename in (".", "..") or "/" in filename or "\\" in filename:
        raise RuntimeError("invalid validation filename")
      path = os.path.join(local_folder, filename)
      expected_size = int(expected.get("size") or -1)
      expected_sha256 = str(expected.get("sha256") or "").lower()
      if expected_size < 0 or len(expected_sha256) != 64 or not os.path.isfile(path):
        raise RuntimeError(f"invalid validation manifest for {filename}")
      if os.path.getsize(path) != expected_size:
        raise RuntimeError(f"local validation file size changed: {filename}")

      url = api_url(base_url, "validation", "upload", capture_id, segment, filename)
      last_error: Exception | None = None
      for _attempt in range(2):
        sent = 0
        digest = hashlib.sha256()

        async def send_file(
          source_path: str = path,
          upload_name: str = filename,
          upload_size: int = expected_size,
          upload_digest: Any = digest,
        ):
          nonlocal sent
          if on_progress:
            on_progress(upload_name, 0, upload_size, 0)
          check_cancel()
          with open(source_path, "rb") as source:
            while True:
              check_cancel()
              chunk = source.read(1024 * 1024)
              if not chunk:
                break
              upload_digest.update(chunk)
              sent += len(chunk)
              if on_progress:
                on_progress(upload_name, sent, upload_size, len(chunk))
              yield chunk

        try:
          check_cancel()
          async with session.put(
            url,
            data=send_file(),
            headers={
              "X-File-Size": str(expected_size),
              "X-Content-SHA256": expected_sha256,
            },
            allow_redirects=False,
          ) as response:
            text = await response.text()
            try:
              body = json.loads(text)
            except Exception:
              body = None
            local_sha256 = digest.hexdigest()
            if (
              not 200 <= response.status < 300
              or not isinstance(body, dict)
              or body.get("ok") is not True
              or int(body.get("size") or -1) != sent
              or str(body.get("sha256") or "").lower() != local_sha256
              or sent != expected_size
              or local_sha256 != expected_sha256
            ):
              error = str((body or {}).get("error") or text or "validation receipt mismatch")[:300]
              raise RuntimeError(f"validation upload HTTP {response.status}: {error}")
          last_error = None
          break
        except Exception as exc:
          check_cancel()
          last_error = exc
      if last_error is not None:
        raise RuntimeError(f"{filename}: {last_error}") from last_error
      check_cancel()
  return True


async def send_web_upload_complete(base_url: str, token: str, payload: dict[str, Any]) -> dict[str, Any]:
  if not token:
    return {"ok": False, "error": "upload session is not configured"}
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      async with session.post(
        api_url(base_url, "complete"),
        json=payload,
        headers={"Authorization": f"Bearer {token}"},
      ) as resp:
        text = await resp.text()
        try:
          body = json.loads(text)
        except Exception:
          body = None

        result = dict(body) if isinstance(body, dict) else {}
        result["status"] = resp.status
        result["ok"] = 200 <= resp.status < 300 and isinstance(body, dict) and body.get("ok") is True
        if not result["ok"] and not result.get("error"):
          result["error"] = text[:300] or "upload completion was not acknowledged"
        return result
  except Exception as e:
    return {"ok": False, "error": str(e)}


async def send_validation_upload_complete(
  base_url: str,
  token: str,
  payload: dict[str, Any],
  should_continue: Callable[[], bool] | None = None,
) -> dict[str, Any]:
  if not token:
    return {"ok": False, "error": "authenticated validation upload session is not configured"}
  try:
    request_payload = {
      "deviceId": str(payload.get("deviceId") or ""),
      "captureId": str(payload.get("captureId") or ""),
      "files": list(payload.get("files") or []),
      "validationCapture": (
        dict(payload.get("validationCapture"))
        if isinstance(payload.get("validationCapture"), Mapping)
        else {}
      ),
    }
    timeout = ClientTimeout(
      total=None,
      connect=VALIDATION_COMPLETION_CONNECT_TIMEOUT_SECONDS,
      sock_read=VALIDATION_COMPLETION_READ_TIMEOUT_SECONDS,
    )
    async with ClientSession(timeout=timeout) as session:
      _ensure_validation_request_safe(should_continue)
      async with session.post(
        api_url(base_url, "validation", "complete"),
        json=request_payload,
        headers={"Authorization": f"Bearer {token}"},
        allow_redirects=False,
      ) as response:
        text = await response.text()
        try:
          body = json.loads(text)
        except Exception:
          body = None
        result = dict(body) if isinstance(body, dict) else {}
        result["status"] = response.status
        try:
          expected_device_id = request_payload["deviceId"]
          expected_capture_id = request_payload["captureId"]
          expected_manifest_sha256 = validation_manifest_sha256(
            expected_device_id,
            expected_capture_id,
            request_payload["files"],
            request_payload["validationCapture"],
          )
          expected_receipt_id = validation_receipt_id(expected_manifest_sha256)
        except Exception:
          expected_device_id = ""
          expected_capture_id = ""
          expected_manifest_sha256 = ""
          expected_receipt_id = ""
        result["ok"] = (
          200 <= response.status < 300
          and isinstance(body, dict)
          and body.get("ok") is True
          and body.get("receiptVersion") == 1
          and bool(expected_device_id)
          and bool(expected_capture_id)
          and body.get("deviceId") == expected_device_id
          and body.get("verifiedDeviceId") == expected_device_id
          and body.get("captureId") == expected_capture_id
          and body.get("manifestSha256") == expected_manifest_sha256
          and body.get("receiptId") == expected_receipt_id
        )
        if not result["ok"] and not result.get("error"):
          result["error"] = text[:300] or "validation completion receipt was not verified"
        return result
  except Exception as exc:
    return {"ok": False, "error": str(exc)}
