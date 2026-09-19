import base64
import json
import os
import subprocess
from typing import Any

from aiohttp import ClientSession, ClientTimeout

from openpilot.common.external_data import (
  third_party_data_sharing_generation,
  third_party_data_sharing_generation_matches,
)
from openpilot.selfdrive.carrot.community_data import (
  community_data_sharing_enabled,
  community_data_sharing_generation,
  community_data_sharing_generation_matches,
)
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.carrot.web_upload import guarded_async_bytes, web_upload_settings

from ...config import DASHCAM_DEFAULT_DISCORD_KEY, DASHCAM_DEFAULT_DISCORD_WEBHOOK
from ...services.dashcam_upload_report import (
  discord_content,
  upload_message_lines as upload_message_lines,
  upload_share_text as upload_share_text,
)
from ...services.params import HAS_PARAMS, Params


_CONSENT_GENERATION_UNSET = object()
from ...services.web_settings import read_web_settings


def param_text(params: Any, key: str, default: str = "unknown") -> str:
  try:
    if not params:
      return default
    value = params.get(key)
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    value = str(value or "").strip()
    return value or default
  except Exception:
    return default


def repo_dir() -> str:
  return os.environ.get("CARROT_REPO_DIR", "/data/openpilot")


def git_text(args: list[str], default: str = "") -> str:
  try:
    result = subprocess.run(
      ["git", *args],
      cwd=repo_dir(),
      capture_output=True,
      text=True,
      timeout=4,
    )
    if result.returncode == 0:
      value = (result.stdout or "").strip()
      return value or default
  except Exception:
    pass
  return default


def device_serial(params: Any) -> str:
  for key in ("HardwareSerial", "DeviceSerial", "Serial", "CarrotSerial"):
    value = param_text(params, key, "")
    if value:
      return value
  for env_key in ("CARROT_DEVICE_SERIAL", "DEVICE_SERIAL", "SERIAL"):
    value = os.environ.get(env_key, "").strip()
    if value:
      return value
  try:
    getter = getattr(HARDWARE, "get_serial", None)
    if callable(getter):
      value = str(getter() or "").strip()
      if value:
        return value
  except Exception:
    pass
  return "unknown"


def upload_metadata(params: Any) -> dict[str, str]:
  return {
    "carName": param_text(params, "CarName", "none"),
    "dongleId": param_text(params, "DongleId", "unknown"),
    "serial": device_serial(params),
    "branch": git_text(["branch", "--show-current"], "unknown"),
    "commit": git_text(["rev-parse", "--short", "HEAD"], "unknown"),
    "commitDate": git_text(["show", "-s", "--date=format:%Y-%m-%d %H:%M:%S", "--format=%cd", "HEAD"], "unknown"),
  }


def current_upload_metadata() -> dict[str, str]:
  return upload_metadata(Params() if HAS_PARAMS else None)


def decode_obfuscated(value: str, key: str) -> str:
  try:
    token = str(value or "").strip()
    key_bytes = str(key or "").encode("utf-8")
    if not token or not key_bytes:
      return ""
    raw = base64.urlsafe_b64decode(token + "=" * (-len(token) % 4))
    decoded = bytes(raw[i] ^ key_bytes[i % len(key_bytes)] for i in range(len(raw)))
    return decoded.decode("utf-8", errors="ignore").strip()
  except Exception:
    return ""


def discord_webhook_url(params: Any) -> str:
  for key in ("CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  for key in ("CarrotDiscordWebhookUrl", "CarrotDiscordWebhookURL", "DiscordWebhookUrl", "DiscordWebhookURL"):
    value = param_text(params, key, "")
    if value:
      return value
  if os.environ.get("CARROT_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return ""
  if not community_data_sharing_enabled(params):
    return ""
  return decode_obfuscated(DASHCAM_DEFAULT_DISCORD_WEBHOOK, DASHCAM_DEFAULT_DISCORD_KEY)


async def send_discord_webhook(
  url: str,
  payload: dict[str, Any],
  *,
  params: Any | None = None,
  consent_generation: str | None = None,
  community_generation: Any = _CONSENT_GENERATION_UNSET,
) -> dict[str, Any]:
  if params is None and HAS_PARAMS:
    params = Params()
  if consent_generation is None:
    consent_generation = third_party_data_sharing_generation(params)

  def master_request_allowed() -> bool:
    return third_party_data_sharing_generation_matches(consent_generation, params)

  if not master_request_allowed():
    return {
      "configured": bool(str(url or "").strip()),
      "ok": False,
      "skipped": True,
      "disabled_by_third_party_sharing": True,
    }
  url = (url or "").strip()
  if not url:
    return {"configured": False, "ok": False, "skipped": True}
  if not url.startswith(("http://", "https://")):
    return {"configured": True, "ok": False, "error": "invalid webhook url"}
  default_url = decode_obfuscated(DASHCAM_DEFAULT_DISCORD_WEBHOOK, DASHCAM_DEFAULT_DISCORD_KEY)
  requires_community_consent = url == default_url
  if community_generation is _CONSENT_GENERATION_UNSET:
    community_generation = community_data_sharing_generation(params)

  def request_allowed() -> bool:
    return (
      master_request_allowed()
      and (
        not requires_community_consent
        or community_data_sharing_generation_matches(community_generation, params)
      )
    )

  def blocked_result() -> dict[str, Any]:
    reason = (
      "disabled_by_community_sharing"
      if master_request_allowed() and requires_community_consent
      else "disabled_by_third_party_sharing"
    )
    return {"configured": True, "ok": False, "skipped": True, reason: True}

  if not request_allowed():
    return blocked_result()
  body = {
    "username": "Carrot Dashcam",
    "content": discord_content(payload),
    "allowed_mentions": {"parse": []},
    "flags": 4,
  }
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      if not request_allowed():
        return blocked_result()
      encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
      async with session.post(
        url,
        data=guarded_async_bytes(encoded, request_allowed),
        headers={
          "Content-Type": "application/json",
          "Content-Length": str(len(encoded)),
        },
        allow_redirects=False,
      ) as resp:
        text = await resp.text()
        if not request_allowed():
          return blocked_result()
        if 200 <= resp.status < 300:
          return {"configured": True, "ok": True, "status": resp.status}
        return {"configured": True, "ok": False, "status": resp.status, "error": text[:500]}
  except Exception as e:
    return {"configured": True, "ok": False, "error": str(e)}


def upload_target_settings() -> tuple[str, str]:
  try:
    settings = read_web_settings()
  except Exception:
    settings = {}
  return web_upload_settings(settings)
