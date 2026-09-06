from __future__ import annotations

import asyncio

from openpilot.selfdrive.carrot.server.features.dashcam import upload as dashcam_upload
from openpilot.selfdrive.carrot.server.services import support_discord, vision_diag


def _clear_discord_overrides(monkeypatch) -> None:
  for key in (
    "CARROT_SUPPORT_DISCORD_WEBHOOK_URL",
    "CARROT_VISION_DIAG_DISCORD_WEBHOOK_URL",
    "CARROT_DISCORD_WEBHOOK_URL",
    "DISCORD_WEBHOOK_URL",
  ):
    monkeypatch.delenv(key, raising=False)


def test_bundled_support_webhook_requires_community_consent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.setattr(support_discord, "community_data_sharing_enabled", lambda: False)

  assert support_discord.support_discord_webhook_url() == ""


def test_custom_support_webhook_remains_independent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.setattr(support_discord, "community_data_sharing_enabled", lambda: False)
  monkeypatch.setenv("CARROT_SUPPORT_DISCORD_WEBHOOK_URL", "https://operator.example/support")

  assert support_discord.support_discord_webhook_url() == "https://operator.example/support"


def test_support_send_rechecks_consent_before_bundled_request(monkeypatch):
  monkeypatch.delenv("CARROT_SUPPORT_DISCORD_WEBHOOK_DISABLE", raising=False)
  default_url = support_discord._decode_obfuscated_webhook_url()
  monkeypatch.setattr(support_discord, "support_discord_webhook_url", lambda: default_url)
  monkeypatch.setattr(support_discord, "community_data_sharing_enabled", lambda: False)

  result = asyncio.run(support_discord.send_support_webhook(None, {}))

  assert result["ok"] is False
  assert result["skipped"] is True
  assert result["disabled_by_community_sharing"] is True


def test_bundled_dashcam_completion_webhook_requires_community_consent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.delenv("CARROT_DISCORD_WEBHOOK_DISABLE", raising=False)
  monkeypatch.setattr(dashcam_upload, "community_data_sharing_enabled", lambda _params=None: False)

  assert dashcam_upload.discord_webhook_url(None) == ""


def test_custom_dashcam_completion_webhook_remains_independent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.setattr(dashcam_upload, "community_data_sharing_enabled", lambda _params=None: False)
  monkeypatch.setenv("CARROT_DISCORD_WEBHOOK_URL", "https://operator.example/dashcam")

  assert dashcam_upload.discord_webhook_url(None) == "https://operator.example/dashcam"


def test_dashcam_completion_send_rechecks_consent_before_bundled_request(monkeypatch):
  default_url = dashcam_upload.decode_obfuscated(
    dashcam_upload.DASHCAM_DEFAULT_DISCORD_WEBHOOK,
    dashcam_upload.DASHCAM_DEFAULT_DISCORD_KEY,
  )
  monkeypatch.setattr(dashcam_upload, "community_data_sharing_enabled", lambda _params=None: False)

  result = asyncio.run(dashcam_upload.send_discord_webhook(default_url, {}))

  assert result["ok"] is False
  assert result["skipped"] is True
  assert result["disabled_by_community_sharing"] is True


def test_dashcam_completion_revocation_during_preparation_blocks_post(monkeypatch):
  default_url = dashcam_upload.decode_obfuscated(
    dashcam_upload.DASHCAM_DEFAULT_DISCORD_WEBHOOK,
    dashcam_upload.DASHCAM_DEFAULT_DISCORD_KEY,
  )
  decisions = iter((True, False))
  monkeypatch.setattr(
    dashcam_upload,
    "community_data_sharing_enabled",
    lambda _params=None: next(decisions),
  )

  result = asyncio.run(dashcam_upload.send_discord_webhook(default_url, {}))

  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True


def test_bundled_vision_webhook_requires_community_consent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.setattr(vision_diag, "community_data_sharing_enabled", lambda _params=None: False)

  assert vision_diag.vision_diag_discord_webhook_url(None) == ""


def test_custom_vision_webhook_remains_independent(monkeypatch):
  _clear_discord_overrides(monkeypatch)
  monkeypatch.setattr(vision_diag, "community_data_sharing_enabled", lambda _params=None: False)
  monkeypatch.setenv("CARROT_VISION_DIAG_DISCORD_WEBHOOK_URL", "https://operator.example/vision")

  assert vision_diag.vision_diag_discord_webhook_url(None) == "https://operator.example/vision"


def test_vision_send_rechecks_consent_before_bundled_request(monkeypatch):
  default_url = vision_diag._decode_obfuscated(
    vision_diag.VISION_DIAG_DEFAULT_DISCORD_WEBHOOK,
    vision_diag.VISION_DIAG_DEFAULT_DISCORD_KEY,
  )
  monkeypatch.setattr(vision_diag, "HAS_PARAMS", False)
  monkeypatch.setattr(vision_diag, "vision_diag_discord_webhook_url", lambda _params=None: default_url)
  monkeypatch.setattr(vision_diag, "community_data_sharing_enabled", lambda _params=None: False)

  result = asyncio.run(vision_diag.upload_diagnostic_bundle_to_discord(bundle_text="diagnostic"))

  assert result["ok"] is False
  assert result["skipped"] is True
  assert result["disabled_by_community_sharing"] is True


def test_vision_revocation_during_bundle_preparation_blocks_post(monkeypatch):
  default_url = vision_diag._decode_obfuscated(
    vision_diag.VISION_DIAG_DEFAULT_DISCORD_WEBHOOK,
    vision_diag.VISION_DIAG_DEFAULT_DISCORD_KEY,
  )
  decisions = iter((True, False))
  monkeypatch.setattr(vision_diag, "HAS_PARAMS", False)
  monkeypatch.setattr(vision_diag, "vision_diag_discord_webhook_url", lambda _params=None: default_url)
  monkeypatch.setattr(
    vision_diag,
    "community_data_sharing_enabled",
    lambda _params=None: next(decisions),
  )
  monkeypatch.setattr(vision_diag, "get_server_diagnostic_snapshot", dict)
  monkeypatch.setattr(vision_diag, "_diagnostic_metadata", lambda _params=None: {})

  result = asyncio.run(vision_diag.upload_diagnostic_bundle_to_discord(bundle_text="diagnostic"))

  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True


def test_support_revocation_during_message_preparation_blocks_post(monkeypatch):
  monkeypatch.delenv("CARROT_SUPPORT_DISCORD_WEBHOOK_DISABLE", raising=False)
  default_url = support_discord._decode_obfuscated_webhook_url()
  decisions = iter((True, False))
  monkeypatch.setattr(support_discord, "support_discord_webhook_url", lambda: default_url)
  monkeypatch.setattr(support_discord, "community_data_sharing_enabled", lambda: next(decisions))

  result = asyncio.run(support_discord.send_support_webhook(None, {}))

  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True
