import json
import pytest

from openpilot.selfdrive.carrot.recovery import server as recovery


@pytest.fixture(autouse=True)
def enabled_master_consent(monkeypatch):
  monkeypatch.setattr(recovery, "_third_party_data_sharing_generation", lambda: "master-generation")
  monkeypatch.setattr(
    recovery,
    "_third_party_data_sharing_generation_matches",
    lambda expected: expected == "master-generation",
  )


class _Response:
  status = 204

  @staticmethod
  def read(_limit=-1):
    return b""

  def __enter__(self):
    return self

  def __exit__(self, _exc_type, _exc, _tb):
    return False


def test_recovery_tmux_discord_upload_attaches_log(monkeypatch, tmp_path):
  tmux_log = tmp_path / "tmux.log"
  tmux_log.write_bytes(b"recovery tmux output")
  request_capture = {}

  monkeypatch.setattr(recovery, "TMUX_LOG_PATH", str(tmux_log))
  monkeypatch.setattr(recovery, "_exception_webhook_url", lambda: "https://discord.example/webhook")
  monkeypatch.setattr(recovery, "_read_param", lambda key, default="": {
    "GitBranch": "jominki354/recovery",
    "GitCommit": "0123456789abcdef",
  }.get(key, default))

  def fake_urlopen(request, timeout):
    data = request.data
    if hasattr(data, "read"):
      chunks = []
      while chunk := data.read(64 * 1024):
        chunks.append(chunk)
      request_capture["data"] = b"".join(chunks)
    else:
      request_capture["data"] = data
    request_capture["request"] = request
    request_capture["timeout"] = timeout
    return _Response()

  monkeypatch.setattr(recovery, "_open_no_redirect", fake_urlopen)

  result = recovery._send_tmux_discord("tmux_send")

  assert result == {"configured": True, "ok": True, "status": 204}
  assert request_capture["timeout"] == 12
  request = request_capture["request"]
  assert request.full_url == "https://discord.example/webhook"
  assert request.get_header("User-agent") == "CarrotRecovery/2.0"
  sent_data = request_capture["data"]
  assert b'name="files[0]"' in sent_data
  assert b"recovery tmux output" in sent_data
  assert b"tmux_send-" in sent_data
  payload_start = sent_data.index(b'{"username"')
  payload_end = sent_data.index(b"\r\n", payload_start)
  payload = json.loads(sent_data[payload_start:payload_end])
  assert payload["username"] == "Carrot Exception"
  assert payload["flags"] == 4


def test_server_tmux_log_reports_actual_discord_result(monkeypatch):
  calls = []
  monkeypatch.setattr(recovery, "_capture_tmux_log", lambda: (0, ""))
  monkeypatch.setattr(recovery, "_community_data_sharing_generation", lambda: "community-generation")
  monkeypatch.setattr(recovery, "_send_tmux_destinations", lambda reason, **_kwargs: calls.append(reason) or {
    "ok": False,
    "partial": False,
    "destinations": {},
    "error": "all destinations failed",
  })

  result = recovery._tool_action("server_tmux_log", {})

  assert calls == ["tmux_send"]
  assert result == {
    "ok": False,
    "partial": False,
    "destinations": {},
    "error": "all destinations failed",
    "file": "/download/tmux.log",
  }


def test_recovery_server_tmux_request_cannot_resume_after_master_off_on_aba(monkeypatch):
  state = {"generation": "master-generation"}
  monkeypatch.setattr(recovery, "_third_party_data_sharing_generation", lambda: state["generation"])
  monkeypatch.setattr(
    recovery,
    "_third_party_data_sharing_generation_matches",
    lambda expected: expected == state["generation"],
  )
  monkeypatch.setattr(recovery, "_community_data_sharing_generation", lambda: "community-generation")

  def capture_then_reenable():
    state["generation"] = None
    state["generation"] = "new-master-generation"
    return 0, ""

  monkeypatch.setattr(recovery, "_capture_tmux_log", capture_then_reenable)
  monkeypatch.setattr(
    recovery.Path,
    "read_bytes",
    lambda *_args, **_kwargs: pytest.fail("revoked request must not read the captured log"),
  )

  result = recovery._tool_action("server_tmux_log", {})

  assert result["ok"] is False
  assert result["disabled_by_third_party_sharing"] is True


def test_recovery_tmux_sends_to_all_main_server_destinations(monkeypatch, tmp_path):
  tmux_log = tmp_path / "tmux.log"
  tmux_log.write_bytes(b"tmux-data")
  calls = []
  payload = {"tmux_why": "tmux_send", "dongle_id": "device-id"}

  monkeypatch.setattr(recovery, "TMUX_LOG_PATH", str(tmux_log))
  monkeypatch.setattr(recovery, "_tmux_upload_payload", lambda reason: payload)
  monkeypatch.setattr(
    recovery,
    "_send_tmux_dsm",
    lambda sent_payload, raw, _master: calls.append(("dsm", sent_payload, raw)) or {"ok": True, "status": 200},
  )
  monkeypatch.setattr(
    recovery,
    "_send_tmux_carrot_logs",
    lambda sent_payload, raw, _master, _community: calls.append(("carrot_logs", sent_payload, raw)) or {"ok": True, "status": 200},
  )
  monkeypatch.setattr(
    recovery,
    "_send_tmux_discord",
    lambda reason, raw, web, _master, _community: calls.append(("discord", reason, raw, web)) or {"ok": True, "status": 204},
  )

  result = recovery._send_tmux_destinations("tmux_send")

  assert result["ok"] is True
  assert result["partial"] is False
  assert list(result["destinations"]) == ["dsm", "carrot_logs", "discord"]
  assert calls == [
    ("dsm", payload, b"tmux-data"),
    ("carrot_logs", payload, b"tmux-data"),
    ("discord", "tmux_send", b"tmux-data", {"ok": True, "status": 200}),
  ]


def test_recovery_dsm_upload_uses_automatic_session(monkeypatch):
  calls = []
  payload = {"tmux_why": "tmux_send", "dongle_id": "device-id", "car_name": "TEST"}

  monkeypatch.delenv("CARROT_WEB_UPLOAD_TOKEN", raising=False)
  monkeypatch.setattr(recovery, "_web_upload_base_url", lambda: "https://upload.example")
  monkeypatch.setattr(recovery, "_post_json", lambda url, body, **_kwargs: calls.append(("session", url, body)) or {
    "ok": True,
    "status": 200,
    "body": {"ok": True, "token": "session-token"},
  })
  monkeypatch.setattr(recovery, "_post_tmux_upload", lambda url, headers, body, raw, _allowed: calls.append(("upload", url, headers, body, raw)) or {
    "ok": True,
    "status": 200,
  })

  result = recovery._send_tmux_dsm(payload, b"tmux-data")

  assert result == {"ok": True, "status": 200}
  assert calls[0] == ("session", "https://upload.example/api/v1/session", {
    "tmux_why": "tmux_send",
    "dongle_id": "device-id",
    "car_name": "TEST",
    "deviceId": "device-id",
    "purpose": "tmux",
  })
  assert calls[1] == (
    "upload",
    "https://upload.example/api/v1/tmux/upload",
    {"Authorization": "Bearer session-token"},
    payload,
    b"tmux-data",
  )


def test_recovery_bundled_community_destinations_fail_closed(monkeypatch):
  for key in (
    "CARROT_EXCEPTION_DISCORD_WEBHOOK_URL",
    "CARROT_SUPPORT_DISCORD_WEBHOOK_URL",
    "CARROT_DISCORD_WEBHOOK_URL",
    "DISCORD_WEBHOOK_URL",
  ):
    monkeypatch.delenv(key, raising=False)
  monkeypatch.setattr(recovery, "_read_param", lambda _key, default="": default)
  monkeypatch.setattr(
    recovery,
    "_post_tmux_upload",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("community upload must be blocked")),
  )

  assert recovery._support_webhook_url() == ""
  assert recovery._exception_webhook_url() == ""
  result = recovery._send_tmux_carrot_logs({"dongle_id": "device"}, b"tmux")
  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True


def test_recovery_custom_discord_destinations_remain_independent(monkeypatch):
  monkeypatch.setattr(recovery, "_read_param", lambda _key, default="": default)
  monkeypatch.setenv("CARROT_EXCEPTION_DISCORD_WEBHOOK_URL", "https://operator.example/exception")
  monkeypatch.setenv("CARROT_SUPPORT_DISCORD_WEBHOOK_URL", "https://operator.example/support")

  assert recovery._exception_webhook_url() == "https://operator.example/exception"
  assert recovery._support_webhook_url() == "https://operator.example/support"


def test_recovery_bundled_support_notification_requires_exact_community_generation(monkeypatch):
  default_url = recovery._default_support_webhook_url()
  monkeypatch.setattr(recovery, "_support_webhook_url", lambda: default_url)
  monkeypatch.setattr(recovery, "_community_data_sharing_generation", lambda: "community-1")
  monkeypatch.setattr(
    recovery,
    "_community_data_sharing_generation_matches",
    lambda expected: expected == "community-2",
  )
  monkeypatch.setattr(
    recovery,
    "_open_no_redirect",
    lambda *_args, **_kwargs: pytest.fail("bundled notification must be blocked before network"),
  )

  result = recovery._send_support_webhook({})

  assert result["ok"] is False
  assert result["skipped"] is True
  assert result["disabled_by_community_sharing"] is True


def test_recovery_master_off_blocks_manual_tmux_before_log_read(monkeypatch):
  monkeypatch.setattr(recovery, "_third_party_data_sharing_generation", lambda: None)
  monkeypatch.setattr(
    recovery,
    "_third_party_data_sharing_generation_matches",
    lambda *_args: False,
  )
  monkeypatch.setattr(
    recovery.Path,
    "read_bytes",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("tmux log must not be read")),
  )

  result = recovery._send_tmux_destinations("tmux_send")

  assert result["ok"] is False
  assert result["disabled_by_third_party_sharing"] is True


def test_recovery_guarded_body_rejects_master_off_on_aba(monkeypatch):
  state = {"generation": "generation-1"}
  monkeypatch.setattr(recovery, "_third_party_data_sharing_generation", lambda: state["generation"])
  body = recovery._ConsentBoundBytes(
    b"x" * (128 * 1024),
    lambda: recovery._third_party_data_sharing_generation() == "generation-1",
  )

  assert body.read(64 * 1024) == b"x" * (64 * 1024)
  state["generation"] = None
  state["generation"] = "generation-2"

  with pytest.raises(PermissionError, match="consent changed"):
    body.read(64 * 1024)


def test_recovery_guarded_bodies_cap_unbounded_reads_and_recheck_after_read(monkeypatch):
  master_state = {"generation": "master-generation"}
  community_state = {"generation": "community-generation"}
  monkeypatch.setattr(recovery, "_third_party_data_sharing_generation", lambda: master_state["generation"])
  monkeypatch.setattr(
    recovery,
    "_third_party_data_sharing_generation_matches",
    lambda expected: expected == master_state["generation"],
  )
  monkeypatch.setattr(recovery, "_community_data_sharing_generation", lambda: community_state["generation"])
  monkeypatch.setattr(
    recovery,
    "_community_data_sharing_generation_matches",
    lambda expected: expected == community_state["generation"],
  )

  master_body = recovery._ConsentBoundBytes(
    b"m" * (128 * 1024),
    lambda: recovery._third_party_data_sharing_generation_matches("master-generation"),
  )
  community_body = recovery._CommunityConsentBoundBytes(
    b"c" * (128 * 1024),
    "community-generation",
  )

  assert master_body.read() == b"m" * (64 * 1024)
  assert community_body.read() == b"c" * (64 * 1024)

  master_state["generation"] = "new-master-generation"
  community_state["generation"] = "new-community-generation"
  with pytest.raises(PermissionError, match="consent changed"):
    master_body.read()
  with pytest.raises(PermissionError, match="consent changed"):
    community_body.read()


def test_recovery_sensitive_requests_install_no_redirect_handler(monkeypatch):
  captured = {}

  class Opener:
    def open(self, request, timeout):
      captured.update({"request": request, "timeout": timeout})
      return _Response()

  def build_opener(handler):
    captured["handler"] = handler
    return Opener()

  monkeypatch.setattr(recovery.urllib.request, "build_opener", build_opener)
  request = recovery.urllib.request.Request("https://upload.example/sensitive")

  with recovery._open_no_redirect(request, 12):
    pass

  assert isinstance(captured["handler"], recovery._NoRedirectHandler)
  assert captured["request"] is request
  assert captured["timeout"] == 12


def test_recovery_carrot_logs_revocation_blocks_final_network_call(monkeypatch):
  decisions = iter((True, False))
  monkeypatch.setattr(recovery, "_community_data_sharing_enabled", lambda: next(decisions))
  monkeypatch.setattr(
    recovery,
    "_request_result",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("network called after revocation")),
  )

  result = recovery._send_tmux_carrot_logs({"dongle_id": "device"}, b"tmux")

  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True


def test_recovery_bundled_discord_revocation_blocks_final_network_call(monkeypatch):
  default_url = recovery._default_exception_webhook_url()
  decisions = iter((True, False))
  monkeypatch.setattr(recovery, "_exception_webhook_url", lambda: default_url)
  monkeypatch.setattr(recovery, "_community_data_sharing_enabled", lambda: next(decisions))
  monkeypatch.setattr(
    recovery,
    "_request_result",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("network called after revocation")),
  )

  result = recovery._send_tmux_discord("tmux_send", b"tmux")

  assert result["ok"] is False
  assert result["disabled_by_community_sharing"] is True
