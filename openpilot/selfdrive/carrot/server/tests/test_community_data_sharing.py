import asyncio
from pathlib import Path

import openpilot.common.params as params_module
from openpilot.common.external_data import DK_THIRD_PARTY_DATA_SHARING_PARAM
from openpilot.selfdrive.carrot import community_data, cweb_push
from openpilot.selfdrive.carrot.server.features import setting_popular_values
from openpilot.selfdrive.carrot.server.services import auto_update, heartbeat, popular_values
from openpilot.system.manager import process_config


class FakeParams:
  def __init__(self, sharing_value, values=None):
    self.sharing_value = sharing_value
    self._dk_consent_generation = "generation-1"
    self.values = dict(values or {})
    self.values.setdefault(DK_THIRD_PARTY_DATA_SHARING_PARAM, True)

  def get(self, key, *args, **kwargs):
    del args, kwargs
    if key == community_data.COMMUNITY_DATA_SHARING_PARAM:
      if isinstance(self.sharing_value, Exception):
        raise self.sharing_value
      return self.sharing_value
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))


class SequencedParams(FakeParams):
  def __init__(self, sharing_values, values=None):
    super().__init__(None, values)
    self.sharing_values = iter(sharing_values)

  def get(self, key, *args, **kwargs):
    if key == community_data.COMMUNITY_DATA_SHARING_PARAM:
      return next(self.sharing_values)
    return super().get(key, *args, **kwargs)


class FakeResponse:
  def __init__(self, *, status=200, text="ok", json_data=None):
    self.status = status
    self._text = text
    self._json_data = json_data

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    del exc_type, exc, tb

  async def text(self):
    return self._text

  async def json(self, content_type=None):
    del content_type
    return self._json_data


class NoNetworkSession:
  def get(self, *args, **kwargs):
    raise AssertionError(f"unexpected GET: {args} {kwargs}")

  def post(self, *args, **kwargs):
    raise AssertionError(f"unexpected POST: {args} {kwargs}")


def test_community_data_param_is_persistent_bool_default_off():
  params_keys = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"
  source = params_keys.read_text(encoding="utf-8")
  assert '{"CarrotCommunityDataSharing", {PERSISTENT, BOOL, "0"}}' in source


def test_community_data_gate_fails_closed_for_missing_malformed_and_errors():
  for value in (None, False, 0, 2, b"", b"true", b"garbage", "", "true", "garbage", RuntimeError("read failed")):
    assert not community_data.community_data_sharing_enabled(FakeParams(value))

  for value in (True, 1, b"1", "1"):
    assert community_data.community_data_sharing_enabled(FakeParams(value))


def test_community_data_gate_also_requires_dk_master_consent():
  for value in (None, False, 0, 2, b"", b"true", "", "true"):
    params = FakeParams(True, {DK_THIRD_PARTY_DATA_SHARING_PARAM: value})
    assert not community_data.community_data_sharing_enabled(params)

  params = FakeParams(True, {DK_THIRD_PARTY_DATA_SHARING_PARAM: 1})
  assert community_data.community_data_sharing_enabled(params)


def test_heartbeat_rechecks_consent_before_urlopen(monkeypatch):
  params = SequencedParams(
    [True, False],
    {"Version": "v1", "GithubUsername": "user", "IsOnroad": False},
  )
  monkeypatch.setattr(heartbeat, "get_local_ip", lambda: "192.168.1.10")
  monkeypatch.setattr(heartbeat.urllib.request, "urlopen", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("network called")))

  assert heartbeat.register_my_ip_sync(params) == (False, "Community data sharing disabled")


def test_heartbeat_enabled_preserves_post(monkeypatch):
  calls = []

  class Response:
    status = 200

    def __enter__(self):
      return self

    def __exit__(self, exc_type, exc, tb):
      del exc_type, exc, tb

    def read(self):
      return b"ok"

  def fake_urlopen(request, **kwargs):
    calls.append((request, kwargs))
    return Response()

  params = FakeParams(True, {"Version": "v1", "GithubUsername": "user", "IsOnroad": False})
  monkeypatch.setattr(heartbeat, "get_local_ip", lambda: "192.168.1.10")
  monkeypatch.setattr(heartbeat.urllib.request, "urlopen", fake_urlopen)

  assert heartbeat.register_my_ip_sync(params) == (True, "ok")
  assert len(calls) == 1
  assert calls[0][0].full_url == "https://shind0.synology.me/carrot/api_heartbeat.php"
  assert "context" not in calls[0][1]


def test_cweb_push_is_gated_in_process_and_reporter(monkeypatch):
  assert process_config.enable_cweb_push(False, FakeParams(True), None)
  assert not process_config.enable_cweb_push(True, FakeParams(False), None)
  assert not process_config.enable_cweb_push(False, FakeParams(b"invalid"), None)

  monkeypatch.setattr(cweb_push, "get_local_ip", lambda iface: (_ for _ in ()).throw(AssertionError(f"IP lookup while disabled: {iface}")))
  reporter = cweb_push.CwebPushReporter(
    params=FakeParams(False),
    report_url="https://example.test/report",
    heartbeat_url="https://example.test/heartbeat",
    iface="wlan0",
    port=7000,
    timeout_s=1.0,
    heartbeat_interval_s=10.0,
    debounce_s=0.0,
  )
  monkeypatch.setattr(reporter, "_status", lambda *args, **kwargs: None)
  assert not reporter.poll_once()


def test_cweb_push_rechecks_consent_immediately_before_report(monkeypatch):
  params = SequencedParams([True, True, False], {"DongleId": "dongle"})
  calls = []
  monkeypatch.setattr(cweb_push, "get_local_ip", lambda iface: "192.168.1.10")
  monkeypatch.setattr(cweb_push, "post_json", lambda *args: calls.append(args) or (True, 200, "ok"))
  reporter = cweb_push.CwebPushReporter(
    params=params,
    report_url="https://example.test/report",
    heartbeat_url="https://example.test/heartbeat",
    iface="wlan0",
    port=7000,
    timeout_s=1.0,
    heartbeat_interval_s=10.0,
    debounce_s=0.0,
  )
  monkeypatch.setattr(reporter, "_status", lambda *args, **kwargs: None)

  assert not reporter.poll_once()
  assert not reporter.poll_once()
  assert calls == []


def test_cweb_push_rechecks_consent_immediately_before_heartbeat(monkeypatch):
  calls = []
  params = SequencedParams([True, False], {"DongleId": "dongle"})
  monkeypatch.setattr(cweb_push, "get_local_ip", lambda iface: "192.168.1.10")
  monkeypatch.setattr(cweb_push, "post_json", lambda *args: calls.append(args) or (True, 200, "ok"))
  reporter = cweb_push.CwebPushReporter(
    params=params,
    report_url="https://example.test/report",
    heartbeat_url="https://example.test/heartbeat",
    iface="wlan0",
    port=7000,
    timeout_s=1.0,
    heartbeat_interval_s=10.0,
    debounce_s=0.0,
  )
  reporter.current_candidate_ip = "192.168.1.10"
  reporter.current_candidate_since = 0.0
  reporter.last_success_ip = "192.168.1.10"
  reporter.first_report = False
  reporter.next_heartbeat_at = 0.0
  monkeypatch.setattr(reporter, "_status", lambda *args, **kwargs: None)

  assert not reporter.poll_once()
  assert calls == []


def test_cweb_push_enabled_preserves_reporting(monkeypatch):
  calls = []
  monkeypatch.setattr(cweb_push, "get_local_ip", lambda iface: "192.168.1.10")
  monkeypatch.setattr(cweb_push, "post_json", lambda *args: calls.append(args) or (True, 200, "ok"))
  reporter = cweb_push.CwebPushReporter(
    params=FakeParams(True, {"DongleId": "dongle"}),
    report_url="https://example.test/report",
    heartbeat_url="https://example.test/heartbeat",
    iface="wlan0",
    port=7000,
    timeout_s=1.0,
    heartbeat_interval_s=10.0,
    debounce_s=0.0,
  )
  monkeypatch.setattr(reporter, "_status", lambda *args, **kwargs: None)

  assert not reporter.poll_once()
  assert reporter.poll_once()
  assert calls[0][0] == "https://example.test/report"


def test_popular_value_upload_and_download_are_disabled(monkeypatch):
  params = FakeParams(False, {"CarSelected3": "CAR"})
  monkeypatch.setattr(popular_values, "HAS_PARAMS", True)
  monkeypatch.setattr(popular_values, "Params", lambda: params)
  monkeypatch.setattr(popular_values, "read_popular_values_memory", lambda: {"ok": True, "popular_values": {}})

  session = NoNetworkSession()
  assert asyncio.run(popular_values.download_popular_values_once(session)) is None
  assert not asyncio.run(popular_values.popular_value_upload_once(session))
  assert asyncio.run(popular_values.refresh_popular_values_once(session, upload=True))["uploaded"] is False


def test_popular_value_paths_recheck_consent_before_network(monkeypatch):
  session = NoNetworkSession()
  params = FakeParams(True, {"CarSelected3": "CAR"})
  monkeypatch.setattr(popular_values, "HAS_PARAMS", True)
  monkeypatch.setattr(popular_values, "Params", lambda: params)
  monkeypatch.setattr(popular_values, "_popular_url", lambda params: "https://example.test/popular")
  monkeypatch.setattr(popular_values, "_snapshot_url", lambda params: "https://example.test/snapshot")
  monkeypatch.setattr(popular_values, "_request_headers", lambda params: {})

  def revoke_while_building_hash():
    params._dk_consent_generation = "generation-2"
    return "hash"

  monkeypatch.setattr(popular_values, "_current_settings_hash", revoke_while_building_hash)
  assert asyncio.run(popular_values.download_popular_values_once(session)) is None

  params._dk_consent_generation = "generation-3"

  def revoke_while_building_snapshot():
    params._dk_consent_generation = "generation-4"
    return {"car_key": "CAR", "values": {}}

  monkeypatch.setattr(popular_values, "_current_settings_hash", lambda: "hash")
  monkeypatch.setattr(popular_values, "build_snapshot_payload", revoke_while_building_snapshot)
  assert not asyncio.run(popular_values.popular_value_upload_once(session))


def test_popular_value_enabled_preserves_upload_and_download(monkeypatch):
  params = FakeParams(True, {"CarSelected3": "CAR"})
  monkeypatch.setattr(popular_values, "HAS_PARAMS", True)
  monkeypatch.setattr(popular_values, "Params", lambda: params)
  monkeypatch.setattr(popular_values, "_popular_url", lambda params: "https://example.test/popular")
  monkeypatch.setattr(popular_values, "_snapshot_url", lambda params: "https://example.test/snapshot")
  monkeypatch.setattr(popular_values, "_current_settings_hash", lambda: "hash")
  monkeypatch.setattr(popular_values, "build_snapshot_payload", lambda: {"car_key": "CAR", "values": {"X": 1}})
  monkeypatch.setattr(popular_values, "_request_headers", lambda params: {})

  class Session:
    def __init__(self):
      self.calls = []

    def get(self, url, **kwargs):
      self.calls.append(("GET", url, kwargs))
      return FakeResponse(json_data={"ok": True, "car_key": "CAR", "popular_values": {}})

    def post(self, url, **kwargs):
      self.calls.append(("POST", url, kwargs))
      return FakeResponse()

  session = Session()
  assert asyncio.run(popular_values.popular_value_upload_once(session))
  assert asyncio.run(popular_values.download_popular_values_once(session))["car_key"] == "CAR"
  assert [call[0] for call in session.calls] == ["POST", "GET"]


def test_manual_popular_refresh_cannot_bypass_disabled_gate(monkeypatch):
  params = FakeParams(False, {"CarSelected3": "CAR"})
  monkeypatch.setattr(popular_values, "HAS_PARAMS", True)
  monkeypatch.setattr(popular_values, "Params", lambda: params)
  monkeypatch.setattr(popular_values, "read_popular_values_memory", lambda: {"ok": True, "popular_values": {}})
  monkeypatch.setattr(setting_popular_values, "read_popular_values_memory", lambda: {"ok": True, "popular_values": {}})

  request = type("Request", (), {"app": {"http": NoNetworkSession()}})()
  response = asyncio.run(setting_popular_values.api_setting_popular_values_refresh(request))
  assert response.status == 200


def _patch_notify_inputs(monkeypatch, gate_values):
  params = FakeParams(True, {"DongleId": "dongle"})
  monkeypatch.setattr(params_module, "Params", lambda: params)
  checks = iter(gate_values)
  monkeypatch.setattr(auto_update, "community_data_sharing_generation", lambda params=None: "generation")
  monkeypatch.setattr(auto_update, "community_data_sharing_generation_matches", lambda *_args: next(checks))

  async def fake_git(args, timeout):
    del timeout
    if args == ["rev-parse", "HEAD"]:
      return 0, "new-head"
    if args == ["branch", "--show-current"]:
      return 0, "dkcarrot-wip"
    if args == ["log", "--pretty=%h|%s", "old-head..new-head"]:
      return 0, "abc1234|subject"
    if args == ["diff", "--shortstat", "old-head", "new-head"]:
      return 0, "1 file changed, 2 insertions(+), 1 deletion(-)"
    raise AssertionError(args)

  monkeypatch.setattr(auto_update, "_git", fake_git)


def test_auto_update_notify_rechecks_consent_before_post(monkeypatch):
  _patch_notify_inputs(monkeypatch, [False])
  calls = []
  monkeypatch.setattr(cweb_push, "post_json", lambda *args: calls.append(args) or (True, 200, "ok"))

  asyncio.run(auto_update._notify_cwp("old-head"))
  assert calls == []


def test_auto_update_notify_enabled_preserves_post(monkeypatch):
  _patch_notify_inputs(monkeypatch, [True])
  calls = []
  monkeypatch.setattr(cweb_push, "post_json", lambda *args: calls.append(args) or (True, 200, "ok"))

  asyncio.run(auto_update._notify_cwp("old-head"))
  assert len(calls) == 1
  assert calls[0][0].endswith("/notify")
  assert calls[0][1]["deviceId"] == "dongle"
