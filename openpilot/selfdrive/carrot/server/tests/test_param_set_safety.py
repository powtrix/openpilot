from __future__ import annotations

import asyncio
import json

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import pytest

from openpilot.selfdrive.carrot.server.features import params as params_feature
from openpilot.selfdrive.carrot.server.services import params as params_service
from openpilot.selfdrive.carrot.server.services import web_consent


class FakeParamKeyType:
  BOOL = "BOOL"
  INT = "INT"
  FLOAT = "FLOAT"
  STRING = "STRING"
  JSON = "JSON"
  TIME = "TIME"
  BYTES = "BYTES"


class FakeParams:
  def __init__(self, *, is_offroad: bool, is_onroad: bool):
    self.values = {"IsOffroad": is_offroad, "IsOnroad": is_onroad}

  def get_bool(self, name: str) -> bool:
    return self.values[name]


class FakeRequest:
  def __init__(
    self,
    value,
    params,
    name=None,
    *,
    content_type="application/json",
    headers=None,
    app=None,
    scheme="http",
    remote="127.0.0.1",
  ):
    self._body = {
      "name": name or params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
      "value": value,
      "source": "test",
    }
    self.app = app if app is not None else {"params": params}
    self.app.setdefault("params", params)
    self.scheme = scheme
    self.remote = remote
    self.transport = None
    self.content_type = content_type
    if headers is None:
      self.headers = {
        "Host": "192.168.50.95:7000",
        "Origin": "http://192.168.50.95:7000",
        "Sec-Fetch-Site": "same-origin",
        "X-Carrot-Web-Request": "1",
      }
      web_consent.initialize_web_consent_sessions(self.app)
      token = web_consent.issue_web_consent_session(self)
      assert token is not None
      self.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = token
    else:
      self.headers = headers

  async def json(self):
    return self._body


class FakeRuntimeParams:
  def __init__(self):
    self.values = {
      params_service.VALIDATION_AUTO_UPLOAD_PARAM: 0,
      params_service.COMMUNITY_DATA_SHARING_PARAM: 0,
      params_service.THIRD_PARTY_DATA_SHARING_PARAM: 0,
      "OrdinarySetting": 0,
    }

  def all_keys(self):
    return list(self.values)

  def get_type(self, name):
    if name not in self.values:
      raise KeyError(name)
    return FakeParamKeyType.INT

  def get_default_value(self, name):
    if name not in self.values:
      raise KeyError(name)
    return 0

  def get(self, name, **kwargs):
    if name not in self.values:
      return None
    return self.values[name]

  def get_int(self, name):
    return int(self.values[name])

  def put_int(self, name, value):
    self.values[name] = int(value)


@pytest.fixture
def param_runtime(monkeypatch):
  runtime = FakeRuntimeParams()
  definitions = {
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: {"default": 0, "min": 0, "max": 1},
    params_service.COMMUNITY_DATA_SHARING_PARAM: {"default": 0, "min": 0, "max": 1},
    params_service.THIRD_PARTY_DATA_SHARING_PARAM: {"default": 0, "min": 0, "max": 1},
    "OrdinarySetting": {"default": 0, "min": 0, "max": 10},
  }
  monkeypatch.setattr(params_service, "HAS_PARAMS", True)
  monkeypatch.setattr(params_service, "ParamKeyType", FakeParamKeyType)
  monkeypatch.setattr(params_service, "Params", lambda: runtime)
  monkeypatch.setattr(params_service, "_catalog_definitions", lambda: definitions)
  monkeypatch.setattr(params_feature, "HAS_PARAMS", True)
  monkeypatch.setattr(params_feature, "ParamKeyType", FakeParamKeyType)
  return runtime


def call_param_set(monkeypatch, *, previous, value, params, name=None):
  name = name or params_feature.VALIDATION_AUTO_UPLOAD_PARAM
  writes = []
  setting = {"default": 0, "min": 0, "max": 1}
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: setting,
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {
    name: previous,
  })
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda name, new_value, metadata, **kwargs: writes.append((name, new_value)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)

  response = asyncio.run(params_feature.api_param_set(FakeRequest(value, params, name=name)))
  return response.status, json.loads(response.text), writes


def test_validation_auto_upload_enable_requires_explicit_offroad_state(monkeypatch):
  status, payload, writes = call_param_set(
    monkeypatch,
    previous=0,
    value=1,
    params=FakeParams(is_offroad=False, is_onroad=True),
  )

  assert status == 409
  assert payload["ok"] is False
  assert "offroad" in payload["error"]
  assert writes == []


@pytest.mark.parametrize(("content_type", "headers"), [
  ("text/plain", {"X-Carrot-Web-Request": "1"}),
  ("application/json", {}),
  ("application/json", {"X-Carrot-Web-Request": "0"}),
])
def test_validation_auto_upload_enable_requires_explicit_web_consent_request(
  monkeypatch, content_type, headers,
):
  writes = []
  setting = {"default": 0, "min": 0, "max": 1}
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    params_feature.VALIDATION_AUTO_UPLOAD_PARAM: setting,
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {
    params_feature.VALIDATION_AUTO_UPLOAD_PARAM: 0,
  })
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda *args, **kwargs: writes.append((args, kwargs)),
  )

  request = FakeRequest(
    1,
    FakeParams(is_offroad=True, is_onroad=False),
    content_type=content_type,
    headers=headers,
  )
  response = asyncio.run(params_feature.api_param_set(request))

  assert response.status == 403
  assert json.loads(response.text)["ok"] is False
  assert writes == []


@pytest.mark.parametrize(("previous", "value", "is_offroad", "is_onroad"), [
  (0, 1, True, False),
  (1, 0, False, True),
  (1, 1, True, False),
])
def test_validation_auto_upload_positive_writes_require_offroad(
  monkeypatch, previous, value, is_offroad, is_onroad,
):
  status, payload, writes = call_param_set(
    monkeypatch,
    previous=previous,
    value=value,
    params=FakeParams(is_offroad=is_offroad, is_onroad=is_onroad),
  )

  assert status == 200
  assert payload["ok"] is True
  assert writes == [(params_feature.VALIDATION_AUTO_UPLOAD_PARAM, value)]


def test_validation_auto_upload_onroad_one_to_one_is_still_rejected(monkeypatch):
  status, payload, writes = call_param_set(
    monkeypatch,
    previous=1,
    value=1,
    params=FakeParams(is_offroad=False, is_onroad=True),
  )

  assert status == 409
  assert payload["ok"] is False
  assert writes == []


def test_validation_auto_upload_enable_fails_closed_without_request_params(monkeypatch):
  status, payload, writes = call_param_set(monkeypatch, previous=0, value=1, params=None)

  assert status == 409
  assert payload["ok"] is False
  assert writes == []


@pytest.mark.parametrize("name", [
  params_feature.COMMUNITY_DATA_SHARING_PARAM,
  params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_external_data_sharing_enable_requires_explicit_offroad_consent(monkeypatch, name):
  status, payload, writes = call_param_set(
    monkeypatch,
    previous=0,
    value=1,
    params=FakeParams(is_offroad=False, is_onroad=True),
    name=name,
  )
  assert status == 409
  assert payload["ok"] is False
  assert "offroad" in payload["error"]
  assert writes == []

  status, payload, writes = call_param_set(
    monkeypatch,
    previous=0,
    value=1,
    params=FakeParams(is_offroad=True, is_onroad=False),
    name=name,
  )
  assert status == 200
  assert payload["ok"] is True
  assert writes == [(name, 1)]


@pytest.mark.parametrize("name", [
  params_feature.COMMUNITY_DATA_SHARING_PARAM,
  params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_external_data_sharing_enable_requires_same_origin_json_marker(monkeypatch, name):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: {"default": 0, "min": 0, "max": 1},
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 0})
  monkeypatch.setattr(params_feature, "set_param_value", lambda *args, **kwargs: writes.append((args, kwargs)))

  request = FakeRequest(
    1,
    FakeParams(is_offroad=True, is_onroad=False),
    name=name,
    headers={},
  )
  response = asyncio.run(params_feature.api_param_set(request))

  assert response.status == 403
  assert "explicit" in json.loads(response.text)["error"]
  assert writes == []


@pytest.mark.parametrize("name", [
  params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
  params_feature.COMMUNITY_DATA_SHARING_PARAM,
  params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
])
@pytest.mark.parametrize(("host", "origin", "fetch_site"), [
  ("192.168.50.95:7000", "http://attacker.example", "cross-site"),
  ("attacker.example:7000", "http://attacker.example:7000", "same-origin"),
  ("192.168.50.95:7000", "http://192.168.50.95:7001", "same-origin"),
  ("192.168.50.95:7000", "https://192.168.50.95:7000", "same-origin"),
  ("192.168.50.95:7000", "null", "same-origin"),
])
def test_high_risk_enable_rejects_cross_origin_and_dns_rebinding_requests(
  monkeypatch, name, host, origin, fetch_site,
):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: {"default": 0, "min": 0, "max": 1},
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 0})
  monkeypatch.setattr(params_feature, "set_param_value", lambda *args, **kwargs: writes.append((args, kwargs)))

  app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(app)
  trusted = FakeRequest(
    1,
    app["params"],
    name=name,
    app=app,
    headers={"Host": "192.168.50.95:7000", "Origin": "http://192.168.50.95:7000"},
  )
  token = web_consent.issue_web_consent_session(trusted)
  assert token is not None
  request = FakeRequest(
    1,
    app["params"],
    name=name,
    app=app,
    headers={
      "Host": host,
      "Origin": origin,
      "Sec-Fetch-Site": fetch_site,
      "X-Carrot-Web-Request": "1",
      web_consent.WEB_CONSENT_TOKEN_HEADER: token,
    },
  )
  response = asyncio.run(params_feature.api_param_set(request))

  assert response.status == 403
  assert json.loads(response.text)["error_code"] == "WEB_CONSENT_PROOF_REQUIRED"
  assert writes == []


def test_consent_session_is_short_lived_one_use_and_origin_bound():
  app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(app)
  headers = {
    "Host": "192.168.50.95:7000",
    "Origin": "http://192.168.50.95:7000",
    "Sec-Fetch-Site": "same-origin",
    "X-Carrot-Web-Request": "1",
  }
  request = FakeRequest(1, app["params"], app=app, headers=headers)
  token = web_consent.issue_web_consent_session(request, now=100.0)
  assert token is not None
  request.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = token

  assert web_consent.consume_web_consent_session(request, now=100.1) is True
  assert web_consent.consume_web_consent_session(request, now=100.2) is False

  expired = web_consent.issue_web_consent_session(request, now=200.0)
  assert expired is not None
  request.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = expired
  assert web_consent.consume_web_consent_session(
    request,
    now=200.0 + web_consent.WEB_CONSENT_SESSION_TTL_SECONDS,
  ) is False


def test_consent_session_is_tether_gateway_authenticated_and_peer_bound(monkeypatch):
  app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(app)
  monkeypatch.setattr(
    web_consent,
    "_default_tether_gateway_addresses",
    lambda: frozenset({"192.168.50.1"}),
  )
  headers = {
    "Host": "192.168.50.95:7000",
    "Origin": "http://192.168.50.95:7000",
    "Sec-Fetch-Site": "same-origin",
    "X-Carrot-Web-Request": "1",
  }

  lan_peer = FakeRequest(
    1, app["params"], app=app, headers=dict(headers), remote="192.168.50.20",
  )
  assert web_consent.issue_web_consent_session(lan_peer) is None

  tether_owner = FakeRequest(
    1, app["params"], app=app, headers=dict(headers), remote="192.168.50.1",
  )
  token = web_consent.issue_web_consent_session(tether_owner, now=100.0)
  assert token is not None
  tether_owner.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = token

  # Even a valid token cannot move to another same-origin LAN peer.
  stolen = FakeRequest(
    1, app["params"], app=app, headers=dict(tether_owner.headers), remote="192.168.50.20",
  )
  assert web_consent.consume_web_consent_session(stolen, now=100.1) is False

  # The tether host itself retains the intended positive path with a fresh
  # one-use token after the rejected transfer attempt consumed the first one.
  token = web_consent.issue_web_consent_session(tether_owner, now=101.0)
  assert token is not None
  tether_owner.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = token
  assert web_consent.consume_web_consent_session(tether_owner, now=101.1) is True


def test_default_tether_gateway_parser_uses_lowest_metric_wifi_routes(tmp_path):
  ipv4 = tmp_path / "route"
  ipv4.write_text(
    "".join([
      "Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\n",
      "rmnet0 00000000 0101A8C0 0003 0 0 10 00000000 0 0 0\n",
      "wlan0 00000000 010014AC 0003 0 0 600 00000000 0 0 0\n",
      "wlan0 00000000 020014AC 0003 0 0 700 00000000 0 0 0\n",
    ]),
    encoding="ascii",
  )
  ipv6 = tmp_path / "ipv6_route"
  ipv6.write_text(
    "".join([
      "00000000000000000000000000000000 00 ",
      "00000000000000000000000000000000 00 ",
      "fe800000000000000000000000000001 00000258 00000000 00000000 00000003 wlan0\n",
      "00000000000000000000000000000000 00 ",
      "00000000000000000000000000000000 00 ",
      "fe800000000000000000000000000002 000002bc 00000000 00000000 00000003 wlan0\n",
    ]),
    encoding="ascii",
  )

  assert web_consent._default_tether_gateway_addresses(str(ipv4), str(ipv6)) == {
    "172.20.0.1",
    "fe80::1",
  }


def test_consent_session_endpoint_is_no_store_and_rejects_public_hostnames():
  private_app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(private_app)
  private_request = FakeRequest(
    1,
    private_app["params"],
    app=private_app,
    headers={
      "Host": "192.168.50.95:7000",
      "Referer": "http://192.168.50.95:7000/settings",
      "Sec-Fetch-Site": "same-origin",
    },
  )
  response = asyncio.run(params_feature.api_web_consent_session(private_request))
  payload = json.loads(response.text)
  assert response.status == 200
  assert payload["ok"] is True
  assert payload["expires_in"] == web_consent.WEB_CONSENT_SESSION_TTL_SECONDS
  assert payload["token"] in private_app[web_consent.WEB_CONSENT_SESSIONS_KEY]
  assert response.headers["Cache-Control"] == "no-store"

  public_app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(public_app)
  public_request = FakeRequest(
    1,
    public_app["params"],
    app=public_app,
    headers={
      "Host": "attacker.example:7000",
      "Origin": "http://attacker.example:7000",
      "Sec-Fetch-Site": "same-origin",
    },
  )
  response = asyncio.run(params_feature.api_web_consent_session(public_request))
  assert response.status == 403
  assert json.loads(response.text)["error_code"] == "WEB_CONSENT_CLIENT_REJECTED"


@pytest.mark.parametrize("name", [
  params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
  params_feature.COMMUNITY_DATA_SHARING_PARAM,
  params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
])
@pytest.mark.filterwarnings("ignore::aiohttp.web_exceptions.NotAppKeyWarning")
def test_actual_http_consent_get_then_high_risk_enable_post(monkeypatch, name):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: {"default": 0, "min": 0, "max": 1},
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 0})
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda key, value, metadata, **kwargs: writes.append((key, value, kwargs)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)

  async def run():
    app = web.Application()
    web_consent.initialize_web_consent_sessions(app)
    app["params"] = FakeParams(is_offroad=True, is_onroad=False)
    app.router.add_get("/api/web-consent/session", params_feature.api_web_consent_session)
    app.router.add_post("/api/param_set", params_feature.api_param_set)
    host = "192.168.50.95:7000"
    async with TestClient(TestServer(app)) as client:
      session_response = await client.get(
        "/api/web-consent/session",
        headers={
          "Host": host,
          "Referer": f"http://{host}/settings",
          "Sec-Fetch-Site": "same-origin",
        },
      )
      assert session_response.status == 200, await session_response.text()
      session_body = await session_response.json()
      assert session_body["ok"] is True
      assert session_response.headers["Cache-Control"] == "no-store"

      consent_headers = {
        "Host": host,
        "Origin": f"http://{host}",
        "Sec-Fetch-Site": "same-origin",
        web_consent.WEB_REQUEST_MARKER_HEADER: "1",
        web_consent.WEB_CONSENT_TOKEN_HEADER: session_body["token"],
      }
      enabled = await client.post(
        "/api/param_set",
        json={"name": name, "value": 1, "source": "web"},
        headers=consent_headers,
      )
      assert enabled.status == 200, await enabled.text()
      assert (await enabled.json())["ok"] is True

      replay = await client.post(
        "/api/param_set",
        json={"name": name, "value": 1, "source": "web"},
        headers=consent_headers,
      )
      assert replay.status == 403
      assert (await replay.json())["error_code"] == "WEB_CONSENT_PROOF_REQUIRED"

  asyncio.run(run())
  assert writes == [(
    name,
    1,
    {
      "allow_validation_auto_upload_enable": name == params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
      "allow_community_data_sharing_enable": name == params_feature.COMMUNITY_DATA_SHARING_PARAM,
      "allow_third_party_data_sharing_enable": name == params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
    },
  )]


def test_consent_session_store_is_bounded_without_evicting_the_new_token():
  app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(app)
  request = FakeRequest(
    1,
    app["params"],
    app=app,
    headers={
      "Host": "192.168.50.95:7000",
      "Origin": "http://192.168.50.95:7000",
      "X-Carrot-Web-Request": "1",
    },
  )
  newest = ""
  for index in range(web_consent.WEB_CONSENT_SESSION_LIMIT + 8):
    newest = web_consent.issue_web_consent_session(request, now=float(index)) or ""

  sessions = app[web_consent.WEB_CONSENT_SESSIONS_KEY]
  assert len(sessions) == web_consent.WEB_CONSENT_SESSION_LIMIT
  assert newest in sessions
  request.headers[web_consent.WEB_CONSENT_TOKEN_HEADER] = newest
  assert web_consent.consume_web_consent_session(request, now=50.0) is True


@pytest.mark.parametrize("host", [
  "192.168.0.25:7000",
  "10.0.0.5",
  "172.16.4.8:7000",
  "127.0.0.1:7000",
  "[fd00::5]:7000",
  "[fe80::5]:7000",
  "localhost:7000",
])
def test_consent_session_accepts_documented_private_ip_origins(host):
  app = {"params": FakeParams(is_offroad=True, is_onroad=False)}
  web_consent.initialize_web_consent_sessions(app)
  request = FakeRequest(
    1,
    app["params"],
    app=app,
    headers={"Host": host, "Origin": f"http://{host}", "Sec-Fetch-Site": "same-origin"},
  )
  assert web_consent.issue_web_consent_session(request) is not None


@pytest.mark.parametrize("name", [
  params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
  params_feature.COMMUNITY_DATA_SHARING_PARAM,
  params_feature.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_high_risk_disable_remains_available_without_web_consent_proof(monkeypatch, name):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: {"default": 0, "min": 0, "max": 1},
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 1})
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda key, value, metadata, **kwargs: writes.append((key, value)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)
  request = FakeRequest(
    0,
    FakeParams(is_offroad=False, is_onroad=True),
    name=name,
    headers={"Host": "attacker.example", "Content-Type": "application/json"},
  )

  response = asyncio.run(params_feature.api_param_set(request))

  assert response.status == 200
  assert writes == [(name, 0)]


def test_ordinary_setting_write_does_not_require_a_consent_session(monkeypatch):
  name = "OrdinarySetting"
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    name: {"default": 0, "min": 0, "max": 10},
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 0})
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda key, value, metadata, **kwargs: writes.append((key, value)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)
  request = FakeRequest(
    7,
    FakeParams(is_offroad=False, is_onroad=True),
    name=name,
    headers={"Host": "attacker.example", "Content-Type": "application/json"},
  )

  response = asyncio.run(params_feature.api_param_set(request))

  assert response.status == 200
  assert writes == [(name, 7)]


@pytest.mark.parametrize("name", ["IsOffroad", "IsOnroad", "ControlsReady", "DongleId"])
def test_network_param_set_rejects_runtime_and_identity_params(monkeypatch, name):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {}, []))
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda *args, **kwargs: writes.append((args, kwargs)),
  )

  response = asyncio.run(params_feature.api_param_set(FakeRequest(
    1,
    FakeParams(is_offroad=True, is_onroad=False),
    name=name,
  )))

  assert response.status == 403
  assert json.loads(response.text)["ok"] is False
  assert writes == []


@pytest.mark.parametrize("name", sorted(params_feature.WEB_PARAM_SET_EXTRA_ALLOWED))
def test_explicit_non_catalog_device_settings_remain_writable(monkeypatch, name):
  writes = []
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {}, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {name: 0})
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda key, value, metadata, **kwargs: writes.append((key, value, metadata)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)

  response = asyncio.run(params_feature.api_param_set(FakeRequest(
    1,
    FakeParams(is_offroad=True, is_onroad=False),
    name=name,
  )))

  assert response.status == 200
  assert writes == [(name, 1, None)]


def test_privacy_consents_are_excluded_from_json_and_qr_exports(param_runtime):
  param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] = 1
  param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] = 1
  param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] = 1
  param_runtime.values["OrdinarySetting"] = 7

  backup = params_service.get_all_param_values_for_backup()
  assert backup == {"OrdinarySetting": "7"}

  qr = params_service.build_params_qr_payload({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    params_service.COMMUNITY_DATA_SHARING_PARAM: 1,
    params_service.THIRD_PARTY_DATA_SHARING_PARAM: 1,
    "OrdinarySetting": 7,
  })
  parsed = params_service.parse_params_qr_payload(qr["payload"])
  assert qr["count"] == 1
  assert params_service.VALIDATION_AUTO_UPLOAD_PARAM not in parsed
  assert params_service.COMMUNITY_DATA_SHARING_PARAM not in parsed
  assert params_service.THIRD_PARTY_DATA_SHARING_PARAM not in parsed
  assert parsed["OrdinarySetting"] == "7"


def test_stale_backup_download_strips_all_privacy_consents(tmp_path, monkeypatch):
  backup_path = tmp_path / "params_backup.json"
  backup_path.write_text(json.dumps({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    params_service.COMMUNITY_DATA_SHARING_PARAM: 1,
    params_service.THIRD_PARTY_DATA_SHARING_PARAM: 1,
    "OrdinarySetting": 7,
  }), encoding="utf-8")
  monkeypatch.setattr(params_feature, "PARAMS_BACKUP_PATH", str(backup_path))

  response = asyncio.run(params_feature.handle_download_params_backup(None))
  payload = json.loads(response.text)

  assert response.status == 200
  assert payload == {"OrdinarySetting": 7}
  assert response.headers["Content-Disposition"] == "attachment; filename=params_backup.json"


def test_bulk_restore_cannot_enable_but_can_disable_privacy_consents(param_runtime):
  enabled = params_service.restore_param_values_from_backup({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    params_service.COMMUNITY_DATA_SHARING_PARAM: 1,
    params_service.THIRD_PARTY_DATA_SHARING_PARAM: 1,
    "OrdinarySetting": 7,
  })

  assert enabled["ok_cnt"] == 1
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0
  assert param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] == 0
  assert param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] == 0
  assert param_runtime.values["OrdinarySetting"] == 7

  param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] = 1
  param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] = 1
  param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] = 1
  disabled = params_service.restore_param_values_from_backup({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 0,
    params_service.COMMUNITY_DATA_SHARING_PARAM: 0,
    params_service.THIRD_PARTY_DATA_SHARING_PARAM: 0,
  })

  assert disabled["ok_cnt"] == 3
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0
  assert param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] == 0
  assert param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] == 0


class FakeJsonRestoreRequest:
  def __init__(self, body):
    self._body = body

  async def json(self):
    return self._body


class FakeMultipartPart:
  name = "file"

  def __init__(self, values):
    self.values = values

  async def read(self, *, decode):
    return json.dumps(self.values).encode()


class FakeMultipartReader:
  def __init__(self, values):
    self.part = FakeMultipartPart(values)

  async def next(self):
    part, self.part = self.part, None
    return part


class FakeMultipartRestoreRequest:
  def __init__(self, values):
    self.reader = FakeMultipartReader(values)

  async def multipart(self):
    return self.reader


@pytest.mark.parametrize("name", [
  params_service.VALIDATION_AUTO_UPLOAD_PARAM,
  params_service.COMMUNITY_DATA_SHARING_PARAM,
  params_service.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_json_restore_marks_enabled_privacy_consent_skipped(param_runtime, name):
  response = asyncio.run(params_feature.api_params_restore_json(FakeJsonRestoreRequest({
    "values": {
      name: 1,
      "OrdinarySetting": 6,
    },
  })))
  payload = json.loads(response.text)
  consent = next(
    entry for entry in payload["preview"]["entries"]
    if entry["key"] == name
  )

  assert response.status == 200
  assert consent["status"] == "skipped"
  assert consent["apply"] is False
  assert consent["reason"] == "explicit consent required"
  assert param_runtime.values[name] == 0
  assert param_runtime.values["OrdinarySetting"] == 6


@pytest.mark.parametrize("name", [
  params_service.VALIDATION_AUTO_UPLOAD_PARAM,
  params_service.COMMUNITY_DATA_SHARING_PARAM,
  params_service.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_json_restore_can_disable_privacy_consent(param_runtime, name):
  param_runtime.values[name] = 1
  response = asyncio.run(params_feature.api_params_restore_json(FakeJsonRestoreRequest({
    "values": {name: 0},
  })))
  payload = json.loads(response.text)
  consent = payload["preview"]["entries"][0]

  assert response.status == 200
  assert consent["status"] == "changed"
  assert consent["apply"] is True
  assert payload["result"]["ok_cnt"] == 1
  assert param_runtime.values[name] == 0


@pytest.mark.parametrize("name", [
  params_service.VALIDATION_AUTO_UPLOAD_PARAM,
  params_service.COMMUNITY_DATA_SHARING_PARAM,
  params_service.THIRD_PARTY_DATA_SHARING_PARAM,
])
def test_multipart_restore_cannot_enable_privacy_consent(param_runtime, name):
  response = asyncio.run(params_feature.api_params_restore(FakeMultipartRestoreRequest({
    name: 1,
    "OrdinarySetting": 5,
  })))
  payload = json.loads(response.text)

  assert response.status == 200
  assert payload["result"]["ok_cnt"] == 1
  assert param_runtime.values[name] == 0
  assert param_runtime.values["OrdinarySetting"] == 5


def test_service_write_requires_explicit_enable_capability(param_runtime):
  setting = {"default": 0, "min": 0, "max": 1}
  with pytest.raises(PermissionError, match="explicit consent"):
    params_service.set_param_value(params_service.VALIDATION_AUTO_UPLOAD_PARAM, 1, setting)

  params_service.set_param_value(params_service.VALIDATION_AUTO_UPLOAD_PARAM, 0, setting)
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0

  params_service.set_param_value(
    params_service.VALIDATION_AUTO_UPLOAD_PARAM,
    1,
    setting,
    allow_validation_auto_upload_enable=True,
  )
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 1

  with pytest.raises(PermissionError, match="community data sharing"):
    params_service.set_param_value(params_service.COMMUNITY_DATA_SHARING_PARAM, 1, setting)

  params_service.set_param_value(params_service.COMMUNITY_DATA_SHARING_PARAM, 0, setting)
  assert param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] == 0

  params_service.set_param_value(
    params_service.COMMUNITY_DATA_SHARING_PARAM,
    1,
    setting,
    allow_community_data_sharing_enable=True,
  )
  assert param_runtime.values[params_service.COMMUNITY_DATA_SHARING_PARAM] == 1

  with pytest.raises(PermissionError, match="third-party data sharing"):
    params_service.set_param_value(params_service.THIRD_PARTY_DATA_SHARING_PARAM, 1, setting)

  params_service.set_param_value(params_service.THIRD_PARTY_DATA_SHARING_PARAM, 0, setting)
  assert param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] == 0

  params_service.set_param_value(
    params_service.THIRD_PARTY_DATA_SHARING_PARAM,
    1,
    setting,
    allow_third_party_data_sharing_enable=True,
  )
  assert param_runtime.values[params_service.THIRD_PARTY_DATA_SHARING_PARAM] == 1
