from __future__ import annotations

import asyncio
import json

import pytest

from openpilot.selfdrive.carrot.server.features import params as params_feature
from openpilot.selfdrive.carrot.server.services import params as params_service


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
  def __init__(self, value, params, name=None, *, content_type="application/json", headers=None):
    self._body = {
      "name": name or params_feature.VALIDATION_AUTO_UPLOAD_PARAM,
      "value": value,
      "source": "test",
    }
    self.app = {"params": params}
    self.content_type = content_type
    self.headers = {"X-Carrot-Web-Request": "1"} if headers is None else headers

  async def json(self):
    return self._body


class FakeRuntimeParams:
  def __init__(self):
    self.values = {
      params_service.VALIDATION_AUTO_UPLOAD_PARAM: 0,
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
    "OrdinarySetting": {"default": 0, "min": 0, "max": 10},
  }
  monkeypatch.setattr(params_service, "HAS_PARAMS", True)
  monkeypatch.setattr(params_service, "ParamKeyType", FakeParamKeyType)
  monkeypatch.setattr(params_service, "Params", lambda: runtime)
  monkeypatch.setattr(params_service, "_catalog_definitions", lambda: definitions)
  monkeypatch.setattr(params_feature, "HAS_PARAMS", True)
  monkeypatch.setattr(params_feature, "ParamKeyType", FakeParamKeyType)
  return runtime


def call_param_set(monkeypatch, *, previous, value, params):
  writes = []
  setting = {"default": 0, "min": 0, "max": 1}
  monkeypatch.setattr(params_feature, "get_settings_cached", lambda: ({}, {}, {
    params_feature.VALIDATION_AUTO_UPLOAD_PARAM: setting,
  }, []))
  monkeypatch.setattr(params_feature, "get_param_values", lambda *args, **kwargs: {
    params_feature.VALIDATION_AUTO_UPLOAD_PARAM: previous,
  })
  monkeypatch.setattr(
    params_feature,
    "set_param_value",
    lambda name, new_value, metadata, **kwargs: writes.append((name, new_value)),
  )
  monkeypatch.setattr(params_feature, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(params_feature, "is_drive_engaged", lambda request: False)

  response = asyncio.run(params_feature.api_param_set(FakeRequest(value, params)))
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


def test_validation_consent_is_excluded_from_json_and_qr_exports(param_runtime):
  param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] = 1
  param_runtime.values["OrdinarySetting"] = 7

  backup = params_service.get_all_param_values_for_backup()
  assert backup == {"OrdinarySetting": "7"}

  qr = params_service.build_params_qr_payload({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "OrdinarySetting": 7,
  })
  parsed = params_service.parse_params_qr_payload(qr["payload"])
  assert qr["count"] == 1
  assert params_service.VALIDATION_AUTO_UPLOAD_PARAM not in parsed
  assert parsed["OrdinarySetting"] == "7"


def test_stale_backup_download_strips_validation_consent(tmp_path, monkeypatch):
  backup_path = tmp_path / "params_backup.json"
  backup_path.write_text(json.dumps({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "OrdinarySetting": 7,
  }), encoding="utf-8")
  monkeypatch.setattr(params_feature, "PARAMS_BACKUP_PATH", str(backup_path))

  response = asyncio.run(params_feature.handle_download_params_backup(None))
  payload = json.loads(response.text)

  assert response.status == 200
  assert payload == {"OrdinarySetting": 7}
  assert response.headers["Content-Disposition"] == "attachment; filename=params_backup.json"


def test_bulk_restore_cannot_enable_but_can_disable_validation_consent(param_runtime):
  enabled = params_service.restore_param_values_from_backup({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "OrdinarySetting": 7,
  })

  assert enabled["ok_cnt"] == 1
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0
  assert param_runtime.values["OrdinarySetting"] == 7

  param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] = 1
  disabled = params_service.restore_param_values_from_backup({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 0,
  })

  assert disabled["ok_cnt"] == 1
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0


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


def test_json_restore_marks_enabled_validation_consent_skipped(param_runtime):
  response = asyncio.run(params_feature.api_params_restore_json(FakeJsonRestoreRequest({
    "values": {
      params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
      "OrdinarySetting": 6,
    },
  })))
  payload = json.loads(response.text)
  consent = next(
    entry for entry in payload["preview"]["entries"]
    if entry["key"] == params_service.VALIDATION_AUTO_UPLOAD_PARAM
  )

  assert response.status == 200
  assert consent["status"] == "skipped"
  assert consent["apply"] is False
  assert consent["reason"] == "explicit consent required"
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0
  assert param_runtime.values["OrdinarySetting"] == 6


def test_json_restore_can_disable_validation_consent(param_runtime):
  param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] = 1
  response = asyncio.run(params_feature.api_params_restore_json(FakeJsonRestoreRequest({
    "values": {params_service.VALIDATION_AUTO_UPLOAD_PARAM: 0},
  })))
  payload = json.loads(response.text)
  consent = payload["preview"]["entries"][0]

  assert response.status == 200
  assert consent["status"] == "changed"
  assert consent["apply"] is True
  assert payload["result"]["ok_cnt"] == 1
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0


def test_multipart_restore_cannot_enable_validation_consent(param_runtime):
  response = asyncio.run(params_feature.api_params_restore(FakeMultipartRestoreRequest({
    params_service.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "OrdinarySetting": 5,
  })))
  payload = json.loads(response.text)

  assert response.status == 200
  assert payload["result"]["ok_cnt"] == 1
  assert param_runtime.values[params_service.VALIDATION_AUTO_UPLOAD_PARAM] == 0
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
