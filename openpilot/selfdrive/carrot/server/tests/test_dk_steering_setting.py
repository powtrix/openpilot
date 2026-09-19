from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.server.features import params as api
from openpilot.selfdrive.carrot.server.services import params as service
from openpilot.selfdrive.carrot.server.services.dk_steering_setting import (
  DK_EXPERIMENTAL_STEERING_PARAM as KEY,
  require_steering_setting_offroad,
  steering_setting_value,
)
from openpilot.selfdrive.carrot.server.services.settings import build_menu_categories, group_index


ROOT = Path(__file__).resolve().parents[4]
SETTING = {"name": KEY, "min": 0, "max": 1, "default": 0, "control": "toggle"}


class Runtime:
  def __init__(self, offroad=True, onroad=False):
    self.values = {"IsOffroad": offroad, "IsOnroad": onroad, KEY: 0}

  def get_bool(self, name):
    return bool(self.values[name])


class Request:
  def __init__(self, runtime, value):
    self.app = {"params": runtime}
    self.value = value

  async def json(self):
    return {"name": KEY, "value": self.value}


@pytest.mark.parametrize("value, expected", [(0, 0), (1, 1), (False, 0), (True, 1), ("0", 0), ("1", 1)])
def test_value_accepts_only_explicit_binary_selection(value, expected):
  assert steering_setting_value(value) == expected


@pytest.mark.parametrize("value", [None, "", "ON", "false", "NaN", "inf", float("nan"), -1, 2, 0.3, [], {}])
def test_invalid_values_do_not_silently_enable(value):
  with pytest.raises(ValueError):
    steering_setting_value(value)


@pytest.mark.parametrize("offroad,onroad", [(False, False), (False, True), (True, True)])
def test_offroad_proof_is_fail_closed(offroad, onroad):
  with pytest.raises(PermissionError):
    require_steering_setting_offroad(Runtime(offroad, onroad))


def test_missing_or_unreadable_runtime_fails_closed():
  for runtime in (None, object()):
    with pytest.raises(PermissionError):
      require_steering_setting_offroad(runtime)


@pytest.mark.parametrize("offroad,onroad", [("1", False), (True, None), (1, 0), (True, "0")])
def test_malformed_runtime_flags_do_not_count_as_offroad_proof(offroad, onroad):
  class MalformedRuntime:
    def get_bool(self, name):
      return {"IsOffroad": offroad, "IsOnroad": onroad}[name]

  with pytest.raises(PermissionError):
    require_steering_setting_offroad(MalformedRuntime())


@pytest.mark.parametrize("value", [0, 1])
@pytest.mark.parametrize("offroad,onroad", [(False, False), (False, True), (True, True), (True, False)])
def test_api_both_edges_require_offroad_and_report_restart(monkeypatch, value, offroad, onroad):
  writes = []
  runtime = Runtime(offroad, onroad)
  monkeypatch.setattr(api, "get_settings_cached", lambda: ({}, {}, {KEY: SETTING}, []))
  monkeypatch.setattr(api, "get_param_values", lambda *_: {KEY: 1 - value})
  monkeypatch.setattr(api, "set_param_value", lambda *args, **kwargs: writes.append(args))
  monkeypatch.setattr(api, "append_param_change", lambda *args, **kwargs: None)
  monkeypatch.setattr(api, "is_drive_engaged", lambda *_: False)
  response = asyncio.run(api.api_param_set(Request(runtime, value)))
  allowed = offroad and not onroad
  assert response.status == (200 if allowed else 409)
  assert bool(writes) == allowed
  if allowed:
    payload = json.loads(response.text)
    assert payload["applies_at"] == "controls_start"
    assert payload["restart_recommended"] is True


@pytest.mark.parametrize("value", [2, -1, 0.5, "garbage", "NaN"])
def test_api_rejects_invalid_before_numeric_clamping(monkeypatch, value):
  monkeypatch.setattr(api, "get_settings_cached", lambda: ({}, {}, {KEY: SETTING}, []))
  response = asyncio.run(api.api_param_set(Request(Runtime(), value)))
  assert response.status == 400


@pytest.mark.parametrize("value", [0, 1])
def test_common_write_path_blocks_tool_or_restore_bypass(monkeypatch, value):
  runtime = Runtime(False, True)
  writes = []
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "Params", lambda: runtime)
  monkeypatch.setattr(service, "put_typed", lambda *args: writes.append(args))
  with pytest.raises(PermissionError):
    service.set_param_value(KEY, value, SETTING)
  assert writes == []


@pytest.mark.parametrize("value", [0, 1])
def test_common_write_path_allows_parked_selection(monkeypatch, value):
  runtime = Runtime()
  writes = []
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "Params", lambda: runtime)
  monkeypatch.setattr(service, "put_typed", lambda *args: writes.append(args))
  service.set_param_value(KEY, value, SETTING)
  assert writes == [(runtime, KEY, value, SETTING)]


def test_no_params_fallback_cannot_queue_experiment(monkeypatch):
  monkeypatch.setattr(service, "HAS_PARAMS", False)
  with pytest.raises(PermissionError):
    service.set_param_value(KEY, 1, SETTING)


def test_backup_and_profile_restore_cannot_enable_experiment(monkeypatch):
  assert KEY in service.BACKUP_EXCLUDED_PARAMS
  assert service.filter_param_values_for_backup({KEY: 1, "LatSmoothSec": 13}) == {"LatSmoothSec": 13}
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "ParamKeyType", object())
  monkeypatch.setattr(service, "Params", Runtime)
  monkeypatch.setattr(service, "_catalog_definitions", lambda: {KEY: SETTING})
  result = service.restore_param_values_from_backup({KEY: 1}, source="profile")
  assert result == {"ok_cnt": 0, "fail_cnt": 0, "fails": []}


def test_catalog_menu_default_and_registry_are_consistent():
  settings = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text())
  groups, by_name, _ = group_index(settings)
  setting = by_name[KEY]
  assert (setting["min"], setting["max"], setting["default"]) == (0, 1, 0)
  assert setting["control"] == "toggle"
  assert setting["risk"] == "high"
  assert setting["title"] == "실험용 조향개선"
  for field in ("descr", "edescr", "cdescr"):
    assert "dkcarrot-wip" in setting[field]
  assert "재시작" in setting["descr"]
  assert "restart" in setting["edescr"]
  assert "모델 모드" in setting["descr"] and "14~60km/h" in setting["descr"]
  assert "레인모드·차선변경·유효하지 않은 입력" in setting["descr"]
  assert "model mode" in setting["edescr"] and "14–60 km/h" in setting["edescr"]
  assert "模型模式" in setting["cdescr"] and "14–60 km/h" in setting["cdescr"]
  categories = build_menu_categories(settings, by_name)
  driving = next(c for c in categories if c["id"] == "DRIVING")
  steering = next(g for g in driving["groups"] if g["id"] == "STEER")
  section = next(s for s in steering["sections"] if s["id"] == "STEER_EXPERIMENTAL")
  assert section["items"] == [KEY]
  assert f'{{"{KEY}", {{PERSISTENT, BOOL, "0"}}}}' in (ROOT / "common/params_keys.h").read_text()
