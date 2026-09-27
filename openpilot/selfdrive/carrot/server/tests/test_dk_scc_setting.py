from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.server.features import params as api
from openpilot.selfdrive.carrot.server.services import params as service
from openpilot.selfdrive.carrot.server.services.dk_scc_setting import (
  DK_EXPERIMENTAL_SCC_PARAM as KEY,
  require_scc_setting_offroad,
  require_scc_setting_scope,
  scc_setting_value,
)
from openpilot.cereal import car
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_scope_supported
from openpilot.selfdrive.carrot.server.services import setting_profiles
from openpilot.selfdrive.carrot.server.services.settings import build_menu_categories, group_index


ROOT = Path(__file__).resolve().parents[4]
SETTING = {"name": KEY, "min": 0, "max": 1, "default": 1, "control": "toggle"}


def stock_cp():
  cp = car.CarParams.new_message()
  cp.carFingerprint = str(CAR.KIA_CARNIVAL_4TH_GEN)
  cp.pcmCruise = True
  cp.openpilotLongitudinalControl = False
  cp.flags = int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS)
  cp.safetyConfigs = [{"safetyModel": "hyundaiCanfd", "safetyParam": int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)}]
  return cp


class Runtime:
  def __init__(self, offroad=True, onroad=False):
    self.values = {"IsOffroad": offroad, "IsOnroad": onroad, KEY: 1,
                   "GitBranch": b"dkcarrot-wip", "CarParamsPersistent": stock_cp().to_bytes()}

  def get_bool(self, name):
    return bool(self.values[name])

  def get(self, name):
    return self.values.get(name)


class Request:
  def __init__(self, runtime, value):
    self.app = {"params": runtime}
    self.value = value

  async def json(self):
    return {"name": KEY, "value": self.value}


@pytest.mark.parametrize("value, expected", [(0, 0), (1, 1), (False, 0), (True, 1), ("0", 0), ("1", 1)])
def test_value_accepts_only_explicit_binary_selection(value, expected):
  assert scc_setting_value(value) == expected


@pytest.mark.parametrize("value", [None, "", "ON", "false", "NaN", "inf", float("nan"), -1, 2, 0.3, [], {}])
def test_invalid_values_do_not_silently_enable(value):
  with pytest.raises(ValueError):
    scc_setting_value(value)


@pytest.mark.parametrize("offroad,onroad", [(False, False), (False, True), (True, True)])
def test_offroad_proof_is_fail_closed(offroad, onroad):
  with pytest.raises(PermissionError):
    require_scc_setting_offroad(Runtime(offroad, onroad))


def test_missing_or_unreadable_runtime_fails_closed():
  for runtime in (None, object()):
    with pytest.raises(PermissionError):
      require_scc_setting_offroad(runtime)


@pytest.mark.parametrize("offroad,onroad", [("1", False), (True, None), (1, 0), (True, "0")])
def test_malformed_runtime_flags_do_not_count_as_offroad_proof(offroad, onroad):
  class MalformedRuntime:
    def get_bool(self, name):
      return {"IsOffroad": offroad, "IsOnroad": onroad}[name]

  with pytest.raises(PermissionError):
    require_scc_setting_offroad(MalformedRuntime())


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


@pytest.mark.parametrize("value", [0, 1])
def test_backup_and_profile_restore_cannot_change_experiment(monkeypatch, value):
  assert KEY in service.BACKUP_EXCLUDED_PARAMS
  assert service.filter_param_values_for_backup({KEY: value, "LatSmoothSec": 13}) == {"LatSmoothSec": 13}
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "ParamKeyType", object())
  monkeypatch.setattr(service, "Params", Runtime)
  monkeypatch.setattr(service, "_catalog_definitions", lambda: {KEY: SETTING})
  assert setting_profiles._clean_values({KEY: value, "LatSmoothSec": 13}) == {"LatSmoothSec": 13}
  result = service.restore_param_values_from_backup({KEY: value}, source="profile")
  assert result == {"ok_cnt": 0, "fail_cnt": 0, "fails": []}


@pytest.mark.parametrize("key", [KEY, "DkExperimentalSteering"])
@pytest.mark.parametrize("value", [0, 1])
def test_restore_preview_and_old_native_registry_cannot_reintroduce_selections(monkeypatch, key, value):
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "ParamKeyType", object())
  monkeypatch.setattr(service, "Params", Runtime)
  monkeypatch.setattr(service, "_catalog_definitions", lambda: {KEY: SETTING})
  monkeypatch.setattr(service, "get_param_values", lambda *_: {key: 1 - value})
  # Even a stale Params registry claiming to know a removed setting may not
  # restore it. The excluded entry must never reach type resolution/writing.
  def unexpected(*args, **kwargs):
    raise AssertionError("excluded setting reached the native registry")
  monkeypatch.setattr(service, "resolve_param_type", unexpected)
  monkeypatch.setattr(service, "set_param_value", unexpected)
  result = service.restore_param_values_validated({key: value}, source="profile")
  assert result["preview"]["entries"][0]["status"] == "skipped"
  assert result["result"] == {"ok_cnt": 0, "fail_cnt": 0, "fails": []}
  assert service.filter_param_values_for_backup({key: value, "LatSmoothSec": 13}) == {"LatSmoothSec": 13}
  assert setting_profiles._clean_values({key: value, "LatSmoothSec": 13}) == {"LatSmoothSec": 13}


def test_retired_steering_setting_is_not_writable_through_api(monkeypatch):
  monkeypatch.setattr(api, "get_settings_cached", lambda: ({}, {}, {KEY: SETTING}, []))
  class RetiredRequest(Request):
    async def json(self):
      return {"name": "DkExperimentalSteering", "value": 1}
  response = asyncio.run(api.api_param_set(RetiredRequest(Runtime(), 1)))
  assert response.status == 403


@pytest.mark.parametrize("branch", ["dkcarrot-wip", b"dkcarrot-wip"])
def test_shared_scope_accepts_exact_stock_topology(branch):
  assert dk_scc_scope_supported(stock_cp(), branch)


@pytest.mark.parametrize("branch", ["carrot-wip", "dkcarrot-wip-other", b"dkcarrot-wip\xff", None, ""])
def test_shared_scope_rejects_other_or_unreadable_branch(branch):
  assert not dk_scc_scope_supported(stock_cp(), branch)


@pytest.mark.parametrize("attribute,value", [
  ("carFingerprint", "KIA_SORENTO"), ("pcmCruise", False), ("openpilotLongitudinalControl", True),
  ("passive", True), ("dashcamOnly", True),
])
def test_shared_scope_rejects_non_owner_control_configuration(attribute, value):
  cp = stock_cp()
  setattr(cp, attribute, value)
  assert not dk_scc_scope_supported(cp, "dkcarrot-wip")


@pytest.mark.parametrize("flag", [HyundaiFlags.CANFD, HyundaiFlags.RADAR_SCC, HyundaiFlags.CANFD_ALT_BUTTONS,
                                  HyundaiFlags.CANFD_HDA2, HyundaiFlags.CAMERA_SCC])
def test_shared_scope_rejects_changed_bus_topology(flag):
  cp = stock_cp()
  cp.flags ^= int(flag)
  assert not dk_scc_scope_supported(cp, "dkcarrot-wip")


@pytest.mark.parametrize("flag", [HyundaiSafetyFlags.CANFD_ALT_BUTTONS, HyundaiSafetyFlags.LONG,
                                  HyundaiSafetyFlags.CAMERA_SCC, HyundaiSafetyFlags.CANFD_LKA_STEERING,
                                  HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT])
def test_shared_scope_rejects_safety_configuration_mismatch(flag):
  cp = stock_cp()
  cp.safetyConfigs[0].safetyParam ^= int(flag)
  assert not dk_scc_scope_supported(cp, "dkcarrot-wip")


def test_shared_scope_rejects_missing_or_ambiguous_active_safety():
  for configs in ([], [{"safetyModel": "noOutput"}],
                  [{"safetyModel": "hyundaiCanfd", "safetyParam": 32}] * 2,
                  [{"safetyModel": "hyundai", "safetyParam": 32}]):
    cp = stock_cp()
    cp.safetyConfigs = configs
    assert not dk_scc_scope_supported(cp, "dkcarrot-wip")
  assert not dk_scc_scope_supported(None, "dkcarrot-wip")


def test_no_output_panda_offset_does_not_change_scope():
  cp = stock_cp()
  cp.safetyConfigs = [{"safetyModel": "noOutput"}, {"safetyModel": "hyundaiCanfd", "safetyParam": 32}]
  assert dk_scc_scope_supported(cp, "dkcarrot-wip")


@pytest.mark.parametrize("key,value", [("GitBranch", b"carrot-wip"), ("CarParamsPersistent", None),
                                      ("CarParamsPersistent", b"invalid"), ("CarParams", b"invalid")])
def test_scope_fails_closed_for_unverified_branch_or_vehicle(key, value):
  runtime = Runtime()
  runtime.values[key] = value
  with pytest.raises(PermissionError):
    require_scc_setting_scope(runtime)


@pytest.mark.parametrize("value", [0, 1])
def test_api_and_shared_write_block_unsupported_vehicle(monkeypatch, value):
  runtime = Runtime()
  runtime.values["GitBranch"] = b"carrot-wip"
  writes = []
  monkeypatch.setattr(api, "get_settings_cached", lambda: ({}, {}, {KEY: SETTING}, []))
  monkeypatch.setattr(api, "set_param_value", lambda *a, **k: writes.append(a))
  response = asyncio.run(api.api_param_set(Request(runtime, value)))
  assert response.status == 409
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "Params", lambda: runtime)
  monkeypatch.setattr(service, "put_typed", lambda *a: writes.append(a))
  with pytest.raises(PermissionError):
    service.set_param_value(KEY, value, SETTING)
  assert writes == []


def test_catalog_menu_default_and_registry_are_consistent():
  settings = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text())
  groups, by_name, _ = group_index(settings)
  setting = by_name[KEY]
  assert (setting["min"], setting["max"], setting["default"]) == (0, 1, 1)
  assert setting["control"] == "toggle"
  assert setting["risk"] == "high"
  assert setting["title"] == "SCC 실험"
  assert "DkExperimentalSteering" not in by_name
  for field in ("descr", "edescr", "cdescr"):
    assert "dkcarrot-wip" in setting[field]
    assert "98abba" in setting[field]
  assert "재시작" in setting["descr"]
  assert "restart" in setting["edescr"]
  categories = build_menu_categories(settings, by_name)
  driving = next(c for c in categories if c["id"] == "DRIVING")
  start = next(g for g in driving["groups"] if g["id"] == "START_AUTO")
  section = next(s for s in start["sections"] if s["id"] == "SCC_EXPERIMENTAL")
  assert section["items"] == [KEY]
  steering = next(g for g in driving["groups"] if g["id"] == "STEER")
  assert not any(s["id"] == "STEER_EXPERIMENTAL" for s in steering["sections"])
  registry = (ROOT / "common/params_keys.h").read_text()
  assert f'{{"{KEY}", {{PERSISTENT, BOOL, "1"}}}}' in registry
  assert "DkExperimentalSteering" not in registry
