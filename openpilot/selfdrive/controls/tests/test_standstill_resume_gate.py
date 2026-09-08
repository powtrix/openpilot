import math
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.controlsd import dk_ka4_stock_scc_resume_gate, standstill_resume_requested
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants


def default_plan_should_stop(speeds, tmp_path):
  params = Params(str(tmp_path))
  action_t = params.get_default_value("LongActuatorDelay") * 0.01 + DT_MDL
  stopping_speed = params.get_default_value("VEgoStopping") * 0.01
  _, should_stop, _, _ = get_accel_from_plan(
    speeds, np.zeros(CONTROL_N), ModelConstants.T_IDXS[:CONTROL_N],
    action_t=action_t, vEgoStopping=stopping_speed,
  )
  return should_stop


@pytest.mark.parametrize("speeds", ([], [0.0], [0.05], [0.1], [math.nan], [math.inf], [-math.inf]))
@pytest.mark.parametrize("should_stop", [False, True])
def test_ka4_standstill_resume_rejects_stationary_or_invalid_horizon(speeds, should_stop):
  assert not standstill_resume_requested(True, True, speeds, should_stop, True)


def test_ka4_restores_legacy_resume_for_slow_departure_plan(tmp_path):
  # Constructed planner input, not a vehicle replay: the planned creep remains
  # below the default 0.5 m/s stopping threshold but departs at up to 0.3 m/s.
  speeds = np.minimum(0.3, np.array(ModelConstants.T_IDXS[:CONTROL_N]) * 0.3)
  should_stop = default_plan_should_stop(speeds, tmp_path)
  assert should_stop
  assert speeds[-1] > 0.1

  dk_scope = dk_ka4_stock_scc_resume_gate("dkcarrot-wip", build_ka4_params())
  assert standstill_resume_requested(True, True, speeds, should_stop, dk_scope)
  # The same slow plan remains subject to shouldStop outside the DK scope.
  assert not standstill_resume_requested(True, True, speeds, should_stop, False)


def test_ka4_stationary_plan_does_not_resume(tmp_path):
  speeds = np.zeros(CONTROL_N)
  should_stop = default_plan_should_stop(speeds, tmp_path)
  assert should_stop
  assert not standstill_resume_requested(True, True, speeds, should_stop, True)


def test_ka4_plan_ending_at_stop_keeps_legacy_no_resume(tmp_path):
  speeds = np.interp(ModelConstants.T_IDXS[:CONTROL_N], [0.0, 0.25, 1.25, 2.5], [0.0, 0.6, 0.6, 0.0])
  should_stop = default_plan_should_stop(speeds, tmp_path)
  assert not should_stop
  assert not standstill_resume_requested(True, True, speeds, should_stop, True)


@pytest.mark.parametrize("should_stop", [False, True])
def test_ka4_departure_uses_legacy_final_speed(should_stop):
  assert standstill_resume_requested(True, True, [0.0, 0.11], should_stop, True)


def test_other_vehicles_keep_existing_should_stop_semantics():
  assert standstill_resume_requested(True, True, [0.0], False, False)
  assert standstill_resume_requested(True, True, [math.inf], False, False)
  assert not standstill_resume_requested(True, True, [], False, False)
  assert not standstill_resume_requested(True, True, [1.0], True, False)


@pytest.mark.parametrize(
  "enabled,standstill",
  ((False, True), (True, False), (False, False)),
)
@pytest.mark.parametrize("use_legacy_ka4_resume", [False, True])
@pytest.mark.parametrize("should_stop", [False, True])
def test_standstill_resume_requires_engaged_standstill(enabled, standstill, use_legacy_ka4_resume, should_stop):
  assert not standstill_resume_requested(enabled, standstill, [1.0], should_stop, use_legacy_ka4_resume)


def build_ka4_params():
  return SimpleNamespace(
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS),
  )


@pytest.mark.parametrize("branch", ["carrot-wip", "carrot", "", None])
def test_comparison_and_recovery_branches_do_not_enable_ka4_departure_gate(branch):
  assert not dk_ka4_stock_scc_resume_gate(branch, build_ka4_params())


@pytest.mark.parametrize("branch", ["dkcarrot-wip", b"dkcarrot-wip"])
def test_exact_dk_branch_and_supported_topology_enable_ka4_departure_gate(branch):
  assert dk_ka4_stock_scc_resume_gate(branch, build_ka4_params())


@pytest.mark.parametrize("missing_flag", [
  HyundaiFlags.CANFD, HyundaiFlags.RADAR_SCC, HyundaiFlags.CANFD_ALT_BUTTONS,
])
def test_ka4_departure_gate_requires_supported_topology_flags(missing_flag):
  CP = build_ka4_params()
  CP.flags &= ~missing_flag
  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)


@pytest.mark.parametrize("excluded_flag", [HyundaiFlags.CANFD_HDA2, HyundaiFlags.CAMERA_SCC])
def test_ka4_departure_gate_excludes_other_scc_topologies(excluded_flag):
  CP = build_ka4_params()
  CP.flags |= excluded_flag
  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)


@pytest.mark.parametrize("mismatch", ["fingerprint", "pcm", "longitudinal"])
def test_ka4_departure_gate_rejects_other_vehicle_contract(mismatch):
  CP = build_ka4_params()
  if mismatch == "fingerprint":
    CP.carFingerprint = CAR.KIA_EV6
  elif mismatch == "pcm":
    CP.pcmCruise = False
  elif mismatch == "longitudinal":
    CP.openpilotLongitudinalControl = True

  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)
