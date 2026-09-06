import math

import pytest
from types import SimpleNamespace

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.controls.controlsd import dk_ka4_stock_scc_resume_gate, standstill_resume_requested


@pytest.mark.parametrize("speeds", ([], [0.0], [0.05], [0.1], [math.nan], [math.inf], [-math.inf]))
def test_ka4_standstill_resume_rejects_stationary_or_invalid_horizon(speeds):
  assert not standstill_resume_requested(True, True, speeds, False, True)


def test_ka4_standstill_resume_requires_both_departure_signals():
  assert standstill_resume_requested(True, True, [0.0, 0.11], False, True)
  assert not standstill_resume_requested(True, True, [0.0, 0.11], True, True)


def test_other_vehicles_keep_existing_should_stop_semantics():
  assert standstill_resume_requested(True, True, [0.0], False, False)
  assert standstill_resume_requested(True, True, [math.inf], False, False)
  assert not standstill_resume_requested(True, True, [], False, False)
  assert not standstill_resume_requested(True, True, [1.0], True, False)


@pytest.mark.parametrize(
  "enabled,standstill",
  ((False, True), (True, False), (False, False)),
)
def test_standstill_resume_requires_engaged_standstill(enabled, standstill):
  assert not standstill_resume_requested(enabled, standstill, [1.0], False, False)


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
def test_exact_dk_branch_and_owner_topology_enable_ka4_departure_gate(branch):
  assert dk_ka4_stock_scc_resume_gate(branch, build_ka4_params())


@pytest.mark.parametrize("missing_flag", [
  HyundaiFlags.CANFD, HyundaiFlags.RADAR_SCC, HyundaiFlags.CANFD_ALT_BUTTONS,
])
def test_ka4_departure_gate_requires_owner_topology_flags(missing_flag):
  CP = build_ka4_params()
  CP.flags &= ~missing_flag
  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)


@pytest.mark.parametrize("excluded_flag", [HyundaiFlags.CANFD_HDA2, HyundaiFlags.CAMERA_SCC])
def test_ka4_departure_gate_excludes_other_scc_topologies(excluded_flag):
  CP = build_ka4_params()
  CP.flags |= excluded_flag
  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)


@pytest.mark.parametrize("mismatch", ["fingerprint", "pcm", "longitudinal"])
def test_ka4_departure_gate_rejects_non_owner_vehicle_contract(mismatch):
  CP = build_ka4_params()
  if mismatch == "fingerprint":
    CP.carFingerprint = CAR.KIA_EV6
  elif mismatch == "pcm":
    CP.pcmCruise = False
  elif mismatch == "longitudinal":
    CP.openpilotLongitudinalControl = True

  assert not dk_ka4_stock_scc_resume_gate("dkcarrot-wip", CP)
