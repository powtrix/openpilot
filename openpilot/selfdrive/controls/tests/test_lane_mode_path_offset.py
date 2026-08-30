from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.drive_helpers import CAR_ROTATION_RADIUS, CONTROL_N, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.lateral_planner import apply_static_path_offset, publish_offset_evidence


V_EGO = 20.0
PATH_OFFSET_M = 0.10
STEER_ACTUATOR_DELAY = 0.10


def run_straight_path_mpc(path_offset_m):
  mpc = LateralMpc()
  mpc.set_weights(1.0, 0.1, 0.0, 0.05, 800.0)

  x0 = np.zeros(4)
  params = np.column_stack([
    np.full(LAT_MPC_N + 1, V_EGO),
    np.full(LAT_MPC_N + 1, CAR_ROTATION_RADIUS),
  ])
  straight_path_y = np.zeros(LAT_MPC_N + 1)
  y_pts = straight_path_y + path_offset_m
  heading_pts = np.zeros(LAT_MPC_N + 1)
  yaw_rate_pts = np.zeros(LAT_MPC_N + 1)

  for _ in range(10):
    mpc.run(x0, params, y_pts, heading_pts, yaw_rate_pts)

  assert mpc.solution_status == 0
  return mpc.x_sol


class FakeParams:
  def get_bool(self, key):
    return False

  def get_int(self, key):
    return {
      "AlwaysLateral": 0,
      "UseLaneLineCurveSpeed": 80,
    }.get(key, 0)

  def get_float(self, key):
    return {
      "CustomSR": 0.0,
      "LatSmoothSec": 0.0,
      "SteerActuatorDelay": STEER_ACTUATOR_DELAY * 100.0,
      "SteerRatioRate": 100.0,
    }.get(key, 0.0)


class FakeVehicleModel:
  def update_params(self, stiffness_factor, steer_ratio):
    pass

  def calc_curvature(self, steer_angle, v_ego, roll):
    return 0.0


class FakeCarInterface:
  def get_pid_accel_limits(self, CP, v_ego, v_cruise):
    return -3.0, 2.0


class FakeLongControl:
  long_control_state = car.CarControl.Actuators.LongControlState.off
  pid = SimpleNamespace(p=0.0, i=0.0, f=0.0)

  def reset(self):
    pass

  def update(self, *args):
    return 0.0, 0.0, 0.0


class FakeLatControl:
  def reset(self):
    pass

  def update(self, *args, **kwargs):
    return 0.0, 0.0, SimpleNamespace()


class FakeCarrotControls:
  def lat_suspend_control(self, CS, lat_active):
    return lat_active


class FakeSubMaster(dict):
  frame = 1
  recv_frame = {"longitudinalPlan": 1}


def build_controls(lateral_plan, raw_model_curvature):
  CP = car.CarParams.new_message()
  CP.minSteerSpeed = 0.0
  CP.openpilotLongitudinalControl = False

  CS = car.CarState.new_message()
  CS.gearShifter = car.CarState.GearShifter.drive
  CS.vEgo = V_EGO
  CS.vCruise = 100.0
  CS.latEnabled = True
  CS.standstill = False

  model = log.ModelDataV2.new_message()
  model.action.desiredCurvature = raw_model_curvature
  model.meta.laneChangeState = log.LaneChangeState.off
  model.meta.laneChangeDirection = log.LaneChangeDirection.none

  sm = FakeSubMaster({
    "carState": CS,
    "carrotMan": SimpleNamespace(vTurnSpeed=0),
    "lateralPlan": lateral_plan,
    "liveDelay": SimpleNamespace(lateralDelay=STEER_ACTUATOR_DELAY),
    "liveParameters": SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    "liveTorqueParameters": SimpleNamespace(useParams=False),
    "longitudinalPlan": SimpleNamespace(),
    "modelV2": model,
    "onroadEvents": [],
    "radarState": SimpleNamespace(),
    "selfdriveState": SimpleNamespace(enabled=True, active=True),
  })

  controls = Controls.__new__(Controls)
  controls.CI = FakeCarInterface()
  controls.CP = CP
  controls.LaC = FakeLatControl()
  controls.LoC = FakeLongControl()
  controls.VM = FakeVehicleModel()
  controls.carrot_controls = FakeCarrotControls()
  controls.curvature = 0.0
  controls.desired_curvature = 0.0
  controls.is_vw_meb = False
  controls.params = FakeParams()
  controls.sm = sm
  controls.steer_limited_by_safety = False
  return controls


def test_straight_path_offset_mpc_plan_reaches_controlsd():
  centered_solution = run_straight_path_mpc(0.0)
  offset_solution = run_straight_path_mpc(PATH_OFFSET_M)

  centered_curvatures = centered_solution[:CONTROL_N, 3] / V_EGO
  offset_curvatures = offset_solution[:CONTROL_N, 3] / V_EGO
  assert centered_curvatures == pytest.approx(0.0, abs=1e-9)
  early_offset_curvatures = offset_curvatures[1:CONTROL_N // 2]
  strongest_early_curvature = early_offset_curvatures[np.argmax(np.abs(early_offset_curvatures))]
  assert abs(strongest_early_curvature) > 1e-6
  assert strongest_early_curvature > 0.0

  lateral_plan = SimpleNamespace(
    useLaneLines=True,
    psis=offset_solution[:CONTROL_N, 2].tolist(),
    curvatures=offset_curvatures.tolist(),
    distances=offset_solution[:CONTROL_N, 0].tolist(),
  )
  raw_model_curvature = -0.001
  controls = build_controls(lateral_plan, raw_model_curvature)
  expected_plan_curvature = get_lag_adjusted_curvature(
    controls.CP, V_EGO, lateral_plan.psis, lateral_plan.curvatures,
    STEER_ACTUATOR_DELAY, lateral_plan.distances,
  )

  CC, _ = controls.state_control()

  assert controls.lanefull_mode_enabled is True
  assert expected_plan_curvature > 0.0
  assert controls.desired_curvature == pytest.approx(expected_plan_curvature)
  assert CC.actuators.curvature == pytest.approx(expected_plan_curvature)
  assert np.sign(controls.desired_curvature) != np.sign(raw_model_curvature)


def test_static_and_dynamic_offsets_are_logged_as_numeric_evidence():
  path_xyz = np.zeros((LAT_MPC_N + 1, 3))
  path_xyz[:, 1] = np.linspace(-0.02, 0.02, LAT_MPC_N + 1)
  expected_before = path_xyz[:, 1].copy()

  before = apply_static_path_offset(path_xyz, PATH_OFFSET_M)

  assert before == pytest.approx(expected_before)
  assert path_xyz[:, 1] - before == pytest.approx(PATH_OFFSET_M)

  plan = log.LateralPlan.new_message()
  publish_offset_evidence(plan, PATH_OFFSET_M, -0.03, before)
  assert plan.staticPathOffset == pytest.approx(PATH_OFFSET_M)
  assert plan.dynamicLaneOffset == pytest.approx(-0.03)
  assert list(plan.pathBeforeStaticOffset) == pytest.approx(expected_before)
