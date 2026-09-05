from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.drive_helpers import CAR_ROTATION_RADIUS, CONTROL_N, clip_curvature, get_lag_adjusted_curvature
import openpilot.selfdrive.controls.lib.lane_planner_2 as lane_planner_module
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N as LAT_MPC_N
import openpilot.selfdrive.controls.lib.lateral_planner as lateral_planner_module
from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner, apply_static_path_offset, publish_offset_evidence


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


class PlannerParams:
  def __init__(self, path_offset_cm=10, adjust_lane_offset_cm=0):
    self.path_offset_cm = path_offset_cm
    self.adjust_lane_offset_cm = adjust_lane_offset_cm

  def get_int(self, key):
    return {
      "AdjustLaneOffset": self.adjust_lane_offset_cm,
      "PathOffset": self.path_offset_cm,
      "UseLaneLineSpeed": 0,
    }.get(key, 0)

  def get_float(self, key):
    return {
      "LatMpcAccelCost": 120.0,
      "LatMpcInputOffset": 4.0,
      "LatMpcJerkCost": 4.0,
      "LatMpcMotionCost": 7.0,
      "LatMpcPathCost": 200.0,
      "LatMpcSteeringRateCost": 7.0,
    }.get(key, 0.0)


class PlannerSubMaster(dict):
  def __init__(self, values, model_mono_time):
    super().__init__(values)
    self.logMonoTime = {"modelV2": model_mono_time}

  def all_checks(self, service_list=None):
    return True


class CapturePubMaster:
  def send(self, service, message):
    assert service == "lateralPlan"
    self.message = message


class FakeVehicleModel:
  def update_params(self, stiffness_factor, steer_ratio):
    pass

  def calc_curvature(self, steer_angle, v_ego, roll):
    return 0.0

  def roll_compensation(self, roll, v_ego):
    return 0.0


class FakeMebCurvaturePid:
  def reset(self):
    pass

  def update(self, *args, **kwargs):
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
  def __init__(self, values, *, lateral_plan_checks=True, model_mono_time=1_000_000_000):
    super().__init__(values)
    self.frame = 1
    self.recv_frame = {"longitudinalPlan": 1}
    self.logMonoTime = {"modelV2": model_mono_time}
    self.lateral_plan_checks = lateral_plan_checks

  def all_checks(self, service_list=None):
    if service_list == ['lateralPlan']:
      return self.lateral_plan_checks
    return True


def build_controls(lateral_plan, raw_model_curvature, *, lateral_plan_checks=True,
                   model_mono_time=1_000_000_000, is_vw_meb=False):
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
  }, lateral_plan_checks=lateral_plan_checks, model_mono_time=model_mono_time)

  controls = Controls.__new__(Controls)
  controls.CI = FakeCarInterface()
  controls.CP = CP
  controls.LaC = FakeLatControl()
  controls.LoC = FakeLongControl()
  controls.VM = FakeVehicleModel()
  controls.carrot_controls = FakeCarrotControls()
  controls.curvature = 0.0
  controls.desired_curvature = 0.0
  controls.is_vw_meb = is_vw_meb
  controls.meb_curvature_pid = FakeMebCurvaturePid() if is_vw_meb else None
  controls.calibrated_pose = None
  controls.params = FakeParams()
  controls.sm = sm
  controls.steer_limited_by_safety = False
  return controls


def build_planner_model(raw_path_y, *, valid_path=True, lane_center_y=0.0, lane_mode=False,
                        desire=log.Desire.none, lane_change_state=log.LaneChangeState.off,
                        lane_change_direction=log.LaneChangeDirection.none,
                        lane_width_left=0.0, lane_width_right=0.0):
  trajectory_x = np.linspace(0.0, 100.0, LAT_MPC_N + 1)
  trajectory_t = np.linspace(0.0, 3.2, LAT_MPC_N + 1)
  path_x = trajectory_x if valid_path else trajectory_x[:-1]

  def line(y):
    return SimpleNamespace(
      t=trajectory_t.tolist(),
      x=trajectory_x.tolist(),
      y=np.full(LAT_MPC_N + 1, y).tolist(),
    )

  return SimpleNamespace(
    position=SimpleNamespace(
      x=path_x.tolist(),
      y=raw_path_y.tolist(),
      z=np.zeros(LAT_MPC_N + 1).tolist(),
      t=trajectory_t.tolist(),
    ),
    orientation=SimpleNamespace(
      x=np.zeros(LAT_MPC_N + 1).tolist(),
      z=np.zeros(LAT_MPC_N + 1).tolist(),
    ),
    orientationRate=SimpleNamespace(z=np.zeros(LAT_MPC_N + 1).tolist()),
    velocity=SimpleNamespace(
      x=np.full(LAT_MPC_N + 1, V_EGO).tolist(),
      y=np.zeros(LAT_MPC_N + 1).tolist(),
      z=np.zeros(LAT_MPC_N + 1).tolist(),
    ),
    acceleration=SimpleNamespace(x=np.zeros(LAT_MPC_N + 1).tolist()),
    laneLines=[
      line(lane_center_y - 3.5),
      line(lane_center_y - 1.75),
      line(lane_center_y + 1.75),
      line(lane_center_y + 3.5),
    ],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0] if lane_mode else [0.0, 0.0, 0.0, 0.0],
    laneLineStds=[1.0, 0.0, 0.0, 1.0] if lane_mode else [1.0, 1.0, 1.0, 1.0],
    roadEdges=[line(lane_center_y - 5.0), line(lane_center_y + 5.0)],
    roadEdgeStds=[0.0, 0.0],
    meta=SimpleNamespace(
      desire=desire,
      desireState=[],
      laneWidthLeft=lane_width_left,
      laneWidthRight=lane_width_right,
      laneChangeState=lane_change_state,
      laneChangeDirection=lane_change_direction,
    ),
  )


def build_lateral_planner(params, monkeypatch):
  monkeypatch.setattr(lateral_planner_module, "Params", lambda: params)
  monkeypatch.setattr(lane_planner_module, "Params", lambda: params)

  CP = car.CarParams.new_message()
  CP.mass = 2200.0
  CP.wheelbase = 3.09
  CP.centerToFront = 1.236
  CP.tireStiffnessRear = 140000.0
  return LateralPlanner(CP)


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
    mpcSolutionValid=True,
    modelMonoTime=1_000_000_000,
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


@pytest.mark.parametrize("is_vw_meb", [False, True], ids=["standard", "vw-meb"])
@pytest.mark.parametrize(("failure", "lateral_plan_checks", "plan_mono_time", "mpc_solution_valid"), [
  ("invalid", False, 2_000_000_000, True),
  ("duplicate-old-epoch", True, 1_950_000_000, True),
  ("stale-service-and-epoch", False, 1_950_000_000, True),
  ("mpc-invalid", True, 2_000_000_000, False),
])
def test_unusable_lane_plan_falls_back_to_current_model_curvature(
    is_vw_meb, failure, lateral_plan_checks, plan_mono_time, mpc_solution_valid,
):
  del failure
  current_model_mono_time = 2_000_000_000
  raw_model_curvature = -0.001
  stale_lane_curvature = 0.001
  lateral_plan = SimpleNamespace(
    useLaneLines=True,
    mpcSolutionValid=mpc_solution_valid,
    modelMonoTime=plan_mono_time,
    psis=np.zeros(CONTROL_N).tolist(),
    curvatures=np.full(CONTROL_N, stale_lane_curvature).tolist(),
    distances=np.linspace(0.0, 50.0, CONTROL_N).tolist(),
  )
  controls = build_controls(
    lateral_plan,
    raw_model_curvature,
    lateral_plan_checks=lateral_plan_checks,
    model_mono_time=current_model_mono_time,
    is_vw_meb=is_vw_meb,
  )

  controls.state_control()

  raw_target = raw_model_curvature
  if not is_vw_meb:
    raw_target *= 1.0 - np.exp(-DT_CTRL / 0.1)
  expected_curvature, _ = clip_curvature(V_EGO, 0.0, raw_target, 0.0)
  assert controls.lanefull_mode_enabled is False
  assert controls.desired_curvature == pytest.approx(expected_curvature)
  assert controls.desired_curvature < 0.0


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


def test_laneless_path_does_not_apply_configured_static_offset(monkeypatch):
  params = PlannerParams()
  planner = build_lateral_planner(params, monkeypatch)
  raw_path_y = np.linspace(-0.02, 0.02, LAT_MPC_N + 1)
  car_state = SimpleNamespace(vEgo=V_EGO, useLaneLineSpeed=0.0)
  sm = PlannerSubMaster({
    "carState": car_state,
    "carrotMan": SimpleNamespace(vTurnSpeed=0),
    "controlsState": SimpleNamespace(curvature=0.0),
    "modelV2": build_planner_model(raw_path_y),
  }, model_mono_time=1_000_000_000)
  carrot = SimpleNamespace(atc_active=False)

  planner.update(sm, carrot)

  assert planner.pathOffset == pytest.approx(PATH_OFFSET_M)
  assert planner.appliedPathOffset == pytest.approx(0.0)
  assert not planner.lanelines_active
  assert planner.LP.adjustLaneOffset == pytest.approx(0.0)
  assert planner.raw_model_path_xyz[:, 1] == pytest.approx(raw_path_y)
  assert planner.path_before_static_offset == pytest.approx(raw_path_y)
  assert planner.path_xyz[:, 1] == pytest.approx(raw_path_y)

  sm["modelV2"] = build_planner_model(
    np.full(LAT_MPC_N + 1, 5.0),
    valid_path=False,
    lane_center_y=1.0,
    lane_mode=True,
  )
  sm.logMonoTime["modelV2"] = 2_000_000_000
  planner.update(sm, carrot)

  assert planner.appliedPathOffset == pytest.approx(0.0)
  assert not planner.lanelines_active
  assert planner.path_before_static_offset == pytest.approx(raw_path_y)
  assert planner.path_xyz[:, 1] == pytest.approx(raw_path_y)

  pm = CapturePubMaster()
  planner.publish(sm, pm, carrot)
  assert pm.message.valid is False
  assert pm.message.lateralPlan.modelMonoTime == 1_000_000_000
  assert pm.message.lateralPlan.useLaneLines is False
  assert pm.message.lateralPlan.staticPathOffset == pytest.approx(0.0)
  assert pm.message.lateralPlan.dynamicLaneOffset == pytest.approx(0.0)


def test_laneless_path_does_not_apply_filtered_dynamic_offset(monkeypatch):
  params = PlannerParams(adjust_lane_offset_cm=10)
  planner = build_lateral_planner(params, monkeypatch)
  raw_path_y = np.zeros(LAT_MPC_N + 1)
  sm = PlannerSubMaster({
    "carState": SimpleNamespace(vEgo=V_EGO, useLaneLineSpeed=0.0),
    "carrotMan": SimpleNamespace(vTurnSpeed=0),
    "controlsState": SimpleNamespace(curvature=0.0),
    "modelV2": build_planner_model(
      raw_path_y,
      lane_mode=True,
      lane_width_left=2.2,
      lane_width_right=2.1,
    ),
  }, model_mono_time=1_000_000_000)
  carrot = SimpleNamespace(atc_active=False)

  for frame in range(80):
    sm.logMonoTime["modelV2"] = 1_000_000_000 + frame * 50_000_000
    planner.update(sm, carrot)

  assert planner.LP.lane_offset_filtered.x > 0.0
  assert planner.LP.offset_total == pytest.approx(0.0)
  assert not planner.lanelines_active
  assert planner.appliedPathOffset == pytest.approx(0.0)
  assert planner.path_before_static_offset == pytest.approx(raw_path_y)
  assert planner.path_xyz[:, 1] == pytest.approx(raw_path_y)

  pm = CapturePubMaster()
  planner.publish(sm, pm, carrot)
  assert pm.message.lateralPlan.staticPathOffset == pytest.approx(0.0)
  assert pm.message.lateralPlan.dynamicLaneOffset == pytest.approx(0.0)


def test_invalid_model_cycles_freeze_complete_lane_epoch_and_apply_static_offset_once(monkeypatch):
  params = PlannerParams()
  planner = build_lateral_planner(params, monkeypatch)
  raw_path_y = np.zeros(LAT_MPC_N + 1)
  car_state = SimpleNamespace(vEgo=V_EGO, useLaneLineSpeed=1.0)
  sm = PlannerSubMaster({
    "carState": car_state,
    "carrotMan": SimpleNamespace(vTurnSpeed=0),
    "controlsState": SimpleNamespace(curvature=0.0),
    "modelV2": build_planner_model(raw_path_y, lane_mode=True),
  }, model_mono_time=1_000_000_000)
  carrot = SimpleNamespace(atc_active=False)

  for frame in range(50):
    sm.logMonoTime["modelV2"] = 1_000_000_000 + frame * 50_000_000
    planner.update(sm, carrot)

  valid_model_mono_time = sm.logMonoTime["modelV2"]
  expected_raw_model_path = planner.raw_model_path_xyz.copy()
  expected_t_idxs = planner.t_idxs.copy()
  expected_pre_static = planner.path_before_static_offset.copy()
  expected_plan_yaw = planner.plan_yaw.copy()
  expected_plan_yaw_rate = planner.plan_yaw_rate.copy()
  expected_velocity_xyz = planner.velocity_xyz.copy()
  expected_v_plan = planner.v_plan.copy()
  expected_plan_a = planner.plan_a.copy()
  expected_lane_width_left = planner.LP.lane_width_left
  expected_lane_width_right = planner.LP.lane_width_right
  assert planner.lanelines_active
  assert planner.appliedPathOffset == pytest.approx(PATH_OFFSET_M)
  assert planner.path_xyz[:, 1] == pytest.approx(expected_pre_static + PATH_OFFSET_M)

  invalid_model = build_planner_model(
    np.full(LAT_MPC_N + 1, 5.0),
    valid_path=False,
    lane_center_y=1.0,
    lane_mode=True,
    desire=log.Desire.laneChangeRight,
    lane_change_state=log.LaneChangeState.preLaneChange,
    lane_change_direction=log.LaneChangeDirection.right,
    lane_width_left=3.0,
    lane_width_right=1.0,
  )
  invalid_model.position.z = np.full(LAT_MPC_N + 1, 2.0).tolist()
  invalid_model.position.t = np.linspace(1.0, 4.2, LAT_MPC_N + 1).tolist()
  invalid_model.orientation.z = np.full(LAT_MPC_N + 1, 0.3).tolist()
  invalid_model.orientationRate.z = np.full(LAT_MPC_N + 1, 0.4).tolist()
  invalid_model.velocity.x = np.full(LAT_MPC_N + 1, 5.0).tolist()
  invalid_model.velocity.y = np.full(LAT_MPC_N + 1, 2.0).tolist()
  invalid_model.acceleration.x = np.full(LAT_MPC_N + 1, -1.0).tolist()
  sm["modelV2"] = invalid_model
  for frame in range(3):
    sm.logMonoTime["modelV2"] = 10_000_000_000 + frame * 50_000_000
    planner.update(sm, carrot)

    assert planner.model_mono_time == valid_model_mono_time
    assert planner.raw_model_path_xyz == pytest.approx(expected_raw_model_path)
    assert planner.t_idxs == pytest.approx(expected_t_idxs)
    assert planner.plan_yaw == pytest.approx(expected_plan_yaw)
    assert planner.plan_yaw_rate == pytest.approx(expected_plan_yaw_rate)
    assert planner.velocity_xyz == pytest.approx(expected_velocity_xyz)
    assert planner.v_plan == pytest.approx(expected_v_plan)
    assert planner.plan_a == pytest.approx(expected_plan_a)
    assert (planner.LP.lll_y + planner.LP.rll_y) / 2.0 == pytest.approx(0.0)
    assert planner.LP.lane_width_left == pytest.approx(expected_lane_width_left)
    assert planner.LP.lane_width_right == pytest.approx(expected_lane_width_right)
    assert planner.LP.lane_change_multiplier == pytest.approx(1.0)
    assert planner.model_desire == log.Desire.none
    assert planner.path_before_static_offset == pytest.approx(expected_pre_static)
    assert planner.path_xyz[:, 1] == pytest.approx(expected_pre_static + PATH_OFFSET_M)
    assert planner.lanelines_active
    assert planner.appliedPathOffset == pytest.approx(PATH_OFFSET_M)

  pm = CapturePubMaster()
  planner.publish(sm, pm, carrot)
  lateral_plan = pm.message.lateralPlan
  assert pm.message.valid is False
  assert lateral_plan.modelMonoTime == valid_model_mono_time
  assert lateral_plan.useLaneLines is False
  assert lateral_plan.staticPathOffset == pytest.approx(0.0)
  assert lateral_plan.dynamicLaneOffset == pytest.approx(0.0)
  assert lateral_plan.pathBeforeStaticOffset == pytest.approx(expected_pre_static)
  assert lateral_plan.laneChangeState == log.LaneChangeState.off
  assert lateral_plan.laneChangeDirection == log.LaneChangeDirection.none
