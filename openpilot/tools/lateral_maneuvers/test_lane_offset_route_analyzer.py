from types import SimpleNamespace
import math

import numpy as np
import pytest

from openpilot.tools.lateral_maneuvers.lane_offset_route_analyzer import (
  FrameEvidence,
  MpcReplayResult,
  _load_lateral_mpc,
  _yaw_from_path,
  analyze_events,
  analyze_physical_ab_events,
  fit_ego_right_from_lane_lines,
  parse_debug_offsets,
  replay_mpc_static_counterfactual,
)


class FakeEvent:
  def __init__(self, name, mono_time, payload):
    self.logMonoTime = mono_time
    self._name = name
    setattr(self, name, payload)

  def which(self):
    return self._name


def namespace(**kwargs):
  return SimpleNamespace(**kwargs)


def init_event(params):
  entries = [namespace(key=key, value=str(value).encode()) for key, value in params.items()]
  return FakeEvent("initData", 1, namespace(
    params=namespace(entries=entries),
    dongleId="synthetic-dongle",
    gitBranch="carrot-wip",
    gitCommit="",
  ))


def model_event(mono_time, *, lane_center_y=0.0, speed_ms=20.0):
  count = 33
  x = np.linspace(0.0, 100.0, count).tolist()
  t = np.linspace(0.0, 10.0, count).tolist()
  def line(y):
    return namespace(x=x, y=[y] * count, t=t)
  model = namespace(
    position=namespace(x=x, y=[0.0] * count, z=[0.0] * count, t=t),
    velocity=namespace(x=[speed_ms] * count, y=[0.0] * count, z=[0.0] * count),
    laneLines=[line(lane_center_y - 5.2), line(lane_center_y - 1.75),
               line(lane_center_y + 1.75), line(lane_center_y + 5.2)],
    laneLineProbs=[0.1, 0.95, 0.95, 0.1],
    laneLineStds=[0.3, 0.1, 0.1, 0.3],
    action=namespace(desiredCurvature=0.0),
    meta=namespace(desire="none"),
  )
  return FakeEvent("modelV2", mono_time, model)


def plan_event(event_time, model_time, *, dynamic_cm=2.0, include_dynamic=True, include_numeric=True,
               observed_static_m=0.10, published_static_m=None, pre_static_y_m=None):
  dynamic_m = dynamic_cm * 0.01
  if published_static_m is None:
    published_static_m = observed_static_m
  if pre_static_y_m is None:
    pre_static_y_m = dynamic_m
  debug = "lanemode | 3.0m | 3.5m | 3.0m"
  if include_dynamic:
    debug += f" | offset={dynamic_cm:.1f}cm turn=0km/h"
  count = 33
  curvatures = [0.0] + [0.00012] * 7 + [0.00002] * 9
  distances = np.linspace(0.0, 50.0, 17)
  control_target = 0.0001
  plan_values = dict(
    modelMonoTime=model_time,
    useLaneLines=True,
    dPathPoints=[pre_static_y_m + observed_static_m] * count,
    curvatures=curvatures,
    psis=(0.5 * control_target * distances).tolist(),
    distances=distances.tolist(),
    position=namespace(x=np.linspace(0.0, 100.0, count).tolist(), y=[0.0] * count),
    latDebugText=debug,
    laneChangeState="off",
  )
  if include_numeric:
    plan_values.update(
      staticPathOffset=published_static_m,
      dynamicLaneOffset=dynamic_m,
      pathBeforeStaticOffset=[pre_static_y_m] * count,
    )
  plan = namespace(**plan_values)
  return FakeEvent("lateralPlan", event_time, plan)


def car_params_event():
  return FakeEvent("carParams", 2, namespace(
    carFingerprint="KIA_CARNIVAL_4TH_GEN",
    steerControlType="torque",
    wheelbase=3.09,
    centerToFront=1.35,
    mass=2200.0,
    tireStiffnessRear=140000.0,
  ))


def linked_control_events(base_time, model_time, *, desired_curvature=0.0001, torque=-0.2,
                          can_torque=-100.0, speed_ms=20.0):
  controls_time = base_time + 2_000_000
  return [
    FakeEvent("carState", controls_time, namespace(vEgo=speed_ms, steeringPressed=False)),
    FakeEvent("liveDelay", controls_time, namespace(lateralDelay=0.1)),
    FakeEvent("controlsState", controls_time, namespace(
      lateralPlanMonoTime=model_time,
      desiredCurvature=desired_curvature,
      activeLaneLine=True,
    )),
    FakeEvent("carControl", controls_time, namespace(
      latActive=True,
      actuators=namespace(torque=torque, steeringAngleDeg=-1.0),
    )),
    FakeEvent("carOutput", controls_time, namespace(
      actuatorsOutput=namespace(torque=-0.18, torqueOutputCan=can_torque, steeringAngleDeg=-1.0),
    )),
  ]


def synthetic_route(*, include_dynamic=True, include_numeric=True, observed_static_m=0.10,
                    published_static_m=None, path_offset_raw=10):
  params = {
    "PathOffset": path_offset_raw,
    "AdjustLaneOffset": 20,
    "LatMpcInputOffset": 0,
    "LatMpcPathCost": 200,
    "LatMpcMotionCost": 7,
    "LatMpcAccelCost": 120,
    "LatMpcJerkCost": 4,
    "LatMpcSteeringRateCost": 7,
    "SteerActuatorDelay": 10,
    "LatSmoothSec": 0,
  }
  events = [init_event(params), car_params_event()]
  for index in range(6):
    model_time = 1_000_000_000 + index * 50_000_000
    plan_time = model_time + 1_000_000
    events.extend((
      model_event(model_time),
      plan_event(plan_time, model_time, include_dynamic=include_dynamic, include_numeric=include_numeric,
                 observed_static_m=observed_static_m, published_static_m=published_static_m),
      *linked_control_events(plan_time, model_time),
    ))
  return events


def gps_event(mono_time, latitude, longitude, *, speed_ms=20.0, heading_deg=0.0):
  return FakeEvent("gpsLocationExternal", mono_time, namespace(
    hasFix=True,
    latitude=latitude,
    longitude=longitude,
    speed=speed_ms,
    bearingDeg=heading_deg,
    horizontalAccuracy=1.0,
    bearingAccuracyDeg=1.0,
    speedAccuracy=0.2,
  ))


def synthetic_physical_route(*, static_offset_m, ego_right_m, dynamic_offset_m=0.0,
                             longitude_shift_deg=0.0, camera_bias_y_m=0.03):
  params = {
    "PathOffset": round(static_offset_m * 100),
    "AdjustLaneOffset": 0,
  }
  events = [init_event(params), car_params_event()]
  latitude_start = 37.0
  longitude = 127.0 + longitude_shift_deg
  spacing_m = 25.0
  latitude_step = math.degrees(spacing_m / 6_371_000.0)
  lane_center_y = camera_bias_y_m - ego_right_m
  for index in range(12):
    model_time = 1_000_000_000 + index * 1_000_000_000
    plan_time = model_time + 1_000_000
    events.extend((
      model_event(model_time, lane_center_y=lane_center_y),
      plan_event(
        plan_time,
        model_time,
        dynamic_cm=dynamic_offset_m * 100.0,
        observed_static_m=static_offset_m,
        published_static_m=static_offset_m,
        pre_static_y_m=lane_center_y + dynamic_offset_m,
      ),
      *linked_control_events(plan_time, model_time),
      gps_event(plan_time, latitude_start + index * latitude_step, longitude),
    ))
  return events


def fake_right_mpc_replay(_frame):
  return MpcReplayResult(
    usable=True,
    observed_rmse=1e-6,
    right_curvature_delta=2e-5,
    right_curvature_peak=4e-5,
  )


def test_debug_offset_parser_keeps_dynamic_and_turn_separate():
  dynamic, turn = parse_debug_offsets("lanemode | offset=-3.2cm turn=74km/h")
  assert dynamic == -0.032
  assert turn == 74.0


def test_strict_route_proves_static_dynamic_mpc_and_control_propagation():
  report = analyze_events(
    synthetic_route(),
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["configured_path_offset"].status == "proven"
  assert report.stages["dynamic_adjust_lane_offset"].status == "proven"
  assert report.stages["dynamic_adjust_lane_offset"].details["applied_dynamic_median_m"] == 0.02
  assert report.stages["lateral_plan_static_offset"].status == "proven"
  assert report.stages["lateral_plan_static_offset"].details["median_static_residual_m"] == 0.10
  assert report.stages["mpc_static_offset_effect"].status == "proven"
  assert report.stages["controls_lane_plan_selection"].status == "proven"
  assert report.stages["carcontroller_command_tracking"].status == "proven"
  assert report.stages["physical_rightward_displacement"].status == "missing"


def test_direct_numeric_fields_take_priority_when_legacy_debug_text_is_absent():
  report = analyze_events(
    synthetic_route(include_dynamic=False, include_numeric=True),
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["dynamic_adjust_lane_offset"].status == "proven"
  assert report.stages["dynamic_adjust_lane_offset"].details["direct_numeric_samples"] == 6
  assert report.stages["lateral_plan_static_offset"].status == "proven"
  assert report.stages["lateral_plan_static_offset"].details["direct_numeric_frames"] == 6


def test_missing_dynamic_field_never_claims_static_decomposition():
  report = analyze_events(
    synthetic_route(include_dynamic=False, include_numeric=False),
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["configured_path_offset"].status == "proven"
  assert report.stages["dynamic_adjust_lane_offset"].status == "missing"
  assert report.stages["lateral_plan_static_offset"].status == "missing"
  assert report.stages["mpc_static_offset_effect"].status == "missing"


def test_route_contradicts_static_offset_when_dpath_contains_only_dynamic_offset():
  report = analyze_events(
    synthetic_route(observed_static_m=0.0, published_static_m=0.10),
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["configured_path_offset"].status == "proven"
  assert report.stages["lateral_plan_static_offset"].status == "contradicted"
  assert report.stages["lateral_plan_static_offset"].details["median_static_residual_m"] == 0.0
  assert report.stages["mpc_static_offset_effect"].status == "missing"


def test_wrong_configured_path_offset_is_reported_as_contradicted():
  report = analyze_events(
    synthetic_route(path_offset_raw=0, observed_static_m=0.0),
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["configured_path_offset"].status == "contradicted"


def test_lane_center_coordinate_sign_is_vehicle_right_positive():
  event = model_event(1, lane_center_y=-0.10)
  # A far-field outlier should not move the robust origin fit materially.
  event.modelV2.laneLines[1].y[15] += 1.5
  event.modelV2.laneLines[2].y[15] -= 1.0

  lane_fit, error = fit_ego_right_from_lane_lines(event.modelV2)

  assert error == ""
  assert lane_fit is not None
  assert lane_fit.lane_center_y_m == pytest.approx(-0.10, abs=0.01)
  assert lane_fit.ego_right_of_lane_center_m == pytest.approx(0.10, abs=0.01)


def test_physical_ab_proves_positive_ten_centimeter_shift_with_fixed_camera_bias():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10),
  )

  assert evidence.status == "proven"
  assert evidence.sample_count >= 8
  assert evidence.details["median_rightward_delta_m"] == pytest.approx(0.10, abs=0.005)
  assert evidence.details["baseline_accepted_samples"] == 12
  assert evidence.details["variant_accepted_samples"] == 12


def test_physical_ab_wrong_sign_is_contradicted():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=-0.10),
  )

  assert evidence.status == "contradicted"
  assert evidence.details["median_rightward_delta_m"] == pytest.approx(-0.10, abs=0.005)


def test_physical_ab_insufficient_spatial_overlap_is_missing():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, longitude_shift_deg=0.01),
  )

  assert evidence.status == "missing"
  assert evidence.details["spatial_bin_count"] == 0


def test_physical_ab_nonzero_dynamic_offset_is_missing():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, dynamic_offset_m=0.02),
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_accepted_samples"] == 0
  assert evidence.details["variant_rejections"]["dynamic_offset_nonzero"] == 12


def test_real_mpc_counterfactual_is_right_positive_and_matches_recorded_solution():
  lateral_mpc, horizon = _load_lateral_mpc()
  speed = np.full(horizon + 1, 20.0)
  x_path = np.linspace(0.0, 100.0, horizon + 1)
  y_target = np.full(horizon + 1, 0.10)
  zeros = np.zeros(horizon + 1)
  car_params = namespace(
    wheelbase=3.09,
    centerToFront=1.35,
    mass=2200.0,
    tireStiffnessRear=140000.0,
  )
  factor1 = car_params.wheelbase - car_params.centerToFront
  factor2 = car_params.centerToFront * car_params.mass / (car_params.wheelbase * car_params.tireStiffnessRear)
  solver_params = np.column_stack((speed, np.clip(factor1 - factor2 * speed**2, 0.0, np.inf)))
  heading, yaw_rate = _yaw_from_path(np.column_stack((x_path, y_target, zeros)), speed)
  x0 = np.zeros(4)
  mpc = lateral_mpc(x0=x0)
  mpc.set_weights(2.0, 0.07, 1.2, 0.04, 7.0)
  for _ in range(5):
    mpc.run(x0, solver_params, y_target, heading, yaw_rate)

  plan = namespace(
    dPathPoints=y_target.tolist(),
    curvatures=(mpc.x_sol[:17, 3] / speed[:17]).tolist(),
    psis=mpc.x_sol[:17, 2].tolist(),
    position=namespace(x=mpc.x_sol[:, 0].tolist(), y=mpc.x_sol[:, 1].tolist()),
  )
  model = namespace(
    position=namespace(x=x_path.tolist(), z=zeros.tolist()),
    velocity=namespace(x=speed.tolist(), y=zeros.tolist(), z=zeros.tolist()),
  )
  frame = FrameEvidence(
    mono_time=0,
    model_mono_time=0,
    configured_path_offset_m=0.10,
    path_offset_m=0.10,
    adjust_lane_offset_m=0.0,
    dynamic_offset_m=0.0,
    static_residual_m=0.10,
    static_max_error_m=0.0,
    base_target_near_m=0.0,
    lane_center_curvature=0.0,
    model_desired_curvature=0.0,
    plan_early_curvature=0.0,
    plan_peak_curvature=0.0,
    geometry_exact=True,
    evidence_source="direct_numeric",
    straight=True,
    _plan=plan,
    _model=model,
    _car_params=car_params,
    _params={
      "LatMpcPathCost": "200",
      "LatMpcMotionCost": "7",
      "LatMpcAccelCost": "120",
      "LatMpcJerkCost": "4",
      "LatMpcSteeringRateCost": "7",
    },
  )

  result = replay_mpc_static_counterfactual(frame)

  assert result.usable is True
  assert result.observed_rmse < 1e-10
  assert result.right_curvature_delta > 0.0
