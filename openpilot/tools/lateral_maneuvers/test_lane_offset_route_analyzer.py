from copy import deepcopy
from types import SimpleNamespace
import math

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.tools.lateral_maneuvers.lane_offset_route_analyzer import (
  CODE_PATHS,
  FrameEvidence,
  MpcReplayResult,
  PHYSICAL_AB_LATERAL_PARAM_KEYS,
  _load_lateral_mpc,
  _yaw_from_path,
  analyze_events,
  analyze_physical_ab_events,
  extract_physical_lane_samples,
  fit_ego_right_from_lane_lines,
  parse_debug_offsets,
  replay_mpc_static_counterfactual,
)


class FakeEvent:
  def __init__(self, name, mono_time, payload):
    self.logMonoTime = mono_time
    self.valid = True
    self._name = name
    setattr(self, name, payload)

  def which(self):
    return self._name


def namespace(**kwargs):
  return SimpleNamespace(**kwargs)


def init_event(params, *, branch="carrot-wip", commit="synthetic-commit", dirty=False):
  entries = [namespace(key=key, value=str(value).encode()) for key, value in params.items()]
  values = dict(
    params=namespace(entries=entries),
    dongleId="synthetic-dongle",
    gitBranch=branch,
    gitCommit=commit,
  )
  if dirty is not None:
    values["dirty"] = dirty
  return FakeEvent("initData", 1, namespace(**values))


def model_event(mono_time, *, lane_center_y=0.0, speed_ms=20.0):
  count = 33
  x = np.linspace(0.0, 100.0, count).tolist()
  t = np.linspace(0.0, 10.0, count).tolist()

  def line(y):
    return namespace(x=x, y=[y] * count, t=t)

  model = namespace(
    position=namespace(x=x, y=[0.0] * count, z=[0.0] * count, t=t),
    velocity=namespace(x=[speed_ms] * count, y=[0.0] * count, z=[0.0] * count),
    laneLines=[line(lane_center_y - 5.2), line(lane_center_y - 1.75), line(lane_center_y + 1.75), line(lane_center_y + 5.2)],
    laneLineProbs=[0.1, 0.95, 0.95, 0.1],
    laneLineStds=[0.3, 0.1, 0.1, 0.3],
    action=namespace(desiredCurvature=0.0),
    meta=namespace(desire="none"),
  )
  return FakeEvent("modelV2", mono_time, model)


def plan_event(
  event_time, model_time, *, dynamic_cm=2.0, include_dynamic=True, include_numeric=True, observed_static_m=0.10, published_static_m=None, pre_static_y_m=None
):
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
    mpcSolutionValid=True,
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


def capnp_plan_event(event):
  source = event.lateralPlan
  message = messaging.new_message("lateralPlan")
  message.logMonoTime = event.logMonoTime
  message.valid = event.valid
  plan = message.lateralPlan
  plan.modelMonoTime = source.modelMonoTime
  plan.useLaneLines = source.useLaneLines
  plan.mpcSolutionValid = source.mpcSolutionValid
  plan.dPathPoints = source.dPathPoints
  plan.curvatures = source.curvatures
  plan.psis = source.psis
  plan.distances = source.distances
  plan.position.x = source.position.x
  plan.position.y = source.position.y
  plan.latDebugText = source.latDebugText
  plan.laneChangeState = source.laneChangeState
  plan.staticPathOffset = source.staticPathOffset
  plan.dynamicLaneOffset = source.dynamicLaneOffset
  plan.pathBeforeStaticOffset = source.pathBeforeStaticOffset
  return message


def car_params_event():
  return FakeEvent(
    "carParams",
    2,
    namespace(
      carFingerprint="KIA_CARNIVAL_4TH_GEN",
      steerControlType="torque",
      wheelbase=3.09,
      centerToFront=1.35,
      mass=2200.0,
      tireStiffnessRear=140000.0,
    ),
  )


def linked_control_events(base_time, model_time, *, desired_curvature=0.0001, torque=-0.2, can_torque=-100.0, speed_ms=20.0):
  controls_time = base_time + 2_000_000
  control_input_time = controls_time - 1_000_000
  car_control_time = controls_time + 1_000_000
  runtime_time = base_time - 1_000_000
  return [
    FakeEvent("carState", control_input_time, namespace(vEgo=speed_ms, steeringPressed=False)),
    FakeEvent(
      "carOutput",
      control_input_time,
      namespace(
        actuatorsOutput=namespace(torque=torque * 0.975, torqueOutputCan=can_torque, steeringAngleDeg=-1.0),
      ),
    ),
    FakeEvent(
      "liveParameters",
      runtime_time,
      namespace(
        valid=True,
        angleOffsetValid=True,
        angleOffsetAverageValid=True,
        stiffnessFactorValid=True,
        steerRatioValid=True,
        steerRatio=15.0,
        stiffnessFactor=1.0,
        angleOffsetDeg=0.1,
        angleOffsetAverageDeg=0.1,
        roll=0.01,
        steerRatioStd=0.1,
        stiffnessFactorStd=0.02,
        angleOffsetAverageStd=0.05,
        angleOffsetFastStd=0.05,
      ),
    ),
    FakeEvent("liveTorqueParameters", runtime_time, namespace(useParams=False, liveValid=False)),
    FakeEvent("liveDelay", runtime_time, namespace(lateralDelay=0.1, status="unestimated")),
    FakeEvent(
      "controlsState",
      controls_time,
      namespace(
        lateralPlanMonoTime=model_time,
        desiredCurvature=desired_curvature,
        activeLaneLine=True,
        lateralControlState=namespace(
          which=lambda: "torqueState",
          torqueState=namespace(active=True, saturated=False),
        ),
      ),
    ),
    FakeEvent(
      "carControl",
      car_control_time,
      namespace(
        latActive=True,
        actuators=namespace(torque=torque, steeringAngleDeg=-1.0),
      ),
    ),
  ]


def synthetic_route(*, include_dynamic=True, include_numeric=True, observed_static_m=0.10, published_static_m=None, path_offset_raw=10):
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
    events.extend(
      (
        model_event(model_time),
        plan_event(
          plan_time,
          model_time,
          include_dynamic=include_dynamic,
          include_numeric=include_numeric,
          observed_static_m=observed_static_m,
          published_static_m=published_static_m,
        ),
        *linked_control_events(plan_time, model_time),
      )
    )
  return events


def gps_event(mono_time, latitude, longitude, *, speed_ms=20.0, heading_deg=0.0, source="ublox"):
  return FakeEvent(
    "gpsLocationExternal",
    mono_time,
    namespace(
      hasFix=True,
      latitude=latitude,
      longitude=longitude,
      speed=speed_ms,
      bearingDeg=heading_deg,
      horizontalAccuracy=1.0,
      bearingAccuracyDeg=1.0,
      speedAccuracy=0.2,
      source=source,
    ),
  )


def llk_event(
  mono_time, latitude, longitude, *, speed_ms=20.0, heading_deg=0.0, position_std_m=1.0, velocity_std_ms=0.2, status="valid", gps_ok=True, inputs_ok=True
):
  heading_rad = math.radians(heading_deg)
  north_velocity = speed_ms * math.cos(heading_rad)
  east_velocity = speed_ms * math.sin(heading_rad)
  latitude_std_deg = math.degrees(position_std_m / 6_371_000.0)
  longitude_std_deg = latitude_std_deg / math.cos(math.radians(latitude))
  return FakeEvent(
    "liveLocationKalmanDEPRECATED",
    mono_time,
    namespace(
      status=status,
      gpsOK=gps_ok,
      inputsOK=inputs_ok,
      positionGeodetic=namespace(
        value=[latitude, longitude, 0.0],
        std=[latitude_std_deg, longitude_std_deg, position_std_m],
        valid=True,
      ),
      velocityNED=namespace(
        value=[north_velocity, east_velocity, 0.0],
        std=[velocity_std_ms, velocity_std_ms, velocity_std_ms],
        valid=True,
      ),
    ),
  )


def calibration_event(mono_time, *, rpy=(0.0, 0.02, 0.01), height_m=1.4, status="calibrated"):
  return FakeEvent(
    "liveCalibration",
    mono_time,
    namespace(
      calStatus=status,
      rpyCalib=list(rpy),
      height=[height_m],
    ),
  )


def physical_lateral_params(**overrides):
  params = dict.fromkeys(PHYSICAL_AB_LATERAL_PARAM_KEYS, 0)
  params.update(
    {
      "LatMpcPathCost": 200,
      "LatMpcMotionCost": 7,
      "LatMpcAccelCost": 120,
      "LatMpcJerkCost": 4,
      "LatMpcSteeringRateCost": 7,
      "SteerActuatorDelay": 10,
      "LateralTorqueAccelFactor": 2500,
      "LateralTorqueFriction": 100,
      "LateralTorqueKpV": 100,
      "LateralTorqueKiV": 10,
      "LateralTorqueKf": 100,
      "MaxAngleFrames": 89,
    }
  )
  params.update(overrides)
  return params


def synthetic_physical_route(
  *,
  static_offset_m,
  ego_right_m,
  dynamic_offset_m=0.0,
  longitude_shift_deg=0.0,
  camera_bias_y_m=0.03,
  branch="carrot-wip",
  commit="synthetic-commit",
  lateral_param_overrides=None,
  calibration_rpy=(0.0, 0.02, 0.01),
  calibration_height_m=1.4,
  calibration_status="calibrated",
  gps_accuracy_m=1.0,
  gps_source="ublox",
  dirty=False,
  use_llk=False,
  llk_position_std_m=1.0,
):
  params = physical_lateral_params(**(lateral_param_overrides or {}))
  params.update(
    {
      "PathOffset": round(static_offset_m * 100),
      "AdjustLaneOffset": 0,
    }
  )
  events = [init_event(params, branch=branch, commit=commit, dirty=dirty), car_params_event()]
  latitude_start = 37.0
  longitude = 127.0 + longitude_shift_deg
  spacing_m = 25.0
  latitude_step = math.degrees(spacing_m / 6_371_000.0)
  lane_center_y = camera_bias_y_m - ego_right_m
  for index in range(12):
    model_time = 1_000_000_000 + index * 1_000_000_000
    plan_time = model_time + 1_000_000
    location = (
      llk_event(
        plan_time,
        latitude_start + index * latitude_step,
        longitude,
        position_std_m=llk_position_std_m,
      )
      if use_llk
      else gps_event(plan_time, latitude_start + index * latitude_step, longitude, source=gps_source)
    )
    if not use_llk:
      location.gpsLocationExternal.horizontalAccuracy = gps_accuracy_m
    events.extend(
      (
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
        calibration_event(
          plan_time,
          rpy=calibration_rpy,
          height_m=calibration_height_m,
          status=calibration_status,
        ),
        location,
      )
    )
  return events


def service_payloads(events, service):
  return [getattr(event, service) for event in events if event.which() == service]


def enable_live_torque(events, *, factor=2.5, offset=0.0, friction=0.1):
  for payload in service_payloads(events, "liveTorqueParameters"):
    payload.useParams = True
    payload.liveValid = True
    payload.latAccelFactorFiltered = factor
    payload.latAccelOffsetFiltered = offset
    payload.frictionCoefficientFiltered = friction


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


def test_physical_extractor_accepts_production_33_point_capnp_offset_evidence():
  _, lateral_mpc_horizon = _load_lateral_mpc()

  assert lateral_mpc_horizon + 1 == 33
  events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  plan_index = next(index for index, event in enumerate(events) if event.which() == "lateralPlan")
  events[plan_index] = capnp_plan_event(events[plan_index])

  samples = extract_physical_lane_samples(events, expected_static_offset_m=0.0)

  assert len(events[plan_index].lateralPlan.dPathPoints) == lateral_mpc_horizon + 1
  assert len(events[plan_index].lateralPlan.pathBeforeStaticOffset) == lateral_mpc_horizon + 1
  assert len(samples.samples) == 12


def test_physical_extractor_rejects_nonproduction_direct_evidence_horizon():
  events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  plan = next(event.lateralPlan for event in events if event.which() == "lateralPlan")
  plan.dPathPoints = plan.dPathPoints[:17]
  plan.pathBeforeStaticOffset = plan.pathBeforeStaticOffset[:17]

  samples = extract_physical_lane_samples(events, expected_static_offset_m=0.0)

  assert len(samples.samples) == 11
  assert samples.rejections["direct_numeric_evidence_invalid"] == 1


@pytest.mark.parametrize("mpc_value", [None, False])
def test_physical_ab_requires_explicit_valid_mpc_solution(mpc_value):
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "lateralPlan"):
    if mpc_value is None:
      del payload.mpcSolutionValid
    else:
      payload.mpcSolutionValid = mpc_value

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["mpc_solution_invalid"] == 12


@pytest.mark.parametrize("saturated", [None, True])
def test_physical_ab_requires_explicit_unsaturated_lateral_control(saturated):
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "controlsState"):
    if saturated is None:
      del payload.lateralControlState
    else:
      payload.lateralControlState.torqueState.saturated = saturated

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  rejection = "lateral_control_saturation_missing" if saturated is None else "lateral_control_saturated"
  assert evidence.details["variant_rejections"][rejection] == 12


def test_physical_ab_rejects_safety_limited_steering_command():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "carOutput"):
    payload.actuatorsOutput.torque = -0.10

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["steering_command_safety_limited"] == 12


def test_physical_ab_requires_car_output_for_safety_limit_check():
  variant = [event for event in synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10) if event.which() != "carOutput"]

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["car_output_missing"] == 12


def test_physical_ab_rejects_future_car_output_from_next_control_phase():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for event in variant:
    if event.which() == "carOutput":
      event.logMonoTime += 2_000_000

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["car_output_missing"] == 12


def test_physical_ab_rejects_future_car_state_from_next_control_phase():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for event in variant:
    if event.which() == "carState":
      event.logMonoTime += 2_000_000

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["car_state_missing"] == 12


@pytest.mark.parametrize(
  ("service", "rejection"),
  (("carOutput", "car_output_missing"), ("carState", "car_state_missing")),
)
def test_physical_ab_rejects_stale_control_inputs(service, rejection):
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for event in variant:
    if event.which() == service:
      event.logMonoTime -= 20_000_000

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"][rejection] == 12


def test_physical_ab_rejects_previous_cycle_car_control():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for event in variant:
    if event.which() == "carControl":
      event.logMonoTime -= 2_000_000

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["car_control_missing_or_misordered"] == 12


def test_physical_ab_rejects_car_control_after_next_controls_state():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  next_controls_events = [
    FakeEvent("controlsState", event.logMonoTime + 500_000, deepcopy(event.controlsState)) for event in variant if event.which() == "controlsState"
  ]
  variant.extend(next_controls_events)

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["car_control_missing_or_misordered"] == 12


def test_physical_ab_supports_unsaturated_angle_control_tracking():
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for events in (baseline, variant):
    service_payloads(events, "carParams")[0].steerControlType = "angle"
    for payload in service_payloads(events, "controlsState"):
      payload.lateralControlState = namespace(
        which=lambda: "angleState",
        angleState=namespace(active=True, saturated=False),
      )

  evidence = analyze_physical_ab_events(baseline, variant)

  assert evidence.status == "proven"


def test_physical_ab_rejects_nonfinite_lane_confidence():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "modelV2"):
    payload.laneLineProbs[1] = math.nan

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["inner_lane_confidence_invalid"] == 12


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


@pytest.mark.parametrize(
  ("field", "value"),
  [
    ("branch", "carrot-egpu"),
    ("commit", "different-commit"),
  ],
)
def test_physical_ab_requires_same_branch_and_commit(field, value):
  variant_options = {field: value}
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, **variant_options),
  )

  assert evidence.status == "missing"
  assert not evidence.details["same_branch_and_commit"]


def test_physical_ab_requires_matching_pathoffset_independent_lateral_params():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      lateral_param_overrides={"CameraYawTrimDeg": 25},
    ),
  )

  assert evidence.status == "missing"
  assert evidence.details["mismatched_or_missing_lateral_params"] == ["CameraYawTrimDeg"]


@pytest.mark.parametrize("param", ["CustomSR", "SteerRatioRate"])
def test_physical_ab_requires_matching_vehicle_model_steer_ratio_params(param):
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      lateral_param_overrides={param: 200},
    ),
  )

  assert evidence.status == "missing"
  assert evidence.details["mismatched_or_missing_lateral_params"] == [param]


@pytest.mark.parametrize(
  ("route_name", "dirty"),
  [
    ("baseline", True),
    ("variant", True),
    ("baseline", None),
    ("variant", None),
  ],
)
def test_physical_ab_requires_explicit_clean_init_data_on_both_routes(route_name, dirty):
  options = {route_name: {"dirty": dirty}}
  baseline = synthetic_physical_route(
    static_offset_m=0.0,
    ego_right_m=0.0,
    **options.get("baseline", {}),
  )
  variant = synthetic_physical_route(
    static_offset_m=0.10,
    ego_right_m=0.10,
    **options.get("variant", {}),
  )

  evidence = analyze_physical_ab_events(baseline, variant)

  assert evidence.status == "missing"
  assert not evidence.details[f"{route_name}_explicitly_clean"]


def test_physical_ab_rejects_multiple_init_data_snapshots_per_input():
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  duplicate_init = deepcopy(next(event for event in baseline if event.which() == "initData"))
  duplicate_init.logMonoTime += 1
  baseline.append(duplicate_init)

  evidence = analyze_physical_ab_events(
    baseline,
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10),
  )

  assert evidence.status == "missing"
  assert evidence.details["baseline_dirty_values"] == [False, False]
  assert not evidence.details["baseline_explicitly_clean"]


@pytest.mark.parametrize("service", ["liveParameters", "liveTorqueParameters", "liveDelay"])
def test_physical_ab_requires_runtime_lateral_service_at_each_sample(service):
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  variant = [event for event in variant if event.which() != service]

  evidence = analyze_physical_ab_events(baseline, variant)

  assert evidence.status == "missing"
  assert not evidence.details["runtime_lateral_state_stable_and_matched"]
  rejection_key = {
    "liveParameters": "live_parameters_missing_or_stale",
    "liveTorqueParameters": "live_torque_parameters_missing_or_stale",
    "liveDelay": "live_delay_missing_or_stale",
  }[service]
  assert evidence.details["variant_rejections"][rejection_key] == 12


def test_physical_ab_rejects_invalid_live_parameters_payload():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "liveParameters"):
    payload.valid = False

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["live_parameters_invalid"] == 12


def test_physical_ab_rejects_invalid_enabled_live_torque_payload():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "liveTorqueParameters"):
    payload.useParams = True
    payload.liveValid = False

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["live_torque_parameters_invalid"] == 12


def test_physical_ab_rejects_invalid_live_delay_payload():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "liveDelay"):
    payload.status = "invalid"

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_rejections"]["live_delay_invalid"] == 12


@pytest.mark.parametrize(
  ("service", "field", "value", "expected_mismatch"),
  [
    ("liveParameters", "steerRatio", 17.0, "liveParameters.steerRatio"),
    ("liveDelay", "lateralDelay", 0.20, "liveDelay.lateralDelay"),
  ],
)
def test_physical_ab_requires_runtime_lateral_value_parity(service, field, value, expected_mismatch):
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, service):
    setattr(payload, field, value)

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  mismatches = evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]
  assert expected_mismatch in mismatches


def test_physical_ab_requires_live_parameter_uncertainty_parity():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "liveParameters"):
    payload.steerRatioStd = 2.0

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert "liveParameters.steerRatioStd" in evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]


def test_physical_ab_requires_live_torque_mode_and_values_to_match():
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  enable_live_torque(variant)

  mode_evidence = analyze_physical_ab_events(baseline, variant)

  assert mode_evidence.status == "missing"
  assert "liveTorqueParameters.useParams" in mode_evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]

  enable_live_torque(baseline)
  enable_live_torque(variant, factor=3.0)
  value_evidence = analyze_physical_ab_events(baseline, variant)

  assert value_evidence.status == "missing"
  assert "liveTorqueParameters.latAccelFactorFiltered" in value_evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]


def test_physical_ab_rejects_unstable_runtime_lateral_state():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for index, payload in enumerate(service_payloads(variant, "liveParameters")):
    payload.steerRatio = 15.0 if index % 2 == 0 else 18.0

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  comparison = evidence.details["runtime_lateral_state"]["field_comparison"]["liveParameters.steerRatio"]
  assert not comparison["variant_stable"]


def test_physical_ab_requires_live_delay_status_and_estimate_parity():
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)

  def estimated(events, *, estimate=0.1):
    for payload in service_payloads(events, "liveDelay"):
      payload.status = "estimated"
      payload.lateralDelayEstimate = estimate
      payload.lateralDelayEstimateStd = 0.02
      payload.validBlocks = 10

  estimated(variant)
  status_evidence = analyze_physical_ab_events(baseline, variant)

  assert status_evidence.status == "missing"
  assert "liveDelay.status" in status_evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]

  estimated(baseline)
  estimated(variant, estimate=0.25)
  estimate_evidence = analyze_physical_ab_events(baseline, variant)

  assert estimate_evidence.status == "missing"
  assert "liveDelay.lateralDelayEstimate" in estimate_evidence.details["runtime_lateral_state"]["mismatched_missing_or_unstable_fields"]


def test_physical_sample_runtime_state_must_precede_plan():
  events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  for event in events:
    if event.which() == "liveParameters":
      event.logMonoTime += 5_000_000

  samples = extract_physical_lane_samples(events, expected_static_offset_m=0.0)

  assert not samples.samples
  assert samples.rejections["live_parameters_missing_or_stale"] == 12


def test_physical_ab_requires_stable_matching_calibration():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      calibration_rpy=(0.0, 0.02, 0.01 + math.radians(1.0)),
    ),
  )

  assert evidence.status == "missing"
  assert not evidence.details["calibration_stable_and_matched"]
  assert evidence.details["calibration_rpy_delta_deg"][2] == pytest.approx(1.0)


def test_physical_ab_rejects_uncalibrated_route_samples():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      calibration_status="recalibrating",
    ),
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_accepted_samples"] == 0
  assert evidence.details["variant_rejections"]["calibration_missing_or_invalid"] == 12


def test_physical_ab_rejects_low_accuracy_gps():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, gps_accuracy_m=5.0),
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0
  assert evidence.details["variant_rejections"]["location_match_missing"] == 12


def test_physical_ab_accepts_accurate_valid_llk_fixes():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0, use_llk=True),
    synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, use_llk=True),
  )

  assert evidence.status == "proven"
  assert evidence.details["baseline_location_source"] == "liveLocationKalman"
  assert evidence.details["variant_location_source"] == "liveLocationKalman"


def test_physical_ab_rejects_llk_with_huge_position_standard_deviation():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0, use_llk=True),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      use_llk=True,
      llk_position_std_m=100.0,
    ),
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0
  assert evidence.details["variant_rejections"]["location_match_missing"] == 12


def test_physical_ab_rejects_llk_with_huge_velocity_standard_deviation():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, use_llk=True)
  for payload in service_payloads(variant, "liveLocationKalmanDEPRECATED"):
    payload.velocityNED.std = [100.0, 100.0, 100.0]

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0, use_llk=True),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0


def test_physical_ab_rejects_zero_default_llk_uncertainty():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, use_llk=True)
  for payload in service_payloads(variant, "liveLocationKalmanDEPRECATED"):
    payload.positionGeodetic.std = [0.0, 0.0, 0.0]
    payload.velocityNED.std = [0.0, 0.0, 0.0]

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0, use_llk=True),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0


@pytest.mark.parametrize(
  ("field", "value"),
  [
    ("status", "uncalibrated"),
    ("gpsOK", False),
    ("inputsOK", False),
  ],
)
def test_physical_ab_rejects_llk_with_invalid_health(field, value):
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10, use_llk=True)
  for payload in service_payloads(variant, "liveLocationKalmanDEPRECATED"):
    setattr(payload, field, value)

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0, use_llk=True),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0


def test_physical_ab_rejects_non_external_gps_source():
  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      gps_source="android",
    ),
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0


def test_physical_ab_rejects_zero_default_gps_accuracy():
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  for payload in service_payloads(variant, "gpsLocationExternal"):
    payload.horizontalAccuracy = 0.0
    payload.bearingAccuracyDeg = 0.0
    payload.speedAccuracy = 0.0

  evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    variant,
  )

  assert evidence.status == "missing"
  assert evidence.details["variant_location_fixes"] == 0


def test_physical_ab_requires_same_single_location_measurement_source():
  source_evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    synthetic_physical_route(
      static_offset_m=0.10,
      ego_right_m=0.10,
      gps_source="external",
    ),
  )

  assert source_evidence.status == "missing"
  assert not source_evidence.details["same_single_location_source"]
  assert source_evidence.details["baseline_sample_location_sources"] == ["gpsLocationExternal:ublox"]
  assert source_evidence.details["variant_sample_location_sources"] == ["gpsLocationExternal:external"]

  mixed_variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  service_payloads(mixed_variant, "gpsLocationExternal")[0].source = "trimble"
  mixed_evidence = analyze_physical_ab_events(
    synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0),
    mixed_variant,
  )

  assert mixed_evidence.status == "missing"
  assert len(mixed_evidence.details["variant_sample_location_sources"]) == 2


def test_physical_sample_calibration_must_precede_plan():
  events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  for event in events:
    if event.which() == "liveCalibration":
      event.logMonoTime += 100_000_000

  samples = extract_physical_lane_samples(events, expected_static_offset_m=0.0)

  assert len(samples.samples) == 11
  assert samples.rejections["calibration_missing_or_invalid"] == 1


def test_physical_sample_extraction_rejects_duplicate_model_plan_reuse():
  events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  original_plan = next(event for event in events if event.which() == "lateralPlan")
  duplicate_plan = deepcopy(original_plan)
  duplicate_plan.logMonoTime += 1
  events.append(duplicate_plan)

  samples = extract_physical_lane_samples(events, expected_static_offset_m=0.0)

  assert len(samples.samples) == 12
  assert samples.rejections["model_link_reused"] == 1
  assert len(samples.model_mono_times) == len(set(samples.model_mono_times))


def test_route_analysis_does_not_count_duplicate_model_plans_as_independent_frames():
  events = synthetic_route()
  plans = [event for event in events if event.which() == "lateralPlan"]
  model_time = plans[0].lateralPlan.modelMonoTime
  for index, event in enumerate(plans):
    event.logMonoTime = model_time + 1_000_000 + index
    event.lateralPlan.modelMonoTime = model_time

  report = analyze_events(
    events,
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["lateral_plan_static_offset"].status == "missing"
  assert report.stages["lateral_plan_static_offset"].details["temporal_link_rejections"]["model_link_reused"] == 5


def test_route_analysis_rejects_stale_model_plan_links():
  events = synthetic_route()
  for event in events:
    if event.which() == "lateralPlan":
      event.logMonoTime = event.lateralPlan.modelMonoTime + 500_000_000

  report = analyze_events(
    events,
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["lateral_plan_static_offset"].status == "missing"
  assert report.stages["lateral_plan_static_offset"].details["temporal_link_rejections"]["model_link_stale_or_future"] == 6


def test_route_analysis_does_not_count_invalid_plan_events():
  events = synthetic_route()
  for event in events:
    if event.which() == "lateralPlan":
      event.valid = False

  report = analyze_events(
    events,
    require_code_compatibility=False,
    mpc_replayer=fake_right_mpc_replay,
  )

  assert report.stages["lateral_plan_static_offset"].status == "missing"
  assert report.stages["lateral_plan_static_offset"].sample_count == 0


def test_physical_sample_extraction_rejects_stale_and_reused_location_fixes():
  stale_events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  for event in stale_events:
    if event.which() == "gpsLocationExternal":
      event.logMonoTime += 500_000_000
  stale_samples = extract_physical_lane_samples(stale_events, expected_static_offset_m=0.0)

  assert not stale_samples.samples
  assert stale_samples.rejections["location_match_stale"] == 12

  reused_events = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  first_model = next(event for event in reused_events if event.which() == "modelV2")
  plans = [event for event in reused_events if event.which() == "lateralPlan"]
  second_model_time = first_model.logMonoTime + 50_000_000
  second_original_model_time = plans[1].lateralPlan.modelMonoTime
  second_original_plan_time = plans[1].logMonoTime
  shift = second_original_plan_time - (second_model_time + 1_000_000)
  for event in reused_events:
    if event.which() == "modelV2" and event.logMonoTime == second_original_model_time:
      event.logMonoTime = second_model_time
    elif second_original_plan_time <= event.logMonoTime <= second_original_plan_time + 3_000_000:
      event.logMonoTime -= shift
      if event.which() == "controlsState":
        event.controlsState.lateralPlanMonoTime = second_model_time
  plans[1].lateralPlan.modelMonoTime = second_model_time
  second_gps = next(event for event in reused_events if event.which() == "gpsLocationExternal" and event.logMonoTime == second_original_plan_time - shift)
  reused_events.remove(second_gps)

  reused_samples = extract_physical_lane_samples(reused_events, expected_static_offset_m=0.0)

  assert reused_samples.rejections["location_fix_reused"] == 1
  assert len(reused_samples.location_mono_times) == len(set(reused_samples.location_mono_times))


def test_physical_ab_never_reuses_one_baseline_fix_for_many_variant_samples():
  baseline = synthetic_physical_route(static_offset_m=0.0, ego_right_m=0.0)
  variant = synthetic_physical_route(static_offset_m=0.10, ego_right_m=0.10)
  first_location = next(event.gpsLocationExternal for event in variant if event.which() == "gpsLocationExternal")
  for event in variant:
    if event.which() == "gpsLocationExternal":
      event.gpsLocationExternal.latitude = first_location.latitude
      event.gpsLocationExternal.longitude = first_location.longitude

  evidence = analyze_physical_ab_events(baseline, variant)

  assert evidence.status == "missing"
  assert evidence.details["candidate_sample_pairs"] == 12
  assert evidence.details["matched_sample_pairs"] == 1
  assert evidence.details["unique_baseline_samples_matched"] == 1
  assert evidence.details["unique_variant_samples_matched"] == 1
  assert evidence.details["one_to_one_matching"]


def test_code_compatibility_covers_runtime_lateral_and_hyundai_paths():
  assert {
    "openpilot/selfdrive/controls/lib/drive_helpers.py",
    "openpilot/selfdrive/car/cruise.py",
    "openpilot/selfdrive/car/card.py",
    "opendbc_repo/opendbc/car/hyundai/carcontroller.py",
  }.issubset(CODE_PATHS)


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
