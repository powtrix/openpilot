import ast
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.dk_experimental_steering import (
  DkExperimentalSteering, MAX_EXTRA_ACCEL, MAX_EXTRA_CURVATURE, MAX_EXTRA_JERK,
  PreviewPath, PreviewState, SOURCE_MAX_AGE, TIMES, _prediction_errors, lateral_rotation_radius,
  model_path, path_phase, preview_candidate,
)
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature


def geometry(fn, speed=8., duration=4.):
  times = np.arange(0., duration + .0005, .001)
  curvature = fn(times)
  heading = np.cumsum(curvature) * speed * .001
  heading -= heading[0]
  x = np.cumsum(np.cos(heading)) * speed * .001
  y = np.cumsum(np.sin(heading)) * speed * .001
  x -= x[0]
  y -= y[0]
  return times, x, y, heading, curvature


def reference(fn, speed=8.):
  t, x, y, heading, curvature = geometry(fn, speed)
  return PreviewPath(*(np.interp(TIMES, t, a) for a in (x, y, heading, curvature)))


def model_for(fn, speed=8., action=.025):
  t, x, y, heading, curvature = geometry(fn, speed)
  model_times = np.linspace(0., 4., 33)
  def interp(a):
    return np.interp(model_times, t, a).tolist()
  return NS(position=NS(x=interp(x), y=interp(y)), orientation=NS(z=interp(heading)),
            orientationRate=NS(z=interp(curvature * speed)),
            velocity=NS(x=[speed] * 33, y=[0.] * 33, z=[0.] * 33),
            meta=NS(laneChangeState="off"), frameId=1, action=NS(desiredCurvature=action))


def ENTRY(t):
  return .001 + .025 * np.clip((t - .2) / 1., 0., 1.)


def EXIT(t):
  return .025 * np.clip(1. - (t - .2) / 1.4, 0., 1.)


def HOLD(t):
  return np.full_like(t, .025)


@pytest.mark.parametrize("direction", [-1., 1.])
def test_autonomous_entry_and_exit_have_no_driver_trigger(direction):
  entry = preview_candidate(reference(lambda t: direction * ENTRY(t)),
                            PreviewState(8., direction * .001, direction * .001, .3), direction * .003)
  assert entry.phase == "entry" and entry.accepted
  assert abs(entry.target) > .003
  exit_result = preview_candidate(reference(lambda t: direction * EXIT(t)),
                                 PreviewState(8., direction * .025, direction * .025, .3), direction * .025)
  assert exit_result.phase == "exit" and exit_result.accepted
  assert 0. < abs(exit_result.target) < .025
  for result in (entry, exit_result):
    assert result.candidate_cost < result.baseline_cost
    assert result.candidate_max_error <= result.baseline_max_error + .01


@pytest.mark.parametrize("speed", [4., 6., 8., 12., 16.7])
@pytest.mark.parametrize("direction", [-1., 1.])
def test_sustained_curve_has_no_premature_unwind(speed, direction):
  curve = direction * .01
  result = preview_candidate(reference(lambda t: np.full_like(t, curve), speed),
                             PreviewState(speed, curve, curve, .3), curve)
  assert result.phase == "hold"
  assert result.target == curve
  assert not result.accepted


def test_s_bend_and_straight_do_not_activate():
  for fn, phase in ((lambda t: np.zeros_like(t), "straight"),
                    (lambda t: .02 * np.cos(t * math.pi), "changing_direction")):
    path = reference(fn)
    result = preview_candidate(path, PreviewState(8., 0., 0., .3), 0.)
    assert result.phase == phase
    assert not result.accepted


@pytest.mark.parametrize("field,value", [
  ("speed", 0.), ("speed", 3.99), ("speed", 16.71), ("speed", float("nan")),
  ("measured_curvature", float("inf")), ("steering_curvature", .5),
  ("actuator_delay", .01), ("actuator_delay", .7), ("output_torque", float("nan")),
  ("rotation_radius", float("nan")), ("rotation_radius", -1.), ("rotation_radius", 5.01),
])
def test_invalid_state_cannot_change_target(field, value):
  values = dict(speed=8., measured_curvature=.025, steering_curvature=.025, actuator_delay=.3)
  values[field] = value
  result = preview_candidate(reference(EXIT), PreviewState(**values), .025)
  assert not result.accepted and result.target == .025


def test_model_geometry_uses_distance_and_model_speed_not_future_motion_time():
  model = model_for(EXIT)
  result = model_path(model, 8.)
  assert path_phase(result, .025) == "exit"
  # Halve velocity and yaw together: same geometric curvature, not half the
  # target just because a model predicts a speed stock SCC may not execute.
  model.velocity.x = [4.] * 33
  model.orientationRate.z = [v * .5 for v in model.orientationRate.z]
  slower = model_path(model, 8.)
  np.testing.assert_array_equal(slower.curvature, result.curvature)
  np.testing.assert_array_equal(slower.y, result.y)


@pytest.mark.parametrize("mutate", [
  lambda m: setattr(m.position, "x", [0.] * 32),
  lambda m: m.position.y.__setitem__(5, float("nan")),
  lambda m: setattr(m.position, "x", [0.] * 33),
  lambda m: setattr(m.orientation, "z", [1.] * 33),
  lambda m: setattr(m.orientationRate, "z", [10.] * 33),
])
def test_invalid_geometry_rejected(mutate):
  model = model_for(EXIT)
  mutate(model)
  with pytest.raises(ValueError):
    model_path(model, 8.)


class FakeSM(dict):
  pass


def adapter_fixture():
  now = [10_000_000_000]
  model = model_for(EXIT)
  sm = FakeSM({"carState": NS(vEgo=8., canValid=True, steeringPressed=False, canTimeout=False,
                              steerFaultTemporary=False, steerFaultPermanent=False, standstill=False, gearShifter="drive"),
               "modelV2": model, "livePose": NS(inputsOK=True, posenetOK=True, sensorsOK=True,
                                                angularVelocityDevice=NS(valid=True)),
               "carOutput": NS(actuatorsOutput=NS(torque=0.)), "liveParameters": NS(roll=0., valid=True, sensorValid=True)})
  sm.valid = dict.fromkeys(SOURCE_MAX_AGE, True)
  sm.alive = dict(sm.valid)
  sm.logMonoTime = dict.fromkeys(SOURCE_MAX_AGE, now[0])
  controls = NS(sm=sm, curvature=.025, lanefull_mode_enabled=False,
                CP=NS(wheelbase=3.09, centerToFront=1.236, mass=2223., tireStiffnessRear=307905.25),
                calibrated_pose=NS(angular_velocity=NS(yaw=.2)), pose_calibrator=NS(calib_valid=True))
  records = []
  helper = DkExperimentalSteering(records.append, lambda: now[0])
  helper.legacy_curvature = .025
  cc = NS(latActive=True)
  return helper, controls, cc, now, records


def tick(helper, controls, cc, now, new_model=False, baseline=.025):
  now[0] += 10_000_000
  for name in SOURCE_MAX_AGE:
    if name != "modelV2":
      controls.sm.logMonoTime[name] = now[0]
  if new_model:
    controls.sm["modelV2"].frameId += 1
    controls.sm.logMonoTime["modelV2"] = now[0]
  return helper.update_from_controls(controls, cc, baseline, .3)


def test_three_unique_model_frames_required_not_three_controls_ticks():
  helper, controls, cc, now, _ = adapter_fixture()
  for _ in range(5):
    assert tick(helper, controls, cc, now) == .025
  assert helper.phase_frames == 1
  assert tick(helper, controls, cc, now, True) == .025
  for _ in range(4):
    assert tick(helper, controls, cc, now) == .025
  assert tick(helper, controls, cc, now, True) < .025


def test_shadow_baseline_never_accumulates_preview_delta_and_withdrawal_is_smooth():
  helper, controls, cc, now, _ = adapter_fixture()
  values = []
  for i in range(400):
    values.append(tick(helper, controls, cc, now, i % 5 == 0))
    assert helper.legacy_curvature == .025
    assert abs(values[-1] - .025) <= min(MAX_EXTRA_CURVATURE, MAX_EXTRA_ACCEL / 64) + 1e-12
  assert values[-1] < .025
  controls.lanefull_mode_enabled = True
  for i in range(100):
    values.append(tick(helper, controls, cc, now, i % 5 == 0))
  assert values[-1] == .025
  assert max(abs(a - b) for a, b in zip(values, values[1:], strict=False)) <= MAX_EXTRA_JERK * .01 / 64 + 1e-12


@pytest.mark.parametrize("source", list(SOURCE_MAX_AGE))
def test_invalid_or_stale_sources_withdraw(source):
  helper, controls, cc, now, _ = adapter_fixture()
  for i in range(60):
    tick(helper, controls, cc, now, i % 5 == 0)
  assert helper.delta < 0.
  controls.sm.valid[source] = False
  helper.update_from_controls(controls, cc, .025, .3)
  assert helper.target_delta == 0. and helper.record["reason"] == "stale_source"


def test_driver_is_withdrawal_only_and_disengagement_clears_state():
  helper, controls, cc, now, _ = adapter_fixture()
  for i in range(60):
    tick(helper, controls, cc, now, i % 5 == 0)
  controls.sm["carState"].steeringPressed = True
  tick(helper, controls, cc, now, True)
  assert helper.target_delta == 0. and helper.record["reason"] == "driver_override"
  cc.latActive = False
  expected = clip_curvature(8., helper.legacy_curvature, .01, 0.)[0]
  assert tick(helper, controls, cc, now, baseline=.01) == expected
  assert helper.delta == 0. and helper.legacy_curvature == expected


def test_frame_reset_rewarms_and_logs_are_json_safe_bounded_and_local():
  helper, controls, cc, now, records = adapter_fixture()
  for i in range(60):
    tick(helper, controls, cc, now, i % 5 == 0)
    helper.emit(-.5)
  assert len(records) <= 7
  assert records and all(json.dumps(row) for row in records)
  controls.sm["modelV2"].frameId = 0
  tick(helper, controls, cc, now, True)
  assert helper.phase_frames == 1 and helper.target_delta == 0.
  helper.logger = lambda _: (_ for _ in ()).throw(RuntimeError("logger"))
  now[0] += 200_000_000
  helper.emit(-.5)


def test_production_opt_in_gate_is_startup_only_and_existing_limits_remain_after_candidate():
  source = (Path(__file__).parents[1] / "controlsd.py").read_text()
  tree = ast.parse(source)
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Controls")
  methods = {node.name: node for node in cls.body if isinstance(node, ast.FunctionDef)}
  init = ast.unparse(methods["__init__"])
  state = ast.unparse(methods["state_control"])
  assert init.count("get_bool('DkExperimentalSteering')") == 1
  assert "DkExperimentalSteering" not in state
  gate = next(node for node in ast.walk(methods["__init__"]) if isinstance(node, ast.If) and
              "get_bool('DkExperimentalSteering')" in ast.unparse(node.test))
  assert "dk_ka4_stock_scc_resume_gate" in ast.unparse(gate.test)
  assert "SteerControlType.torque" in ast.unparse(gate.test)
  assert "lateralTuning.which() == 'torque'" in ast.unparse(gate.test)
  assert "from openpilot.selfdrive.controls.lib.dk_experimental_steering import" in ast.unparse(gate)
  assert state.index("update_from_controls") < state.index("self.desired_curvature, curvature_limited = clip_curvature") < state.index("self.LaC.update(")
  assert "Params" not in inspect.getsource(preview_candidate)


def test_final_curvature_safety_limits_still_bound_proposed_candidate():
  path = reference(ENTRY)
  state = PreviewState(8., .001, .001, .3)
  proposal = preview_candidate(path, state, .003)
  final, _ = clip_curvature(state.speed, .003, proposal.target, 0.)
  assert abs(final - .003) <= 5. * .01 / state.speed ** 2 + 1e-12


@pytest.mark.parametrize("scope,torque,enabled,angle_flag,expected", [
  (False, True, True, False, False), (True, False, True, False, False),
  (True, True, False, False, False), (True, True, True, False, True),
  (True, True, True, True, False),
])
def test_startup_gate_never_imports_or_constructs_when_off(monkeypatch, scope, torque, enabled, angle_flag, expected):
  import builtins
  from openpilot.cereal import car
  from opendbc.car.hyundai.values import HyundaiFlags
  source = (Path(__file__).parents[1] / "controlsd.py").read_text()
  cls = next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef) and node.name == "Controls")
  init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  statements = [node for node in init.body if "dk_experimental_steering" in ast.unparse(node)]
  calls = []
  original_import = builtins.__import__

  def importing(name, *args, **kwargs):
    if name.endswith("dk_experimental_steering"):
      calls.append(name)
    return original_import(name, *args, **kwargs)

  monkeypatch.setattr(builtins, "__import__", importing)
  self = NS(dk_ka4_stock_scc_resume_gate=scope,
            CP=NS(steerControlType=car.CarParams.SteerControlType.torque,
                  flags=HyundaiFlags.ANGLE_CONTROL if angle_flag else 0,
                  lateralTuning=NS(which=lambda: "torque" if torque else "pid")),
            params=NS(get_bool=lambda _: enabled))
  exec(compile(ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[])), "startup", "exec"),
       {"self": self, "car": car, "HyundaiFlags": HyundaiFlags,
        "cloudlog": NS(debug=lambda _: None, exception=lambda _: None)})
  assert bool(calls) == expected
  assert (self.dk_experimental_steering is not None) == expected


def test_unbuilt_native_param_key_does_not_stop_control_startup():
  from openpilot.cereal import car
  from opendbc.car.hyundai.values import HyundaiFlags
  source = (Path(__file__).parents[1] / "controlsd.py").read_text()
  cls = next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef) and node.name == "Controls")
  init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  statements = [node for node in init.body if "dk_experimental_steering" in ast.unparse(node)]
  self = NS(dk_ka4_stock_scc_resume_gate=True,
            CP=NS(steerControlType=car.CarParams.SteerControlType.torque, flags=0, lateralTuning=NS(which=lambda: "torque")),
            params=NS(get_bool=lambda _: (_ for _ in ()).throw(RuntimeError("UnknownKeyName"))))
  errors = []
  exec(compile(ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[])), "startup", "exec"),
       {"self": self, "car": car, "HyundaiFlags": HyundaiFlags,
        "cloudlog": NS(debug=lambda _: None, exception=errors.append)})
  assert self.dk_experimental_steering is None and len(errors) == 1


@pytest.mark.parametrize("direction", [-1., 1.])
def test_residual_correction_cannot_reverse_a_crossing_legacy_target(direction):
  helper, controls, cc, now, _ = adapter_fixture()
  controls.lanefull_mode_enabled = True
  helper.legacy_curvature = -direction * .0001
  helper.delta = direction * .004
  helper.target_delta = direction * .004
  result = tick(helper, controls, cc, now, baseline=-direction * .0001)
  assert result == 0.
  assert helper.legacy_curvature * result >= 0.
  result = tick(helper, controls, cc, now, baseline=0.)
  assert result == 0. and helper.delta == 0.


def test_actual_cereal_lane_change_enum_and_vehicle_types_are_supported():
  from openpilot.cereal import car, log
  helper, controls, cc, now, _ = adapter_fixture()
  model = controls.sm["modelV2"]
  # The native enum intentionally cannot be cast with int().
  model.meta.laneChangeState = log.ModelDataV2.new_message().meta.laneChangeState
  cs = car.CarState.new_message()
  cs.vEgo, cs.canValid, cs.gearShifter = 8., True, car.CarState.GearShifter.drive
  controls.sm["carState"] = cs
  for i in range(20):
    tick(helper, controls, cc, now, i % 5 == 0)
  assert helper.delta < 0. and helper.record["accepted"]


@pytest.mark.parametrize("failure", ["clock", "missing_speed", "nonfinite_speed"])
def test_adapter_input_failure_returns_legacy_without_throwing(failure):
  helper, controls, cc, now, _ = adapter_fixture()
  if failure == "clock":
    helper.clock = lambda: (_ for _ in ()).throw(RuntimeError("clock"))
  elif failure == "missing_speed":
    del controls.sm["carState"].vEgo
  else:
    controls.sm["carState"].vEgo = float("nan")
  assert helper.update_from_controls(controls, cc, .025, .3) == .025
  assert helper.delta == 0.


@pytest.mark.parametrize("speed", [4., 8., 16.7])
def test_rotation_radius_matches_existing_lateral_planner(speed):
  _, controls, _, _, _ = adapter_fixture()
  cp = controls.CP
  expected = max(0., cp.wheelbase - cp.centerToFront -
                 cp.centerToFront * cp.mass / (cp.wheelbase * cp.tireStiffnessRear) * speed ** 2)
  assert lateral_rotation_radius(cp, speed) == expected


@pytest.mark.parametrize("field,value", [("wheelbase", 0.), ("centerToFront", 4.), ("mass", float("nan")),
                                         ("tireStiffnessRear", 0.)])
def test_bad_vehicle_geometry_withdraws_optional_preview(field, value):
  helper, controls, cc, now, _ = adapter_fixture()
  setattr(controls.CP, field, value)
  for i in range(20):
    assert tick(helper, controls, cc, now, i % 5 == 0) == .025
  assert helper.record["reason"] == "invalid_input_or_solver"


@pytest.mark.parametrize("direction", [-1., 1.])
def test_constant_turn_prediction_includes_lateral_velocity_not_just_heading(direction):
  # Analytic solution of the existing MPC's constant-speed, constant-yaw
  # equations. These are kinematic consistency checks, not a KA4 torque plant.
  speed, curve, radius = 8., direction * .025, 1.6
  heading = speed * curve * TIMES
  path = PreviewPath(np.sin(heading) / curve + radius * (np.cos(heading) - 1.),
                     (1. - np.cos(heading)) / curve + radius * np.sin(heading),
                     heading, np.full_like(TIMES, curve))
  state = PreviewState(speed, curve, curve, .3, rotation_radius=radius)
  errors, headings = _prediction_errors(path, state, np.array([curve]), .3, .24)
  assert np.max(np.abs(errors)) < .001
  np.testing.assert_allclose(headings, 0., atol=1e-12)
  wrong_state = PreviewState(speed, curve, curve, .3)
  wrong_errors, _ = _prediction_errors(path, wrong_state, np.array([curve]), .3, .24)
  assert np.max(np.abs(wrong_errors)) > .5


def test_adapter_logs_actual_vehicle_rotation_radius():
  helper, controls, cc, now, _ = adapter_fixture()
  tick(helper, controls, cc, now, True)
  assert helper.record["rotation_radius_m"] == lateral_rotation_radius(controls.CP, 8.)


@pytest.mark.parametrize("direction", [-1., 1.])
def test_model_geometry_does_not_confuse_body_heading_with_path_tangent(direction):
  speed, curve, radius = 4., direction * .115, 1.8
  t = np.linspace(0., 4., 33)
  heading = speed * curve * t
  yaw = speed * curve
  model = NS(position=NS(x=(np.sin(heading) / curve + radius * (np.cos(heading) - 1.)).tolist(),
                         y=((1. - np.cos(heading)) / curve + radius * np.sin(heading)).tolist()),
             orientation=NS(z=heading.tolist()), orientationRate=NS(z=[yaw] * 33),
             velocity=NS(x=(speed * np.cos(heading) - radius * yaw * np.sin(heading)).tolist(),
                         y=(speed * np.sin(heading) + radius * yaw * np.cos(heading)).tolist(), z=[0.] * 33))
  # The old zero-lateral-velocity guard incorrectly rejects this valid curve.
  with pytest.raises(ValueError, match="inconsistent_geometry"):
    model_path(model, speed)
  path = model_path(model, speed, radius)
  assert path_phase(path, curve) == "hold"
  np.testing.assert_allclose(path.curvature, yaw / math.hypot(speed, radius * yaw))


@pytest.mark.parametrize("where,field", [("carState", "canTimeout"), ("carState", "steerFaultTemporary"),
                                          ("carState", "steerFaultPermanent"), ("carState", "standstill"),
                                          ("livePose", "inputsOK"), ("livePose", "posenetOK"), ("livePose", "sensorsOK"),
                                          ("liveParameters", "valid"), ("liveParameters", "sensorValid")])
def test_vehicle_and_pose_invalidity_prevent_experimental_target(where, field):
  helper, controls, cc, now, _ = adapter_fixture()
  setattr(controls.sm[where], field, where == "carState")
  for i in range(20):
    assert tick(helper, controls, cc, now, i % 5 == 0) == .025
  assert helper.record["reason"] == "invalid_vehicle_state"


def test_legacy_shadow_matches_independent_baseline_across_disengage_and_reengage():
  helper, controls, cc, now, _ = adapter_fixture()
  cc.latActive = False
  expected = clip_curvature(8., helper.legacy_curvature, .004, 0.)[0]
  tick(helper, controls, cc, now, baseline=.004)
  assert helper.legacy_curvature == expected
  cc.latActive = True
  for i in range(200):
    action = .025 if i < 100 else .015
    baseline = (1. - math.exp(-.01 / .1)) * action + math.exp(-.01 / .1) * helper.legacy_curvature
    expected_baseline = (1. - math.exp(-.01 / .1)) * action + math.exp(-.01 / .1) * expected
    expected = clip_curvature(8., expected, expected_baseline, 0.)[0]
    target = tick(helper, controls, cc, now, i % 5 == 0, baseline)
    assert helper.legacy_curvature == expected
    assert abs(target - expected) <= min(MAX_EXTRA_CURVATURE, MAX_EXTRA_ACCEL / 64) + 1e-12


@pytest.mark.parametrize("bad_sensor", ["sensorsOK", "angularVelocityDevice"])
def test_native_pose_message_valid_does_not_hide_invalid_sensor_input(bad_sensor):
  from openpilot.cereal import log
  helper, controls, cc, now, _ = adapter_fixture()
  pose = log.LivePose.new_message()
  pose.inputsOK, pose.posenetOK, pose.sensorsOK = True, True, True
  pose.angularVelocityDevice.valid = True
  if bad_sensor == "sensorsOK":
    pose.sensorsOK = False
  else:
    pose.angularVelocityDevice.valid = False
  controls.sm["livePose"] = pose
  controls.sm.valid["livePose"] = True
  for i in range(20):
    assert tick(helper, controls, cc, now, i % 5 == 0) == .025
  assert helper.record["reason"] == "invalid_vehicle_state"


def test_post_clip_target_remains_bounded_and_returns_to_legacy_on_source_loss():
  helper, controls, cc, now, records = adapter_fixture()
  actual = .025
  limit = min(MAX_EXTRA_CURVATURE, MAX_EXTRA_ACCEL / 64)
  values = []
  for i in range(240):
    if i == 100:
      controls.sm.valid["modelV2"] = False
    proposal = tick(helper, controls, cc, now, i % 5 == 0)
    actual, _ = clip_curvature(8., actual, proposal, 0.)
    values.append(actual)
    assert abs(actual - helper.legacy_curvature) <= limit + 1e-12
    helper.emit(-.5, actual)
  assert min(values) < .025
  assert actual == helper.legacy_curvature == .025
  assert records[-1]["published_curvature"] == actual
  assert records[-1]["published_delta"] == 0.


@pytest.mark.parametrize("direction", [-1., 1.])
def test_entry_assistance_is_not_carried_into_detected_exit(direction):
  helper, controls, cc, now, _ = adapter_fixture()
  controls.sm["modelV2"] = model_for(lambda t: direction * EXIT(t))
  controls.curvature = helper.legacy_curvature = direction * .025
  controls.calibrated_pose.angular_velocity.yaw = direction * .2
  helper.delta = direction * .004  # Already established entry/hold assistance.
  helper.phase, helper.phase_frames = "hold", 8
  actual_previous = helper.legacy_curvature + helper.delta
  proposal = tick(helper, controls, cc, now, True, baseline=direction * .025)
  assert helper.record["phase"] == "exit" and helper.phase_frames == 1
  assert proposal == helper.legacy_curvature and helper.delta == 0.
  assert helper.phase_projection_count == 1
  # Existing control-level safety, not the experiment, bounds actual release.
  applied, _ = clip_curvature(8., actual_previous, proposal, 0.)
  assert abs(applied - actual_previous) <= 5. * .01 / 64 + 1e-12


@pytest.mark.parametrize("fn", [HOLD, lambda t: np.zeros_like(t), lambda t: .02 * np.cos(t * math.pi)])
def test_exit_reduction_not_carried_into_hold_straight_or_reversing_curve(fn):
  helper, controls, cc, now, _ = adapter_fixture()
  controls.sm["modelV2"] = model_for(fn)
  helper.delta = -.004
  helper.phase, helper.phase_frames = "exit", 8
  proposal = tick(helper, controls, cc, now, True)
  assert helper.record["phase"] in ("hold", "straight", "changing_direction")
  assert proposal == helper.legacy_curvature
  assert helper.delta == 0.
