import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.dk_lateral_diagnostics import DkLateralDiagnostics, SAMPLE_NS


CONTROLSD = Path(__file__).parents[1] / "controlsd.py"


class SubMaster(dict):
  frame = 10

  def __init__(self):
    super().__init__({
      "carState": NS(vEgo=8.0, steeringAngleDeg=22.0, steeringRateDeg=18.0, steeringTorque=0.3, steeringPressed=False),
      "carOutput": NS(actuatorsOutput=NS(torque=0.3, torqueOutputCan=81, steeringAngleDeg=0.0)),
      "modelV2": NS(action=NS(desiredCurvature=-0.02)),
      "selfdriveState": NS(active=True),
    })
    self.logMonoTime = dict.fromkeys(self, 900_000_000)
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)


def context():
  pid = PIDController(1.2, 0.3, k_d=0.4, k_f=0.9, pos_limit=1, neg_limit=-1)
  pid.update(0.1, speed=8, feedforward=0.2)
  controls = NS(sm=SubMaster(), lanefull_mode_enabled=False, curvature=0.015,
                desired_curvature=0.011, steer_limited_by_safety=True,
                VM=NS(sR=15.8, cF=90000, cR=95000),
                LaC=NS(pid=pid, torque_params=NS(latAccelFactor=2.8, latAccelOffset=0.1, friction=0.12,
                                               steeringAngleDeadzoneDeg=0.2), lateralTorqueCustom=1,
                       use_steering_angle=True, use_nnff=False, use_nnff_lite=False))
  CC = NS(latActive=True, actuators=NS(torque=-0.2, steeringAngleDeg=-10.0, curvature=0.011))
  return controls, CC


def capture(observer, frame=10, lane_curvature=None, smoothed=0.012, fresh=False):
  observer.capture(frame, 0.015, lane_curvature, smoothed, False, True, fresh, 0.4, 0.3)


def test_records_effective_live_values_without_mutation():
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  before = copy.deepcopy((controls, CC))
  capture(observer)
  observer.emit(controls, CC)
  assert len(records) == 1
  record = records[0]
  assert record["event"] == "dk_lateral_stage"
  assert record["target_source"] == "model"
  assert record["before_smoothing_curvature"] == -0.02
  assert record["effective_smoothing_s"] == 0.1  # Not configured LatSmoothSec=0.4.
  assert record["selected_actuator_delay_s"] == 0.3
  assert record["lane_lag_adjustment_delay_s"] is None
  assert record["smoothed_curvature"] == 0.012
  assert record["clipped_curvature"] == 0.011
  assert record["curvature_changed_by_clip"]
  assert not record["curvature_limited"]  # Existing flag excludes rate/jerk clipping.
  assert record["pid"]["k_p"] == 1.2
  assert record["pid"]["k_i"] == 0.3
  assert record["pid"]["k_d"] == 0.4
  assert record["pid"]["k_f"] == 0.9
  assert record["torque_params"]["latAccelFactor"] == 2.8
  assert record["torque_params"]["friction"] == 0.12
  assert record["mono_ns"] == record["published_mono_ns"] == 1_000_000_000
  assert record["sources"]["carOutput"]["mono_ns"] == 900_000_000
  assert len(json.dumps(record, allow_nan=False)) < 4096
  assert vars(controls.LaC.pid) == vars(before[0].LaC.pid)
  assert vars(controls.LaC.torque_params) == vars(before[0].LaC.torque_params)
  assert vars(CC.actuators) == vars(before[1].actuators)
  assert controls.desired_curvature == before[0].desired_curvature


@pytest.mark.parametrize("active,lane,lane_curvature,expected,tau", [
  (True, True, 0.023, "lane", 0.4),
  (True, True, None, "lane_empty", 0.0),
  (False, True, None, "inactive", 0.0),
  (False, False, None, "inactive", 0.0),
  (True, False, None, "model", 0.1),
])
def test_actual_target_selection(active, lane, lane_curvature, expected, tau):
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  CC.latActive, controls.lanefull_mode_enabled = active, lane
  capture(observer, lane_curvature=lane_curvature, fresh=lane)
  observer.emit(controls, CC)
  assert records[0]["target_source"] == expected
  assert records[0]["effective_smoothing_s"] == tau
  if expected == "lane":
    assert records[0]["before_smoothing_curvature"] == 0.023
    assert records[0]["lane_lag_adjustment_delay_s"] == pytest.approx(0.7)


def test_always_lateral_held_limit_flag_is_not_mislabeled_as_current_rejection():
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  controls.sm["selfdriveState"].active = False
  capture(observer)
  observer.emit(controls, CC)
  assert records[0]["lat_active"]
  assert records[0]["steer_limited_input"]
  assert not records[0]["steer_limit_flag_refreshed"]


def test_cadence_is_bounded_by_frames_and_wall_clock():
  now = [1_000_000_000]
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: now[0])
  controls, CC = context()
  for frame in range(1000):
    controls.sm.frame = frame
    capture(observer, frame)
    observer.emit(controls, CC)
    now[0] += 1_000_000  # Artificial 1000 Hz controller still logs at <=10 Hz.
  assert len(records) == 10
  assert all(b["published_mono_ns"] - a["published_mono_ns"] >= SAMPLE_NS for a, b in zip(records, records[1:], strict=False))
  observer.emit(controls, CC)
  assert len(records) == 10  # A pending sample is consumed exactly once.


def test_source_frame_mismatch_discards_sample():
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  capture(observer, frame=20)
  observer.emit(controls, CC)
  assert not records


def test_nonfinite_lane_target_remains_lane_source_not_empty_plan():
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  controls.lanefull_mode_enabled = True
  capture(observer, lane_curvature=math.nan)
  observer.emit(controls, CC)
  assert records[0]["target_source"] == "lane"
  assert records[0]["before_smoothing_curvature"] is None


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, 1e50, "arbitrary text", [0] * 10000])
def test_nonfinite_unbounded_or_non_numeric_fields_are_missing(bad):
  records = []
  observer = DkLateralDiagnostics(records.append, lambda: 1_000_000_000)
  controls, CC = context()
  controls.LaC.torque_params.friction = bad
  capture(observer, smoothed=bad)
  observer.emit(controls, CC)
  assert records[0]["smoothed_curvature"] is None
  assert records[0]["torque_params"]["friction"] is None
  assert len(json.dumps(records[0], allow_nan=False)) < 4096


def test_broken_clock_logger_or_source_never_escapes():
  def broken(*args):
    raise RuntimeError("injected observer failure")
  controls, CC = context()
  observer = DkLateralDiagnostics(broken, broken)
  capture(observer)
  observer.emit(controls, CC)
  observer.clock = lambda: 1_000_000_000
  capture(observer)
  observer.emit(controls, CC)
  assert observer.last_emit_ns == 1_000_000_000
  observer.clock = lambda: 2_000_000_000
  capture(observer)
  controls.LaC = None
  observer.emit(controls, CC)
  assert observer.pending is None


def method_ast(name):
  cls = next(node for node in ast.parse(CONTROLSD.read_text()).body if isinstance(node, ast.ClassDef) and node.name == "Controls")
  return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)


def diagnostic_statement(node):
  if isinstance(node, ast.Try):
    return "dk_lateral_diagnostics" in ast.unparse(node)
  if isinstance(node, ast.Assign):
    return any(ast.unparse(target) in ("self.dk_lateral_diagnostics", "dk_previous_desired_curvature") for target in node.targets)
  return False


def scc_off_method(node):
  """Exclude only the exact, independently tested SCC startup/target hooks.

  Matching complete statements at their expected locations prevents this
  historical lateral baseline check from masking unrelated control changes.
  The SCC helper's behavior is covered by its own runtime integration tests.
  """
  def signature(statement):
    return ast.dump(statement, include_attributes=False)

  if node.name == "__init__":
    expected = ast.parse("\n".join([
      "self.dk_experimental_scc = None",
      "from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_experiment_enabled",
      "if dk_scc_experiment_enabled(self.params, self.CP):",
      "  from openpilot.selfdrive.controls.lib.dk_experimental_scc import DkExperimentalScc",
      "  self.dk_experimental_scc = DkExperimentalScc()",
    ])).body
    matches = [i for i in range(len(node.body) - len(expected) + 1)
               if [signature(s) for s in node.body[i:i + len(expected)]] == [signature(s) for s in expected]]
    assert len(matches) == 1
    del node.body[matches[0]:matches[0] + len(expected)]
  elif node.name == "publish":
    pcm_blocks = [s for s in node.body if isinstance(s, ast.If) and ast.unparse(s.test) == "self.CP.pcmCruise"]
    assert len(pcm_blocks) == 1
    expected = ast.parse("\n".join([
      "if self.dk_experimental_scc is not None:",
      "  hudControl.setSpeed = float(self.dk_experimental_scc.update(",
      "    self.sm, CC, hudControl.setSpeed, setSpeed, speed_from_pcm, time.monotonic_ns(),",
      "  ))",
    ])).body[0]
    body = pcm_blocks[0].body
    matches = [i for i, s in enumerate(body) if signature(s) == signature(expected)]
    assert len(matches) == 1
    del body[matches[0]]
  return node


def steering_unavailable_method(node):
  """Remove only exact preview hooks to check the out-of-scope legacy path.

  Full preview-enabled methods are separately pinned to 98abba's ON behavior,
  including their position relative to filtering, clipping and publication.
  """
  expected = {
    "__init__": ["7cdacc359e1d3259ec27010fedbd95c7f4dac5cbd4439827db1945044105b076",
                 "22b8015f79559806a46a5daa2104965f31cbb008a7202c4c0ee895e139e7ca8f"],
    "state_control": ["b8b9031c9c0713ed93414db7329b42678b03d7d5df3a4f921aaf66dccc24bf25",
                      "2abc8581d02a90d25462c1cca8a608a9e9eaa5be9e6f478ceecd3ee53d45be83",
                      "dd4dfa4ace457465223b455ff548f78bf7bb33e8ca712bdce7ba35d226ba951f"],
    "publish": ["6e1dd772b0c7b7315a14b777ded3b4fe7fe831c6dce2b5f3f8365779db3ec5eb"],
  }[node.name]
  for digest in expected:
    matches = [i for i, s in enumerate(node.body)
               if hashlib.sha256(ast.dump(s, include_attributes=False).encode()).hexdigest() == digest]
    assert len(matches) == 1
    del node.body[matches[0]]

  class LegacySmoothing(ast.NodeTransformer):
    def visit_Name(self, value):
      if value.id == "smoothing_previous":
        assert isinstance(value.ctx, ast.Load)
        return ast.Attribute(value=ast.Name(id="self", ctx=ast.Load()), attr="desired_curvature", ctx=ast.Load())
      return value

  if node.name == "state_control":
    assert sum(isinstance(n, ast.Name) and n.id == "smoothing_previous" for n in ast.walk(node)) == 3
    node = LegacySmoothing().visit(node)
  return node


@pytest.mark.parametrize("method,original_on_hash", [
  ("__init__", "b8bed37c165f6c5bf777ad47ec4bfde9b0786d8b31d95c0c5a08c4abaca3e31b"),
  ("state_control", "728d43aadb383c9db19cd44a49b89a8128257eb419d1407f979315cae3898369"),
  ("publish", "6c4599468162a6997563cf21d9fdd9087da71c32d38cebf18333917c3c452261"),
])
def test_preview_enabled_control_path_matches_original_98abba_on(method, original_on_hash):
  # Derived from 98abba097d8d09d5edb09ccd8f3fdbabff61d912 with ONLY helper/
  # variable/error-text renames and removal of the old startup Param conjunct.
  # Keep SCC's separately tested hooks; exclude only their exact AST for this comparison.
  node = scc_off_method(method_ast(method))
  assert hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest() == original_on_hash


@pytest.mark.parametrize("method,baseline_hash", [
  ("__init__", "d40c3dfa12df0cc6740c5fe49c5c7688affefb24140c03dff7c27b9569562821"),
  ("state_control", "19e31d33648a56646e9c1550c2842ab98952b9f913b4a488e4f84279c2cace0f"),
  ("publish", "ce8c17ca6491eb9d733bed6878514070f721544c2d9ee385ac81e1f9cf9e23b0"),
])
def test_control_math_order_and_publication_are_identical_to_pre_observer_baseline(method, baseline_hash):
  # Baseline: shipped 6810e4e9bb. Observer, preview-unavailable and exact SCC OFF hooks
  # aside, every existing expression and ordering remains AST equal. Keep
  # these historical hashes unchanged when modifying optional control paths.
  node = method_ast(method)
  node.body = [statement for statement in node.body if not diagnostic_statement(statement)]
  node = scc_off_method(node)
  node = steering_unavailable_method(node)
  digest = hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()
  assert digest == baseline_hash


@pytest.mark.parametrize("enabled,torque,expected", [(False, True, False), (True, False, False), (True, True, True)])
def test_initialization_scope_and_no_extra_params_access(enabled, torque, expected):
  statements = [node for node in method_ast("__init__").body if diagnostic_statement(node)]
  code = compile(ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[])), str(CONTROLSD), "exec")
  self = NS(dk_ka4_stock_scc_resume_gate=enabled, CP=NS(lateralTuning=NS(which=lambda: "torque" if torque else "pid")))
  exec(code, {"self": self, "cloudlog": NS(debug=lambda _: None)})
  assert (self.dk_lateral_diagnostics is not None) is expected


@pytest.mark.parametrize("failure", ["import", "constructor"])
def test_optional_initialization_failure_does_not_escape(monkeypatch, failure):
  import builtins
  import openpilot.selfdrive.controls.lib.dk_lateral_diagnostics as diagnostics_module
  statements = [node for node in method_ast("__init__").body if diagnostic_statement(node)]
  code = compile(ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[])), str(CONTROLSD), "exec")
  self = NS(dk_ka4_stock_scc_resume_gate=True, CP=NS(lateralTuning=NS(which=lambda: "torque")))
  original_import = builtins.__import__

  def broken(*args, **kwargs):
    raise RuntimeError("injected optional observer startup failure")

  def importing(name, *args, **kwargs):
    if name == diagnostics_module.__name__:
      broken()
    return original_import(name, *args, **kwargs)

  if failure == "import":
    monkeypatch.setattr(builtins, "__import__", importing)
  else:
    monkeypatch.setattr(diagnostics_module, "DkLateralDiagnostics", broken)
  exec(code, {"self": self, "cloudlog": NS(debug=lambda _: None)})
  assert self.dk_lateral_diagnostics is None


@pytest.mark.parametrize("hook", ["capture", "emit"])
def test_injected_observer_failure_is_caught_at_control_boundary(hook):
  def broken(*args):
    raise RuntimeError("broken observer")
  method = method_ast("state_control" if hook == "capture" else "publish")
  guarded = next(node for node in method.body if isinstance(node, ast.Try) and "dk_lateral_diagnostics" in ast.unparse(node))
  code = compile(ast.fix_missing_locations(ast.Module(body=[guarded], type_ignores=[])), str(CONTROLSD), "exec")
  controls, CC = context()
  controls.dk_lateral_diagnostics = NS(capture=broken, emit=broken)
  exec(code, {"self": controls, "CC": CC, "dk_previous_desired_curvature": 0.01, "lat_plan": NS(curvatures=[]),
              "new_desired_curvature": 0.0, "curvature_limited": False, "lat_plan_fresh": False,
              "lat_smooth_seconds": 0.4, "steer_actuator_delay": 0.3})


def test_emission_is_after_both_original_publications():
  body = method_ast("publish").body
  sends = [index for index, node in enumerate(body) if isinstance(node, ast.Expr) and "self.pm.send(" in ast.unparse(node)]
  emission = next(index for index, node in enumerate(body) if isinstance(node, ast.Try) and "dk_lateral_diagnostics" in ast.unparse(node))
  assert len(sends) == 2 and max(sends) < emission


@pytest.mark.parametrize("lane_mode", [False, True])
@pytest.mark.parametrize("retired_steering_setting", [None, False, True])
def test_actual_control_and_torque_pid_outputs_identical_with_optional_observer(monkeypatch, lane_mode, retired_steering_setting):
  from openpilot.cereal import car, log
  from opendbc.car.vehicle_model import VehicleModel
  import openpilot.selfdrive.controls.lib.latcontrol_torque as torque_module
  from openpilot.selfdrive.controls.controlsd import Controls

  class FakeParams:
    bool_reads = []

    def get_float(self, key):
      return {"LatSmoothSec": 40.0, "SteerActuatorDelay": 30.0, "SteerRatioRate": 100.0}.get(key, 0.0)

    def get_int(self, key):
      return 0

    def get_bool(self, key):
      self.bool_reads.append(key)
      return retired_steering_setting if key == "DkExperimentalSteering" else False

  monkeypatch.setattr(torque_module, "Params", FakeParams)

  def broken(*args):
    raise RuntimeError("injected observer failure")

  def run(mode, scc_enabled=False):
    records = []
    plan = NS(useLaneLines=lane_mode, mpcSolutionValid=True, modelMonoTime=1_000_000_000,
              curvatures=[0.02] * 17, psis=[0.004 * i for i in range(17)], distances=[i * 0.2 for i in range(17)])
    controls = Controls.__new__(Controls)
    if mode == "legacy_ast":
      from types import MethodType
      import openpilot.selfdrive.controls.controlsd as controls_module
      legacy_node = method_ast("state_control")
      legacy_node.body = [statement for statement in legacy_node.body if not diagnostic_statement(statement)]
      legacy_node = steering_unavailable_method(legacy_node)
      namespace = dict(vars(controls_module))
      exec(compile(ast.fix_missing_locations(ast.Module(body=[legacy_node], type_ignores=[])), str(CONTROLSD), "exec"), namespace)
      controls.state_control = MethodType(namespace["state_control"], controls)
    CP = controls.CP = car.CarParams.new_message()
    controls.params = FakeParams()
    controls.sm = SubMaster()
    CS = car.CarState.new_message()
    CS.gearShifter, CS.latEnabled = car.CarState.GearShifter.drive, True
    model = log.ModelDataV2.new_message()
    model.meta.laneChangeState = log.LaneChangeState.off
    model.meta.laneChangeDirection = log.LaneChangeDirection.none
    controls.sm.update({
      "carState": CS, "carrotMan": NS(vTurnSpeed=0), "lateralPlan": plan,
      "liveDelay": NS(lateralDelay=0.3), "modelV2": model,
      "liveParameters": NS(stiffnessFactor=1.0, steerRatio=15.8, angleOffsetDeg=0.0, roll=0.0),
      "liveTorqueParameters": NS(useParams=False), "longitudinalPlan": NS(),
      "onroadEvents": [], "radarState": NS(), "selfdriveState": NS(enabled=True, active=True),
    })
    controls.sm.all_checks = lambda *args: True
    controls.sm.recv_frame = {"longitudinalPlan": 0}
    controls.sm.logMonoTime["modelV2"] = 1_000_000_000
    controls.LoC = NS(long_control_state=car.CarControl.Actuators.LongControlState.off,
                      reset=lambda: None, update=lambda *args: (0.0, 0.0, 0.0))
    controls.carrot_controls = NS(lat_suspend_control=lambda state, active: active)
    controls.curvature, controls.desired_curvature = 0.0, 0.0
    if mode in ("preview_throws", "preview_nan"):
      controls.dk_steering_preview = NS(legacy_curvature=0.0,
                                      update_from_controls=broken if mode == "preview_throws" else lambda *args: math.nan)
    controls.is_vw_meb, controls.steer_limited_by_safety = False, False
    CP.mass, CP.rotationalInertia = 2400, 4000
    CP.wheelbase, CP.centerToFront, CP.steerRatio = 3.1, 1.2, 15.8
    CP.tireStiffnessFront, CP.tireStiffnessRear = 90000, 95000
    CP.lateralTuning.init("torque")
    for key, value in {"kp": 1.0, "ki": 0.2, "kf": 1.0, "latAccelFactor": 2.8,
                       "latAccelOffset": 0.0, "friction": 0.12, "useSteeringAngle": True}.items():
      setattr(CP.lateralTuning.torque, key, value)
    controls.CI = NS(get_pid_accel_limits=lambda *args: (-3.0, 2.0), use_nnff=False, use_nnff_lite=False,
                     torque_from_lateral_accel=lambda: lambda inputs, params, *args, **kwargs: inputs.lateral_acceleration / params.latAccelFactor)
    controls.VM = VehicleModel(CP)
    controls.LaC = torque_module.LatControlTorque(CP.as_reader(), controls.CI)
    controls.sm["carOutput"] = NS(actuatorsOutput=car.CarControl.Actuators.new_message())
    controls.sm.valid = dict.fromkeys(controls.sm, True)
    controls.sm.alive = dict(controls.sm.valid)
    controls.sm["carState"].vEgo = 8.0
    controls.sm["carState"].steeringAngleDeg = 25.0
    if mode == "preview":
      from openpilot.selfdrive.controls.lib.dk_experimental_scc import DkExperimentalScc
      from openpilot.selfdrive.controls.lib.dk_steering_preview import DkSteeringPreview, SOURCE_MAX_AGE
      from openpilot.selfdrive.controls.tests.test_dk_steering_preview import EXIT, model_for
      controls.dk_experimental_scc = DkExperimentalScc() if scc_enabled else None
      controls.dk_steering_preview = DkSteeringPreview(clock=lambda: controls.sm.frame * 10_000_000)
      controls.dk_steering_preview.legacy_curvature = controls.desired_curvature = .025
      geometry = model_for(EXIT)
      for group in ("position", "orientation", "orientationRate", "velocity"):
        for key, value in vars(getattr(geometry, group)).items():
          setattr(getattr(model, group), key, value)
      controls.sm["carState"].canValid = True
      controls.sm["carState"].steeringAngleDeg = -math.degrees(controls.VM.get_steer_from_curvature(.025, 8., 0.))
      controls.sm["liveParameters"].valid = controls.sm["liveParameters"].sensorValid = True
      controls.sm["livePose"] = NS(inputsOK=True, posenetOK=True, sensorsOK=True, angularVelocityDevice=NS(valid=True))
      controls.calibrated_pose = NS(angular_velocity=NS(yaw=.2))
      controls.pose_calibrator = NS(calib_valid=True)
      controls.sm.valid.update(dict.fromkeys(SOURCE_MAX_AGE, True))
      controls.sm.alive.update(dict.fromkeys(SOURCE_MAX_AGE, True))
    observer = DkLateralDiagnostics(broken if mode == "logger_fails" else records.append,
                                     lambda: controls.sm.frame * 10_000_000)
    controls.dk_lateral_diagnostics = (None if mode in ("disabled", "legacy_ast") else
                                       NS(capture=broken, emit=broken) if mode == "hook_fails" else observer)
    samples = []
    for frame in range(1, 41):
      controls.sm.frame = frame
      controls.sm["modelV2"].action.desiredCurvature = 0.02 if frame < 15 else -0.005
      plan.curvatures = [0.02 if frame < 15 else -0.005] * 17
      if mode == "preview":
        model.action.desiredCurvature = .025
        for service in SOURCE_MAX_AGE:
          if service != "modelV2" or frame % 5 == 1:
            controls.sm.logMonoTime[service] = frame * 10_000_000
        model.frameId = (frame - 1) // 5 + 1
      CC, lateral = controls.state_control()
      if mode in ("preview_throws", "preview_nan"):
        assert controls.dk_steering_preview is None
      if controls.dk_lateral_diagnostics is observer:
        observer.emit(controls, CC)
      if mode == "preview":
        controls.dk_steering_preview.emit(float(CC.actuators.torque), controls.desired_curvature)
      samples.append((CC.to_dict(), lateral.to_dict(), controls.LaC.pid.i, controls.desired_curvature))
    return samples, records

  baseline, _ = run("disabled")
  # Compare the actual control path with the pre-observer method whose
  # complete AST is independently pinned to the historical hash above.
  assert run("legacy_ast")[0] == baseline
  observed, records = run("enabled")
  assert observed == baseline
  assert len(records) == 4  # Actual control path produced stage records, not merely swallowed exceptions.
  assert records[-1]["target_source"] == ("lane" if lane_mode else "model")
  assert run("logger_fails")[0] == baseline
  assert run("hook_fails")[0] == baseline
  assert run("preview_throws")[0] == baseline
  assert run("preview_nan")[0] == baseline
  # Run actual state_control + the real torque PID with an active preview.
  # The SCC helper's presence must not alter any lateral output or PID state.
  preview_off, _ = run("preview", scc_enabled=False)
  preview_on, _ = run("preview", scc_enabled=True)
  assert preview_on == preview_off
  if not lane_mode:
    assert preview_on[-1][-1] < .025  # Exercise a real admitted correction.
  # A stale on-disk value must not be consulted, even when it remains enabled.
  assert "DkExperimentalSteering" not in FakeParams.bool_reads
