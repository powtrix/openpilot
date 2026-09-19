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


def experimental_off_method(node):
  """Evaluate only the explicit opt-in hooks with the startup option OFF.

  Retain the pre-existing baseline hashes below: the new option must not turn
  a test of legacy equivalence into a freshly blessed changed baseline.
  """
  class OffPrevious(ast.NodeTransformer):
    def visit_Name(self, name):
      if name.id == "smoothing_previous":
        return ast.copy_location(ast.Attribute(value=ast.Name(id="self", ctx=ast.Load()),
                                               attr="desired_curvature", ctx=ast.Load()), name)
      return name

  retained = []
  for statement in node.body:
    if isinstance(statement, ast.Try) and "dk_experimental_steering" in ast.unparse(statement):
      continue
    if isinstance(statement, ast.Assign) and any(ast.unparse(target) in (
        "self.dk_experimental_steering", "dk_experimental", "smoothing_previous") for target in statement.targets):
      if any(ast.unparse(target) == "smoothing_previous" for target in statement.targets):
        assert isinstance(statement.value, ast.IfExp)
        assert ast.unparse(statement.value.orelse) == "self.desired_curvature"
      continue
    if isinstance(statement, ast.If) and any(token in ast.unparse(statement.test) for token in (
        "get_bool('DkExperimentalSteering')", "dk_experimental is not None", "'dk_experimental_steering'")):
      assert not statement.orelse
      continue
    retained.append(statement)
  node.body = retained
  return OffPrevious().visit(node)


@pytest.mark.parametrize("method,baseline_hash", [
  ("__init__", "d40c3dfa12df0cc6740c5fe49c5c7688affefb24140c03dff7c27b9569562821"),
  ("state_control", "19e31d33648a56646e9c1550c2842ab98952b9f913b4a488e4f84279c2cace0f"),
  ("publish", "ce8c17ca6491eb9d733bed6878514070f721544c2d9ee385ac81e1f9cf9e23b0"),
])
def test_control_math_order_and_publication_are_identical_to_pre_observer_baseline(method, baseline_hash):
  # Baseline: shipped 6810e4e9bb. With the explicit experiment OFF, removing observer hooks must
  # leave every existing control expression and ordering byte-for-byte AST equal.
  node = method_ast(method)
  node.body = [statement for statement in node.body if not diagnostic_statement(statement)]
  node = experimental_off_method(node)
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
def test_actual_control_and_torque_pid_outputs_identical_with_optional_observer(monkeypatch, lane_mode):
  from openpilot.cereal import car, log
  from opendbc.car.vehicle_model import VehicleModel
  import openpilot.selfdrive.controls.lib.latcontrol_torque as torque_module
  from openpilot.selfdrive.controls.controlsd import Controls

  class FakeParams:
    def get_float(self, key):
      return {"LatSmoothSec": 40.0, "SteerActuatorDelay": 30.0, "SteerRatioRate": 100.0}.get(key, 0.0)

    def get_int(self, key):
      return 0

    def get_bool(self, key):
      return False

  monkeypatch.setattr(torque_module, "Params", FakeParams)

  def broken(*args):
    raise RuntimeError("injected observer failure")

  def run(mode):
    records = []
    plan = NS(useLaneLines=lane_mode, mpcSolutionValid=True, modelMonoTime=1_000_000_000,
              curvatures=[0.02] * 17, psis=[0.004 * i for i in range(17)], distances=[i * 0.2 for i in range(17)])
    controls = Controls.__new__(Controls)
    if mode in ("experiment_throws", "experiment_nan"):
      def experimental_failure(*args):
        if mode == "experiment_throws":
          raise RuntimeError("injected experimental control failure")
        return float("nan")
      controls.dk_experimental_steering = NS(legacy_curvature=0.0, update_from_controls=experimental_failure)
    if mode == "legacy_ast":
      from types import MethodType
      import openpilot.selfdrive.controls.controlsd as controls_module
      legacy_node = experimental_off_method(method_ast("state_control"))
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
    observer = DkLateralDiagnostics(broken if mode == "logger_fails" else records.append,
                                     lambda: controls.sm.frame * 10_000_000)
    controls.dk_lateral_diagnostics = (None if mode in ("disabled", "legacy_ast") else
                                       NS(capture=broken, emit=broken) if mode == "hook_fails" else observer)
    samples = []
    for frame in range(1, 41):
      controls.sm.frame = frame
      controls.sm["modelV2"].action.desiredCurvature = 0.02 if frame < 15 else -0.005
      plan.curvatures = [0.02 if frame < 15 else -0.005] * 17
      CC, lateral = controls.state_control()
      if controls.dk_lateral_diagnostics is observer:
        observer.emit(controls, CC)
      samples.append((CC.to_dict(), lateral.to_dict(), controls.LaC.pid.i, controls.desired_curvature))
    if mode in ("experiment_throws", "experiment_nan"):
      assert controls.dk_experimental_steering is None
    return samples, records

  baseline, _ = run("disabled")
  # Compare the actual OFF code path with the pre-experiment method whose
  # complete AST is independently pinned to the historical hash above.
  assert run("legacy_ast")[0] == baseline
  observed, records = run("enabled")
  assert observed == baseline
  assert len(records) == 4  # Actual control path produced stage records, not merely swallowed exceptions.
  assert records[-1]["target_source"] == ("lane" if lane_mode else "model")
  assert run("logger_fails")[0] == baseline
  assert run("hook_fails")[0] == baseline
  assert run("experiment_throws")[0] == baseline
  assert run("experiment_nan")[0] == baseline
