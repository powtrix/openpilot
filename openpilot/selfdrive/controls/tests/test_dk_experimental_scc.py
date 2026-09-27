import math
import ast
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, messaging
from openpilot.selfdrive.controls.lib.dk_experimental_scc import DkExperimentalScc, INPUT_SERVICES, MIN_SET_SPEED


class ReplayInputs(dict):
  def all_checks(self, services):
    return all(self.valid[s] for s in services)


def inputs():
  sm = ReplayInputs()
  sm['carState'] = car.CarState.new_message(canValid=True, vEgo=20.0, vCluRatio=1.0, cruiseState={'enabled': True, 'speed': 80 / 3.6})
  sm['longitudinalPlan'] = messaging.new_message('longitudinalPlan').longitudinalPlan
  sm['longitudinalPlan'].speeds = [20 - 10 * i / 16 for i in range(17)]
  sm['longitudinalPlan'].accels = [-1.0] * 17
  sm['longitudinalPlan'].hasLead = True
  sm['longitudinalPlan'].longitudinalPlanSource = 'lead0'
  sm['radarState'] = messaging.new_message('radarState').radarState
  sm['radarState'].leadOne = {'status': True, 'dRel': 25.0, 'vRel': -3.0, 'vLead': 17.0, 'modelProb': 0.99}
  sm['modelV2'] = messaging.new_message('modelV2').modelV2
  sm['modelV2'].leadsV3 = [{'prob': 0.99}]
  sm.valid = dict.fromkeys(INPUT_SERVICES, True)
  sm.now = 10_000_000_000
  advance(sm)
  cc = car.CarControl.new_message(enabled=True)
  return sm, cc


def advance(sm):
  sm.now += 50_000_000
  sm.logMonoTime = dict.fromkeys(INPUT_SERVICES, sm.now - 10000000)
  sm['longitudinalPlan'].modelMonoTime = sm.logMonoTime['modelV2']
  sm['longitudinalPlan'].deprecated.radarStateMonoTime = sm.now - 60_000_000
  sm['radarState'].mdMonoTime = sm.logMonoTime['modelV2']
  sm['modelV2'].timestampEof = sm.now - 30_000_000


def apply(policy, sm, cc, *, target=10.0, baseline=25.0, mode=2):
  return policy.update(sm, cc, baseline, target, mode, sm.now)


def ready():
  sm, cc = inputs()
  policy = DkExperimentalScc()
  assert apply(policy, sm, cc) == 25.0  # engagement frame retains baseline
  advance(sm)
  return policy, sm, cc


def test_valid_moving_lead_lowers_only_existing_target():
  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  assert policy.holding
  assert apply(policy, sm, cc, baseline=9.0) == 9.0
  assert apply(policy, sm, cc, target=0.0) == MIN_SET_SPEED


@pytest.mark.parametrize('service', INPUT_SERVICES)
def test_invalid_or_stale_input_cannot_initiate_reduction(service):
  policy, sm, cc = ready()
  sm.valid[service] = False
  assert apply(policy, sm, cc) == 25.0
  sm.valid[service] = True
  sm.logMonoTime[service] = sm.now - 151_000_000
  assert apply(policy, sm, cc) == 25.0
  assert not policy.holding


@pytest.mark.parametrize(
  'case',
  [
    'model_mismatch',
    'radar_mismatch',
    'stale_plan_radar',
    'future_model',
    'stale_camera',
    'lead1',
    'lead2',
    'cruise',
    'e2e',
    'no_lead',
    'missing_selection',
    'not_closing',
    'bad_distance',
    'low_confidence',
    'bad_model_probability',
    'pedal',
    'brake',
    'hold',
    'park',
    'fault',
    'short_plan',
    'nan_speed',
    'infinite_accel',
    'zero_reset',
    'nondecelerating',
  ],
)
def test_rejects_unqualified_trajectory(case):
  policy, sm, cc = ready()
  plan, radar, model, cs = (sm[n] for n in ('longitudinalPlan', 'radarState', 'modelV2', 'carState'))
  if case == 'model_mismatch':
    plan.modelMonoTime -= 50_000_000
  elif case == 'radar_mismatch':
    radar.mdMonoTime -= 50_000_000
  elif case == 'stale_plan_radar':
    plan.deprecated.radarStateMonoTime = sm.now - 151_000_000
  elif case == 'future_model':
    model.timestampEof = sm.now + 1
  elif case == 'stale_camera':
    model.timestampEof = sm.now - 151_000_000
  elif case in ('lead1', 'lead2', 'cruise', 'e2e'):
    plan.longitudinalPlanSource = case
  elif case == 'no_lead':
    plan.hasLead = False
  elif case == 'missing_selection':
    radar.leadOne.status = False
  elif case == 'not_closing':
    radar.leadOne.vRel = 0.0
  elif case == 'bad_distance':
    radar.leadOne.dRel = math.nan
  elif case == 'low_confidence':
    radar.leadOne.modelProb = 0.49
  elif case == 'bad_model_probability':
    model.leadsV3[0].prob = math.nan
  elif case == 'pedal':
    cs.gasPressed = True
  elif case == 'brake':
    cs.brakePressed = True
  elif case == 'hold':
    cs.brakeHoldActive = True
  elif case == 'park':
    cs.parkingBrake = True
  elif case == 'fault':
    cs.accFaulted = True
  elif case == 'short_plan':
    plan.speeds = [10.0]
  elif case == 'nan_speed':
    plan.speeds = [math.nan] + [10.0] * 16
  elif case == 'infinite_accel':
    plan.accels = [math.inf] + [0.0] * 16
  elif case == 'zero_reset':
    plan.speeds = [0.0] * 17
  elif case == 'nondecelerating':
    plan.speeds = [20.0] * 17
  assert apply(policy, sm, cc) == 25.0
  assert not policy.holding


def test_loss_recovery_and_invalid_data_never_raise_above_oem_after_cap():
  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  sm['carState'].cruiseState.speed = 12.0
  sm['radarState'].leadOne.status = False
  assert apply(policy, sm, cc) == 12.0
  sm['radarState'].leadOne.status = True
  assert apply(policy, sm, cc, target=18.0) == 12.0
  sm['carState'].canValid = False
  sm['carState'].cruiseState.speed = math.nan
  sm.valid['longitudinalPlan'] = False
  assert apply(policy, sm, cc) == 12.0


@pytest.mark.parametrize('button', ['accelCruise', 'decelCruise', 'resumeCruise', 'setCruise'])
def test_manual_adjustment_releases_hold_and_requires_new_model_after_release(button):
  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  sm['carState'].buttonEvents = [{'type': button, 'pressed': True}]
  assert apply(policy, sm, cc) == 25.0
  advance(sm)
  sm['carState'].buttonEvents = []
  assert apply(policy, sm, cc) == 25.0  # physical button is still held
  sm['carState'].buttonEvents = [{'type': button, 'pressed': False}]
  assert apply(policy, sm, cc) == 25.0
  sm['carState'].buttonEvents = []
  assert apply(policy, sm, cc) == 25.0  # stale pre-release plan cannot re-arm
  advance(sm)
  assert apply(policy, sm, cc) == 10.0


def test_disengage_and_reengage_require_a_new_model():
  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  sm['carState'].cruiseState.enabled = False
  assert apply(policy, sm, cc) == 25.0
  sm['carState'].cruiseState.enabled = True
  assert apply(policy, sm, cc) == 25.0
  assert apply(policy, sm, cc) == 25.0
  advance(sm)
  assert apply(policy, sm, cc) == 10.0


@pytest.mark.parametrize('case', ['wheel_stop', 'scc_stop', 'resume', 'cancel', 'disabled', 'other_mode'])
def test_existing_stop_resume_cancel_and_other_modes_pass_through(case):
  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  if case == 'wheel_stop':
    sm['carState'].standstill = True
  elif case == 'scc_stop':
    sm['carState'].cruiseState.standstill = True
  elif case == 'resume':
    cc.cruiseControl.resume = True
  elif case == 'cancel':
    cc.cruiseControl.cancel = True
  elif case == 'disabled':
    cc.enabled = False
  before = cc.to_dict()
  assert apply(policy, sm, cc, mode=0 if case == 'other_mode' else 2) == 25.0
  assert cc.to_dict() == before


def supported_cp():
  from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags

  return car.CarParams.new_message(
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
    pcmCruise=True,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS),
    safetyConfigs=[{'safetyModel': 'hyundaiCanfd', 'safetyParam': int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)}],
  )


@pytest.mark.parametrize(
  'value,expected',
  [
    (None, True),
    (True, True),
    (False, False),
    (b'1', True),
    ('1', True),
    (b'0', False),
    ('0', False),
    (b'', False),
    (b'yes', False),
    (b'2', False),
    (1, False),
  ],
)
def test_startup_selection_defaults_on_but_invalid_or_off_is_not_enabled(value, expected):
  from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_experiment_enabled

  params = SimpleNamespace(get=lambda key: value if key == 'DkExperimentalScc' else b'dkcarrot-wip')
  assert dk_scc_experiment_enabled(params, supported_cp()) == expected


def test_unreadable_selection_fails_closed():
  from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_experiment_enabled

  def fail(key):
    raise OSError('read unavailable')

  assert not dk_scc_experiment_enabled(SimpleNamespace(get=fail), supported_cp())


def test_native_bool_param_selection(tmp_path):
  from openpilot.common.params import Params
  from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_experiment_enabled

  params = Params(str(tmp_path))
  params.put('GitBranch', 'dkcarrot-wip')
  params.remove('DkExperimentalScc')
  assert dk_scc_experiment_enabled(params, supported_cp())
  params.put_bool('DkExperimentalScc', False)
  assert not dk_scc_experiment_enabled(params, supported_cp())
  params.put_bool('DkExperimentalScc', True)
  assert dk_scc_experiment_enabled(params, supported_cp())


@pytest.mark.parametrize('selection', [False, True, None])
def test_controls_initializes_selection_before_interface_once(monkeypatch, selection):
  import openpilot.selfdrive.controls.controlsd as module

  cp = supported_cp()
  cp_bytes = cp.to_bytes()
  reads = []

  def get(key, **kwargs):
    reads.append(key)
    return {'CarParams': cp_bytes, 'GitBranch': 'dkcarrot-wip', 'DkExperimentalScc': selection}[key]

  class InitializationBoundary(Exception):
    pass

  def interface_boundary(_):
    raise InitializationBoundary

  monkeypatch.setattr(module, 'Params', lambda: SimpleNamespace(get=get))
  monkeypatch.setitem(module.interfaces, cp.carFingerprint, interface_boundary)
  controls = module.Controls.__new__(module.Controls)
  with pytest.raises(InitializationBoundary):
    controls.__init__()
  assert (controls.dk_experimental_scc is not None) == (selection is not False)
  assert reads.count('DkExperimentalScc') == 1


def publish_fixture(sm, enabled):
  from openpilot.selfdrive.controls.controlsd import Controls

  controls = Controls.__new__(Controls)
  controls.CP = supported_cp()
  controls.CP.lateralTuning.init('torque')
  controls.sm = sm
  for name in ('selfdriveState', 'carrotMan', 'carOutput', 'driverAssistance', 'driverMonitoringState'):
    sm[name] = getattr(messaging.new_message(name), name)
    sm.valid[name] = True
  sm['carState'].vCruiseCluster = 90.0
  sm['carrotMan'].desiredSpeed = 90
  controls.params = SimpleNamespace(get_int=lambda key: 2 if key == 'SpeedFromPCM' else 0)
  controls.dk_experimental_scc = DkExperimentalScc() if enabled else None
  controls.dk_ka4_stock_scc_resume_gate = True
  controls.curvature = controls.desired_curvature = 0.001
  controls.calibrated_pose = None
  controls.is_vw_meb = controls.lanefull_mode_enabled = False
  controls.dk_lateral_diagnostics = None
  controls.LoC = SimpleNamespace(long_control_state=0, pid=SimpleNamespace(p=0.0, i=0.0, f=0.0))
  controls.pm = SimpleNamespace(send=lambda *args: None)
  return controls


def legacy_publish():
  # The separate lateral regression pins the historical full publish AST.
  # Strip only the independent experiment invocation for a behavioral A/B run.
  import openpilot.selfdrive.controls.controlsd as module

  tree = ast.parse(Path(module.__file__).read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
  node = copy.deepcopy(next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish'))

  class StripExperiment(ast.NodeTransformer):
    def visit_If(self, stmt):
      if ast.unparse(stmt.test) == 'self.dk_experimental_scc is not None':
        return None
      return self.generic_visit(stmt)

  node = StripExperiment().visit(node)
  namespace = dict(vars(module))
  exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(module.__file__), 'exec'), namespace)
  return namespace['publish']


@pytest.mark.parametrize('mode', [0, 1, 2, 3])
def test_off_publish_matches_previous_all_control_fields(monkeypatch, mode):
  import openpilot.selfdrive.controls.controlsd as module

  sm, cc = inputs()
  controls = publish_fixture(sm, False)
  controls.params = SimpleNamespace(get_int=lambda key: mode if key == 'SpeedFromPCM' else 0)
  monkeypatch.setattr(module.time, 'monotonic_ns', lambda: sm.now)
  lac_log = messaging.new_message('controlsState').controlsState.lateralControlState.init('torqueState')
  current, previous = cc.as_reader().as_builder(), cc.as_reader().as_builder()
  controls.publish(current, lac_log)
  legacy_publish()(controls, previous, lac_log)
  assert current.to_dict() == previous.to_dict()


def test_actual_publish_changes_only_set_speed_and_releases_on_driver_input(monkeypatch):
  import openpilot.selfdrive.controls.controlsd as module

  sm, cc = inputs()
  controls = publish_fixture(sm, True)
  monkeypatch.setattr(module.time, 'monotonic_ns', lambda: sm.now)
  lac_log = messaging.new_message('controlsState').controlsState.lateralControlState.init('torqueState')
  controls.publish(cc, lac_log)  # first engaged frame uses baseline
  advance(sm)
  controls.publish(cc, lac_log)
  assert cc.hudControl.setSpeed == 10.0
  baseline = cc.as_reader().as_builder()
  legacy_publish()(controls, baseline, lac_log)
  result = cc.to_dict()
  result['hudControl']['setSpeed'] = baseline.hudControl.setSpeed
  assert result == baseline.to_dict()
  sm['carState'].cruiseState.speed = 12.0
  sm['radarState'].leadOne.status = False
  controls.publish(cc, lac_log)
  assert cc.hudControl.setSpeed == 12.0
  sm['carState'].buttonEvents = [{'type': 'accelCruise', 'pressed': True}]
  controls.publish(cc, lac_log)
  assert cc.hudControl.setSpeed == 25.0
  sm['carState'].buttonEvents = [{'type': 'accelCruise', 'pressed': False}]
  controls.publish(cc, lac_log)
  sm['carState'].buttonEvents = []
  sm['radarState'].leadOne.status = True
  controls.publish(cc, lac_log)
  assert cc.hudControl.setSpeed == 25.0  # pre-release source is blocked
  advance(sm)
  controls.publish(cc, lac_log)
  assert cc.hudControl.setSpeed == 10.0


def test_stale_controlsd_hold_cannot_increase_newer_card_oem_speed():
  from opendbc.car.hyundai.tests.test_stock_scc_resume import build_controller, build_state
  from opendbc.car.hyundai.values import Buttons

  policy, sm, cc = ready()
  assert apply(policy, sm, cc) == 10.0
  sm['carState'].cruiseState.speed = 50 / 3.6
  sm['radarState'].leadOne.status = False
  apply(policy, sm, cc)
  sm.now += 160_000_000
  cc.hudControl.setSpeed = apply(policy, sm, cc)
  assert cc.hudControl.setSpeed == pytest.approx(50 / 3.6)
  card_cs = build_state(standstill=False, v_ego=15.0, v_ego_raw=15.0)
  card_cs.out.cruiseState.speed = 49 / 3.6
  controller = build_controller()
  controller.speed_from_pcm = 2
  controller.frame, controller.prev_clu_speed = 100, 49
  controller.dk_experimental_scc = False
  assert controller.make_spam_button(cc, card_cs, stock_scc_source_fresh=True) == Buttons.RES_ACCEL
  controller.dk_experimental_scc = True
  assert controller.make_spam_button(cc, card_cs, stock_scc_source_fresh=True) == Buttons.NONE
