import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.carrot.dk_vehicle_diagnostics import (
  DkVehicleDiagnostics, MAX_TRANSITIONS, PARAM_KEYS, RAW_FIELDS, SERVICES,
  make_dk_vehicle_diagnostics, raw_snapshot,
)


class FakeSM(dict):
  def __init__(self, **values):
    super().__init__(values)
    self.valid = dict.fromkeys(SERVICES, True)
    self.alive = dict.fromkeys(SERVICES, True)
    self.logMonoTime = dict.fromkeys(SERVICES, 1_000_000_000)

  def all_alive(self, services):
    assert services == ['carControl']
    return True


def fixture_objects():
  CS = NS(vEgo=10.0, vEgoRaw=10.0, aEgo=0.0, steeringAngleDeg=0.0, steeringRateDeg=0.0,
          steeringTorque=0.0, steeringTorqueEps=0.0, standstill=False, canValid=True,
          cruiseState=NS(standstill=False, enabled=True, available=True), brakePressed=False,
          gasPressed=False, brakeHoldActive=False, parkingBrake=False, steeringPressed=False,
          steerFaultTemporary=False, steerFaultPermanent=False, accFaulted=False)
  CC = NS(enabled=True, latActive=True, longActive=False, actuators=NS(steeringAngleDeg=0.0, torque=0.1, curvature=0.0, accel=0.0),
          cruiseControl=NS(resume=False, cancel=False, override=False))
  raw = NS(**dict.fromkeys(RAW_FIELDS))
  raw.scc_control = {'COUNTER': 4, 'InfoDisplay': 0, 'aReqRaw': -0.2, 'aReqValue': -0.1}
  raw.adrv_0x161 = {'COUNTER': 3, 'ALERTS_5': 0}
  raw.cp = NS(bus=0, ts_nanos={'SCC_CONTROL': {'COUNTER': 990_000_000, 'InfoDisplay': 990_000_000}})
  CI = NS(CS=raw, CC=NS(frame=1, apply_angle_last=0.0, apply_torque_last=1, stock_scc_keepalive_request_count=0))
  sm = FakeSM(longitudinalPlan=NS(speeds=[0.0, 0.3], shouldStop=True, aTarget=0.1),
              radarState=NS(leadOne=NS(status=True, radar=True, dRel=30.0, vRel=0.0, vLead=10.0, aLeadK=0.0, radarTrackId=2)),
              lateralPlan=NS(curvatures=[0.0] * 17), modelV2=NS(laneLines=[], laneLineProbs=[]), controlsState=NS())
  return CS, CC, CI, sm


def record(observer, CS, CC, CI, sm, now=1_000_000_000, can_sends=()):
  token = observer.begin(CS, CC, CI, now)
  observer.finish(token, CS, CC, CI, sm, CC.actuators, can_sends, now)


def cp(fingerprint=CAR.KIA_CARNIVAL_4TH_GEN, long=False):
  return NS(carFingerprint=fingerprint, flags=0, extFlags=0, pcmCruise=not long, openpilotLongitudinalControl=long,
            steerControlType='angle', lateralTuning=NS(which=lambda: 'torque'), mass=2100.0, steerRatio=15.0)


class FakeParams:
  def __init__(self, branch=b'dkcarrot-wip'):
    self.branch = branch
    self.read = []

  def get(self, key):
    self.read.append(key)
    if key == 'GitBranch':
      return self.branch
    if key == 'GitCommit':
      return b'3f53ed3'
    return b'50'


@pytest.mark.parametrize('branch', [b'carrot-wip', b'carrot', b'', None, b'dkcarrot-wip-extra'])
def test_factory_only_exact_dk_branch(branch):
  assert make_dk_vehicle_diagnostics(cp(), FakeParams(branch), logger=lambda _: None) is None


def test_factory_only_ka4():
  assert make_dk_vehicle_diagnostics(cp(CAR.KIA_EV6), FakeParams(), logger=lambda _: None) is None


def test_all_initial_param_keys_exist():
  from openpilot.common.params import Params

  params = Params()
  for key in PARAM_KEYS:
    params.get_type(key)


def test_real_cereal_planner_source_enum_is_not_silently_missing():
  from openpilot.cereal import log

  CS, CC, CI, sm = fixture_objects()
  sm['longitudinalPlan'] = log.LongitudinalPlan.new_message()
  sm['longitudinalPlan'].longitudinalPlanSource = 'lead0'
  logs = []
  record(DkVehicleDiagnostics({}, logs.append), CS, CC, CI, sm)
  assert logs[-1]['planner']['source'] == 'lead0'


@pytest.mark.parametrize('longitudinal', [False, True])
def test_factory_records_misconfiguration_and_only_allowlisted_initial_params(longitudinal):
  params = FakeParams()
  logs = []
  observer = make_dk_vehicle_diagnostics(cp(long=longitudinal), params, logs.append)
  assert observer is not None
  before_reads = list(params.read)
  record(observer, *fixture_objects())
  assert params.read == before_reads
  assert set(params.read) == {*PARAM_KEYS, 'GitBranch', 'GitCommit'}
  session = logs[0]
  assert session['kind'] == 'session'
  assert session['commit'] == '3f53ed3'
  assert session['branch'] == 'dkcarrot-wip'
  assert session['cp']['openpilotLongitudinalControl'] == longitudinal
  assert session['cp']['carFingerprint'] == str(CAR.KIA_CARNIVAL_4TH_GEN)
  assert set(session['initial_params']) == set(PARAM_KEYS)


def test_raw_snapshot_is_immutable_missing_and_packet_age_are_explicit():
  _, _, CI, _ = fixture_objects()
  CI.CS.cruise_buttons_msg = {'COUNTER': [255], 'CRUISE_BUTTONS': [0]}
  snapshot = raw_snapshot(CI.CS, 1_000_000_000)
  CI.CS.cruise_buttons_msg['COUNTER'][0] = 0
  CI.CS.scc_control['InfoDisplay'] = 4
  assert snapshot['cruise_buttons_msg']['values']['COUNTER'] == 255
  assert snapshot['scc_control']['values']['InfoDisplay'] == 0
  assert snapshot['scc_control']['packet_sources'][0]['age_ms'] == 10.0
  assert snapshot['ccnc_0x162']['missing']
  assert snapshot['ccnc_0x162']['packet_sources'] == []


def test_observer_never_mutates_inputs_or_can_and_distinguishes_before_after():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  inputs = copy.deepcopy((CS, CC, CI, sm))
  token = observer.begin(CS, CC, CI, 1_000_000_000)
  assert (CS, CC, CI, sm) == inputs
  # An external controller mutation, not a diagnostic operation.
  CI.CS.scc_control['InfoDisplay'] = 4
  CI.CC.apply_angle_last = 3.0
  sends = [CanData(0x1AA, bytes([0, 0, 5, 0, 16]) + bytes(11), 2)]
  before_finish = copy.deepcopy((CS, CC, CI, sm, sends))
  observer.finish(token, CS, CC, CI, sm, CC.actuators, sends, 1_000_000_000)
  assert (CS, CC, CI, sm, sends) == before_finish
  sample = logs[-1]
  assert sample['raw_before']['scc_control']['values']['InfoDisplay'] == 0
  assert sample['raw_after']['scc_control']['values']['InfoDisplay'] == 4
  assert sample['controller_before']['apply_angle_last'] == 0.0
  assert sample['controller_after']['apply_angle_last'] == 3.0
  assert sample['submitted_can']['buttons'] == [{'address': 0x1AA, 'bus': 2, 'button': 1, 'count': 1}]


def test_session_and_sample_json_are_finite_and_contain_no_arbitrary_params():
  CS, CC, CI, sm = fixture_objects()
  CS.aEgo = math.nan
  CC.actuators.torque = math.inf
  CI.CS.scc_control['aReqRaw'] = -math.inf
  logs = []
  record(DkVehicleDiagnostics({}, logs.append), CS, CC, CI, sm)
  for message in logs:
    json.dumps(message, allow_nan=False)
  assert logs[-1]['car']['accel_mps2'] is None
  assert logs[-1]['output']['torque'] is None
  assert logs[-1]['raw_before']['scc_control']['values']['aReqRaw'] is None


def test_periodic_sampling_is_ten_hz_and_all_submitted_buttons_are_counted():
  objects = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  for i in range(101):
    sends = [CanData(0x1AA, bytes([0, 0, i % 256, 0, 16]) + bytes(11), 2)]
    record(observer, *objects, now=1_000_000_000 + i * 10_000_000, can_sends=sends)
  samples = [msg for msg in logs if msg['kind'] == 'sample']
  assert len(samples) == 11
  assert sum(s['submitted_can']['count'] for s in samples) == 101
  assert sum(b['count'] for s in samples for b in s['submitted_can']['buttons']) == 101
  assert samples[-1]['counters']['rate_skipped'] == 90


def test_edges_are_bounded_twenty_hz_and_short_transitions_survive_skipping():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  for i in range(101):
    CC.cruiseControl.resume = bool(i % 2)
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 10_000_000)
  samples = [msg for msg in logs if msg['kind'] == 'sample']
  assert len(samples) == 21
  transitions = [t for s in samples for t in s['transitions'] if t['field'] == 'resume']
  assert len(transitions) == 100
  assert sum('resume' in s['topics'] for s in samples) == 1
  assert all(b['mono_ns'] - a['mono_ns'] >= 50_000_000 for a, b in zip(samples, samples[1:], strict=False))


@pytest.mark.parametrize('topic', ['resume', 'engage_warning', 'curve', 'unwind', 'braking'])
def test_candidate_topics_are_observations_only_and_use_cooldowns(topic):
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  for i in range(50):
    if topic == 'resume':
      CS.standstill = True
    elif topic == 'curve':
      CC.actuators.curvature = 0.005
    elif topic == 'unwind':
      CS.steeringAngleDeg = 40.0
      CC.actuators.steeringAngleDeg = 30.0 - i * 0.2
    elif topic == 'braking':
      sm['radarState'].leadOne.vRel = -3.0
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 100_000_000)
  matching = [s for s in logs if topic in s['topics']]
  assert len(matching) == 1
  assert matching[0]['interpretation'] == 'candidate_observation_not_diagnosis'


def test_shadow_resume_and_braking_are_not_control_outputs():
  CS, CC, CI, sm = fixture_objects()
  CS.standstill = CS.cruiseState.standstill = True
  sm['radarState'].leadOne.vRel = -4.0
  logs = []
  record(DkVehicleDiagnostics({}, logs.append), CS, CC, CI, sm)
  sample = logs[-1]
  assert sample['shadow']['legacy_resume']
  assert not sample['shadow']['should_stop_resume']
  assert not CC.cruiseControl.resume
  assert sample['shadow']['gentle_decel_mps2'] < 0
  assert CC.actuators.accel == 0.0
  assert sample['shadow']['interpretation'] == 'hypothesis_not_vehicle_response'


def test_unwind_step_to_fixed_center_records_lagging_return():
  CS, CC, CI, sm = fixture_objects()
  CS.steeringAngleDeg = CC.actuators.steeringAngleDeg = 20.0
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, CS, CC, CI, sm)
  CC.actuators.steeringAngleDeg = 0.0
  for i in range(1, 5):
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 100_000_000)
  matching = [s for s in logs if 'unwind' in s['topics']]
  assert len(matching) == 1
  assert matching[0]['request']['angle_deg'] == 0.0
  assert matching[0]['car']['steering_angle_deg'] == 20.0
  assert matching[0]['shadow']['requested_angle_rate_dps'] == 0.0
  assert matching[0]['shadow']['unwind_active']
  assert logs[-1]['shadow']['unwind_active']
  assert 'unwind' not in logs[-1]['topics']


@pytest.mark.parametrize('attr,name', [('ccnc_0x162', 'FAULT_DAS'), ('scc_control', 'TakeOverReq'),
                                     ('scc_control', 'SysFailState'), ('mdps', 'LKA_FAULT')])
def test_warning_changes_are_captured_at_edge_rate(attr, name):
  CS, CC, CI, sm = fixture_objects()
  setattr(CI.CS, attr, {name: 0})
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, CS, CC, CI, sm)
  getattr(CI.CS, attr)[name] = 1
  record(observer, CS, CC, CI, sm, now=1_050_000_000)
  assert logs[-1]['mono_ns'] == 1_050_000_000
  assert any(t['field'].endswith(name) and t['after'] == 1 for t in logs[-1]['transitions'])


def test_session_is_repeated_for_later_log_segments():
  objects = fixture_objects()
  logs = []
  observer = make_dk_vehicle_diagnostics(cp(), FakeParams(), logs.append)
  record(observer, *objects)
  record(observer, *objects, now=61_000_000_000)
  sessions = [msg for msg in logs if msg['kind'] == 'session']
  assert len(sessions) == 2
  assert all(msg['diagnostics_version'] == 'dk-vehicle-diag-v2' for msg in sessions)


def test_driver_brake_without_lead_survives_recent_disengagement_and_ordinary_cooldown():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  sm['radarState'].leadOne.status = False
  record(observer, CS, CC, CI, sm)
  observer.last_topic_ns['braking'] = 1_000_000_000
  CC.enabled = CS.cruiseState.enabled = False
  record(observer, CS, CC, CI, sm, now=1_100_000_000)
  CS.brakePressed = True
  record(observer, CS, CC, CI, sm, now=1_150_000_000)
  sample = logs[-1]
  assert 'braking' in sample['topics']
  assert sample['braking_capture']['priority'] == 1
  assert sample['braking_capture']['reasons'][0]['reason'] == 'driver_brake_intervention'
  assert not sample['request']['enabled'] and not sample['radar']['status']


def test_takeover_and_hard_deceleration_promote_same_episode_after_driver_disengages():
  CS, CC, CI, sm = fixture_objects()
  CI.CS.scc_control['TakeOverReq'] = 0
  sm['radarState'].leadOne.status = False
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, CS, CC, CI, sm)
  CI.CS.scc_control['TakeOverReq'] = 1
  record(observer, CS, CC, CI, sm, now=1_050_000_000)
  assert logs[-1]['braking_capture']['priority'] == 2
  assert logs[-1]['braking_capture']['reasons'][0]['reason'] == 'scc_takeover_request'
  CC.enabled = CS.cruiseState.enabled = False
  CS.brakePressed = True
  for i in range(10, 101):
    CS.aEgo = -6.5 if i == 25 else -3.5
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 10_000_000)
  hard = [s for s in logs if s.get('braking_capture', {}).get('priority') == 2
          and any(r['reason'] == 'hard_deceleration' for r in s['braking_capture']['reasons'])]
  assert len(hard) == 1
  assert not hard[0]['request']['enabled']
  assert min(s['braking_observation']['window_min_accel_mps2'] for s in logs if s['kind'] == 'sample') == -6.5
  assert any(s['braking_observation']['window_min_accel_mono_ns'] == 1_250_000_000 for s in logs if s['kind'] == 'sample')
  assert all(not s['braking_observation']['lead_required'] for s in hard)


def test_hard_deceleration_requires_dwell_and_recent_motion_not_valid_lead_or_control():
  CS, CC, CI, sm = fixture_objects()
  CC.enabled = CS.cruiseState.enabled = False
  sm['radarState'].leadOne.status = False
  CS.aEgo = -4.0
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, CS, CC, CI, sm)
  record(observer, CS, CC, CI, sm, now=1_050_000_000)
  assert not any(s.get('braking_capture', {}).get('priority') == 2 for s in logs)
  record(observer, CS, CC, CI, sm, now=1_100_000_000)
  assert logs[-1]['braking_capture']['priority'] == 2
  CS.vEgo = 0.0
  record(observer, CS, CC, CI, sm, now=25_000_000_000)
  assert logs[-1]['braking_capture']['priority'] == 0


def test_braking_reason_buffers_and_capture_rate_are_bounded_even_if_signals_toggle():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  for i in range(500):
    CS.brakePressed = bool(i % 2)
    CI.CS.scc_control['TakeOverReq'] = i % 2
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 10_000_000)
  reasons = [r for s in logs if s['kind'] == 'sample' for r in s['braking_capture']['reasons']]
  assert len(reasons) == 2
  assert len(observer.last_braking_reason_ns) <= 3
  assert len(observer.pending_braking_reasons) <= 3


def test_torque_semantics_limits_model_hypotheses_and_parser_popup_values_are_explicit():
  CS, CC, CI, sm = fixture_objects()
  CS.useLaneLineSpeed = 0
  CC.actuators.torqueOutputCan = 57
  CI.CC.params = NS(STEER_MAX=270, STEER_DELTA_UP=2, STEER_DELTA_DOWN=3, STEER_DRIVER_ALLOWANCE=250)
  CI.CS.lfahda_cluster = {'HDA_InfoPUDis': 0}
  CI.CS.cp_cam = NS(bus=2, ts_nanos={'LFAHDA_CLUSTER': {'HDA_InfoPUDis': 990_000_000}},
                    vl={'LFAHDA_CLUSTER': {'HDA_InfoPUDis': 3}})
  sm['controlsState'].activeLaneLine = False
  sm['modelV2'].action = NS(desiredCurvature=0.003)
  sm['modelV2'].leadsV3 = [NS(prob=0.9, probTime=0.0, x=list(range(10)), y=[0.0], v=[-10.0], a=[0.0])] * 4
  logs = []
  record(DkVehicleDiagnostics({'cp': {'steerControlType': 'torque'}}, logs.append), CS, CC, CI, sm)
  sample = logs[-1]
  assert sample['lateral']['angle_semantics'] == 'not_an_eps_angle_command_when_torque_or_unknown'
  assert sample['output']['torque_output_can'] == 57
  assert sample['controller_before']['limits']['STEER_DELTA_DOWN'] == 3
  assert sample['car']['use_lane_line_speed_kph'] == 0
  assert sample['lateral']['active_lane_line'] is False
  assert sample['lateral']['model_desired_curvature'] == 0.003
  assert len(sample['perception']['model_leads']) == 3
  assert len(sample['perception']['model_leads'][0]['x']) == 6
  raw = sample['raw_before']['lfahda_cluster']
  assert raw['values']['HDA_InfoPUDis'] == 0
  assert raw['packet_sources'][0]['decoded_values']['HDA_InfoPUDis'] == 3
  assert len(json.dumps(sample, allow_nan=False)) < 64 * 1024


def test_always_lateral_curve_and_unwind_are_observed_without_overall_engagement():
  CS, CC, CI, sm = fixture_objects()
  CC.enabled = False
  CC.latActive = True
  CC.actuators.curvature = 0.005
  CC.actuators.steeringAngleDeg = CS.steeringAngleDeg = 20.0
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  for i in range(5):
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 100_000_000)
  assert any('curve' in s['topics'] for s in logs)
  CC.actuators.steeringAngleDeg = 0.0
  for i in range(5, 10):
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 100_000_000)
  assert any('unwind' in s['topics'] for s in logs)
  assert not CC.enabled and CC.latActive
  CC.enabled = True
  CC.latActive = False
  record(observer, CS, CC, CI, sm, now=2_000_000_000)
  assert not logs[-1]['shadow']['curve_active']
  assert not logs[-1]['shadow']['unwind_active']


def test_torque_tracking_hints_work_without_angle_fields_and_without_overall_enabled():
  CS, CC, CI, sm = fixture_objects()
  CC.enabled = False
  CC.actuators.steeringAngleDeg = None
  CC.actuators.torque = 0.5
  sm['controlsState'].lateralControlState = NS(which=lambda: 'torque', torque=NS(actualLateralAccel=0.0, desiredLateralAccel=1.2))
  output = NS(torque=-0.96, torqueOutputCan=-260, steeringAngleDeg=None)
  logs = []
  observer = DkVehicleDiagnostics({'cp': {'steerControlType': 'torque'}}, logs.append)
  for i in range(10):
    now = 1_000_000_000 + i * 100_000_000
    token = observer.begin(CS, CC, CI, now)
    observer.finish(token, CS, CC, CI, sm, output, (), now)
  assert sum('curve' in s['topics'] for s in logs) == 1
  assert sum('unwind' in s['topics'] for s in logs) == 1
  sample = logs[-1]
  assert sample['lateral']['accel_error_mps2'] == 1.2
  assert sample['shadow']['torque_saturation_active']
  assert sample['shadow']['lateral_accel_error_active']
  assert sample['shadow']['torque_opposed_active']
  assert not sample['shadow']['unwind_active']  # Separate from angle-based hypothesis.
  assert not CC.enabled and CC.actuators.torque == 0.5 and output.torque == -0.96


def test_model_hypothesis_and_lane_iteration_does_not_read_past_bounded_prefix():
  CS, CC, CI, sm = fixture_objects()

  def bounded_input(count):
    for _ in range(count):
      yield NS(prob=0.9, probTime=0.0, x=[1.0], y=[0.0], v=[-1.0], a=[0.0])
    raise AssertionError('diagnostic observer consumed beyond bounded prefix')

  sm['modelV2'].leadsV3 = bounded_input(3)
  sm['modelV2'].laneLines = bounded_input(4)
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, CS, CC, CI, sm)
  assert observer.errors == 0
  assert len(logs[-1]['perception']['model_leads']) == 3
  assert len(logs[-1]['lateral']['lane_lines']) == 4


def test_logger_fault_is_fail_open_and_buffers_stay_bounded():
  def fail(_):
    raise RuntimeError('logger unavailable')

  CS, CC, CI, sm = fixture_objects()
  observer = DkVehicleDiagnostics({}, fail)
  for i in range(1000):
    CC.cruiseControl.resume = bool(i % 2)
    record(observer, CS, CC, CI, sm, now=1_000_000_000 + i * 10_000_000)
  assert observer.errors > 0
  assert len(observer.transitions) == MAX_TRANSITIONS
  assert observer.transitions_dropped > 0
  assert len(observer.pending_topics) <= 5


def test_real_cereal_messages_emit_complete_sample():
  from openpilot.cereal import car, log

  CS = car.CarState.new_message()
  CC = car.CarControl.new_message()
  CP = car.CarParams.new_message()
  CP.carFingerprint = str(CAR.KIA_CARNIVAL_4TH_GEN)
  _, _, CI, _ = fixture_objects()
  sm = FakeSM(longitudinalPlan=log.LongitudinalPlan.new_message(), radarState=log.RadarState.new_message(),
              lateralPlan=log.LateralPlan.new_message(), modelV2=log.ModelDataV2.new_message(), controlsState=log.ControlsState.new_message())
  logs = []
  observer = make_dk_vehicle_diagnostics(CP, FakeParams(), logs.append)
  assert observer is not None
  record(observer, CS, CC, CI, sm)
  assert observer.errors == 0
  assert [entry['kind'] for entry in logs] == ['session', 'sample']
  json.dumps(logs, allow_nan=False)


@pytest.mark.parametrize('observer_mode', ['none', 'normal', 'begin_fault', 'finish_fault', 'logger_fault'])
def test_card_actuation_and_submission_unchanged_and_logging_is_after_send(monkeypatch, observer_mode):
  from openpilot.selfdrive.car import card

  CS, CC, CI, sm = fixture_objects()
  order = []
  expected_output = NS(steeringAngleDeg=2.0, torque=0.25, curvature=0.001, accel=-0.2)
  expected_sends = [CanData(0x1AA, bytes([0, 0, 8, 0, 16]) + bytes(11), 2)]

  def apply(actual_cc, now, model):
    assert actual_cc is CC
    order.append('apply')
    return expected_output, expected_sends

  def send(name, payload):
    assert name == 'sendcan'
    assert payload == expected_sends
    order.append('sendcan')

  def log_message(_):
    assert 'sendcan' in order
    order.append('log')
    if observer_mode == 'logger_fault':
      raise RuntimeError('logging unavailable')

  def fail(*args):
    raise RuntimeError('diagnostic fault')

  CI.apply = apply
  instance = card.Car.__new__(card.Car)
  instance.CI = CI
  instance.sm = sm
  instance.pm = NS(send=send)
  instance.initialized_prev = True
  instance.card_diag_recv_ns = 0
  instance.card_diag_stage_names = ('apply', 'sendcan', 'total')
  instance.card_diag_stage_current = dict.fromkeys(instance.card_diag_stage_names, 0)
  instance.card_diag_stage_sum_us = dict.fromkeys(instance.card_diag_stage_names, 0)
  instance.card_diag_stage_max_us = dict.fromkeys(instance.card_diag_stage_names, 0)
  instance.card_diag_process_max_us = instance.card_diag_slow_process = instance.card_diag_frames = 0
  instance.dk_vehicle_diagnostics = None if observer_mode == 'none' else DkVehicleDiagnostics({}, log_message)
  if observer_mode == 'begin_fault':
    instance.dk_vehicle_diagnostics.begin = fail
  if observer_mode == 'finish_fault':
    instance.dk_vehicle_diagnostics.finish = fail
  monkeypatch.setattr(card, 'can_list_to_can_capnp', lambda msgs, **kwargs: msgs)
  monkeypatch.setattr(card, 'REPLAY', False)
  before_inputs = copy.deepcopy((CS, CC, expected_sends))
  instance.controls_update(CS, CC)
  assert (CS, CC, expected_sends) == before_inputs
  assert order[:2] == ['apply', 'sendcan']
  assert order.count('apply') == order.count('sendcan') == 1
  assert instance.last_actuators_output is expected_output
  assert instance.CC_prev is CC
