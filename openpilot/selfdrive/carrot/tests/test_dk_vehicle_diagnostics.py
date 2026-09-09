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
  assert all(msg['diagnostics_version'] == 'dk-vehicle-diag-v1' for msg in sessions)


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
