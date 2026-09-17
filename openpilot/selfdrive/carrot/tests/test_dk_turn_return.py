import copy
import json
import math

import pytest

from openpilot.selfdrive.carrot.dk_turn_return import (
  EPISODE_MAX_NS, REASON_PRIORITIES, TurnReturnObserver,
)
from openpilot.selfdrive.carrot.dk_vehicle_diagnostics import DkVehicleDiagnostics, steering_angle_source
from openpilot.selfdrive.carrot.tests.test_dk_vehicle_diagnostics import fixture_objects


def update(observer, t, **overrides):
  now_ns = 1_000_000_000 + int(t * 1e9)
  values = dict(angle=40.0, angle_mono_ns=now_ns, speed=8.0, can_valid=True, active=True, pressed=False,
                driver_torque=0.0, request_fresh=True, torque_mode=True, request_torque=0.5, output_torque=0.5,
                request_angle=0.0, desired_curvature=None, transitions=())
  values.update(overrides)
  before = copy.deepcopy(values)
  result = observer.update(now_ns, **values)
  assert values == before or values.get('angle') is not None and math.isnan(values['angle'])
  json.dumps(result, allow_nan=False)
  return result


def enter(observer, direction=1, speed=8):
  for i in range(4):
    result = update(observer, i / 10, angle=40.0 * direction, speed=speed,
                    request_torque=0.5 * direction, output_torque=0.5 * direction)
  assert result[0]['phase'] == 'turn'
  return result


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('speed', [1.5, 20 / 3.6, 35 / 3.6, 50 / 3.6])
def test_city_and_slow_turn_return_complete_both_directions_without_high_speed_gate(direction, speed):
  observer = TurnReturnObserver()
  entered, _ = enter(observer, direction, speed)
  returning, reasons = update(observer, 0.4, angle=30 * direction, speed=speed,
                              request_torque=0.3 * direction, output_torque=0.4 * direction)
  assert returning['phase'] == 'return'
  assert returning['episode_id'] == entered['episode_id']
  assert returning['direction'] == ('left_positive_angle' if direction > 0 else 'right_negative_angle')
  assert returning['signed_angle_rate_dps'] == pytest.approx(-100 * direction)
  assert returning['centerward_angle_rate_dps'] == pytest.approx(100)
  assert returning['peak_angle_deg'] == 40
  assert returning['angle_returning_from_peak']
  assert reasons == [{'reason': 'turn_return_candidate', 'priority': 1, 'mono_ns': 1_400_000_000}]
  complete, reasons = update(observer, 0.5, angle=4 * direction, speed=speed,
                             request_torque=0.0, output_torque=0.0)
  assert complete['phase'] == 'complete'
  assert not reasons
  assert observer.episode is None


@pytest.mark.parametrize('direction', [-1, 1])
def test_slow_return_same_sign_output_decay_captures_without_requiring_torque_reversal(direction):
  observer = TurnReturnObserver()
  enter(observer, direction)
  collected = []
  for i in range(4, 12):
    observation, reasons = update(observer, i / 10, angle=40 * direction,
                                  request_torque=0.15 * direction, output_torque=(0.50 - i * 0.015) * direction)
    collected.extend(reasons)
  names = {r['reason'] for r in collected}
  assert names == {'turn_return_candidate', 'return_slow_angle_response', 'return_request_output_gap'}
  assert observation['same_sign_torque_output_lag']
  assert observation['request_output_same_sign']
  assert not observation['request_output_opposed']
  assert observation['output_torque_falling_in_turn_direction']
  assert observation['centerward_angle_rate_dps'] == 0
  assert len(collected) == 3


def test_torque_opposition_after_observed_turn_is_a_hint_not_mechanical_delay():
  observer = TurnReturnObserver()
  enter(observer)
  collected = []
  for i in range(4, 8):
    observation, reasons = update(observer, i / 10, request_torque=-0.1, output_torque=0.3)
    collected.extend(reasons)
  assert observation['request_output_opposed']
  assert not observation['request_output_same_sign']
  assert 'return_request_output_gap' in {r['reason'] for r in collected}
  assert 'not_fault_or_lane_departure' in observation['interpretation']


@pytest.mark.parametrize('direction', [-1, 1])
def test_driver_intervention_and_release_during_same_sign_turn_get_priority_two(direction):
  observer = TurnReturnObserver()
  enter(observer, direction)
  observation, reasons = update(observer, 0.4, angle=40 * direction, pressed=True, driver_torque=-2 * direction,
                                request_torque=0.5 * direction, output_torque=0.5 * direction)
  assert observation['driver_centerward_torque']
  assert observation['driver_press_mono_ns'] == 1_400_000_000
  assert {r['reason']: r['priority'] for r in reasons} == {
    'turn_return_candidate': 1, 'turn_driver_steering_intervention': 2,
  }
  observation, reasons = update(observer, 0.5, angle=35 * direction, pressed=False,
                                request_torque=0.5 * direction, output_torque=0.5 * direction)
  assert observation['driver_release_mono_ns'] == 1_500_000_000
  assert not reasons


def test_deactivation_and_driver_intervention_persist_in_recent_active_grace():
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, 0.4, active=False)
  assert observation['recent_lateral_active']
  assert observation['lateral_deactivation_mono_ns'] == 1_400_000_000
  assert any(r['reason'] == 'turn_lateral_deactivation' and r['priority'] == 2 for r in reasons)
  observation, reasons = update(observer, 0.5, active=False, pressed=True)
  assert any(r['reason'] == 'turn_driver_steering_intervention' and r['priority'] == 2 for r in reasons)
  for i in range(6, 36):
    observation, _ = update(observer, i / 10, active=False)
  assert observation['phase'] == 'inactive'
  assert observer.episode is None


def test_short_edges_between_samples_are_recorded_without_inventing_current_driver_state():
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, 0.4, transitions=[
    {'field': 'steering_pressed', 'before': False, 'after': True, 'mono_ns': 1_310_000_000},
    {'field': 'steering_pressed', 'before': True, 'after': False, 'mono_ns': 1_330_000_000},
    {'field': 'lat_active', 'before': True, 'after': False, 'mono_ns': 1_350_000_000},
    {'field': 'lat_active', 'before': False, 'after': True, 'mono_ns': 1_370_000_000},
  ])
  assert observation['driver_press_mono_ns'] == 1_310_000_000
  assert observation['driver_release_mono_ns'] == 1_330_000_000
  assert observation['lateral_deactivation_mono_ns'] == 1_350_000_000
  assert {r['reason'] for r in reasons} == {
    'turn_return_candidate', 'turn_driver_steering_intervention', 'turn_lateral_deactivation',
  }


@pytest.mark.parametrize('overrides', [
  {'angle': 0}, {'angle': 14.9}, {'speed': 0.5}, {'active': False},
])
def test_straight_stationary_small_angle_or_never_active_does_not_create_turn(overrides):
  observer = TurnReturnObserver()
  for i in range(20):
    observation, reasons = update(observer, i / 10, **overrides)
    assert observation['episode_id'] is None
    assert not reasons


def test_torque_mode_ignores_requested_angle_as_eps_command():
  observer = TurnReturnObserver()
  for i in range(4):
    update(observer, i / 10, request_angle=40)
  for i in range(4, 15):
    observation, reasons = update(observer, i / 10, request_angle=0)
    assert observation['phase'] == 'turn'
    assert not observation['angle_goal_reducing']
    assert not reasons
  assert observation['requested_angle_semantics'] == 'not_an_eps_angle_command'


def test_angle_control_can_capture_angle_goal_reduction():
  observer = TurnReturnObserver()
  for i in range(4):
    update(observer, i / 10, torque_mode=False, request_angle=40)
  observation, reasons = update(observer, 0.4, torque_mode=False, request_angle=0)
  assert observation['angle_goal_reducing']
  assert [r['reason'] for r in reasons] == ['turn_return_candidate']


def test_turn_peak_from_entry_confirmation_window_is_preserved():
  observer = TurnReturnObserver()
  update(observer, 0.0, angle=60, request_torque=0.7)
  update(observer, 0.1, angle=50, request_torque=0.6)
  observation, reasons = update(observer, 0.2, angle=40, request_torque=0.5)
  assert observation['peak_angle_deg'] == 60
  assert observation['peak_mono_ns'] == 1_000_000_000
  assert observation['phase'] == 'return'
  assert [r['reason'] for r in reasons] == ['turn_return_candidate']


@pytest.mark.parametrize('overrides', [
  {'angle': math.nan}, {'angle': math.inf}, {'speed': math.nan}, {'angle': 5000},
  {'angle_mono_ns': None}, {'angle_mono_ns': math.inf}, {'angle_mono_ns': True},
  {'angle_mono_ns': 100}, {'angle_mono_ns': 5_000_000_000},
  {'can_valid': False}, {'request_fresh': False},
])
def test_invalid_or_stale_data_resets_without_nonfinite_log_values(overrides):
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, 0.4, **overrides)
  assert observation['phase'] == 'reset'
  assert observer.episode is None
  assert not reasons


@pytest.mark.parametrize('t', [0.2, 0.3, 1.0])
def test_nonmonotonic_or_long_sample_gap_never_computes_false_rate(t):
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, t)
  assert observation['phase'] == 'reset'
  assert observation['signed_angle_rate_dps'] is None
  assert not reasons


def test_repeated_source_timestamp_is_not_a_zero_rate_or_slow_response():
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, 0.4, angle_mono_ns=1_300_000_000)
  assert not observation['angle_source_updated']
  assert observation['signed_angle_rate_dps'] is None
  assert not reasons
  observation, reasons = update(observer, 0.5, angle_mono_ns=1_500_000_000, angle=20)
  assert observation['signed_angle_rate_dps'] == pytest.approx(-100)


def test_driver_edge_survives_one_repeated_can_source_sample():
  observer = TurnReturnObserver()
  enter(observer)
  observation, reasons = update(observer, 0.4, angle_mono_ns=1_300_000_000, transitions=[
    {'field': 'steering_pressed', 'before': False, 'after': True, 'mono_ns': 1_340_000_000},
    {'field': 'steering_pressed', 'before': True, 'after': False, 'mono_ns': 1_360_000_000},
  ])
  assert observation['signed_angle_rate_dps'] is None
  assert any(r['reason'] == 'turn_driver_steering_intervention' for r in reasons)
  assert observation['driver_release_mono_ns'] == 1_360_000_000


def test_episode_timeout_and_many_turns_keep_constant_size_state():
  observer = TurnReturnObserver()
  observations = []
  for i in range(int(EPISODE_MAX_NS / 100_000_000) + 20):
    observation, _ = update(observer, i / 10)
    observations.append(observation)
    if observer.episode:
      assert len(observer.episode['emitted']) <= len(REASON_PRIORITIES)
  assert any(o.get('reset_reason') == 'episode_timeout' for o in observations)
  assert observer.sequence <= 2
  assert not any(isinstance(v, list) for v in vars(observer).values())


def test_integration_new_reason_bypasses_old_topic_cooldown_and_uses_actual_can_time():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({'cp': {'steerControlType': 'torque', 'flags': 0}}, logs.append)
  observer.last_topic_ns['unwind'] = 1_000_000_000
  CC.enabled = False  # AlwaysLateral is independent of SCC/global engagement.
  CS.steeringAngleDeg = 40
  CC.actuators.torque = 0.5
  output = copy.copy(CC.actuators)
  for i in range(8):
    now = 1_000_000_000 + i * 100_000_000
    sm.logMonoTime['carControl'] = sm.logMonoTime['controlsState'] = now
    CI.CS.cp.ts_nanos['STEERING_SENSORS'] = {'STEERING_ANGLE': now - 10_000_000}
    CS.steeringPressed = i >= 5
    if i >= 4:
      CC.actuators.torque = 0.2
    if i == 7:
      CS.steeringAngleDeg = 30
    token = observer.begin(CS, CC, CI, now)
    observer.finish(token, CS, CC, CI, sm, output, (), now)
  samples = [s for s in logs if s['kind'] == 'sample']
  assert observer.errors == 0
  baseline = next(s for s in samples if s['unwind_capture']['priority'] == 1)
  intervention = next(s for s in samples if s['unwind_capture']['priority'] == 2)
  assert baseline['unwind_capture']['episode_id'] == intervention['unwind_capture']['episode_id']
  assert 'unwind' in intervention['topics']
  assert samples[-1]['car']['steering_signed_rate_dps'] == pytest.approx(-100)
  assert samples[-1]['turn_return']['angle_source_age_ms'] == 10
  assert all(s['unwind_capture']['priority'] == 0 for s in samples[:4])
  json.dumps(logs, allow_nan=False)


def test_integration_optional_can_source_absent_is_explicit_without_disabling_existing_logs():
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({'cp': {'steerControlType': 'torque'}}, logs.append)
  now = 1_000_000_000
  observer.finish(observer.begin(CS, CC, CI, now), CS, CC, CI, sm, CC.actuators, (), now)
  assert observer.errors == 0
  assert logs[-1]['turn_return']['reset_reason'] == 'missing_or_invalid_timestamp'
  assert logs[-1]['unwind_capture']['priority'] == 0
  assert logs[-1]['car']['steering_signed_rate_dps'] is None


def test_angle_source_follows_exact_carstate_source_not_other_parser_latest_timestamp():
  from opendbc.car.hyundai.values import HyundaiFlags

  _, _, CI, _ = fixture_objects()
  CI.CS.cp.ts_nanos['STEERING_SENSORS'] = {'STEERING_ANGLE': 1_000_000_000}
  CI.CS.cp.ts_nanos['MDPS'] = {'STEERING_ANGLE_2': 2_000_000_000}
  assert steering_angle_source(CI.CS, {'flags': 0})['mono_ns'] == 1_000_000_000
  assert steering_angle_source(CI.CS, {'flags': int(HyundaiFlags.ANGLE_CONTROL)})['mono_ns'] == 2_000_000_000


def test_optional_turn_observer_exception_does_not_suppress_existing_samples(monkeypatch):
  CS, CC, CI, sm = fixture_objects()
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)

  def fail(*args, **kwargs):
    raise RuntimeError('optional observer unavailable')

  monkeypatch.setattr(observer.turn_return_observer, 'update', fail)
  now = 1_000_000_000
  observer.finish(observer.begin(CS, CC, CI, now), CS, CC, CI, sm, CC.actuators, (), now)
  assert observer.errors == 1
  assert logs[-1]['kind'] == 'sample'
  assert logs[-1]['turn_return']['reset_reason'] == 'turn_observer_error'
  assert logs[-1]['unwind_capture']['priority'] == 0
  assert logs[-1]['request']['torque'] == CC.actuators.torque


def test_failed_logger_pending_reasons_never_mix_episode_ids_and_clear_on_reset(monkeypatch):
  CS, CC, CI, sm = fixture_objects()

  def fail(_):
    raise RuntimeError('logger unavailable')

  observer = DkVehicleDiagnostics({}, fail)
  for index, reason in enumerate(('turn_return_candidate', 'turn_driver_steering_intervention')):
    now = 1_000_000_000 + index * 100_000_000
    snapshot = {'phase': 'return', 'episode_id': f'turn-{now}', 'signed_angle_rate_dps': 0.0}
    reasons = [{'reason': reason, 'priority': REASON_PRIORITIES[reason], 'mono_ns': now}]
    monkeypatch.setattr(observer.turn_return_observer, 'update', lambda *a, s=snapshot, r=reasons, **kw: (s, r))
    observer.finish(observer.begin(CS, CC, CI, now), CS, CC, CI, sm, CC.actuators, (), now)
    assert observer.unwind_episode_id == f'turn-{now}'
    assert list(observer.pending_unwind_reasons) == [reason]
  now = 1_200_000_000
  monkeypatch.setattr(observer.turn_return_observer, 'update', lambda *a, **kw: (
    {'phase': 'reset', 'episode_id': None, 'signed_angle_rate_dps': None}, []))
  observer.finish(observer.begin(CS, CC, CI, now), CS, CC, CI, sm, CC.actuators, (), now)
  assert observer.pending_unwind_reasons == {}
  assert observer.unwind_episode_id is None
