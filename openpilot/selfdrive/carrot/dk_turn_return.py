"""Bounded, passive turn-return observations; never used by steering control.

An episode and its retention hints are hypotheses for reviewing the full rlog,
not evidence that the vehicle departed a lane or that a limit needs increasing.
Only timestamped, fresh steering observations can advance the state machine.
"""
import math
from numbers import Integral, Real


TURN_MIN_ANGLE_DEG = 15.0
TURN_MIN_SPEED_MPS = 1.0
TURN_DWELL_NS = 200_000_000
SOURCE_MAX_AGE_NS = 200_000_000
MAX_SAMPLE_GAP_NS = 500_000_000
RECENT_LATERAL_NS = 3_000_000_000
EPISODE_MAX_NS = 30_000_000_000
SLOW_RETURN_DWELL_NS = 500_000_000
GAP_DWELL_NS = 200_000_000
REASON_PRIORITIES = {
  'turn_return_candidate': 1,
  'return_slow_angle_response': 1,
  'return_request_output_gap': 1,
  'turn_driver_steering_intervention': 2,
  'turn_lateral_deactivation': 2,
}


def finite(value, bound=1e6):
  return float(value) if isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value) and abs(value) <= bound else None


def valid_time(value):
  return isinstance(value, Integral) and not isinstance(value, bool) and 0 < value < 2**63


class TurnReturnObserver:
  """One episode, one preceding sample, five possible reasons: bounded memory."""
  def __init__(self):
    self.previous = None
    self.last_active_ns = None
    self.entry_start_ns = None
    self.entry_direction = 0
    self.entry_peak = None
    self.episode = None
    self.sequence = 0
    self.slow_start_ns = None
    self.gap_start_ns = None

  def _clear(self):
    self.previous = None
    self.last_active_ns = None
    self.entry_start_ns = None
    self.entry_direction = 0
    self.entry_peak = None
    self.episode = None
    self.slow_start_ns = self.gap_start_ns = None

  def update(self, now_ns, *, angle, angle_mono_ns, speed, can_valid, active, pressed, driver_torque,
             request_fresh, torque_mode, request_torque, output_torque, request_angle,
             desired_curvature, transitions=()):
    observation = {
      'interpretation': 'turn_return_candidate_not_fault_or_lane_departure_diagnosis',
      'phase': 'idle', 'episode_id': None, 'direction': None,
      'angle_source_mono_ns': int(angle_mono_ns) if valid_time(angle_mono_ns) else None,
      'angle_source_age_ms': ((now_ns - angle_mono_ns) / 1e6
                            if valid_time(now_ns) and valid_time(angle_mono_ns) and now_ns >= angle_mono_ns else None),
      'signed_angle_rate_dps': None, 'angle_rate_semantics': 'signed_angle_delta_over_CAN_source_time',
      'request_fresh': bool(request_fresh), 'recent_lateral_active': False,
      'torque_mode': bool(torque_mode),
      'requested_angle_semantics': ('not_an_eps_angle_command' if torque_mode else 'angle_command_only_if_angle_control'),
      'reasons': [],
    }
    angle, speed = finite(angle, 1080.0), finite(speed, 100.0)
    request_torque, output_torque = finite(request_torque, 10.0), finite(output_torque, 10.0)
    request_angle, desired_curvature = finite(request_angle, 1080.0), finite(desired_curvature, 1.0)
    driver_torque = finite(driver_torque, 1e4)
    invalid = None
    if not valid_time(now_ns) or not valid_time(angle_mono_ns):
      invalid = 'missing_or_invalid_timestamp'
    elif not 0 <= now_ns - angle_mono_ns <= SOURCE_MAX_AGE_NS:
      invalid = 'stale_or_future_angle'
    elif not can_valid or angle is None or speed is None or speed < 0:
      invalid = 'invalid_car_state'
    elif not request_fresh:
      invalid = 'stale_or_invalid_request'
    elif self.previous and (now_ns <= self.previous['now_ns'] or
                            now_ns - self.previous['now_ns'] > MAX_SAMPLE_GAP_NS or
                            angle_mono_ns < self.previous['angle_mono_ns']):
      invalid = 'nonmonotonic_time_or_sample_gap'
    if invalid:
      observation.update(phase='reset', reset_reason=invalid)
      self._clear()
      return observation, []

    previous = self.previous
    if active:
      self.last_active_ns = now_ns
    recent_active = self.last_active_ns is not None and 0 <= now_ns - self.last_active_ns <= RECENT_LATERAL_NS
    observation['recent_lateral_active'] = recent_active
    source_updated = previous is None or angle_mono_ns > previous['angle_mono_ns']
    observation['angle_source_updated'] = source_updated
    if not source_updated:
      # Do not interpret a cached sample as zero steering rate. Keep processing
      # fresh control/driver edges, which must survive a short CAN arrival gap.
      angle = previous['angle']
    dt = (now_ns - previous['now_ns']) / 1e9 if previous else None
    signed_rate = ((angle - previous['angle']) / ((angle_mono_ns - previous['angle_mono_ns']) / 1e9)
                   if previous is not None and source_updated else None)
    request_rate = ((request_torque - previous['request_torque']) / dt
                    if dt and request_torque is not None and previous['request_torque'] is not None else None)
    output_rate = ((output_torque - previous['output_torque']) / dt
                   if dt and output_torque is not None and previous['output_torque'] is not None else None)
    self.previous = {'now_ns': now_ns, 'angle_mono_ns': angle_mono_ns, 'angle': angle,
                     'request_torque': request_torque, 'output_torque': output_torque,
                     'pressed': bool(pressed), 'active': bool(active)}
    observation.update(signed_angle_rate_dps=signed_rate, request_torque_rate_per_s=request_rate,
                       output_torque_rate_per_s=output_rate)
    if not recent_active:
      self.entry_start_ns = None
      self.entry_peak = None
      self.episode = None
      self.slow_start_ns = self.gap_start_ns = None
      observation['phase'] = 'inactive'
      return observation, []

    direction = 1 if angle > 0 else -1
    if self.episode is None:
      if active and speed >= TURN_MIN_SPEED_MPS and abs(angle) >= TURN_MIN_ANGLE_DEG:
        if self.entry_start_ns is None or self.entry_direction != direction:
          self.entry_start_ns, self.entry_direction = now_ns, direction
          self.entry_peak = {'angle': abs(angle), 'mono_ns': now_ns, 'request_torque': 0.0,
                             'request_angle': 0.0, 'desired_curvature': 0.0}
        if source_updated:
          if abs(angle) > self.entry_peak['angle']:
            self.entry_peak['angle'], self.entry_peak['mono_ns'] = abs(angle), now_ns
          for name, value in (('request_torque', direction * request_torque if request_torque is not None else None),
                              ('request_angle', direction * request_angle if request_angle is not None else None),
                              ('desired_curvature', abs(desired_curvature) if desired_curvature is not None else None)):
            if value is not None:
              self.entry_peak[name] = max(self.entry_peak[name], value)
        if source_updated and now_ns - self.entry_start_ns >= TURN_DWELL_NS:
          self.sequence += 1
          self.episode = {
            'id': f'turn-{self.entry_start_ns}', 'start_ns': self.entry_start_ns, 'direction': direction,
            'peak_angle': self.entry_peak['angle'], 'peak_ns': self.entry_peak['mono_ns'],
            'return_ns': None, 'emitted': set(),
            'peak_request_torque': self.entry_peak['request_torque'],
            'peak_request_angle': self.entry_peak['request_angle'],
            'peak_desired_curvature': self.entry_peak['desired_curvature'],
            'driver_pressed_on_entry': bool(pressed), 'driver_press_ns': None, 'driver_release_ns': None,
            'lateral_deactivation_ns': None,
          }
      else:
        self.entry_start_ns = None
        self.entry_peak = None
      if self.episode is None:
        observation['phase'] = 'entry_candidate' if self.entry_start_ns is not None else 'idle'
        return observation, []

    episode = self.episode
    if now_ns - episode['start_ns'] > EPISODE_MAX_NS:
      observation.update(phase='reset', episode_id=episode['id'], reset_reason='episode_timeout')
      self._clear()
      return observation, []
    turn_direction = episode['direction']
    projected_angle = turn_direction * angle
    if projected_angle > episode['peak_angle']:
      episode['peak_angle'], episode['peak_ns'] = projected_angle, now_ns
    if request_torque is not None:
      episode['peak_request_torque'] = max(episode['peak_request_torque'], turn_direction * request_torque)
    if request_angle is not None:
      episode['peak_request_angle'] = max(episode['peak_request_angle'], turn_direction * request_angle)
    if desired_curvature is not None:
      episode['peak_desired_curvature'] = max(episode['peak_desired_curvature'], abs(desired_curvature))
    angle_returning = episode['peak_angle'] - projected_angle >= 3.0
    torque_reducing = bool(torque_mode and request_torque is not None and
                           episode['peak_request_torque'] - turn_direction * request_torque >= 0.08)
    angle_goal_reducing = bool(not torque_mode and request_angle is not None and
                              episode['peak_request_angle'] - turn_direction * request_angle >= 3.0)
    curvature_reducing = bool(desired_curvature is not None and episode['peak_desired_curvature'] > 0.002 and
                             abs(desired_curvature) < episode['peak_desired_curvature'] * 0.7)
    pressed_rising = bool(previous is not None and pressed and not previous['pressed'])
    pressed_falling = bool(previous is not None and not pressed and previous['pressed'])
    deactivated = bool(previous is not None and previous['active'] and not active)
    # begin() keeps short 100 Hz transitions even when this 10 Hz sample misses
    # the instant of override or deactivation. Only current, bounded edges count.
    edge_times = {}
    for edge in transitions:
      timestamp = edge.get('mono_ns')
      if not valid_time(timestamp) or not 0 <= now_ns - timestamp <= MAX_SAMPLE_GAP_NS or timestamp < episode['start_ns']:
        continue
      if edge.get('field') == 'steering_pressed':
        if edge.get('before') is False and edge.get('after') is True:
          pressed_rising = True
          edge_times['press'] = timestamp
        if edge.get('before') is True and edge.get('after') is False:
          pressed_falling = True
          edge_times['release'] = timestamp
      elif edge.get('field') == 'lat_active' and edge.get('before') is True and edge.get('after') is False:
        deactivated = True
        edge_times['deactivation'] = timestamp
    if pressed_rising:
      episode['driver_press_ns'] = edge_times.get('press', now_ns)
    if pressed_falling:
      episode['driver_release_ns'] = edge_times.get('release', now_ns)
    if deactivated:
      episode['lateral_deactivation_ns'] = edge_times.get('deactivation', now_ns)

    reasons = []

    def emit(reason, timestamp=now_ns):
      if reason not in episode['emitted']:
        episode['emitted'].add(reason)
        reasons.append({'reason': reason, 'priority': REASON_PRIORITIES[reason], 'mono_ns': int(timestamp)})

    return_evidence = angle_returning or torque_reducing or angle_goal_reducing or curvature_reducing
    if episode['return_ns'] is None and (return_evidence or pressed_rising or deactivated):
      episode['return_ns'] = now_ns
      emit('turn_return_candidate')
    if pressed_rising:
      emit('turn_driver_steering_intervention', episode['driver_press_ns'])
    if deactivated:
      emit('turn_lateral_deactivation', episode['lateral_deactivation_ns'])

    centerward_rate = -turn_direction * signed_rate if signed_rate is not None else None
    same_sign = bool(request_torque is not None and output_torque is not None and request_torque * output_torque > 0)
    opposed = bool(request_torque is not None and output_torque is not None and
                   abs(request_torque) > 0.05 and abs(output_torque) > 0.05 and request_torque * output_torque < 0)
    same_sign_lag = bool(same_sign and request_torque is not None and output_torque is not None and
                         turn_direction * (output_torque - request_torque) >= 0.08)
    returning = episode['return_ns'] is not None
    slow = bool(returning and speed >= TURN_MIN_SPEED_MPS and projected_angle >= 8 and
                centerward_rate is not None and centerward_rate < 5)
    gap = bool(returning and torque_mode and active and (opposed or same_sign_lag))
    self.slow_start_ns = (self.slow_start_ns or now_ns) if slow else None
    self.gap_start_ns = (self.gap_start_ns or now_ns) if gap else None
    if self.slow_start_ns is not None and now_ns - self.slow_start_ns >= SLOW_RETURN_DWELL_NS:
      emit('return_slow_angle_response')
    if self.gap_start_ns is not None and now_ns - self.gap_start_ns >= GAP_DWELL_NS:
      emit('return_request_output_gap')
    completed = bool(returning and (abs(angle) <= 5 or projected_angle <= 0))
    observation.update({
      'phase': 'complete' if completed else ('return' if returning else 'turn'),
      'episode_id': episode['id'], 'episode_sequence': self.sequence,
      'direction': 'left_positive_angle' if turn_direction > 0 else 'right_negative_angle',
      'start_mono_ns': episode['start_ns'], 'peak_angle_deg': episode['peak_angle'],
      'peak_mono_ns': episode['peak_ns'], 'return_start_mono_ns': episode['return_ns'],
      'elapsed_s': (now_ns - episode['start_ns']) / 1e9,
      'return_elapsed_s': (now_ns - episode['return_ns']) / 1e9 if returning else None,
      'centerward_angle_rate_dps': centerward_rate, 'remaining_angle_deg': abs(angle),
      'angle_returning_from_peak': angle_returning, 'torque_request_reducing': torque_reducing,
      'angle_goal_reducing': angle_goal_reducing, 'curvature_goal_reducing': curvature_reducing,
      'request_output_same_sign': same_sign, 'request_output_opposed': opposed,
      'same_sign_torque_output_lag': same_sign_lag,
      'output_torque_falling_in_turn_direction': bool(output_rate is not None and turn_direction * output_rate < 0),
      'driver_centerward_torque': bool(driver_torque is not None and turn_direction * driver_torque < 0),
      'driver_pressed_on_entry': episode['driver_pressed_on_entry'],
      'driver_press_mono_ns': episode['driver_press_ns'], 'driver_release_mono_ns': episode['driver_release_ns'],
      'lateral_deactivation_mono_ns': episode['lateral_deactivation_ns'],
      'slow_angle_response_observed': slow, 'request_output_gap_observed': gap,
      'reasons': sorted(episode['emitted']),
    })
    if completed:
      self.episode = None
      self.entry_start_ns = None
      self.entry_peak = None
      self.slow_start_ns = self.gap_start_ns = None
    return observation, reasons
