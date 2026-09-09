"""Passive, bounded KA4 observations for full rlog; never a control decision.

Schema 1 deliberately distinguishes cached decoded CAN, submitted CAN, planner
shadows and measured vehicle state. Submission is not ECU acceptance. No VIN,
dongle/account ID, GPS, arbitrary Params, raw CAN payload or network IO is logged.
"""
import math
import re
from numbers import Integral, Real

from opendbc.car.hyundai.values import CAR
from openpilot.common.swaglog import cloudlog


SAMPLE_NS = 100_000_000
EDGE_NS = 50_000_000
TOPIC_COOLDOWN_NS = 30_000_000_000
SESSION_NS = 60_000_000_000
DIAGNOSTICS_VERSION = 'dk-vehicle-diag-v1'
MAX_TRANSITIONS = 32
PARAM_KEYS = (
  'PathOffset', 'AdjustLaneOffset', 'UseLaneLineSpeed', 'SteerActuatorDelay', 'SteerRatioRate',
  'MaxAngleFrames', 'CustomSteerMax', 'CustomSteerDeltaUp', 'CustomSteerDeltaDown', 'CustomSteerDeltaUpLC',
  'CustomSteerDeltaDownLC', 'LongActuatorDelay', 'VEgoStopping', 'StoppingAccel', 'StopDistanceCarrot',
  'TrafficLightDetectMode', 'ExperimentalMode', 'AlphaLongitudinalEnabled', 'CanfdHDA2',
  'HyundaiCameraSCC', 'EnableRadarTracks', 'CruiseButtonTest1', 'CruiseButtonTest2', 'CruiseButtonTest3',
  'AutoCruiseControl', 'SpeedFromPCM', 'Ka4StockSccStandstillRearm',
)
CONTROLLER_FIELDS = (
  'frame', 'apply_angle_last', 'apply_torque_last', 'angle_limit_counter', 'lkas_max_torque',
  'angle_max_torque', 'steering_pressed_prev', 'recovering_from_override', 'full_recovery_frames',
  'repeated_override_count', 'override_latched', 'override_release_frames', 'driver_torque_filtered',
  'driver_torque_filtered_prev', 'pre_override_frames', 'steerDeltaUp', 'steerDeltaDown',
  'last_button_frame', 'last_cancel_frame', 'button_wait', 'button_spamming_count',
  'cruise_buttons_msg_cnt', 'dk_ka4_runtime_branch', 'ka4_stock_scc_standstill_rearm',
  'stock_scc_stop_start_frame', 'stock_scc_near_zero_frames', 'stock_scc_stopped_lead_frames',
  'stock_scc_keepalive_pending', 'stock_scc_keepalive_press_frames', 'stock_scc_keepalive_requested',
  'stock_scc_keepalive_request_count', 'stock_scc_button_source_counter', 'stock_scc_last_keepalive_frame',
  'stock_scc_keepalive_warning_recovery', 'stock_scc_warning_recovery_requested',
  'stock_scc_alert_abort_latched', 'stock_scc_resume_alert_suppressed', 'accel_last', 'accel_value_last',
)
RAW_FIELDS = {
  'scc_control': ('SCC_CONTROL', ('COUNTER', 'ACCMode', 'MainMode_ACC', 'InfoDisplay', 'DriverAlert',
    'SysFailState', 'TakeOverReq', 'StopReq', 'aReqRaw', 'aReqValue', 'ACC_ObjDist', 'ACC_ObjRelSpd',
    'HUD_LEAD_INFO', 'VSetDis', 'DISTANCE_SETTING', 'JerkUpperLimit', 'JerkLowerLimit')),
  'adrv_0x161': ('ADRV_0x161', ('COUNTER', 'ALERTS_1', 'ALERTS_2', 'ALERTS_3', 'ALERTS_4', 'ALERTS_5',
    'SOUNDS_1', 'SOUNDS_2', 'SOUNDS_3', 'SOUNDS_4', 'MUTE', 'LFA_ICON', 'HDA_ICON', 'LKA_ICON')),
  'ccnc_0x162': ('CCNC_0x162', ('COUNTER', 'FAULT_FSS', 'FAULT_FCA', 'FAULT_LSS', 'FAULT_SCC',
    'FAULT_LFA', 'FAULT_HDA', 'FAULT_LCA', 'FAULT_HDP', 'FAULT_DAS', 'FAULT_ESS')),
  'lfahda_cluster': ('LFAHDA_CLUSTER', ('COUNTER', 'HDA_OptUsmSta', 'LFA_OptUsmSta', 'HDA_CntrlModSta',
    'HDA_InfoPUDis', 'HDA_InfoPUDis1', 'HDA_LFA_SymSta', 'HDA_LFA_WrnSnd')),
  'mdps': ('MDPS', ('COUNTER', 'LKA_ACTIVE', 'LKA_FAULT', 'LFA2_ACTIVE', 'LFA2_FAULT',
    'STEERING_OUT_TORQUE', 'STEERING_COL_TORQUE', 'STEERING_ANGLE', 'STEERING_ANGLE_2')),
  'tcs': ('TCS', ('COUNTER', 'aBasis', 'ACCEL_REF_ACC', 'ACCEnable', 'ACC_REQ', 'ESC_StdStillVal',
    'DriverBraking', 'DriverBrakingLowSens', 'BrakeLight', 'ESC_PrkBrkActvSta', 'SCC_ReqLimSta')),
  'cruise_buttons_msg': ('CRUISE_BUTTONS_ALT', ('COUNTER', 'CRUISE_BUTTONS', 'ADAPTIVE_CRUISE_MAIN_BTN',
    'NORMAL_CRUISE_MAIN_BTN', 'LFA_BTN')),
}
SERVICES = ('carControl', 'longitudinalPlan', 'radarState', 'modelV2', 'lateralPlan', 'controlsState')


def scalar(value):
  """Copy only bounded JSON scalar values, with non-finite numbers missing."""
  if value is None or isinstance(value, bool):
    return value
  if isinstance(value, Integral):
    return int(value)
  if isinstance(value, Real):
    return float(value) if math.isfinite(value) else None
  if isinstance(value, str):
    return value[:96]
  return None


def field(obj, name):
  return scalar(obj.get(name) if isinstance(obj, dict) else getattr(obj, name, None))


def fields(obj, names):
  return {name: field(obj, name) for name in names}


def numbers(values, limit=17):
  return [scalar(values[i]) for i in range(min(len(values), limit))] if values is not None else []


def signal(cache, key):
  value = cache.get(key) if isinstance(cache, dict) else None
  # Hyundai's cached ALT button message sometimes contains vl_all lists.
  if isinstance(value, (list, tuple)):
    value = value[0] if value else None
  return scalar(value)


def raw_snapshot(carstate, now_ns):
  result = {}
  for attr, (message, names) in RAW_FIELDS.items():
    if attr == 'cruise_buttons_msg' and getattr(carstate, 'cruise_btns_msg_canfd', None) == 'CRUISE_BUTTONS':
      message = 'CRUISE_BUTTONS'
    cache = getattr(carstate, attr, None)
    values = {name: signal(cache, name) for name in names}
    sources = []
    for parser_name in ('cp', 'cp_cam', 'cp_alt'):
      parser = getattr(carstate, parser_name, None)
      timestamps = getattr(parser, 'ts_nanos', {}).get(message, {})
      last_ns = max((timestamps.get(name, 0) for name in names), default=0)
      if last_ns > 0:
        sources.append({'parser': parser_name, 'bus': field(parser, 'bus'), 'mono_ns': int(last_ns),
                        'age_ms': (now_ns - last_ns) / 1e6 if now_ns >= last_ns else None})
    result[attr] = {'values': values, 'message': message, 'missing': not bool(cache),
                    'missing_fields': [name for name, value in values.items() if value is None],
                    'packet_sources': sources,
                    'provenance': 'cached_decoded_signals_may_be_modified_by_controller'}
  return result


def service(sm, name):
  try:
    return sm[name]
  except (KeyError, TypeError):
    return None


def freshness(sm, now_ns):
  result = {}
  for name in SERVICES:
    ts = getattr(sm, 'logMonoTime', {}).get(name, 0)
    result[name] = {'valid': bool(getattr(sm, 'valid', {}).get(name, False)),
                    'alive': bool(getattr(sm, 'alive', {}).get(name, False)),
                    'mono_ns': int(ts), 'age_ms': (now_ns - ts) / 1e6 if ts > 0 and now_ns >= ts else None}
  return result


def actuators(obj):
  return {'angle_deg': field(obj, 'steeringAngleDeg'), 'torque': field(obj, 'torque'),
          'curvature': field(obj, 'curvature'), 'accel_mps2': field(obj, 'accel')}


def make_dk_vehicle_diagnostics(CP, params, logger=None):
  """Fail-open construction, with every Params read confined to initialization."""
  try:
    branch = params.get('GitBranch')
    branch = branch.decode('utf-8', errors='replace') if isinstance(branch, bytes) else str(branch or '')
    if branch.strip() != 'dkcarrot-wip' or CP.carFingerprint != CAR.KIA_CARNIVAL_4TH_GEN:
      return None
    commit = params.get('GitCommit')
    commit = commit.decode('ascii', errors='replace') if isinstance(commit, bytes) else str(commit or '')
    initial_params = {}
    for key in PARAM_KEYS:
      try:
        value = params.get(key)
        value = value.decode('ascii', errors='replace') if isinstance(value, bytes) else value
        # Only numeric configuration. Never pass through arbitrary stored text.
        initial_params[key] = scalar(float(value)) if value is not None else None
      except (KeyError, ValueError, TypeError):
        initial_params[key] = None
    metadata = {'branch': 'dkcarrot-wip', 'commit': commit if re.fullmatch('[0-9a-fA-F]{7,64}', commit) else None,
                'diagnostics_version': DIAGNOSTICS_VERSION, 'params_snapshot_scope': 'initial_numeric_raw_params_only',
                'cp': fields(CP, ('flags', 'extFlags', 'pcmCruise', 'openpilotLongitudinalControl',
                  'radarUnavailable', 'mass', 'steerRatio', 'wheelbase', 'centerToFront',
                  'steerActuatorDelay', 'minSteerSpeed', 'passive', 'dashcamOnly')),
                'initial_params': initial_params}
    metadata['cp']['steerControlType'] = str(CP.steerControlType)
    metadata['cp']['lateralTuning'] = str(CP.lateralTuning.which())
    metadata['cp']['carFingerprint'] = str(CP.carFingerprint)
    return DkVehicleDiagnostics(metadata, logger or cloudlog.debug)
  except Exception:
    return None


class DkVehicleDiagnostics:
  """No reference to Params or sockets; observer errors never escape to card."""
  def __init__(self, metadata, logger):
    self.metadata = metadata
    self.logger = logger
    self.session_sent = False
    self.last_session_ns = None
    self.last_sample_ns = None
    self.last_edge = None
    self.transitions = []
    self.transitions_dropped = 0
    self.pending_topics = set()
    self.last_topic_ns = {}
    self.dwell_start = {}
    self.tx_counts = {}
    self.tx_buttons = {}
    self.tx_counters = {}
    self.frames = self.samples = self.rate_skipped = self.errors = 0
    self.previous_motion = None
    self.unwind_until_ns = 0

  def _candidate(self, topic, now_ns):
    last = self.last_topic_ns.get(topic)
    if last is None or now_ns - last >= TOPIC_COOLDOWN_NS:
      self.pending_topics.add(topic)

  def _transition(self, label, before, after, now_ns, topic):
    self._candidate(topic, now_ns)
    if len(self.transitions) < MAX_TRANSITIONS:
      self.transitions.append({'field': label, 'before': before, 'after': after, 'mono_ns': int(now_ns)})
    else:
      self.transitions_dropped += 1

  def _dwell(self, name, active, now_ns, dwell_ns, topic):
    if not active:
      self.dwell_start.pop(name, None)
    elif now_ns - self.dwell_start.setdefault(name, now_ns) >= dwell_ns:
      self._candidate(topic, now_ns)

  def begin(self, CS, CC, CI, now_ns):
    """Small primitive copy before apply; never log or access Params here."""
    try:
      self.frames += 1
      state = CI.CS
      cruise = getattr(CC, 'cruiseControl', None)
      edge = {
        'enabled': field(CC, 'enabled'), 'lat_active': field(CC, 'latActive'),
        'standstill': field(CS, 'standstill'), 'cruise_standstill': field(CS.cruiseState, 'standstill'),
        'resume': field(cruise, 'resume'), 'steering_pressed': field(CS, 'steeringPressed'),
        'steer_fault_temporary': field(CS, 'steerFaultTemporary'), 'steer_fault_permanent': field(CS, 'steerFaultPermanent'),
        'acc_faulted': field(CS, 'accFaulted'),
        'scc_info_display': signal(getattr(state, 'scc_control', None), 'InfoDisplay'),
        'adrv_alert_5': signal(getattr(state, 'adrv_0x161', None), 'ALERTS_5'),
        'ccnc_fault_hda': signal(getattr(state, 'ccnc_0x162', None), 'FAULT_HDA'),
        'ccnc_fault_lfa': signal(getattr(state, 'ccnc_0x162', None), 'FAULT_LFA'),
      }
      for attr, prefix, names in (
        ('scc_control', 'scc', ('SysFailState', 'TakeOverReq', 'DriverAlert')),
        ('adrv_0x161', 'adrv', ('ALERTS_1', 'ALERTS_2', 'ALERTS_3', 'ALERTS_4', 'SOUNDS_1', 'SOUNDS_2', 'SOUNDS_3', 'SOUNDS_4')),
        ('lfahda_cluster', 'lfahda', ('HDA_InfoPUDis', 'HDA_InfoPUDis1', 'HDA_LFA_WrnSnd')),
        ('ccnc_0x162', 'ccnc', ('FAULT_DAS', 'FAULT_SCC', 'FAULT_LSS')),
        ('mdps', 'mdps', ('LKA_FAULT', 'LFA2_FAULT')),
      ):
        for name in names:
          edge[f'{prefix}_{name}'] = signal(getattr(state, attr, None), name)
      if self.last_edge is not None:
        for name, value in edge.items():
          if value != self.last_edge[name]:
            topic = 'resume' if name in ('standstill', 'cruise_standstill', 'resume', 'scc_info_display', 'adrv_alert_5') else 'engage_warning'
            self._transition(name, self.last_edge[name], value, now_ns, topic)
      elif edge['enabled']:
        self._candidate('engage_warning', now_ns)
      self.last_edge = edge
      self._dwell('stopped', edge['enabled'] and edge['standstill'], now_ns, 3_000_000_000, 'resume')
      elapsed = now_ns - self.last_sample_ns if self.last_sample_ns is not None else SAMPLE_NS
      if elapsed < EDGE_NS or (elapsed < SAMPLE_NS and not self.transitions and not self.pending_topics):
        self.rate_skipped += 1
        return None
      self.last_sample_ns = now_ns
      return {'mono_ns': int(now_ns), 'controller_before': fields(getattr(CI, 'CC', None), CONTROLLER_FIELDS),
              'raw_before': raw_snapshot(state, now_ns)}
    except Exception:
      self.errors += 1
      return None

  def _submitted(self, can_sends):
    # Count every 100 Hz submission even when the detail sample is skipped.
    # Decode ONLY the fixed Hyundai CAN-FD button bits, not arbitrary payloads.
    for address, dat, bus in can_sends:
      key = (int(address), int(bus))
      if key not in self.tx_counts and len(self.tx_counts) >= 64:
        continue
      self.tx_counts[key] = self.tx_counts.get(key, 0) + 1
      if address == 0x1AA and len(dat) == 16:
        button, counter = (dat[4] >> 4) & 7, dat[2]
      elif address == 0x1CF and len(dat) == 8:
        button, counter = dat[2] & 7, (dat[1] >> 4) & 15
      else:
        continue
      bkey = (int(address), int(bus), int(button))
      self.tx_buttons[bkey] = self.tx_buttons.get(bkey, 0) + 1
      prior = self.tx_counters.setdefault(key, [int(counter), int(counter)])
      prior[1] = int(counter)

  def finish(self, token, CS, CC, CI, sm, output, can_sends, now_ns):
    """Called only after sendcan submission; never writes a control input."""
    try:
      self._submitted(can_sends)
      if token is None:
        return
      sample = self._sample(token, CS, CC, CI, sm, output, now_ns)
      if self.last_session_ns is None or now_ns - self.last_session_ns >= SESSION_NS:
        self.logger({'event': 'dk_vehicle_diag', 'schema': 1, 'kind': 'session', 'mono_ns': int(now_ns),
                     'topics': [], 'observer_enabled': True, **self.metadata})
        self.session_sent = True
        self.last_session_ns = now_ns
      self.logger(sample)
      self.samples += 1
      for topic in self.pending_topics:
        self.last_topic_ns[topic] = now_ns
      self.pending_topics.clear()
      self.transitions.clear()
      self.tx_counts.clear()
      self.tx_buttons.clear()
      self.tx_counters.clear()
    except Exception:
      self.errors += 1

  def _sample(self, token, CS, CC, CI, sm, output, now_ns):
    cruise = getattr(CC, 'cruiseControl', None)
    request = actuators(CC.actuators)
    request.update({'enabled': field(CC, 'enabled'), 'lat_active': field(CC, 'latActive'), 'long_active': field(CC, 'longActive'),
                    'resume': field(cruise, 'resume'), 'cancel': field(cruise, 'cancel'), 'override': field(cruise, 'override')})
    car = {'speed_mps': field(CS, 'vEgo'), 'raw_speed_mps': field(CS, 'vEgoRaw'), 'accel_mps2': field(CS, 'aEgo'),
            'steering_angle_deg': field(CS, 'steeringAngleDeg'), 'steering_rate_dps': field(CS, 'steeringRateDeg'),
            'driver_torque': field(CS, 'steeringTorque'), 'eps_torque': field(CS, 'steeringTorqueEps'),
            'standstill': field(CS, 'standstill'), 'cruise_standstill': field(CS.cruiseState, 'standstill'),
            'cruise_enabled': field(CS.cruiseState, 'enabled'), 'cruise_available': field(CS.cruiseState, 'available'),
            'can_valid': field(CS, 'canValid'), 'brake_pressed': field(CS, 'brakePressed'), 'gas_pressed': field(CS, 'gasPressed'),
            'brake_hold_active': field(CS, 'brakeHoldActive'), 'parking_brake': field(CS, 'parkingBrake'),
            'steering_pressed': field(CS, 'steeringPressed'), 'steer_fault_temporary': field(CS, 'steerFaultTemporary'),
            'steer_fault_permanent': field(CS, 'steerFaultPermanent'), 'acc_faulted': field(CS, 'accFaulted')}
    plan = service(sm, 'longitudinalPlan')
    speeds = numbers(getattr(plan, 'speeds', None), 17)
    plan_source = getattr(plan, 'longitudinalPlanSource', None)
    planner = {'should_stop': field(plan, 'shouldStop'), 'final_speed_mps': speeds[-1] if speeds else None,
                'speeds_mps': speeds, 'a_target_mps2': field(plan, 'aTarget'), 'v_target_mps': field(plan, 'vTargetNow'),
                'source': str(plan_source)[:32] if plan_source is not None else None, 'cruise_target_kph': field(plan, 'cruiseTarget'),
                'has_lead': field(plan, 'hasLead')}
    lead = getattr(service(sm, 'radarState'), 'leadOne', None)
    radar = {'status': field(lead, 'status'), 'radar': field(lead, 'radar'), 'd_rel_m': field(lead, 'dRel'),
              'v_rel_mps': field(lead, 'vRel'), 'v_lead_mps': field(lead, 'vLead'), 'a_lead_mps2': field(lead, 'aLeadK'),
              'track_id': field(lead, 'radarTrackId')}
    lateral_plan = service(sm, 'lateralPlan')
    model = service(sm, 'modelV2')
    control = service(sm, 'controlsState')
    lateral = {'lane_width_m': field(lateral_plan, 'laneWidth'), 'use_lane_lines': field(lateral_plan, 'useLaneLines'),
                'static_offset_m': field(lateral_plan, 'staticPathOffset'), 'dynamic_offset_m': field(lateral_plan, 'dynamicLaneOffset'),
                'path_y_m': numbers(getattr(lateral_plan, 'dPathPoints', None)),
                'path_before_static_y_m': numbers(getattr(lateral_plan, 'pathBeforeStaticOffset', None)),
                'plan_curvatures': numbers(getattr(lateral_plan, 'curvatures', None)),
                'plan_curvature_rates': numbers(getattr(lateral_plan, 'curvatureRates', None)),
                'lane_probabilities': numbers(getattr(model, 'laneLineProbs', None), 4),
                'lane_lines': [{'x_m': numbers(getattr(line, 'x', None), 9), 'y_m': numbers(getattr(line, 'y', None), 9)}
                               for line in list(getattr(model, 'laneLines', []))[:4]],
                'actual_curvature': field(control, 'curvature'), 'desired_curvature': field(control, 'desiredCurvature')}
    lateral_state = getattr(control, 'lateralControlState', None)
    if lateral_state is not None:
      union_name = lateral_state.which()
      lateral['controller_type'] = str(union_name)
      lateral['controller'] = fields(getattr(lateral_state, union_name), ('active', 'saturated', 'output', 'p', 'i', 'f',
        'steeringAngleDeg', 'steeringAngleDesiredDeg', 'angleError', 'error', 'actualLateralAccel', 'desiredLateralAccel'))
    base_resume = bool(request['enabled'] and car['cruise_standstill'] and speeds)
    shadow = {'interpretation': 'hypothesis_not_vehicle_response',
               'legacy_resume': base_resume and planner['final_speed_mps'] is not None and planner['final_speed_mps'] > 0.1,
               'should_stop_resume': base_resume and planner['should_stop'] is False,
               'gentle_decel_mps2': None, 'gentle_decel_assumed_gap_m': 8.0,
               'gentle_decel_model': 'constant_lead_speed_close_rate_squared_over_twice_available_gap_clamped_minus1p5_to0'}
    distance, relative = radar['d_rel_m'], radar['v_rel_mps']
    if radar['status'] and distance is not None and relative is not None and distance > 8.0:
      shadow['gentle_decel_mps2'] = -min(1.5, max(0.0, -relative)**2 / (2.0 * (distance - 8.0)))
    previous = self.previous_motion
    car['jerk_mps3'] = None
    shadow['requested_angle_rate_dps'] = shadow['output_angle_rate_dps'] = None
    output_values = actuators(output)
    if previous is not None and 0 < now_ns - previous[0] <= 1_000_000_000:
      dt = (now_ns - previous[0]) / 1e9
      if car['accel_mps2'] is not None and previous[1] is not None:
        car['jerk_mps3'] = (car['accel_mps2'] - previous[1]) / dt
      for name, current, old in (('requested_angle_rate_dps', request['angle_deg'], previous[2]),
                                  ('output_angle_rate_dps', output_values['angle_deg'], previous[3])):
        if current is not None and old is not None:
          shadow[name] = (current - old) / dt
      if request['enabled'] and car['accel_mps2'] is not None and previous[1] is not None and car['accel_mps2'] < -0.3 <= previous[1]:
        self._candidate('braking', now_ns)
    angle_error = abs(request['angle_deg'] - car['steering_angle_deg']) if request['angle_deg'] is not None and car['steering_angle_deg'] is not None else 0
    enabled = request['enabled'] is True
    curve_active = bool(enabled and car['speed_mps'] is not None and car['speed_mps'] > 3 and
                        ((request['curvature'] is not None and abs(request['curvature']) > 0.002) or angle_error > 5))
    self._dwell('curve', curve_active, now_ns, 300_000_000, 'curve')
    requested_angle, measured_angle = request['angle_deg'], car['steering_angle_deg']
    if (enabled and previous is not None and requested_angle is not None and previous[2] is not None and
        abs(previous[2]) > 5 and abs(requested_angle) < abs(previous[2])):
      self.unwind_until_ns = now_ns + 10_000_000_000
    # Keep observing a lagging return after a step to center, even when the
    # subsequent demand stays fixed at zero instead of declining every sample.
    centerward_lag = (requested_angle is not None and measured_angle is not None and
                     abs(requested_angle) + 3 < abs(measured_angle))
    unwind_active = bool(enabled and now_ns <= self.unwind_until_ns and centerward_lag)
    self._dwell('unwind', unwind_active, now_ns, 200_000_000, 'unwind')
    # Ongoing descriptive conditions, unlike topics which are cooldown-limited
    # capture triggers. They are hypotheses, not vehicle/controller state flags.
    shadow['curve_active'] = curve_active
    shadow['unwind_active'] = unwind_active
    self._dwell('closing', enabled and radar['status'] and distance is not None and relative is not None and
                0 < distance < 50 and relative < -0.8, now_ns, 300_000_000, 'braking')
    self.previous_motion = (now_ns, car['accel_mps2'], request['angle_deg'], output_values['angle_deg'])
    return {'event': 'dk_vehicle_diag', 'schema': 1, 'kind': 'sample', 'mono_ns': int(now_ns),
            'before_mono_ns': token['mono_ns'], 'topics': sorted(self.pending_topics),
            'interpretation': 'candidate_observation_not_diagnosis',
            'counters': {'frames': self.frames, 'samples': self.samples + 1, 'rate_skipped': self.rate_skipped,
                         'errors': self.errors, 'transitions_dropped': self.transitions_dropped},
            'transitions': list(self.transitions), 'car': car, 'request': request, 'output': output_values,
            'controller_before': token['controller_before'], 'controller_after': fields(getattr(CI, 'CC', None), CONTROLLER_FIELDS),
            'raw_before': token['raw_before'], 'raw_after': raw_snapshot(CI.CS, now_ns),
            'planner': planner, 'radar': radar, 'lateral': lateral, 'freshness': freshness(sm, now_ns), 'shadow': shadow,
            'submitted_can': {'interpretation': 'host_submission_not_panda_or_ecu_acceptance',
              'count': sum(self.tx_counts.values()), 'addresses': [
                {'address': addr, 'bus': bus, 'count': count} for (addr, bus), count in sorted(self.tx_counts.items())],
              'buttons': [{'address': addr, 'bus': bus, 'button': button, 'count': count}
                          for (addr, bus, button), count in sorted(self.tx_buttons.items())],
              'button_counters': [{'address': addr, 'bus': bus, 'first': val[0], 'last': val[1]}
                                  for (addr, bus), val in sorted(self.tx_counters.items())]}}
