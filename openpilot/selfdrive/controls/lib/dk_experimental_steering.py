"""Opt-in KA4 model-path preview, upstream of all existing steering limits.

The short-horizon responses below are deliberately an uncertainty envelope,
NOT an identified KA4 plant. A command is admitted only when position and
heading tracking improve without cutting the near path in every scenario.
Nothing here writes Params, sends CAN, changes speed, or raises steering limits.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature


TIMES = np.arange(1, 21, dtype=float) * 0.1
MAX_EXTRA_ACCEL = 0.35  # m/s^2 relative to an independent legacy target
MAX_EXTRA_JERK = 0.5  # m/s^3; also applied when withdrawing the experiment
MAX_EXTRA_CURVATURE = 0.004
MIN_SPEED = 4.0
MAX_SPEED = 16.7
SOURCE_MAX_AGE = {"modelV2": .15, "carState": .08, "carOutput": .08,
                  "livePose": .10, "liveParameters": 2.5, "liveDelay": 2.5, "liveCalibration": 2.5}


@dataclass(frozen=True)
class PreviewPath:
  x: np.ndarray
  y: np.ndarray
  heading: np.ndarray
  curvature: np.ndarray


@dataclass(frozen=True)
class PreviewState:
  speed: float
  measured_curvature: float
  steering_curvature: float
  actuator_delay: float
  requested_torque: float = 0.0
  output_torque: float = 0.0
  # Existing lateral MPC's v_lateral / yaw_rate coefficient, not the camera
  # mounting offset. Zero is only appropriate for a rear-axle reference model.
  rotation_radius: float = 0.0


@dataclass(frozen=True)
class PreviewResult:
  target: float
  phase: str
  accepted: bool
  reason: str
  baseline_cost: float = 0.0
  candidate_cost: float = 0.0
  baseline_max_error: float = 0.0
  candidate_max_error: float = 0.0


def model_path(model, speed: float, rotation_radius: float = 0.) -> PreviewPath:
  """Resample geometry by distance, not the model's assumed future speed.

  Stock SCC need not execute model velocity predictions. Its recorded current
  speed is used to traverse the geometric path; yaw-rate is divided by *model*
  speed before that spatial resampling. No lane confidence is required.
  """
  arrays = [np.asarray(values, dtype=float) for values in (
    model.position.x, model.position.y, model.orientation.z,
    model.orientationRate.z, model.velocity.x, model.velocity.y, model.velocity.z,
  )]
  if any(a.shape != (33,) or not np.isfinite(a).all() for a in arrays):
    raise ValueError("incomplete_geometry")
  x, y, heading, yaw_rate, vx, vy, vz = arrays
  distance = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
  model_speed = np.sqrt(vx * vx + vy * vy + vz * vz)
  if (not math.isfinite(speed) or not MIN_SPEED <= speed <= MAX_SPEED or
      not math.isfinite(rotation_radius) or not 0. <= rotation_radius <= 5. or
      np.any(np.diff(distance) <= 0) or distance[-1] < speed * TIMES[-1] or
      np.any(model_speed[distance <= speed * TIMES[-1] + speed * .1] < 1.) or
      abs(x[0]) > .2 or abs(y[0]) > .2 or abs(heading[0]) > .05):
    raise ValueError("unsupported_geometry")
  s = speed * TIMES
  result = PreviewPath(*(np.interp(s, distance, a) for a in (x, y, np.unwrap(heading), yaw_rate / np.maximum(model_speed, 1.))))
  # Reject a conflicting position/heading/yaw description, not low lane-line
  # confidence. These loose bounds are data-integrity checks, not road margins.
  tangent = np.arctan2(np.diff(result.y), np.diff(result.x))
  geometric_curvature = np.diff(result.heading) / (speed * .1)
  expected_curvature = (result.curvature[:-1] + result.curvature[1:]) * .5
  # Body heading differs from the trajectory tangent when lateral velocity is
  # nonzero. Here curvature = yaw_rate / *total* model speed, so sin(sideslip)
  # equals radius * curvature. This is a loose consistency guard, not a precise
  # slip estimator (the model's future speed may differ from stock SCC).
  tangent_heading = ((result.heading[:-1] + result.heading[1:]) * .5 +
                     np.arcsin(np.clip(rotation_radius * expected_curvature, -1., 1.)))
  if (np.max(np.abs(result.heading)) > 1.0 or np.max(np.abs(result.curvature)) > .15 or
      np.max(np.abs(tangent - tangent_heading)) > .20 or
      np.max(np.abs(geometric_curvature - expected_curvature)) > .025):
    raise ValueError("inconsistent_geometry")
  return result


def path_phase(path: PreviewPath, baseline: float) -> str:
  near = float(np.mean(path.curvature[:5]))
  middle = float(np.mean(path.curvature[6:11]))
  far = float(np.mean(path.curvature[15:]))
  dominant = middle if abs(middle) > .0015 else near if abs(near) > .0015 else baseline
  sign = float(np.sign(dominant))
  if np.any(path.curvature * sign < -.0015):
    return "changing_direction"
  if max(abs(near), abs(middle), abs(far), abs(baseline)) < .0015:
    return "straight"
  if abs(far) < max(abs(near), abs(middle)) * .65 and abs(far) < abs(near) + .0005:
    return "exit"
  if abs(middle) > abs(near) * 1.25 + .0005 and abs(far) >= abs(middle) * .85:
    return "entry"
  return "hold"


def lateral_rotation_radius(CP, speed: float) -> float:
  """Use the same quasi-steady bicycle kinematics as LateralPlanner.

  A CG/body heading is not generally tangent to its position trajectory:
  lateral velocity contributes r * yaw_rate in addition to v * sin(heading).
  Missing/invalid CP must reject the optional preview, not silently assume
  a different reference point or a guessed KA4 mounting distance.
  """
  wheelbase, front, mass, rear_stiffness = (float(v) for v in
      (CP.wheelbase, CP.centerToFront, CP.mass, CP.tireStiffnessRear))
  if (not all(math.isfinite(v) for v in (wheelbase, front, mass, rear_stiffness, speed)) or
      not 0. < front < wheelbase <= 5. or mass <= 0. or rear_stiffness <= 0. or speed < 0.):
    raise ValueError("invalid_vehicle_geometry")
  return max(0., wheelbase - front - front * mass / (wheelbase * rear_stiffness) * speed ** 2)


def _prediction_errors(path: PreviewPath, state: PreviewState, targets: np.ndarray, delay: float, tau: float):
  """Same future feedforward tail for every first-command counterfactual."""
  k = np.full(targets.shape, state.measured_curvature)
  x, y, heading = (np.zeros(targets.shape) for _ in range(3))
  errors, heading_errors = [], []
  alpha = -math.expm1(-.1 / tau)
  for i, t in enumerate(TIMES):
    # The previous measured response persists until a new request can act.
    command = state.measured_curvature if t <= delay else targets if t <= delay + .5 else path.curvature[i]
    next_k = k + alpha * (command - k)
    next_heading = heading + state.speed * .1 * (k + next_k) * .5
    midpoint = (heading + next_heading) * .5
    angle_delta = next_heading - heading
    x += state.speed * .1 * np.cos(midpoint) - state.rotation_radius * np.sin(midpoint) * angle_delta
    y += state.speed * .1 * np.sin(midpoint) + state.rotation_radius * np.cos(midpoint) * angle_delta
    heading, k = next_heading, next_k
    errors.append(-(x - path.x[i]) * np.sin(path.heading[i]) + (y - path.y[i]) * np.cos(path.heading[i]))
    heading_errors.append(heading - path.heading[i])
  return np.array(errors).T, np.array(heading_errors).T


def preview_candidate(path: PreviewPath, state: PreviewState, baseline: float) -> PreviewResult:
  """Pure bounded target computation; driver input is not an activation signal."""
  finite = (state.speed, state.measured_curvature, state.steering_curvature, state.actuator_delay,
            state.requested_torque, state.output_torque, state.rotation_radius, baseline)
  if (not all(math.isfinite(v) for v in finite) or not MIN_SPEED <= state.speed <= MAX_SPEED or
      not .05 <= state.actuator_delay <= .6 or
      not 0. <= state.rotation_radius <= 5. or
      abs(state.measured_curvature - state.steering_curvature) * state.speed ** 2 > 1.5 or
      any(a.shape != TIMES.shape or not np.isfinite(a).all() for a in (path.x, path.y, path.heading, path.curvature))):
    return PreviewResult(baseline, "unavailable", False, "invalid_state")
  phase = path_phase(path, baseline)
  if phase in ("straight", "changing_direction"):
    return PreviewResult(baseline, phase, False, phase)
  limit = min(MAX_EXTRA_CURVATURE, MAX_EXTRA_ACCEL / state.speed ** 2)
  targets = baseline + np.linspace(-limit, limit, 17)
  # No speculative direction reversal or larger existing acceleration envelope.
  same_direction = targets * baseline >= 0 if abs(baseline) > .0005 else targets * np.mean(path.curvature) >= 0
  if phase in ("entry", "hold"):
    same_direction &= np.abs(targets) >= abs(baseline)
  elif phase == "exit":
    same_direction &= np.abs(targets) <= abs(baseline)
  allowed = same_direction.copy()
  costs = np.zeros(targets.shape)
  max_errors = np.zeros(targets.shape)
  baseline_idx = len(targets) // 2
  # Normalized CAN request/output discrepancy bounds additional uncertainty;
  # this is NOT an EPS torque/curvature conversion or a learned physical delay.
  slew_uncertainty = min(.20, abs(state.requested_torque - state.output_torque) * 1.35)
  scenarios = ((max(.05, state.actuator_delay - .08), .18),
               (state.actuator_delay, .24),
               (min(.8, state.actuator_delay + .08 + slew_uncertainty), .30))
  for delay, tau in scenarios:
    error, heading_error = _prediction_errors(path, state, targets, delay, tau)
    cost = np.mean(error ** 2 + (2.5 * heading_error) ** 2, axis=1)
    max_error = np.max(np.abs(error), axis=1)
    # A heading-only improvement must not trade away the remaining near curve.
    near = TIMES <= 1.2
    allowed &= np.all(np.abs(error[:, near]) <= np.abs(error[baseline_idx, near]) + .02, axis=1)
    allowed &= max_error <= max_error[baseline_idx] + .01
    allowed &= cost <= cost[baseline_idx] + 1e-12
    costs += cost
    max_errors = np.maximum(max_errors, max_error)
  allowed[baseline_idx] = True
  index = int(np.argmin(np.where(allowed, costs, np.inf)))
  accepted = bool(index != baseline_idx and costs[index] < costs[baseline_idx] * .995)
  return PreviewResult(float(targets[index]) if accepted else baseline, phase, accepted,
                       "geometry_improves" if accepted else "no_robust_improvement",
                       float(costs[baseline_idx]), float(costs[index]),
                       float(max_errors[baseline_idx]), float(max_errors[index]))


class DkExperimentalSteering:
  """100 Hz adapter; expensive geometry is evaluated once per new model epoch."""
  def __init__(self, logger=None, clock=time.monotonic_ns):
    self.logger, self.clock = logger, clock
    self.legacy_curvature = 0.0
    self.delta = 0.0
    self.target_delta = 0.0
    self.previous_request = 0.0
    self.last_model_ns = 0
    self.last_frame_id = None
    self.phase = "unavailable"
    self.phase_frames = 0
    self.last_log_ns = 0
    self.phase_projection_count = 0
    self.record = None

  def _withdraw(self, reason):
    self.target_delta = 0.0
    self.phase_frames = 0
    self.record = {"reason": reason, "phase": "unavailable", "accepted": False}

  def update_from_controls(self, controls, CC, baseline: float, delay: float) -> float:
    """Fail closed to the independent legacy target, with bounded withdrawal."""
    try:
      CS, sm = controls.sm["carState"], controls.sm
      speed, roll = float(CS.vEgo), float(sm["liveParameters"].roll)
      if not all(math.isfinite(v) for v in (speed, roll, baseline, self.legacy_curvature)):
        self.delta = 0.
        self._withdraw("nonfinite_baseline")
        self.legacy_curvature = baseline if math.isfinite(baseline) else 0.
        return baseline
      self.legacy_curvature = clip_curvature(speed, self.legacy_curvature, baseline, roll)[0]
      now = self.clock()
    except Exception:
      self.delta = 0.
      self._withdraw("unavailable_baseline")
      self.legacy_curvature = baseline if math.isfinite(baseline) else 0.
      return baseline
    try:
      if not CC.latActive:
        self.delta = 0.
        self._withdraw("inactive")
      elif controls.lanefull_mode_enabled or not MIN_SPEED <= speed <= MAX_SPEED:
        self._withdraw("lane_or_speed_scope")
      elif any(not sm.valid.get(name, False) or not sm.alive.get(name, False) or
               not 0 <= (now - sm.logMonoTime.get(name, 0)) / 1e9 <= age
               for name, age in SOURCE_MAX_AGE.items()):
        self._withdraw("stale_source")
      elif (controls.calibrated_pose is None or not controls.pose_calibrator.calib_valid or
            not sm["livePose"].inputsOK or not sm["livePose"].posenetOK or not sm["livePose"].sensorsOK or
            not sm["livePose"].angularVelocityDevice.valid or
            not sm["liveParameters"].valid or not sm["liveParameters"].sensorValid or
            not CS.canValid or CS.canTimeout or CS.steerFaultTemporary or CS.steerFaultPermanent or
            CS.standstill or str(CS.gearShifter) in ("neutral", "park", "reverse", "unknown")):
        self._withdraw("invalid_vehicle_state")
      elif str(sm["modelV2"].meta.laneChangeState) != "off":
        self._withdraw("lane_change")
      elif CS.steeringPressed:
        # Manual override remains a withdrawal condition, never the trigger
        # for autonomous entry or return.
        self._withdraw("driver_override")
      else:
        stamp = int(sm.logMonoTime["modelV2"])
        if stamp != self.last_model_ns:
          model = sm["modelV2"]
          frame_id = int(model.frameId)
          continuous = (self.last_frame_id is not None and frame_id == self.last_frame_id + 1 and
                        0 < stamp - self.last_model_ns <= 100_000_000)
          self.last_model_ns, self.last_frame_id = stamp, frame_id
          if not continuous:
            self.phase_frames = 0
          radius = lateral_rotation_radius(controls.CP, speed)
          path = model_path(model, speed, radius)
          state = PreviewState(speed, float(controls.calibrated_pose.angular_velocity.yaw) / speed,
                               float(controls.curvature), float(delay), self.previous_request,
                               float(sm["carOutput"].actuatorsOutput.torque),
                               radius)
          result = preview_candidate(path, state, self.legacy_curvature)
          self.phase_frames = self.phase_frames + 1 if continuous and result.phase == self.phase else 1
          self.phase = result.phase
          self.target_delta = result.target - self.legacy_curvature if result.accepted and self.phase_frames >= 3 else 0.
          self.record = {**result.__dict__, "phase_frames": self.phase_frames,
                         "measured_curvature": state.measured_curvature,
                         "steering_curvature": state.steering_curvature,
                         "rotation_radius_m": state.rotation_radius,
                         "selected_delay_s": delay, "model_mono_ns": stamp,
                         "path_curvature_near": float(np.mean(path.curvature[:5])),
                         "path_curvature_far": float(np.mean(path.curvature[15:]))}
    except Exception:
      self._withdraw("invalid_input_or_solver")
    # Absolute offset bound is relative to the independent legacy trajectory,
    # never to last tick's already-corrected/filter-fed value.
    limit = min(MAX_EXTRA_CURVATURE, MAX_EXTRA_ACCEL / max(speed, MIN_SPEED) ** 2)
    step = MAX_EXTRA_JERK * .01 / max(speed, MIN_SPEED) ** 2
    desired_delta = float(np.clip(self.target_delta, -limit, limit))
    self.delta = float(np.clip(self.delta + np.clip(desired_delta - self.delta, -step, step), -limit, limit))
    phase = self.record.get("phase") if self.record is not None else None
    incompatible = (phase in ("straight", "changing_direction") or
                    (phase == "exit" and self.delta * self.legacy_curvature > 0.) or
                    (phase in ("entry", "hold") and self.delta * self.legacy_curvature < 0.))
    if incompatible and self.delta != 0.:
      # Withdrawing at the extra-jerk rate would carry entry assistance into
      # an already detected exit (or exit reduction into a renewed curve).
      # Project the *proposal* to legacy immediately; the unchanged final
      # clip_curvature and CAN slew limits retain actual-command continuity.
      self.delta = 0.
      self.phase_projection_count += 1
    # A decaying exit correction cannot cross zero when a changing model
    # target passes through straight ahead. Existing final clip_curvature still
    # bounds the applied transition even at this direction-change guard.
    if self.legacy_curvature * (self.legacy_curvature + self.delta) < 0 or self.legacy_curvature == 0.:
      self.delta = -self.legacy_curvature if self.legacy_curvature else 0.
    return self.legacy_curvature + self.delta

  def emit(self, requested_torque: float, published_curvature: float | None = None):
    """Existing local logMessage path only, after normal controls publication."""
    self.previous_request = requested_torque
    try:
      now = self.clock()
      if self.logger is not None and self.record is not None and now - self.last_log_ns >= 100_000_000:
        self.last_log_ns = now
        self.logger({"event": "dk_experimental_steering", "schema": 1, "mono_ns": now,
                     "legacy_curvature": self.legacy_curvature, "delta": self.delta,
                     "target_delta": self.target_delta,
                     "phase_projection_count": self.phase_projection_count,
                     "published_curvature": published_curvature,
                     "published_delta": (published_curvature - self.legacy_curvature
                                         if published_curvature is not None else None), **self.record})
    except Exception:
      pass
