"""Passive, bounded control-stage observations; never an input to control.

The caller supplies values from the completed control calculation and emits only
after both normal messages were published. No Params, CAN, files, or sockets are
accessed here. The supplied debug logger uses the existing local logMessage path.
"""
from __future__ import annotations

import math
import time
from numbers import Integral, Real


SAMPLE_NS = 100_000_000
SOURCE_SERVICES = ("carState", "carOutput", "modelV2", "lateralPlan", "liveDelay",
                   "liveParameters", "liveTorqueParameters", "selfdriveState")


def number(value):
  if isinstance(value, bool):
    return None
  if isinstance(value, Integral):
    return int(value) if abs(value) <= 2 ** 63 - 1 else None
  if isinstance(value, Real):
    return float(value) if math.isfinite(value) and abs(value) <= 1e12 else None
  return None


def values(obj, names):
  return {name: number(getattr(obj, name, None)) for name in names}


class DkLateralDiagnostics:
  def __init__(self, logger, clock=time.monotonic_ns):
    self.logger = logger
    self.clock = clock
    self.pending = None
    self.last_emit_ns = None

  def capture(self, frame, previous_curvature, lane_curvature, smoothed_curvature,
              curvature_limited, steer_limited_input, lat_plan_fresh,
              configured_smoothing_s, effective_delay_s):
    """Copy only already computed scalars; keep at most one 10-frame sample."""
    try:
      self.pending = None
      if frame % 10:
        return
      self.pending = {
        "frame": number(frame), "mono_ns": number(self.clock()),
        "previous_curvature": number(previous_curvature),
        "lane_target_available": lane_curvature is not None,
        "lane_lag_adjusted_curvature": number(lane_curvature),
        "smoothed_curvature": number(smoothed_curvature),
        "curvature_limited": bool(curvature_limited),
        "steer_limited_input": bool(steer_limited_input),
        "lat_plan_fresh": bool(lat_plan_fresh),
        "configured_smoothing_s": number(configured_smoothing_s),
        "selected_actuator_delay_s": number(effective_delay_s),
      }
    except Exception:
      self.pending = None

  def emit(self, controls, CC):
    """Never propagate observer/logging failures into the control process."""
    try:
      sample, self.pending = self.pending, None
      if sample is None or sample["frame"] != controls.sm.frame:
        return
      now_ns = self.clock()
      if self.last_emit_ns is not None and now_ns - self.last_emit_ns < SAMPLE_NS:
        return
      # Account for attempted writes too, so a broken logger cannot cause a flood.
      self.last_emit_ns = now_ns
      sm = controls.sm
      CS = sm["carState"]
      pid = controls.LaC.pid
      torque_params = controls.LaC.torque_params
      lane_mode = bool(controls.lanefull_mode_enabled)
      source = ("inactive" if not CC.latActive else
                "lane" if lane_mode and sample["lane_target_available"] else
                "lane_empty" if lane_mode else "model")
      before_smoothing = (sample["lane_lag_adjusted_curvature"] if source == "lane" else
                          number(sm["modelV2"].action.desiredCurvature) if source == "model" else
                          number(controls.curvature))
      effective_smoothing = sample["configured_smoothing_s"] if source == "lane" else 0.1 if source == "model" else 0.0
      clipped_curvature = number(controls.desired_curvature)
      sources = {}
      for name in SOURCE_SERVICES:
        stamp = number(sm.logMonoTime.get(name, 0))
        sources[name] = {"mono_ns": stamp, "valid": bool(sm.valid.get(name, False)),
                         "alive": bool(sm.alive.get(name, False))}
      record = {
        "event": "dk_lateral_stage", "schema": 1, "kind": "sample", **sample,
        "published_mono_ns": number(now_ns), "sources": sources,
        "lat_active": bool(CC.latActive), "selfdrive_active": bool(sm["selfdriveState"].active),
        "target_source": source, "before_smoothing_curvature": before_smoothing,
        "effective_smoothing_s": effective_smoothing,
        "clipped_curvature": clipped_curvature,
        "curvature_changed_by_clip": (sample["smoothed_curvature"] != clipped_curvature
                                      if sample["smoothed_curvature"] is not None and clipped_curvature is not None else None),
        "curvature_limit_flag_scope": "acceleration_or_absolute_limit_excludes_jerk",
        "lane_lag_adjustment_applied": source == "lane",
        "lane_lag_adjustment_delay_s": (number(sample["selected_actuator_delay_s"] + effective_smoothing)
                                        if source == "lane" and sample["selected_actuator_delay_s"] is not None
                                        and effective_smoothing is not None else None),
        "steer_limited_after_publish": bool(controls.steer_limited_by_safety),
        # This input flag is held by existing control code when selfdriveState
        # is inactive. It is not proof that panda/EPS rejected this command.
        "steer_limit_flag_refreshed": bool(sm["selfdriveState"].active),
        "driver_pressed": bool(CS.steeringPressed),
        "integrator_freeze_applies_only_when_lat_active": True,
        "integrator_freeze_condition": bool(sample["steer_limited_input"] or CS.steeringPressed or CS.vEgo < 5),
        "steering_rate_semantics": "KA4 unsigned magnitude; infer direction from angle/time",
        "vehicle": values(CS, ("vEgo", "steeringAngleDeg", "steeringRateDeg", "steeringTorque")),
        "request": values(CC.actuators, ("torque", "steeringAngleDeg", "curvature")),
        # carOutput is the latest received output, not a synchronized ECU ACK.
        "latest_output": values(sm["carOutput"].actuatorsOutput, ("torque", "torqueOutputCan", "steeringAngleDeg")),
        "pid": values(pid, ("k_p", "k_i", "k_d", "k_f", "speed", "p", "i", "d", "f", "control", "pos_limit", "neg_limit")),
        "torque_params": values(torque_params, ("latAccelFactor", "latAccelOffset", "friction", "steeringAngleDeadzoneDeg")),
        "torque_mode": {"custom": number(controls.LaC.lateralTorqueCustom),
                         "use_steering_angle": bool(controls.LaC.use_steering_angle),
                         "nnff": bool(controls.LaC.use_nnff), "nnff_lite": bool(controls.LaC.use_nnff_lite)},
        "vehicle_model": values(controls.VM, ("sR", "cF", "cR")),
      }
      self.logger(record)
    except Exception:
      pass
