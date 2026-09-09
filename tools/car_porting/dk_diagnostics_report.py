#!/usr/bin/env python3
"""Summarize existing local DK diagnostic records; never publish, upload, or change Params.

Usage: python3 tools/car_porting/dk_diagnostics_report.py /path/to/rlog.zst
       python3 tools/car_porting/dk_diagnostics_report.py --jsonl /path/to/capture.jsonl

The full cereal LogReader decodes one local segment at a time. Report aggregates
and the event timeline are bounded; LogReader itself materializes that segment.
JSONL input is streamed. Sampled requests and outputs are observations, not proof
of ECU acceptance, correct control, or the cause of a vehicle warning.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
  sys.path.insert(0, str(ROOT))

TOPICS = ("resume", "engage_warning", "curve", "unwind", "braking")
SERVICES = ("carControl", "controlsState", "longitudinalPlan", "lateralPlan", "radarState", "modelV2")
RAW_SOURCES = ("scc_control", "adrv_0x161", "ccnc_0x162", "lfahda_cluster", "mdps", "tcs", "cruise_buttons_msg")
TRANSITION_FIELDS = ("enabled", "lat_active", "standstill", "cruise_standstill", "resume", "steering_pressed",
                     "steer_fault_temporary", "steer_fault_permanent", "acc_faulted", "scc_info_display",
                     "adrv_alert_5", "ccnc_fault_hda", "ccnc_fault_lfa",
                     "scc_SysFailState", "scc_TakeOverReq", "scc_DriverAlert",
                     "adrv_ALERTS_1", "adrv_ALERTS_2", "adrv_ALERTS_3", "adrv_ALERTS_4",
                     "adrv_SOUNDS_1", "adrv_SOUNDS_2", "adrv_SOUNDS_3", "adrv_SOUNDS_4",
                     "lfahda_HDA_InfoPUDis", "lfahda_HDA_InfoPUDis1", "lfahda_HDA_LFA_WrnSnd",
                     "ccnc_FAULT_DAS", "ccnc_FAULT_SCC", "ccnc_FAULT_LSS", "mdps_LKA_FAULT", "mdps_LFA2_FAULT")
CP_FIELDS = ("carFingerprint", "flags", "extFlags", "pcmCruise", "openpilotLongitudinalControl", "steerControlType", "lateralTuning",
             "steerRatio", "steerActuatorDelay", "mass", "wheelbase", "radarUnavailable", "passive", "dashcamOnly")
CONFIG_KEYS = ("PathOffset", "AdjustLaneOffset", "UseLaneLineSpeed", "SteerActuatorDelay", "SteerRatioRate",
               "MaxAngleFrames", "CustomSteerMax", "CustomSteerDeltaUp", "CustomSteerDeltaDown", "CustomSteerDeltaUpLC",
               "CustomSteerDeltaDownLC", "LongActuatorDelay", "VEgoStopping", "StoppingAccel", "StopDistanceCarrot",
               "TrafficLightDetectMode", "ExperimentalMode", "AlphaLongitudinalEnabled", "CanfdHDA2",
               "HyundaiCameraSCC", "EnableRadarTracks", "CruiseButtonTest1", "CruiseButtonTest2", "CruiseButtonTest3",
               "AutoCruiseControl", "SpeedFromPCM", "Ka4StockSccStandstillRearm")
MAX_JSON_LINE = 1_048_576
MAX_TIMELINE = 200


def get(data: Any, path: str, default=None):
  for key in path.split("."):
    if not isinstance(data, dict) or key not in data:
      return default
    data = data[key]
  return data


def number(value):
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return None
  return float(value) if math.isfinite(value) else None


@dataclass
class Stats:
  count: int = 0
  missing: int = 0
  minimum: float | None = None
  maximum: float | None = None
  mean: float = 0.0
  mean_abs: float = 0.0

  def add(self, value):
    value = number(value)
    if value is None:
      self.missing += 1
      return
    self.count += 1
    self.mean = self.mean * (1 - 1 / self.count) + value / self.count
    self.mean_abs = self.mean_abs * (1 - 1 / self.count) + abs(value) / self.count
    self.minimum = value if self.minimum is None else min(self.minimum, value)
    self.maximum = value if self.maximum is None else max(self.maximum, value)

  def report(self):
    return {"count": self.count, "missing_or_nonfinite": self.missing,
            "min": self.minimum, "max": self.maximum,
            "mean": self.mean if self.count else None, "mean_abs": self.mean_abs if self.count else None}


class DiagnosticsReport:
  def __init__(self, timeline_limit=MAX_TIMELINE, stale_ms=1000.0):
    self.timeline_limit = min(MAX_TIMELINE, max(0, timeline_limit))
    self.stale_ms = stale_ms
    self.counts = Counter()
    self.service_counts = Counter()
    self.topic_counts = Counter()
    self.timeline = []
    self.timeline_dropped = 0
    self.metadata = {"branch": [], "commit": [], "fingerprint": []}
    self.current_metadata = dict.fromkeys(self.metadata)
    self.metadata_truncated = False
    self.configuration = []
    self.current_configuration = None
    self.first_ns = None
    self.last_ns = None
    self.previous = None
    self.source_start_ns = None
    self.source = "input"
    self.quality = {name: Counter() for name in SERVICES}
    self.raw_quality = {name: Counter() for name in RAW_SOURCES}
    self.state_counts = {name: Counter() for name in ("scc_info_display", "scc_acc_mode", "scc_failure", "scc_takeover")}
    self.resume = Counter()
    self.interlock_observations = Counter()
    self.engage = Counter()
    self.transitions = Counter()
    self.braking = Counter()
    self.closing_start_ns = None
    self.metrics = {name: Stats() for name in (
      "car.speed_mps", "car.accel_mps2", "car.jerk_mps3", "request.accel_mps2", "output.accel_mps2",
      "radar.d_rel_m", "radar.v_rel_mps", "shadow.gentle_decel_mps2",
      "raw_before.scc_control.aReqValue", "raw_before.scc_control.aReqRaw",
    )}
    self.braking_comparison = {name: Stats() for name in ("scc_minus_shadow_accel_mps2", "measured_minus_scc_accel_mps2")}
    self.lateral = {topic: {bucket: {field: Stats() for field in (
      "request_angle_deg", "output_angle_deg", "measured_angle_deg", "output_minus_measured_angle_deg",
      "request_torque", "output_torque", "request_curvature", "output_curvature", "measured_steering_rate_dps",
      "actual_curvature", "desired_minus_actual_curvature", "requested_angle_rate_dps", "output_angle_rate_dps",
    )} for bucket in ("below_30_kph", "30_to_80_kph", "80_kph_and_above", "speed_missing")} for topic in ("curve", "unwind")}
    self.onset_delay = {"scc_request_seconds_after_closing": Stats(), "measured_decel_seconds_after_closing": Stats()}

  def begin_source(self, source):
    self.source = str(source)
    self.source_start_ns = None
    # A segment boundary or missing record is not a measured transition.
    self.previous = None
    self.closing_start_ns = None
    self.current_metadata = dict.fromkeys(self.metadata)
    self.current_configuration = None
    self.counts["input_files"] += 1

  def _metadata(self, name, value):
    if value is None or not isinstance(value, (str, int, float)):
      return
    value = str(value)[:128]
    self.current_metadata[name] = value
    if value not in self.metadata[name]:
      if len(self.metadata[name]) < 16:
        self.metadata[name].append(value)
      else:
        self.metadata_truncated = True

  def _event(self, name, ns, **details):
    self.counts["event_" + name] += 1
    if len(self.timeline) >= self.timeline_limit:
      self.timeline_dropped += 1
      return
    self.timeline.append({"event": name, "source": self.source, "mono_ns": ns,
                          "source_seconds": (ns - self.source_start_ns) / 1e9,
                          "branch": self.current_metadata["branch"], "commit": self.current_metadata["commit"], **details})

  @staticmethod
  def _warnings(record):
    paths = {"scc_failure": "scc_control.SysFailState", "scc_takeover": "scc_control.TakeOverReq", "scc_driver_alert": "scc_control.DriverAlert"}
    paths.update({f"adrv_alert_{i}": f"adrv_0x161.ALERTS_{i}" for i in range(1, 6)})
    paths.update({f"adrv_sound_{i}": f"adrv_0x161.SOUNDS_{i}" for i in range(1, 5)})
    paths.update({name: "lfahda_cluster." + name for name in ("HDA_InfoPUDis", "HDA_InfoPUDis1", "HDA_LFA_WrnSnd")})
    paths.update({name: "ccnc_0x162." + name for name in ("FAULT_HDA", "FAULT_LFA", "FAULT_DAS", "FAULT_LSS", "FAULT_SCC",
                                                          "FAULT_FSS", "FAULT_FCA", "FAULT_LCA", "FAULT_HDP", "FAULT_ESS")})
    paths.update({name: "mdps." + name for name in ("LKA_FAULT", "LFA2_FAULT")})
    values = {label: number(get(record, "raw_before." + path)) for label, path in paths.items()}
    values.update({name: get(record, "car." + name) for name in ("steer_fault_temporary", "steer_fault_permanent", "acc_faulted")})
    return values

  def feed_record(self, record):
    if not isinstance(record, dict) or record.get("event") != "dk_vehicle_diag":
      self.counts["non_diagnostic_records"] += 1
      return
    if record.get("schema") != 1:
      self.counts["unsupported_schema"] += 1
      return
    if record.get("kind") == "session":
      self.counts["sessions"] += 1
      self._metadata("branch", record.get("branch"))
      self._metadata("commit", record.get("commit"))
      self._metadata("fingerprint", get(record, "cp.carFingerprint"))
      config = {"branch": self.current_metadata["branch"], "commit": self.current_metadata["commit"],
                "diagnostics_version": record.get("diagnostics_version", "unknown")[:64]
                  if isinstance(record.get("diagnostics_version", "unknown"), str) else None,
                "params_snapshot_scope": record.get("params_snapshot_scope", "unknown")[:64]
                  if isinstance(record.get("params_snapshot_scope", "unknown"), str) else None,
                "cp": {name: value if isinstance(value := get(record, "cp." + name), bool)
                       else value[:96] if name in ("carFingerprint", "steerControlType", "lateralTuning") and isinstance(value, str)
                       else number(value) for name in CP_FIELDS},
                "initial_params": {name: number(get(record, "initial_params." + name)) for name in CONFIG_KEYS}}
      if config == self.current_configuration:
        self.counts["session_reannouncements"] += 1
      else:
        if len(self.configuration) < 8:
          self.configuration.append({"source": self.source, **config})
        else:
          self.counts["configuration_snapshots_omitted"] += 1
        self.current_configuration = config
        self.previous = None
        self.closing_start_ns = None
      return
    ns = record.get("mono_ns")
    if record.get("kind") != "sample" or isinstance(ns, bool) or not isinstance(ns, int) or ns < 0:
      self.counts["invalid_diagnostic_records"] += 1
      return
    ns = int(ns)
    self.counts["samples"] += 1
    if self.first_ns is None:
      self.first_ns = ns
    if self.source_start_ns is None:
      self.source_start_ns = ns
    if self.last_ns is not None and ns < self.last_ns:
      self.counts["backward_timestamps"] += 1
      self.previous = None
      self.closing_start_ns = None
      self.source_start_ns = ns
    self.last_ns = ns
    previous = self.previous
    if previous is not None and ns - previous["mono_ns"] > 2e9:
      self.counts["sample_gaps_over_2_seconds"] += 1
      previous = None
      self.closing_start_ns = None
    for service, quality in self.quality.items():
      status = get(record, "freshness." + service)
      if not isinstance(status, dict):
        quality["missing"] += 1
        continue
      quality["present"] += 1
      for flag in ("valid", "alive"):
        quality["not_" + flag if status.get(flag) is False else flag if status.get(flag) is True else flag + "_unknown"] += 1
      age = number(status.get("age_ms"))
      quality["age_unknown" if age is None else "stale" if age > self.stale_ms or age < 0 else "within_age_limit"] += 1
    raw_values = {}
    for name, quality in self.raw_quality.items():
      raw = get(record, "raw_before." + name)
      if not isinstance(raw, dict) or not isinstance(raw.get("values"), dict) or raw.get("missing") is True:
        quality["missing"] += 1
        raw_values[name] = {}
        continue
      quality["present"] += 1
      raw_values[name] = raw["values"]
      sources = raw.get("packet_sources", [])
      ages = [number(source.get("age_ms")) for source in sources if isinstance(source, dict)] if isinstance(sources, list) else []
      ages = [age for age in ages if age is not None]
      quality["packet_age_unknown" if not ages else "all_packet_sources_stale" if all(age < 0 or age > self.stale_ms for age in ages)
              else "has_recent_packet_source"] += 1
    record = {**record, "raw_before": raw_values}
    topics = record.get("topics", [])
    topics = set(topics) & set(TOPICS) if isinstance(topics, list) and all(isinstance(t, str) for t in topics) else set()
    self.topic_counts.update(topics)
    for transition in record.get("transitions", [])[:32] if isinstance(record.get("transitions"), list) else ():
      if not isinstance(transition, dict) or transition.get("field") not in TRANSITION_FIELDS:
        continue
      transition_ns = transition.get("mono_ns")
      if isinstance(transition_ns, bool) or not isinstance(transition_ns, int) or not 0 <= transition_ns <= ns:
        continue
      name = transition["field"]
      self.transitions[name] += 1
      before, after = transition.get("before"), transition.get("after")
      before = before if isinstance(before, bool) else number(before)
      after = after if isinstance(after, bool) else number(after)
      self._event("captured_transition", transition_ns, field=name, previous=before, current=after)
    for path, stats in self.metrics.items():
      stats.add(get(record, path))
    for name, signal in (("scc_info_display", "InfoDisplay"), ("scc_acc_mode", "ACCMode"),
                         ("scc_failure", "SysFailState"), ("scc_takeover", "TakeOverReq")):
      value = number(get(record, "raw_before.scc_control." + signal))
      key = "missing" if value is None else str(int(value)) if 0 <= value <= 255 and value.is_integer() else "out_of_range"
      self.state_counts[name][key] += 1
    self._resume_and_engage(record, previous, ns)
    self._lateral(record)
    self._braking(record, previous, ns)
    self.previous = {"mono_ns": ns, "request": record.get("request", {}), "car": record.get("car", {}),
                     "raw_before": record.get("raw_before", {})}

  def _resume_and_engage(self, record, previous, ns):
    requested = get(record, "request.resume")
    self.resume["request_true_samples" if requested is True else "request_false_samples" if requested is False else "request_missing_samples"] += 1
    for field in ("brake_pressed", "gas_pressed", "brake_hold_active", "parking_brake", "acc_faulted"):
      if get(record, "car." + field) is True:
        self.interlock_observations[field] += 1
    if get(record, "car.can_valid") is False:
      self.interlock_observations["can_invalid"] += 1
    if get(record, "controller_before.ka4_stock_scc_standstill_rearm") is True:
      self.resume["experimental_rearm_enabled_samples"] += 1
    if get(record, "controller_before.stock_scc_keepalive_pending") is True:
      self.resume["experimental_rearm_pending_samples"] += 1
    for shadow in ("legacy_resume", "should_stop_resume"):
      value = get(record, "shadow." + shadow)
      if isinstance(value, bool) and isinstance(requested, bool):
        self.resume[shadow + ("_matches_request" if value == requested else "_differs_from_request")] += 1
      else:
        self.resume[shadow + "_missing"] += 1
    buttons = get(record, "submitted_can.buttons")
    if isinstance(buttons, list):
      self.resume["host_submission_coverage_samples"] += 1
      for button in buttons:
        if isinstance(button, dict) and button.get("button") == 1:
          count = button.get("count")
          if isinstance(count, int) and not isinstance(count, bool) and count >= 0:
            self.resume["host_submitted_res_frames"] += count
    else:
      self.resume["host_submission_missing_samples"] += 1
    warnings = self._warnings(record)
    self.engage["nonzero_warning_signal_samples" if any(v is not None and v != 0 for v in warnings.values()) else
                "warning_signals_all_missing" if all(v is None for v in warnings.values()) else "zero_observed_warning_signal_samples"] += 1
    if previous is None:
      return
    if get(previous, "request.enabled") is False and get(record, "request.enabled") is True:
      self._event("engage_transition", ns, warnings_before=self._warnings(previous), warnings_at_sample=warnings,
                  lat_active=get(record, "request.lat_active"), long_active=get(record, "request.long_active"))
    if warnings != self._warnings(previous):
      self._event("warning_signal_change", ns, previous=self._warnings(previous), current=warnings,
                  request_enabled=get(record, "request.enabled"))
    if requested is True and get(previous, "request.resume") is False:
      self._event("resume_request_rise", ns, planner=record.get("planner", {}), shadow=record.get("shadow", {}),
                  scc_info_display=get(record, "raw_before.scc_control.InfoDisplay"))
    for old, new, label in ((False, True, "physical_standstill_observed"), (True, False, "physical_motion_observed")):
      if get(previous, "car.standstill") is old and get(record, "car.standstill") is new:
        self._event(label, ns, speed_mps=get(record, "car.speed_mps"), resume_requested=requested)

  def _lateral(self, record):
    speed = number(get(record, "car.speed_mps"))
    bucket = "speed_missing" if speed is None else "below_30_kph" if speed * 3.6 < 30 else "30_to_80_kph" if speed * 3.6 < 80 else "80_kph_and_above"
    output, measured = number(get(record, "output.angle_deg")), number(get(record, "car.steering_angle_deg"))
    requested = number(get(record, "request.angle_deg"))
    curvature, actual = number(get(record, "request.curvature")), number(get(record, "lateral.actual_curvature"))
    rate = number(get(record, "shadow.requested_angle_rate_dps"))
    phases = set()
    for topic in ("curve", "unwind"):
      active = get(record, "shadow." + topic + "_active")
      if isinstance(active, bool):
        self.counts[topic + "_explicit_phase_samples"] += 1
        if active:
          phases.add(topic)
      else:
        self.counts[topic + "_fallback_phase_samples"] += 1
        if get(record, "request.enabled") is True and speed is not None and speed > 3:
          if topic == "curve" and curvature is not None and abs(curvature) > 0.002:
            phases.add(topic)
          if topic == "unwind" and requested is not None and rate is not None and requested * rate < 0:
            phases.add(topic)
    values = {"request_angle_deg": get(record, "request.angle_deg"), "output_angle_deg": output,
              "measured_angle_deg": measured, "output_minus_measured_angle_deg": None if output is None or measured is None else output - measured,
              "request_torque": get(record, "request.torque"), "output_torque": get(record, "output.torque"),
              "request_curvature": get(record, "request.curvature"), "output_curvature": get(record, "output.curvature"),
              "measured_steering_rate_dps": get(record, "car.steering_rate_dps"), "actual_curvature": actual,
              "desired_minus_actual_curvature": None if curvature is None or actual is None else curvature - actual,
              "requested_angle_rate_dps": rate, "output_angle_rate_dps": get(record, "shadow.output_angle_rate_dps")}
    for topic in phases:
      for field, value in values.items():
        self.lateral[topic][bucket][field].add(value)

  def _braking(self, record, previous, ns):
    distance, rel_speed = number(get(record, "radar.d_rel_m")), number(get(record, "radar.v_rel_mps"))
    closing = get(record, "radar.status") is True and distance is not None and rel_speed is not None and 0 < distance <= 80 and rel_speed < -0.5
    scc, shadow, measured = (number(get(record, path)) for path in
                             ("raw_before.scc_control.aReqValue", "shadow.gentle_decel_mps2", "car.accel_mps2"))
    self.braking_comparison["scc_minus_shadow_accel_mps2"].add(None if scc is None or shadow is None else scc - shadow)
    self.braking_comparison["measured_minus_scc_accel_mps2"].add(None if scc is None or measured is None else measured - scc)
    self.braking["closing_lead_samples" if closing else "not_closing_or_missing_samples"] += 1
    if not closing:
      self.closing_start_ns = None
    elif self.closing_start_ns is None:
      self.closing_start_ns = ns
      self._event("closing_lead_observed", ns, lead_distance_m=distance, relative_speed_mps=rel_speed)
    if previous is None:
      return
    for name, path in (("scc_request", "raw_before.scc_control.aReqValue"), ("measured_decel", "car.accel_mps2")):
      before, current = number(get(previous, path)), number(get(record, path))
      if before is not None and current is not None and before >= -0.1 > current:
        delay = None if self.closing_start_ns is None else (ns - self.closing_start_ns) / 1e9
        self.onset_delay[name + "_seconds_after_closing"].add(delay)
        self._event(name + "_onset", ns, seconds_after_observed_closing=delay, lead_distance_m=distance,
                    relative_speed_mps=rel_speed, accel_mps2=current, measured_jerk_mps3=get(record, "car.jerk_mps3"))

  def feed_event(self, event):
    kind = event.which()
    self.service_counts[kind] += 1
    if kind == "initData":
      for name, attr in (("branch", "gitBranch"), ("commit", "gitCommit")):
        self._metadata(name, getattr(event.initData, attr, None))
    elif kind == "carParams":
      self._metadata("fingerprint", getattr(event.carParams, "carFingerprint", None))
    elif kind == "logMessage":
      self.feed_json(event.logMessage)

  def feed_json(self, text):
    try:
      value = json.loads(text)
    except (ValueError, TypeError):
      self.counts["unparseable_json_messages"] += 1
      return
    if isinstance(value, dict) and isinstance(value.get("msg"), dict):
      value = value["msg"]
    self.feed_record(value)

  def report(self):
    return {"report_schema": 1, "assessment": "observations_only_no_vehicle_acceptance_verdict",
            "limitations": ["Submitted CAN and resume requests do not prove ECU acceptance or vehicle response.",
                            "Shadow values are hypotheses, not control changes or measured outcomes.",
                            "All onset times are sampled observations, not exact ECU reaction times or causality.",
                            "Statistics include recorded values; consult per-service missing/invalid/stale coverage.",
                            "Raw fields are cached decoded signals, not independently observed physical CAN outcomes.",
                            "Steering angle error is descriptive, not tracking validation on torque-controlled cars."],
            "criteria": {"closing_lead": "radar status true, 0 < distance <= 80 m and relative speed < -0.5 m/s",
                         "deceleration_onset": "adjacent recorded acceleration crosses from >= -0.1 to < -0.1 m/s^2",
                         "curve_samples": "shadow.curve_active; fallback: enabled, speed > 3 m/s, |request curvature| > 0.002 /m",
                         "unwind_samples": "shadow.unwind_active (includes fixed-target lag); fallback: enabled, speed > 3 m/s, request angle * rate < 0",
                         "source_stale_after_ms": self.stale_ms, "transition_gap_limit_seconds": 2.0},
            "counts": dict(self.counts), "source_service_counts": dict(self.service_counts),
            "metadata": self.metadata, "mixed_metadata": any(len(v) > 1 for v in self.metadata.values()),
            "metadata_truncated": self.metadata_truncated,
            "configuration": self.configuration,
            "time": {"first_sample_mono_ns": self.first_ns, "last_sample_mono_ns": self.last_ns,
                     "basis": "first diagnostic sample within each input, never initData"},
            "source_quality": {k: dict(v) for k, v in self.quality.items()},
            "raw_source_quality": {k: dict(v) for k, v in self.raw_quality.items()},
            "topic_candidate_counts": dict(self.topic_counts),
            "captured_transition_counts": dict(self.transitions),
            "resume": {**dict(self.resume), "scc_states": {k: dict(v) for k, v in self.state_counts.items()},
                       "observed_interlocks_not_causal_verdict": dict(self.interlock_observations)},
            "engage_warning": dict(self.engage), "braking": {**dict(self.braking),
              "onset_delays": {k: v.report() for k, v in self.onset_delay.items()},
              "descriptive_comparisons_not_control_targets": {k: v.report() for k, v in self.braking_comparison.items()}},
            "metrics": {k: v.report() for k, v in self.metrics.items()},
            "lateral_by_speed": {topic: {bucket: {field: stats.report() for field, stats in fields.items()}
                                        for bucket, fields in buckets.items() if any(s.count or s.missing for s in fields.values())}
                                 for topic, buckets in self.lateral.items()},
            "timeline": self.timeline, "timeline_limit": self.timeline_limit, "timeline_dropped": self.timeline_dropped}


def analyze_local(paths, *, jsonl=False, timeline_limit=MAX_TIMELINE):
  analyzer = DiagnosticsReport(timeline_limit=timeline_limit)
  for value in paths:
    path = Path(value).expanduser().resolve(strict=True)
    if not path.is_file():
      raise ValueError(f"Only existing local files are accepted: {path}")
    analyzer.begin_source(path)
    if jsonl:
      with path.open(encoding="utf-8") as stream:
        while line := stream.readline(MAX_JSON_LINE + 1):
          if len(line) > MAX_JSON_LINE:
            raise ValueError("JSONL record exceeds 1 MiB limit")
          analyzer.feed_json(line)
    else:
      from openpilot.tools.lib.logreader import LogReader
      # A local resolved path and empty remote-source list prevent route lookup.
      for event in LogReader(str(path), sources=[], only_union_types=False):
        analyzer.feed_event(event)
  return analyzer.report()


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("paths", nargs="+", help="Existing local rlogs, or local JSONL files with --jsonl")
  parser.add_argument("--jsonl", action="store_true")
  parser.add_argument("--timeline-limit", type=int, default=MAX_TIMELINE, choices=range(MAX_TIMELINE + 1), metavar="0..200")
  args = parser.parse_args(argv)
  try:
    report = analyze_local(args.paths, jsonl=args.jsonl, timeline_limit=args.timeline_limit)
  except (OSError, ValueError) as error:
    parser.error(str(error))
  print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
  return 0 if report["counts"].get("samples", 0) else 2


if __name__ == "__main__":
  raise SystemExit(main())
