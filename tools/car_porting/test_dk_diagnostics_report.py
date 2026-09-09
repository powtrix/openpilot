import json
from types import SimpleNamespace

import pytest

from tools.car_porting.dk_diagnostics_report import DiagnosticsReport, analyze_local, main  # noqa: TID251


def sample(ns=1_000_000_000, *, enabled=True, resume=False, stopped=True, accel=0.0, scc_accel=0.0,
           angle=2.0, output_angle=3.0, speed=10.0, warning=0, should_stop=True, legacy_resume=False):
  return {"event": "dk_vehicle_diag", "schema": 1, "kind": "sample", "mono_ns": ns,
          "topics": ["resume", "engage_warning", "curve", "unwind", "braking"],
          "car": {"speed_mps": speed, "accel_mps2": accel, "jerk_mps3": -0.5, "standstill": stopped,
                  "steering_angle_deg": angle, "steering_rate_dps": -1.0},
          "request": {"enabled": enabled, "resume": resume, "angle_deg": 4.0, "curvature": 0.003, "lat_active": True, "long_active": False},
          "output": {"angle_deg": output_angle, "torque": 0.2},
          "radar": {"status": True, "d_rel_m": 10.0, "v_rel_mps": -1.0},
          "planner": {"should_stop": should_stop, "final_speed_mps": 0.3},
          "raw_before": {"scc_control": {"values": {"InfoDisplay": 4, "ACCMode": 1, "aReqValue": scc_accel,
                                                     "SysFailState": warning, "TakeOverReq": 0}, "missing": False,
                                         "packet_sources": [{"age_ms": 5}]}},
          "shadow": {"legacy_resume": legacy_resume, "should_stop_resume": False,
                     "gentle_decel_mps2": -0.4, "requested_angle_rate_dps": -1.0,
                     "interpretation": "hypothesis_not_vehicle_response"},
          "freshness": {"radarState": {"valid": True, "alive": True, "age_ms": 20}}}


def session(branch="dkcarrot-wip", commit="abc"):
  return {"event": "dk_vehicle_diag", "schema": 1, "kind": "session", "mono_ns": 0,
          "branch": branch, "commit": commit, "initial_params": {}, "cp": {"carFingerprint": "KIA_CARNIVAL_4TH_GEN"}}


def test_observed_requests_do_not_become_acceptance_or_physical_resume():
  analyzer = DiagnosticsReport()
  analyzer.begin_source("local-rlog")
  analyzer.feed_record(session())
  analyzer.feed_record(sample(enabled=False))
  analyzer.feed_record(sample(1_500_000_000, resume=True, accel=-0.4, scc_accel=-0.3, warning=1, legacy_resume=True))
  report = analyzer.report()
  events = {e["event"]: e for e in report["timeline"]}
  assert events["engage_transition"]["source_seconds"] == 0.5
  assert events["engage_transition"]["warnings_before"]["scc_failure"] == 0
  assert events["engage_transition"]["warnings_at_sample"]["scc_failure"] == 1
  assert events["scc_request_onset"]["seconds_after_observed_closing"] == 0.5
  assert events["measured_decel_onset"]["seconds_after_observed_closing"] == 0.5
  assert events["resume_request_rise"]["branch"] == "dkcarrot-wip"
  assert "physical_motion_observed" not in events
  assert report["resume"]["request_true_samples"] == 1
  assert report["assessment"] == "observations_only_no_vehicle_acceptance_verdict"
  assert report["lateral_by_speed"]["curve"]["30_to_80_kph"]["output_minus_measured_angle_deg"]["mean"] == 1


def test_quality_missing_invalid_stale_are_not_converted_to_zero():
  analyzer = DiagnosticsReport()
  record = sample()
  record["freshness"]["radarState"] = {"valid": False, "alive": False, "age_ms": 2000}
  record["car"]["jerk_mps3"] = float("nan")
  record["output"].pop("angle_deg")
  analyzer.feed_record(record)
  report = analyzer.report()
  assert report["source_quality"]["radarState"]["stale"] == 1
  assert report["source_quality"]["radarState"]["not_valid"] == 1
  assert report["source_quality"]["carControl"]["missing"] == 1
  assert report["raw_source_quality"]["scc_control"]["has_recent_packet_source"] == 1
  assert report["metrics"]["car.jerk_mps3"]["mean"] is None
  assert report["lateral_by_speed"]["unwind"]["30_to_80_kph"]["output_minus_measured_angle_deg"]["count"] == 0
  json.dumps(report, allow_nan=False)


def test_timeline_and_metadata_memory_are_bounded():
  analyzer = DiagnosticsReport(timeline_limit=10000)
  for i in range(1000):
    analyzer.feed_record(session(commit=str(i)))
    analyzer.feed_record(sample(i * 1_000_000_000))
  report = analyzer.report()
  assert len(report["timeline"]) == 200
  assert report["timeline_dropped"] == 800
  assert len(report["metadata"]["commit"]) == 16
  assert report["metadata_truncated"]


def test_boundaries_and_gaps_do_not_invent_onsets_or_metadata():
  analyzer = DiagnosticsReport()
  analyzer.begin_source("a")
  analyzer.feed_record(session(commit="A"))
  analyzer.feed_record(sample(enabled=False))
  analyzer.begin_source("b")
  analyzer.feed_record(sample(2_000_000_000, resume=True, accel=-1.0))
  analyzer.feed_record(sample(9_000_000_000, enabled=False))
  analyzer.feed_record(sample(10_000_000_000))
  report = analyzer.report()
  assert report["counts"].get("event_measured_decel_onset", 0) == 0
  assert report["counts"]["sample_gaps_over_2_seconds"] == 1
  engage = next(e for e in report["timeline"] if e["event"] == "engage_transition")
  assert engage["commit"] is None


def test_full_schema_fake_events_count_services_and_ignore_initdata_timestamp():
  analyzer = DiagnosticsReport()
  analyzer.feed_event(SimpleNamespace(which=lambda: "initData", logMonoTime=99_000_000_000,
                                      initData=SimpleNamespace(gitBranch="dkcarrot-wip", gitCommit="abc")))
  analyzer.feed_event(SimpleNamespace(which=lambda: "carState", logMonoTime=1_000_000_000))
  analyzer.feed_event(SimpleNamespace(which=lambda: "logMessage", logMessage=json.dumps({"msg": sample()})))
  report = analyzer.report()
  assert report["source_service_counts"]["carState"] == 1
  assert report["time"]["first_sample_mono_ns"] == 1_000_000_000


def test_actual_capnp_rlog_roundtrip_uses_full_reader(tmp_path):
  from openpilot.cereal import log
  path = tmp_path / "rlog"
  events = []
  for typ in ("carState", "logMessage", "logMessage"):
    event = log.Event.new_message()
    event.logMonoTime = len(events) * 500_000_000
    if typ == "carState":
      event.init(typ)
    else:
      event.logMessage = json.dumps({"msg": session() if len(events) == 1 else sample()})
    events.append(event)
  path.write_bytes(b"".join(e.to_bytes() for e in events))
  report = analyze_local([path])
  assert report["source_service_counts"] == {"carState": 1, "logMessage": 2}
  assert report["counts"]["samples"] == 1
  assert report["metadata"]["commit"] == ["abc"]


def test_jsonl_schema_validation_cli_and_no_remote_lookup(tmp_path, capsys):
  path = tmp_path / "capture.jsonl"
  path.write_text("\n".join(["invalid", json.dumps({**sample(), "schema": 2}), json.dumps(session()), json.dumps(sample())]))
  assert main(["--jsonl", str(path)]) == 0
  report = json.loads(capsys.readouterr().out)
  assert report["counts"]["unsupported_schema"] == 1
  assert report["counts"]["unparseable_json_messages"] == 1
  with pytest.raises((ValueError, OSError)):
    analyze_local(["https://example.com/rlog.zst"])


def test_sample_rejects_invalid_time_and_missing_sample_is_not_success():
  analyzer = DiagnosticsReport()
  analyzer.feed_record({**sample(), "mono_ns": float("nan")})
  analyzer.feed_record(session())
  assert analyzer.report()["counts"]["invalid_diagnostic_records"] == 1
  assert analyzer.report()["time"]["first_sample_mono_ns"] is None


def test_actual_observer_to_full_rlog_to_report(tmp_path):
  from openpilot.cereal import log
  from openpilot.selfdrive.carrot.dk_vehicle_diagnostics import DkVehicleDiagnostics
  from openpilot.selfdrive.carrot.tests.test_dk_vehicle_diagnostics import fixture_objects, record

  logs = []
  observer = DkVehicleDiagnostics({"branch": "dkcarrot-wip", "commit": "abcdef1"}, logs.append)
  cs, cc, ci, sm = fixture_objects()
  cs.standstill = cs.cruiseState.standstill = True
  for i in range(21):
    cc.cruiseControl.resume = bool(i % 2)
    record(observer, cs, cc, ci, sm, now=1_000_000_000 + i * 10_000_000,
           can_sends=[(0x1AA, bytes([0, 0, i, 0, 16]) + bytes(11), 2)])
  events = []
  for message in logs:
    event = log.Event.new_message()
    event.logMonoTime = message["mono_ns"]
    event.logMessage = json.dumps({"msg": message})
    events.append(event.to_bytes())
  path = tmp_path / "rlog"
  path.write_bytes(b"".join(events))
  report = analyze_local([path])
  assert report["metadata"]["commit"] == ["abcdef1"]
  assert report["resume"]["host_submitted_res_frames"] == 21
  assert report["captured_transition_counts"]["resume"] == 20
  assert report["raw_source_quality"]["scc_control"]["has_recent_packet_source"] == report["counts"]["samples"]
  assert report["resume"]["scc_states"]["scc_info_display"] == {"0": report["counts"]["samples"]}
  assert report["metrics"]["raw_before.scc_control.aReqValue"]["mean"] == pytest.approx(-0.1)
  assert all(e["commit"] == "abcdef1" for e in report["timeline"])


def test_periodic_identical_sessions_preserve_transition_and_deduplicate_config():
  analyzer = DiagnosticsReport()
  analyzer.feed_record(session())
  analyzer.feed_record(sample(enabled=False))
  analyzer.feed_record(session())
  analyzer.feed_record(sample(1_500_000_000, enabled=True))
  report = analyzer.report()
  assert report["counts"]["session_reannouncements"] == 1
  assert report["counts"]["event_engage_transition"] == 1
  assert len(report["configuration"]) == 1


def test_configuration_uses_exact_current_observer_keys_and_identity():
  from openpilot.selfdrive.carrot.dk_vehicle_diagnostics import PARAM_KEYS, make_dk_vehicle_diagnostics
  from openpilot.selfdrive.carrot.tests.test_dk_vehicle_diagnostics import FakeParams, cp, fixture_objects, record
  from tools.car_porting.dk_diagnostics_report import CONFIG_KEYS  # noqa: TID251

  assert set(CONFIG_KEYS) == set(PARAM_KEYS)
  logs = []
  observer = make_dk_vehicle_diagnostics(cp(), FakeParams(), logs.append)
  record(observer, *fixture_objects())
  analyzer = DiagnosticsReport()
  for message in logs:
    analyzer.feed_record(message)
  config = analyzer.report()["configuration"][0]
  assert config["diagnostics_version"] == "dk-vehicle-diag-v1"
  assert config["params_snapshot_scope"] == "initial_numeric_raw_params_only"
  assert config["cp"]["carFingerprint"] == str(cp().carFingerprint)
  assert config["initial_params"]["StopDistanceCarrot"] == 50
  assert config["initial_params"]["AlphaLongitudinalEnabled"] == 50
  assert config["initial_params"]["CustomSteerDeltaDownLC"] == 50


def test_explicit_unwind_phase_counts_constant_zero_target_and_cooldown_samples():
  analyzer = DiagnosticsReport()
  for index in range(3):
    record = sample((index + 1) * 100_000_000, angle=20, output_angle=0)
    record["request"]["angle_deg"] = 0
    record["shadow"].update({"requested_angle_rate_dps": 0, "unwind_active": True, "curve_active": False})
    record["topics"] = ["unwind"] if index == 0 else []
    analyzer.feed_record(record)
  report = analyzer.report()
  assert report["topic_candidate_counts"]["unwind"] == 1
  assert report["counts"]["unwind_explicit_phase_samples"] == 3
  assert report["lateral_by_speed"]["unwind"]["30_to_80_kph"]["measured_angle_deg"]["count"] == 3
  assert report["lateral_by_speed"]["curve"] == {}


def test_all_new_warning_transitions_survive_real_observer_schema():
  from openpilot.selfdrive.carrot.dk_vehicle_diagnostics import DkVehicleDiagnostics, RAW_FIELDS
  from openpilot.selfdrive.carrot.tests.test_dk_vehicle_diagnostics import fixture_objects, record

  cs, cc, ci, sm = fixture_objects()
  for name, (_, fields) in RAW_FIELDS.items():
    setattr(ci.CS, name, dict.fromkeys(fields, 0))
  logs = []
  observer = DkVehicleDiagnostics({}, logs.append)
  record(observer, cs, cc, ci, sm)
  expected = []
  for attr, prefix, names in (
    ("scc_control", "scc", ("SysFailState", "TakeOverReq", "DriverAlert")),
    ("adrv_0x161", "adrv", ("ALERTS_1", "ALERTS_2", "ALERTS_3", "ALERTS_4", "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4")),
    ("lfahda_cluster", "lfahda", ("HDA_InfoPUDis", "HDA_InfoPUDis1", "HDA_LFA_WrnSnd")),
    ("ccnc_0x162", "ccnc", ("FAULT_DAS", "FAULT_SCC", "FAULT_LSS")),
    ("mdps", "mdps", ("LKA_FAULT", "LFA2_FAULT")),
  ):
    for name in names:
      getattr(ci.CS, attr)[name] = 1
      expected.append(prefix + "_" + name)
  record(observer, cs, cc, ci, sm, now=1_100_000_000)
  analyzer = DiagnosticsReport()
  for message in logs:
    analyzer.feed_record(message)
  report = analyzer.report()
  assert set(report["captured_transition_counts"]) == set(expected)
  assert all(count == 1 for count in report["captured_transition_counts"].values())
  changed = next(item for item in report["timeline"] if item["event"] == "warning_signal_change")
  assert changed["current"]["FAULT_DAS"] == 1
  assert changed["current"]["adrv_sound_4"] == 1
  assert changed["current"]["HDA_InfoPUDis1"] == 1
