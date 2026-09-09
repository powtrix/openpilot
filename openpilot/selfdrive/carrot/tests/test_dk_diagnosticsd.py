import hashlib
import json
import os

import pytest

from openpilot.selfdrive.carrot.dk_diagnosticsd import CaptureStore, TTL_SECONDS, parse_record, read_manifest


def sample(topic="resume"):
  return {"event": "dk_vehicle_diag", "schema": 1, "kind": "sample", "mono_ns": 123456,
          "topics": [topic], "shadow": {"interpretation": "hypothesis_not_vehicle_response"}}


def segment(root, index, data=b"full-rlog", locked=False):
  directory = root / f"route--{index}"
  directory.mkdir(parents=True, exist_ok=True)
  (directory / "rlog.zst").write_bytes(data)
  if locked:
    (directory / "rlog.lock").touch()
  return directory


@pytest.fixture
def store(tmp_path):
  logs = tmp_path / "realdata"
  logs.mkdir()
  return CaptureStore(logs, tmp_path / "dk-diagnostics", min_free_bytes=0)


def test_structured_debug_log_parse_and_rejection():
  assert parse_record(json.dumps({"msg": sample()})) == sample()
  assert parse_record("invalid") is None
  assert parse_record({"event": "other", "schema": 1}) is None
  assert parse_record({**sample(), "bad": float("nan")}) is None
  assert parse_record({**sample(), "bad": "x" * 70000}) is None


def test_no_topics_no_copy_and_session_retained(store):
  segment(store.log_root, 0)
  session = {"event": "dk_vehicle_diag", "schema": 1, "kind": "session", "commit": "test"}
  store.accept(session, "route", 1000)
  assert store.accept({**sample(), "topics": []}, "route", 1000) is None
  assert store.entries() == []
  cap = store.accept(sample(), "route", 1001)
  assert read_manifest(store.root / cap)["session"] == session


def test_copy_closed_pre_event_and_post_logs_without_touching_originals(store):
  segment(store.log_root, 0, b"pre")
  current = segment(store.log_root, 1, b"event", locked=True)
  cap = store.accept(sample(), "route", 1000)
  store.tick(1001)
  store.tick(1006)
  pending = read_manifest(store.root / cap)
  assert pending["status"] == "pending"
  assert [f["segment_index"] for f in pending["files"]] == [0]
  assert (current / "rlog.lock").exists()
  (current / "rlog.lock").unlink()
  segment(store.log_root, 2, b"post")
  store.tick(1100)
  store.tick(1105)
  manifest = read_manifest(store.root / cap)
  assert manifest["status"] == "ready"
  assert manifest["missing"] == []
  assert len(manifest["files"]) == 3
  for item in manifest["files"]:
    copy = store.root / cap / item["name"]
    original = store.log_root / item["segment"] / "rlog.zst"
    assert copy.read_bytes() == original.read_bytes()
    assert item["sha256"] == hashlib.sha256(copy.read_bytes()).hexdigest()


def test_pending_survives_process_restart_and_missing_context_is_explicit(store):
  segment(store.log_root, 0)
  cap = store.accept(sample(), "route", 1000)
  restored = CaptureStore(store.log_root, store.root, min_free_bytes=0)
  restored.tick(1181)
  restored.tick(1186)
  manifest = read_manifest(store.root / cap)
  assert manifest["status"] == "partial"
  assert len(manifest["files"]) == 1
  assert manifest["missing"] == [{"segment": "route--1", "reason": "not_available"}]


def test_cooldown_per_topic_quota_and_ttl_only_remove_our_copies(store):
  original = segment(store.log_root, 0)
  first = store.accept(sample(), "route", 1000)
  assert store.accept(sample(), "route", 1001) is None
  second = store.accept(sample(), "route", 1120)
  third = store.accept(sample(), "route", 1240)
  assert not (store.root / first).exists()
  assert (store.root / second).exists() and (store.root / third).exists()
  for topic in ("engage_warning", "curve", "unwind", "braking"):
    store.accept(sample(topic), "route", 1241)
    store.accept(sample(topic), "route", 1361)
  assert len(store.entries()) == 10
  store.prune(1400 + TTL_SECONDS)
  assert store.entries() == []
  assert (original / "rlog.zst").read_bytes() == b"full-rlog"


def test_budget_and_free_space_do_not_copy_unbounded_logs(store, monkeypatch):
  segment(store.log_root, 0, b"x" * 200000)
  store.max_bytes = 200000
  cap = store.accept(sample(), "route", 1000)
  assert cap
  store.tick(1181)
  store.tick(1186)
  manifest = read_manifest(store.root / cap)
  assert manifest["files"] == []
  assert manifest["status"] == "partial"
  assert store.size_bytes() < store.max_bytes
  assert not list((store.root / cap).glob("*.part"))


def test_route_and_symlink_boundaries(store, tmp_path):
  outside = tmp_path / "outside"
  outside.mkdir()
  (outside / "rlog.zst").write_bytes(b"private")
  (store.log_root / "route--0").symlink_to(outside, target_is_directory=True)
  assert store.accept(sample(), "../outside", 1000) is None
  assert store.accept(sample(), "route", 1000) is None
  with pytest.raises(ValueError):
    store._remove(outside)
  assert (outside / "rlog.zst").read_bytes() == b"private"


def test_branch_gate_does_not_enable_on_comparison_branch():
  from types import SimpleNamespace
  from openpilot.system.manager.process_config import dk_diagnostics_enabled
  for branch in ("dkcarrot-wip", b"dkcarrot-wip"):
    assert dk_diagnostics_enabled(False, SimpleNamespace(get=lambda key, value=branch: value), None)
  for branch in (None, "carrot-wip", "carrot", "carrot-bmr_v6"):
    assert not dk_diagnostics_enabled(True, SimpleNamespace(get=lambda key, value=branch: value), None)


def test_orphan_and_corrupt_manifest_directories_obey_ttl_count_and_size_limits(store):
  original = segment(store.log_root, 0)
  for index in range(12):
    directory = store.root / f"{index:032x}"
    directory.mkdir()
    (directory / "manifest.json").write_text("invalid")
    (directory / "0-rlog").write_bytes(b"x" * 100)
    os.utime(directory, (1000 + index, 1000 + index))
  store.prune(1100)
  assert len(store.directories()) == 10
  store.max_bytes = 150
  store.prune(1101)
  assert store.size_bytes() <= 150
  store.prune(1200 + TTL_SECONDS)
  assert store.directories() == []
  assert (original / "rlog.zst").exists()


def test_deep_or_malformed_records_do_not_interrupt_later_captures(store):
  assert parse_record('[' * 2000 + ']' * 2000) is None
  segment(store.log_root, 0)
  assert store.accept({**sample(), "topics": [{}]}, "route", 1000) is None
  cap = store.accept(sample(), "route", 1000)
  path = store.root / cap / "manifest.json"
  valid = json.loads(path.read_text())
  for key in ("expires_at", "missing"):
    malformed = dict(valid)
    malformed.pop(key)
    path.write_text(json.dumps(malformed))
    assert read_manifest(path.parent) is None
  path.write_text('[' * 2000 + ']' * 2000)
  assert read_manifest(path.parent) is None


def test_unlocked_file_must_remain_stable_across_ticks_before_copy(store):
  path = segment(store.log_root, 0) / "rlog.zst"
  cap = store.accept(sample(), "route", 1000)
  store.tick(1001)
  assert read_manifest(store.root / cap)["files"] == []
  path.write_bytes(b"final compression footer")
  store.tick(1006)
  assert read_manifest(store.root / cap)["files"] == []
  store.tick(1011)
  manifest = read_manifest(store.root / cap)
  assert len(manifest["files"]) == 1
  assert (store.root / cap / manifest["files"][0]["name"]).read_bytes() == path.read_bytes()


def test_retained_full_rlogs_are_decodable_by_offline_report(store):
  from openpilot.cereal import log
  from tools.car_porting.dk_diagnostics_report import analyze_local  # noqa: TID251

  events = []
  session = {"event": "dk_vehicle_diag", "schema": 1, "kind": "session", "branch": "dkcarrot-wip", "commit": "abcdef1"}
  for record in (session, sample("curve")):
    event = log.Event.new_message()
    event.logMonoTime = 123456
    event.logMessage = json.dumps({"msg": record})
    events.append(event.to_bytes())
  before = store.log_root / "route--0"
  before.mkdir()
  (before / "rlog").write_bytes(b"".join(events))
  store.accept(session, "route", 1000)
  cap = store.accept(sample("curve"), "route", 1000)
  after = store.log_root / "route--1"
  after.mkdir()
  (after / "rlog").write_bytes(b"".join(events))
  store.tick(1001)
  store.tick(1006)
  manifest = read_manifest(store.root / cap)
  assert manifest["status"] == "ready"
  report = analyze_local([store.root / cap / item["name"] for item in manifest["files"]])
  assert report["counts"]["samples"] == 2
  assert report["metadata"]["commit"] == ["abcdef1"]
  assert report["source_service_counts"]["logMessage"] == 4
