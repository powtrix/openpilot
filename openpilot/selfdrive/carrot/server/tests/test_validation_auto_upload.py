from __future__ import annotations

import asyncio
import json
import stat
import threading
from pathlib import Path

import pytest

from openpilot.cereal import car, messaging
from openpilot.selfdrive.carrot.server.features.dashcam import upload, upload_jobs
from openpilot.selfdrive.carrot.server.features.dashcam import routes as dashcam_routes
from openpilot.selfdrive.carrot.server.services import validation_auto_upload as auto_upload
from openpilot.system.loggerd.deleter import (
  PRESERVE_ATTR_NAME,
  PRESERVE_ATTR_VALUE,
  VALIDATION_PRESERVE_COUNT,
)

from opendbc.car.hyundai.values import CAR, HyundaiFlags

TEST_DONGLE_ID = "test-dk-validation-device"


class FakeParams:
  def __init__(self, values=None):
    self.values = {"DongleId": TEST_DONGLE_ID, **dict(values or {})}

  def get(self, key):
    return self.values.get(key)

  def get_int(self, key):
    return int(self.values.get(key) or 0)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put_bool(self, key, value):
    self.values[key] = bool(value)


@pytest.fixture(autouse=True)
def trusted_validation_receiver(monkeypatch):
  monkeypatch.setattr(auto_upload, "VALIDATION_UPLOAD_BASE_URL", auto_upload.DK_VALIDATION_UPLOAD_ORIGIN)
  monkeypatch.setattr(
    auto_upload,
    "DK_VALIDATION_ALLOWED_DEVICE_ID_SHA256",
    frozenset({auto_upload.hashlib.sha256(TEST_DONGLE_ID.encode("utf-8")).hexdigest()}),
  )


def safe_device_state() -> bool:
  return True


def safe_network_state() -> bool:
  return True


def validation_receipt(
  device_id: str,
  capture_id: str,
  files: list[dict],
  metadata: dict | None = None,
) -> dict:
  manifest = auto_upload.validation_manifest_sha256(
    device_id, capture_id, files, metadata,
  )
  return {
    "ok": True,
    "receiptVersion": 1,
    "receiptId": auto_upload.validation_receipt_id(manifest),
    "manifestSha256": manifest,
    "files": files,
    "verifiedDeviceId": device_id,
    "deviceId": device_id,
    "captureId": capture_id,
  }


def test_validation_receiver_defaults_and_canonicalizes_only_to_pinned_origin():
  expected = auto_upload.DK_VALIDATION_UPLOAD_ORIGIN
  assert auto_upload._configured_validation_upload_base_url("") == expected
  assert auto_upload._configured_validation_upload_base_url("  ") == expected
  assert auto_upload._configured_validation_upload_base_url("https://ADOT.SYNOLOGY.ME/") == expected
  assert auto_upload._configured_validation_upload_base_url("https://adot.synology.me:443/") == expected


@pytest.mark.parametrize("override", [
  "http://trusted.example",
  "https://user:password@trusted.example",
  "not-a-url",
  "https://example.com",
  "https://api.commadotai.com",
  "https://connect.comma.ai/validation",
  "https://tmux.carrotpilot.app",
  "https://subdomain.carrotpilot.app/validation",
  "https://shind0.synology.me",
  "https://upload.shind0.synology.me/validation",
  "https://op.wjcloud.kr",
  "http://adot.synology.me",
  "https://user:password@adot.synology.me",
  "https://sub.adot.synology.me",
  "https://adot.synology.me.example",
  "https://adot-synology.me",
  "https://adot.synology.me:444",
  "https://adot.synology.me:",
  "https://adot.synology.me:0443",
  "https://adot.synology.me/api/v1/validation",
  "https://adot.synology.me/?",
  "https://adot.synology.me/#",
  "https://adot.synology.me/?receiver=other",
  "https://adot.synology.me/#other",
])
def test_every_unpinned_validation_receiver_override_fails_closed(override):
  assert auto_upload._configured_validation_upload_base_url(override) == ""


def test_private_nas_remains_a_valid_validation_receiver():
  assert auto_upload._configured_validation_upload_base_url("https://adot.synology.me/") == (
    "https://adot.synology.me"
  )


def test_empty_trusted_receiver_can_never_validate_campaign(monkeypatch):
  monkeypatch.setattr(auto_upload, "VALIDATION_UPLOAD_BASE_URL", "")
  campaign = {
    "id": "campaign",
    "started_at": 100,
    "expires_at": 200,
    "base_url": "",
  }
  state = auto_upload._default_state()
  state["campaign"] = campaign

  assert auto_upload._sanitize_state(state)["status"] == "state_invalid"
  allowed, error = auto_upload._upload_runtime_safety_allows(
    FakeParams(),
    campaign,
    device_state_safe=lambda: pytest.fail("invalid receiver must fail first"),
    network_state_safe=lambda: pytest.fail("invalid receiver must fail before network guard"),
  )
  assert allowed is False
  assert "not trusted" in error


def sample(now: float, **overrides) -> auto_upload.ValidationSample:
  values = {
    "now": now,
    "standstill": False,
    "cruise_enabled": True,
    "can_valid": True,
    "engaged": True,
    "lat_active": True,
    "v_ego": 15.0,
    "a_ego": 0.0,
    "steering_pressed": False,
    "brake_pressed": False,
    "gas_pressed": False,
    "brake_hold_active": False,
    "parking_brake": False,
    "acc_faulted": False,
    "cancel_requested": False,
    "physical_res_pressed": False,
    "ka4_keepalive_request_count": 0,
    "ka4_keepalive_qualified": False,
    "ka4_keepalive_stopped_sec": 0.0,
    "lane_plan_valid": True,
    "use_lane_lines": True,
    "static_path_offset": 0.0,
    "dynamic_lane_offset": 0.0,
    "radar_valid": True,
    "lead_status": False,
    "lead_d_rel": 0.0,
    "lead_v_rel": 0.0,
  }
  values.update(overrides)
  return auto_upload.ValidationSample(**values)


def advance_detector(
  detector: auto_upload.ValidationEventDetector,
  settings: dict[str, int],
  until: float,
  *,
  steady: dict | None = None,
  final: dict | None = None,
) -> list[dict]:
  """Feed continuous 10 Hz samples; dwell must never rely on missing telemetry."""
  steady = dict(steady or {})
  final_values = {**steady, **dict(final or {})}
  assert detector.last_sample_at is not None
  current = detector.last_sample_at + 0.1
  events = []
  while current < until - 1e-6:
    events.extend(detector.update(sample(current, **steady), settings))
    current += 0.1
  events.extend(detector.update(sample(until, **final_values), settings))
  return events


def test_detector_captures_standstill_start_without_control_intervention():
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": -1, "AdjustLaneOffset": 0}

  assert detector.update(sample(0.0, standstill=True, v_ego=0.0), settings) == []
  events = advance_detector(detector, settings, 0.5, steady={"standstill": True, "v_ego": 0.0})
  assert [event["condition"] for event in events] == ["standstill_on_no_request"]
  assert events[0]["trigger"] == "standstill_observed"
  assert events[0]["qualified"] is False
  assert advance_detector(detector, settings, 2.0, steady={"standstill": True, "v_ego": 0.0}) == []

  # Retain compatibility with a queued diagnostic from the superseded build
  # if its controller counter changed before the observation dwell elapsed.
  detector.reset()
  detector.update(sample(0.0, standstill=True, v_ego=0.0, ka4_keepalive_request_count=0), settings)
  events = advance_detector(
    detector,
    settings,
    0.3,
    steady={"standstill": True, "v_ego": 0.0},
    final={
      "ka4_keepalive_request_count": 1,
      "ka4_keepalive_qualified": True,
      "ka4_keepalive_stopped_sec": 0.3,
    },
  )
  assert [event["condition"] for event in events] == ["standstill_on"]
  assert events[0]["trigger"] == "keepalive_requested"
  assert events[0]["keepaliveRequestDelta"] == 1

  detector.reset()
  detector.update(sample(0.0, standstill=True, v_ego=0.0, ka4_keepalive_request_count=7), settings)
  events = advance_detector(
    detector,
    settings,
    0.5,
    steady={"standstill": True, "v_ego": 0.0, "ka4_keepalive_request_count": 0},
  )
  assert [event["condition"] for event in events] == ["standstill_on_no_request"]
  assert events[0]["trigger"] == "standstill_observed"


def test_detector_observes_standstill_despite_interlocks_and_classifies_stable_lane_offsets():
  detector = auto_upload.ValidationEventDetector()
  stop_settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": -1, "AdjustLaneOffset": 0}
  detector.update(sample(0.0, standstill=True, v_ego=0.0), stop_settings)
  events = advance_detector(
    detector,
    stop_settings,
    0.5,
    steady={"standstill": True, "v_ego": 0.0, "brake_pressed": True},
  )
  assert [event["trigger"] for event in events] == ["standstill_observed"]

  for path_offset, published, condition in (
    (0, 0.0, "lane_offset_0"),
    (10, 0.1, "lane_offset_10"),
  ):
    detector.reset()
    settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": path_offset, "AdjustLaneOffset": 0}
    assert detector.update(sample(0.0, static_path_offset=published), settings) == []
    events = advance_detector(detector, settings, 16.0, steady={"static_path_offset": published})
    assert [event["condition"] for event in events] == [condition]
    assert advance_detector(detector, settings, 17.0, steady={"static_path_offset": published}) == []


def test_detector_captures_sustained_stock_scc_acceleration_while_closing():
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": -1, "AdjustLaneOffset": 0}
  closing = {
    "v_ego": 12.0,
    "a_ego": 0.9,
    "radar_valid": True,
    "lead_status": True,
    "lead_d_rel": 16.0,
    "lead_v_rel": -1.2,
  }

  assert detector.update(sample(0.0, **closing), settings) == []
  events = advance_detector(detector, settings, 0.6, steady=closing)

  assert [event["condition"] for event in events] == ["stock_scc_close_accel"]
  assert events[0]["trigger"] == "accelerating_while_closing"
  assert events[0]["timeGap"] < 1.5

  detector.reset()
  assert detector.update(sample(0.0, gas_pressed=True, **closing), settings) == []
  assert advance_detector(detector, settings, 1.0, steady={"gas_pressed": True, **closing}) == []


def test_detector_breaks_dwell_across_telemetry_gap():
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": 0, "AdjustLaneOffset": 0}
  assert detector.update(sample(0.0), settings) == []

  # A gap larger than the service tolerance must restart, not complete, dwell.
  assert detector.update(sample(20.0), settings) == []
  assert detector.lane_started_at == 20.0

  detector.invalidate_sample()
  assert detector.update(sample(21.0, standstill=True, v_ego=0.0), settings) == []
  assert detector.stop_started_at == 21.0


def test_detector_rearms_when_event_was_not_durably_recorded():
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": -1, "AdjustLaneOffset": 0}
  detector.update(sample(0.0, standstill=True, v_ego=0.0), settings)
  events = advance_detector(detector, settings, 0.5, steady={"standstill": True, "v_ego": 0.0})
  assert [event["condition"] for event in events] == ["standstill_on_no_request"]

  detector.nack("standstill_on_no_request")
  retried = detector.update(sample(0.6, standstill=True, v_ego=0.0), settings)
  assert [event["condition"] for event in retried] == ["standstill_on_no_request"]


def test_campaign_condition_partition_keeps_legacy_ids_restore_only():
  assert auto_upload.TARGET_CONDITIONS == {
    "standstill_on_no_request",
    "lane_offset_10",
    "stock_scc_close_accel",
  }
  assert auto_upload.OPTIONAL_CONDITIONS == {
    "standstill_on",
    "lane_offset_0",
  }
  assert auto_upload.LEGACY_CONDITIONS == {
    "standstill_off",
    "standstill_off_physical_res",
  }
  assert auto_upload.TARGET_CONDITIONS.isdisjoint(auto_upload.OPTIONAL_CONDITIONS)
  assert auto_upload.TARGET_CONDITIONS.isdisjoint(auto_upload.LEGACY_CONDITIONS)
  assert auto_upload.OPTIONAL_CONDITIONS.isdisjoint(auto_upload.LEGACY_CONDITIONS)
  assert auto_upload.CAPTURE_CONDITIONS == (
    auto_upload.TARGET_CONDITIONS
    | auto_upload.OPTIONAL_CONDITIONS
    | auto_upload.LEGACY_CONDITIONS
  )
  assert auto_upload.NEW_CAMPAIGN_CONDITIONS == (
    auto_upload.TARGET_CONDITIONS | auto_upload.OPTIONAL_CONDITIONS
  )
  assert auto_upload.MAX_NEW_CAMPAIGN_CAPTURES == 10
  assert auto_upload.MAX_NEW_CAMPAIGN_RLOGS == 30
  assert auto_upload.MAX_COMPAT_CAPTURE_RECORDS == 14
  assert auto_upload.MAX_COMPAT_RLOG_RECORDS == 42


class ActualMessageSubMaster:
  def __init__(self):
    self.data = {name: messaging.new_message(name).__getattribute__(name) for name in (
      "carState", "carControl", "selfdriveState", "lateralPlan", "radarState",
    )}
    self.valid = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)
    self.updated = dict.fromkeys(self.data, True)
    self.logMonoTime = dict.fromkeys(self.data, 1_000_000_000)

  def __getitem__(self, name):
    return self.data[name]

  def observe_at(self, now: float) -> None:
    self.updated = dict.fromkeys(self.data, True)
    self.logMonoTime = dict.fromkeys(self.data, round(now * 1e9))


def test_sample_uses_radar_error_bits_not_truthiness_of_capnp_struct():
  sm = ActualMessageSubMaster()
  sm["carState"].canValid = True
  sm["carState"].cruiseState.enabled = True
  sm["selfdriveState"].enabled = True
  sm["radarState"].leadOne.status = True
  sm["radarState"].leadOne.dRel = 15.0
  sm["radarState"].leadOne.vRel = -1.0

  assert bool(sm["radarState"].radarErrors) is True  # the empty Cap'n Proto struct is truthy
  valid = auto_upload.sample_from_submaster(sm, 1.0)
  assert valid is not None
  assert valid.radar_valid is True
  assert valid.lead_status is True

  sm["radarState"].radarErrors.canError = True
  invalid = auto_upload.sample_from_submaster(sm, 1.1)
  assert invalid is not None
  assert invalid.radar_valid is False


def test_sample_rejects_unupdated_or_stale_core_cereal_messages():
  sm = ActualMessageSubMaster()
  sm.observe_at(10.0)
  assert auto_upload.sample_from_submaster(sm, 10.0) is not None

  sm.updated["carState"] = False
  assert auto_upload.sample_from_submaster(sm, 10.1) is None
  sm.updated["carState"] = True
  assert auto_upload.sample_from_submaster(sm, 10.36) is None


def _configure_close_accel_sample(sm: ActualMessageSubMaster) -> None:
  sm["carState"].canValid = True
  sm["carState"].cruiseState.enabled = True
  sm["carState"].vEgo = 12.0
  sm["carState"].aEgo = 0.8
  sm["selfdriveState"].enabled = True
  sm["radarState"].leadOne.status = True
  sm["radarState"].leadOne.dRel = 20.0
  sm["radarState"].leadOne.vRel = -1.0


def test_stale_cereal_values_cannot_accumulate_close_accel_dwell():
  sm = ActualMessageSubMaster()
  _configure_close_accel_sample(sm)
  sm.observe_at(10.0)
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": 0, "AdjustLaneOffset": 0}
  events = []

  # Even if a malformed SubMaster facade leaves updated=True, the unchanged
  # logMonoTime ages out before the 0.5 s close-accel dwell can complete.
  for step in range(7):
    now = 10.0 + step * 0.1
    sample_value = auto_upload.sample_from_submaster(sm, now)
    if sample_value is None:
      detector.invalidate_sample()
    else:
      events.extend(detector.update(sample_value, settings))

  assert events == []


def test_fresh_ten_hz_cereal_samples_still_detect_close_accel():
  sm = ActualMessageSubMaster()
  _configure_close_accel_sample(sm)
  detector = auto_upload.ValidationEventDetector()
  settings = {"Ka4StockSccStandstillRearm": 0, "PathOffset": 0, "AdjustLaneOffset": 0}
  events = []

  for step in range(7):
    now = 10.0 + step * 0.1
    sm.observe_at(now)
    if step % 2:
      sm.updated["carControl"] = False
      sm.updated["selfdriveState"] = False
    sample_value = auto_upload.sample_from_submaster(sm, now)
    assert sample_value is not None
    events.extend(detector.update(sample_value, settings))

  assert [event["condition"] for event in events] == ["stock_scc_close_accel"]


def _car_params_bytes(*, fingerprint=str(CAR.KIA_CARNIVAL_4TH_GEN), openpilot_long=False,
                      camera_scc=False, hda2=False):
  cp = car.CarParams.new_message()
  cp.carFingerprint = fingerprint
  cp.pcmCruise = True
  cp.openpilotLongitudinalControl = openpilot_long
  flags = int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC)
  if camera_scc:
    flags |= int(HyundaiFlags.CAMERA_SCC)
  if hda2:
    flags |= int(HyundaiFlags.CANFD_HDA2)
  cp.flags = flags
  cp.safetyConfigs = [{"safetyModel": "hyundaiCanfd", "safetyParam": 0}]
  return cp.to_bytes()


def test_vehicle_gate_accepts_only_ka4_stock_radar_scc_without_longitudinal():
  accepted, metadata = auto_upload.ka4_stock_scc_gate(FakeParams({
    "DongleId": TEST_DONGLE_ID,
    "CarParamsPersistent": _car_params_bytes(),
  }))
  assert accepted is True
  assert metadata["gatePassed"] is True
  assert metadata["deviceAllowed"] is True
  assert metadata["safetyConfigs"]

  for kwargs in (
    {"fingerprint": "TEST_CAR"},
    {"openpilot_long": True},
    {"camera_scc": True},
    {"hda2": True},
  ):
    accepted, _metadata = auto_upload.ka4_stock_scc_gate(FakeParams({
      "DongleId": TEST_DONGLE_ID,
      "CarParamsPersistent": _car_params_bytes(**kwargs),
    }))
    assert accepted is False

  for device_id in (None, "", "another-device"):
    accepted, metadata = auto_upload.ka4_stock_scc_gate(FakeParams({
      "DongleId": device_id,
      "CarParamsPersistent": _car_params_bytes(),
    }))
    assert accepted is False
    assert metadata["reason"] == "device_not_allowed"
    assert metadata["deviceAllowed"] is False


def test_upload_runtime_gate_rechecks_exact_owner_before_live_or_network_guards():
  campaign = auto_upload._new_campaign(now=int(auto_upload.time.time()))
  guard_reads = []

  allowed, error = auto_upload._upload_runtime_safety_allows(
    FakeParams({
      "DongleId": "another-device",
      auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1,
      "IsOffroad": True,
    }),
    campaign,
    device_state_safe=lambda: guard_reads.append("device") or True,
    network_state_safe=lambda: guard_reads.append("network") or True,
  )

  assert allowed is False
  assert "allowlisted" in error
  assert guard_reads == []


def test_route_settings_publish_disabled_standstill_actuator_metadata():
  params = FakeParams({
    "Ka4StockSccStandstillRearm": 0,
    "PathOffset": 10,
    "AdjustLaneOffset": 0,
  })

  assert auto_upload._route_settings(params) == {
    "Ka4StockSccStandstillRearm": 0,
    "PathOffset": 10,
    "AdjustLaneOffset": 0,
  }


def _make_segment(root: Path, route: str, index: int, *, locked=False, size=8) -> str:
  name = f"{route}--{index}"
  segment = root / name
  segment.mkdir()
  (segment / "rlog.zst").write_bytes(b"r" * size)
  if locked:
    (segment / "rlog.lock").write_text("recording", encoding="utf-8")
  return name


def _persisted_capture(route: str, segment: str, index: int = 0, *, size: int = 8) -> dict:
  return {
    "id": f"{index + 1:032x}",
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {},
    "files": [],
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": size,
  }


def test_capture_segments_never_cross_a_recording_gap(tmp_path):
  route = "00000001--0123456789"
  _make_segment(tmp_path, route, 0)
  fourth = _make_segment(tmp_path, route, 4)
  fifth = _make_segment(tmp_path, route, 5)

  assert auto_upload.capture_segment_names(str(tmp_path), route, 5) == [fourth, fifth]


def test_close_accel_capture_includes_following_segment_when_available(tmp_path):
  route = "00000001--0123456789"
  previous = _make_segment(tmp_path, route, 3)
  anchor = _make_segment(tmp_path, route, 4)
  following = _make_segment(tmp_path, route, 5)

  assert auto_upload.capture_event_segment_names(
    str(tmp_path), route, 4, "stock_scc_close_accel",
  ) == [previous, anchor, following]
  assert auto_upload.capture_event_segment_names(
    str(tmp_path), route, 4, "standstill_off",
  ) == [previous, anchor]


def test_close_accel_following_segment_is_preserved_as_soon_as_directory_appears(tmp_path):
  route = "00000001--0123456789"
  anchor = _make_segment(tmp_path, route, 4)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {},
    "events": [{
      "condition": "stock_scc_close_accel",
      "anchor_segment": 4,
      "owned_preserve": [anchor],
    }],
  }

  unchanged, changed, owned, error = auto_upload._protect_close_accel_following_segments(
    state, str(tmp_path),
  )
  assert (unchanged, changed, owned, error) == (state, False, [], "")

  following = _make_segment(tmp_path, route, 5, locked=True)
  protected, changed, owned, error = auto_upload._protect_close_accel_following_segments(
    state, str(tmp_path),
  )

  assert (changed, owned, error) == (True, [following], "")
  assert protected["active_route"]["events"][0]["owned_preserve"] == [anchor, following]
  assert auto_upload.getxattr_direct(
    str(tmp_path / following), auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_close_accel_following_protection_does_not_scan_without_candidate(monkeypatch):
  route = "00000001--0123456789"
  state = auto_upload._default_state()
  state["active_route"] = {
    "route": route,
    "events": [{
      "condition": "lane_offset_0",
      "anchor_segment": 4,
      "owned_preserve": [f"{route}--4"],
    }],
  }
  scans = 0

  def count_scan(*_args):
    nonlocal scans
    scans += 1
    return []

  monkeypatch.setattr(auto_upload, "_segment_dirs", count_scan)

  result = auto_upload._protect_close_accel_following_segments(state, "/unused")

  assert result == (state, False, [], "")
  assert scans == 0


def test_close_accel_following_candidate_scan_is_throttled_to_one_hz(monkeypatch):
  route = "00000001--0123456789"
  state = auto_upload._default_state()
  state["active_route"] = {
    "route": route,
    "events": [{
      "condition": "stock_scc_close_accel",
      "anchor_segment": 4,
      "owned_preserve": [f"{route}--4"],
    }],
  }

  scans = 0

  def count_scan(*_args):
    nonlocal scans
    scans += 1
    return []

  monkeypatch.setattr(auto_upload, "_segment_dirs", count_scan)

  def poll(now, next_check_at):
    due, updated_check = auto_upload._close_accel_following_scan_due(
      state, now=now, next_check_at=next_check_at,
    )
    if due:
      auto_upload._protect_close_accel_following_segments(state, "/unused")
    return due, updated_check

  due, next_check = poll(10.0, 0.0)
  assert (due, next_check) == (True, 11.0)
  assert poll(10.5, next_check) == (False, 11.0)
  assert poll(10.99, next_check) == (False, 11.0)
  assert scans == 1
  assert poll(11.0, next_check) == (True, 12.0)
  assert scans == 2


def test_route_setting_epoch_preserves_each_events_detection_settings(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["active_route"] = {
    "route": route,
    "settings_epoch": 0,
    "settings": {"Ka4StockSccStandstillRearm": 0, "PathOffset": 0, "AdjustLaneOffset": 0},
    "identity": {},
    "events": [{
      "condition": "lane_offset_0",
      "anchor_segment": 0,
      "owned_preserve": [segment],
      "detected_at": 101,
    }],
  }

  changed_state, changed = auto_upload._update_active_route_settings(state, {
    "Ka4StockSccStandstillRearm": 0,
    "PathOffset": 10,
    "AdjustLaneOffset": 0,
  })

  assert changed is True
  active = changed_state["active_route"]
  assert active["settings_epoch"] == 1
  assert active["settings"]["PathOffset"] == 10
  assert active["events"][0]["settings_epoch"] == 0
  assert active["events"][0]["settings"]["PathOffset"] == 0

  queued, _changed = auto_upload.enqueue_active_route_captures(
    changed_state, str(tmp_path), now_epoch=200,
  )
  assert queued["queue"][0]["metadata"]["routeSettings"]["PathOffset"] == 0
  assert queued["queue"][0]["metadata"]["settingsEpoch"] == 0


def test_complete_segment_stays_true_when_a_lower_priority_rlog_is_empty(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  (tmp_path / segment / "rlog").write_bytes(b"")

  assert auto_upload.segment_is_complete_at(str(tmp_path), segment) is True


def test_validation_retention_release_never_clears_user_bookmark(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  path = str(tmp_path / segment)
  auto_upload.setxattr(path, PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)

  owned, error = auto_upload._preserve_segments(str(tmp_path), [segment])

  assert error == ""
  assert owned == [segment]
  assert auto_upload._release_preserve(str(tmp_path), segment) is True
  assert auto_upload.getxattr_direct(path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE
  assert auto_upload.getxattr_direct(path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME) == b"0"


def test_enqueue_keeps_prior_capture_when_later_event_is_not_finalized(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  first = _make_segment(tmp_path, route, 0)
  _make_segment(tmp_path, route, 1, locked=True)
  state = auto_upload._default_state()
  state["campaign"] = {"id": "campaign", "base_url": auto_upload.DK_VALIDATION_UPLOAD_ORIGIN}
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {},
    "events": [
      {"condition": "standstill_off", "anchor_segment": 0},
      {"condition": "lane_offset_0", "anchor_segment": 1},
    ],
  }
  monkeypatch.setattr(auto_upload, "_preserve_segments", lambda root, segments: (list(segments), ""))

  state, changed = auto_upload.enqueue_active_route_captures(state, str(tmp_path))

  assert changed is True
  assert [item["condition"] for item in state["queue"]] == ["standstill_off"]
  assert state["queue"][0]["segments"] == [first]
  assert state["active_route"] is not None


def test_enqueue_journals_prior_cleanup_before_queue_limit_return(tmp_path):
  route = "00000001--0123456789"
  obsolete = _make_segment(tmp_path, route, 0)
  current = _make_segment(tmp_path, route, 1)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [
    {
      "id": f"existing-{index}",
      "condition": "standstill_off",
      "bytes": 1,
      "created_at": index + 1,
    }
    for index in range(auto_upload.MAX_PENDING_CAPTURES)
  ]
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {},
    "events": [
      {"condition": "obsolete", "anchor_segment": 0, "owned_preserve": [obsolete]},
      {"condition": "lane_offset_0", "anchor_segment": 1, "owned_preserve": [current]},
    ],
  }

  result, changed = auto_upload.enqueue_active_route_captures(state, str(tmp_path), now_epoch=200)

  assert changed is True
  assert result["status"] == "queue_limit"
  assert result["cleanup_preserve"] == [obsolete]
  assert [event["condition"] for event in result["active_route"]["events"]] == ["lane_offset_0"]


def test_enqueue_preserve_failure_keeps_current_and_future_events_and_cleanup(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  obsolete = _make_segment(tmp_path, route, 0)
  _make_segment(tmp_path, route, 1)
  _make_segment(tmp_path, route, 2)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {},
    "events": [
      {"condition": "obsolete", "anchor_segment": 0, "owned_preserve": [obsolete]},
      {"condition": "lane_offset_0", "anchor_segment": 1, "owned_preserve": []},
      {"condition": "standstill_off", "anchor_segment": 2, "owned_preserve": []},
    ],
  }
  monkeypatch.setattr(auto_upload, "_preserve_segments", lambda *_args: ([], "xattr failed"))

  result, changed = auto_upload.enqueue_active_route_captures(state, str(tmp_path), now_epoch=200)

  assert changed is True
  assert result["status"] == "preserve_failed"
  assert result["cleanup_preserve"] == [obsolete]
  assert [event["condition"] for event in result["active_route"]["events"]] == [
    "lane_offset_0",
    "standstill_off",
  ]


def test_worker_does_not_adopt_enqueued_capture_when_durable_write_fails(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  next_route = "00000002--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=int(auto_upload.time.time()))
  state["active_route"] = {
    "route": route,
    "started_at": 100,
    "finalize_wait_started_at": 0,
    "settings_epoch": 0,
    "settings": {},
    "identity": {},
    "events": [{
      "condition": "standstill_off",
      "anchor_segment": 0,
      "owned_preserve": [segment],
      "detected_at": 100,
      "settings_epoch": 0,
      "settings": {},
    }],
  }
  params = FakeParams({
    auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "IsOnroad": True,
    "IsOffroad": False,
    "CurrentRoute": next_route,
  })
  device_state = type("DeviceState", (), {"started": True})()

  class FakeSubMaster:
    valid = {"deviceState": True}
    alive = {"deviceState": True}
    updated = {"deviceState": True}
    logMonoTime = {"deviceState": 1}

    def update(self, _timeout):
      return None

    def __getitem__(self, _name):
      return device_state

  attempted_states = []

  def fail_write(candidate, _path):
    attempted_states.append(candidate)
    return False

  async def stop_on_sleep(_delay):
    raise asyncio.CancelledError

  monkeypatch.setattr(auto_upload, "HAS_PARAMS", True)
  monkeypatch.setattr(auto_upload, "Params", lambda: params)
  monkeypatch.setattr(auto_upload, "read_validation_upload_state", lambda _path: state)
  monkeypatch.setattr(auto_upload, "write_validation_upload_state", fail_write)
  monkeypatch.setattr(auto_upload.messaging, "SubMaster", lambda _services: FakeSubMaster())
  monkeypatch.setattr(auto_upload.messaging, "sub_sock", lambda *_args, **_kwargs: object())
  monkeypatch.setattr(auto_upload.messaging, "drain_sock", lambda _sock: [])
  monkeypatch.setattr(auto_upload.asyncio, "sleep", stop_on_sleep)

  with pytest.raises(asyncio.CancelledError):
    asyncio.run(auto_upload._validation_auto_upload_worker(
      state_path=str(tmp_path / "state.json"),
      root=str(tmp_path),
      poll_interval=0.1,
    ))

  assert attempted_states and attempted_states[-1]["queue"]
  assert state["queue"] == []
  assert state["active_route"]["events"][0]["owned_preserve"] == [segment]
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_owned_anchor_is_transferred_to_overlapping_pending_capture():
  first = {"owned_preserve": ["route--2"]}
  remaining = [{"segments": ["route--1", "route--2"], "owned_preserve": []}]

  cleanup = auto_upload._transfer_or_defer_owned(first, remaining)

  assert remaining[0]["owned_preserve"] == ["route--2"]
  assert cleanup == []


def test_required_capture_can_evict_oldest_optional_without_losing_shared_owner():
  shared = "00000001--0123456789--2"
  state = auto_upload._default_state()
  state["queue"] = [
    {
      "id": "optional",
      "condition": "standstill_on",
      "segments": [shared],
      "owned_preserve": [shared],
      "created_at": 1,
    },
    {
      "id": "required",
      "condition": "stock_scc_close_accel",
      "segments": [shared],
      "owned_preserve": [],
      "created_at": 2,
    },
  ]

  assert auto_upload._evict_oldest_optional_capture(state) is True
  assert [item["id"] for item in state["queue"]] == ["required"]
  assert state["queue"][0]["owned_preserve"] == [shared]
  assert state["cleanup_preserve"] == []


def test_required_capture_evicts_optional_capture_to_fit_byte_budget(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  optional_segment = _make_segment(tmp_path, route, 0, size=8)
  required_anchor = _make_segment(tmp_path, route, 1, size=8)
  state = auto_upload._default_state()
  state["campaign"] = {"id": "campaign", "base_url": auto_upload.DK_VALIDATION_UPLOAD_ORIGIN}
  state["queue"] = [{
    "id": "optional",
    "condition": "standstill_on",
    "route": route,
    "segments": [optional_segment],
    "owned_preserve": [optional_segment],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 1,
    "bytes": 8,
  }]
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {},
    "events": [{
      "condition": "stock_scc_close_accel",
      "anchor_segment": 1,
      "owned_preserve": [required_anchor],
    }],
  }
  monkeypatch.setattr(auto_upload, "MAX_PENDING_BYTES", 20)

  result, changed = auto_upload.enqueue_active_route_captures(state, str(tmp_path), now_epoch=100)

  assert changed is True
  assert [capture["condition"] for capture in result["queue"]] == ["stock_scc_close_accel"]
  assert result["queue"][0]["bytes"] == 16
  assert result["active_route"] is None


def test_failed_retention_cleanup_remains_in_durable_journal(monkeypatch):
  state = auto_upload._default_state()
  state["cleanup_preserve"] = ["00000001--0123456789--0"]
  monkeypatch.setattr(auto_upload, "_release_preserve", lambda _root, _segment: False)

  assert auto_upload._finish_preserve_cleanup(state, "/tmp") is False
  assert state["cleanup_preserve"] == ["00000001--0123456789--0"]
  assert "will be retried" in state["last_error"]


def test_terminal_cleanup_moves_ownership_before_release():
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{"owned_preserve": ["00000001--0123456789--0"]}]
  state["active_route"] = {
    "events": [{"owned_preserve": ["00000002--0123456789--1"]}],
  }

  terminal = auto_upload._terminal_cleanup_state(state, "expired")

  assert terminal["status"] == "expired"
  assert terminal["campaign"] is None
  assert terminal["queue"] == []
  assert terminal["active_route"] is None
  assert terminal["cleanup_preserve"] == [
    "00000001--0123456789--0",
    "00000002--0123456789--1",
  ]


def test_terminal_cleanup_maximum_ownership_survives_atomic_write_and_restart(tmp_path):
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  for capture_index in range(auto_upload.MAX_PENDING_CAPTURES):
    route = f"{capture_index + 1:08d}--0123456789"
    segments = [f"{route}--{index}" for index in range(auto_upload.MAX_SEGMENTS_PER_CAPTURE)]
    capture = _persisted_capture(route, segments[-1], capture_index)
    capture["segments"] = segments
    capture["owned_preserve"] = segments
    state["queue"].append(capture)
  state["cleanup_preserve"] = [
    f"{index + 10:08d}--0123456789--0"
    for index in range(auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS // 2)
  ]

  terminal = auto_upload._terminal_cleanup_state(state, "expired")
  state_path = tmp_path / "state.json"

  assert auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS == VALIDATION_PRESERVE_COUNT == 30
  assert len(terminal["cleanup_preserve"]) == auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS
  assert auto_upload.write_validation_upload_state(terminal, str(state_path)) is True
  restored = auto_upload.read_validation_upload_state(str(state_path))
  assert restored["status"] == "expired"
  assert restored["queue"] == []
  assert restored["active_route"] is None
  assert restored["cleanup_preserve"] == terminal["cleanup_preserve"]


def test_invalid_candidate_never_replaces_existing_durable_owner_state(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [_persisted_capture(route, segment)]
  state_path = tmp_path / "state.json"
  assert auto_upload.write_validation_upload_state(state, str(state_path)) is True
  durable_bytes = state_path.read_bytes()

  candidate = dict(state)
  candidate["cleanup_preserve"] = [
    f"{index + 10:08d}--0123456789--0"
    for index in range(auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS + 1)
  ]

  assert auto_upload.write_validation_upload_state(candidate, str(state_path)) is False
  assert state_path.read_bytes() == durable_bytes
  assert auto_upload.read_validation_upload_state(str(state_path))["queue"][0]["id"] == f"{1:032x}"
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_cleanup_overlap_with_live_owner_fails_closed_and_never_clears_marker(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  raw["queue"] = [_persisted_capture(route, segment)]
  raw["cleanup_preserve"] = [segment]

  sanitized = auto_upload._sanitize_state(raw)
  assert sanitized["status"] == "state_invalid"
  assert auto_upload._reconcile_validation_preserve(sanitized, str(tmp_path)) == (False, False)
  auto_upload._finish_preserve_cleanup(raw, str(tmp_path))
  assert raw["status"] == "state_invalid"
  assert raw["cleanup_preserve"] == [segment]
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_reconcile_reasserts_durable_owner_and_journals_orphan_before_release(tmp_path):
  route = "00000001--0123456789"
  owner = _make_segment(tmp_path, route, 0)
  orphan = _make_segment(tmp_path, route, 1)
  orphan_path = str(tmp_path / orphan)
  auto_upload.setxattr(orphan_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)

  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [owner],
    "owned_preserve": [owner],
    "metadata": {},
    "files": [],
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]

  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))

  assert ready is True
  assert changed is True
  assert auto_upload.getxattr_direct(
    str(tmp_path / owner), auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE
  assert state["cleanup_preserve"] == [orphan]
  # Reconciliation only journals the orphan; release follows a durable write.
  assert auto_upload.getxattr_direct(orphan_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE

  assert auto_upload.write_validation_upload_state(state, str(tmp_path / "state.json")) is True
  auto_upload._finish_preserve_cleanup(state, str(tmp_path))
  assert auto_upload.getxattr_direct(orphan_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME) == b"0"


def test_reconcile_ignores_ordinary_segments_without_validation_marker(tmp_path):
  route = "00000001--0123456789"
  _make_segment(tmp_path, route, 0)

  changed, ready = auto_upload._reconcile_validation_preserve(auto_upload._default_state(), str(tmp_path))

  assert (changed, ready) == (False, True)


def test_reconcile_missing_root_keeps_cleanup_journal_and_blocks_ready(tmp_path):
  segment = "00000001--0123456789--0"
  state = auto_upload._default_state()
  state["cleanup_preserve"] = [segment]

  changed, ready = auto_upload._reconcile_validation_preserve(
    state, str(tmp_path / "not-mounted"),
  )

  assert (changed, ready) == (False, False)
  assert state["cleanup_preserve"] == [segment]


@pytest.mark.parametrize("failure", ["scandir", "getxattr"])
def test_reconcile_scan_errors_block_retention_ready(tmp_path, monkeypatch, failure):
  route = "00000001--0123456789"
  _make_segment(tmp_path, route, 0)
  if failure == "scandir":
    monkeypatch.setattr(auto_upload.os, "scandir", lambda _root: (_ for _ in ()).throw(OSError("scan")))
  else:
    monkeypatch.setattr(
      auto_upload,
      "getxattr_direct",
      lambda *_args: (_ for _ in ()).throw(OSError("xattr")),
    )

  state = auto_upload._default_state()
  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))

  assert (changed, ready) == (False, False)
  assert state["status"] == "preserve_recovery_failed"


def test_reconcile_batches_overflow_orphans_before_collection_can_resume(tmp_path):
  route = "00000001--0123456789"
  segments = [
    _make_segment(tmp_path, route, index)
    for index in range(auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS + 1)
  ]
  for segment in segments:
    auto_upload.setxattr(
      str(tmp_path / segment), auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
    )
  state = auto_upload._default_state()

  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))

  assert changed is True
  assert ready is False
  assert len(state["cleanup_preserve"]) == auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS
  assert auto_upload.write_validation_upload_state(state, str(tmp_path / "state.json")) is True
  assert auto_upload._finish_preserve_cleanup(state, str(tmp_path)) is True
  assert state["cleanup_preserve"] == []
  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))
  assert (changed, ready) == (True, True)
  assert len(state["cleanup_preserve"]) == 1
  assert state["cleanup_preserve"][0] in segments
  assert auto_upload.getxattr_direct(
    str(tmp_path / state["cleanup_preserve"][0]),
    auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_invalid_state_never_releases_existing_validation_markers(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path,
    auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
    PRESERVE_ATTR_VALUE,
  )
  invalid = auto_upload._default_state()
  invalid["status"] = "state_invalid"

  changed, ready = auto_upload._reconcile_validation_preserve(invalid, str(tmp_path))

  assert (changed, ready) == (False, False)
  assert invalid["cleanup_preserve"] == []
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE
  params = FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1})
  assert auto_upload._disable_invalid_state_consent(invalid, params) is True
  assert params.get_bool(auto_upload.VALIDATION_AUTO_UPLOAD_PARAM) is False
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


@pytest.mark.parametrize("owner_kind", ["queue", "event"])
def test_malformed_ownership_entry_fails_closed_without_orphan_release(tmp_path, owner_kind):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  if owner_kind == "queue":
    raw["queue"] = [{
      **_persisted_capture(route, segment),
      "route": "malformed-route",
    }]
  else:
    raw["active_route"] = {
      "route": route,
      "settings": {},
      "identity": {},
      "events": [{
        "condition": "malformed-condition",
        "anchor_segment": 0,
        "owned_preserve": [segment],
      }],
    }

  state = auto_upload._sanitize_state(raw)
  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))

  assert state["status"] == "state_invalid"
  assert (changed, ready) == (False, False)
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


@pytest.mark.parametrize("owner_kind", ["queue", "cleanup"])
def test_over_limit_ownership_fails_closed_without_releasing_truncated_marker(tmp_path, owner_kind):
  route = "00000001--0123456789"
  limit = (
    auto_upload.MAX_PENDING_CAPTURES
    if owner_kind == "queue"
    else auto_upload.MAX_VALIDATION_PRESERVE_SEGMENTS
  )
  segments = [_make_segment(tmp_path, route, index) for index in range(limit + 1)]
  dropped = segments[-1]
  dropped_path = str(tmp_path / dropped)
  auto_upload.setxattr(
    dropped_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  if owner_kind == "queue":
    raw["queue"] = [
      _persisted_capture(route, segment, index)
      for index, segment in enumerate(segments)
    ]
  else:
    raw["cleanup_preserve"] = segments

  state = auto_upload._sanitize_state(raw)
  changed, ready = auto_upload._reconcile_validation_preserve(state, str(tmp_path))

  assert state["status"] == "state_invalid"
  assert (changed, ready) == (False, False)
  assert auto_upload.getxattr_direct(
    dropped_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


@pytest.mark.parametrize("bad_id", [None, "", "not-a-capture-id", "g" * 32])
def test_retained_capture_requires_valid_id_and_nonempty_owned_list(tmp_path, bad_id):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  capture = _persisted_capture(route, segment)
  capture["id"] = bad_id
  raw["queue"] = [capture]

  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"

  capture["id"] = "a" * 32
  capture["owned_preserve"] = []
  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"


@pytest.mark.parametrize("owned", [
  lambda segment: [segment, segment],
  lambda segment: [segment] * (auto_upload.MAX_SEGMENTS_PER_CAPTURE + 1),
])
def test_retained_ownership_rejects_duplicates_and_overlong_lists(owned):
  route = "00000001--0123456789"
  segment = f"{route}--0"
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  capture = _persisted_capture(route, segment)
  capture["owned_preserve"] = owned(segment)
  raw["queue"] = [capture]

  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"


def test_cleanup_journal_rejects_duplicate_ownership():
  segment = "00000001--0123456789--0"
  raw = auto_upload._default_state()
  raw["cleanup_preserve"] = [segment, segment]

  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"


@pytest.mark.parametrize("suffix", ["00", "01", "0002"])
def test_noncanonical_segment_suffix_fails_closed(suffix):
  route = "00000001--0123456789"
  segment = f"{route}--{suffix}"
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  raw["queue"] = [_persisted_capture(route, segment)]

  assert auto_upload._safe_segment_name(segment) == ""
  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"


@pytest.mark.parametrize("owner_kind", ["oldest_queue_segment", "wrong_active_route"])
def test_semantically_wrong_owner_fails_closed_without_releasing_marker(
  tmp_path, owner_kind,
):
  route = "00000001--0123456789"
  segments = [_make_segment(tmp_path, route, index) for index in range(3)]
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  if owner_kind == "oldest_queue_segment":
    capture = _persisted_capture(route, segments[-1])
    capture["segments"] = segments
    capture["owned_preserve"] = [segments[0]]
    raw["queue"] = [capture]
    owned = segments[0]
  else:
    owned = "00000002--0123456789--0"
    (tmp_path / owned).mkdir()
    raw["active_route"] = {
      "route": route,
      "settings": {},
      "identity": {},
      "events": [{
        "condition": "lane_offset_0",
        "anchor_segment": 0,
        "owned_preserve": [owned],
      }],
    }
  auto_upload.setxattr(
    str(tmp_path / owned), auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )

  state = auto_upload._sanitize_state(raw)
  assert state["status"] == "state_invalid"
  assert auto_upload._reconcile_validation_preserve(state, str(tmp_path)) == (False, False)
  assert auto_upload.getxattr_direct(
    str(tmp_path / owned), auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_restored_queue_byte_accounting_is_strict_and_checked_against_disk(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0, size=8)
  segment_path = str(tmp_path / segment)
  auto_upload.setxattr(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE,
  )
  raw = auto_upload._default_state()
  raw["campaign"] = auto_upload._new_campaign(now=100)
  capture = _persisted_capture(route, segment)
  raw["queue"] = [capture]

  capture["bytes"] = "8"
  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"

  capture["bytes"] = auto_upload.MAX_PENDING_BYTES + 1
  assert auto_upload._sanitize_state(raw)["status"] == "state_invalid"

  capture["bytes"] = 1
  restored = auto_upload._sanitize_state(raw)
  assert restored["status"] != "state_invalid"
  assert auto_upload._validate_restored_queue_bytes(restored, str(tmp_path)) is False
  assert restored["status"] == "state_invalid"
  assert auto_upload._reconcile_validation_preserve(restored, str(tmp_path)) == (False, False)
  assert auto_upload.getxattr_direct(
    segment_path, auto_upload.VALIDATION_PRESERVE_ATTR_NAME,
  ) == PRESERVE_ATTR_VALUE


def test_restored_queue_rejects_cumulative_byte_limit():
  route = "00000001--0123456789"
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  half_plus_one = auto_upload.MAX_PENDING_BYTES // 2 + 1
  state["queue"] = [
    _persisted_capture(route, f"{route}--{index}", index, size=half_plus_one)
    for index in range(2)
  ]

  assert auto_upload._sanitize_state(state)["status"] == "state_invalid"


def test_state_sanitizer_handles_malformed_numeric_and_collection_fields():
  route = "00000001--0123456789"
  segment = f"{route}--0"
  raw = auto_upload._default_state()
  raw["last_uploaded_at"] = "not-an-int"
  raw["campaign"] = auto_upload._new_campaign(now=100)
  raw["active_route"] = {
    "route": route,
    "started_at": "bad",
    "finalize_wait_started_at": object(),
    "settings_epoch": "bad",
    "settings": {"PathOffset": "bad"},
    "events": [{
      "condition": "lane_offset_0",
      "anchor_segment": "bad",
      "owned_preserve": None,
      "duration": "bad",
      "keepaliveRequestDelta": object(),
      "controllerStoppedSec": "nan",
      "detected_at": [],
      "settings": {"AdjustLaneOffset": "bad"},
    }],
  }
  raw["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": None,
    "files": [{"segment": segment, "name": "rlog.zst", "size": "bad", "sha256": "a" * 64}],
    "attempts": "bad",
    "next_retry_at": object(),
    "created_at": "bad",
    "bytes": float("nan"),
  }]

  clean = auto_upload._sanitize_state(raw)

  assert clean["last_uploaded_at"] == 0
  assert clean["active_route"]["started_at"] == 0
  assert clean["active_route"]["events"][0]["duration"] == 0.0
  assert clean["active_route"]["events"][0]["owned_preserve"] == []
  assert clean["queue"][0]["attempts"] == 0
  assert clean["queue"][0]["created_at"] == 0
  assert clean["queue"][0]["files"] == []


def test_state_sanitizer_bounds_identity_and_capture_metadata_to_explicit_schema():
  route = "00000001--0123456789"
  segment = f"{route}--0"
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["active_route"] = {
    "route": route,
    "settings": {},
    "identity": {
      "branch": "carrot-wip",
      "commit": "a" * 40,
      "dirty": True,
      "secretRemote": "https://token@example.invalid/repo",
      "topology": {
        "carFingerprint": str(CAR.KIA_CARNIVAL_4TH_GEN),
        "pcmCruise": True,
        "openpilotLongitudinalControl": False,
        "flags": 123,
        "alternativeExperience": 0,
        "safetyConfigs": [{"model": "hyundaiCanfd", "param": 7, "secret": "drop"}],
        "gatePassed": True,
        "privateBlob": {"nested": ["drop"]},
      },
    },
    "events": [{
      "condition": "lane_offset_0",
      "anchor_segment": 0,
      "owned_preserve": [segment],
      "settings": {},
    }],
  }
  capture = _persisted_capture(route, segment)
  capture["metadata"] = {
    "schemaVersion": 1,
    "campaignId": state["campaign"]["id"],
    "captureId": capture["id"],
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "anchorSegment": 0,
    "duration": 10.5,
    "trigger": "duration",
    "keepaliveRequestDelta": 1,
    "qualified": True,
    "controllerStoppedSec": 10.5,
    "vEgo": 1.0,
    "aEgo": 0.5,
    "leadDRel": 12.0,
    "leadVRel": -1.0,
    "timeGap": 1.2,
    "ttc": 4.0,
    "detectedAt": 101,
    "settingsEpoch": 2,
    "routeSettings": {"PathOffset": 10},
    "git": state["active_route"]["identity"],
    "preciseLocation": "drop-me",
    "nestedSecret": {"value": ["drop-me"]},
  }
  state["queue"] = [capture]

  clean = auto_upload._sanitize_state(state)

  assert clean["status"] != "state_invalid"
  identity = clean["active_route"]["identity"]
  assert identity["branch"] == "carrot-wip"
  assert identity["topology"]["safetyConfigs"] == [{"model": "hyundaiCanfd", "param": 7}]
  metadata = clean["queue"][0]["metadata"]
  assert metadata["duration"] == 10.5
  assert metadata["routeSettings"]["PathOffset"] == 10
  assert metadata["git"]["topology"]["gatePassed"] is True
  serialized = json.dumps(clean)
  for secret in ("secretRemote", "privateBlob", "preciseLocation", "nestedSecret", "drop-me"):
    assert secret not in serialized


def test_state_is_atomic_sanitized_and_corruption_fails_closed(tmp_path):
  path = tmp_path / "state.json"
  state = auto_upload._default_state()
  state["status"] = "armed"
  state["campaign"] = auto_upload._new_campaign(now=100)
  assert auto_upload.write_validation_upload_state(state, str(path)) is True
  assert auto_upload.read_validation_upload_state(str(path))["campaign"]["base_url"] == auto_upload.DK_VALIDATION_UPLOAD_ORIGIN
  assert stat.S_IMODE(path.stat().st_mode) == 0o600

  path.write_text("{broken", encoding="utf-8")
  invalid = auto_upload.read_validation_upload_state(str(path))
  assert invalid["status"] == "state_invalid"
  assert invalid["campaign"] is None


def test_campaign_clock_rollback_blocks_upload_policy(monkeypatch):
  campaign = auto_upload._new_campaign(now=10_000)
  monkeypatch.setattr(auto_upload.time, "time", lambda: 9_000)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))

  allowed, error = auto_upload._upload_runtime_safety_allows(
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    campaign,
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
  )

  assert allowed is False
  assert "clock" in error


def test_campaign_is_pinned_to_immutable_validation_receiver(monkeypatch):
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://attacker.example", "token"))

  campaign = auto_upload._new_campaign(now=100)

  assert campaign["base_url"] == auto_upload.VALIDATION_UPLOAD_BASE_URL
  assert campaign["base_url"] != upload.upload_target_settings()[0]
  for untrusted in (
    "https://attacker.example",
    "https://adot.synology.me/subpath",
    "https://adot.synology.me////",
    "https://adot.synology.me/?",
  ):
    campaign["base_url"] = untrusted
    state = auto_upload._default_state()
    state["campaign"] = campaign
    assert auto_upload._sanitize_state(state)["status"] == "state_invalid"


@pytest.mark.parametrize("campaign_id", ["short", "A" * 24, "g" * 24, "a" * 25])
def test_campaign_id_requires_exact_lowercase_protocol_token(campaign_id):
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["campaign"]["id"] = campaign_id

  assert auto_upload._sanitize_state(state)["status"] == "state_invalid"


def test_new_campaign_uses_cryptographic_24_hex_token(monkeypatch):
  monkeypatch.setattr(auto_upload.secrets, "token_hex", lambda size: "ab" * size)

  campaign = auto_upload._new_campaign(now=100)

  assert campaign["id"] == "ab" * 12


def test_live_device_state_guard_rejects_missing_stale_invalid_and_started_samples():
  clock = [10.0]
  guard = auto_upload.LiveDeviceStateStoppedGuard(
    freshness_seconds=1.0,
    monotonic=lambda: clock[0],
  )
  device_state = type("DeviceState", (), {"started": False})()
  sm = type("SubMaster", (), {
    "valid": {"deviceState": True},
    "alive": {"deviceState": True},
    "updated": {"deviceState": True},
    "logMonoTime": {"deviceState": 100},
    "__getitem__": lambda self, _name: device_state,
  })()

  assert guard.allows_upload() is False
  guard.observe_submaster(sm)
  assert guard.allows_upload() is True
  sm.updated["deviceState"] = False
  clock[0] = 10.1
  sm.valid["deviceState"] = False
  guard.observe_submaster(sm)
  assert guard.allows_upload() is False
  sm.valid["deviceState"] = True
  sm.logMonoTime["deviceState"] = 200
  guard.observe_submaster(sm)
  assert guard.allows_upload() is True
  clock[0] = 11.2
  guard.observe_submaster(sm)
  assert guard.allows_upload() is False
  clock[0] = 12.0
  device_state.started = True
  sm.updated["deviceState"] = True
  sm.logMonoTime["deviceState"] = 300
  guard.observe_submaster(sm)
  assert guard.allows_upload() is False


def test_live_wifi_guard_rejects_missing_stale_changed_and_disconnected_samples():
  clock = [10.0]
  guard = auto_upload.LiveWifiGuard(
    freshness_seconds=0.5,
    monotonic=lambda: clock[0],
  )

  assert guard.allows_upload() is False
  guard.observe_network_type(auto_upload.log.DeviceState.NetworkType.wifi)
  assert guard.allows_upload() is True
  clock[0] = 10.6
  assert guard.allows_upload() is False
  guard.observe_network_type("cellular")
  assert guard.allows_upload() is False
  guard.observe_network_type(None)
  assert guard.allows_upload() is False
  guard.invalidate()
  assert guard.allows_upload() is False


def test_live_wifi_watcher_invalidates_immediately_on_network_change(monkeypatch):
  network_reads = 0

  def network_type():
    nonlocal network_reads
    network_reads += 1
    return auto_upload.log.DeviceState.NetworkType.wifi if network_reads == 1 else "cellular"

  monkeypatch.setattr(auto_upload.HARDWARE, "get_network_type", network_type)

  async def run():
    guard = auto_upload.LiveWifiGuard(freshness_seconds=1.0)
    initial = asyncio.Event()
    task = asyncio.create_task(auto_upload._refresh_live_wifi(
      guard,
      poll_interval=0.01,
      initial_observation=initial,
    ))
    await asyncio.wait_for(initial.wait(), timeout=1.0)
    assert guard.allows_upload() is True
    for _ in range(100):
      if not guard.allows_upload():
        break
      await asyncio.sleep(0.01)
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    return guard.allows_upload()

  assert asyncio.run(run()) is False


def test_upload_runtime_guard_consumes_live_wifi_cache():
  campaign = auto_upload._new_campaign(now=int(auto_upload.time.time()))
  params = FakeParams({
    auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "IsOffroad": True,
  })

  allowed, error = auto_upload._upload_runtime_safety_allows(
    params,
    campaign,
    device_state_safe=safe_device_state,
    network_state_safe=lambda: False,
  )

  assert allowed is False
  assert "Wi-Fi" in error


def test_upload_policy_uses_cached_wifi_without_direct_hardware_probe(monkeypatch):
  monkeypatch.setattr(
    auto_upload.HARDWARE,
    "get_network_type",
    lambda: pytest.fail("policy must not start an unbounded hardware probe"),
  )
  campaign = auto_upload._new_campaign(now=int(auto_upload.time.time()))

  allowed, error = asyncio.run(auto_upload._upload_policy_allows(
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    campaign,
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
  ))

  assert (allowed, error) == (True, "")


def test_repeated_upload_wrappers_share_one_blocked_wifi_probe(monkeypatch):
  release_probe = threading.Event()
  probe_stopped = threading.Event()
  calls = 0
  active = 0
  max_active = 0
  lock = threading.Lock()

  def blocked_network_probe():
    nonlocal calls, active, max_active
    with lock:
      calls += 1
      active += 1
      max_active = max(max_active, active)
    try:
      release_probe.wait(timeout=2.0)
      return auto_upload.log.DeviceState.NetworkType.wifi
    finally:
      with lock:
        active -= 1
      probe_stopped.set()

  class FakeDeviceGuard:
    @staticmethod
    def allows_upload():
      return True

  async def hold_device_watcher(*_args, **_kwargs):
    await asyncio.Event().wait()

  async def record_fail_closed(
    state, _params, *, device_state_safe, network_state_safe, state_path, root,
  ):
    assert device_state_safe() is True
    assert network_state_safe() is False
    return state

  async def run():
    monkeypatch.setattr(auto_upload.HARDWARE, "get_network_type", blocked_network_probe)
    monkeypatch.setattr(auto_upload, "_refresh_live_device_state", hold_device_watcher)
    monkeypatch.setattr(auto_upload, "_upload_first_capture", record_fail_closed)
    monkeypatch.setattr(auto_upload, "NETWORK_GUARD_INITIAL_TIMEOUT_SECONDS", 0.03)
    network_guard = auto_upload.LiveWifiGuard()
    for _ in range(2):
      await auto_upload._upload_first_capture_with_live_device_state(
        {"queue": []},
        FakeParams(),
        object(),
        FakeDeviceGuard(),
        network_guard,
        state_path="state",
        root="root",
        poll_interval=0.01,
      )
    assert calls == 1
    assert max_active == 1
    assert active == 1
    release_probe.set()
    assert await asyncio.to_thread(probe_stopped.wait, 1.0)
    assert active == 0

  asyncio.run(run())


def test_canceled_wifi_watcher_reaps_late_probe_exception(monkeypatch):
  release_probe = threading.Event()
  started = threading.Event()

  def failing_network_probe():
    started.set()
    release_probe.wait(timeout=2.0)
    raise RuntimeError("late platform failure")

  class FakeDeviceGuard:
    @staticmethod
    def allows_upload():
      return True

  async def hold_device_watcher(*_args, **_kwargs):
    await asyncio.Event().wait()

  async def record_fail_closed(
    state, _params, *, device_state_safe, network_state_safe, state_path, root,
  ):
    assert network_state_safe() is False
    return state

  async def run():
    loop = asyncio.get_running_loop()
    unhandled = []
    loop.set_exception_handler(lambda _loop, context: unhandled.append(context))
    monkeypatch.setattr(auto_upload.HARDWARE, "get_network_type", failing_network_probe)
    monkeypatch.setattr(auto_upload, "_refresh_live_device_state", hold_device_watcher)
    monkeypatch.setattr(auto_upload, "_upload_first_capture", record_fail_closed)
    monkeypatch.setattr(auto_upload, "NETWORK_GUARD_INITIAL_TIMEOUT_SECONDS", 0.03)
    network_guard = auto_upload.LiveWifiGuard()
    await auto_upload._upload_first_capture_with_live_device_state(
      {"queue": []},
      FakeParams(),
      object(),
      FakeDeviceGuard(),
      network_guard,
      state_path="state",
      root="root",
      poll_interval=0.01,
    )
    assert started.is_set()
    release_probe.set()
    for _ in range(100):
      if network_guard._hardware_probe is None:
        break
      await asyncio.sleep(0.01)
    assert network_guard._hardware_probe is None
    assert unhandled == []

  asyncio.run(run())


def test_manifest_hash_checks_live_safety_between_chunks(tmp_path):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0, size=3 * 1024 * 1024)
  checks = 0

  def stop_during_hash():
    nonlocal checks
    checks += 1
    return checks < 4

  files, error = auto_upload._capture_file_manifest(
    str(tmp_path), [segment], should_continue=stop_during_hash,
  )

  assert files == []
  assert error == "automatic upload safety changed while hashing"
  assert checks == 4


def test_state_rejects_captures_without_consent_bound_campaign():
  state = auto_upload._default_state()
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": "00000001--0123456789",
    "segments": ["00000001--0123456789--0"],
    "owned_preserve": ["00000001--0123456789--0"],
    "metadata": {},
    "files": [],
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]

  sanitized = auto_upload._sanitize_state(state)

  assert sanitized["status"] == "state_invalid"
  assert sanitized["queue"][0]["id"] == "a" * 32
  assert "no consent-bound campaign" in sanitized["last_error"]


def test_git_identity_does_not_collect_remote_url(monkeypatch):
  def fake_git_text(args, default=""):
    return {
      ("branch", "--show-current"): "carrot-wip",
      ("rev-parse", "HEAD"): "a" * 40,
      ("status", "--porcelain"): " M local-file",
    }.get(tuple(args), default)

  monkeypatch.setattr(upload, "git_text", fake_git_text)

  identity = auto_upload._git_identity(FakeParams())

  assert identity == {"branch": "carrot-wip", "commit": "a" * 40, "dirty": True}
  assert "remote" not in identity


def test_rlog_only_job_options_do_not_leak_or_notify_discord(monkeypatch):
  segment = "00000001--0123456789--0"
  seen = {"discord": 0}

  def fake_summary(_path, *, artifact_kinds=None):
    assert set(artifact_kinds or ()) == {"rlog"}
    return [{"kind": "rlog", "name": "rlog.zst", "size": 8}]

  files = [{"segment": segment, "name": "rlog.zst", "size": 8, "sha256": "a" * 64}]

  async def fake_validation_session(_base_url, metadata, *, should_continue=None):
    assert metadata["dongleId"] == "device"
    assert should_continue is None
    return "validation-token"

  async def fake_upload_folder(_path, uploaded_segment, capture_id, _url, token, expected_files, *_args, **_kwargs):
    assert (uploaded_segment, capture_id, token) == (segment, "capture", "validation-token")
    assert expected_files == files
    return True

  async def fake_complete(_base_url, _token, payload, *, should_continue=None):
    assert should_continue is None
    assert payload["validationCapture"] == {"captureId": "capture"}
    return {
      "status": 200,
      **validation_receipt(
        payload["deviceId"], payload["captureId"], payload["files"], payload.get("validationCapture"),
      ),
    }

  async def fail_discord(*_args, **_kwargs):
    seen["discord"] += 1
    raise AssertionError("automatic validation upload must not notify Discord")

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda _params: {"carName": "KA4", "dongleId": "device"})
  monkeypatch.setattr(upload, "upload_share_text", lambda _payload: "unused")
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda _segment: "/tmp/segment")
  monkeypatch.setattr(upload_jobs, "segment_file_summary", fake_summary)
  monkeypatch.setattr(upload_jobs, "create_validation_upload_session", fake_validation_session)
  monkeypatch.setattr(upload_jobs, "upload_validation_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_validation_upload_complete", fake_complete)
  monkeypatch.setattr(upload, "send_discord_webhook", fail_discord)

  upload_jobs.jobs().clear()
  options = {
    "artifact_kinds": frozenset({"rlog"}),
    "notify_discord": False,
    "concurrency_override": 1,
    "completion_metadata": {"captureId": "capture"},
    "validation_capture_id": "capture",
    "validation_files": files,
  }
  job = upload_jobs.create_job([segment], action="validation_auto_upload", run_options=options)
  assert upload_jobs.snapshot(job)["action"] == "validation_auto_upload"
  assert "_run_options" not in upload_jobs.snapshot(job)
  asyncio.run(upload_jobs.run_job(job))

  assert job["status"] == "done"
  assert job["result"]["discord"]["skipped"] is True
  assert seen["discord"] == 0
  upload_jobs.jobs().clear()


def test_automatic_upload_keeps_capture_queued_when_durable_commit_fails(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state_path = tmp_path / "validation-state.json"
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["status"] = "queued"
  state["queue"] = [{
    "id": "capture",
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {"captureId": "capture"},
    "files": auto_upload._capture_file_manifest(
      str(tmp_path), [segment], should_continue=safe_device_state,
    )[0],
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]
  write_results = iter((True, False))
  released = []

  async def finish_immediately(job):
    capture = state["queue"][0]
    upload_jobs.finish(job, ok=True, result={
      "ok": True,
      "deviceId": "device",
      "captureId": capture["id"],
      "results": [{"segment": segment, "ok": True}],
      "webComplete": validation_receipt(
        "device", capture["id"], capture["files"], capture["metadata"],
      ),
    })

  def fake_start_job(job):
    return asyncio.create_task(finish_immediately(job))

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload_jobs, "start_job", fake_start_job)
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)
  monkeypatch.setattr(
    auto_upload,
    "write_validation_upload_state",
    lambda _state, _path: next(write_results),
  )
  monkeypatch.setattr(auto_upload, "_release_preserve", lambda _root, name: released.append(name))

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(state_path),
    root=str(tmp_path),
  ))

  assert result["status"] == "state_write_failed"
  assert [item["id"] for item in result["queue"]] == ["capture"]
  assert result["completed"] == []
  assert result["last_uploaded_at"] == 0
  assert released == []
  upload_jobs.jobs().clear()


def test_automatic_upload_refuses_untrusted_receiver_in_campaign(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["campaign"]["base_url"] = "https://untrusted.example"
  state["queue"] = [{
    "id": "capture",
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]

  monkeypatch.setattr(
    upload_jobs,
    "create_job",
    lambda *_args, **_kwargs: pytest.fail("destination change must be rejected before creating a job"),
  )

  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=lambda: pytest.fail("untrusted receiver must fail before live guard consumption"),
    network_state_safe=lambda: pytest.fail("untrusted receiver must fail before network guard consumption"),
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["status"] == "destination_changed"
  assert result["queue"][0]["id"] == "capture"
  assert "not trusted" in result["last_error"]


def test_manual_upload_started_during_async_policy_check_wins_single_job_registry(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]

  async def policy_check_with_manual_start(*_args, **_kwargs):
    upload_jobs.create_job([segment], action="manual")
    return True, ""

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)
  monkeypatch.setattr(auto_upload, "_upload_policy_allows", policy_check_with_manual_start)

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["status"] == "manual_upload_active"
  assert len(upload_jobs.jobs()) == 1
  assert next(iter(upload_jobs.jobs().values()))["action"] == "manual"
  upload_jobs.jobs().clear()


def test_automatic_upload_cancels_child_when_safety_guard_raises(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {},
    "files": auto_upload._capture_file_manifest(
      str(tmp_path), [segment], should_continue=safe_device_state,
    )[0],
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]
  child_cancelled = False

  class GuardFailureParams(FakeParams):
    offroad_reads = 0

    def get_bool(self, key):
      if key == "IsOffroad":
        self.offroad_reads += 1
        if self.offroad_reads > 2:
          raise RuntimeError("params unavailable")
      return super().get_bool(key)

  async def wait_forever(_job):
    nonlocal child_cancelled
    try:
      await asyncio.Event().wait()
    finally:
      child_cancelled = True

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload_jobs, "start_job", lambda job: asyncio.create_task(wait_forever(job)))
  monkeypatch.setattr(auto_upload, "_retry_delay", lambda _attempts: 30)
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    GuardFailureParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert child_cancelled is True
  assert result["status"] == "retry_wait"
  assert result["queue"][0]["attempts"] == 1
  assert "safety check failed" in result["last_error"]
  upload_jobs.jobs().clear()


def test_automatic_upload_retries_when_completion_manifest_fails(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["status"] = "queued"
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {"captureId": "a" * 32},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]
  released = []

  async def finish_without_manifest(job):
    upload_jobs.finish(job, ok=True, result={
      "ok": True,
      "results": [{"segment": segment, "ok": True}],
      "webComplete": {"ok": False, "error": "manifest unavailable"},
    })

  def fake_start_job(job):
    return asyncio.create_task(finish_without_manifest(job))

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload_jobs, "start_job", fake_start_job)
  monkeypatch.setattr(auto_upload, "_retry_delay", lambda _attempts: 30)
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)
  monkeypatch.setattr(auto_upload, "_release_preserve", lambda _root, name: released.append(name))

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["status"] == "retry_wait"
  assert result["queue"][0]["attempts"] == 1
  assert result["queue"][0]["next_retry_at"] == 1_030
  assert result["last_error"] == "manifest unavailable"
  assert result["completed"] == []
  assert released == []
  upload_jobs.jobs().clear()


@pytest.mark.parametrize(("receipt_field", "replacement"), [
  ("receiptId", "0" * 64),
  ("receiptId", "UPPERCASE"),
  ("receiptId", "short"),
  ("deviceId", "different-device"),
  ("captureId", "different-capture"),
])
def test_automatic_upload_never_completes_with_mismatched_receipt(
  tmp_path, monkeypatch, receipt_field, replacement,
):
  route = "00000001--0123456789"
  segment = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [segment],
    "owned_preserve": [segment],
    "metadata": {"captureId": "a" * 32},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]

  async def finish_with_bad_receipt(job):
    capture = state["queue"][0]
    receipt = validation_receipt(
      "device", capture["id"], capture["files"], capture["metadata"],
    )
    if replacement == "UPPERCASE":
      receipt[receipt_field] = receipt[receipt_field].upper()
    else:
      receipt[receipt_field] = replacement
    upload_jobs.finish(job, ok=True, result={
      "ok": True,
      "deviceId": "device",
      "captureId": capture["id"],
      "results": [{"segment": segment, "ok": True}],
      "webComplete": receipt,
    })

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(
    upload_jobs,
    "start_job",
    lambda job: asyncio.create_task(finish_with_bad_receipt(job)),
  )
  monkeypatch.setattr(auto_upload, "_retry_delay", lambda _attempts: 30)
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["status"] == "retry_wait"
  assert result["queue"][0]["attempts"] == 1
  assert result["completed"] == []
  upload_jobs.jobs().clear()


def test_automatic_upload_skips_retrying_head_and_uploads_next_ready_capture(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  first = _make_segment(tmp_path, route, 0)
  second = _make_segment(tmp_path, route, 1)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [
    {
      "id": "a" * 32,
      "condition": "standstill_off",
      "route": route,
      "segments": [first],
      "owned_preserve": [first],
      "metadata": {},
      "attempts": 1,
      "next_retry_at": 2_000,
      "created_at": 100,
      "bytes": 8,
    },
    {
      "id": "b" * 32,
      "condition": "lane_offset_0",
      "route": route,
      "segments": [second],
      "owned_preserve": [second],
      "metadata": {},
      "attempts": 0,
      "next_retry_at": 0,
      "created_at": 100,
      "bytes": 8,
    },
  ]

  async def finish_ready(job):
    capture = state["queue"][0]
    upload_jobs.finish(job, ok=True, result={
      "ok": True,
      "deviceId": "device",
      "captureId": capture["id"],
      "results": [{"segment": second, "ok": True}],
      "webComplete": validation_receipt(
        "device", capture["id"], capture["files"], capture["metadata"],
      ),
    })

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload_jobs, "start_job", lambda job: asyncio.create_task(finish_ready(job)))
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert [item["id"] for item in result["queue"]] == ["a" * 32]
  assert [item["id"] for item in result["completed"]] == ["b" * 32]
  upload_jobs.jobs().clear()


def test_completed_capture_keeps_anchor_owned_by_still_active_event(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  shared = _make_segment(tmp_path, route, 0)
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["active_route"] = {
    "route": route,
    "started_at": 100,
    "finalize_wait_started_at": 100,
    "settings": {},
    "identity": {},
    "events": [{
      "condition": "lane_offset_0",
      "anchor_segment": 0,
      "owned_preserve": [shared],
      "detected_at": 100,
    }],
  }
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [shared],
    "owned_preserve": [shared],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]
  released = []

  async def finish_upload(job):
    capture = state["queue"][0]
    upload_jobs.finish(job, ok=True, result={
      "ok": True,
      "deviceId": "device",
      "captureId": capture["id"],
      "results": [{"segment": shared, "ok": True}],
      "webComplete": validation_receipt(
        "device", capture["id"], capture["files"], capture["metadata"],
      ),
    })

  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload_jobs, "start_job", lambda job: asyncio.create_task(finish_upload(job)))
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)
  monkeypatch.setattr(auto_upload, "_release_preserve", lambda _root, segment: released.append(segment) or True)

  upload_jobs.jobs().clear()
  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["queue"] == []
  assert result["active_route"]["events"][0]["owned_preserve"] == [shared]
  assert result["cleanup_preserve"] == []
  assert released == []
  upload_jobs.jobs().clear()


def test_unavailable_queued_capture_is_durably_quarantined_after_timeout(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  missing = f"{route}--0"
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [missing],
    "owned_preserve": [missing],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 100,
    "bytes": 8,
  }]
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)

  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(tmp_path / "state.json"),
    root=str(tmp_path),
  ))

  assert result["queue"] == []
  assert result["status"] == "capture_unavailable"
  assert auto_upload.read_validation_upload_state(str(tmp_path / "state.json"))["queue"] == []


def test_missing_capture_created_at_repair_is_persisted(tmp_path, monkeypatch):
  route = "00000001--0123456789"
  missing = f"{route}--0"
  state_path = tmp_path / "state.json"
  state = auto_upload._default_state()
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["queue"] = [{
    "id": "a" * 32,
    "condition": "standstill_off",
    "route": route,
    "segments": [missing],
    "owned_preserve": [missing],
    "metadata": {},
    "attempts": 0,
    "next_retry_at": 0,
    "created_at": 0,
    "bytes": 8,
  }]
  monkeypatch.setattr(auto_upload.time, "time", lambda: 1_000)

  result = asyncio.run(auto_upload._upload_first_capture(
    state,
    FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1, "IsOffroad": True}),
    device_state_safe=safe_device_state,
    network_state_safe=safe_network_state,
    state_path=str(state_path),
    root=str(tmp_path),
  ))

  assert result["status"] == "waiting_for_route_finalize"
  assert auto_upload.read_validation_upload_state(str(state_path))["queue"][0]["created_at"] == 1_000


def test_live_device_watcher_wraps_production_upload_and_is_cancelled(monkeypatch):
  refreshed = asyncio.Event()
  network_refreshed = asyncio.Event()
  cancelled = set()

  class FakeGuard:
    safe = False

    def allows_upload(self):
      return self.safe

  guard = FakeGuard()
  network_guard = FakeGuard()

  async def fake_refresh(_sm, live_guard, *, poll_interval):
    assert poll_interval == 0.01
    live_guard.safe = True
    refreshed.set()
    try:
      await asyncio.Event().wait()
    finally:
      cancelled.add("device")

  async def fake_network_refresh(live_guard, *, poll_interval, initial_observation):
    assert poll_interval == 0.01
    live_guard.safe = True
    network_refreshed.set()
    initial_observation.set()
    try:
      await asyncio.Event().wait()
    finally:
      cancelled.add("network")

  async def fake_upload(
    state, _params, *, device_state_safe, network_state_safe, state_path, root,
  ):
    assert refreshed.is_set()
    assert network_refreshed.is_set()
    assert device_state_safe() is True
    assert network_state_safe() is True
    assert (state_path, root) == ("state", "root")
    return {**state, "wrapped": True}

  async def run():
    monkeypatch.setattr(auto_upload, "_refresh_live_device_state", fake_refresh)
    monkeypatch.setattr(auto_upload, "_refresh_live_wifi", fake_network_refresh)
    monkeypatch.setattr(auto_upload, "_upload_first_capture", fake_upload)
    return await auto_upload._upload_first_capture_with_live_device_state(
      {"queue": []},
      FakeParams(),
      object(),
      guard,
      network_guard,
      state_path="state",
      root="root",
      poll_interval=0.01,
    )

  result = asyncio.run(run())

  assert result["wrapped"] is True
  assert cancelled == {"device", "network"}


def test_disabled_worker_uses_one_second_low_duty_poll(tmp_path, monkeypatch):
  device_state = type("DeviceState", (), {"started": False})()

  class FakeSubMaster:
    valid = {"deviceState": True}
    alive = {"deviceState": True}
    updated = {"deviceState": True}
    logMonoTime = {"deviceState": 1}

    def update(self, _timeout):
      return None

    def __getitem__(self, _name):
      return device_state

  sleeps = []

  async def stop_on_sleep(delay):
    sleeps.append(delay)
    raise asyncio.CancelledError

  monkeypatch.setattr(auto_upload, "HAS_PARAMS", True)
  monkeypatch.setattr(auto_upload, "Params", lambda: FakeParams())
  monkeypatch.setattr(auto_upload.messaging, "SubMaster", lambda _services: FakeSubMaster())
  monkeypatch.setattr(auto_upload.messaging, "sub_sock", lambda *_args, **_kwargs: object())
  monkeypatch.setattr(auto_upload.messaging, "drain_sock", lambda _sock: [])
  monkeypatch.setattr(auto_upload.asyncio, "sleep", stop_on_sleep)

  with pytest.raises(asyncio.CancelledError):
    asyncio.run(auto_upload._validation_auto_upload_worker(
      state_path=str(tmp_path / "state.json"),
      root=str(tmp_path),
      poll_interval=0.1,
    ))

  assert sleeps == [1.0]


def test_disallowed_owner_device_cannot_arm_campaign_or_reach_network(tmp_path, monkeypatch):
  params = FakeParams({
    "DongleId": "another-device",
    auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1,
    "IsOffroad": True,
    "IsOnroad": False,
  })
  state = auto_upload._default_state()
  device_state = type("DeviceState", (), {"started": False})()

  class FakeSubMaster:
    valid = {"deviceState": True}
    alive = {"deviceState": True}
    updated = {"deviceState": True}
    logMonoTime = {"deviceState": 1}

    def update(self, _timeout):
      return None

    def __getitem__(self, _name):
      return device_state

  saved = []

  async def stop_on_sleep(_delay):
    raise asyncio.CancelledError

  monkeypatch.setattr(auto_upload, "HAS_PARAMS", True)
  monkeypatch.setattr(auto_upload, "Params", lambda: params)
  monkeypatch.setattr(auto_upload, "read_validation_upload_state", lambda _path: state)
  monkeypatch.setattr(
    auto_upload,
    "write_validation_upload_state",
    lambda candidate, _path: saved.append(json.loads(json.dumps(candidate))) or True,
  )
  monkeypatch.setattr(auto_upload.messaging, "SubMaster", lambda _services: FakeSubMaster())
  monkeypatch.setattr(auto_upload.messaging, "sub_sock", lambda *_args, **_kwargs: object())
  monkeypatch.setattr(auto_upload.messaging, "drain_sock", lambda _sock: [])
  monkeypatch.setattr(
    auto_upload,
    "_upload_first_capture_with_live_device_state",
    lambda *_args, **_kwargs: pytest.fail("disallowed device must not reach upload"),
  )
  monkeypatch.setattr(auto_upload.asyncio, "sleep", stop_on_sleep)

  with pytest.raises(asyncio.CancelledError):
    asyncio.run(auto_upload._validation_auto_upload_worker(
      state_path=str(tmp_path / "state.json"),
      root=str(tmp_path),
      poll_interval=0.1,
    ))

  assert params.get_bool(auto_upload.VALIDATION_AUTO_UPLOAD_PARAM) is False
  assert saved[-1]["status"] == "device_not_allowed"
  assert saved[-1]["campaign"] is None
  assert saved[-1]["queue"] == []


def test_validation_upload_status_endpoint_does_not_expose_capture_details(tmp_path, monkeypatch):
  state = auto_upload._default_state()
  state["status"] = "retry_wait"
  state["last_error"] = "/data/media/0/realdata/private-route/rlog.zst: receiver detail"
  state["campaign"] = auto_upload._new_campaign(now=100)
  state["campaign"]["base_url"] = "https://private-upload.example"
  state["active_route"] = {"route": "00000009--private-location", "events": []}
  state["queue"] = [{
    "id": "secret-capture-id",
    "condition": "standstill_off",
    "route": "00000001--0123456789",
    "segments": ["00000001--0123456789--0"],
    "metadata": {"preciseLocation": "secret-location"},
  }]
  monkeypatch.setattr(auto_upload, "read_validation_upload_state", lambda: state)
  request = type("Request", (), {
    "app": {"params": FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1})},
  })()

  response = asyncio.run(dashcam_routes.api_validation_auto_upload_status(request))
  payload = json.loads(response.text)

  assert payload["enabled"] is True
  assert payload["statusCode"] == "retry_wait"
  assert payload["hasActiveRoute"] is True
  assert payload["pendingCaptures"] == 1
  assert payload["serviceRunning"] is False
  assert "campaign" not in payload
  assert "queue" not in payload
  assert "activeRoute" not in payload
  assert "lastError" not in payload
  serialized = response.text
  assert "private-upload.example" not in serialized
  assert "secret-capture-id" not in serialized
  assert "secret-location" not in serialized
  assert "private-route" not in serialized
  assert "receiver detail" not in serialized


def test_validation_service_supervisor_reports_and_restarts_initialization_failure(tmp_path, monkeypatch):
  calls = 0
  restarted = asyncio.Event()

  async def flaky_worker(**_kwargs):
    nonlocal calls
    calls += 1
    if calls == 1:
      raise RuntimeError("submaster init failed")
    restarted.set()
    await asyncio.Event().wait()

  async def run():
    monkeypatch.setattr(auto_upload, "HAS_PARAMS", True)
    monkeypatch.setattr(auto_upload, "Params", object)
    monkeypatch.setattr(auto_upload, "_validation_auto_upload_worker", flaky_worker)
    state_path = tmp_path / "state.json"
    task = asyncio.create_task(auto_upload.validation_auto_upload_loop(
      state_path=str(state_path),
      root=str(tmp_path),
      restart_delay=0.01,
    ))
    await asyncio.wait_for(restarted.wait(), timeout=1.0)
    state = auto_upload.read_validation_upload_state(str(state_path))
    assert calls == 2
    assert state["status"] == "service_error"
    assert "submaster init failed" in state["last_error"]
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    assert task.cancelled()

  asyncio.run(run())


def test_validation_supervisor_never_rewrites_invalid_ownership_state(monkeypatch):
  attempts = 0
  retried = asyncio.Event()
  params = FakeParams({auto_upload.VALIDATION_AUTO_UPLOAD_PARAM: 1})
  invalid = auto_upload._default_state()
  invalid["status"] = "state_invalid"

  async def fail_worker(**_kwargs):
    nonlocal attempts
    attempts += 1
    if attempts >= 2:
      retried.set()
    raise RuntimeError("initialization failed")

  async def run():
    monkeypatch.setattr(auto_upload, "HAS_PARAMS", True)
    monkeypatch.setattr(auto_upload, "Params", lambda: params)
    monkeypatch.setattr(auto_upload, "_validation_auto_upload_worker", fail_worker)
    monkeypatch.setattr(auto_upload, "read_validation_upload_state", lambda _path: invalid)
    monkeypatch.setattr(
      auto_upload,
      "write_validation_upload_state",
      lambda *_args: pytest.fail("invalid ownership state must never be rewritten"),
    )
    task = asyncio.create_task(auto_upload.validation_auto_upload_loop(restart_delay=0.01))
    await asyncio.wait_for(retried.wait(), timeout=1.0)
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)

  asyncio.run(run())

  assert params.get_bool(auto_upload.VALIDATION_AUTO_UPLOAD_PARAM) is False


def test_validation_setting_is_default_off_and_system_record_scoped():
  root = Path(__file__).resolve().parents[3]
  settings = json.loads((root / "carrot_settings.json").read_text(encoding="utf-8"))
  item = next(value for value in settings["params"] if value["name"] == auto_upload.VALIDATION_AUTO_UPLOAD_PARAM)
  assert (item["min"], item["max"], item["default"]) == (0, 1, 0)
  assert item["control"] == "toggle"
  assert item["risk"] == "high"
  system = next(category for category in settings["menu"] if category["id"] == "SYSTEM")
  record = next(group for group in system["groups"] if group["id"] == "SYS_RECORD")
  basic = next(group for group in record["groups"] if group["id"] == "SYS_RECORD_BASIC")
  assert auto_upload.VALIDATION_AUTO_UPLOAD_PARAM in basic["params"]
