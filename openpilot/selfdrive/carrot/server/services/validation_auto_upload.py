from __future__ import annotations

import asyncio
import copy
import hashlib
import json
import math
import os
import random
import secrets
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from openpilot.cereal import car, log, messaging
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE
from openpilot.system.loggerd.deleter import PRESERVE_ATTR_VALUE, VALIDATION_PRESERVE_ATTR_NAME
from openpilot.system.loggerd.xattr_cache import getxattr_direct, setxattr
from openpilot.selfdrive.carrot.web_upload import (
  DEFAULT_WEB_UPLOAD_URL,
  validation_manifest_sha256,
  validation_receipt_id,
)

from opendbc.car.hyundai.values import CAR, HyundaiFlags

from ..config import CARROT_VALIDATION_UPLOAD_STATE_PATH, DASHCAM_ROOT
from ..features.dashcam import upload, upload_jobs
from ..features.dashcam.paths import route_name, segment_index
from .params import HAS_PARAMS, Params


VALIDATION_AUTO_UPLOAD_PARAM = "CarrotValidationAutoUpload"
STATE_SCHEMA_VERSION = 1
CAMPAIGN_LIFETIME_SECONDS = 7 * 24 * 60 * 60
CLOCK_ROLLBACK_TOLERANCE_SECONDS = 60
MAX_SEGMENTS_PER_CAPTURE = 3
MAX_PENDING_CAPTURES = 5
MAX_VALIDATION_PRESERVE_SEGMENTS = MAX_PENDING_CAPTURES * MAX_SEGMENTS_PER_CAPTURE * 2
MAX_SEGMENT_NAME_BYTES = 255
MAX_ROUTE_NAME_BYTES = 220
MAX_PENDING_BYTES = 750 * 1024 * 1024
MAX_CAPTURES_PER_CONDITION = 2
OFFROAD_STABLE_SECONDS = 10.0
STOP_CAPTURE_SECONDS = 30.5
SHORT_STOP_MIN_SECONDS = 8.0
LANE_CAPTURE_SECONDS = 15.0
LANE_CAPTURE_MIN_SPEED = 10.0
SCC_CLOSE_ACCEL_DWELL_SECONDS = 0.5
SCC_CLOSE_ACCEL_MIN_SPEED = 3.0
SCC_CLOSE_ACCEL_MIN_ACCEL = 0.7
SCC_CLOSE_ACCEL_MIN_CLOSING_SPEED = 0.3
SCC_CLOSE_ACCEL_MAX_DISTANCE = 60.0
SCC_CLOSE_ACCEL_MAX_TIME_GAP = 2.0
SCC_CLOSE_ACCEL_MAX_TTC = 5.0
SCC_CLOSE_ACCEL_COOLDOWN_SECONDS = 30.0
MAX_SAMPLE_GAP_SECONDS = 0.35
PHYSICAL_RES_LATCH_SECONDS = 1.0
CAPTURE_FINALIZE_TIMEOUT_SECONDS = 10 * 60
MAX_OPTIONAL_PENDING_CAPTURES = 2
POLL_INTERVAL = 0.1
ROUTE_REFRESH_INTERVAL = 1.0
FOLLOWING_SEGMENT_CHECK_INTERVAL_SECONDS = 1.0
NETWORK_CHECK_INTERVAL = 5.0
DEVICE_STATE_FRESHNESS_SECONDS = 1.0
# There is no portable socket-to-Wi-Fi bind on comma hardware. Poll the
# hardware network type at 10 Hz while upload work is active and let the
# synchronous request/chunk guards reject a stale cache after 0.5 s. Thus a
# transition can only escape detection for the hardware probe plus this short
# poll interval; probe stalls fail closed when the cached sample expires.
NETWORK_GUARD_POLL_INTERVAL_SECONDS = 0.1
NETWORK_STATE_FRESHNESS_SECONDS = 0.5
NETWORK_GUARD_INITIAL_TIMEOUT_SECONDS = 1.0
RETRY_DELAYS = (30.0, 120.0, 600.0, 3600.0, 6 * 3600.0)

TARGET_CONDITIONS = frozenset({
  "standstill_off",
  "standstill_off_physical_res",
  "standstill_on",
  "lane_offset_0",
  "lane_offset_10",
})
OPTIONAL_CONDITIONS = frozenset({
  "standstill_on_no_request",
  "stock_scc_close_accel",
})
CAPTURE_CONDITIONS = TARGET_CONDITIONS | OPTIONAL_CONDITIONS


def _is_https_upload_url(value: Any) -> bool:
  try:
    parsed = urlsplit(str(value or "").strip())
    return parsed.scheme.lower() == "https" and bool(parsed.hostname) and not parsed.username and not parsed.password
  except ValueError:
    return False


def _configured_validation_upload_base_url(raw_override: Any) -> str:
  """Resolve the immutable receiver, failing closed on a malformed override."""
  raw = str(raw_override or "").strip()
  if not raw:
    return DEFAULT_WEB_UPLOAD_URL.rstrip("/")
  normalized = raw.rstrip("/")
  return normalized if _is_https_upload_url(normalized) else ""


# The validation receiver sees a short-lived device JWT while establishing an
# authenticated upload session. Until comma exposes a purpose-scoped proof,
# automatic collection must only trust the operator-controlled built-in
# receiver (or one immutable deployment-time override), never a Web UI value.
# A non-empty malformed override disables the feature instead of silently
# falling back to a different receiver than the deployer intended.
VALIDATION_UPLOAD_BASE_URL = _configured_validation_upload_base_url(
  os.environ.get("CARROT_VALIDATION_UPLOAD_URL", ""),
)


def _campaign_uses_trusted_receiver(campaign: Any) -> bool:
  return (
    isinstance(campaign, dict)
    and _is_https_upload_url(VALIDATION_UPLOAD_BASE_URL)
    and str(campaign.get("base_url") or "").rstrip("/") == VALIDATION_UPLOAD_BASE_URL
  )


def _safe_int(
  value: Any,
  default: int = 0,
  *,
  low: int | None = None,
  high: int | None = None,
) -> int:
  try:
    number = int(value)
  except (TypeError, ValueError, OverflowError):
    number = default
  if low is not None:
    number = max(low, number)
  if high is not None:
    number = min(high, number)
  return number


def _safe_float(
  value: Any,
  default: float = 0.0,
  *,
  low: float | None = None,
  high: float | None = None,
) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    number = default
  if not math.isfinite(number):
    number = default
  if low is not None:
    number = max(low, number)
  if high is not None:
    number = min(high, number)
  return number


def _sanitize_route_settings(raw: Any) -> dict[str, int]:
  values = raw if isinstance(raw, dict) else {}
  return {
    "Ka4StockSccStandstillRearm": _safe_int(values.get("Ka4StockSccStandstillRearm")),
    "PathOffset": _safe_int(values.get("PathOffset")),
    "AdjustLaneOffset": _safe_int(values.get("AdjustLaneOffset")),
  }


@dataclass(frozen=True)
class ValidationSample:
  now: float
  standstill: bool
  cruise_enabled: bool
  can_valid: bool
  engaged: bool
  lat_active: bool
  v_ego: float
  a_ego: float
  steering_pressed: bool
  brake_pressed: bool
  gas_pressed: bool
  brake_hold_active: bool
  parking_brake: bool
  acc_faulted: bool
  cancel_requested: bool
  physical_res_pressed: bool
  ka4_keepalive_request_count: int
  ka4_keepalive_qualified: bool
  ka4_keepalive_stopped_sec: float
  lane_plan_valid: bool
  use_lane_lines: bool
  static_path_offset: float
  dynamic_lane_offset: float
  radar_valid: bool
  lead_status: bool
  lead_d_rel: float
  lead_v_rel: float


class LiveDeviceStateStoppedGuard:
  """Thread-safe-by-assignment, fail-closed view of live deviceState."""

  def __init__(
    self,
    *,
    freshness_seconds: float = DEVICE_STATE_FRESHNESS_SECONDS,
    monotonic: Callable[[], float] = time.monotonic,
  ) -> None:
    self._freshness_seconds = max(0.0, float(freshness_seconds))
    self._monotonic = monotonic
    self._observation: tuple[float, bool] | None = None
    self._last_log_mono_time: int | None = None

  def observe_submaster(self, sm: Any, *, now: float | None = None) -> None:
    observed_at = self._monotonic() if now is None else float(now)
    try:
      valid = bool(sm.valid["deviceState"])
      alive = bool(sm.alive["deviceState"])
      started = bool(sm["deviceState"].started)
      updated = bool(sm.updated["deviceState"])
      log_mono_time = _safe_int(sm.logMonoTime["deviceState"], low=0)
    except Exception:
      self.invalidate(now=observed_at)
      return
    genuinely_new = updated or (
      log_mono_time > 0 and log_mono_time != self._last_log_mono_time
    )
    if not valid or not alive or started:
      if genuinely_new:
        self._last_log_mono_time = log_mono_time
      self.invalidate(now=observed_at)
      return
    if not genuinely_new:
      # A polling loop seeing the same cereal message must not make it fresh.
      return
    self._last_log_mono_time = log_mono_time
    # Replacing one immutable tuple keeps readers in the hashing thread from
    # observing a partially updated timestamp/value pair.
    self._observation = (observed_at, True)

  def invalidate(self, *, now: float | None = None) -> None:
    self._observation = (self._monotonic() if now is None else float(now), False)

  def allows_upload(self) -> bool:
    observation = self._observation
    if observation is None:
      return False
    observed_at, stopped = observation
    age = self._monotonic() - observed_at
    return stopped and 0.0 <= age <= self._freshness_seconds


class LiveWifiGuard:
  """Fail-closed cache consumed synchronously before requests and chunks."""

  def __init__(
    self,
    *,
    freshness_seconds: float = NETWORK_STATE_FRESHNESS_SECONDS,
    monotonic: Callable[[], float] = time.monotonic,
  ) -> None:
    self._freshness_seconds = max(0.0, float(freshness_seconds))
    self._monotonic = monotonic
    self._observation: tuple[float, bool] | None = None
    # asyncio cancellation cannot stop a get_network_type call already running
    # in a worker thread. Keep that task as a single-flight lease so a later
    # upload attempt waits on it instead of leaking another blocked thread.
    self._hardware_probe: asyncio.Task[Any] | None = None

  def observe_network_type(self, network_type: Any, *, now: float | None = None) -> None:
    observed_at = self._monotonic() if now is None else float(now)
    self._observation = (observed_at, network_type == log.DeviceState.NetworkType.wifi)

  def invalidate(self, *, now: float | None = None) -> None:
    self._observation = (self._monotonic() if now is None else float(now), False)

  def allows_upload(self) -> bool:
    observation = self._observation
    if observation is None:
      return False
    observed_at, wifi = observation
    age = self._monotonic() - observed_at
    return wifi and 0.0 <= age <= self._freshness_seconds

  async def refresh_from_hardware(self) -> None:
    probe = self._hardware_probe
    if probe is None:
      def probe_hardware() -> tuple[float, Any]:
        network_type = HARDWARE.get_network_type()
        return self._monotonic(), network_type

      probe = asyncio.create_task(asyncio.to_thread(probe_hardware))
      self._hardware_probe = probe

      def finish_probe(finished: asyncio.Task[Any]) -> None:
        # If the watcher was canceled while to_thread was running there may be
        # no later awaiter to retrieve an exception. Consume it here, and
        # release only this completed lease so the next attempt probes fresh.
        if not finished.cancelled():
          finished.exception()
        if self._hardware_probe is finished:
          self._hardware_probe = None

      probe.add_done_callback(finish_probe)
    try:
      observed_at, network_type = await asyncio.shield(probe)
      self.observe_network_type(network_type, now=observed_at)
    except asyncio.CancelledError:
      # shield leaves the in-flight thread represented by `probe`; retaining
      # it prevents the next wrapper invocation from starting another one.
      raise
    except Exception:
      self.invalidate()
    finally:
      if probe.done() and self._hardware_probe is probe:
        self._hardware_probe = None


async def _refresh_live_device_state(
  sm: Any,
  guard: LiveDeviceStateStoppedGuard,
  *,
  poll_interval: float = POLL_INTERVAL,
) -> None:
  """Continuously refresh the non-Params power signal during hash/upload."""
  while True:
    try:
      sm.update(0)
      guard.observe_submaster(sm)
    except asyncio.CancelledError:
      raise
    except Exception:
      guard.invalidate()
    await asyncio.sleep(max(0.01, poll_interval))


async def _refresh_live_wifi(
  guard: LiveWifiGuard,
  *,
  poll_interval: float = NETWORK_GUARD_POLL_INTERVAL_SECONDS,
  initial_observation: asyncio.Event | None = None,
) -> None:
  """Continuously refresh the network cache without blocking upload chunks."""
  first_observation = True
  while True:
    try:
      await guard.refresh_from_hardware()
    except asyncio.CancelledError:
      raise
    except Exception:
      guard.invalidate()
    finally:
      if first_observation and initial_observation is not None:
        initial_observation.set()
      first_observation = False
    await asyncio.sleep(max(0.01, poll_interval))


class ValidationEventDetector:
  """Pure route-local event detector; filesystem and network work live outside it."""

  def __init__(self) -> None:
    self.stop_started_at: float | None = None
    self.stop_physical_res = False
    self.stop_emitted = False
    self.last_keepalive_request_count: int | None = None
    self.lane_started_at: float | None = None
    self.lane_condition: str | None = None
    self.lane_emitted = False
    self.close_accel_started_at: float | None = None
    self.close_accel_cooldown_until = 0.0
    self.last_sample_at: float | None = None

  def reset(self) -> None:
    self.__init__()

  def invalidate_sample(self) -> None:
    """Break continuity when a required cereal service is unavailable."""
    self.stop_started_at = None
    self.stop_physical_res = False
    self.stop_emitted = False
    self.last_keepalive_request_count = None
    self.lane_started_at = None
    self.lane_condition = None
    self.lane_emitted = False
    self.close_accel_started_at = None
    self.last_sample_at = None

  def nack(self, condition: str) -> None:
    """Re-arm a detected event whose retention record was not durably saved."""
    if condition.startswith("standstill_"):
      self.stop_emitted = False
    elif condition.startswith("lane_offset_"):
      self.lane_emitted = False
    elif condition == "stock_scc_close_accel":
      self.close_accel_started_at = self.last_sample_at
      self.close_accel_cooldown_until = 0.0

  @staticmethod
  def _driver_interlock(sample: ValidationSample) -> bool:
    return (
      sample.brake_pressed
      or sample.gas_pressed
      or sample.brake_hold_active
      or sample.parking_brake
      or sample.acc_faulted
      or sample.cancel_requested
    )

  @classmethod
  def _standstill_active(cls, sample: ValidationSample) -> bool:
    return (
      sample.standstill
      and sample.cruise_enabled
      and sample.can_valid
      and sample.engaged
      and abs(sample.v_ego) <= 0.05
      and not cls._driver_interlock(sample)
    )

  @staticmethod
  def _standstill_condition(route_settings: dict[str, int], physical_res: bool) -> str:
    if int(route_settings.get("Ka4StockSccStandstillRearm", 0)) > 0:
      return "standstill_on_no_request"
    return "standstill_off_physical_res" if physical_res else "standstill_off"

  @staticmethod
  def _lane_condition_for_settings(route_settings: dict[str, int]) -> str | None:
    if int(route_settings.get("AdjustLaneOffset", 0)) != 0:
      return None
    path_offset = int(route_settings.get("PathOffset", 0))
    if path_offset == 0:
      return "lane_offset_0"
    if path_offset == 10:
      return "lane_offset_10"
    return None

  def update(self, sample: ValidationSample, route_settings: dict[str, int]) -> list[dict[str, Any]]:
    if (
      self.last_sample_at is not None
      and (sample.now <= self.last_sample_at or sample.now - self.last_sample_at > MAX_SAMPLE_GAP_SECONDS)
    ):
      self.invalidate_sample()
    self.last_sample_at = sample.now
    events: list[dict[str, Any]] = []
    standstill_active = self._standstill_active(sample)
    keepalive_request_delta = (
      (sample.ka4_keepalive_request_count - self.last_keepalive_request_count) & 0xFFFFFFFF
      if self.last_keepalive_request_count is not None else 0
    )
    keepalive_requested = (
      0 < keepalive_request_delta <= 10
      and sample.ka4_keepalive_qualified
      and sample.ka4_keepalive_stopped_sec > 0.0
    )
    self.last_keepalive_request_count = sample.ka4_keepalive_request_count
    if self.stop_started_at is not None:
      # RES is a one-frame edge and can share the 100 ms service sample in
      # which the lead starts moving. Latch it before evaluating stop end.
      self.stop_physical_res |= sample.physical_res_pressed
    if standstill_active:
      if self.stop_started_at is None:
        self.stop_started_at = sample.now
        self.stop_physical_res = False
        self.stop_emitted = False
      self.stop_physical_res |= sample.physical_res_pressed
      duration = max(0.0, sample.now - self.stop_started_at)
      rearm_enabled = int(route_settings.get("Ka4StockSccStandstillRearm", 0)) > 0
      if rearm_enabled and keepalive_requested and not self.stop_emitted:
        events.append({
          "condition": "standstill_on",
          "duration": round(sample.ka4_keepalive_stopped_sec, 3),
          "trigger": "keepalive_requested",
          "keepaliveRequestDelta": keepalive_request_delta,
          "qualified": True,
        })
        self.stop_emitted = True
      elif duration >= STOP_CAPTURE_SECONDS and not self.stop_emitted:
        events.append({
          "condition": self._standstill_condition(route_settings, self.stop_physical_res),
          "duration": round(duration, 3),
          "trigger": "duration_no_keepalive_request" if rearm_enabled else "duration",
          "qualified": bool(sample.ka4_keepalive_qualified),
          "controllerStoppedSec": round(max(0.0, sample.ka4_keepalive_stopped_sec), 3),
        })
        self.stop_emitted = True
    elif self.stop_started_at is not None:
      duration = max(0.0, sample.now - self.stop_started_at)
      clean_stop_end = (
        sample.can_valid
        and not sample.standstill
        and sample.v_ego > 0.05
        and not self._driver_interlock(sample)
      )
      if duration >= SHORT_STOP_MIN_SECONDS and clean_stop_end and not self.stop_emitted:
        events.append({
          "condition": self._standstill_condition(route_settings, self.stop_physical_res),
          "duration": round(duration, 3),
          "trigger": (
            "stop_ended_before_keepalive_request" if int(route_settings.get("Ka4StockSccStandstillRearm", 0)) > 0
            else "stop_ended_early"
          ),
          "qualified": bool(sample.ka4_keepalive_qualified),
          "controllerStoppedSec": round(max(0.0, sample.ka4_keepalive_stopped_sec), 3),
        })
      self.stop_started_at = None
      self.stop_physical_res = False
      self.stop_emitted = False

    lane_condition = self._lane_condition_for_settings(route_settings)
    expected_offset = 0.1 if lane_condition == "lane_offset_10" else 0.0
    lane_active = (
      lane_condition is not None
      and sample.engaged
      and sample.lat_active
      and sample.can_valid
      and sample.v_ego >= LANE_CAPTURE_MIN_SPEED
      and not sample.steering_pressed
      and sample.lane_plan_valid
      and sample.use_lane_lines
      and math.isfinite(sample.static_path_offset)
      and abs(sample.static_path_offset - expected_offset) <= 0.005
      and math.isfinite(sample.dynamic_lane_offset)
      and abs(sample.dynamic_lane_offset) <= 0.02
    )
    if lane_active:
      if self.lane_started_at is None or self.lane_condition != lane_condition:
        self.lane_started_at = sample.now
        self.lane_condition = lane_condition
        self.lane_emitted = False
      duration = max(0.0, sample.now - self.lane_started_at)
      if duration >= LANE_CAPTURE_SECONDS and not self.lane_emitted:
        events.append({
          "condition": lane_condition,
          "duration": round(duration, 3),
          "trigger": "stable_lane_control",
        })
        self.lane_emitted = True
    else:
      self.lane_started_at = None
      self.lane_condition = None
      self.lane_emitted = False

    lead_distance = sample.lead_d_rel
    closing_speed = max(0.0, -sample.lead_v_rel)
    time_gap = lead_distance / max(sample.v_ego, 0.1)
    ttc = lead_distance / max(closing_speed, 0.1)
    close_accel_active = (
      sample.cruise_enabled
      and sample.can_valid
      and sample.engaged
      and not sample.standstill
      and sample.v_ego >= SCC_CLOSE_ACCEL_MIN_SPEED
      and not self._driver_interlock(sample)
      and sample.radar_valid
      and sample.lead_status
      and math.isfinite(sample.a_ego)
      and sample.a_ego >= SCC_CLOSE_ACCEL_MIN_ACCEL
      and math.isfinite(lead_distance)
      and 0.0 < lead_distance <= SCC_CLOSE_ACCEL_MAX_DISTANCE
      and math.isfinite(sample.lead_v_rel)
      and sample.lead_v_rel <= -SCC_CLOSE_ACCEL_MIN_CLOSING_SPEED
      and (
        time_gap <= SCC_CLOSE_ACCEL_MAX_TIME_GAP
        or ttc <= SCC_CLOSE_ACCEL_MAX_TTC
      )
    )
    if close_accel_active:
      if self.close_accel_started_at is None:
        self.close_accel_started_at = sample.now
      duration = max(0.0, sample.now - self.close_accel_started_at)
      if duration >= SCC_CLOSE_ACCEL_DWELL_SECONDS and sample.now >= self.close_accel_cooldown_until:
        events.append({
          "condition": "stock_scc_close_accel",
          "duration": round(duration, 3),
          "trigger": "accelerating_while_closing",
          "vEgo": round(sample.v_ego, 3),
          "aEgo": round(sample.a_ego, 3),
          "leadDRel": round(lead_distance, 3),
          "leadVRel": round(sample.lead_v_rel, 3),
          "timeGap": round(time_gap, 3),
          "ttc": round(ttc, 3),
        })
        self.close_accel_cooldown_until = sample.now + SCC_CLOSE_ACCEL_COOLDOWN_SECONDS
        self.close_accel_started_at = None
    else:
      self.close_accel_started_at = None

    return events


def _default_state() -> dict[str, Any]:
  return {
    "schema_version": STATE_SCHEMA_VERSION,
    "status": "disabled",
    "campaign": None,
    "active_route": None,
    "queue": [],
    "cleanup_preserve": [],
    "completed": [],
    "last_error": "",
    "last_uploaded_at": 0,
    "updated_at": int(time.time()),  # noqa: TID251 - durable state uses wall-clock epoch
  }


def _safe_segment_name(value: Any) -> str:
  name = str(value or "").strip()
  if (
    not name
    or len(name.encode("utf-8", errors="surrogatepass")) > MAX_SEGMENT_NAME_BYTES
    or any("\ud800" <= character <= "\udfff" for character in name)
    or name in {".", ".."}
    or "/" in name
    or "\\" in name
    or "\0" in name
  ):
    return ""
  parts = name.rsplit("--", 1)
  return (
    name
    if (
      len(parts) == 2
      and parts[0]
      and parts[1].isdigit()
      and str(int(parts[1])) == parts[1]
    )
    else ""
  )


def _safe_route_name(value: Any) -> str:
  name = str(value or "").strip()
  if (
    not name
    or len(name.encode("utf-8", errors="surrogatepass")) > MAX_ROUTE_NAME_BYTES
    or any("\ud800" <= character <= "\udfff" for character in name)
    or name in {".", ".."}
    or "/" in name
    or "\\" in name
    or "\0" in name
    or "--" not in name
  ):
    return ""
  return name


def _bounded_float(value: Any, low: float, high: float) -> float:
  return _safe_float(value, low=low, high=high)


def _mark_state_invalid(state: dict[str, Any], error: str) -> None:
  state["status"] = "state_invalid"
  state["last_error"] = error


def _sanitize_owned_preserve(raw: Any, *, required: bool = False) -> tuple[list[str], bool]:
  if raw is None:
    return [], not required
  if not isinstance(raw, list):
    return [], False
  sanitized = [_safe_segment_name(value) for value in raw]
  if any(not segment for segment in sanitized):
    return list(dict.fromkeys(filter(None, sanitized)))[:MAX_SEGMENTS_PER_CAPTURE], False
  unique = list(dict.fromkeys(sanitized))
  valid = (
    len(raw) <= MAX_SEGMENTS_PER_CAPTURE
    and len(unique) == len(raw)
    and (bool(unique) or not required)
  )
  return unique[:MAX_SEGMENTS_PER_CAPTURE], valid


def _bounded_text(value: Any, limit: int) -> str:
  return value[:limit] if isinstance(value, str) else ""


def _sanitize_identity(raw: Any) -> dict[str, Any]:
  values = raw if isinstance(raw, dict) else {}
  identity: dict[str, Any] = {
    "branch": _bounded_text(values.get("branch"), 128),
    "commit": _bounded_text(values.get("commit"), 64),
    "dirty": bool(values.get("dirty")) if isinstance(values.get("dirty"), bool) else False,
  }
  topology_raw = values.get("topology")
  if isinstance(topology_raw, dict):
    configs_raw = topology_raw.get("safetyConfigs")
    configs = []
    if isinstance(configs_raw, list):
      for config in configs_raw[:8]:
        if not isinstance(config, dict):
          continue
        configs.append({
          "model": _bounded_text(config.get("model"), 64),
          "param": _safe_int(config.get("param"), low=0, high=0xFFFFFFFF),
        })
    identity["topology"] = {
      "carFingerprint": _bounded_text(topology_raw.get("carFingerprint"), 128),
      "pcmCruise": bool(topology_raw.get("pcmCruise")) if isinstance(topology_raw.get("pcmCruise"), bool) else False,
      "openpilotLongitudinalControl": (
        bool(topology_raw.get("openpilotLongitudinalControl"))
        if isinstance(topology_raw.get("openpilotLongitudinalControl"), bool) else False
      ),
      "flags": _safe_int(topology_raw.get("flags"), low=0, high=0xFFFFFFFFFFFFFFFF),
      "alternativeExperience": _safe_int(
        topology_raw.get("alternativeExperience"), low=0, high=0xFFFFFFFF,
      ),
      "safetyConfigs": configs,
      "gatePassed": bool(topology_raw.get("gatePassed")) if isinstance(topology_raw.get("gatePassed"), bool) else False,
    }
  return identity


def _optional_float(value: Any, *, low: float, high: float) -> float | None:
  return None if value is None else _safe_float(value, low=low, high=high)


def _optional_int(value: Any, *, low: int, high: int) -> int | None:
  return None if value is None else _safe_int(value, low=low, high=high)


def _sanitize_capture_metadata(
  raw: Any,
  *,
  campaign_id: str,
  capture_id: str,
  condition: str,
  route: str,
  segments: list[str],
) -> dict[str, Any]:
  values = raw if isinstance(raw, dict) else {}
  qualified = values.get("qualified") if isinstance(values.get("qualified"), bool) else None
  return {
    "schemaVersion": 1,
    "campaignId": campaign_id,
    "captureId": capture_id,
    "condition": condition,
    "route": route,
    "segments": list(segments),
    "anchorSegment": _safe_int(values.get("anchorSegment"), low=0),
    "duration": _optional_float(values.get("duration"), low=0.0, high=CAMPAIGN_LIFETIME_SECONDS),
    "trigger": _bounded_text(values.get("trigger"), 64),
    "keepaliveRequestDelta": _optional_int(values.get("keepaliveRequestDelta"), low=0, high=10),
    "qualified": qualified,
    "controllerStoppedSec": _optional_float(
      values.get("controllerStoppedSec"), low=0.0, high=CAMPAIGN_LIFETIME_SECONDS,
    ),
    "vEgo": _optional_float(values.get("vEgo"), low=0.0, high=100.0),
    "aEgo": _optional_float(values.get("aEgo"), low=-20.0, high=20.0),
    "leadDRel": _optional_float(values.get("leadDRel"), low=0.0, high=500.0),
    "leadVRel": _optional_float(values.get("leadVRel"), low=-100.0, high=100.0),
    "timeGap": _optional_float(values.get("timeGap"), low=0.0, high=100.0),
    "ttc": _optional_float(values.get("ttc"), low=0.0, high=100.0),
    "detectedAt": _safe_int(values.get("detectedAt"), low=0),
    "settingsEpoch": _safe_int(values.get("settingsEpoch"), low=0),
    "routeSettings": _sanitize_route_settings(values.get("routeSettings")),
    "git": _sanitize_identity(values.get("git")),
  }


def _safe_capture_id(value: Any) -> str:
  capture_id = _bounded_text(value, 32).lower()
  return capture_id if len(capture_id) == 32 and all(character in "0123456789abcdef" for character in capture_id) else ""


def _sanitize_state(raw: Any) -> dict[str, Any]:
  state = _default_state()
  if not isinstance(raw, dict) or raw.get("schema_version") != STATE_SCHEMA_VERSION:
    state["status"] = "state_invalid"
    state["last_error"] = "automatic validation upload state schema is not supported"
    return state
  state["status"] = str(raw.get("status") or "idle")[:64]
  state["last_error"] = str(raw.get("last_error") or "")[-1000:]
  state["last_uploaded_at"] = _safe_int(raw.get("last_uploaded_at"), low=0)

  campaign = raw.get("campaign")
  if isinstance(campaign, dict):
    base_url = str(campaign.get("base_url") or "").strip()
    campaign_id = str(campaign.get("id") or "")
    started_at = _safe_int(campaign.get("started_at"), low=0)
    expires_at = _safe_int(campaign.get("expires_at"), low=0)
    if (
      len(campaign_id) == 24
      and all(character in "0123456789abcdef" for character in campaign_id)
      and _campaign_uses_trusted_receiver({"base_url": base_url})
      and started_at > 0
      and started_at < expires_at <= started_at + CAMPAIGN_LIFETIME_SECONDS
    ):
      state["campaign"] = {
        "id": campaign_id,
        "started_at": started_at,
        "expires_at": expires_at,
        "base_url": base_url.rstrip("/")[:500],
      }
    else:
      _mark_state_invalid(state, "automatic validation upload campaign is not valid")
  elif campaign is not None:
    _mark_state_invalid(state, "automatic validation upload campaign is not valid")

  active = raw.get("active_route")
  if isinstance(active, dict) and _safe_route_name(active.get("route")):
    settings = _sanitize_route_settings(active.get("settings"))
    events = []
    raw_events = active.get("events")
    if not isinstance(raw_events, list):
      _mark_state_invalid(state, "automatic validation active events are not valid")
      raw_events = []
    if len(raw_events) > MAX_PENDING_CAPTURES:
      _mark_state_invalid(state, "automatic validation active event limit was exceeded")
    for event in raw_events[:MAX_PENDING_CAPTURES]:
      if not isinstance(event, dict) or event.get("condition") not in CAPTURE_CONDITIONS:
        _mark_state_invalid(state, "automatic validation active event entry is not valid")
        continue
      owned_preserve, owned_valid = _sanitize_owned_preserve(event.get("owned_preserve"), required=True)
      if not owned_valid:
        _mark_state_invalid(state, "automatic validation active event ownership is not valid")
      raw_anchor = event.get("anchor_segment")
      anchor = raw_anchor if isinstance(raw_anchor, int) and not isinstance(raw_anchor, bool) else -1
      if anchor < 0:
        _mark_state_invalid(state, "automatic validation active event anchor is not valid")
        anchor = 0
      allowed_owned = {f"{_safe_route_name(active.get('route'))}--{anchor}"}
      if event["condition"] == "stock_scc_close_accel":
        allowed_owned.add(f"{_safe_route_name(active.get('route'))}--{anchor + 1}")
      if (
        f"{_safe_route_name(active.get('route'))}--{anchor}" not in owned_preserve
        or any(segment not in allowed_owned for segment in owned_preserve)
      ):
        _mark_state_invalid(state, "automatic validation active event ownership does not match its anchor")
      event_settings = _sanitize_route_settings(event.get("settings", settings))
      events.append({
        "condition": event["condition"],
        "anchor_segment": anchor,
        "owned_preserve": owned_preserve,
        "duration": _safe_float(event.get("duration"), low=0.0),
        "trigger": str(event.get("trigger") or "")[:64],
        "keepaliveRequestDelta": _safe_int(event.get("keepaliveRequestDelta"), low=0, high=10),
        "qualified": bool(event.get("qualified")),
        "controllerStoppedSec": _safe_float(event.get("controllerStoppedSec"), low=0.0),
        "vEgo": _bounded_float(event.get("vEgo"), 0.0, 100.0),
        "aEgo": _bounded_float(event.get("aEgo"), -20.0, 20.0),
        "leadDRel": _bounded_float(event.get("leadDRel"), 0.0, 500.0),
        "leadVRel": _bounded_float(event.get("leadVRel"), -100.0, 100.0),
        "timeGap": _bounded_float(event.get("timeGap"), 0.0, 100.0),
        "ttc": _bounded_float(event.get("ttc"), 0.0, 100.0),
        "detected_at": _safe_int(event.get("detected_at"), low=0),
        "settings_epoch": _safe_int(event.get("settings_epoch"), low=0),
        "settings": event_settings,
      })
    state["active_route"] = {
      "route": _safe_route_name(active.get("route")),
      "started_at": _safe_int(active.get("started_at"), low=0),
      "finalize_wait_started_at": _safe_int(active.get("finalize_wait_started_at"), low=0),
      "settings_epoch": _safe_int(active.get("settings_epoch"), low=0),
      "settings": settings,
      "identity": _sanitize_identity(active.get("identity")),
      "events": events,
    }
  elif active is not None:
    _mark_state_invalid(state, "automatic validation active route is not valid")

  queue = []
  raw_queue = raw.get("queue")
  if not isinstance(raw_queue, list):
    _mark_state_invalid(state, "automatic validation queue is not valid")
    raw_queue = []
  if len(raw_queue) > MAX_PENDING_CAPTURES:
    _mark_state_invalid(state, "automatic validation queue limit was exceeded")
  pending_bytes = 0
  campaign_id = str((state.get("campaign") or {}).get("id") or "")
  for item in raw_queue[:MAX_PENDING_CAPTURES]:
    if not isinstance(item, dict) or item.get("condition") not in CAPTURE_CONDITIONS:
      _mark_state_invalid(state, "automatic validation queue entry is not valid")
      continue
    raw_owned = item.get("owned_preserve")
    owned_preserve, owned_valid = _sanitize_owned_preserve(raw_owned, required=True)
    if not owned_valid:
      _mark_state_invalid(state, "automatic validation queue ownership is not valid")
    route = _safe_route_name(item.get("route"))
    raw_segments = item.get("segments")
    item_segments = raw_segments if isinstance(raw_segments, list) else []
    segments = list(dict.fromkeys(filter(None, (_safe_segment_name(v) for v in item_segments))))
    segment_indices = [segment_index(segment) for segment in segments]
    segments_valid = (
      isinstance(raw_segments, list)
      and len(segments) == len(item_segments)
      and len(segments) <= MAX_SEGMENTS_PER_CAPTURE
      and bool(route)
      and bool(segments)
      and all(route_name(segment) == route for segment in segments)
      and segment_indices == sorted(segment_indices)
      and all(
        right == left + 1
        for left, right in zip(segment_indices, segment_indices[1:], strict=False)
      )
    )
    if not segments_valid:
      _mark_state_invalid(state, "automatic validation queue segments are not valid")
      continue
    if any(segment not in segments for segment in owned_preserve) or (
      segments and segments[-1] not in owned_preserve
    ):
      _mark_state_invalid(state, "automatic validation queue ownership does not match its segments")
    capture_id = _safe_capture_id(item.get("id"))
    if not capture_id:
      _mark_state_invalid(state, "automatic validation capture id is not valid")
    raw_bytes = item.get("bytes")
    capture_bytes = raw_bytes if isinstance(raw_bytes, int) and not isinstance(raw_bytes, bool) else 0
    if capture_bytes <= 0 or capture_bytes > MAX_PENDING_BYTES:
      _mark_state_invalid(state, "automatic validation capture byte accounting is not valid")
      capture_bytes = max(0, min(MAX_PENDING_BYTES, capture_bytes))
    pending_bytes += capture_bytes
    raw_files = item.get("files", [])
    if isinstance(raw_files, list) and len(raw_files) > MAX_SEGMENTS_PER_CAPTURE:
      _mark_state_invalid(state, "automatic validation capture file manifest limit was exceeded")
    files = _sanitize_capture_files(
      raw_files[:MAX_SEGMENTS_PER_CAPTURE] if isinstance(raw_files, list) else raw_files,
      segments,
    )
    if not isinstance(raw_files, list) or (raw_files and not files):
      _mark_state_invalid(state, "automatic validation capture file manifest is not valid")
    if files and sum(file["size"] for file in files) != capture_bytes:
      _mark_state_invalid(state, "automatic validation capture byte accounting does not match its manifest")
    queue.append({
      "id": capture_id,
      "condition": item["condition"],
      "route": route,
      "segments": segments,
      "owned_preserve": [segment for segment in owned_preserve if segment in segments],
      "metadata": _sanitize_capture_metadata(
        item.get("metadata"),
        campaign_id=campaign_id,
        capture_id=capture_id,
        condition=item["condition"],
        route=route,
        segments=segments,
      ),
      "files": files,
      "attempts": _safe_int(item.get("attempts"), low=0),
      "next_retry_at": _safe_int(item.get("next_retry_at"), low=0),
      "created_at": _safe_int(item.get("created_at"), low=0),
      "bytes": capture_bytes,
    })
  if pending_bytes > MAX_PENDING_BYTES:
    _mark_state_invalid(state, "automatic validation pending byte limit was exceeded")
  state["queue"] = queue[:MAX_PENDING_CAPTURES]
  raw_cleanup = raw.get("cleanup_preserve")
  if not isinstance(raw_cleanup, list):
    _mark_state_invalid(state, "automatic validation cleanup journal is not valid")
    raw_cleanup = []
  cleanup_limit = MAX_VALIDATION_PRESERVE_SEGMENTS
  cleanup = [_safe_segment_name(value) for value in raw_cleanup[:cleanup_limit]]
  if any(not segment for segment in cleanup):
    _mark_state_invalid(state, "automatic validation cleanup ownership is not valid")
  cleanup = list(dict.fromkeys(filter(None, cleanup)))
  if (
    len(raw_cleanup) > cleanup_limit
    or len(cleanup) > cleanup_limit
    or len(cleanup) != len(raw_cleanup)
  ):
    _mark_state_invalid(state, "automatic validation cleanup ownership limit was exceeded")
  state["cleanup_preserve"] = cleanup[:cleanup_limit]

  live_owned: list[str] = []
  for capture in state["queue"]:
    live_owned.extend(capture.get("owned_preserve") or [])
  if state["active_route"] is not None:
    for event in state["active_route"].get("events") or []:
      live_owned.extend(event.get("owned_preserve") or [])
  live_owned_set = set(live_owned)
  cleanup_set = set(state["cleanup_preserve"])
  if live_owned_set & cleanup_set:
    _mark_state_invalid(state, "automatic validation cleanup overlaps live ownership")
  if len(live_owned_set | cleanup_set) > MAX_VALIDATION_PRESERVE_SEGMENTS:
    _mark_state_invalid(state, "automatic validation marker ownership limit was exceeded")

  completed = []
  raw_completed = raw.get("completed")
  if not isinstance(raw_completed, list):
    _mark_state_invalid(state, "automatic validation completion list is not valid")
    raw_completed = []
  for item in raw_completed[-20:]:
    if not isinstance(item, dict) or item.get("condition") not in CAPTURE_CONDITIONS:
      continue
    completed.append({
      "id": str(item.get("id") or "")[:64],
      "condition": item["condition"],
      "route": _safe_route_name(item.get("route")),
      "uploaded_at": _safe_int(item.get("uploaded_at"), low=0),
      "receipt_id": str(item.get("receipt_id") or "")[:128],
      "manifest_sha256": str(item.get("manifest_sha256") or "")[:64],
    })
  state["completed"] = completed[-20:]
  if state["campaign"] is None and (state["active_route"] is not None or state["queue"]):
    _mark_state_invalid(state, "automatic validation captures have no consent-bound campaign")
  active_event_count = len(state["active_route"].get("events") or []) if state["active_route"] else 0
  if len(raw_queue) + active_event_count > MAX_PENDING_CAPTURES:
    _mark_state_invalid(state, "automatic validation retained capture limit was exceeded")
  return state


def read_validation_upload_state(path: str = CARROT_VALIDATION_UPLOAD_STATE_PATH) -> dict[str, Any]:
  try:
    with open(path, encoding="utf-8") as f:
      return _sanitize_state(json.load(f))
  except Exception:
    state = _default_state()
    if os.path.exists(path):
      state["status"] = "state_invalid"
      state["last_error"] = "automatic validation upload state could not be read"
    return state


def write_validation_upload_state(state: dict[str, Any], path: str = CARROT_VALIDATION_UPLOAD_STATE_PATH) -> bool:
  clean = _sanitize_state({**state, "schema_version": STATE_SCHEMA_VERSION})
  # Never replace a proven durable ownership journal with a sanitized/truncated
  # invalid candidate. Callers must treat False as a failed transaction and
  # keep every existing validation marker intact.
  if clean.get("status") == "state_invalid":
    return False
  clean["updated_at"] = int(time.time())  # noqa: TID251 - public status uses wall-clock epoch
  tmp_path = f"{path}.tmp"
  try:
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(tmp_path, "w", encoding="utf-8") as f:
      os.chmod(tmp_path, 0o600)
      json.dump(clean, f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_path, path)
    directory_fd = os.open(directory, os.O_RDONLY)
    try:
      os.fsync(directory_fd)
    finally:
      os.close(directory_fd)
    return True
  except Exception:
    try:
      os.unlink(tmp_path)
    except OSError:
      pass
    return False


def _param_int(params: Any, key: str) -> int:
  try:
    return int(params.get_int(key))
  except Exception:
    try:
      value = params.get(key)
      if isinstance(value, bytes):
        value = value.decode("utf-8", errors="ignore")
      return int(value or 0)
    except Exception:
      return 0


def _param_text(params: Any, key: str) -> str:
  try:
    value = params.get(key)
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    return str(value or "").strip()
  except Exception:
    return ""


def _disable_consent(params: Any) -> bool:
  try:
    params.put_bool(VALIDATION_AUTO_UPLOAD_PARAM, False)
    return not bool(params.get_bool(VALIDATION_AUTO_UPLOAD_PARAM))
  except Exception:
    return False


def _disable_invalid_state_consent(state: dict[str, Any], params: Any) -> bool:
  return state.get("status") == "state_invalid" and _disable_consent(params)


def _route_settings(params: Any) -> dict[str, int]:
  return {
    "Ka4StockSccStandstillRearm": _param_int(params, "Ka4StockSccStandstillRearm"),
    "PathOffset": _param_int(params, "PathOffset"),
    "AdjustLaneOffset": _param_int(params, "AdjustLaneOffset"),
  }


def _update_active_route_settings(
  state: dict[str, Any],
  settings: dict[str, int],
) -> tuple[dict[str, Any], bool]:
  active = state.get("active_route")
  if not isinstance(active, dict):
    return state, False
  previous = _sanitize_route_settings(active.get("settings"))
  current = _sanitize_route_settings(settings)
  if previous == current:
    return state, False
  candidate = copy.deepcopy(state)
  candidate_active = candidate["active_route"]
  previous_epoch = _safe_int(candidate_active.get("settings_epoch"), low=0)
  for event in candidate_active.get("events") or []:
    # Old state versions did not persist settings per event. Backfill before
    # changing the route snapshot so queued metadata remains attributable.
    if not isinstance(event.get("settings"), dict):
      event["settings"] = previous
      event["settings_epoch"] = previous_epoch
  candidate_active["settings"] = current
  candidate_active["settings_epoch"] = previous_epoch + 1
  candidate["active_route"] = candidate_active
  return candidate, True


def _git_identity(params: Any) -> dict[str, Any]:
  branch = upload.git_text(["branch", "--show-current"], _param_text(params, "GitBranch") or "unknown")
  return {
    "branch": branch,
    "commit": upload.git_text(["rev-parse", "HEAD"], _param_text(params, "GitCommit") or "unknown"),
    "dirty": bool(upload.git_text(["status", "--porcelain"], "")),
  }


def ka4_stock_scc_gate(params: Any) -> tuple[bool, dict[str, Any]]:
  raw = None
  for key in ("CarParams", "CarParamsPersistent"):
    try:
      raw = params.get(key)
    except Exception:
      raw = None
    if raw:
      break
  if not raw:
    return False, {"reason": "missing_car_params"}
  try:
    cp = messaging.log_from_bytes(raw, car.CarParams)
    flags = int(cp.flags)
    gate = (
      str(cp.carFingerprint) == str(CAR.KIA_CARNIVAL_4TH_GEN)
      and bool(cp.pcmCruise)
      and not bool(cp.openpilotLongitudinalControl)
      and bool(flags & int(HyundaiFlags.CANFD))
      and bool(flags & int(HyundaiFlags.RADAR_SCC))
      and not bool(flags & int(HyundaiFlags.CAMERA_SCC))
    )
    return gate, {
      "carFingerprint": str(cp.carFingerprint),
      "pcmCruise": bool(cp.pcmCruise),
      "openpilotLongitudinalControl": bool(cp.openpilotLongitudinalControl),
      "flags": flags,
      "alternativeExperience": int(cp.alternativeExperience),
      "safetyConfigs": [
        {"model": str(config.safetyModel), "param": int(config.safetyParam)}
        for config in cp.safetyConfigs
      ],
      "gatePassed": gate,
    }
  except Exception as exc:
    return False, {"reason": "invalid_car_params", "error": str(exc)[:200]}


def _segment_dirs(root: str, route: str) -> list[tuple[int, str, str]]:
  route = _safe_route_name(route)
  if not route or not os.path.isdir(root):
    return []
  prefix = f"{route}--"
  found = []
  try:
    with os.scandir(root) as entries:
      for entry in entries:
        name = _safe_segment_name(entry.name)
        if not name.startswith(prefix) or not entry.is_dir(follow_symlinks=False):
          continue
        found.append((segment_index(name), name, entry.path))
  except OSError:
    return []
  return sorted(found)


def latest_route_segment(root: str, route: str) -> int | None:
  segments = _segment_dirs(root, route)
  return segments[-1][0] if segments else None


def capture_segment_names(root: str, route: str, anchor_segment: int) -> list[str]:
  segments = _segment_dirs(root, route)
  by_index = {index: name for index, name, _path in segments}
  anchor = int(anchor_segment)
  # The deleter protects an anchor and its two immediate predecessors. Do not
  # include an older segment across a recording gap: the anchor xattr would not
  # protect it under storage pressure.
  return [
    by_index[index]
    for index in range(max(0, anchor - MAX_SEGMENTS_PER_CAPTURE + 1), anchor + 1)
    if index in by_index and all(candidate in by_index for candidate in range(index, anchor + 1))
  ]


def capture_event_segment_names(
  root: str,
  route: str,
  anchor_segment: int,
  condition: str,
) -> list[str]:
  """Select a bounded window, including one post-event segment when present.

  The acceleration-while-closing trigger is intended to explain the braking
  response immediately after it. A following segment is therefore more useful
  than the second predecessor when the event happens near a segment boundary.
  """
  if condition != "stock_scc_close_accel":
    return capture_segment_names(root, route, anchor_segment)

  segments = _segment_dirs(root, route)
  by_index = {index: name for index, name, _path in segments}
  anchor = int(anchor_segment)
  if anchor not in by_index:
    return []
  end = anchor + 1 if anchor + 1 in by_index else anchor
  start = max(0, end - MAX_SEGMENTS_PER_CAPTURE + 1)
  while start < anchor and any(index not in by_index for index in range(start, end + 1)):
    start += 1
  return [by_index[index] for index in range(start, end + 1) if index in by_index]


def segment_is_complete_at(root: str, segment: str) -> bool:
  segment = _safe_segment_name(segment)
  path = os.path.abspath(os.path.join(root, segment))
  root_abs = os.path.abspath(root)
  if not segment or not path.startswith(root_abs + os.sep) or not os.path.isdir(path):
    return False
  has_rlog = False
  try:
    with os.scandir(path) as entries:
      for entry in entries:
        if entry.name.endswith(".lock"):
          return False
        if entry.name not in ("rlog.zst", "rlog.bz2", "rlog"):
          continue
        has_rlog |= entry.is_file(follow_symlinks=False) and entry.stat(follow_symlinks=False).st_size > 0
  except OSError:
    return False
  return has_rlog


def _rlog_file(root: str, segment: str) -> Path | None:
  segment = _safe_segment_name(segment)
  root_path = Path(root).resolve()
  segment_path = (root_path / segment).resolve()
  if not segment or root_path not in segment_path.parents:
    return None
  for name in ("rlog.zst", "rlog.bz2", "rlog"):
    path = segment_path / name
    try:
      if path.is_file() and path.stat().st_size > 0:
        return path
    except OSError:
      continue
  return None


def capture_size(root: str, segments: list[str]) -> int | None:
  total = 0
  for segment in segments:
    path = _rlog_file(root, segment)
    if path is None:
      return None
    try:
      total += path.stat().st_size
    except OSError:
      return None
  return total


def _validate_restored_queue_bytes(state: dict[str, Any], root: str) -> bool:
  """Verify persisted retention accounting against complete on-disk rlogs."""
  if state.get("status") == "state_invalid":
    return False
  pending_bytes = 0
  for capture in state.get("queue") or []:
    declared = _safe_int(capture.get("bytes"), low=0)
    if declared <= 0 or declared > MAX_PENDING_BYTES:
      _mark_state_invalid(state, "automatic validation capture byte accounting is not valid")
      return False
    segments = list(capture.get("segments") or [])
    if segments and all(segment_is_complete_at(root, segment) for segment in segments):
      actual = capture_size(root, segments)
      if actual is None or actual != declared:
        _mark_state_invalid(state, "automatic validation capture byte accounting does not match disk")
        return False
      declared = actual
    pending_bytes += declared
    if pending_bytes > MAX_PENDING_BYTES:
      _mark_state_invalid(state, "automatic validation pending byte limit was exceeded")
      return False
  return True


def _capture_file_manifest(
  root: str,
  segments: list[str],
  *,
  should_continue: Callable[[], bool],
) -> tuple[list[dict[str, Any]], str]:
  files: list[dict[str, Any]] = []
  for segment in segments:
    if not should_continue():
      return [], "automatic upload safety changed while hashing"
    path = _rlog_file(root, segment)
    if path is None:
      return [], f"full rlog is unavailable for {segment}"
    try:
      before = path.stat()
      digest = hashlib.sha256()
      with path.open("rb") as source:
        while True:
          if not should_continue():
            return [], "automatic upload safety changed while hashing"
          chunk = source.read(1024 * 1024)
          if not chunk:
            break
          digest.update(chunk)
      if not should_continue():
        return [], "automatic upload safety changed while hashing"
      after = path.stat()
    except OSError as exc:
      return [], f"cannot hash {segment}: {exc}"
    if (
      before.st_size <= 0
      or before.st_size != after.st_size
      or before.st_mtime_ns != after.st_mtime_ns
      or before.st_ino != after.st_ino
    ):
      return [], f"full rlog changed while hashing {segment}"
    files.append({
      "segment": segment,
      "name": path.name,
      "size": before.st_size,
      "sha256": digest.hexdigest(),
    })
  return files, ""


def _sanitize_capture_files(raw: Any, segments: list[str]) -> list[dict[str, Any]]:
  if not isinstance(raw, list):
    return []
  files = []
  for item in raw:
    if not isinstance(item, dict):
      return []
    segment = _safe_segment_name(item.get("segment"))
    name = str(item.get("name") or "")
    size = _safe_int(item.get("size"), low=0)
    sha256 = str(item.get("sha256") or "").lower()
    if (
      segment not in segments
      or name not in {"rlog", "rlog.bz2", "rlog.zst"}
      or size <= 0
      or len(sha256) != 64
      or any(character not in "0123456789abcdef" for character in sha256)
    ):
      return []
    files.append({"segment": segment, "name": name, "size": size, "sha256": sha256})
  if len(files) != len(segments) or {item["segment"] for item in files} != set(segments):
    return []
  return files


def _preserve_segments(root: str, segments: list[str]) -> tuple[list[str], str]:
  owned: list[str] = []
  newly_marked: list[str] = []
  for segment in segments:
    path = os.path.abspath(os.path.join(root, segment))
    try:
      previous = getxattr_direct(path, VALIDATION_PRESERVE_ATTR_NAME)
      if previous != PRESERVE_ATTR_VALUE:
        setxattr(path, VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)
        _fsync_path(path)
        newly_marked.append(segment)
      # Dedicated markers may be shared by overlapping captures. Recording
      # every durable owner lets ownership transfer keep the marker until the
      # final overlapping capture is removed.
      owned.append(segment)
    except OSError as exc:
      for marked_segment in newly_marked:
        _release_preserve(root, marked_segment)
      return [], f"cannot preserve {segment}: {exc}"
  return owned, ""


def _has_unprotected_close_accel_following_candidate(state: dict[str, Any]) -> bool:
  active = state.get("active_route")
  if not isinstance(active, dict):
    return False
  route = _safe_route_name(active.get("route"))
  if not route:
    return False
  for event in active.get("events") or []:
    if event.get("condition") != "stock_scc_close_accel":
      continue
    anchor = _safe_int(event.get("anchor_segment"), low=0)
    following = f"{route}--{anchor + 1}"
    if following not in (event.get("owned_preserve") or []):
      return True
  return False


def _close_accel_following_scan_due(
  state: dict[str, Any],
  *,
  now: float,
  next_check_at: float,
) -> tuple[bool, float]:
  if not _has_unprotected_close_accel_following_candidate(state):
    return False, next_check_at
  if now < next_check_at:
    return False, next_check_at
  return True, now + FOLLOWING_SEGMENT_CHECK_INTERVAL_SECONDS


def _protect_close_accel_following_segments(
  state: dict[str, Any],
  root: str,
) -> tuple[dict[str, Any], bool, list[str], str]:
  """Own a close-accel post-event segment as soon as loggerd creates it."""
  # Most routes never contain this optional diagnostic. Avoid a full
  # realdata scandir/sort unless an event still lacks its following segment.
  if not _has_unprotected_close_accel_following_candidate(state):
    return state, False, [], ""
  candidate = copy.deepcopy(state)
  active = candidate.get("active_route")
  if not isinstance(active, dict):
    return state, False, [], ""
  route = _safe_route_name(active.get("route"))
  if not route:
    return state, False, [], ""
  by_index = {index: name for index, name, _path in _segment_dirs(root, route)}
  added: list[str] = []
  for event in active.get("events") or []:
    if event.get("condition") != "stock_scc_close_accel":
      continue
    following = by_index.get(_safe_int(event.get("anchor_segment"), low=0) + 1)
    owned = list(event.get("owned_preserve") or [])
    if not following or following in owned:
      continue
    preserved, error = _preserve_segments(root, [following])
    if error:
      _defer_unreferenced_owned(candidate, added)
      return candidate, bool(added), added, error
    event["owned_preserve"] = list(dict.fromkeys(owned + preserved))[:MAX_SEGMENTS_PER_CAPTURE]
    added.extend(preserved)
  if not added:
    return state, False, [], ""
  candidate["active_route"] = active
  return candidate, True, list(dict.fromkeys(added)), ""


def _fsync_path(path: str) -> None:
  fd = os.open(path, os.O_RDONLY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def _release_preserve(root: str, segment: str) -> bool:
  path = os.path.abspath(os.path.join(root, _safe_segment_name(segment)))
  try:
    if getxattr_direct(path, VALIDATION_PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE:
      setxattr(path, VALIDATION_PRESERVE_ATTR_NAME, b"0")
      _fsync_path(path)
    return True
  except FileNotFoundError:
    return True
  except OSError:
    return False


def _capture_id(campaign_id: str, route: str, condition: str, anchor_segment: int) -> str:
  payload = f"{campaign_id}|{route}|{condition}|{anchor_segment}".encode()
  return hashlib.sha256(payload).hexdigest()[:32]


def _condition_count(state: dict[str, Any], condition: str, *, include_active: bool = True) -> int:
  count = sum(item.get("condition") == condition for item in state.get("completed", []))
  count += sum(item.get("condition") == condition for item in state.get("queue", []))
  active = state.get("active_route")
  if include_active and isinstance(active, dict):
    count += sum(item.get("condition") == condition for item in active.get("events", []))
  return count


def _retained_capture_count(state: dict[str, Any]) -> int:
  count = len(state.get("queue") or [])
  active = state.get("active_route")
  if isinstance(active, dict):
    count += len(active.get("events") or [])
  return count


def _condition_has_pending_capture(state: dict[str, Any], condition: str) -> bool:
  if any(item.get("condition") == condition for item in state.get("queue") or []):
    return True
  active = state.get("active_route")
  return isinstance(active, dict) and any(
    item.get("condition") == condition for item in active.get("events") or []
  )


def _defer_unreferenced_owned(state: dict[str, Any], segments: list[str]) -> None:
  referenced = set(_owned_preserve_segments(state))
  cleanup = list(state.get("cleanup_preserve") or [])
  cleanup.extend(segment for segment in segments if segment not in referenced)
  state["cleanup_preserve"] = list(dict.fromkeys(cleanup))


def _evict_oldest_optional_capture(state: dict[str, Any], *, include_active: bool = True) -> bool:
  """Journal one optional capture for cleanup so a required event gets space."""
  queue = list(state.get("queue") or [])
  indexed_queue = [
    (_safe_int(item.get("created_at"), low=0), index)
    for index, item in enumerate(queue)
    if item.get("condition") in OPTIONAL_CONDITIONS
  ]
  if indexed_queue:
    _created_at, index = min(indexed_queue)
    capture = queue.pop(index)
    cleanup = _transfer_or_defer_owned(capture, queue)
    state["queue"] = queue
    _defer_unreferenced_owned(state, cleanup)
    return True

  if not include_active:
    return False
  active = state.get("active_route")
  if not isinstance(active, dict):
    return False
  events = list(active.get("events") or [])
  indexed_events = [
    (_safe_int(item.get("detected_at"), low=0), index)
    for index, item in enumerate(events)
    if item.get("condition") in OPTIONAL_CONDITIONS
  ]
  if not indexed_events:
    return False
  _detected_at, index = min(indexed_events)
  event = events.pop(index)
  active["events"] = events
  state["active_route"] = active
  _defer_unreferenced_owned(state, list(event.get("owned_preserve") or []))
  return True


def enqueue_active_route_captures(
  state: dict[str, Any],
  root: str = DASHCAM_ROOT,
  *,
  now_epoch: int | None = None,
) -> tuple[dict[str, Any], bool]:
  candidate = copy.deepcopy(state)
  active = candidate.get("active_route")
  campaign = candidate.get("campaign")
  if not isinstance(active, dict) or not isinstance(campaign, dict):
    return state, False

  now = int(now_epoch if now_epoch is not None else time.time())  # noqa: TID251 - finalize timeout survives reboot
  changed = False
  queue = list(candidate.get("queue") or [])
  pending_bytes = sum(_safe_int(item.get("bytes"), low=0) for item in queue)
  remaining_events: list[dict[str, Any]] = []
  cleanup_candidates: list[str] = []
  events = list(active.get("events") or [])
  for event_index, event in enumerate(events):
    condition = str(event.get("condition") or "")
    anchor = _safe_int(event.get("anchor_segment"), low=0)
    if condition not in CAPTURE_CONDITIONS or _condition_count(
      candidate, condition, include_active=False,
    ) >= MAX_CAPTURES_PER_CONDITION:
      cleanup_candidates.extend(event.get("owned_preserve") or [])
      changed = True
      continue
    optional_pending = sum(item.get("condition") in OPTIONAL_CONDITIONS for item in queue)
    if condition in OPTIONAL_CONDITIONS and optional_pending >= MAX_OPTIONAL_PENDING_CAPTURES:
      cleanup_candidates.extend(event.get("owned_preserve") or [])
      changed = True
      continue

    segments = capture_event_segment_names(root, active["route"], anchor, condition)
    if not segments or not all(segment_is_complete_at(root, segment) for segment in segments):
      remaining_events.append(event)
      continue
    size = capture_size(root, segments)
    if size is None:
      remaining_events.append(event)
      continue
    if condition in TARGET_CONDITIONS:
      # Optional diagnostics must not consume the bounded retention budget
      # needed by one of the campaign's required conditions. Evict as many
      # queued optional captures as needed for both count and byte limits.
      while len(queue) >= MAX_PENDING_CAPTURES or pending_bytes + size > MAX_PENDING_BYTES:
        candidate["queue"] = queue
        if not _evict_oldest_optional_capture(candidate, include_active=False):
          break
        queue = list(candidate.get("queue") or [])
        pending_bytes = sum(_safe_int(item.get("bytes"), low=0) for item in queue)
        changed = True
    if len(queue) >= MAX_PENDING_CAPTURES or pending_bytes + size > MAX_PENDING_BYTES:
      remaining_events.append(event)
      remaining_events.extend(events[event_index + 1:])
      active["events"] = remaining_events
      candidate["active_route"] = active
      candidate["queue"] = queue
      _defer_unreferenced_owned(candidate, cleanup_candidates)
      candidate["status"] = "queue_limit"
      candidate["last_error"] = "automatic validation upload queue limit reached"
      return candidate, True
    capture_id = _capture_id(campaign["id"], active["route"], condition, anchor)
    if any(item.get("id") == capture_id for item in queue) or any(
      item.get("id") == capture_id for item in candidate.get("completed", [])
    ):
      cleanup_candidates.extend(event.get("owned_preserve") or [])
      changed = True
      continue
    event_owned = [
      segment for segment in (event.get("owned_preserve") or [])
      if _safe_segment_name(segment) and segment in segments
    ][:MAX_SEGMENTS_PER_CAPTURE]
    newest = segments[-1]
    additional_owned, error = (
      ([], "") if newest in event_owned else _preserve_segments(root, [newest])
    )
    if error:
      remaining_events.append(event)
      remaining_events.extend(events[event_index + 1:])
      active["events"] = remaining_events
      candidate["active_route"] = active
      candidate["queue"] = queue
      _defer_unreferenced_owned(candidate, cleanup_candidates)
      candidate["status"] = "preserve_failed"
      candidate["last_error"] = error
      return candidate, True
    owned = list(dict.fromkeys(event_owned + additional_owned))
    metadata = {
      "schemaVersion": 1,
      "campaignId": campaign["id"],
      "captureId": capture_id,
      "condition": condition,
      "route": active["route"],
      "segments": segments,
      "anchorSegment": anchor,
      "duration": event.get("duration"),
      "trigger": event.get("trigger"),
      "keepaliveRequestDelta": event.get("keepaliveRequestDelta"),
      "qualified": event.get("qualified"),
      "controllerStoppedSec": event.get("controllerStoppedSec"),
      "vEgo": event.get("vEgo"),
      "aEgo": event.get("aEgo"),
      "leadDRel": event.get("leadDRel"),
      "leadVRel": event.get("leadVRel"),
      "timeGap": event.get("timeGap"),
      "ttc": event.get("ttc"),
      "detectedAt": event.get("detected_at"),
      "settingsEpoch": _safe_int(event.get("settings_epoch"), low=0),
      "routeSettings": _sanitize_route_settings(event.get("settings", active.get("settings"))),
      "git": _sanitize_identity(active.get("identity")),
    }
    queue.append({
      "id": capture_id,
      "condition": condition,
      "route": active["route"],
      "segments": segments,
      "owned_preserve": owned,
      "metadata": metadata,
      "files": [],
      "attempts": 0,
      "next_retry_at": 0,
      "created_at": now,
      "bytes": size,
    })
    pending_bytes += size
    changed = True

  candidate["queue"] = queue
  active["events"] = remaining_events
  candidate["active_route"] = active if remaining_events else None
  if cleanup_candidates:
    _defer_unreferenced_owned(candidate, cleanup_candidates)
  if remaining_events:
    wait_started = _safe_int(active.get("finalize_wait_started_at"), low=0)
    if wait_started <= 0:
      active["finalize_wait_started_at"] = now
      changed = True
    elif now - wait_started >= CAPTURE_FINALIZE_TIMEOUT_SECONDS:
      cleanup = [
        segment
        for pending_event in remaining_events
        for segment in pending_event.get("owned_preserve") or []
      ]
      candidate["active_route"] = None
      _defer_unreferenced_owned(candidate, cleanup)
      candidate["status"] = "capture_unavailable"
      candidate["last_error"] = "route did not finalize before the automatic capture timeout"
      return candidate, True
    candidate["status"] = "waiting_for_route_finalize"
    return candidate, changed

  candidate["status"] = "queued" if queue else "armed"
  candidate["last_error"] = ""
  return candidate, True


def _new_campaign(now: int | None = None) -> dict[str, Any]:
  started_at = int(now if now is not None else time.time())  # noqa: TID251 - campaign expiry survives reboot
  token = secrets.token_hex(12)
  return {
    "id": token,
    "started_at": started_at,
    "expires_at": started_at + CAMPAIGN_LIFETIME_SECONDS,
    "base_url": VALIDATION_UPLOAD_BASE_URL,
  }


def _campaign_clock_rolled_back(campaign: dict[str, Any], now_epoch: int) -> bool:
  return now_epoch < _safe_int(campaign.get("started_at"), low=0) - CLOCK_ROLLBACK_TOLERANCE_SECONDS


def _button_type_name(value: Any) -> str:
  return str(value or "").strip().lower().rsplit(".", 1)[-1]


def _physical_res_from_car_state_messages(messages: list[Any]) -> bool:
  for message in messages:
    try:
      if any(
        bool(event.pressed) and _button_type_name(event.type) == "accelcruise"
        for event in message.carState.buttonEvents
      ):
        return True
    except Exception:
      continue
  return False


def _radar_errors_present(errors: Any) -> bool:
  """Cap'n Proto RadarData.Error is a truthy struct even when all bits are false."""
  try:
    values = errors.to_dict().values()
  except Exception:
    if isinstance(errors, dict):
      values = errors.values()
    elif isinstance(errors, (list, tuple, set, frozenset)):
      values = errors
    else:
      return bool(errors)
  return any(bool(value) for value in values)


def _submaster_service_fresh(
  sm: Any,
  service: str,
  now: float,
  *,
  require_updated: bool,
) -> bool:
  try:
    if not bool(sm.valid[service]) or not bool(sm.alive[service]):
      return False
    if require_updated and not bool(sm.updated[service]):
      return False
    log_mono_time = int(sm.logMonoTime[service])
    age = float(now) - (log_mono_time / 1e9)
    return log_mono_time > 0 and 0.0 <= age <= MAX_SAMPLE_GAP_SECONDS
  except Exception:
    return False


def sample_from_submaster(
  sm: Any,
  now: float,
  *,
  physical_res_latched: bool = False,
) -> ValidationSample | None:
  try:
    # carState drives the detector cadence and must be a new sample. The
    # control services can legitimately skip an individual 10 Hz poll, so
    # accept their most recent value only while its cereal clock stays fresh.
    if not _submaster_service_fresh(sm, "carState", now, require_updated=True):
      return None
    if not all(
      _submaster_service_fresh(sm, name, now, require_updated=False)
      for name in ("carControl", "selfdriveState")
    ):
      return None
    cs = sm["carState"]
    cc = sm["carControl"]
    selfdrive = sm["selfdriveState"]
    lane_plan = sm["lateralPlan"]
    radar_state = sm["radarState"]
    lane_service_valid = _submaster_service_fresh(
      sm, "lateralPlan", now, require_updated=False,
    )
    radar_service_valid = (
      _submaster_service_fresh(sm, "radarState", now, require_updated=False)
      and not _radar_errors_present(radar_state.radarErrors)
    )
    physical_res = physical_res_latched or any(
      bool(getattr(event, "pressed", False)) and _button_type_name(getattr(event, "type", "")) == "accelcruise"
      for event in cs.buttonEvents
    )
    return ValidationSample(
      now=now,
      standstill=bool(cs.standstill),
      cruise_enabled=bool(cs.cruiseState.enabled),
      can_valid=bool(cs.canValid),
      engaged=bool(selfdrive.enabled),
      lat_active=bool(cc.latActive),
      v_ego=float(cs.vEgo),
      a_ego=float(cs.aEgo),
      steering_pressed=bool(cs.steeringPressed),
      brake_pressed=bool(cs.brakePressed),
      gas_pressed=bool(cs.gasPressed),
      brake_hold_active=bool(cs.brakeHoldActive),
      parking_brake=bool(cs.parkingBrake),
      acc_faulted=bool(cs.accFaulted),
      cancel_requested=bool(cc.cruiseControl.cancel),
      physical_res_pressed=physical_res,
      ka4_keepalive_request_count=int(getattr(cs, "ka4StockSccKeepaliveRequestCount", 0)),
      ka4_keepalive_qualified=bool(getattr(cs, "ka4StockSccKeepaliveQualified", False)),
      ka4_keepalive_stopped_sec=float(getattr(cs, "ka4StockSccKeepaliveStoppedSec", 0.0)),
      lane_plan_valid=lane_service_valid and bool(lane_plan.mpcSolutionValid),
      use_lane_lines=bool(lane_plan.useLaneLines),
      static_path_offset=float(lane_plan.staticPathOffset),
      dynamic_lane_offset=float(lane_plan.dynamicLaneOffset),
      radar_valid=radar_service_valid,
      lead_status=radar_service_valid and bool(radar_state.leadOne.status),
      lead_d_rel=float(radar_state.leadOne.dRel) if radar_service_valid else 0.0,
      lead_v_rel=float(radar_state.leadOne.vRel) if radar_service_valid else 0.0,
    )
  except Exception:
    return None


PUBLIC_STATUS_CODES = frozenset({
  "armed",
  "capture_unavailable",
  "cleanup_pending",
  "clock_rollback",
  "complete",
  "complete_disable_failed",
  "destination_changed",
  "disabled",
  "error",
  "expired",
  "expired_disable_failed",
  "https_required",
  "idle",
  "manual_upload_active",
  "parked_consent_required",
  "preserve_failed",
  "preserve_recovery_failed",
  "queue_limit",
  "queued",
  "reconsent_required",
  "recording",
  "retry_wait",
  "service_error",
  "state_invalid",
  "state_write_failed",
  "upload_blocked",
  "uploading",
  "vehicle_not_supported",
  "waiting_for_route_finalize",
})


def _public_status_code(value: Any) -> str:
  status = str(value or "disabled")
  return status if status in PUBLIC_STATUS_CODES else "error"


def public_validation_upload_status(state: dict[str, Any], enabled: bool) -> dict[str, Any]:
  campaign = state.get("campaign") if isinstance(state.get("campaign"), dict) else {}
  active = state.get("active_route") if isinstance(state.get("active_route"), dict) else {}
  status_code = _public_status_code(state.get("status"))
  return {
    "enabled": bool(enabled),
    "statusCode": status_code,
    "hasActiveRoute": bool(active),
    "expiresAt": _safe_int(campaign.get("expires_at"), low=0),
    "pendingCaptures": len(state.get("queue") or []),
    "completedConditions": sorted({
      item.get("condition")
      for item in state.get("completed", [])
      if item.get("condition") in CAPTURE_CONDITIONS
    }),
    "lastUploadedAt": _safe_int(state.get("last_uploaded_at"), low=0),
    "lastErrorCode": status_code if state.get("last_error") else "",
  }


def _upload_runtime_safety_allows(
  params: Any,
  campaign: dict[str, Any],
  *,
  device_state_safe: Callable[[], bool],
  network_state_safe: Callable[[], bool],
) -> tuple[bool, str]:
  """Fail-closed checks safe to call before every streamed file chunk."""
  try:
    if not _campaign_uses_trusted_receiver(campaign):
      return False, "automatic upload receiver is not trusted"
    if not device_state_safe():
      return False, "live device state is stale, invalid, or started"
    if not network_state_safe():
      return False, "live Wi-Fi state is stale or disconnected"
    if _param_int(params, VALIDATION_AUTO_UPLOAD_PARAM) <= 0:
      return False, "automatic upload consent was withdrawn"
    if not bool(params.get_bool("IsOffroad")) or bool(params.get_bool("IsOnroad")):
      return False, "device is no longer offroad"
    now_epoch = int(time.time())  # noqa: TID251 - persisted expiry
    if _campaign_clock_rolled_back(campaign, now_epoch):
      return False, "device clock moved behind the consent time"
    if now_epoch >= _safe_int(campaign.get("expires_at"), low=0):
      return False, "automatic upload campaign expired"
    return True, ""
  except Exception as exc:
    return False, f"automatic upload safety check failed: {exc}"


async def _upload_policy_allows(
  params: Any,
  campaign: dict[str, Any],
  *,
  device_state_safe: Callable[[], bool],
  network_state_safe: Callable[[], bool],
) -> tuple[bool, str]:
  allowed, error = _upload_runtime_safety_allows(
    params,
    campaign,
    device_state_safe=device_state_safe,
    network_state_safe=network_state_safe,
  )
  return (True, "") if allowed else (False, error)


def _retry_delay(attempts: int) -> int:
  base = RETRY_DELAYS[min(max(0, attempts - 1), len(RETRY_DELAYS) - 1)]
  return max(1, round(base * random.uniform(0.9, 1.1)))


def _drop_capture_durably(
  state: dict[str, Any],
  index: int,
  reason: str,
  *,
  state_path: str,
  root: str,
) -> dict[str, Any]:
  candidate = copy.deepcopy(state)
  queue = list(candidate.get("queue") or [])
  capture = queue.pop(index)
  cleanup = _transfer_or_defer_owned(capture, queue)
  candidate["queue"] = queue
  _defer_unreferenced_owned(candidate, cleanup)
  candidate["status"] = "capture_unavailable"
  candidate["last_error"] = reason[-1000:]
  if not write_validation_upload_state(candidate, state_path):
    state["status"] = "state_write_failed"
    state["last_error"] = "unavailable capture could not be removed durably"
    return state
  _finish_preserve_cleanup(candidate, root)
  write_validation_upload_state(candidate, state_path)
  return candidate


async def _upload_first_capture(
  state: dict[str, Any],
  params: Any,
  *,
  device_state_safe: Callable[[], bool],
  network_state_safe: Callable[[], bool],
  state_path: str = CARROT_VALIDATION_UPLOAD_STATE_PATH,
  root: str = DASHCAM_ROOT,
) -> dict[str, Any]:
  queue = list(state.get("queue") or [])
  if not queue:
    return state
  if not _validate_restored_queue_bytes(state, root):
    return state
  now_epoch = int(time.time())  # noqa: TID251 - persisted retry/finalize deadlines
  selected_index: int | None = None
  waiting_for_retry = False
  repaired_created_at = False
  for index, queued_capture in enumerate(queue):
    created_at = _safe_int(queued_capture.get("created_at"), low=0)
    if created_at <= 0:
      queued_capture["created_at"] = now_epoch
      created_at = now_epoch
      repaired_created_at = True
    retry_at = _safe_int(queued_capture.get("next_retry_at"), low=0)
    if retry_at > now_epoch:
      waiting_for_retry = True
      continue
    if all(segment_is_complete_at(root, segment) for segment in queued_capture.get("segments") or []):
      selected_index = index
      break
    if now_epoch - created_at >= CAPTURE_FINALIZE_TIMEOUT_SECONDS:
      return _drop_capture_durably(
        state,
        index,
        "queued capture files did not remain available before the finalize timeout",
        state_path=state_path,
        root=root,
      )

  if selected_index is None:
    state["queue"] = queue
    state["status"] = "retry_wait" if waiting_for_retry else "waiting_for_route_finalize"
    if repaired_created_at and not write_validation_upload_state(state, state_path):
      state["status"] = "state_write_failed"
      state["last_error"] = "capture creation time repair could not be saved"
    return state
  if selected_index:
    queue.insert(0, queue.pop(selected_index))
    state["queue"] = queue
  capture = queue[0]
  if repaired_created_at and not write_validation_upload_state(state, state_path):
    state["status"] = "state_write_failed"
    state["last_error"] = "capture creation time repair could not be saved"
    return state
  campaign = state.get("campaign") or {}
  runtime_ok, runtime_error = _upload_runtime_safety_allows(
    params,
    campaign,
    device_state_safe=device_state_safe,
    network_state_safe=network_state_safe,
  )
  if not runtime_ok:
    state["status"] = "destination_changed" if "receiver" in runtime_error else "upload_blocked"
    state["last_error"] = runtime_error
    return state
  files = _sanitize_capture_files(capture.get("files"), list(capture.get("segments") or []))
  if not files:
    def hash_safety_check() -> bool:
      allowed, _error = _upload_runtime_safety_allows(
        params,
        state.get("campaign") or {},
        device_state_safe=device_state_safe,
        network_state_safe=network_state_safe,
      )
      return allowed

    files, manifest_error = await asyncio.to_thread(
      _capture_file_manifest,
      root,
      list(capture.get("segments") or []),
      should_continue=hash_safety_check,
    )
    if manifest_error:
      capture["attempts"] = _safe_int(capture.get("attempts"), low=0) + 1
      capture["next_retry_at"] = now_epoch + _retry_delay(capture["attempts"])
      state["queue"] = queue
      state["status"] = "retry_wait"
      state["last_error"] = manifest_error[-1000:]
      return state
    capture["files"] = files
    state["queue"] = queue
    if not write_validation_upload_state(state, state_path):
      state["status"] = "state_write_failed"
      state["last_error"] = "validation file hashes could not be saved before upload"
      return state
  if upload_jobs.has_running_job():
    state["status"] = "manual_upload_active"
    return state

  policy_ok, policy_error = await _upload_policy_allows(
    params,
    campaign,
    device_state_safe=device_state_safe,
    network_state_safe=network_state_safe,
  )
  if not policy_ok:
    state["status"] = "destination_changed" if "receiver" in policy_error else "upload_blocked"
    state["last_error"] = policy_error
    return state
  # Recheck immediately before synchronous registry insertion so a manual
  # upload cannot race the cached runtime-policy decision above.
  if upload_jobs.has_running_job():
    state["status"] = "manual_upload_active"
    return state

  def chunk_safety_check() -> bool:
    allowed, _error = _upload_runtime_safety_allows(
      params,
      campaign,
      device_state_safe=device_state_safe,
      network_state_safe=network_state_safe,
    )
    return allowed

  job = upload_jobs.create_job(
    list(capture["segments"]),
    action="validation_auto_upload",
    run_options={
      "artifact_kinds": frozenset({"rlog"}),
      "notify_discord": False,
      "concurrency_override": 1,
      "completion_metadata": dict(capture.get("metadata") or {}),
      "base_url_override": str(campaign.get("base_url") or ""),
      "safety_check": chunk_safety_check,
      "validation_capture_id": str(capture.get("id") or ""),
      "validation_files": files,
    },
  )
  state["status"] = "uploading"
  state["last_error"] = ""
  if not write_validation_upload_state(state, state_path):
    state["status"] = "state_write_failed"
    state["last_error"] = "automatic validation upload queue could not be saved"
    upload_jobs.jobs().pop(job["id"], None)
    return state

  task = upload_jobs.start_job(job)
  parent_cancelled = False
  upload_error = ""
  try:
    while not task.done():
      try:
        await asyncio.wait_for(asyncio.shield(task), timeout=1.0)
      except TimeoutError:
        policy_ok, upload_error = await _upload_policy_allows(
          params,
          campaign,
          device_state_safe=device_state_safe,
          network_state_safe=network_state_safe,
        )
        if not policy_ok:
          upload_jobs.cancel_job(job["id"])
          task.cancel()
    await task
  except asyncio.CancelledError:
    current_task = asyncio.current_task()
    parent_cancelled = bool(current_task is not None and current_task.cancelling())
    if not parent_cancelled:
      upload_error = "automatic upload child task was canceled"
  except Exception as exc:
    upload_error = f"automatic upload safety check failed: {exc}"
  finally:
    if not task.done():
      upload_jobs.cancel_job(job["id"])
      task.cancel()
      await asyncio.gather(task, return_exceptions=True)

  if parent_cancelled:
    upload_jobs.fail_running_job(job, "automatic upload service stopped")
    raise asyncio.CancelledError
  if upload_error:
    upload_jobs.fail_running_job(job, upload_error)

  final_policy_ok, final_policy_error = await _upload_policy_allows(
    params,
    campaign,
    device_state_safe=device_state_safe,
    network_state_safe=network_state_safe,
  )
  if not final_policy_ok:
    upload_error = final_policy_error

  result = job.get("result") if isinstance(job.get("result"), dict) else {}
  successes = {
    item.get("segment") for item in result.get("results", [])
    if isinstance(item, dict) and item.get("ok")
  }
  web_complete = result.get("webComplete") if isinstance(result.get("webComplete"), dict) else {}
  requested_device_id = str(result.get("deviceId") or "")
  capture_id = str(capture.get("id") or "")
  try:
    expected_manifest_sha256 = validation_manifest_sha256(
      requested_device_id,
      capture_id,
      files,
      capture.get("metadata") if isinstance(capture.get("metadata"), dict) else None,
    )
  except Exception:
    expected_manifest_sha256 = ""
  expected_receipt_id = validation_receipt_id(expected_manifest_sha256)
  receipt_valid = (
    web_complete.get("ok") is True
    and web_complete.get("receiptVersion") == 1
    and bool(requested_device_id)
    and result.get("captureId") == capture_id
    and web_complete.get("deviceId") == requested_device_id
    and web_complete.get("verifiedDeviceId") == requested_device_id
    and web_complete.get("captureId") == capture_id
    and web_complete.get("manifestSha256") == expected_manifest_sha256
    and web_complete.get("receiptId") == expected_receipt_id
  )
  if not upload_error and result.get("ok") and receipt_valid and successes == set(capture["segments"]):
    previous_completed = list(state.get("completed") or [])
    previous_last_uploaded_at = _safe_int(state.get("last_uploaded_at"), low=0)
    previous_cleanup_preserve = list(state.get("cleanup_preserve") or [])
    remaining_queue = queue[1:]
    cleanup_preserve = _transfer_or_defer_owned(capture, remaining_queue)
    uploaded_at = int(time.time())  # noqa: TID251 - persisted/public upload timestamp
    state["queue"] = remaining_queue
    # A still-active event can overlap this capture's anchor while waiting for
    # its following segment. Journal only anchors that no remaining capture or
    # event references, otherwise cleanup could expose that event to deleter.
    _defer_unreferenced_owned(state, cleanup_preserve)
    state["completed"] = (list(state.get("completed") or []) + [{
      "id": capture["id"],
      "condition": capture["condition"],
      "route": capture["route"],
      "uploaded_at": uploaded_at,
      "receipt_id": str(web_complete.get("receiptId") or ""),
      "manifest_sha256": str(web_complete.get("manifestSha256") or ""),
    }])[-20:]
    state["last_uploaded_at"] = uploaded_at
    state["status"] = "queued" if state["queue"] else "armed"
    state["last_error"] = ""
    if write_validation_upload_state(state, state_path):
      _finish_preserve_cleanup(state, root)
      write_validation_upload_state(state, state_path)
    else:
      # Keep the disk's older queued record and its preserve marker intact so
      # a reboot safely retries rather than losing the only durable reference.
      state["queue"] = queue
      state["completed"] = previous_completed
      state["last_uploaded_at"] = previous_last_uploaded_at
      state["cleanup_preserve"] = previous_cleanup_preserve
      state["status"] = "state_write_failed"
      state["last_error"] = "upload completed but queue state could not be committed"
  else:
    capture["attempts"] = _safe_int(capture.get("attempts"), low=0) + 1
    capture["next_retry_at"] = int(time.time()) + _retry_delay(capture["attempts"])  # noqa: TID251 - persisted deadline
    state["queue"] = queue
    state["status"] = "retry_wait"
    completion_error = web_complete.get("error") if isinstance(web_complete, dict) else ""
    state["last_error"] = str(
      upload_error or result.get("error") or completion_error or result.get("message") or "automatic upload failed"
    )[-1000:]
  return state


async def _upload_first_capture_with_live_device_state(
  state: dict[str, Any],
  params: Any,
  sm: Any,
  device_state_guard: LiveDeviceStateStoppedGuard,
  network_guard: LiveWifiGuard,
  *,
  state_path: str,
  root: str,
  poll_interval: float,
) -> dict[str, Any]:
  watcher = asyncio.create_task(_refresh_live_device_state(
    sm,
    device_state_guard,
    poll_interval=poll_interval,
  ))
  network_ready = asyncio.Event()
  network_watcher = asyncio.create_task(_refresh_live_wifi(
    network_guard,
    poll_interval=min(poll_interval, NETWORK_GUARD_POLL_INTERVAL_SECONDS),
    initial_observation=network_ready,
  ))
  try:
    # No hash or request may start from an uninitialized network cache. A
    # blocked platform probe times out and remains fail closed.
    try:
      await asyncio.wait_for(
        network_ready.wait(),
        timeout=NETWORK_GUARD_INITIAL_TIMEOUT_SECONDS,
      )
    except TimeoutError:
      network_guard.invalidate()
    # Let the deviceState watcher refresh once before disk hashing starts.
    await asyncio.sleep(0)
    return await _upload_first_capture(
      state,
      params,
      device_state_safe=device_state_guard.allows_upload,
      network_state_safe=network_guard.allows_upload,
      state_path=state_path,
      root=root,
    )
  finally:
    watcher.cancel()
    network_watcher.cancel()
    await asyncio.gather(watcher, network_watcher, return_exceptions=True)


def _live_owned_preserve_segments(state: dict[str, Any]) -> list[str]:
  owned: list[str] = []
  for capture in state.get("queue") or []:
    owned.extend(capture.get("owned_preserve") or [])
  active = state.get("active_route")
  if isinstance(active, dict):
    for event in active.get("events") or []:
      owned.extend(event.get("owned_preserve") or [])
  return list(dict.fromkeys(filter(None, (_safe_segment_name(value) for value in owned))))


def _owned_preserve_segments(state: dict[str, Any]) -> list[str]:
  owned = list(state.get("cleanup_preserve") or [])
  owned.extend(_live_owned_preserve_segments(state))
  return list(dict.fromkeys(filter(None, (_safe_segment_name(value) for value in owned))))


def _terminal_cleanup_state(state: dict[str, Any], status: str) -> dict[str, Any]:
  terminal = _default_state()
  terminal["status"] = status
  terminal["cleanup_preserve"] = _owned_preserve_segments(state)
  terminal["completed"] = list(state.get("completed") or [])[-20:]
  terminal["last_uploaded_at"] = _safe_int(state.get("last_uploaded_at"), low=0)
  terminal["last_error"] = ""
  return terminal


def _transfer_or_defer_owned(
  capture: dict[str, Any],
  remaining_queue: list[dict[str, Any]],
) -> list[str]:
  """Transfer shared anchors and return unreferenced anchors for durable cleanup."""
  cleanup: list[str] = []
  for segment in capture.get("owned_preserve") or []:
    next_owner = next(
      (item for item in remaining_queue if segment in (item.get("segments") or [])),
      None,
    )
    if next_owner is None:
      cleanup.append(segment)
      continue
    owned = list(next_owner.get("owned_preserve") or [])
    if segment not in owned:
      next_owner["owned_preserve"] = (owned + [segment])[:MAX_SEGMENTS_PER_CAPTURE]
  return cleanup


def _finish_preserve_cleanup(state: dict[str, Any], root: str) -> bool:
  segments = list(state.get("cleanup_preserve") or [])
  live_owned = set(_live_owned_preserve_segments(state))
  overlap = [segment for segment in segments if segment in live_owned]
  remaining = [
    segment
    for segment in segments
    if segment in live_owned or not _release_preserve(root, segment)
  ]
  changed = remaining != segments
  state["cleanup_preserve"] = remaining
  if overlap:
    _mark_state_invalid(state, "automatic validation cleanup overlaps live ownership")
  elif remaining:
    state["last_error"] = "automatic validation retention cleanup will be retried"
  elif segments and state.get("last_error") == "automatic validation retention cleanup will be retried":
    state["last_error"] = ""
  return changed


def _reconcile_validation_preserve(state: dict[str, Any], root: str) -> tuple[bool, bool]:
  """Reassert durable owners, then journal orphan markers before releasing."""
  # An unreadable/corrupt state cannot prove which validation marker owns which
  # capture. Fail closed: leave every marker untouched and require a deliberate
  # recovery rather than converting all retained logs into apparent orphans.
  if state.get("status") == "state_invalid":
    return False, False
  referenced = set(_owned_preserve_segments(state))
  if not os.path.isdir(root):
    return False, False

  for segment in referenced:
    path = os.path.abspath(os.path.join(root, segment))
    try:
      if getxattr_direct(path, VALIDATION_PRESERVE_ATTR_NAME) != PRESERVE_ATTR_VALUE:
        setxattr(path, VALIDATION_PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)
        _fsync_path(path)
    except FileNotFoundError:
      # The bounded finalize timeout will remove an unavailable capture. Do not
      # release unrelated markers merely because this route is already gone.
      continue
    except OSError as exc:
      state["status"] = "preserve_recovery_failed"
      state["last_error"] = f"automatic validation retention could not be recovered: {exc}"[-1000:]
      return False, False

  orphans: list[str] = []
  try:
    with os.scandir(root) as entries:
      for entry in entries:
        segment = _safe_segment_name(entry.name)
        if not segment or segment in referenced or not entry.is_dir(follow_symlinks=False):
          continue
        try:
          if getxattr_direct(entry.path, VALIDATION_PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE:
            orphans.append(segment)
        except OSError as exc:
          state["status"] = "preserve_recovery_failed"
          state["last_error"] = f"automatic validation retention scan failed: {exc}"[-1000:]
          return False, False
  except OSError as exc:
    state["status"] = "preserve_recovery_failed"
    state["last_error"] = f"automatic validation retention scan failed: {exc}"[-1000:]
    return False, False

  previous = list(state.get("cleanup_preserve") or [])
  available = max(0, MAX_VALIDATION_PRESERVE_SEGMENTS - len(referenced))
  batch = orphans[:available]
  state["cleanup_preserve"] = list(dict.fromkeys(previous + batch))
  all_orphans_journaled = len(batch) == len(orphans)
  cleanup_batch_ready = state["cleanup_preserve"] != previous or (
    not all_orphans_journaled and bool(state["cleanup_preserve"])
  )
  return cleanup_batch_ready, all_orphans_journaled


async def _validation_auto_upload_worker(
  *,
  state_path: str = CARROT_VALIDATION_UPLOAD_STATE_PATH,
  root: str = DASHCAM_ROOT,
  poll_interval: float = POLL_INTERVAL,
) -> None:
  if not HAS_PARAMS or Params is None:
    return
  params = Params()
  sm = messaging.SubMaster([
    "carState",
    "carControl",
    "selfdriveState",
    "lateralPlan",
    "radarState",
    "deviceState",
  ])
  # carState.buttonEvents are one-frame edges. Keep a second non-conflated
  # socket and drain it so the 10 Hz service loop cannot miss physical RES.
  car_state_events_sock = messaging.sub_sock("carState", conflate=False)
  device_state_guard = LiveDeviceStateStoppedGuard()
  network_guard = LiveWifiGuard()
  state = read_validation_upload_state(state_path)
  _validate_restored_queue_bytes(state, root)
  if state.get("status") == "state_invalid":
    # Never rewrite the damaged ownership journal or release markers. Only the
    # consent bit is changed, so collection/upload remains disabled fail-closed.
    _disable_invalid_state_consent(state, params)
    reconciled, retention_ready = False, False
  else:
    reconciled, retention_ready = _reconcile_validation_preserve(state, root)
    if reconciled or (retention_ready and state.get("cleanup_preserve")):
      if write_validation_upload_state(state, state_path):
        _finish_preserve_cleanup(state, root)
        write_validation_upload_state(state, state_path)
      else:
        retention_ready = False
    elif not retention_ready:
      write_validation_upload_state(state, state_path)
  detector = ValidationEventDetector()
  last_enabled = _param_int(params, VALIDATION_AUTO_UPLOAD_PARAM) > 0
  campaign_deadline_id = ""
  campaign_monotonic_deadline: float | None = None
  offroad_since: float | None = None
  physical_res_latched_until = 0.0
  next_route_refresh = 0.0
  next_following_segment_check = 0.0
  next_finalize_check = 0.0
  next_upload_check = 0.0
  next_cleanup_check = 0.0
  current_route = ""

  while True:
    try:
      physical_res_edge = _physical_res_from_car_state_messages(
        messaging.drain_sock(car_state_events_sock),
      )
      sm.update(0)
      now_mono = time.monotonic()
      device_state_guard.observe_submaster(sm, now=now_mono)
      if physical_res_edge:
        physical_res_latched_until = now_mono + PHYSICAL_RES_LATCH_SECONDS
      now_epoch = int(time.time())  # noqa: TID251 - campaign and route metadata survive reboot
      enabled = _param_int(params, VALIDATION_AUTO_UPLOAD_PARAM) > 0
      enabled_rising = enabled and not last_enabled
      last_enabled = enabled
      campaign = state.get("campaign") if isinstance(state.get("campaign"), dict) else None
      if campaign is not None and str(campaign.get("id") or "") != campaign_deadline_id:
        remaining = max(0, min(
          CAMPAIGN_LIFETIME_SECONDS,
          _safe_int(campaign.get("expires_at"), low=0) - now_epoch,
        ))
        campaign_deadline_id = str(campaign.get("id") or "")
        campaign_monotonic_deadline = now_mono + remaining

      if state.get("status") == "state_invalid":
        if enabled:
          disabled = _disable_invalid_state_consent(state, params)
          last_enabled = False if disabled else enabled
        detector.reset()
        physical_res_latched_until = 0.0
        await asyncio.sleep(max(1.0, poll_interval))
        continue

      if not retention_ready:
        if now_mono >= next_cleanup_check:
          next_cleanup_check = now_mono + NETWORK_CHECK_INTERVAL
          reconciled, retention_ready = _reconcile_validation_preserve(state, root)
          if reconciled or (retention_ready and state.get("cleanup_preserve")):
            if write_validation_upload_state(state, state_path):
              _finish_preserve_cleanup(state, root)
              write_validation_upload_state(state, state_path)
            else:
              retention_ready = False
          elif not retention_ready:
            write_validation_upload_state(state, state_path)
        if not retention_ready:
          await asyncio.sleep(max(1.0, poll_interval))
          continue

      if state.get("cleanup_preserve") and now_mono >= next_cleanup_check:
        next_cleanup_check = now_mono + NETWORK_CHECK_INTERVAL
        _finish_preserve_cleanup(state, root)
        write_validation_upload_state(state, state_path)

      if not enabled:
        terminal_status = state.get("status") in {"complete", "expired", "state_invalid"}
        if campaign or state.get("queue") or state.get("active_route"):
          desired_status = (
            "expired"
            if campaign is not None and now_epoch >= _safe_int(campaign.get("expires_at"), low=0)
            else "disabled"
          )
          terminal = _terminal_cleanup_state(state, desired_status)
          if write_validation_upload_state(terminal, state_path):
            state = terminal
            _finish_preserve_cleanup(state, root)
            write_validation_upload_state(state, state_path)
          else:
            state["status"] = "state_write_failed"
            state["last_error"] = "retention cleanup could not be journaled"
        elif not terminal_status and state.get("status") != "disabled":
          state["status"] = "disabled"
          state["last_error"] = ""
          write_validation_upload_state(state, state_path)
        detector.reset()
        physical_res_latched_until = 0.0
        await asyncio.sleep(max(1.0, poll_interval))
        continue

      if campaign is None:
        if state.get("cleanup_preserve"):
          state["status"] = "cleanup_pending"
          state["last_error"] = "finish retention cleanup before starting a new campaign"
          write_validation_upload_state(state, state_path)
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        if not enabled_rising:
          disabled = _disable_consent(params)
          last_enabled = False if disabled else enabled
          state["status"] = "reconsent_required"
          state["last_error"] = "automatic collection requires a new parked off-to-on consent action"
          write_validation_upload_state(state, state_path)
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        try:
          parked = bool(params.get_bool("IsOffroad")) and not bool(params.get_bool("IsOnroad"))
        except Exception:
          parked = False
        if not parked:
          disabled = _disable_consent(params)
          last_enabled = False if disabled else enabled
          state["status"] = "parked_consent_required"
          state["last_error"] = "automatic collection can only be enabled while parked and offroad"
          write_validation_upload_state(state, state_path)
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        if not _is_https_upload_url(VALIDATION_UPLOAD_BASE_URL):
          disabled = _disable_consent(params)
          last_enabled = False if disabled else enabled
          state["status"] = "https_required"
          state["last_error"] = "automatic validation receiver deployment requires HTTPS"
          write_validation_upload_state(state, state_path)
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        candidate = _default_state()
        candidate["campaign"] = _new_campaign(now_epoch)
        candidate["status"] = "armed"
        candidate["last_error"] = ""
        if not write_validation_upload_state(candidate, state_path):
          disabled = _disable_consent(params)
          last_enabled = False if disabled else enabled
          state["status"] = "state_write_failed"
          state["last_error"] = "consent state could not be saved; automatic collection was not started"
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        state = candidate
        campaign = state["campaign"]
        campaign_deadline_id = str(campaign.get("id") or "")
        campaign_monotonic_deadline = now_mono + CAMPAIGN_LIFETIME_SECONDS

      clock_rollback = _campaign_clock_rolled_back(campaign, now_epoch)
      if (
        clock_rollback
        or now_epoch >= _safe_int(campaign.get("expires_at"), low=0)
        or (campaign_monotonic_deadline is not None and now_mono >= campaign_monotonic_deadline)
      ):
        if not _disable_consent(params):
          if state.get("status") != "expired_disable_failed":
            state["status"] = "expired_disable_failed"
            state["last_error"] = "campaign expired but consent setting could not be disabled"
            write_validation_upload_state(state, state_path)
          await asyncio.sleep(max(1.0, poll_interval))
          continue
        last_enabled = False
        terminal = _terminal_cleanup_state(state, "clock_rollback" if clock_rollback else "expired")
        if write_validation_upload_state(terminal, state_path):
          state = terminal
          _finish_preserve_cleanup(state, root)
          write_validation_upload_state(state, state_path)
        else:
          state["status"] = "state_write_failed"
          state["last_error"] = "expired retention cleanup could not be journaled"
        detector.reset()
        await asyncio.sleep(max(1.0, poll_interval))
        continue

      completed_conditions = {item.get("condition") for item in state.get("completed", [])}
      if (
        TARGET_CONDITIONS.issubset(completed_conditions)
        and not state.get("queue")
        and state.get("active_route") is None
      ):
        if _disable_consent(params):
          last_enabled = False
          state["status"] = "complete"
          state["campaign"] = None
          state["last_error"] = ""
          write_validation_upload_state(state, state_path)
        elif state.get("status") != "complete_disable_failed":
          state["status"] = "complete_disable_failed"
          state["last_error"] = "campaign completed but consent setting could not be disabled"
          write_validation_upload_state(state, state_path)
        detector.reset()
        await asyncio.sleep(max(1.0, poll_interval))
        continue

      is_onroad = bool(params.get_bool("IsOnroad"))
      is_offroad = bool(params.get_bool("IsOffroad")) and not is_onroad
      active = state.get("active_route") if isinstance(state.get("active_route"), dict) else None

      if is_onroad:
        offroad_since = None
        next_finalize_check = 0.0
        if now_mono >= next_route_refresh:
          current_route = _safe_route_name(_param_text(params, "CurrentRoute"))
          next_route_refresh = now_mono + ROUTE_REFRESH_INTERVAL
          if current_route and active is not None and active.get("route") != current_route:
            candidate, changed = enqueue_active_route_captures(state, root, now_epoch=now_epoch)
            if changed:
              if write_validation_upload_state(candidate, state_path):
                state = candidate
              else:
                state["status"] = "state_write_failed"
                state["last_error"] = "route transition capture queue could not be saved"
                retention_ready = False
                detector.invalidate_sample()
                await asyncio.sleep(max(1.0, poll_interval))
                continue
            active = state.get("active_route") if isinstance(state.get("active_route"), dict) else None

          controls_ready = bool(params.get_bool("ControlsReady"))
          if current_route and active is None and controls_ready:
            gate, topology = ka4_stock_scc_gate(params)
            if not gate:
              state["status"] = "vehicle_not_supported"
              state["last_error"] = str(topology.get("reason") or "KA4 stock-SCC topology gate failed")
              state["active_route"] = None
              write_validation_upload_state(state, state_path)
              active = None
            else:
              active = {
                "route": current_route,
                "started_at": now_epoch,
                "finalize_wait_started_at": 0,
                "settings_epoch": 0,
                "settings": _route_settings(params),
                "identity": {**await asyncio.to_thread(_git_identity, params), "topology": topology},
                "events": [],
              }
              state["active_route"] = active
              state["status"] = "recording"
              state["last_error"] = ""
              detector.reset()
              write_validation_upload_state(state, state_path)

        if active is not None and current_route and active.get("route") == current_route:
          settings_candidate, settings_changed = _update_active_route_settings(state, _route_settings(params))
          if settings_changed:
            if write_validation_upload_state(settings_candidate, state_path):
              state = settings_candidate
              active = state.get("active_route")
              detector.reset()
            else:
              state["status"] = "state_write_failed"
              state["last_error"] = "route setting change could not be saved"
              detector.invalidate_sample()
              active = None

        following_scan_due, next_following_segment_check = _close_accel_following_scan_due(
          state,
          now=now_mono,
          next_check_at=next_following_segment_check,
        )
        if active is not None and following_scan_due:
          preserve_candidate, preserve_changed, newly_owned, preserve_error = (
            _protect_close_accel_following_segments(state, root)
          )
          if preserve_changed:
            if write_validation_upload_state(preserve_candidate, state_path):
              state = preserve_candidate
              active = state.get("active_route")
            else:
              _defer_unreferenced_owned(state, newly_owned)
              if write_validation_upload_state(state, state_path):
                _finish_preserve_cleanup(state, root)
                write_validation_upload_state(state, state_path)
              state["status"] = "state_write_failed"
              state["last_error"] = "post-event retention ownership could not be saved"
              active = None
          if preserve_error:
            state["status"] = "preserve_failed"
            state["last_error"] = preserve_error
            write_validation_upload_state(state, state_path)

        physical_res_latched = now_mono <= physical_res_latched_until
        sample = sample_from_submaster(sm, now_mono, physical_res_latched=physical_res_latched)
        if sample is None:
          detector.invalidate_sample()
        elif physical_res_latched:
          physical_res_latched_until = 0.0
        if (
          active is not None
          and current_route
          and active.get("route") == current_route
          and sample is not None
        ):
          for event in detector.update(sample, active.get("settings") or {}):
            condition = event["condition"]
            if (
              _condition_count(state, condition) >= MAX_CAPTURES_PER_CONDITION
              or _condition_has_pending_capture(state, condition)
            ):
              continue
            retained = _retained_capture_count(state)
            optional_retained = sum(
              item.get("condition") in OPTIONAL_CONDITIONS for item in state.get("queue") or []
            ) + sum(
              item.get("condition") in OPTIONAL_CONDITIONS for item in active.get("events") or []
            )
            if condition in OPTIONAL_CONDITIONS and optional_retained >= MAX_OPTIONAL_PENDING_CAPTURES:
              continue
            if retained >= MAX_PENDING_CAPTURES:
              if condition not in TARGET_CONDITIONS:
                state["status"] = "queue_limit"
                state["last_error"] = "automatic validation retention limit reached"
                write_validation_upload_state(state, state_path)
                continue
              candidate = copy.deepcopy(state)
              if not _evict_oldest_optional_capture(candidate):
                state["status"] = "queue_limit"
                state["last_error"] = "automatic validation retention limit reached"
                write_validation_upload_state(state, state_path)
                detector.nack(condition)
                continue
              if not write_validation_upload_state(candidate, state_path):
                state["status"] = "state_write_failed"
                state["last_error"] = "optional capture eviction could not be saved"
                detector.nack(condition)
                continue
              state = candidate
              active = state.get("active_route") if isinstance(state.get("active_route"), dict) else None
              if active is None:
                detector.nack(condition)
                continue
            anchor = latest_route_segment(root, active["route"])
            if anchor is None:
              detector.nack(condition)
              continue
            anchor_names = capture_segment_names(root, active["route"], anchor)
            if not anchor_names:
              detector.nack(condition)
              continue
            owned, preserve_error = _preserve_segments(root, anchor_names[-1:])
            if preserve_error:
              state["status"] = "preserve_failed"
              state["last_error"] = preserve_error
              write_validation_upload_state(state, state_path)
              detector.nack(condition)
              continue
            captured_event = {
              **event,
              "anchor_segment": anchor,
              "owned_preserve": owned,
              "detected_at": now_epoch,
              "settings_epoch": _safe_int(active.get("settings_epoch"), low=0),
              "settings": _sanitize_route_settings(active.get("settings")),
            }
            active["events"].append(captured_event)
            state["active_route"] = active
            if not write_validation_upload_state(state, state_path):
              active["events"].pop()
              _defer_unreferenced_owned(state, owned)
              if write_validation_upload_state(state, state_path):
                _finish_preserve_cleanup(state, root)
                write_validation_upload_state(state, state_path)
              state["status"] = "state_write_failed"
              state["last_error"] = "automatic validation event could not be saved"
              detector.nack(condition)
      elif is_offroad:
        current_route = ""
        next_route_refresh = 0.0
        next_following_segment_check = 0.0
        offroad_since = offroad_since or now_mono
        if active is not None and now_mono >= next_finalize_check:
          next_finalize_check = now_mono + ROUTE_REFRESH_INTERVAL
          candidate, changed = enqueue_active_route_captures(state, root, now_epoch=now_epoch)
          if changed:
            if write_validation_upload_state(candidate, state_path):
              state = candidate
            else:
              state["status"] = "state_write_failed"
              state["last_error"] = "finalized capture queue could not be saved"
              retention_ready = False
              await asyncio.sleep(max(1.0, poll_interval))
              continue
          active = state.get("active_route") if isinstance(state.get("active_route"), dict) else None
        device_stopped = device_state_guard.allows_upload()
        if state.get("queue") and device_stopped and now_mono - offroad_since >= OFFROAD_STABLE_SECONDS:
          if now_mono >= next_upload_check:
            next_upload_check = now_mono + NETWORK_CHECK_INTERVAL
            if write_validation_upload_state(state, state_path):
              first_capture = (state.get("queue") or [{}])[0]
              before = (
                state.get("status"),
                state.get("last_error"),
                len(state.get("queue") or []),
                len(state.get("completed") or []),
                sum(int(item.get("attempts") or 0) for item in state.get("queue") or []),
                str(first_capture.get("id") or ""),
              )
              state = await _upload_first_capture_with_live_device_state(
                state,
                params,
                sm,
                device_state_guard,
                network_guard,
                state_path=state_path,
                root=root,
                poll_interval=poll_interval,
              )
              after_capture = (state.get("queue") or [{}])[0]
              after = (
                state.get("status"),
                state.get("last_error"),
                len(state.get("queue") or []),
                len(state.get("completed") or []),
                sum(int(item.get("attempts") or 0) for item in state.get("queue") or []),
                str(after_capture.get("id") or ""),
              )
              if after != before:
                write_validation_upload_state(state, state_path)
      else:
        current_route = ""
        next_route_refresh = 0.0
        next_following_segment_check = 0.0
        offroad_since = None

    except asyncio.CancelledError:
      raise
    except Exception as exc:
      if state.get("status") == "state_invalid":
        _disable_invalid_state_consent(state, params)
      else:
        state["status"] = "error"
        state["last_error"] = str(exc)[-1000:]
        write_validation_upload_state(state, state_path)
    await asyncio.sleep(poll_interval)


async def validation_auto_upload_loop(
  *,
  state_path: str = CARROT_VALIDATION_UPLOAD_STATE_PATH,
  root: str = DASHCAM_ROOT,
  poll_interval: float = POLL_INTERVAL,
  restart_delay: float = 1.0,
) -> None:
  """Supervise initialization as well as the long-running collector loop."""
  if not HAS_PARAMS or Params is None:
    return
  while True:
    try:
      await _validation_auto_upload_worker(
        state_path=state_path,
        root=root,
        poll_interval=poll_interval,
      )
      return
    except asyncio.CancelledError:
      raise
    except Exception as exc:
      cloudlog.exception("automatic validation upload service crashed")
      state = read_validation_upload_state(state_path)
      if state.get("status") == "state_invalid":
        try:
          _disable_invalid_state_consent(state, Params())
        except Exception:
          pass
      else:
        state["status"] = "service_error"
        state["last_error"] = f"automatic collection service will restart: {exc}"[-1000:]
        write_validation_upload_state(state, state_path)
      await asyncio.sleep(max(0.01, restart_delay))
