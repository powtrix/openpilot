#!/usr/bin/env python3
"""Passive KA4 stock-SCC standstill re-arm probe.

This tool never publishes CAN, changes Params, or writes a report file. It only
subscribes to already-running openpilot services, or reads an existing rlog,
then prints a report to stdout.

On a comma 3X, use a controlled, legal test location. Start the capture
*before* the car reaches a full stop behind a stationary lead vehicle, keep
the roadway clear, and leave a licensed driver ready to brake at all times.
Never rely on this diagnostic to control the car::

  cd /data/openpilot
  python3 tools/car_porting/ka4_stock_scc_probe.py live --duration 45

Analyze an existing full rlog (qlogs commonly omit the evidence needed here)::

  python3 tools/car_porting/ka4_stock_scc_probe.py log /path/to/rlog.zst

Machine-readable output and an offline self-check are available with::

  python3 tools/car_porting/ka4_stock_scc_probe.py --json demo --outcome pass

The report separates a sendcan request from Panda's TX-return echo
(``can.src == physical_bus + 0x80``) and safety rejection echo
(``can.src == physical_bus + 0xC0``). A TX-return proves that Panda accepted
the frame into its transmit path, not that it won arbitration or that an ECU
accepted or acted on it. The probe also records the raw camera-side and
host replacement paths for ADRV_0x161 (including ``ALERTS_5=5``) and
LFAHDA_CLUSTER. A sendcan request by itself is not proof that anything reached
the vehicle bus. A TX-return records Panda's queued transmit attempt, but does
not prove arbitration or ECU acceptance; neither it nor SCC_CONTROL
InfoDisplay acknowledges that the SCC ECU reset its timer.
Exit status is 0 only when the complete correlated observation matches the
requested boundary, 1 for direct FAIL evidence, and 2 for partial evidence.
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import io
import json
import math
import sys
import time
from collections import Counter, defaultdict
from collections.abc import Callable, Iterable
from dataclasses import asdict, dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any


OPENPILOT_ROOT = Path(__file__).resolve().parents[2]
if str(OPENPILOT_ROOT) not in sys.path:
  sys.path.insert(0, str(OPENPILOT_ROOT))


DBC_NAME = "hyundai_canfd_generated"
SCC_CONTROL_ADDRESS = 0x1A0
ADRV_0X161_ADDRESS = 0x161
LFAHDA_CLUSTER_ADDRESS = 0x1E0
CRUISE_BUTTONS_ALT_ADDRESS = 0x1AA
CRUISE_BUTTONS_ADDRESS = 0x1CF

PANDA_RETURNED_BUS_OFFSET = 0x80
PANDA_REJECTED_BUS_OFFSET = 0xC0
PANDA_REJECTED_AND_RETURNED_BUS_OFFSET = PANDA_RETURNED_BUS_OFFSET + PANDA_REJECTED_BUS_OFFSET

BUTTON_NONE = 0
BUTTON_RES_ACCEL = 1
BUTTON_NAMES = {
  0: "NONE",
  1: "RES_ACCEL",
  2: "SET_DECEL",
  3: "GAP_DIST",
  4: "CANCEL",
  5: "LFA_BUTTON",
}

REPORT_SCHEMA_VERSION = 4
TARGET_FINGERPRINT = "KIA_CARNIVAL_4TH_GEN"
# Older cereal logs serialized the human-readable platform value.
TARGET_FINGERPRINT_ALIASES = (TARGET_FINGERPRINT, "KIA CARNIVAL 4TH GEN")
MIN_EXACT_PROOF_DURATION = 30.25
EARLIEST_EXPECTED_FINAL_WARNING = 29.5
LATEST_EXPECTED_FINAL_WARNING = 31.5
TX_MATCH_WINDOW = 0.100
BUTTON_SOURCE_MAX_AGE = 0.025
BUTTON_SOURCE_FUTURE_TOLERANCE = 0.002
BUTTON_SOURCE_CADENCE_TOLERANCE = 0.006
STOP_STREAM_GAP = 0.500
REGULAR_REARM_GROUP_STARTS = (
  (2.50, 5.04, 7.58, 10.12, 12.66, 15.20, 17.74, 20.28, 22.82, 25.36, 26.96),
  (2.51, 5.05, 7.59, 10.13, 12.67, 15.21, 17.75, 20.29, 22.83, 25.37, 26.95),
)
RECOVERY_REARM_GROUP_STARTS = (
  (0.30, 2.84, 5.38, 7.92, 10.46, 13.00, 15.54, 18.08, 20.62, 23.16, 25.70, 26.96),
  (0.31, 2.85, 5.39, 7.93, 10.47, 13.01, 15.55, 18.09, 20.63, 23.17, 25.71, 26.95),
)
SCHEDULE_MODE_REGULAR = "regular"
SCHEDULE_MODE_INITIAL_RECOVERY = "initial_info_display_4_recovery"
SCHEDULE_MODE_MIXED = "mixed_info_display_recovery"
SCHEDULE_MODE_UNKNOWN = "unknown"
INITIAL_RECOVERY_REQUIRED_DURATION = 0.30
INITIAL_RECOVERY_START_TOLERANCE = 0.060
REARM_START_TIMING_TOLERANCE = 0.060
REARM_CADENCE_TOLERANCE = 0.008
FINAL_REARM_FRAME_TIME = 27.00
USE_SWITCH_OR_PEDAL_TO_ACCELERATE = 5
CLUSTER_STREAM_MAX_GAP = 0.250
PROOF_STREAM_MAX_GAP = 0.100
PROOF_STREAM_EDGE_TOLERANCE = 0.100

LIVE_PARAM_KEYS = (
  "CarName",
  "CarSelected3",
  "HyundaiCameraSCC",
  "CanfdHDA2",
  "AlphaLongitudinalEnabled",
  "Ka4StockSccStandstillRearm",
  "AutoEngage",
  "GitBranch",
  "GitCommit",
  "GitCommitDate",
  "GitRemote",
)


def _safe_get(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _finite_float(value: Any, default: float = 0.0) -> float:
  try:
    parsed = float(value)
    return parsed if math.isfinite(parsed) else default
  except (TypeError, ValueError):
    return default


def _decode_param(value: bytes | str | None) -> str | None:
  if value is None:
    return None
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="backslashreplace")
  return str(value)


def _flag_names(enum_type: Any, value: int) -> list[str]:
  return [flag.name for flag in enum_type if flag.value != 0 and value & int(flag.value)]


def classify_can_source(service: str, src: int) -> tuple[str, int]:
  """Return (origin, physical_bus) for a cereal CAN source value."""
  if service == "sendcan":
    return "send_request", src
  if src >= PANDA_REJECTED_AND_RETURNED_BUS_OFFSET:
    return "tx_rejected_returned", src - PANDA_REJECTED_AND_RETURNED_BUS_OFFSET
  if src >= PANDA_REJECTED_BUS_OFFSET:
    return "tx_rejected", src - PANDA_REJECTED_BUS_OFFSET
  if src >= PANDA_RETURNED_BUS_OFFSET:
    return "tx_returned", src - PANDA_RETURNED_BUS_OFFSET
  return "vehicle_rx", src


@dataclass(frozen=True)
class StateSample:
  t: float
  standstill: bool
  cruise_enabled: bool
  cruise_standstill: bool
  v_ego: float
  v_ego_raw: float
  can_valid: bool
  brake_pressed: bool
  gas_pressed: bool
  brake_hold_active: bool
  parking_brake: bool
  acc_faulted: bool

  @property
  def stop_active(self) -> bool:
    return self.standstill and self.cruise_enabled

  @property
  def interlock_active(self) -> bool:
    return any((self.brake_pressed, self.gas_pressed, self.brake_hold_active, self.parking_brake, self.acc_faulted))


@dataclass(frozen=True)
class ControlSample:
  t: float
  enabled: bool
  cancel: bool


@dataclass(frozen=True)
class AlertSample:
  t: float
  enabled: bool
  active: bool
  alert_type: str
  alert_text_1: str
  alert_text_2: str


@dataclass(frozen=True)
class SccSample:
  t: float
  src: int
  bus: int
  data_hex: str
  checksum_valid: bool | None
  counter: int
  info_display: int
  acc_mode: int
  lead_info: int
  lead_distance: float
  lead_relative_speed: float
  sys_fail_state: int
  takeover_request: int

  @property
  def raw_lead_safe(self) -> bool:
    return (
      self.info_display in (0, 4)
      and self.acc_mode in (1, 2)
      and self.sys_fail_state == 0
      and self.takeover_request == 0
      and self.lead_info in (2, 3)
      and 0.0 < self.lead_distance <= 20.0
      and abs(self.lead_relative_speed) <= 0.5
    )


@dataclass(frozen=True)
class ButtonSample:
  t: float
  service: str
  origin: str
  src: int
  bus: int
  address: int
  data_hex: str
  checksum_valid: bool | None
  counter: int
  button: int
  adaptive_main: int
  normal_main: int
  lfa_button: int
  non_button_values: tuple[tuple[str, float | int], ...]


@dataclass(frozen=True)
class ClusterCanSample:
  t: float
  service: str
  origin: str
  src: int
  bus: int
  address: int
  message_name: str
  data_hex: str
  checksum_valid: bool | None
  counter: int
  alert_5: int | None
  hda_control_state: int | None


@dataclass
class StreamStat:
  first_t: float
  last_t: float
  count: int = 0
  checksum_valid: int = 0
  checksum_invalid: int = 0
  checksum_unavailable: int = 0

  def update(self, t: float, checksum_valid: bool | None) -> None:
    self.first_t = min(self.first_t, t)
    self.last_t = max(self.last_t, t)
    self.count += 1
    if checksum_valid is True:
      self.checksum_valid += 1
    elif checksum_valid is False:
      self.checksum_invalid += 1
    else:
      self.checksum_unavailable += 1


class HyundaiCanFdDecoder:
  def __init__(self) -> None:
    from opendbc.can.dbc import DBC

    # The generic DBC loader announces its checksum choice. Keep stdout clean,
    # especially for --json, without changing the shared loader.
    with contextlib.redirect_stdout(io.StringIO()):
      self.dbc = DBC(DBC_NAME)

  def decode(self, message_name: str, data: bytes) -> tuple[dict[str, float | int], bool | None]:
    from opendbc.can.parser import get_raw_value

    message = self.dbc.name_to_msg[message_name]
    if len(data) != message.size:
      raise ValueError(f"{message_name} expects {message.size} bytes, got {len(data)}")

    values: dict[str, float | int] = {}
    checksum_valid: bool | None = None
    for name, signal in message.sigs.items():
      raw = get_raw_value(data, signal)
      if signal.is_signed:
        raw -= ((raw >> (signal.size - 1)) & 1) * (1 << signal.size)
      scaled = raw * signal.factor + signal.offset
      values[name] = int(scaled) if signal.factor == 1.0 and signal.offset == 0.0 else scaled
      if name == "CHECKSUM" and signal.calc_checksum is not None:
        checksum_valid = raw == signal.calc_checksum(message.address, signal, bytearray(data))
    return values, checksum_valid


def summarize_car_params(cp: Any) -> dict[str, Any]:
  from opendbc.car.hyundai.values import HyundaiExtFlags, HyundaiFlags, HyundaiSafetyFlags

  flags = int(_safe_get(cp, "flags", 0))
  ext_flags = int(_safe_get(cp, "extFlags", 0))
  fingerprint = str(_safe_get(cp, "carFingerprint", ""))
  pcm_cruise = bool(_safe_get(cp, "pcmCruise", False))
  openpilot_long = bool(_safe_get(cp, "openpilotLongitudinalControl", False))
  canfd_hda2 = bool(flags & int(HyundaiFlags.CANFD_HDA2))
  safety_configs = []
  safety_canfd_alt_buttons = False
  for cfg in _safe_get(cp, "safetyConfigs", ()) or ():
    model = str(_safe_get(cfg, "safetyModel", ""))
    safety_param = int(_safe_get(cfg, "safetyParam", 0))
    safety_configs.append({"model": model, "param": safety_param})
    safety_canfd_alt_buttons |= (
      model == "hyundaiCanfd"
      and bool(safety_param & int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS))
    )
  gate = {
    "fingerprintIsKa4": fingerprint in TARGET_FINGERPRINT_ALIASES,
    "pcmCruise": pcm_cruise,
    "stockLongitudinal": not openpilot_long,
    "canFd": bool(flags & int(HyundaiFlags.CANFD)),
    "radarScc": bool(flags & int(HyundaiFlags.RADAR_SCC)),
    "notCameraScc": not bool(flags & int(HyundaiFlags.CAMERA_SCC)),
    "canFdAltButtons": bool(flags & int(HyundaiFlags.CANFD_ALT_BUTTONS)),
    "hyundaiCanfdSafetyAltButtons": safety_canfd_alt_buttons,
  }
  return {
    "carFingerprint": fingerprint,
    "brand": str(_safe_get(cp, "brand", "")),
    "pcmCruise": pcm_cruise,
    "openpilotLongitudinalControl": openpilot_long,
    "alphaLongitudinalAvailable": bool(_safe_get(cp, "alphaLongitudinalAvailable", False)),
    "autoResumeSng": bool(_safe_get(cp, "autoResumeSng", False)),
    "canFdHda2": canfd_hda2,
    "flags": flags,
    "flagsHex": f"0x{flags:X}",
    "flagNames": _flag_names(HyundaiFlags, flags),
    "extFlags": ext_flags,
    "extFlagsHex": f"0x{ext_flags:X}",
    "extFlagNames": _flag_names(HyundaiExtFlags, ext_flags),
    "safetyConfigs": safety_configs,
    "pandaBusOffset": 4 * max(0, len(safety_configs) - 1),
    "ka4StockSccGate": gate,
    "ka4StockSccGatePassed": all(gate.values()),
  }


def summarize_fingerprints(raw: str | None) -> dict[str, Any] | None:
  if not raw:
    return None
  try:
    parsed = ast.literal_eval(raw)
    buses = parsed if isinstance(parsed, (list, tuple)) else [parsed]
    result = []
    for bus, fingerprint in enumerate(buses):
      if not isinstance(fingerprint, dict):
        continue
      result.append({
        "bus": bus,
        "addressCount": len(fingerprint),
        "0x1A0": fingerprint.get(SCC_CONTROL_ADDRESS),
        "0x1AA": fingerprint.get(CRUISE_BUTTONS_ALT_ADDRESS),
        "0x1CF": fingerprint.get(CRUISE_BUTTONS_ADDRESS),
      })
    return {"parsed": True, "buses": result}
  except (SyntaxError, ValueError, TypeError) as exc:
    return {"parsed": False, "error": str(exc), "rawLength": len(raw)}


def read_live_params() -> tuple[dict[str, Any], Any | None, str | None]:
  from openpilot.cereal import car, messaging
  from openpilot.common.params import Params

  params = Params()
  result = {}
  param_read_errors = {}
  for key in LIVE_PARAM_KEYS:
    try:
      result[key] = _decode_param(params.get(key))
    except Exception as exc:
      # This is expected when probing an older installed build that predates a
      # diagnostic Param. Record it as evidence rather than crashing.
      result[key] = None
      param_read_errors[key] = type(exc).__name__
  if param_read_errors:
    result["ParamReadErrors"] = param_read_errors
  try:
    fingerprints = _decode_param(params.get("FingerPrints"))
  except Exception as exc:
    fingerprints = None
    result.setdefault("ParamReadErrors", {})["FingerPrints"] = type(exc).__name__
  result["FingerPrintsSummary"] = summarize_fingerprints(fingerprints)

  cp = None
  cp_source = None
  for key in ("CarParams", "CarParamsPersistent", "CarParamsCache"):
    try:
      raw = params.get(key)
    except Exception:
      continue
    if raw is None:
      continue
    try:
      cp = messaging.log_from_bytes(raw, car.CarParams)
      cp_source = key
      break
    except Exception:
      continue
  return result, cp, cp_source


class ProbeAnalyzer:
  def __init__(self, mode: str, source: str) -> None:
    self.mode = mode
    self.source = source
    self.decoder = HyundaiCanFdDecoder()
    self.car_params: dict[str, Any] | None = None
    self.car_params_source: str | None = None
    self.params: dict[str, Any] = {}
    self.states: list[StateSample] = []
    self.controls: list[ControlSample] = []
    self.alerts: list[AlertSample] = []
    self.scc: list[SccSample] = []
    self.buttons: list[ButtonSample] = []
    self.cluster_can: list[ClusterCanSample] = []
    self.decode_errors: Counter[str] = Counter()
    self.streams: dict[tuple[str, str, int, int, int], StreamStat] = {}
    self.first_t = math.inf
    self.last_t = -math.inf

  def _touch(self, t: float) -> None:
    self.first_t = min(self.first_t, t)
    self.last_t = max(self.last_t, t)

  def set_car_params(self, cp: Any, source: str) -> None:
    self.car_params = summarize_car_params(cp)
    self.car_params_source = source

  def feed_can(self, service: str, t: float, src: int, address: int, data: bytes) -> None:
    self._touch(t)
    origin, bus = classify_can_source(service, src)
    message_name = None
    if address == SCC_CONTROL_ADDRESS:
      message_name = "SCC_CONTROL"
    elif address == ADRV_0X161_ADDRESS:
      message_name = "ADRV_0x161"
    elif address == LFAHDA_CLUSTER_ADDRESS:
      message_name = "LFAHDA_CLUSTER"
    elif address == CRUISE_BUTTONS_ALT_ADDRESS:
      message_name = "CRUISE_BUTTONS_ALT"
    elif address == CRUISE_BUTTONS_ADDRESS:
      message_name = "CRUISE_BUTTONS"
    else:
      return

    try:
      values, checksum_valid = self.decoder.decode(message_name, data)
    except (KeyError, ValueError, IndexError) as exc:
      self.decode_errors[f"{service}:{origin}:0x{address:X}:{exc}"] += 1
      checksum_valid = None
      values = {}

    key = (service, origin, src, bus, address)
    if key not in self.streams:
      self.streams[key] = StreamStat(t, t)
    self.streams[key].update(t, checksum_valid)

    if not values:
      return
    if address in (ADRV_0X161_ADDRESS, LFAHDA_CLUSTER_ADDRESS):
      self.cluster_can.append(ClusterCanSample(
        t=t,
        service=service,
        origin=origin,
        src=src,
        bus=bus,
        address=address,
        message_name=message_name,
        data_hex=data.hex(),
        checksum_valid=checksum_valid,
        counter=int(values.get("COUNTER", 0)),
        alert_5=int(values["ALERTS_5"]) if "ALERTS_5" in values else None,
        hda_control_state=int(values["HDA_CntrlModSta"]) if "HDA_CntrlModSta" in values else None,
      ))
      return
    if address == SCC_CONTROL_ADDRESS:
      # Only physical ingress is an authoritative stock SCC status. Keep TX
      # copies in stream counts, but never mistake them for an ECU response.
      if service == "can" and origin == "vehicle_rx":
        self.scc.append(SccSample(
          t=t,
          src=src,
          bus=bus,
          data_hex=data.hex(),
          checksum_valid=checksum_valid,
          counter=int(values.get("COUNTER", 0)),
          info_display=int(values.get("InfoDisplay", 0)),
          acc_mode=int(values.get("ACCMode", 0)),
          lead_info=int(values.get("HUD_LEAD_INFO", 0)),
          lead_distance=_finite_float(values.get("ACC_ObjDist")),
          lead_relative_speed=_finite_float(values.get("ACC_ObjRelSpd")),
          sys_fail_state=int(values.get("SysFailState", 0)),
          takeover_request=int(values.get("TakeOverReq", 0)),
        ))
      return

    self.buttons.append(ButtonSample(
      t=t,
      service=service,
      origin=origin,
      src=src,
      bus=bus,
      address=address,
      data_hex=data.hex(),
      checksum_valid=checksum_valid,
      counter=int(values.get("COUNTER", 0)),
      button=int(values.get("CRUISE_BUTTONS", 0)),
      adaptive_main=int(values.get("ADAPTIVE_CRUISE_MAIN_BTN", 0)),
      normal_main=int(values.get("NORMAL_CRUISE_MAIN_BTN", 0)),
      lfa_button=int(values.get("LFA_BTN", 0)),
      non_button_values=tuple(sorted(
        (name, value) for name, value in values.items()
        if name not in ("CHECKSUM", "_CHECKSUM", "COUNTER", "CRUISE_BUTTONS")
      )),
    ))

  def feed_state(self, t: float, state: Any) -> None:
    self._touch(t)
    cruise = _safe_get(state, "cruiseState", SimpleNamespace())
    v_ego = _finite_float(_safe_get(state, "vEgo", 0.0))
    self.states.append(StateSample(
      t=t,
      standstill=bool(_safe_get(state, "standstill", False)),
      cruise_enabled=bool(_safe_get(cruise, "enabled", False)),
      cruise_standstill=bool(_safe_get(cruise, "standstill", False)),
      v_ego=v_ego,
      v_ego_raw=_finite_float(_safe_get(state, "vEgoRaw", v_ego)),
      can_valid=bool(_safe_get(state, "canValid", False)),
      brake_pressed=bool(_safe_get(state, "brakePressed", False)),
      gas_pressed=bool(_safe_get(state, "gasPressed", False)),
      brake_hold_active=bool(_safe_get(state, "brakeHoldActive", False)),
      parking_brake=bool(_safe_get(state, "parkingBrake", False)),
      acc_faulted=bool(_safe_get(state, "accFaulted", False)),
    ))

  def feed_control(self, t: float, control: Any) -> None:
    self._touch(t)
    cruise_control = _safe_get(control, "cruiseControl", SimpleNamespace())
    self.controls.append(ControlSample(
      t=t,
      enabled=bool(_safe_get(control, "enabled", False)),
      cancel=bool(_safe_get(cruise_control, "cancel", False)),
    ))

  def feed_selfdrive(self, t: float, state: Any) -> None:
    self._touch(t)
    self.alerts.append(AlertSample(
      t=t,
      enabled=bool(_safe_get(state, "enabled", False)),
      active=bool(_safe_get(state, "active", False)),
      alert_type=str(_safe_get(state, "alertType", "")),
      alert_text_1=str(_safe_get(state, "alertText1", "")),
      alert_text_2=str(_safe_get(state, "alertText2", "")),
    ))

  def feed_cereal_event(self, event: Any) -> None:
    t = int(_safe_get(event, "logMonoTime", 0)) * 1e-9
    try:
      which = event.which()
    except Exception:
      return
    if which in ("can", "sendcan"):
      for frame in _safe_get(event, which, ()):
        self.feed_can(which, t, int(frame.src), int(frame.address), bytes(frame.dat))
    elif which == "carState":
      self.feed_state(t, event.carState)
    elif which == "carControl":
      self.feed_control(t, event.carControl)
    elif which == "selfdriveState":
      self.feed_selfdrive(t, event.selfdriveState)
    elif which == "carParams":
      self.set_car_params(event.carParams, "rlog:carParams")

  def _stream_report(self) -> list[dict[str, Any]]:
    result = []
    for (service, origin, src, bus, address), stat in sorted(self.streams.items()):
      duration = stat.last_t - stat.first_t
      result.append({
        "service": service,
        "origin": origin,
        "src": src,
        "srcHex": f"0x{src:X}",
        "physicalBus": bus,
        "address": address,
        "addressHex": f"0x{address:X}",
        "count": stat.count,
        "first": self._rel(stat.first_t),
        "last": self._rel(stat.last_t),
        "observedHz": stat.count / duration if duration > 0 else None,
        "checksumValid": stat.checksum_valid,
        "checksumInvalid": stat.checksum_invalid,
        "checksumUnavailable": stat.checksum_unavailable,
      })
    return result

  def _stock_rx_bus(self) -> int | None:
    button_counts = Counter(
      sample.bus for sample in self.buttons
      if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
    )
    if button_counts:
      return max(button_counts, key=lambda bus: (button_counts[bus], -bus))
    scc_counts = Counter(sample.bus for sample in self.scc)
    return max(scc_counts, key=lambda bus: (scc_counts[bus], -bus)) if scc_counts else None

  def _scc_report(self) -> dict[str, Any]:
    result: dict[str, Any] = {
      "preferredStockBus": self._stock_rx_bus(),
      "buses": [],
    }
    grouped: dict[int, list[SccSample]] = defaultdict(list)
    for sample in self.scc:
      grouped[sample.bus].append(sample)

    for bus, samples in sorted(grouped.items()):
      samples.sort(key=lambda sample: sample.t)
      deltas = [(curr.counter - prev.counter) & 0xFF for prev, curr in zip(samples, samples[1:], strict=False)]
      duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0
      transitions = []
      previous = None
      for sample in samples:
        if sample.info_display != previous:
          transitions.append({"t": self._rel(sample.t), "value": sample.info_display})
          previous = sample.info_display
      result["buses"].append({
        "bus": bus,
        "count": len(samples),
        "first": self._rel(samples[0].t),
        "last": self._rel(samples[-1].t),
        "observedHz": len(samples) / duration if duration > 0 else None,
        "infoDisplayCounts": {str(value): count for value, count in sorted(Counter(
          sample.info_display for sample in samples).items())},
        "infoDisplayTransitions": transitions[:100],
        "accModeCounts": {str(value): count for value, count in sorted(Counter(
          sample.acc_mode for sample in samples).items())},
        "rawLeadSafeSamples": sum(sample.raw_lead_safe for sample in samples),
        "counter": {
          "first": samples[0].counter,
          "last": samples[-1].counter,
          "incrementByOne": sum(delta == 1 for delta in deltas),
          "duplicates": sum(delta == 0 for delta in deltas),
          "jumps": sum(delta not in (0, 1) for delta in deltas),
        },
        "firstDecoded": {
          "InfoDisplay": samples[0].info_display,
          "ACCMode": samples[0].acc_mode,
          "HUD_LEAD_INFO": samples[0].lead_info,
          "ACC_ObjDist": samples[0].lead_distance,
          "ACC_ObjRelSpd": samples[0].lead_relative_speed,
          "SysFailState": samples[0].sys_fail_state,
          "TakeOverReq": samples[0].takeover_request,
          "COUNTER": samples[0].counter,
        },
        "lastDecoded": {
          "InfoDisplay": samples[-1].info_display,
          "ACCMode": samples[-1].acc_mode,
          "HUD_LEAD_INFO": samples[-1].lead_info,
          "ACC_ObjDist": samples[-1].lead_distance,
          "ACC_ObjRelSpd": samples[-1].lead_relative_speed,
          "SysFailState": samples[-1].sys_fail_state,
          "TakeOverReq": samples[-1].takeover_request,
          "COUNTER": samples[-1].counter,
        },
      })
    return result

  def _button_report(self) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str, int, int], list[ButtonSample]] = defaultdict(list)
    for sample in self.buttons:
      grouped[(sample.service, sample.origin, sample.bus, sample.address)].append(sample)

    result = []
    for (service, origin, bus, address), samples in sorted(grouped.items()):
      samples.sort(key=lambda sample: sample.t)
      duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0
      button_counts = Counter(BUTTON_NAMES.get(sample.button, str(sample.button)) for sample in samples)
      result.append({
        "service": service,
        "origin": origin,
        "bus": bus,
        "addressHex": f"0x{address:X}",
        "count": len(samples),
        "first": self._rel(samples[0].t),
        "last": self._rel(samples[-1].t),
        "observedHz": len(samples) / duration if duration > 0 else None,
        "buttonCounts": dict(sorted(button_counts.items())),
        "firstCounter": samples[0].counter,
        "lastCounter": samples[-1].counter,
        "adaptiveMainNonzero": sum(bool(sample.adaptive_main) for sample in samples),
        "normalMainNonzero": sum(bool(sample.normal_main) for sample in samples),
        "lfaButtonNonzero": sum(bool(sample.lfa_button) for sample in samples),
      })
    return result

  def _cluster_tx_matches(self) -> tuple[list[dict[str, Any]], dict[int, str]]:
    requests = sorted(
      (sample for sample in self.cluster_can if sample.origin == "send_request"),
      key=lambda sample: sample.t,
    )
    echoes = sorted(
      (sample for sample in self.cluster_can if sample.origin.startswith("tx_")),
      key=lambda sample: sample.t,
    )
    echo_buckets: dict[tuple[int, int, str], list[tuple[int, ClusterCanSample]]] = defaultdict(list)
    for index, echo in enumerate(echoes):
      echo_buckets[(echo.address, echo.bus, echo.data_hex)].append((index, echo))

    used_echoes: set[int] = set()
    status_by_id: dict[int, str] = {}
    matches: list[dict[str, Any]] = []
    for request in requests:
      candidates = []
      for index, echo in echo_buckets[(request.address, request.bus, request.data_hex)]:
        if index in used_echoes:
          continue
        delta = echo.t - request.t
        if abs(delta) <= TX_MATCH_WINDOW:
          candidates.append((abs(delta), index, echo, delta))
      if not candidates:
        status_by_id[id(request)] = "unobserved"
        matches.append({
          "requestTime": self._rel(request.t),
          "message": request.message_name,
          "addressHex": f"0x{request.address:X}",
          "bus": request.bus,
          "counter": request.counter,
          "ALERTS_5": request.alert_5,
          "HDA_CntrlModSta": request.hda_control_state,
          "dataHex": request.data_hex,
          "status": "unobserved",
        })
        continue

      _, index, echo, delta = min(candidates, key=lambda item: item[0])
      used_echoes.add(index)
      status = "returned" if echo.origin == "tx_returned" else "rejected"
      status_by_id[id(request)] = status
      matches.append({
        "requestTime": self._rel(request.t),
        "echoTime": self._rel(echo.t),
        "echoDeltaMs": round(delta * 1000.0, 3),
        "message": request.message_name,
        "addressHex": f"0x{request.address:X}",
        "bus": request.bus,
        "counter": request.counter,
        "ALERTS_5": request.alert_5,
        "HDA_CntrlModSta": request.hda_control_state,
        "dataHex": request.data_hex,
        "status": status,
        "echoOrigin": echo.origin,
        "echoSrcHex": f"0x{echo.src:X}",
      })
    return matches, status_by_id

  def _cluster_report(self, tx_matches: list[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[tuple[str, str, int, int], list[ClusterCanSample]] = defaultdict(list)
    for sample in self.cluster_can:
      grouped[(sample.service, sample.origin, sample.bus, sample.address)].append(sample)

    streams = []
    for (service, origin, bus, address), samples in sorted(grouped.items()):
      samples.sort(key=lambda sample: sample.t)
      duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0
      attr = "alert_5" if address == ADRV_0X161_ADDRESS else "hda_control_state"
      signal_name = "ALERTS_5" if address == ADRV_0X161_ADDRESS else "HDA_CntrlModSta"
      transitions = []
      previous: int | None | object = object()
      for sample in samples:
        value = getattr(sample, attr)
        if value != previous:
          transitions.append({
            "t": self._rel(sample.t),
            "value": value,
            "counter": sample.counter,
            "dataHex": sample.data_hex,
          })
          previous = value
      checksum_counts = Counter(sample.checksum_valid for sample in samples)
      streams.append({
        "service": service,
        "origin": origin,
        "bus": bus,
        "message": samples[0].message_name,
        "addressHex": f"0x{address:X}",
        "count": len(samples),
        "first": self._rel(samples[0].t),
        "last": self._rel(samples[-1].t),
        "observedHz": len(samples) / duration if duration > 0 else None,
        "checksumValid": checksum_counts[True],
        "checksumInvalid": checksum_counts[False],
        "checksumUnavailable": checksum_counts[None],
        "signal": signal_name,
        "valueCounts": {str(value): count for value, count in sorted(Counter(
          getattr(sample, attr) for sample in samples).items(), key=lambda item: str(item[0]))},
        "transitions": transitions[:100],
      })

    panda_bus_offset = int(self.car_params.get("pandaBusOffset", 0)) if self.car_params else 0
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    expected_raw_camera_bus = panda_bus_offset + 2
    expected_host_bus = None if canfd_hda2 else panda_bus_offset
    return {
      "topology": {
        "pandaBusOffset": panda_bus_offset,
        "canFdHda2": canfd_hda2,
        "expectedRawCameraBus": expected_raw_camera_bus,
        "expectedHostReplacementBus": expected_host_bus,
        "stockLongBehavior": (
          "HDA2 stock-long leaves raw camera ADRV/LFAHDA traffic on the unmodified forwarding path"
          if canfd_hda2 else
          "HDA1 stock-long replaces ADRV/LFAHDA on ECAN while preserving stock-owned ALERTS_5; masking is a FAIL"
        ),
        "inferenceWarning": (
          "Bus topology describes the current branch; a vehicle_rx frame alone is not proof of cluster display."
        ),
      },
      "streams": streams,
      "txMatchCounts": dict(sorted(Counter(match["status"] for match in tx_matches).items())),
      "txMatches": tx_matches,
    }

  def _rel(self, t: float) -> float:
    return round(t - self.first_t, 6) if math.isfinite(self.first_t) else 0.0

  def _match_button_tx(self) -> tuple[list[dict[str, Any]], dict[int, str]]:
    requests = [sample for sample in self.buttons if sample.origin == "send_request"]
    echoes = [sample for sample in self.buttons if sample.origin.startswith("tx_")]
    used_echoes: set[int] = set()
    status_by_id: dict[int, str] = {}
    matches: list[dict[str, Any]] = []

    for request in requests:
      candidates = []
      for index, echo in enumerate(echoes):
        if index in used_echoes:
          continue
        if (echo.address, echo.bus, echo.data_hex) != (request.address, request.bus, request.data_hex):
          continue
        delta = echo.t - request.t
        if abs(delta) <= TX_MATCH_WINDOW:
          candidates.append((abs(delta), index, echo, delta))
      if not candidates:
        status_by_id[id(request)] = "unobserved"
        matches.append({
          "requestTime": self._rel(request.t),
          "addressHex": f"0x{request.address:X}",
          "bus": request.bus,
          "counter": request.counter,
          "button": BUTTON_NAMES.get(request.button, str(request.button)),
          "status": "unobserved",
        })
        continue

      _, index, echo, delta = min(candidates, key=lambda item: item[0])
      used_echoes.add(index)
      status = "returned" if echo.origin == "tx_returned" else "rejected"
      status_by_id[id(request)] = status
      matches.append({
        "requestTime": self._rel(request.t),
        "echoTime": self._rel(echo.t),
        "echoDeltaMs": round(delta * 1000.0, 3),
        "addressHex": f"0x{request.address:X}",
        "bus": request.bus,
        "counter": request.counter,
        "button": BUTTON_NAMES.get(request.button, str(request.button)),
        "status": status,
        "echoSrcHex": f"0x{echo.src:X}",
      })
    return matches, status_by_id

  def _res_groups(self, status_by_id: dict[int, str]) -> list[dict[str, Any]]:
    requests = sorted(
      (sample for sample in self.buttons if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL),
      key=lambda sample: sample.t,
    )
    groups: list[list[ButtonSample]] = []
    for request in requests:
      if not groups or request.t - groups[-1][-1].t > 0.100:
        groups.append([])
      groups[-1].append(request)

    result = []
    stock_bus = self._stock_rx_bus()
    stock = sorted(
      (sample for sample in self.buttons
       if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
       and (stock_bus is None or sample.bus == stock_bus)),
      key=lambda sample: sample.t,
    )
    for group in groups:
      statuses = Counter(status_by_id.get(id(sample), "unobserved") for sample in group)
      previous_stock = [sample for sample in stock if 0 <= group[0].t - sample.t <= 0.100]
      oem_counter = previous_stock[-1].counter if previous_stock else None
      counters = [sample.counter for sample in group]
      source_plus_one = []
      latest_stock_preserved = []
      source_counters = []
      source_times = []
      source_age_ms = []
      source_indices = []
      used_source_indices: set[int] = set()
      for request in group:
        # CAN and sendcan batches representing the same 50 Hz source tick can
        # differ by sub-millisecond publication/float rounding. A source up to
        # 2 ms later is still the same-tick source, not a future counter.
        candidates = [
          (abs(request.t - sample.t), index, sample, request.t - sample.t)
          for index, sample in enumerate(stock)
          if index not in used_source_indices
          and -BUTTON_SOURCE_FUTURE_TOLERANCE <= request.t - sample.t <= BUTTON_SOURCE_MAX_AGE
        ]
        if candidates:
          _, source_index, source, source_age = min(candidates, key=lambda candidate: candidate[0])
          used_source_indices.add(source_index)
        else:
          source = None
          source_index = None
          source_age = None
        source_indices.append(source_index)
        source_counters.append(source.counter if source is not None else None)
        source_times.append(self._rel(source.t) if source is not None else None)
        source_age_ms.append(round(source_age * 1000.0, 3) if source_age is not None else None)
        source_plus_one.append(source is not None and request.counter == (source.counter + 1) & 0xFF)
        latest_stock_preserved.append(
          source is not None and request.non_button_values == source.non_button_values
        )
      expected_counters = [
        None if source_counter is None else (source_counter + 1) & 0xFF
        for source_counter in source_counters
      ]
      source_counter_deltas = [
        None if prev is None or curr is None else (curr - prev) & 0xFF
        for prev, curr in zip(source_counters, source_counters[1:], strict=False)
      ]
      source_timestamp_deltas_ms = [
        None if prev is None or curr is None else round((curr - prev) * 1000.0, 3)
        for prev, curr in zip(source_times, source_times[1:], strict=False)
      ]
      emitted_counter_deltas = [
        (curr - prev) & 0xFF
        for prev, curr in zip(counters, counters[1:], strict=False)
      ]
      source_counters_fresh_sequential = (
        all(counter is not None for counter in source_counters)
        and all(delta == 1 for delta in source_counter_deltas)
      )
      source_timestamps_fresh_sequential = (
        all(source_time is not None for source_time in source_times)
        and all(
          abs(delta - 20.0) <= BUTTON_SOURCE_CADENCE_TOLERANCE * 1000.0
          for delta in source_timestamp_deltas_ms
        )
      )
      source_sequence_fully_observed = (
        all(counter is not None for counter in source_counters)
        and source_timestamps_fresh_sequential
      )
      emitted_counters_fresh_sequential = all(delta == 1 for delta in emitted_counter_deltas)
      post_burst_candidates = []
      post_burst_sequence_observed = False
      if counters and source_indices and source_indices[-1] is not None:
        final_source = stock[source_indices[-1]]
        candidates = stock[source_indices[-1] + 1:source_indices[-1] + 3]
        post_burst_candidates = [{
          "time": self._rel(sample.t),
          "afterFinalSourceMs": round((sample.t - final_source.t) * 1000.0, 3),
          "counter": sample.counter,
          "button": BUTTON_NAMES.get(sample.button, str(sample.button)),
          "checksumValid": sample.checksum_valid,
        } for sample in candidates]
        if len(candidates) == 2:
          same_counter_raw, next_counter_raw = candidates
          post_burst_sequence_observed = (
            same_counter_raw.counter == counters[-1]
            and next_counter_raw.counter == ((counters[-1] + 1) & 0xFF)
            and abs((same_counter_raw.t - final_source.t) - 0.020) <= BUTTON_SOURCE_CADENCE_TOLERANCE
            and abs((next_counter_raw.t - same_counter_raw.t) - 0.020) <= BUTTON_SOURCE_CADENCE_TOLERANCE
            and same_counter_raw.button == BUTTON_NONE
            and next_counter_raw.button == BUTTON_NONE
            and same_counter_raw.checksum_valid is True
            and next_counter_raw.checksum_valid is True
          )
      result.append({
        "start": self._rel(group[0].t),
        "end": self._rel(group[-1].t),
        "addressHex": f"0x{group[0].address:X}",
        "bus": group[0].bus,
        "frameCount": len(group),
        "counters": counters,
        "precedingOemCounter": oem_counter,
        "sourceTimesPerFrame": source_times,
        "sourceAgeMsPerFrame": source_age_ms,
        "sourceTimestampDeltasMs": source_timestamp_deltas_ms,
        "sourceCountersPerFrame": source_counters,
        "sourceCounterDeltasMod256": source_counter_deltas,
        "sourceAvailableAllFrames": all(counter is not None for counter in source_counters),
        "sourceTimestampsFreshSequential": source_timestamps_fresh_sequential,
        "sourceSequenceFullyObserved": source_sequence_fully_observed,
        "sourceCountersFreshSequential": source_counters_fresh_sequential,
        "expectedCurrentControllerCounters": expected_counters,
        "counterPatternMatchesCurrentController": (
          all(counter is not None for counter in source_counters) and counters == expected_counters
        ),
        "emittedCounterDeltasMod256": emitted_counter_deltas,
        "emittedCountersFreshSequential": emitted_counters_fresh_sequential,
        "postBurstRawCandidates": post_burst_candidates,
        "postBurstSameCounterThenNextCounterObserved": post_burst_sequence_observed,
        "postBurstObservationNote": (
          "These are host-observed raw source candidates only; the log does not expose whether Panda " +
          "suppressed the same-counter frame or forwarded the next-counter release frame to the camera bus."
        ),
        "sourcePlusOneAllFrames": all(source_plus_one),
        "latestStockNonButtonFieldsPreserved": all(latest_stock_preserved),
        "allRequestChecksumsValid": all(sample.checksum_valid is True for sample in group),
        "frameSpacingMs": [round((curr.t - prev.t) * 1000.0, 3)
                           for prev, curr in zip(group, group[1:], strict=False)],
        "statuses": dict(statuses),
        "allReturned": statuses.get("returned", 0) == len(group),
      })
    return result

  def _raw_button_counter_report(self) -> list[dict[str, Any]]:
    groups: dict[tuple[int, int], list[ButtonSample]] = defaultdict(list)
    for sample in self.buttons:
      if sample.origin == "vehicle_rx":
        groups[(sample.address, sample.bus)].append(sample)

    result = []
    for (address, bus), samples in sorted(groups.items()):
      samples.sort(key=lambda sample: sample.t)
      modulus = 256 if address == CRUISE_BUTTONS_ALT_ADDRESS else 16
      deltas = [(curr.counter - prev.counter) % modulus for prev, curr in zip(samples, samples[1:], strict=False)]
      duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0
      result.append({
        "addressHex": f"0x{address:X}",
        "bus": bus,
        "count": len(samples),
        "observedHz": len(samples) / duration if duration > 0 else None,
        "incrementByOne": sum(delta == 1 for delta in deltas),
        "duplicates": sum(delta == 0 for delta in deltas),
        "jumps": sum(delta not in (0, 1) for delta in deltas),
        "firstCounter": samples[0].counter,
        "lastCounter": samples[-1].counter,
        "nonNoneButtons": sum(sample.button != BUTTON_NONE for sample in samples),
      })
    return result

  def _stop_episodes(self) -> list[tuple[float, float, bool]]:
    states = sorted(self.states, key=lambda sample: sample.t)
    episodes: list[tuple[float, float, bool]] = []
    current_start: float | None = None
    current_last: float | None = None
    start_observed = False
    previous: StateSample | None = None

    for sample in states:
      if current_start is not None and current_last is not None and sample.t - current_last > STOP_STREAM_GAP:
        episodes.append((current_start, current_last, start_observed))
        current_start = current_last = None
        start_observed = False
      if sample.stop_active:
        if current_start is None:
          current_start = sample.t
          start_observed = previous is not None and not previous.stop_active and sample.t - previous.t <= STOP_STREAM_GAP
        current_last = sample.t
      elif current_start is not None and current_last is not None:
        episodes.append((current_start, current_last, start_observed))
        current_start = current_last = None
        start_observed = False
      previous = sample
    if current_start is not None and current_last is not None:
      episodes.append((current_start, current_last, start_observed))
    return episodes

  def _state_report(self) -> dict[str, Any]:
    states = sorted(self.states, key=lambda sample: sample.t)

    def max_continuous_duration(predicate: Callable[[StateSample], bool]) -> float:
      current_start: float | None = None
      current_last: float | None = None
      maximum = 0.0
      for sample in states:
        if predicate(sample):
          if current_start is None or (current_last is not None and sample.t - current_last > STOP_STREAM_GAP):
            current_start = sample.t
          current_last = sample.t
          maximum = max(maximum, sample.t - current_start)
        else:
          current_start = current_last = None
      return round(maximum, 3)

    transitions = []
    previous: tuple[bool, ...] | None = None
    for sample in states:
      gate_state = (
        sample.standstill,
        sample.cruise_enabled,
        sample.can_valid,
        sample.brake_pressed,
        sample.gas_pressed,
        sample.brake_hold_active,
        sample.parking_brake,
        sample.acc_faulted,
      )
      if gate_state != previous:
        transitions.append({
          "t": self._rel(sample.t),
          "standstill": sample.standstill,
          "cruiseEnabled": sample.cruise_enabled,
          "cruiseStandstill": sample.cruise_standstill,
          "vEgo": sample.v_ego,
          "vEgoRaw": sample.v_ego_raw,
          "canValid": sample.can_valid,
          "brakePressed": sample.brake_pressed,
          "gasPressed": sample.gas_pressed,
          "brakeHoldActive": sample.brake_hold_active,
          "parkingBrake": sample.parking_brake,
          "accFaulted": sample.acc_faulted,
          "eligibleStop": sample.stop_active,
        })
        previous = gate_state

    return {
      "sampleCount": len(states),
      "standstillSamples": sum(sample.standstill for sample in states),
      "cruiseEnabledSamples": sum(sample.cruise_enabled for sample in states),
      "eligibleStopSamples": sum(sample.stop_active for sample in states),
      "canInvalidSamples": sum(not sample.can_valid for sample in states),
      "brakePressedSamples": sum(sample.brake_pressed for sample in states),
      "gasPressedSamples": sum(sample.gas_pressed for sample in states),
      "brakeHoldActiveSamples": sum(sample.brake_hold_active for sample in states),
      "parkingBrakeSamples": sum(sample.parking_brake for sample in states),
      "accFaultedSamples": sum(sample.acc_faulted for sample in states),
      "minAbsVEgo": min((abs(sample.v_ego) for sample in states), default=None),
      "minAbsVEgoRaw": min((abs(sample.v_ego_raw) for sample in states), default=None),
      "maxContinuousStandstill": max_continuous_duration(lambda sample: sample.standstill),
      "maxContinuousCruiseEnabled": max_continuous_duration(lambda sample: sample.cruise_enabled),
      "maxContinuousEligibleStop": max_continuous_duration(lambda sample: sample.stop_active),
      "gateTransitions": transitions[:100],
      "gateTransitionCount": len(transitions),
    }

  def _correlated_timeline(self, button_tx_status_by_id: dict[int, str],
                           cluster_tx_status_by_id: dict[int, str]) -> list[dict[str, Any]]:
    events: list[tuple[float, str, dict[str, Any]]] = []

    previous_stop: bool | None = None
    for sample in sorted(self.states, key=lambda item: item.t):
      if sample.stop_active != previous_stop:
        events.append((sample.t, "physicalStop", {
          "active": sample.stop_active,
          "standstill": sample.standstill,
          "cruiseEnabled": sample.cruise_enabled,
          "vEgo": sample.v_ego,
          "vEgoRaw": sample.v_ego_raw,
        }))
        previous_stop = sample.stop_active

    scc_by_bus: dict[int, list[SccSample]] = defaultdict(list)
    for sample in self.scc:
      scc_by_bus[sample.bus].append(sample)
    for bus, samples in sorted(scc_by_bus.items()):
      previous_info: int | None = None
      for sample in sorted(samples, key=lambda item: item.t):
        if sample.info_display != previous_info:
          events.append((sample.t, "sccInfoDisplay", {
            "bus": bus,
            "value": sample.info_display,
            "counter": sample.counter,
            "dataHex": sample.data_hex,
          }))
          previous_info = sample.info_display

    for sample in sorted(self.buttons, key=lambda item: item.t):
      if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL:
        events.append((sample.t, "resHostRequest", {
          "bus": sample.bus,
          "addressHex": f"0x{sample.address:X}",
          "counter": sample.counter,
          "txStatus": button_tx_status_by_id.get(id(sample), "unobserved"),
          "dataHex": sample.data_hex,
        }))

    cluster_groups: dict[tuple[str, str, int, int], list[ClusterCanSample]] = defaultdict(list)
    for sample in self.cluster_can:
      cluster_groups[(sample.service, sample.origin, sample.bus, sample.address)].append(sample)
    for (service, origin, bus, address), samples in sorted(cluster_groups.items()):
      attr = "alert_5" if address == ADRV_0X161_ADDRESS else "hda_control_state"
      signal = "ALERTS_5" if address == ADRV_0X161_ADDRESS else "HDA_CntrlModSta"
      previous: int | None | object = object()
      for sample in sorted(samples, key=lambda item: item.t):
        value = getattr(sample, attr)
        if value != previous:
          detail = {
            "service": service,
            "origin": origin,
            "bus": bus,
            "message": sample.message_name,
            "addressHex": f"0x{address:X}",
            "signal": signal,
            "value": value,
            "counter": sample.counter,
            "dataHex": sample.data_hex,
          }
          if origin == "send_request":
            detail["txStatus"] = cluster_tx_status_by_id.get(id(sample), "unobserved")
          events.append((sample.t, "clusterCanTransition", detail))
          previous = value
        elif origin == "send_request" and cluster_tx_status_by_id.get(id(sample)) in ("rejected", "unobserved"):
          events.append((sample.t, "clusterCanTxProblem", {
            "service": service,
            "origin": origin,
            "bus": bus,
            "message": sample.message_name,
            "addressHex": f"0x{address:X}",
            "signal": signal,
            "value": value,
            "counter": sample.counter,
            "txStatus": cluster_tx_status_by_id.get(id(sample)),
            "dataHex": sample.data_hex,
          }))

    previous_alert: tuple[str, str, str] | None = None
    for sample in sorted(self.alerts, key=lambda item: item.t):
      value = (sample.alert_type, sample.alert_text_1, sample.alert_text_2)
      if value != previous_alert and any(value):
        events.append((sample.t, "selfdriveAlert", {
          "alertType": sample.alert_type,
          "alertText1": sample.alert_text_1,
          "alertText2": sample.alert_text_2,
        }))
      previous_alert = value

    return [{"t": self._rel(t), "event": kind, **detail}
            for t, kind, detail in sorted(events, key=lambda item: (item[0], item[1]))]

  @staticmethod
  def _nearest_bool(samples: list[Any], t: float, name: str, tolerance: float = 0.150) -> bool | None:
    if not samples:
      return None
    nearest = min(samples, key=lambda sample: abs(sample.t - t))
    if abs(nearest.t - t) > tolerance:
      return None
    return bool(getattr(nearest, name))

  def _warning_periods(self, samples: list[SccSample], start: float) -> list[dict[str, float]]:
    periods: list[tuple[float, float]] = []
    period_start: float | None = None
    last_t: float | None = None
    for sample in samples:
      active = sample.info_display == 4
      if period_start is not None and last_t is not None and sample.t - last_t > 0.150:
        periods.append((period_start, last_t))
        period_start = None
      if active and period_start is None:
        period_start = sample.t
      elif not active and period_start is not None:
        periods.append((period_start, sample.t))
        period_start = None
      last_t = sample.t
    if period_start is not None and last_t is not None:
      periods.append((period_start, last_t))
    return [{
      "startAfterStop": round(period_start - start, 3),
      "endAfterStop": round(period_end - start, 3),
      "duration": round(period_end - period_start, 3),
    } for period_start, period_end in periods]

  def _schedule_mode(self, samples: list[SccSample], start: float) -> tuple[str, list[dict[str, float]]]:
    """Classify the only two schedules implemented by the controller.

    InfoDisplay=4 is an authoritative SCC state input, not proof that the
    cluster showed the driver-action warning. The early recovery schedule is
    supported only when that state is already active at the physical stop and
    remains continuously active through the 0.30 s qualification dwell.
    """
    samples = sorted(samples, key=lambda sample: sample.t)
    periods = self._warning_periods(samples, start)
    if not samples or samples[0].t - start > INITIAL_RECOVERY_START_TOLERANCE:
      return SCHEDULE_MODE_UNKNOWN, periods

    early_periods = [
      period for period in periods
      if period["startAfterStop"] < EARLIEST_EXPECTED_FINAL_WARNING
    ]
    if not early_periods:
      return SCHEDULE_MODE_REGULAR, periods

    first = early_periods[0]
    initial_recovery = (
      len(early_periods) == 1
      and samples[0].info_display == 4
      and first["startAfterStop"] <= INITIAL_RECOVERY_START_TOLERANCE
      and first["endAfterStop"] >= INITIAL_RECOVERY_REQUIRED_DURATION
    )
    return (SCHEDULE_MODE_INITIAL_RECOVERY if initial_recovery else SCHEDULE_MODE_MIXED), periods

  @staticmethod
  def _expected_rearm_schedules(schedule_mode: str) -> tuple[tuple[float, ...], ...]:
    if schedule_mode == SCHEDULE_MODE_REGULAR:
      return REGULAR_REARM_GROUP_STARTS
    if schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY:
      return RECOVERY_REARM_GROUP_STARTS
    return ()

  @staticmethod
  def _cluster_signal_periods(samples: list[ClusterCanSample], start: float, attr: str,
                              active_value: int) -> list[dict[str, float]]:
    periods: list[tuple[float, float]] = []
    period_start: float | None = None
    last_t: float | None = None
    for sample in sorted(samples, key=lambda item: item.t):
      active = getattr(sample, attr) == active_value
      if period_start is not None and last_t is not None and sample.t - last_t > CLUSTER_STREAM_MAX_GAP:
        periods.append((period_start, last_t))
        period_start = None
      if active and period_start is None:
        period_start = sample.t
      elif not active and period_start is not None:
        periods.append((period_start, sample.t))
        period_start = None
      last_t = sample.t
    if period_start is not None and last_t is not None:
      periods.append((period_start, last_t))
    return [{
      "startAfterStop": round(period_start - start, 3),
      "endAfterStop": round(period_end - start, 3),
      "duration": round(period_end - period_start, 3),
    } for period_start, period_end in periods]

  @staticmethod
  def _continuous_coverage(samples: list[Any], start: float, through: float, *,
                           max_gap: float = PROOF_STREAM_MAX_GAP,
                           edge_tolerance: float = PROOF_STREAM_EDGE_TOLERANCE) -> dict[str, Any]:
    times = sorted(
      float(sample.t) for sample in samples
      if start - edge_tolerance <= float(sample.t) <= through + edge_tolerance
    )
    gaps = [curr - prev for prev, curr in zip(times, times[1:], strict=False)]
    start_edge_covered = bool(times) and times[0] <= start + edge_tolerance
    end_edge_covered = bool(times) and times[-1] >= through - edge_tolerance
    gaps_within_limit = bool(times) and all(gap <= max_gap + 1e-9 for gap in gaps)
    continuous = start_edge_covered and end_edge_covered and gaps_within_limit
    return {
      "requiredStartAfterStop": 0.0,
      "requiredEndAfterStop": round(through - start, 3),
      "maxAllowedGapMs": max_gap * 1000.0,
      "edgeToleranceMs": edge_tolerance * 1000.0,
      "sampleCount": len(times),
      "firstSampleAfterStop": round(times[0] - start, 3) if times else None,
      "lastSampleAfterStop": round(times[-1] - start, 3) if times else None,
      "startEdgeCovered": start_edge_covered,
      "endEdgeCovered": end_edge_covered,
      "maxObservedGapMs": round(max(gaps) * 1000.0, 3) if gaps else None,
      "gapsOverLimit": sum(gap > max_gap + 1e-9 for gap in gaps),
      "continuous": continuous,
    }

  @staticmethod
  def _pair_cluster_frames_by_counter(
      sources: list[ClusterCanSample], outputs: list[ClusterCanSample], start: float, through: float,
  ) -> tuple[list[tuple[ClusterCanSample, ClusterCanSample]], dict[str, Any]]:
    """Pair every proof-window source with one same-counter, same-tick output.

    Counters wrap several times during a 30 second capture, so a counter match
    alone is insufficient. Conversely, nearest-time matching without consuming
    the output lets one sparse host frame stand in for several raw frames. Keep
    the time bound and consume each output at most once.
    """
    all_sources = sorted(sources, key=lambda sample: sample.t)
    all_outputs = sorted(outputs, key=lambda sample: sample.t)
    used_outputs: set[int] = set()
    all_pairs: list[tuple[ClusterCanSample, int, ClusterCanSample]] = []
    for source in all_sources:
      candidates = [
        (abs(output.t - source.t), index, output)
        for index, output in enumerate(all_outputs)
        if index not in used_outputs
        and output.counter == source.counter
        and abs(output.t - source.t) <= TX_MATCH_WINDOW
      ]
      if not candidates:
        continue
      _, output_index, output = min(candidates, key=lambda candidate: candidate[0])
      used_outputs.add(output_index)
      all_pairs.append((source, output_index, output))

    proof_sources = [source for source in all_sources if start <= source.t <= through]
    proof_pairs_with_index = [pair for pair in all_pairs if start <= pair[0].t <= through]
    pairs = [(source, output) for source, _, output in proof_pairs_with_index]
    proof_output_indices = {index for _, index, _ in proof_pairs_with_index}
    unpaired_proof_outputs = [
      output for index, output in enumerate(all_outputs)
      if index not in used_outputs
      and (
        start <= output.t <= through
        or any(
          source.counter == output.counter and abs(source.t - output.t) <= TX_MATCH_WINDOW
          for source in proof_sources
        )
      )
    ]
    source_count = len(proof_sources)
    paired_count = len(pairs)
    output_count = len(proof_output_indices) + len(unpaired_proof_outputs)
    unused_output_count = len(unpaired_proof_outputs)
    exact_one_to_one = (
      source_count > 0
      and source_count == output_count == paired_count
      and unused_output_count == 0
    )
    return pairs, {
      "sourceFramesInProofWindow": source_count,
      "outputFramesInProofWindow": output_count,
      "pairedFrames": paired_count,
      "unpairedSourceFrames": source_count - paired_count,
      "unusedOutputFrames": unused_output_count,
      "sourceCoverageRatio": paired_count / source_count if source_count else 0.0,
      "allSourceFramesPairedByCounterAndTime": source_count > 0 and paired_count == source_count,
      "outputsReused": 0,
      "exactOneToOneByCounterAndTime": exact_one_to_one,
    }

  def _cluster_episode_evidence(self, start: float, end: float,
                                tx_status_by_id: dict[int, str]) -> dict[str, Any]:
    panda_bus_offset = int(self.car_params.get("pandaBusOffset", 0)) if self.car_params else 0
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    raw_camera_bus = panda_bus_offset + 2
    host_bus = panda_bus_offset
    samples = [sample for sample in self.cluster_can if start - 0.150 <= sample.t <= end + 0.150]
    raw_adrv = sorted((sample for sample in samples
                       if sample.origin == "vehicle_rx" and sample.address == ADRV_0X161_ADDRESS
                       and sample.bus == raw_camera_bus), key=lambda sample: sample.t)
    raw_hda = sorted((sample for sample in samples
                      if sample.origin == "vehicle_rx" and sample.address == LFAHDA_CLUSTER_ADDRESS
                      and sample.bus == raw_camera_bus), key=lambda sample: sample.t)
    adrv_requests = sorted((sample for sample in samples
                            if sample.origin == "send_request" and sample.address == ADRV_0X161_ADDRESS),
                           key=lambda sample: sample.t)
    hda_requests = sorted((sample for sample in samples
                           if sample.origin == "send_request" and sample.address == LFAHDA_CLUSTER_ADDRESS),
                          key=lambda sample: sample.t)
    returned_adrv = sorted((sample for sample in samples
                            if sample.origin == "tx_returned" and sample.address == ADRV_0X161_ADDRESS),
                           key=lambda sample: sample.t)
    returned_hda = sorted((sample for sample in samples
                           if sample.origin == "tx_returned" and sample.address == LFAHDA_CLUSTER_ADDRESS),
                          key=lambda sample: sample.t)
    raw_cluster_samples = [*raw_adrv, *raw_hda]
    replacement_cluster_samples = [*adrv_requests, *hda_requests, *returned_adrv, *returned_hda]
    cluster_request_samples = [*adrv_requests, *hda_requests]
    invalid_cluster_samples = [sample for sample in (*raw_cluster_samples, *replacement_cluster_samples)
                               if sample.checksum_valid is False]
    rejected_cluster_requests = [request for request in cluster_request_samples
                                 if tx_status_by_id.get(id(request)) == "rejected"]

    coverage_through = min(end, start + MIN_EXACT_PROOF_DURATION)
    host_adrv_requests = [sample for sample in adrv_requests if sample.bus == host_bus]
    host_hda_requests = [sample for sample in hda_requests if sample.bus == host_bus]
    host_returned_adrv = [sample for sample in returned_adrv if sample.bus == host_bus]
    host_returned_hda = [sample for sample in returned_hda if sample.bus == host_bus]
    raw_adrv_coverage = self._continuous_coverage(raw_adrv, start, coverage_through)
    raw_hda_coverage = self._continuous_coverage(raw_hda, start, coverage_through)
    host_stream_coverage = {
      "adrvRequests": self._continuous_coverage(host_adrv_requests, start, coverage_through),
      "adrvReturned": self._continuous_coverage(host_returned_adrv, start, coverage_through),
      "lfaHdaRequests": self._continuous_coverage(host_hda_requests, start, coverage_through),
      "lfaHdaReturned": self._continuous_coverage(host_returned_hda, start, coverage_through),
    }
    _, adrv_request_pairing = self._pair_cluster_frames_by_counter(
      raw_adrv, host_adrv_requests, start, coverage_through,
    )
    _, adrv_return_pairing = self._pair_cluster_frames_by_counter(
      raw_adrv, host_returned_adrv, start, coverage_through,
    )
    _, hda_request_pairing = self._pair_cluster_frames_by_counter(
      raw_hda, host_hda_requests, start, coverage_through,
    )
    hda_return_pairs, hda_return_pairing = self._pair_cluster_frames_by_counter(
      raw_hda, host_returned_hda, start, coverage_through,
    )
    exact_adrv_replacement_pairing = (
      adrv_request_pairing["exactOneToOneByCounterAndTime"]
      and adrv_return_pairing["exactOneToOneByCounterAndTime"]
    )
    exact_hda_replacement_pairing = (
      hda_request_pairing["exactOneToOneByCounterAndTime"]
      and hda_return_pairing["exactOneToOneByCounterAndTime"]
    )

    raw_warning_periods = self._cluster_signal_periods(
      [sample for sample in raw_adrv if start <= sample.t <= end], start, "alert_5",
      USE_SWITCH_OR_PEDAL_TO_ACCELERATE,
    )
    first_raw_warning = raw_warning_periods[0]["startAfterStop"] if raw_warning_periods else None
    raw_warning_frames = [sample for sample in raw_adrv
                          if start <= sample.t <= end
                          and sample.alert_5 == USE_SWITCH_OR_PEDAL_TO_ACCELERATE]
    warning_output_pairs = []
    used_warning_outputs: set[int] = set()
    for raw in raw_warning_frames:
      candidates = [
        (abs(sample.t - raw.t), index, sample)
        for index, sample in enumerate(returned_adrv)
        if index not in used_warning_outputs
        and sample.bus == host_bus
        and sample.counter == raw.counter
        and abs(sample.t - raw.t) <= TX_MATCH_WINDOW
      ]
      if candidates:
        _, index, output = min(candidates, key=lambda candidate: candidate[0])
        used_warning_outputs.add(index)
        warning_output_pairs.append((raw, output))
    masked_warning_frames = sum(output.alert_5 != USE_SWITCH_OR_PEDAL_TO_ACCELERATE
                                for _, output in warning_output_pairs)
    forwarded_warning_frames = sum(output.alert_5 == USE_SWITCH_OR_PEDAL_TO_ACCELERATE
                                   for _, output in warning_output_pairs)
    unpaired_warning_frames = len(raw_warning_frames) - len(warning_output_pairs)
    if canfd_hda2:
      warning_preservation_ok = not adrv_requests and not returned_adrv
      warning_preservation_topology = "hda2_raw_no_host_replacement"
    elif not raw_warning_frames:
      warning_preservation_ok = True
      warning_preservation_topology = "hda1_no_raw_warning_to_preserve"
    else:
      warning_preservation_ok = (
        unpaired_warning_frames == 0
        and masked_warning_frames == 0
        and forwarded_warning_frames == len(raw_warning_frames)
      )
      warning_preservation_topology = "hda1_host_replacement"

    hda_state_mismatches = sum(raw.hda_control_state != output.hda_control_state
                               for raw, output in hda_return_pairs)

    if raw_warning_frames and masked_warning_frames:
      warning_path = "hostReplacementMaskedRawWarning"
    elif raw_warning_frames and forwarded_warning_frames:
      warning_path = "hostReplacementForwardedRawWarning"
    elif raw_warning_frames and canfd_hda2 and not adrv_requests:
      warning_path = "hda2RawUnmodifiedForwardingTopology"
    elif raw_warning_frames:
      warning_path = "rawWarningObservedButDeliveryUnproven"
    else:
      warning_path = "noRawWarningObserved"

    all_adrv_returned = bool(adrv_requests) and all(
      tx_status_by_id.get(id(request)) == "returned" for request in adrv_requests)
    all_hda_returned = bool(hda_requests) and all(
      tx_status_by_id.get(id(request)) == "returned" for request in hda_requests)
    if canfd_hda2:
      hda_path_consistent = (
        raw_adrv_coverage["continuous"]
        and raw_hda_coverage["continuous"]
        and not adrv_requests
        and not hda_requests
        and not returned_adrv
        and not returned_hda
        and warning_preservation_ok
      )
    else:
      hda_path_consistent = (
        raw_adrv_coverage["continuous"]
        and raw_hda_coverage["continuous"]
        and all(coverage["continuous"] for coverage in host_stream_coverage.values())
        and exact_adrv_replacement_pairing
        and exact_hda_replacement_pairing
        and all_adrv_returned
        and all_hda_returned
        and hda_state_mismatches == 0
        and all(request.bus == host_bus for request in (*adrv_requests, *hda_requests))
        and all(sample.bus == host_bus for sample in (*returned_adrv, *returned_hda))
        and warning_preservation_ok
      )

    raw_hda_transitions = []
    previous_hda_state: int | None | object = object()
    for sample in raw_hda:
      if sample.hda_control_state != previous_hda_state:
        raw_hda_transitions.append({
          "t": self._rel(sample.t),
          "afterStop": round(sample.t - start, 3),
          "value": sample.hda_control_state,
          "counter": sample.counter,
          "dataHex": sample.data_hex,
        })
        previous_hda_state = sample.hda_control_state

    res_requests = [sample for sample in self.buttons
                    if start <= sample.t <= end and sample.origin == "send_request"
                    and sample.button == BUTTON_RES_ACCEL]
    last_res_after_stop = round(max(sample.t for sample in res_requests) - start, 3) if res_requests else None
    warning_after_last_res = (
      round(first_raw_warning - last_res_after_stop, 3)
      if first_raw_warning is not None and last_res_after_stop is not None else None
    )
    return {
      "rawCameraBus": raw_camera_bus,
      "hostReplacementBus": None if canfd_hda2 else host_bus,
      "canFdHda2": canfd_hda2,
      "rawAdrvSamples": len(raw_adrv),
      "rawLfaHdaSamples": len(raw_hda),
      "rawAdrvContinuousThrough30_25s": raw_adrv_coverage["continuous"],
      "rawLfaHdaContinuousThrough30_25s": raw_hda_coverage["continuous"],
      "rawAdrvCoverage": raw_adrv_coverage,
      "rawLfaHdaCoverage": raw_hda_coverage,
      "hostReplacementCoverage": host_stream_coverage,
      "hostReplacementPairing": {
        "adrvRawToRequest": adrv_request_pairing,
        "adrvRawToReturned": adrv_return_pairing,
        "lfaHdaRawToRequest": hda_request_pairing,
        "lfaHdaRawToReturned": hda_return_pairing,
        "exactAdrvRawRequestReturnedPairing": exact_adrv_replacement_pairing,
        "exactLfaHdaRawRequestReturnedPairing": exact_hda_replacement_pairing,
      },
      "integrity": {
        "rawAdrvAllCrcValid": bool(raw_adrv) and all(sample.checksum_valid is True for sample in raw_adrv),
        "rawLfaHdaAllCrcValid": bool(raw_hda) and all(sample.checksum_valid is True for sample in raw_hda),
        "hostReplacementAllCrcValid": (
          not replacement_cluster_samples if canfd_hda2 else
          bool(replacement_cluster_samples)
          and all(sample.checksum_valid is True for sample in replacement_cluster_samples)
        ),
        "invalidSampleCount": len(invalid_cluster_samples),
        "rejectedRequestCount": len(rejected_cluster_requests),
      },
      "rawAccelerateWarningPeriods": raw_warning_periods,
      "firstRawAccelerateWarningAfterStop": first_raw_warning,
      "warningPathObservation": warning_path,
      "warningOutputPairCounts": {
        "rawWarningFrames": len(raw_warning_frames),
        "pairedWithTxReturn": len(warning_output_pairs),
        "unpairedRawWarningFrames": unpaired_warning_frames,
        "maskedByReturnedFrame": masked_warning_frames,
        "forwardedByReturnedFrame": forwarded_warning_frames,
      },
      "warningPreservation": {
        "topology": warning_preservation_topology,
        "requiredForObservedRawWarning": bool(raw_warning_frames),
        "allRawWarningFramesPairedByCounterAndTime": unpaired_warning_frames == 0,
        "allPairedReturnedFramesPreserveAlert5": masked_warning_frames == 0,
        "pathPreservesRawWarning": warning_preservation_ok,
      },
      "hdaControlStateCounts": {str(value): count for value, count in sorted(Counter(
        sample.hda_control_state for sample in raw_hda).items(), key=lambda item: str(item[0]))},
      "hdaControlStateTransitions": raw_hda_transitions[:100],
      "hdaReplacementComparison": {
        "pairedFrames": len(hda_return_pairs),
        "stateMismatches": hda_state_mismatches,
        "allAdrvRequestsReturned": all_adrv_returned,
        "allLfaHdaRequestsReturned": all_hda_returned,
        "pathConsistentWithCurrentStockLongTopology": hda_path_consistent,
      },
      "postResObservation": {
        "lastResAfterStop": last_res_after_stop,
        "firstRawWarningAfterLastRes": warning_after_last_res,
        "note": "Temporal ordering is observation only; it is not an SCC timer-reset acknowledgement.",
      },
    }

  def _episode_report(self, start: float, end: float, start_observed: bool,
                      res_groups: list[dict[str, Any]], cluster_tx_status_by_id: dict[int, str]) -> dict[str, Any]:
    states = [sample for sample in self.states if start <= sample.t <= end]
    stock_bus = self._stock_rx_bus()
    scc = [sample for sample in self.scc if start <= sample.t <= end and
           (stock_bus is None or sample.bus == stock_bus)]
    controls = [sample for sample in self.controls if start <= sample.t <= end]
    alerts = [sample for sample in self.alerts if start <= sample.t <= end and
              (sample.alert_type or sample.alert_text_1 or sample.alert_text_2)]
    start_rel = self._rel(start)
    end_rel = self._rel(end)
    episode_groups = [group for group in res_groups if start_rel <= group["start"] <= end_rel]
    duration = end - start

    raw_alt_stock_bus = [sample for sample in self.buttons if start <= sample.t <= end and
                         sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS and
                         sample.bus == stock_bus]
    raw_standard_stock_bus = [sample for sample in self.buttons if start <= sample.t <= end and
                              sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ADDRESS and
                              sample.bus == stock_bus]
    rearm_requests = [sample for sample in self.buttons if start <= sample.t <= end and
                      sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL]

    safe_scc = [sample for sample in scc if sample.raw_lead_safe]
    valid_state = [sample for sample in states if sample.can_valid]
    no_interlock = [sample for sample in states if not sample.interlock_active]
    enabled_controls = [sample for sample in controls if sample.enabled]
    control_cancel_clear = bool(controls) and all(not sample.cancel for sample in controls)
    raw_driver_buttons_clear = bool(raw_alt_stock_bus) and all(
      sample.button == BUTTON_NONE
      and sample.adaptive_main == 0
      and sample.normal_main == 0
      and sample.lfa_button == 0
      for sample in raw_alt_stock_bus
    )
    schedule_mode, info_display_4_periods = self._schedule_mode(scc, start)
    cluster_evidence = self._cluster_episode_evidence(start, end, cluster_tx_status_by_id)
    first_raw_warning = cluster_evidence["firstRawAccelerateWarningAfterStop"]

    group_times = [group["start"] - start_rel for group in episode_groups]
    gaps = [curr - prev for prev, curr in zip(group_times, group_times[1:], strict=False)]
    expected_schedules = self._expected_rearm_schedules(schedule_mode)
    matching_source_phases = [
      phase for phase, expected in enumerate(expected_schedules)
      if len(group_times) == len(expected)
      and abs(group_times[0] - expected[0]) <= REARM_START_TIMING_TOLERANCE
      and all(abs(actual_gap - expected_gap) <= REARM_CADENCE_TOLERANCE
              for actual_gap, expected_gap in zip(
                gaps,
                (curr - prev for prev, curr in zip(expected, expected[1:], strict=False)),
                strict=True,
              ))
    ]
    matched_source_phase = matching_source_phases[0] if len(matching_source_phases) == 1 else None
    exact_group_times = matched_source_phase is not None
    exact_three_frame_groups = bool(episode_groups) and all(
      group["frameCount"] == 3
      and len(group["frameSpacingMs"]) == 2
      and all(abs(spacing - 20.0) <= 6.0 for spacing in group["frameSpacingMs"])
      for group in episode_groups
    )
    no_late_rearm = not any(request.t - start > FINAL_REARM_FRAME_TIME + 0.005 for request in rearm_requests)
    returned_schedule_ok = exact_group_times and exact_three_frame_groups and no_late_rearm
    schedule = {
      "scheduleMode": schedule_mode,
      "firstGroupAfterStop": round(group_times[0], 3) if group_times else None,
      "lastGroupAfterStop": round(group_times[-1], 3) if group_times else None,
      "maxGroupGap": round(max(gaps), 3) if gaps else None,
      "expectedGroupStartVariants": [
        {"buttonSourcePhaseFrames": phase, "groupStarts": list(expected)}
        for phase, expected in enumerate(expected_schedules)
      ],
      "matchingButtonSourcePhaseFrames": matching_source_phases,
      "matchedButtonSourcePhaseFrames": matched_source_phase,
      "startTimingToleranceMs": REARM_START_TIMING_TOLERANCE * 1000.0,
      "cadenceToleranceMs": REARM_CADENCE_TOLERANCE * 1000.0,
      "actualGroupStarts": [round(group_time, 3) for group_time in group_times],
      "exactExpectedGroupTimes": exact_group_times,
      "exactThreeFrameGroups": exact_three_frame_groups,
      "noResAfter27_00s": no_late_rearm,
      "exactSupportedRearmSchedule": returned_schedule_ok,
    }

    coverage_through = min(end, start + MIN_EXACT_PROOF_DURATION)
    stream_coverage = {
      "carState": self._continuous_coverage(states, start, coverage_through),
      "carControl": self._continuous_coverage(controls, start, coverage_through),
      "rawScc0x1a0": self._continuous_coverage(scc, start, coverage_through),
      "rawStockButtons0x1aa": self._continuous_coverage(raw_alt_stock_bus, start, coverage_through),
    }
    gate_passed = bool(self.car_params and self.car_params.get("ka4StockSccGatePassed"))
    state_coverage_ok = bool(states) and len(valid_state) == len(states)
    no_interlocks = (
      bool(states)
      and len(no_interlock) == len(states)
      and control_cancel_clear
      and raw_driver_buttons_clear
    )
    control_coverage_ok = bool(controls) and len(enabled_controls) == len(controls)
    raw_scc_coverage_ok = bool(scc) and len(safe_scc) == len(scc)
    stopped_motion_ok = bool(states) and all(
      sample.standstill
      and abs(sample.v_ego) <= 0.05
      and abs(sample.v_ego_raw) <= 0.03
      for sample in states
    )
    exact_duration = duration >= MIN_EXACT_PROOF_DURATION
    all_tx_returned = bool(episode_groups) and all(group["allReturned"] for group in episode_groups)
    any_rejected = any(group["statuses"].get("rejected", 0) for group in episode_groups)
    all_scc_crc_valid = bool(scc) and all(sample.checksum_valid is True for sample in scc)
    all_raw_button_crc_valid = bool(raw_alt_stock_bus) and all(
      sample.checksum_valid is True for sample in raw_alt_stock_bus
    )
    all_rearm_crc_valid = bool(episode_groups) and all(group["allRequestChecksumsValid"] for group in episode_groups)
    source_plus_one = bool(episode_groups) and all(group["sourcePlusOneAllFrames"] for group in episode_groups)
    source_counters_fresh = bool(episode_groups) and all(
      group["sourceCountersFreshSequential"] for group in episode_groups)
    source_timestamps_fresh = bool(episode_groups) and all(
      group["sourceTimestampsFreshSequential"] for group in episode_groups)
    emitted_counters_fresh = bool(episode_groups) and all(
      group["emittedCountersFreshSequential"] for group in episode_groups)
    post_burst_release_candidates_observed = bool(episode_groups) and all(
      group["postBurstSameCounterThenNextCounterObserved"] for group in episode_groups)
    stock_fields_preserved = bool(episode_groups) and all(
      group["latestStockNonButtonFieldsPreserved"] for group in episode_groups)
    panda_bus_offset = int(self.car_params.get("pandaBusOffset", 0)) if self.car_params else 0
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    expected_send_bus = stock_bus if canfd_hda2 else panda_bus_offset + 2
    send_bus_matches = (
      expected_send_bus is not None
      and bool(rearm_requests)
      and all(request.bus == expected_send_bus for request in rearm_requests)
    )
    same_safety_panda = (
      stock_bus is not None
      and stock_bus // 4 == panda_bus_offset // 4
      and bool(rearm_requests)
      and all(request.bus // 4 == panda_bus_offset // 4 for request in rearm_requests)
    )
    exact_alt_layout = (
      bool(raw_alt_stock_bus)
      and not raw_standard_stock_bus
      and send_bus_matches
      and same_safety_panda
      and all(request.address == CRUISE_BUTTONS_ALT_ADDRESS for request in rearm_requests)
    )

    # Look just beyond the final standstill sample. Capping this window at
    # ``start + 30`` makes it empty for every proof-length episode (whose end
    # is already later than 30 s), silently hiding a false start at release.
    next_states = [sample for sample in self.states if end < sample.t <= end + 0.250]
    last_safe_scc = scc[-1].raw_lead_safe if scc else False
    potential_false_start = any(
      not sample.standstill
      and sample.cruise_enabled
      and max(abs(sample.v_ego), abs(sample.v_ego_raw)) > 0.05
      and not sample.interlock_active
      for sample in next_states
    ) and last_safe_scc
    supported_group_limit = (
      len(expected_schedules[0]) if expected_schedules else
      max(len(schedule) for schedule in (*REGULAR_REARM_GROUP_STARTS, *RECOVERY_REARM_GROUP_STARTS))
    )
    direct_schedule_violation = (
      len(episode_groups) > supported_group_limit
      or len(rearm_requests) > supported_group_limit * 3
      or any(request.t - start > FINAL_REARM_FRAME_TIME + 0.005 for request in rearm_requests)
      or any(group["frameCount"] > 3 for group in episode_groups)
    )
    direct_crc_failure = (
      any(sample.checksum_valid is False for sample in scc)
      or any(sample.checksum_valid is False for sample in raw_alt_stock_bus)
      or any(not group["allRequestChecksumsValid"] for group in episode_groups)
      or cluster_evidence["integrity"]["invalidSampleCount"] > 0
    )
    any_cluster_rejected = cluster_evidence["integrity"]["rejectedRequestCount"] > 0
    direct_warning_masking = (
      not canfd_hda2
      and cluster_evidence["warningOutputPairCounts"]["maskedByReturnedFrame"] > 0
    )
    direct_source_policy_failure = bool(episode_groups) and any(
      group["sourceSequenceFullyObserved"]
      and (
        not group["sourcePlusOneAllFrames"]
        or not group["sourceCountersFreshSequential"]
        or not group["emittedCountersFreshSequential"]
        or not group["latestStockNonButtonFieldsPreserved"]
      )
      for group in episode_groups
    )

    prerequisites = {
      "ka4StockSccCarParamsGate": gate_passed,
      "physicalStopStartObserved": start_observed,
      "durationAtLeast30_25s": exact_duration,
      "carStateContinuousCoverage": stream_coverage["carState"]["continuous"],
      "carControlContinuousCoverage": stream_coverage["carControl"]["continuous"],
      "rawScc0x1a0ContinuousCoverage": stream_coverage["rawScc0x1a0"]["continuous"],
      "rawStockButtons0x1aaContinuousCoverage": stream_coverage["rawStockButtons0x1aa"]["continuous"],
      "carStateCanValidAllSamples": state_coverage_ok,
      "driverInterlocksClearAllSamples": no_interlocks,
      "carControlEnabledAllSamples": control_coverage_ok,
      "wheelStandstillAndRawSpeedNearZeroAllSamples": stopped_motion_ok,
      "noPotentialFalseStart": not potential_false_start,
      "rawSccLeadGateValidAllSamples": raw_scc_coverage_ok,
      "raw0x1aaStockBusPresentAnd0x1cfAbsent": bool(raw_alt_stock_bus) and not raw_standard_stock_bus,
      "send0x1aaUsesExpectedBus": send_bus_matches,
      "stockAndSendBusesUseSafetyPanda": same_safety_panda,
      "allScc0x1a0CrcValid": all_scc_crc_valid,
      "allRaw0x1aaCrcValid": all_raw_button_crc_valid,
      "allRearm0x1aaCrcValid": all_rearm_crc_valid,
      "latestStockNonButtonFieldsPreserved": stock_fields_preserved,
      "sourcePlusOneCounterAllFrames": source_plus_one,
      "sourceTimestampsFreshSequentialAllGroups": source_timestamps_fresh,
      "sourceCountersFreshSequentialAllGroups": source_counters_fresh,
      "emittedCountersFreshSequentialAllGroups": emitted_counters_fresh,
      "rawPostBurstSameCounterAndNextReleaseCandidatesObservedAllGroups": (
        post_burst_release_candidates_observed
      ),
      "exactAltBusAndAddressLayout": exact_alt_layout,
      "exactSupportedRearmSchedule": returned_schedule_ok,
      "allObservedRearmRequestsReturned": all_tx_returned,
      "rawAdrv0x161ContinuousThrough30_25s": cluster_evidence["rawAdrvContinuousThrough30_25s"],
      "rawLfaHda0x1e0ContinuousThrough30_25s": cluster_evidence["rawLfaHdaContinuousThrough30_25s"],
      "rawAdrv0x161CrcValidAllSamples": cluster_evidence["integrity"]["rawAdrvAllCrcValid"],
      "rawLfaHda0x1e0Observed": cluster_evidence["rawLfaHdaSamples"] > 0,
      "rawLfaHda0x1e0CrcValidAllSamples": cluster_evidence["integrity"]["rawLfaHdaAllCrcValid"],
      "hostClusterReplacementCrcValidAllSamples": cluster_evidence["integrity"]["hostReplacementAllCrcValid"],
      "hda1HostReplacementStreamsContinuousOrHda2NoReplacement": (
        canfd_hda2
        or all(coverage["continuous"]
               for coverage in cluster_evidence["hostReplacementCoverage"].values())
      ),
      "hda1RawAdrvExactlyPairedWithHostRequestAndReturnOrHda2NoReplacement": (
        canfd_hda2
        or cluster_evidence["hostReplacementPairing"]["exactAdrvRawRequestReturnedPairing"]
      ),
      "hda1RawLfaHdaExactlyPairedWithHostRequestAndReturnOrHda2NoReplacement": (
        canfd_hda2
        or cluster_evidence["hostReplacementPairing"]["exactLfaHdaRawRequestReturnedPairing"]
      ),
      "rawAdrvWarningPreservedAcrossActiveTopology": cluster_evidence[
        "warningPreservation"]["pathPreservesRawWarning"],
      "hdaPathConsistentWithCurrentStockLongTopology": cluster_evidence[
        "hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"],
    }

    verdict = "INCONCLUSIVE"
    reasons = []
    if any_rejected or any_cluster_rejected:
      verdict = "FAIL"
      reasons.append("Panda safety rejected one or more 0x1AA RES or cluster replacement requests")
    elif direct_warning_masking:
      verdict = "FAIL"
      reasons.append("HDA1 returned host replacement masked raw ADRV_0x161 ALERTS_5=5")
    elif direct_schedule_violation:
      verdict = "FAIL"
      reasons.append("0x1AA RES traffic exceeded the supported schedule or continued after 27.00s")
    elif direct_crc_failure:
      verdict = "FAIL"
      reasons.append("an SCC, button, ADRV, or LFAHDA frame failed the Hyundai CAN-FD CRC")
    elif direct_source_policy_failure:
      verdict = "FAIL"
      reasons.append(
        "a RES group violated fresh sequential source/emitted counters, source+1, or stock non-button fields"
      )
    elif potential_false_start:
      verdict = "FAIL"
      reasons.append("vehicle moved while the stationary-lead gate was still valid; potential false start")
    elif states and not stopped_motion_ok:
      verdict = "FAIL"
      reasons.append("standstill evidence contained motion above the vEgo/vEgoRaw gate or a false standstill sample")
    elif first_raw_warning is not None and first_raw_warning < EARLIEST_EXPECTED_FINAL_WARNING:
      verdict = "FAIL"
      reasons.append(
        f"raw ADRV_0x161 ALERTS_5=5 appeared early at {first_raw_warning:.3f}s after stop"
      )
    elif first_raw_warning is not None and first_raw_warning > LATEST_EXPECTED_FINAL_WARNING:
      verdict = "FAIL"
      reasons.append(
        f"raw ADRV_0x161 ALERTS_5=5 appeared late at {first_raw_warning:.3f}s after stop"
      )
    elif schedule_mode == SCHEDULE_MODE_MIXED:
      reasons.append(
        "InfoDisplay=4 changed during the pre-warning stop window, so neither supported schedule can be proven"
      )
    elif schedule_mode == SCHEDULE_MODE_UNKNOWN:
      reasons.append("authoritative SCC status did not cover the physical stop boundary")
    elif all(prerequisites.values()):
      raw_boundary_observed = (
        first_raw_warning is not None
        and EARLIEST_EXPECTED_FINAL_WARNING <= first_raw_warning <= LATEST_EXPECTED_FINAL_WARNING
      )
      if raw_boundary_observed:
        verdict = "PASS"
        reasons.append(
          " ".join((
            f"the returned {schedule_mode} 0x1AA schedule and raw ADRV ALERTS_5=5 source aligned",
            "at the 30s boundary; TX return does not prove arbitration, ECU acceptance, or an individual timer reset",
          ))
        )
      elif first_raw_warning is None:
        verdict = "PASS_AT_LEAST_30S"
        reasons.append(
          "the continuously observed raw ADRV warning source stayed clear for at least 30s; the final timeout and individual RES timer resets remain unproven"
        )
      else:
        reasons.append("raw ADRV ALERTS_5 did not establish the approximately-30s warning boundary")
    else:
      failed = [name for name, passed in prerequisites.items() if not passed]
      reasons.append("missing proof prerequisites: " + ", ".join(failed))

    return {
      "start": start_rel,
      "end": end_rel,
      "duration": round(duration, 3),
      "startObserved": start_observed,
      "stateSamples": len(states),
      "sccSamples": len(scc),
      "rawSccSafeSamples": len(safe_scc),
      "scheduleMode": schedule_mode,
      "resGroups": episode_groups,
      "returnedSchedule": schedule,
      "streamCoverage": stream_coverage,
      "busLayout": {
        "stockBus": stock_bus,
        "pandaBusOffset": panda_bus_offset,
        "canFdHda2": canfd_hda2,
        "expectedSendBus": expected_send_bus,
        "observedSendBuses": sorted({request.bus for request in rearm_requests}),
      },
      "potentialFalseStart": potential_false_start,
      "infoDisplay4Periods": info_display_4_periods,
      "clusterEvidence": cluster_evidence,
      "alerts": [{**asdict(alert), "t": self._rel(alert.t)} for alert in alerts[:20]],
      "prerequisites": prerequisites,
      "verdict": verdict,
      "reasons": reasons,
    }

  def report(self) -> dict[str, Any]:
    if not math.isfinite(self.first_t):
      self.first_t = self.last_t = 0.0
    tx_matches, status_by_id = self._match_button_tx()
    cluster_tx_matches, cluster_status_by_id = self._cluster_tx_matches()
    res_groups = self._res_groups(status_by_id)
    episodes = [self._episode_report(start, end, observed, res_groups, cluster_status_by_id)
                for start, end, observed in self._stop_episodes()]

    verdict_order = {"FAIL": 4, "PASS": 3, "PASS_AT_LEAST_30S": 2, "INCONCLUSIVE": 1}
    if episodes:
      overall = max((episode["verdict"] for episode in episodes), key=lambda verdict: verdict_order[verdict])
    else:
      overall = "INCONCLUSIVE"

    return {
      "schemaVersion": REPORT_SCHEMA_VERSION,
      "mode": self.mode,
      "source": self.source,
      "capture": {
        "duration": round(max(0.0, self.last_t - self.first_t), 3),
        "firstLogMonoTime": self.first_t,
        "lastLogMonoTime": self.last_t,
      },
      "carParamsSource": self.car_params_source,
      "carParams": self.car_params,
      "params": self.params,
      "stateEvidence": self._state_report(),
      "streams": self._stream_report(),
      "decodeErrors": dict(self.decode_errors),
      "rawButtonCounters": self._raw_button_counter_report(),
      "sccEvidence": self._scc_report(),
      "buttonEvidence": self._button_report(),
      "clusterCanEvidence": self._cluster_report(cluster_tx_matches),
      "txMatches": tx_matches,
      "resGroups": res_groups,
      "correlatedTimeline": self._correlated_timeline(status_by_id, cluster_status_by_id),
      "stopEpisodes": episodes,
      "overallVerdict": overall,
      "interpretation": {
        "PASS": (
          " ".join((
            "A qualified capture observed a supported TX-return schedule plus the raw ADRV warning source",
            "at approximately 30s. This does not prove arbitration, ECU acceptance, or which RES frame reset an SCC timer.",
          ))
        ),
        "PASS_AT_LEAST_30S": (
          "The continuously observed raw ADRV warning source stayed clear for 30s, but the final timeout and individual SCC timer resets remain unproven."
        ),
        "FAIL": "Direct on-wire or stock-SCC status evidence contradicts the requested behavior.",
        "INCONCLUSIVE": "The capture lacks one or more prerequisites; do not treat absence of an error as success.",
      },
      "limitations": [
        "A sendcan row is only a request; can.src +0x80 shows that Panda returned the queued transmit attempt.",
        "A Panda TX-return does not prove that the frame won CAN arbitration or that the SCC ECU accepted or acted on it.",
        (
          "TX-return does not expose Panda's camera-side forwarding decision for intervening raw 0x1AA frames; " +
          "even when the raw same-counter and next-counter release candidates are observed, only a separate " +
          "camera-bus capture can prove release timing and counter continuity seen by the receiving ECU."
        ),
        "Neither +0x80 TX-return nor stock 0x1A0 InfoDisplay acknowledges SCC acceptance or a timer reset.",
        "SCC_CONTROL InfoDisplay=4 selects the regular/recovery state schedule only; it is not used as visible-warning success or failure evidence.",
        "Raw ADRV_0x161 ALERTS_5=5 is source evidence, not display proof; HDA1 PASS requires its returned replacement to preserve value 5.",
        "HDA2 raw-path visibility is inferred from current-branch topology and absence of a host replacement, not a cluster display acknowledgement.",
        "Start the capture before the physical stop and keep recording past 31 seconds.",
        "Use a full rlog when possible; qlogs can omit or downsample CAN/sendcan/state evidence.",
      ],
    }


def run_log(source: str) -> ProbeAnalyzer:
  from openpilot.tools.lib.logreader import LogReader

  analyzer = ProbeAnalyzer("log", source)
  for event in LogReader(source, sort_by_time=True):
    analyzer.feed_cereal_event(event)
  return analyzer


def run_live(duration: float, messaging_address: str) -> ProbeAnalyzer:
  import openpilot.cereal.messaging as messaging

  analyzer = ProbeAnalyzer("live", messaging_address)
  analyzer.params, cp, cp_source = read_live_params()
  if cp is not None:
    analyzer.set_car_params(cp, f"Params:{cp_source}")

  sockets = {
    service: messaging.sub_sock(service, addr=messaging_address, conflate=False)
    for service in ("can", "sendcan", "carState", "carControl", "selfdriveState", "carParams")
  }
  start = time.monotonic()
  last_status = start
  try:
    while duration <= 0 or time.monotonic() - start < duration:
      for sock in sockets.values():
        for event in messaging.drain_sock(sock):
          analyzer.feed_cereal_event(event)
      now = time.monotonic()
      if now - last_status >= 5.0:
        print(f"capturing: {now - start:.1f}s, stock 0x1A0={len(analyzer.scc)}, button frames={len(analyzer.buttons)}",
              file=sys.stderr, flush=True)
        last_status = now
      time.sleep(0.005)
  except KeyboardInterrupt:
    print("capture stopped by user", file=sys.stderr)
  return analyzer


def _demo_car_params(bus_offset: int = 0, hda2: bool = False) -> Any:
  from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags

  return SimpleNamespace(
    carFingerprint=TARGET_FINGERPRINT,
    brand="hyundai",
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    alphaLongitudinalAvailable=True,
    autoResumeSng=False,
    flags=int(
      HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS
      | (HyundaiFlags.CANFD_HDA2 if hda2 else 0)
    ),
    extFlags=0,
    safetyConfigs=(
      [SimpleNamespace(safetyModel="noOutput", safetyParam=0)] if bus_offset else []
    ) + [SimpleNamespace(
      safetyModel="hyundaiCanfd",
      safetyParam=int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS),
    )],
  )


def run_demo(outcome: str, bus_offset: int = 0, hda2: bool = False, *,
             schedule_mode: str = SCHEDULE_MODE_REGULAR, button_source_phase_frames: int = 0,
             raw_warning_time: float | None = None, mask_hda1_warning: bool = False,
             button_counter_offset: int = 0) -> ProbeAnalyzer:
  """Generate deterministic, real-DBC frames for an offline dry run."""
  from opendbc.can import CANPacker

  if schedule_mode not in (SCHEDULE_MODE_REGULAR, SCHEDULE_MODE_INITIAL_RECOVERY, SCHEDULE_MODE_MIXED):
    raise ValueError(f"unsupported demo schedule mode: {schedule_mode}")
  if button_source_phase_frames not in (0, 1):
    raise ValueError("button_source_phase_frames must be 0 or 1")

  analyzer = ProbeAnalyzer("demo", f"{outcome}:{schedule_mode}:phase-{button_source_phase_frames}")
  analyzer.set_car_params(_demo_car_params(bus_offset, hda2), "demo")
  packer = CANPacker(DBC_NAME)
  base = 1_000.0
  if raw_warning_time is None:
    raw_warning_time = 30.0 if outcome == "pass" else 3.0

  def demo_info_display(elapsed: float) -> int:
    final_state = elapsed >= 30.0
    initial_recovery_state = (
      schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY
      and elapsed <= INITIAL_RECOVERY_REQUIRED_DURATION
    )
    mixed_recovery_state = schedule_mode == SCHEDULE_MODE_MIXED and 1.0 <= elapsed <= 1.30
    return 4 if final_state or initial_recovery_state or mixed_recovery_state else 0

  moving_state = SimpleNamespace(
    standstill=False, vEgo=1.0, vEgoRaw=1.0, canValid=True, brakePressed=False, gasPressed=False,
    brakeHoldActive=False, parkingBrake=False, accFaulted=False,
    cruiseState=SimpleNamespace(enabled=True, standstill=False),
  )
  analyzer.feed_state(base - 0.05, moving_state)
  analyzer.feed_control(base - 0.05, SimpleNamespace(enabled=True))

  stop_duration = 35.0
  for frame in range(round(stop_duration * 100) + 1):
    elapsed = frame / 100.0
    t = base + elapsed
    info_display = demo_info_display(elapsed)
    if frame % 2 == 0:
      scc = packer.make_can_msg("SCC_CONTROL", 0, {
        "COUNTER": (frame // 2) & 0xFF,
        "ACCMode": 1,
        "ACC_ObjDist": 5.0,
        "ACC_ObjRelSpd": 0.0,
        "HUD_LEAD_INFO": 2,
        "InfoDisplay": info_display,
        "SysFailState": 0,
        "TakeOverReq": 0,
      })
      analyzer.feed_can("can", t, bus_offset, scc[0], scc[1])

    if frame >= button_source_phase_frames and (frame - button_source_phase_frames) % 2 == 0:
      stock_button = packer.make_can_msg("CRUISE_BUTTONS_ALT", 0, {
        "COUNTER": (((frame - button_source_phase_frames) // 2) + button_counter_offset) & 0xFF,
        "CRUISE_BUTTONS": BUTTON_NONE,
        "DISTANCE_UNIT": 1,
        "SET_ME_2": 3,
      })
      analyzer.feed_can("can", t, bus_offset, stock_button[0], stock_button[1])

    if frame % 4 == 0:
      stopped_state = SimpleNamespace(
        standstill=True, vEgo=0.0, vEgoRaw=0.0, canValid=True, brakePressed=False, gasPressed=False,
        brakeHoldActive=False, parkingBrake=False, accFaulted=False,
        cruiseState=SimpleNamespace(enabled=True, standstill=info_display >= 4),
      )
      analyzer.feed_state(t, stopped_state)
      analyzer.feed_control(t, SimpleNamespace(enabled=True))

  # Real-DBC camera source and host replacement evidence. HDA1 stock-long
  # replaces both messages on ECAN while preserving stock-owned warnings.
  # HDA2 stock-long does not synthesize these frames, leaving the raw camera
  # path unmodified. mask_hda1_warning generates the explicit regression case.
  raw_camera_bus = bus_offset + 2
  for frame in range(round(stop_duration * 20) + 1):
    elapsed = frame / 20.0
    t = base + elapsed
    raw_alert = USE_SWITCH_OR_PEDAL_TO_ACCELERATE if elapsed >= raw_warning_time else 0
    raw_adrv = packer.make_can_msg("ADRV_0x161", raw_camera_bus, {
      "COUNTER": frame & 0xFF,
      "ALERTS_5": raw_alert,
    })
    raw_hda = packer.make_can_msg("LFAHDA_CLUSTER", raw_camera_bus, {
      "COUNTER": frame & 0xFF,
      "HDA_CntrlModSta": 2,
      "HDA_LFA_SymSta": 2,
    })
    analyzer.feed_can("can", t, raw_camera_bus, raw_adrv[0], raw_adrv[1])
    analyzer.feed_can("can", t, raw_camera_bus, raw_hda[0], raw_hda[1])
    if not hda2:
      host_adrv = packer.make_can_msg("ADRV_0x161", bus_offset, {
        "COUNTER": frame & 0xFF,
        "ALERTS_5": 0 if mask_hda1_warning else raw_alert,
      })
      host_hda = packer.make_can_msg("LFAHDA_CLUSTER", bus_offset, {
        "COUNTER": frame & 0xFF,
        "HDA_CntrlModSta": 2,
        "HDA_LFA_SymSta": 2,
      })
      for message in (host_adrv, host_hda):
        analyzer.feed_can("sendcan", t + 0.001, bus_offset, message[0], message[1])
        analyzer.feed_can("can", t + 0.003, bus_offset + PANDA_RETURNED_BUS_OFFSET, message[0], message[1])

  if outcome != "missing-tx":
    schedule_variants = (
      RECOVERY_REARM_GROUP_STARTS
      if schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY else
      REGULAR_REARM_GROUP_STARTS
    )
    group_starts = schedule_variants[button_source_phase_frames]
    for group_start in group_starts:
      for offset in (0.0, 0.02, 0.04):
        source_frame = round((group_start + offset) * 100.0)
        source_counter = (
          ((source_frame - button_source_phase_frames) // 2) + button_counter_offset
        ) & 0xFF
        counter = (source_counter + 1) & 0xFF
        send_bus = bus_offset if hda2 else bus_offset + 2
        tx = packer.make_can_msg("CRUISE_BUTTONS_ALT", send_bus, {
          "COUNTER": counter,
          "CRUISE_BUTTONS": BUTTON_RES_ACCEL,
          "DISTANCE_UNIT": 1,
          "SET_ME_2": 3,
        })
        request_t = base + group_start + offset
        analyzer.feed_can("sendcan", request_t, send_bus, tx[0], tx[1])
        echo_src = send_bus + (PANDA_RETURNED_BUS_OFFSET if outcome == "pass" else PANDA_REJECTED_BUS_OFFSET)
        analyzer.feed_can("can", request_t + 0.002, echo_src, tx[0], tx[1])
  return analyzer


def print_human(report: dict[str, Any]) -> None:
  print("KA4 stock-SCC passive probe")
  print(f"mode={report['mode']} source={report['source']} capture={report['capture']['duration']:.3f}s")
  cp = report.get("carParams")
  if cp:
    print(
      f"CarParams: {cp['carFingerprint']} flags={cp['flagsHex']} ",
      f"pcmCruise={cp['pcmCruise']} openpilotLongitudinalControl={cp['openpilotLongitudinalControl']}",
      sep="",
    )
    print(f"KA4 stock-SCC gate: {'PASS' if cp['ka4StockSccGatePassed'] else 'FAIL'} {cp['ka4StockSccGate']}")
  else:
    print("CarParams: MISSING")

  if report.get("params"):
    print("Params (read-only snapshot):")
    for key, value in report["params"].items():
      print(f"  {key}: {value}")

  print("Relevant CAN streams:")
  for stream in report["streams"]:
    hz = "n/a" if stream["observedHz"] is None else f"{stream['observedHz']:.1f}Hz"
    print(
      f"  {stream['service']:7s} {stream['origin']:21s} src={stream['srcHex']:>4s} ",
      f"bus={stream['physicalBus']} {stream['addressHex']} count={stream['count']} {hz} ",
      f"checksum(valid/invalid/n-a)={stream['checksumValid']}/{stream['checksumInvalid']}/{stream['checksumUnavailable']}",
      sep="",
    )

  print(f"Preferred stock SCC/button bus: {report['sccEvidence']['preferredStockBus']}")
  for bus in report["sccEvidence"]["buses"]:
    print(
      f"  stock 0x1A0 bus={bus['bus']} count={bus['count']} ",
      f"InfoDisplay={bus['infoDisplayCounts']} ACCMode={bus['accModeCounts']} ",
      f"safe-lead={bus['rawLeadSafeSamples']} counter={bus['counter']}",
      sep="",
    )
    if bus["infoDisplayTransitions"]:
      print(f"    InfoDisplay transitions: {bus['infoDisplayTransitions']}")
  for buttons in report["buttonEvidence"]:
    print(
      f"  buttons {buttons['service']}/{buttons['origin']} bus={buttons['bus']} {buttons['addressHex']} ",
      f"count={buttons['count']} values={buttons['buttonCounts']} ",
      f"counters={buttons['firstCounter']}..{buttons['lastCounter']}",
      sep="",
    )
  for counters in report["rawButtonCounters"]:
    print(
      f"    raw counter health {counters['addressHex']} bus={counters['bus']}: ",
      f"+1={counters['incrementByOne']} dup={counters['duplicates']} jumps={counters['jumps']}",
      sep="",
    )

  cluster = report["clusterCanEvidence"]
  print(f"Cluster CAN topology: {cluster['topology']}")
  for stream in cluster["streams"]:
    print(
      f"  {stream['message']} {stream['service']}/{stream['origin']} bus={stream['bus']} ",
      f"values={stream['valueCounts']} transitions={stream['transitions']}",
      sep="",
    )
  print(f"Cluster CAN TX matches: {cluster['txMatchCounts']}")

  state = report["stateEvidence"]
  print(
    f"carState gates: samples={state['sampleCount']} standstill={state['standstillSamples']} ",
    f"cruise-enabled={state['cruiseEnabledSamples']} eligible-stop={state['eligibleStopSamples']} ",
    f"max-continuous(s) standstill={state['maxContinuousStandstill']:.3f} ",
    f"eligible-stop={state['maxContinuousEligibleStop']:.3f}",
    sep="",
  )
  if state["gateTransitions"]:
    print(f"  carState gate transitions: {state['gateTransitions']}")

  print("Stop episodes:")
  if not report["stopEpisodes"]:
    print("  none (need carState.standstill && cruiseState.enabled)")
  for index, episode in enumerate(report["stopEpisodes"], 1):
    print(
      f"  #{index}: {episode['duration']:.3f}s verdict={episode['verdict']} ",
      f"mode={episode['scheduleMode']} 0x1A0={episode['sccSamples']} RES-groups={len(episode['resGroups'])}",
      sep="",
    )
    for reason in episode["reasons"]:
      print(f"    - {reason}")
    if episode["infoDisplay4Periods"]:
      print(f"    InfoDisplay=4 state periods: {episode['infoDisplay4Periods']}")
    print(f"    raw cluster evidence: {episode['clusterEvidence']}")
    if episode["returnedSchedule"]:
      print(f"    schedule: {episode['returnedSchedule']}")
    print(f"    proof stream coverage: {episode['streamCoverage']}")
    for group in episode["resGroups"]:
      print(
        f"    RES @{group['start'] - episode['start']:.3f}s counters={group['counters']} ",
        f"source={group['sourceCountersPerFrame']} source-age-ms={group['sourceAgeMsPerFrame']} tx={group['statuses']} ",
        f"crc={group['allRequestChecksumsValid']} source+1={group['sourcePlusOneAllFrames']} ",
        f"source-time-sequential={group['sourceTimestampsFreshSequential']} ",
        f"source-sequential={group['sourceCountersFreshSequential']} ",
        f"emitted-sequential={group['emittedCountersFreshSequential']} ",
        f"stock-fields={group['latestStockNonButtonFieldsPreserved']}",
        sep="",
      )
    print(f"    proof prerequisites: {episode['prerequisites']}")

  returned = sum(match["status"] == "returned" for match in report["txMatches"])
  rejected = sum(match["status"] == "rejected" for match in report["txMatches"])
  unobserved = sum(match["status"] == "unobserved" for match in report["txMatches"])
  print(f"0x1AA TX correlation: returned={returned} rejected={rejected} unobserved={unobserved}")
  print(f"OVERALL: {report['overallVerdict']}")
  print(report["interpretation"][report["overallVerdict"]])


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Read-only KA4 stock-SCC 30-second re-arm evidence probe",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("--json", action="store_true", help="print one JSON report instead of the human report")
  subparsers = parser.add_subparsers(dest="command", required=True)

  live = subparsers.add_parser("live", help="passively subscribe to a running comma device")
  live.add_argument("--duration", type=float, default=60.0, help="seconds to capture; 0 runs until Ctrl-C")
  live.add_argument("--messaging-address", default="127.0.0.1")

  log = subparsers.add_parser("log", help="read a saved rlog or route segment")
  log.add_argument("source", help="local rlog path or LogReader-compatible route segment")

  demo = subparsers.add_parser("demo", help="offline real-DBC dry run")
  demo.add_argument("--outcome", choices=("pass", "fail", "missing-tx"), default="pass")
  return parser


def main(argv: Iterable[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  if args.command == "live":
    if args.duration != 0 and args.duration < MIN_EXACT_PROOF_DURATION:
      print(f"warning: {args.duration:.2f}s cannot prove the 30s requirement", file=sys.stderr)
    analyzer = run_live(args.duration, args.messaging_address)
  elif args.command == "log":
    analyzer = run_log(args.source)
  else:
    analyzer = run_demo(args.outcome)

  report = analyzer.report()
  if args.json:
    print(json.dumps(report, indent=2, sort_keys=True))
  else:
    print_human(report)
  if report["overallVerdict"] == "PASS":
    return 0
  if report["overallVerdict"] == "FAIL":
    return 1
  # Partial or missing evidence must not look green in automation.
  return 2


if __name__ == "__main__":
  raise SystemExit(main())
