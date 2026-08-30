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

The report separates a 0x1AA sendcan request from Panda's actual TX-return
echo (``can.src == physical_bus + 0x80``) and safety rejection echo
(``can.src == physical_bus + 0xC0``). A sendcan request by itself is not proof
that anything reached the vehicle bus. Likewise, a TX-return echo proves that
Panda transmitted the frame, while the subsequent stock 0x1A0 InfoDisplay
behavior is the evidence used to infer whether the SCC ECU accepted the re-arm.
Exit status is 0 only for an exact PASS, 1 for direct FAIL evidence, and 2 for
partial or inconclusive evidence.
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
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any


OPENPILOT_ROOT = Path(__file__).resolve().parents[2]
if str(OPENPILOT_ROOT) not in sys.path:
  sys.path.insert(0, str(OPENPILOT_ROOT))


DBC_NAME = "hyundai_canfd_generated"
SCC_CONTROL_ADDRESS = 0x1A0
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

TARGET_FINGERPRINT = "KIA CARNIVAL 4TH GEN"
MIN_EXACT_PROOF_DURATION = 30.25
EARLIEST_EXPECTED_FINAL_WARNING = 29.5
LATEST_EXPECTED_FINAL_WARNING = 31.5
TX_MATCH_WINDOW = 0.100
STOP_STREAM_GAP = 0.500
EXPECTED_REARM_GROUP_STARTS = (2.50, 5.02, 7.54, 10.06, 12.58, 15.10, 17.62, 20.14, 22.66, 25.18, 26.98)
REARM_TIMING_TOLERANCE = 0.060
FINAL_REARM_FRAME_TIME = 27.00

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
      and self.lead_info == 2
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
    "fingerprintIsKa4": fingerprint == TARGET_FINGERPRINT,
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
    "flags": flags,
    "flagsHex": f"0x{flags:X}",
    "flagNames": _flag_names(HyundaiFlags, flags),
    "extFlags": ext_flags,
    "extFlagsHex": f"0x{ext_flags:X}",
    "extFlagNames": _flag_names(HyundaiExtFlags, ext_flags),
    "safetyConfigs": safety_configs,
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
    self.states.append(StateSample(
      t=t,
      standstill=bool(_safe_get(state, "standstill", False)),
      cruise_enabled=bool(_safe_get(cruise, "enabled", False)),
      cruise_standstill=bool(_safe_get(cruise, "standstill", False)),
      v_ego=_finite_float(_safe_get(state, "vEgo", 0.0)),
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
      expected_counters = None if oem_counter is None else [
        (oem_counter + 1) & 0xFF,
        (oem_counter + 1) & 0xFF,
        (oem_counter + 2) & 0xFF,
      ]
      counters = [sample.counter for sample in group]
      source_plus_one = []
      latest_stock_preserved = []
      source_counters = []
      for request in group:
        # CAN and sendcan batches representing the same 10 ms control tick can
        # differ by sub-millisecond publication/float rounding. A source up to
        # 2 ms later is still the same-tick source, not a future counter.
        sources = [sample for sample in stock if -0.002 <= request.t - sample.t <= 0.100]
        source = sources[-1] if sources else None
        source_counters.append(source.counter if source is not None else None)
        source_plus_one.append(source is not None and request.counter == (source.counter + 1) & 0xFF)
        latest_stock_preserved.append(
          source is not None and request.non_button_values == source.non_button_values
        )
      result.append({
        "start": self._rel(group[0].t),
        "end": self._rel(group[-1].t),
        "addressHex": f"0x{group[0].address:X}",
        "bus": group[0].bus,
        "frameCount": len(group),
        "counters": counters,
        "precedingOemCounter": oem_counter,
        "sourceCountersPerFrame": source_counters,
        "sourceAvailableAllFrames": all(counter is not None for counter in source_counters),
        "expectedCurrentControllerCounters": expected_counters,
        "counterPatternMatchesCurrentController": counters == expected_counters,
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

  def _episode_report(self, start: float, end: float, start_observed: bool,
                      res_groups: list[dict[str, Any]]) -> dict[str, Any]:
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

    raw_alt_bus0 = [sample for sample in self.buttons if start <= sample.t <= end and
                    sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS and sample.bus == 0]
    raw_standard_bus0 = [sample for sample in self.buttons if start <= sample.t <= end and
                         sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ADDRESS and sample.bus == 0]
    rearm_requests = [sample for sample in self.buttons if start <= sample.t <= end and
                      sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL]

    safe_scc = [sample for sample in scc if sample.raw_lead_safe]
    valid_state = [sample for sample in states if sample.can_valid]
    no_interlock = [sample for sample in states if not sample.interlock_active]
    enabled_controls = [sample for sample in controls if sample.enabled]
    control_cancel_clear = bool(controls) and all(not sample.cancel for sample in controls)
    raw_driver_buttons_clear = bool(raw_alt_bus0) and all(
      sample.button == BUTTON_NONE
      and sample.adaptive_main == 0
      and sample.normal_main == 0
      and sample.lfa_button == 0
      for sample in raw_alt_bus0
    )
    warning_periods = self._warning_periods(scc, start)
    first_warning = warning_periods[0]["startAfterStop"] if warning_periods else None

    group_times = [group["start"] - start_rel for group in episode_groups]
    returned_schedule_ok = False
    schedule = {}
    if group_times:
      gaps = [curr - prev for prev, curr in zip(group_times, group_times[1:], strict=False)]
      exact_group_times = (
        len(group_times) == len(EXPECTED_REARM_GROUP_STARTS)
        and all(abs(actual - expected) <= REARM_TIMING_TOLERANCE
                for actual, expected in zip(group_times, EXPECTED_REARM_GROUP_STARTS, strict=True))
      )
      exact_three_frame_groups = all(
        group["frameCount"] == 3
        and len(group["frameSpacingMs"]) == 2
        and all(abs(spacing - 10.0) <= 6.0 for spacing in group["frameSpacingMs"])
        for group in episode_groups
      )
      no_late_rearm = not any(request.t - start > FINAL_REARM_FRAME_TIME + 0.005 for request in rearm_requests)
      schedule = {
        "firstGroupAfterStop": round(group_times[0], 3),
        "lastGroupAfterStop": round(group_times[-1], 3),
        "maxGroupGap": round(max(gaps), 3) if gaps else None,
        "expectedGroupStarts": list(EXPECTED_REARM_GROUP_STARTS),
        "actualGroupStarts": [round(group_time, 3) for group_time in group_times],
        "exactExpectedGroupTimes": exact_group_times,
        "exactThreeFrameGroups": exact_three_frame_groups,
        "noResAfter27_00s": no_late_rearm,
      }
      returned_schedule_ok = (
        exact_group_times
        and exact_three_frame_groups
        and no_late_rearm
      )

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
    stopped_motion_ok = bool(states) and all(sample.standstill and abs(sample.v_ego) <= 0.05 for sample in states)
    exact_duration = duration >= MIN_EXACT_PROOF_DURATION
    all_tx_returned = bool(episode_groups) and all(group["allReturned"] for group in episode_groups)
    any_rejected = any(group["statuses"].get("rejected", 0) for group in episode_groups)
    all_scc_crc_valid = bool(scc) and all(sample.checksum_valid is True for sample in scc)
    all_raw_button_crc_valid = bool(raw_alt_bus0) and all(sample.checksum_valid is True for sample in raw_alt_bus0)
    all_rearm_crc_valid = bool(episode_groups) and all(group["allRequestChecksumsValid"] for group in episode_groups)
    source_plus_one = bool(episode_groups) and all(group["sourcePlusOneAllFrames"] for group in episode_groups)
    stock_fields_preserved = bool(episode_groups) and all(
      group["latestStockNonButtonFieldsPreserved"] for group in episode_groups)
    exact_alt_layout = (
      bool(raw_alt_bus0)
      and not raw_standard_bus0
      and bool(rearm_requests)
      and all(request.address == CRUISE_BUTTONS_ALT_ADDRESS and request.bus == 2 for request in rearm_requests)
    )

    # Look just beyond the final standstill sample. Capping this window at
    # ``start + 30`` makes it empty for every proof-length episode (whose end
    # is already later than 30 s), silently hiding a false start at release.
    next_states = [sample for sample in self.states if end < sample.t <= end + 0.250]
    last_safe_scc = scc[-1].raw_lead_safe if scc else False
    potential_false_start = any(
      not sample.standstill and sample.cruise_enabled and abs(sample.v_ego) > 0.05 and not sample.interlock_active
      for sample in next_states
    ) and last_safe_scc
    direct_schedule_violation = (
      len(episode_groups) > len(EXPECTED_REARM_GROUP_STARTS)
      or len(rearm_requests) > len(EXPECTED_REARM_GROUP_STARTS) * 3
      or any(request.t - start > FINAL_REARM_FRAME_TIME + 0.005 for request in rearm_requests)
      or any(group["frameCount"] > 3 for group in episode_groups)
    )
    direct_crc_failure = (
      any(sample.checksum_valid is False for sample in scc)
      or any(sample.checksum_valid is False for sample in raw_alt_bus0)
      or any(not group["allRequestChecksumsValid"] for group in episode_groups)
    )
    direct_source_policy_failure = bool(episode_groups) and any(
      group["sourceAvailableAllFrames"]
      and (not group["sourcePlusOneAllFrames"] or not group["latestStockNonButtonFieldsPreserved"])
      for group in episode_groups
    )

    prerequisites = {
      "ka4StockSccCarParamsGate": gate_passed,
      "physicalStopStartObserved": start_observed,
      "durationAtLeast30_25s": exact_duration,
      "carStateCanValidAllSamples": state_coverage_ok,
      "driverInterlocksClearAllSamples": no_interlocks,
      "carControlEnabledAllSamples": control_coverage_ok,
      "wheelStandstillAndVEgoZeroAllSamples": stopped_motion_ok,
      "noPotentialFalseStart": not potential_false_start,
      "rawSccLeadGateValidAllSamples": raw_scc_coverage_ok,
      "raw0x1aaBus0PresentAnd0x1cfAbsent": bool(raw_alt_bus0) and not raw_standard_bus0,
      "send0x1aaUsesBus2": bool(rearm_requests) and all(request.bus == 2 for request in rearm_requests),
      "allScc0x1a0CrcValid": all_scc_crc_valid,
      "allRaw0x1aaCrcValid": all_raw_button_crc_valid,
      "allRearm0x1aaCrcValid": all_rearm_crc_valid,
      "latestStockNonButtonFieldsPreserved": stock_fields_preserved,
      "sourcePlusOneCounterAllFrames": source_plus_one,
      "exactAltBusAndAddressLayout": exact_alt_layout,
      "exact11GroupRearmSchedule": returned_schedule_ok,
      "allObservedRearmRequestsReturned": all_tx_returned,
    }

    verdict = "INCONCLUSIVE"
    reasons = []
    if any_rejected:
      verdict = "FAIL"
      reasons.append("Panda safety rejected one or more 0x1AA RES requests")
    elif direct_schedule_violation:
      verdict = "FAIL"
      reasons.append("0x1AA RES traffic exceeded the exact 11x3 schedule or continued after 27.00s")
    elif direct_crc_failure:
      verdict = "FAIL"
      reasons.append("an SCC_CONTROL or CRUISE_BUTTONS_ALT frame failed the Hyundai CAN-FD CRC")
    elif direct_source_policy_failure:
      verdict = "FAIL"
      reasons.append("a RES frame violated source+1 counter or changed a non-button field from the latest stock 0x1AA")
    elif potential_false_start:
      verdict = "FAIL"
      reasons.append("vehicle moved while the stationary-lead gate was still valid; potential false start")
    elif states and not stopped_motion_ok:
      verdict = "FAIL"
      reasons.append("standstill evidence contained nonzero vEgo or a false standstill sample")
    elif first_warning is not None and first_warning < EARLIEST_EXPECTED_FINAL_WARNING:
      verdict = "FAIL"
      reasons.append(f"stock InfoDisplay=4 appeared early at {first_warning:.3f}s after stop")
    elif all(prerequisites.values()):
      if first_warning is not None and first_warning <= LATEST_EXPECTED_FINAL_WARNING:
        verdict = "PASS"
        reasons.append("returned 0x1AA re-arms covered the stop and stock InfoDisplay=4 appeared at the 30s boundary")
      elif first_warning is None:
        verdict = "PASS_AT_LEAST_30S"
        reasons.append("returned 0x1AA re-arms kept stock InfoDisplay clear for at least 30s; the final OEM timeout was not observed")
      else:
        verdict = "FAIL"
        reasons.append(f"stock InfoDisplay=4 appeared too late at {first_warning:.3f}s; the requested 30s maximum was not shown")
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
      "resGroups": episode_groups,
      "returnedSchedule": schedule,
      "potentialFalseStart": potential_false_start,
      "warningPeriods": warning_periods,
      "alerts": [{**asdict(alert), "t": self._rel(alert.t)} for alert in alerts[:20]],
      "prerequisites": prerequisites,
      "verdict": verdict,
      "reasons": reasons,
    }

  def report(self) -> dict[str, Any]:
    if not math.isfinite(self.first_t):
      self.first_t = self.last_t = 0.0
    tx_matches, status_by_id = self._match_button_tx()
    res_groups = self._res_groups(status_by_id)
    episodes = [self._episode_report(start, end, observed, res_groups)
                for start, end, observed in self._stop_episodes()]

    verdict_order = {"FAIL": 4, "PASS": 3, "PASS_AT_LEAST_30S": 2, "INCONCLUSIVE": 1}
    if episodes:
      overall = max((episode["verdict"] for episode in episodes), key=lambda verdict: verdict_order[verdict])
    else:
      overall = "INCONCLUSIVE"

    return {
      "schemaVersion": 1,
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
      "streams": self._stream_report(),
      "decodeErrors": dict(self.decode_errors),
      "rawButtonCounters": self._raw_button_counter_report(),
      "sccEvidence": self._scc_report(),
      "buttonEvidence": self._button_report(),
      "txMatches": tx_matches,
      "resGroups": res_groups,
      "stopEpisodes": episodes,
      "overallVerdict": overall,
      "interpretation": {
        "PASS": "Panda TX-return plus the stock SCC status proves the requested approximately-30s behavior in a qualified stop.",
        "PASS_AT_LEAST_30S": "The warning stayed clear for 30s, but the final stock timeout was not observed, so the exact maximum is unproven.",
        "FAIL": "Direct on-wire or stock-SCC status evidence contradicts the requested behavior.",
        "INCONCLUSIVE": "The capture lacks one or more prerequisites; do not treat absence of an error as success.",
      },
      "limitations": [
        "A sendcan row is only a request; can.src +0x80 is required to prove Panda transmission.",
        "A +0x80 TX-return does not by itself prove SCC ECU acceptance; stock 0x1A0 timing supplies that evidence.",
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


def _demo_car_params() -> Any:
  from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags

  return SimpleNamespace(
    carFingerprint=TARGET_FINGERPRINT,
    brand="hyundai",
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    alphaLongitudinalAvailable=True,
    autoResumeSng=False,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS),
    extFlags=0,
    safetyConfigs=[SimpleNamespace(
      safetyModel="hyundaiCanfd",
      safetyParam=int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS),
    )],
  )


def run_demo(outcome: str) -> ProbeAnalyzer:
  """Generate deterministic, real-DBC frames for an offline dry run."""
  from opendbc.can import CANPacker

  analyzer = ProbeAnalyzer("demo", outcome)
  analyzer.set_car_params(_demo_car_params(), "demo")
  packer = CANPacker(DBC_NAME)
  base = 1_000.0

  moving_state = SimpleNamespace(
    standstill=False, vEgo=1.0, canValid=True, brakePressed=False, gasPressed=False,
    brakeHoldActive=False, parkingBrake=False, accFaulted=False,
    cruiseState=SimpleNamespace(enabled=True, standstill=False),
  )
  analyzer.feed_state(base - 0.05, moving_state)
  analyzer.feed_control(base - 0.05, SimpleNamespace(enabled=True))

  stop_duration = 35.0
  for frame in range(round(stop_duration * 50) + 1):
    elapsed = frame / 50.0
    t = base + elapsed
    info_display = 4 if (elapsed >= (30.0 if outcome == "pass" else 3.0)) else 0
    scc = packer.make_can_msg("SCC_CONTROL", 0, {
      "COUNTER": frame & 0xFF,
      "ACCMode": 1,
      "ACC_ObjDist": 5.0,
      "ACC_ObjRelSpd": 0.0,
      "HUD_LEAD_INFO": 2,
      "InfoDisplay": info_display,
      "SysFailState": 0,
      "TakeOverReq": 0,
    })
    analyzer.feed_can("can", t, 0, scc[0], scc[1])
    stock_button = packer.make_can_msg("CRUISE_BUTTONS_ALT", 0, {
      "COUNTER": frame & 0xFF,
      "CRUISE_BUTTONS": BUTTON_NONE,
      "DISTANCE_UNIT": 1,
      "SET_ME_2": 3,
    })
    analyzer.feed_can("can", t, 0, stock_button[0], stock_button[1])

    if frame % 2 == 0:
      stopped_state = SimpleNamespace(
        standstill=True, vEgo=0.0, canValid=True, brakePressed=False, gasPressed=False,
        brakeHoldActive=False, parkingBrake=False, accFaulted=False,
        cruiseState=SimpleNamespace(enabled=True, standstill=info_display >= 4),
      )
      analyzer.feed_state(t, stopped_state)
      analyzer.feed_control(t, SimpleNamespace(enabled=True))

  if outcome != "missing-tx":
    group_starts = list(EXPECTED_REARM_GROUP_STARTS)
    for group_start in group_starts:
      for offset in (0.0, 0.01, 0.02):
        source_counter = math.floor((group_start + offset) * 50.0 + 1e-6) & 0xFF
        counter = (source_counter + 1) & 0xFF
        tx = packer.make_can_msg("CRUISE_BUTTONS_ALT", 2, {
          "COUNTER": counter,
          "CRUISE_BUTTONS": BUTTON_RES_ACCEL,
          "DISTANCE_UNIT": 1,
          "SET_ME_2": 3,
        })
        request_t = base + group_start + offset
        analyzer.feed_can("sendcan", request_t, 2, tx[0], tx[1])
        echo_src = 0x82 if outcome == "pass" else 0xC2
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

  print("Stop episodes:")
  if not report["stopEpisodes"]:
    print("  none (need carState.standstill && cruiseState.enabled)")
  for index, episode in enumerate(report["stopEpisodes"], 1):
    print(
      f"  #{index}: {episode['duration']:.3f}s verdict={episode['verdict']} ",
      f"0x1A0={episode['sccSamples']} RES-groups={len(episode['resGroups'])}",
      sep="",
    )
    for reason in episode["reasons"]:
      print(f"    - {reason}")
    if episode["warningPeriods"]:
      print(f"    InfoDisplay=4 periods: {episode['warningPeriods']}")
    if episode["returnedSchedule"]:
      print(f"    schedule: {episode['returnedSchedule']}")
    for group in episode["resGroups"]:
      print(
        f"    RES @{group['start'] - episode['start']:.3f}s counters={group['counters']} ",
        f"source={group['sourceCountersPerFrame']} tx={group['statuses']} ",
        f"crc={group['allRequestChecksumsValid']} source+1={group['sourcePlusOneAllFrames']} ",
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
