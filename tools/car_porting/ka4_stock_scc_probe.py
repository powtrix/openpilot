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
host-replacement paths for ADRV_0x161 (including its stock-owned alert, sound,
DAW, and mute fields) when that message exists, and for LFAHDA_CLUSTER. Public KA4
captures show that 0x161 is not present on every recorded variant. Neither
ALERTS_5 nor SCC_CONTROL InfoDisplay has been correlated here with the user's
visible cluster prompt, and neither acknowledges that the SCC ECU reset a
timer.

This is a single-capture CAN observation tool, not a vehicle-acceptance test.
It can report a direct contradiction, a matching synthetic/on-wire schedule,
or missing evidence, but a controlled on-car A/B with synchronized cluster
video is still required. Exit status is 1 for direct FAIL evidence and 2 for
all non-failing observations so automation cannot mistake CAN correlation for
physical acceptance.
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
ADRV_STOCK_OWNED_FIELDS = (
  "ALERTS_1", "ALERTS_2", "ALERTS_3", "ALERTS_4", "ALERTS_5", "MUTE",
  "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4", "DAW_ICON",
)
LFAHDA_STOCK_OWNED_FIELDS = (
  "HDA_OptUsmSta", "LFA_OptUsmSta", "HDA_CntrlModSta", "HDA_InfoPUDis",
  "HDA_AutoSetSpdSta", "HDA_AutoSetSpdUpdtSta", "HDA_AutoSetSpdVal",
  "HDA_LFA_WrnSnd", "HDA_InfoPUDis1", "HDA_TDMRMDclReq",
)

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

REPORT_SCHEMA_VERSION = 5
TARGET_FINGERPRINT = "KIA_CARNIVAL_4TH_GEN"
# Older cereal logs serialized the human-readable platform value.
TARGET_FINGERPRINT_ALIASES = (TARGET_FINGERPRINT, "KIA CARNIVAL 4TH GEN")
MIN_EXACT_PROOF_DURATION = 30.25
EARLIEST_EXPECTED_FINAL_WARNING = 29.5
LATEST_EXPECTED_FINAL_WARNING = 31.5
TX_MATCH_WINDOW = 0.100
MIN_DISTINCT_CAN_FRAME_GAP = 0.005
STOCK_SOURCE_PERIOD = 0.020
CLUSTER_SOURCE_PERIOD = 0.050
CLUSTER_SOURCE_CADENCE_TOLERANCE = 0.015
CONTROL_SERVICE_PERIOD = 0.010
CONTROL_SERVICE_CADENCE_TOLERANCE = 0.005
STOP_BOUNDARY_STATE_WINDOW = 0.050
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


def _is_finite_number(value: Any) -> bool:
  try:
    return math.isfinite(float(value))
  except (TypeError, ValueError):
    return False


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
  kinematics_finite: bool
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
  adrv_stock_owned_values: tuple[tuple[str, float | int], ...]
  lfahda_stock_owned_values: tuple[tuple[str, float | int], ...]


@dataclass(frozen=True)
class CanDecodeErrorSample:
  t: float
  service: str
  origin: str
  src: int
  bus: int
  address: int
  error: str


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
  safety_canfd_alt_button_indices = []
  for index, cfg in enumerate(_safe_get(cp, "safetyConfigs", ()) or ()):
    model = str(_safe_get(cfg, "safetyModel", ""))
    safety_param = int(_safe_get(cfg, "safetyParam", 0))
    safety_configs.append({"model": model, "param": safety_param})
    if (
      model == "hyundaiCanfd"
      and bool(safety_param & int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS))
    ):
      safety_canfd_alt_button_indices.append(index)
  safety_canfd_alt_buttons = bool(safety_canfd_alt_button_indices)
  safety_panda_uniquely_resolved = len(safety_canfd_alt_button_indices) == 1
  active_safety_param = (
    safety_configs[safety_canfd_alt_button_indices[0]]["param"]
    if safety_panda_uniquely_resolved else None
  )
  panda_bus_offset = (
    4 * safety_canfd_alt_button_indices[0] if safety_panda_uniquely_resolved else None
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
    "exactlyOneHyundaiCanfdSafetyAltButtonsConfig": safety_panda_uniquely_resolved,
    "hyundaiCanfdSafetyHda2MatchesCarParams": (
      active_safety_param is not None
      and bool(active_safety_param & int(HyundaiSafetyFlags.CANFD_LKA_STEERING)) == canfd_hda2
    ),
    "hyundaiCanfdSafetyStockLongitudinal": (
      active_safety_param is not None
      and not bool(active_safety_param & int(HyundaiSafetyFlags.LONG))
    ),
    "hyundaiCanfdSafetyNotCameraScc": (
      active_safety_param is not None
      and not bool(active_safety_param & int(HyundaiSafetyFlags.CAMERA_SCC))
    ),
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
    "hyundaiCanfdSafetyAltButtonsConfigIndices": safety_canfd_alt_button_indices,
    "pandaBusOffset": panda_bus_offset,
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
    self.decode_error_samples: list[CanDecodeErrorSample] = []
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
      self.decode_error_samples.append(CanDecodeErrorSample(
        t=t,
        service=service,
        origin=origin,
        src=src,
        bus=bus,
        address=address,
        error=f"{type(exc).__name__}: {exc}",
      ))
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
        adrv_stock_owned_values=tuple(
          (name, values[name]) for name in ADRV_STOCK_OWNED_FIELDS if name in values
        ),
        lfahda_stock_owned_values=tuple(
          (name, values[name]) for name in LFAHDA_STOCK_OWNED_FIELDS if name in values
        ),
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
    raw_v_ego = _safe_get(state, "vEgo", 0.0)
    raw_v_ego_raw = _safe_get(state, "vEgoRaw", raw_v_ego)
    v_ego = _finite_float(raw_v_ego)
    self.states.append(StateSample(
      t=t,
      standstill=bool(_safe_get(state, "standstill", False)),
      cruise_enabled=bool(_safe_get(cruise, "enabled", False)),
      cruise_standstill=bool(_safe_get(cruise, "standstill", False)),
      v_ego=v_ego,
      v_ego_raw=_finite_float(raw_v_ego_raw),
      kinematics_finite=_is_finite_number(raw_v_ego) and _is_finite_number(raw_v_ego_raw),
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

  def _configured_panda_bus_offset(self) -> int | None:
    if not self.car_params:
      return None
    value = self.car_params.get("pandaBusOffset")
    return int(value) if isinstance(value, int) and value >= 0 else None

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
        if 0.0 <= delta <= TX_MATCH_WINDOW:
          candidates.append((delta, index, echo))
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

      delta, index, echo = min(candidates, key=lambda item: item[0])
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

    panda_bus_offset = self._configured_panda_bus_offset()
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    expected_raw_camera_bus = None if panda_bus_offset is None else panda_bus_offset + 2
    expected_host_bus = None if canfd_hda2 or panda_bus_offset is None else panda_bus_offset
    return {
      "topology": {
        "pandaBusOffset": panda_bus_offset,
        "canFdHda2": canfd_hda2,
        "expectedRawCameraBus": expected_raw_camera_bus,
        "expectedHostReplacementBus": expected_host_bus,
        "stockLongBehavior": (
          "HDA2 stock-long leaves raw camera ADRV/LFAHDA traffic on the unmodified forwarding path"
          if canfd_hda2 else
          "HDA1 stock-long replaces observed ADRV/LFAHDA frames on ECAN; changing a received stock-owned field is a FAIL"
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
        if 0.0 <= delta <= TX_MATCH_WINDOW:
          candidates.append((delta, index, echo))
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

      delta, index, echo = min(candidates, key=lambda item: item[0])
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
          start_observed = (
            previous is not None
            and not previous.stop_active
            and previous.can_valid
            and previous.kinematics_finite
            and previous.cruise_enabled
            and not previous.standstill
            and (
              abs(previous.v_ego) > 0.05
              or abs(previous.v_ego_raw) > 0.03
            )
            and not previous.interlock_active
            and sample.can_valid
            and sample.kinematics_finite
            and not sample.interlock_active
            and sample.t - previous.t <= (
              CONTROL_SERVICE_PERIOD + CONTROL_SERVICE_CADENCE_TOLERANCE
            )
          )
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
        sample.kinematics_finite,
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
          "kinematicsFinite": sample.kinematics_finite,
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
      "kinematicsNonFiniteSamples": sum(not sample.kinematics_finite for sample in states),
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

  def _info_display_4_periods(self, samples: list[SccSample], start: float) -> list[dict[str, float]]:
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

    InfoDisplay=4 is an authoritative SCC state input, not proof of a visible
    cluster prompt. The early recovery schedule is
    supported only when that state is already active at the physical stop and
    remains continuously active through the 0.30 s qualification dwell.
    """
    samples = sorted(samples, key=lambda sample: sample.t)
    periods = self._info_display_4_periods(samples, start)
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
  def _counter_sequence_integrity(samples: list[Any], start: float, through: float, *,
                                  expected_period: float, cadence_tolerance: float,
                                  modulus: int = 256) -> dict[str, Any]:
    """Require one strictly ordered, sequential-counter frame per observed tick.

    Coverage alone cannot distinguish a real periodic stream from a stream with
    duplicated same-tick frames. Treat timestamp collisions, implausibly close
    frames, and skipped/repeated counters as ambiguous CAN evidence.
    """
    window = sorted(
      (sample for sample in samples if start <= float(sample.t) <= through),
      key=lambda sample: sample.t,
    )
    timestamp_gaps = [
      curr.t - prev.t for prev, curr in zip(window, window[1:], strict=False)
    ]
    counter_deltas = [
      (curr.counter - prev.counter) % modulus
      for prev, curr in zip(window, window[1:], strict=False)
    ]
    non_increasing_timestamps = sum(gap <= 0.0 for gap in timestamp_gaps)
    too_close_frames = sum(gap < MIN_DISTINCT_CAN_FRAME_GAP for gap in timestamp_gaps)
    cadence_violations = sum(
      abs(gap - expected_period) > cadence_tolerance for gap in timestamp_gaps
    )
    counter_step_violations = sum(delta != 1 for delta in counter_deltas)
    return {
      "sampleCount": len(window),
      "minimumDistinctFrameGapMs": MIN_DISTINCT_CAN_FRAME_GAP * 1000.0,
      "expectedPeriodMs": expected_period * 1000.0,
      "cadenceToleranceMs": cadence_tolerance * 1000.0,
      "minimumObservedGapMs": round(min(timestamp_gaps) * 1000.0, 3) if timestamp_gaps else None,
      "maximumObservedGapMs": round(max(timestamp_gaps) * 1000.0, 3) if timestamp_gaps else None,
      "nonIncreasingTimestampPairs": non_increasing_timestamps,
      "tooCloseFramePairs": too_close_frames,
      "cadenceViolationPairs": cadence_violations,
      "counterStepViolations": counter_step_violations,
      "clean": bool(window) and not (
        non_increasing_timestamps or too_close_frames or cadence_violations or counter_step_violations
      ),
    }

  @staticmethod
  def _time_sequence_integrity(samples: list[Any], start: float, through: float, *,
                               expected_period: float, cadence_tolerance: float) -> dict[str, Any]:
    window = sorted(
      (sample for sample in samples if start <= float(sample.t) <= through),
      key=lambda sample: sample.t,
    )
    timestamp_gaps = [
      curr.t - prev.t for prev, curr in zip(window, window[1:], strict=False)
    ]
    non_increasing_timestamps = sum(gap <= 0.0 for gap in timestamp_gaps)
    cadence_violations = sum(
      abs(gap - expected_period) > cadence_tolerance for gap in timestamp_gaps
    )
    return {
      "sampleCount": len(window),
      "expectedPeriodMs": expected_period * 1000.0,
      "cadenceToleranceMs": cadence_tolerance * 1000.0,
      "minimumObservedGapMs": round(min(timestamp_gaps) * 1000.0, 3) if timestamp_gaps else None,
      "maximumObservedGapMs": round(max(timestamp_gaps) * 1000.0, 3) if timestamp_gaps else None,
      "nonIncreasingTimestampPairs": non_increasing_timestamps,
      "cadenceViolationPairs": cadence_violations,
      "clean": bool(window) and not (non_increasing_timestamps or cadence_violations),
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
    proof_sources = sorted(
      (source for source in sources if start <= source.t <= through),
      key=lambda sample: sample.t,
    )
    all_outputs = sorted(outputs, key=lambda sample: sample.t)
    proof_output_indices = {
      index for index, output in enumerate(all_outputs)
      if start <= output.t <= through
      or any(
        source.counter == output.counter and 0.0 <= output.t - source.t <= TX_MATCH_WINDOW
        for source in proof_sources
      )
    }
    used_outputs: set[int] = set()
    pairs_with_index: list[tuple[ClusterCanSample, int, ClusterCanSample]] = []
    for source in proof_sources:
      candidates = [
        (output.t - source.t, index, output)
        for index, output in enumerate(all_outputs)
        if index in proof_output_indices
        and index not in used_outputs
        and output.counter == source.counter
        and 0.0 <= output.t - source.t <= TX_MATCH_WINDOW
      ]
      if not candidates:
        continue
      _, output_index, output = min(candidates, key=lambda candidate: candidate[0])
      used_outputs.add(output_index)
      pairs_with_index.append((source, output_index, output))

    pairs = [(source, output) for source, _, output in pairs_with_index]
    unused_output_indices = proof_output_indices - used_outputs
    source_count = len(proof_sources)
    paired_count = len(pairs)
    output_count = len(proof_output_indices)
    unused_output_count = len(unused_output_indices)
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
    panda_bus_offset = self._configured_panda_bus_offset()
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    raw_camera_bus = None if panda_bus_offset is None else panda_bus_offset + 2
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
    rejected_cluster_echoes = [
      sample for sample in samples
      if sample.origin in ("tx_rejected", "tx_rejected_returned")
      and sample.address in (ADRV_0X161_ADDRESS, LFAHDA_CLUSTER_ADDRESS)
    ]
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
    raw_adrv_transport_stats = [
      stat for (_, origin, _, bus, address), stat in self.streams.items()
      if origin == "vehicle_rx"
      and bus == raw_camera_bus
      and address == ADRV_0X161_ADDRESS
      and stat.last_t >= start - 0.150
      and stat.first_t <= end + 0.150
    ]
    raw_adrv_transport_observed = bool(raw_adrv_transport_stats)
    host_adrv_transport_stats = [
      stat for (_, origin, _, bus, address), stat in self.streams.items()
      if origin in ("send_request", "tx_returned", "tx_rejected", "tx_rejected_returned")
      and bus == host_bus
      and address == ADRV_0X161_ADDRESS
      and stat.last_t >= start - 0.150
      and stat.first_t <= end + 0.150
    ]
    host_adrv_transport_observed = bool(host_adrv_transport_stats)
    adrv_transport_entries = [
      (origin, bus, stat)
      for (_, origin, _, bus, address), stat in self.streams.items()
      if address == ADRV_0X161_ADDRESS
      and stat.last_t >= start - 0.150
      and stat.first_t <= end + 0.150
    ]
    any_adrv_transport_observed = bool(adrv_transport_entries)
    unexpected_adrv_transport_observed = any(
      not (
        (origin == "vehicle_rx" and bus == raw_camera_bus)
        or (
          not canfd_hda2
          and origin in ("send_request", "tx_returned", "tx_rejected", "tx_rejected_returned")
          and bus == host_bus
        )
      )
      for origin, bus, _ in adrv_transport_entries
    )
    hda_transport_entries = [
      (origin, bus, stat)
      for (_, origin, _, bus, address), stat in self.streams.items()
      if address == LFAHDA_CLUSTER_ADDRESS
      and stat.last_t >= start - 0.150
      and stat.first_t <= end + 0.150
    ]
    unexpected_hda_transport_observed = any(
      not (
        (origin == "vehicle_rx" and bus == raw_camera_bus)
        or (
          not canfd_hda2
          and origin in ("send_request", "tx_returned", "tx_rejected", "tx_rejected_returned")
          and bus == host_bus
        )
      )
      for origin, bus, _ in hda_transport_entries
    )
    cluster_decode_errors = [
      sample for sample in self.decode_error_samples
      if start - 0.150 <= sample.t <= end + 0.150
      and sample.address in (ADRV_0X161_ADDRESS, LFAHDA_CLUSTER_ADDRESS)
    ]
    adrv_decode_errors = [
      sample for sample in cluster_decode_errors if sample.address == ADRV_0X161_ADDRESS
    ]
    raw_adrv_decode_errors = [
      sample for sample in adrv_decode_errors
      if sample.origin == "vehicle_rx" and sample.bus == raw_camera_bus
    ]
    hda_decode_errors = [
      sample for sample in cluster_decode_errors if sample.address == LFAHDA_CLUSTER_ADDRESS
    ]
    raw_hda_decode_errors = [
      sample for sample in hda_decode_errors
      if sample.origin == "vehicle_rx" and sample.bus == raw_camera_bus
    ]
    host_cluster_decode_errors = [
      sample for sample in cluster_decode_errors if sample.origin != "vehicle_rx"
    ]
    rejected_cluster_decode_errors = [
      sample for sample in cluster_decode_errors
      if sample.origin in ("tx_rejected", "tx_rejected_returned")
    ]
    raw_adrv_coverage = self._continuous_coverage(
      raw_adrv, start, coverage_through,
      max_gap=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
      edge_tolerance=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
    )
    raw_hda_coverage = self._continuous_coverage(
      raw_hda, start, coverage_through,
      max_gap=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
      edge_tolerance=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
    )
    host_stream_coverage = {
      name: self._continuous_coverage(
        stream, start, coverage_through,
        max_gap=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
        edge_tolerance=CLUSTER_SOURCE_PERIOD + CLUSTER_SOURCE_CADENCE_TOLERANCE,
      )
      for name, stream in (
        ("adrvRequests", host_adrv_requests),
        ("adrvReturned", host_returned_adrv),
        ("lfaHdaRequests", host_hda_requests),
        ("lfaHdaReturned", host_returned_hda),
      )
    }
    counter_sequence_integrity = {
      "rawAdrv": self._counter_sequence_integrity(
        raw_adrv, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
      "rawLfaHda": self._counter_sequence_integrity(
        raw_hda, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
      "adrvRequests": self._counter_sequence_integrity(
        host_adrv_requests, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
      "adrvReturned": self._counter_sequence_integrity(
        host_returned_adrv, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
      "lfaHdaRequests": self._counter_sequence_integrity(
        host_hda_requests, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
      "lfaHdaReturned": self._counter_sequence_integrity(
        host_returned_hda, start, end,
        expected_period=CLUSTER_SOURCE_PERIOD,
        cadence_tolerance=CLUSTER_SOURCE_CADENCE_TOLERANCE,
      ),
    }
    _, adrv_request_pairing = self._pair_cluster_frames_by_counter(
      raw_adrv, host_adrv_requests, start, coverage_through,
    )
    adrv_return_pairs, adrv_return_pairing = self._pair_cluster_frames_by_counter(
      raw_adrv, host_returned_adrv, start, coverage_through,
    )
    _, adrv_episode_return_pairing = self._pair_cluster_frames_by_counter(
      raw_adrv, host_returned_adrv, start, end,
    )
    _, hda_request_pairing = self._pair_cluster_frames_by_counter(
      raw_hda, host_hda_requests, start, coverage_through,
    )
    hda_return_pairs, hda_return_pairing = self._pair_cluster_frames_by_counter(
      raw_hda, host_returned_hda, start, coverage_through,
    )
    _, hda_episode_return_pairing = self._pair_cluster_frames_by_counter(
      raw_hda, host_returned_hda, start, end,
    )
    exact_adrv_replacement_pairing = (
      adrv_request_pairing["exactOneToOneByCounterAndTime"]
      and adrv_return_pairing["exactOneToOneByCounterAndTime"]
    )
    exact_hda_replacement_pairing = (
      hda_request_pairing["exactOneToOneByCounterAndTime"]
      and hda_return_pairing["exactOneToOneByCounterAndTime"]
    )

    raw_alert5_value5_periods = self._cluster_signal_periods(
      [sample for sample in raw_adrv if start <= sample.t <= end], start, "alert_5",
      USE_SWITCH_OR_PEDAL_TO_ACCELERATE,
    )
    first_raw_alert5_value5 = (
      raw_alert5_value5_periods[0]["startAfterStop"] if raw_alert5_value5_periods else None
    )
    raw_alert5_value5_frames = [sample for sample in raw_adrv
                                if start <= sample.t <= end
                                and sample.alert_5 == USE_SWITCH_OR_PEDAL_TO_ACCELERATE]
    alert5_value5_output_pairs = []
    used_alert5_value5_outputs: set[int] = set()
    for raw in raw_alert5_value5_frames:
      candidates = [
        (sample.t - raw.t, index, sample)
        for index, sample in enumerate(returned_adrv)
        if index not in used_alert5_value5_outputs
        and sample.bus == host_bus
        and sample.counter == raw.counter
        and 0.0 <= sample.t - raw.t <= TX_MATCH_WINDOW
      ]
      if candidates:
        _, index, output = min(candidates, key=lambda candidate: candidate[0])
        used_alert5_value5_outputs.add(index)
        alert5_value5_output_pairs.append((raw, output))
    changed_alert5_value5_frames = sum(output.alert_5 != USE_SWITCH_OR_PEDAL_TO_ACCELERATE
                                       for _, output in alert5_value5_output_pairs)
    preserved_alert5_value5_frames = sum(output.alert_5 == USE_SWITCH_OR_PEDAL_TO_ACCELERATE
                                         for _, output in alert5_value5_output_pairs)
    unpaired_alert5_value5_frames = len(raw_alert5_value5_frames) - len(alert5_value5_output_pairs)

    if raw_adrv:
      adrv_applicability = "OBSERVED_WITH_DECODE_ERRORS" if adrv_decode_errors else "OBSERVED"
    elif raw_adrv_transport_observed:
      adrv_applicability = "OBSERVED_BUT_UNDECODED"
    elif any_adrv_transport_observed:
      adrv_applicability = "EXPECTED_RAW_NOT_OBSERVED_OTHER_TRAFFIC_PRESENT"
    else:
      adrv_applicability = "NOT_OBSERVED_IN_CAPTURE"

    if adrv_applicability == "NOT_OBSERVED_IN_CAPTURE":
      candidate_alert5_correlation = "NOT_OBSERVED"
    elif adrv_decode_errors:
      candidate_alert5_correlation = "UNAVAILABLE_DUE_TO_DECODE_ERRORS"
    elif not raw_adrv:
      candidate_alert5_correlation = "UNAVAILABLE_WITHOUT_DECODED_RAW"
    elif first_raw_alert5_value5 is None:
      candidate_alert5_correlation = (
        "CLEAR_THROUGH_30S" if raw_adrv_coverage["continuous"] else
        "NO_VALUE_5_WITH_INCOMPLETE_COVERAGE"
      )
    elif first_raw_alert5_value5 < EARLIEST_EXPECTED_FINAL_WARNING:
      candidate_alert5_correlation = "EARLY"
    elif first_raw_alert5_value5 > LATEST_EXPECTED_FINAL_WARNING:
      candidate_alert5_correlation = "LATE"
    else:
      candidate_alert5_correlation = "MATCHED_AT_30S"

    if not raw_alert5_value5_frames:
      alert5_value5_preservation = "NOT_APPLICABLE"
      path_preserves_observed_alert5_value5: bool | None = None
    elif canfd_hda2:
      path_preserves_observed_alert5_value5 = not adrv_requests and not returned_adrv
      alert5_value5_preservation = (
        "RAW_PATH_WITHOUT_HOST_REPLACEMENT" if path_preserves_observed_alert5_value5 else
        "UNEXPECTED_HOST_REPLACEMENT"
      )
    else:
      path_preserves_observed_alert5_value5 = (
        unpaired_alert5_value5_frames == 0
        and changed_alert5_value5_frames == 0
        and preserved_alert5_value5_frames == len(raw_alert5_value5_frames)
      )
      alert5_value5_preservation = (
        "PRESERVED" if path_preserves_observed_alert5_value5 else "CHANGED_OR_UNPAIRED"
      )

    hda_all_return_value_pairs = []
    for output in host_returned_hda:
      if not start <= output.t <= end:
        continue
      candidates = [
        raw for raw in raw_hda
        if raw.counter == output.counter and 0.0 <= output.t - raw.t <= TX_MATCH_WINDOW
      ]
      if candidates:
        raw = min(candidates, key=lambda sample: output.t - sample.t)
        hda_all_return_value_pairs.append((raw, output))
    hda_state_mismatches = sum(raw.hda_control_state != output.hda_control_state
                               for raw, output in hda_all_return_value_pairs)
    lfahda_stock_owned_field_mismatch_counts: Counter[str] = Counter()
    lfahda_stock_owned_mismatch_frames = 0
    for raw, output in hda_all_return_value_pairs:
      raw_values = dict(raw.lfahda_stock_owned_values)
      output_values = dict(output.lfahda_stock_owned_values)
      mismatched_fields = [
        name for name in LFAHDA_STOCK_OWNED_FIELDS if raw_values.get(name) != output_values.get(name)
      ]
      if mismatched_fields:
        lfahda_stock_owned_mismatch_frames += 1
        lfahda_stock_owned_field_mismatch_counts.update(mismatched_fields)
    unexpected_hda_returned_frames = hda_episode_return_pairing["unusedOutputFrames"]
    # Check every returned output independently. A one-to-one matcher may
    # legitimately choose the normal return and leave a second, mutated return
    # unused; that extra output must not evade the ALERTS_5 preservation check.
    adrv_all_return_value_pairs = []
    for output in host_returned_adrv:
      if not start <= output.t <= end:
        continue
      candidates = [
        raw for raw in raw_adrv
        if raw.counter == output.counter and 0.0 <= output.t - raw.t <= TX_MATCH_WINDOW
      ]
      if candidates:
        raw = min(candidates, key=lambda sample: output.t - sample.t)
        adrv_all_return_value_pairs.append((raw, output))
    adrv_alert5_mismatches = sum(raw.alert_5 != output.alert_5
                                 for raw, output in adrv_all_return_value_pairs)
    adrv_stock_owned_field_mismatch_counts: Counter[str] = Counter()
    adrv_stock_owned_mismatch_frames = 0
    for raw, output in adrv_all_return_value_pairs:
      raw_values = dict(raw.adrv_stock_owned_values)
      output_values = dict(output.adrv_stock_owned_values)
      mismatched_fields = [
        name for name in ADRV_STOCK_OWNED_FIELDS if raw_values.get(name) != output_values.get(name)
      ]
      if mismatched_fields:
        adrv_stock_owned_mismatch_frames += 1
        adrv_stock_owned_field_mismatch_counts.update(mismatched_fields)
    adrv_alert5_value5_mismatches = sum(
      raw.alert_5 == USE_SWITCH_OR_PEDAL_TO_ACCELERATE
      and output.alert_5 != USE_SWITCH_OR_PEDAL_TO_ACCELERATE
      for raw, output in adrv_all_return_value_pairs
    )
    unexpected_adrv_returned_frames = adrv_episode_return_pairing["unusedOutputFrames"]
    if raw_alert5_value5_frames and not canfd_hda2 and adrv_alert5_value5_mismatches:
      path_preserves_observed_alert5_value5 = False
      alert5_value5_preservation = "CHANGED_OR_UNPAIRED"

    if not raw_alert5_value5_frames:
      alert5_value_path = "alert5Value5NotObserved"
    elif canfd_hda2:
      alert5_value_path = (
        "hda2RawAlert5Value5NoHostReplacement"
        if path_preserves_observed_alert5_value5 else
        "hda2UnexpectedHostReplacement"
      )
    elif adrv_alert5_value5_mismatches:
      alert5_value_path = "hostReplacementChangedAlert5Value5"
    elif unexpected_adrv_returned_frames:
      alert5_value_path = "hostReplacementHasUnexpectedReturnedFrames"
    elif unpaired_alert5_value5_frames:
      alert5_value_path = "hostReplacementMissingAlert5Value5Pair"
    elif path_preserves_observed_alert5_value5:
      alert5_value_path = "hostReplacementPreservedAlert5Value5"
    else:
      alert5_value_path = "rawAlert5Value5ObservedDeliveryUnproven"

    all_adrv_returned = (
      all(tx_status_by_id.get(id(request)) == "returned" for request in adrv_requests)
      if adrv_requests else None
    )
    all_hda_returned = (
      all(tx_status_by_id.get(id(request)) == "returned" for request in hda_requests)
      if hda_requests else None
    )
    if unexpected_adrv_transport_observed:
      adrv_path_consistent = False
    elif adrv_decode_errors:
      adrv_path_consistent = False
    elif raw_adrv:
      if canfd_hda2:
        adrv_path_consistent = (
          raw_adrv_coverage["continuous"]
          and counter_sequence_integrity["rawAdrv"]["clean"]
          and not adrv_requests
          and not returned_adrv
        )
      else:
        adrv_path_consistent = (
          raw_adrv_coverage["continuous"]
          and host_stream_coverage["adrvRequests"]["continuous"]
          and host_stream_coverage["adrvReturned"]["continuous"]
          and counter_sequence_integrity["rawAdrv"]["clean"]
          and counter_sequence_integrity["adrvRequests"]["clean"]
          and counter_sequence_integrity["adrvReturned"]["clean"]
          and exact_adrv_replacement_pairing
          and all_adrv_returned
          and adrv_stock_owned_mismatch_frames == 0
          and adrv_alert5_mismatches == 0
          and unexpected_adrv_returned_frames == 0
          and all(request.bus == host_bus for request in adrv_requests)
          and all(sample.bus == host_bus for sample in returned_adrv)
        )
    elif raw_adrv_transport_observed:
      # A raw transport stream without decoded samples is not equivalent to
      # a vehicle variant that does not carry 0x161. Keep the proof
      # inconclusive so a DBC/length/decode failure cannot look like absence.
      adrv_path_consistent = False
    else:
      # Current code does not synthesize 0x161 when the received variant does
      # not carry it. Absence is therefore a supported topology, not a missing
      # generic KA4 proof prerequisite.
      adrv_path_consistent = (
        not host_adrv_transport_observed
        and not adrv_requests
        and not returned_adrv
      )

    if unexpected_hda_transport_observed:
      lfa_hda_path_consistent = False
    elif hda_decode_errors:
      lfa_hda_path_consistent = False
    elif canfd_hda2:
      lfa_hda_path_consistent = (
        raw_hda_coverage["continuous"]
        and counter_sequence_integrity["rawLfaHda"]["clean"]
        and not hda_requests
        and not returned_hda
      )
    else:
      lfa_hda_path_consistent = (
        raw_hda_coverage["continuous"]
        and host_stream_coverage["lfaHdaRequests"]["continuous"]
        and host_stream_coverage["lfaHdaReturned"]["continuous"]
        and counter_sequence_integrity["rawLfaHda"]["clean"]
        and counter_sequence_integrity["lfaHdaRequests"]["clean"]
        and counter_sequence_integrity["lfaHdaReturned"]["clean"]
        and exact_hda_replacement_pairing
        and all_hda_returned
        and lfahda_stock_owned_mismatch_frames == 0
        and hda_state_mismatches == 0
        and unexpected_hda_returned_frames == 0
        and all(request.bus == host_bus for request in hda_requests)
        and all(sample.bus == host_bus for sample in returned_hda)
      )
    hda_path_consistent = (
      adrv_path_consistent
      and lfa_hda_path_consistent
      and path_preserves_observed_alert5_value5 is not False
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
    alert5_value5_after_last_res = (
      round(first_raw_alert5_value5 - last_res_after_stop, 3)
      if first_raw_alert5_value5 is not None and last_res_after_stop is not None else None
    )
    return {
      "rawCameraBus": raw_camera_bus,
      "hostReplacementBus": None if canfd_hda2 else host_bus,
      "canFdHda2": canfd_hda2,
      "adrv0x161Applicability": adrv_applicability,
      "rawAdrvTransportObserved": raw_adrv_transport_observed,
      "hostAdrvTransportObserved": host_adrv_transport_observed,
      "anyAdrvTransportObserved": any_adrv_transport_observed,
      "unexpectedAdrvTransportObserved": unexpected_adrv_transport_observed,
      "unexpectedLfaHdaTransportObserved": unexpected_hda_transport_observed,
      "rawAdrvSamples": len(raw_adrv),
      "rawLfaHdaSamples": len(raw_hda),
      "rawAdrvContinuousThrough30_25s": raw_adrv_coverage["continuous"],
      "rawLfaHdaContinuousThrough30_25s": raw_hda_coverage["continuous"],
      "rawAdrvCoverage": raw_adrv_coverage,
      "rawLfaHdaCoverage": raw_hda_coverage,
      "hostReplacementCoverage": host_stream_coverage,
      "counterSequenceIntegrity": counter_sequence_integrity,
      "hostReplacementPairing": {
        "adrvRawToRequest": adrv_request_pairing,
        "adrvRawToReturned": adrv_return_pairing,
        "lfaHdaRawToRequest": hda_request_pairing,
        "lfaHdaRawToReturned": hda_return_pairing,
        "exactAdrvRawRequestReturnedPairing": exact_adrv_replacement_pairing,
        "exactLfaHdaRawRequestReturnedPairing": exact_hda_replacement_pairing,
      },
      "integrity": {
        "rawAdrvAllCrcValid": (
          not raw_adrv_decode_errors
          and all(sample.checksum_valid is True for sample in raw_adrv) if raw_adrv else None
        ),
        "rawLfaHdaAllCrcValid": (
          bool(raw_hda)
          and not raw_hda_decode_errors
          and all(sample.checksum_valid is True for sample in raw_hda)
        ),
        "hostReplacementAllCrcValid": (
          (not replacement_cluster_samples and not host_cluster_decode_errors) if canfd_hda2 else
          bool(replacement_cluster_samples)
          and not host_cluster_decode_errors
          and all(sample.checksum_valid is True for sample in replacement_cluster_samples)
        ),
        "invalidSampleCount": len(invalid_cluster_samples),
        "decodeErrorCount": len(cluster_decode_errors),
        "rejectedRequestCount": len(rejected_cluster_requests),
        "rejectedEchoCount": len(rejected_cluster_echoes) + len(rejected_cluster_decode_errors),
      },
      "candidateAlert5Correlation": candidate_alert5_correlation,
      "rawAlert5Value5Periods": raw_alert5_value5_periods,
      "firstRawAlert5Value5AfterStop": first_raw_alert5_value5,
      "alert5ValuePathObservation": alert5_value_path,
      "alert5ValuePairCounts": {
        "rawAlert5Value5Frames": len(raw_alert5_value5_frames),
        "pairedWithTxReturn": len(alert5_value5_output_pairs),
        "unpairedRawAlert5Value5Frames": unpaired_alert5_value5_frames,
        "changedByReturnedFrame": changed_alert5_value5_frames,
        "allReturnedValueMismatches": adrv_alert5_mismatches,
        "allReturnedValue5Mismatches": adrv_alert5_value5_mismatches,
        "preservedByReturnedFrame": preserved_alert5_value5_frames,
      },
      "alert5ValuePreservation": {
        "assessment": alert5_value5_preservation,
        "requiredForObservedValue5": bool(raw_alert5_value5_frames),
        "allRawValue5FramesPairedByCounterAndTime": (
          unpaired_alert5_value5_frames == 0
          if raw_alert5_value5_frames and not canfd_hda2 else None
        ),
        "allPairedReturnedFramesPreserveValue5": (
          changed_alert5_value5_frames == 0
          if raw_alert5_value5_frames and not canfd_hda2 else None
        ),
        "pathPreservesObservedValue5": path_preserves_observed_alert5_value5,
      },
      "hdaControlStateCounts": {str(value): count for value, count in sorted(Counter(
        sample.hda_control_state for sample in raw_hda).items(), key=lambda item: str(item[0]))},
      "hdaControlStateTransitions": raw_hda_transitions[:100],
      "hdaReplacementComparison": {
        "pairedFrames": len(hda_return_pairs),
        "stateMismatches": hda_state_mismatches,
        "lfaHdaStockOwnedFieldMismatchFrames": lfahda_stock_owned_mismatch_frames,
        "lfaHdaStockOwnedFieldMismatchCounts": dict(sorted(lfahda_stock_owned_field_mismatch_counts.items())),
        "unexpectedLfaHdaReturnedFrames": unexpected_hda_returned_frames,
        "adrvAlert5ValueMismatches": adrv_alert5_mismatches,
        "adrvStockOwnedFieldMismatchFrames": adrv_stock_owned_mismatch_frames,
        "adrvStockOwnedFieldMismatchCounts": dict(sorted(adrv_stock_owned_field_mismatch_counts.items())),
        "unexpectedAdrvReturnedFrames": unexpected_adrv_returned_frames,
        "allAdrvRequestsReturned": all_adrv_returned,
        "allLfaHdaRequestsReturned": all_hda_returned,
        "adrvPathConsistentWithObservedVariant": adrv_path_consistent,
        "lfaHdaPathConsistent": lfa_hda_path_consistent,
        "pathConsistentWithCurrentStockLongTopology": hda_path_consistent,
      },
      "postResObservation": {
        "lastResAfterStop": last_res_after_stop,
        "firstRawAlert5Value5AfterLastRes": alert5_value5_after_last_res,
        "note": "Temporal ordering is observation only; it is not an SCC timer-reset acknowledgement.",
      },
    }

  def _episode_report(self, start: float, end: float, start_observed: bool,
                      res_groups: list[dict[str, Any]], button_tx_status_by_id: dict[int, str],
                      cluster_tx_status_by_id: dict[int, str]) -> dict[str, Any]:
    evidence_start = start - PROOF_STREAM_EDGE_TOLERANCE
    evidence_end = end + PROOF_STREAM_EDGE_TOLERANCE
    panda_bus_offset = self._configured_panda_bus_offset()
    canfd_hda2 = bool(self.car_params and self.car_params.get("canFdHda2"))
    expected_stock_bus = (
      None if panda_bus_offset is None else panda_bus_offset + (1 if canfd_hda2 else 0)
    )
    states = [sample for sample in self.states if start <= sample.t <= end]
    stock_bus = self._stock_rx_bus()
    scc = [sample for sample in self.scc if start <= sample.t <= end and
           (stock_bus is None or sample.bus == stock_bus)]
    scc_evidence_samples = [
      sample for sample in self.scc
      if evidence_start <= sample.t <= evidence_end
      and (stock_bus is None or sample.bus == stock_bus)
    ]
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
    raw_alt_evidence_samples = [
      sample for sample in self.buttons
      if evidence_start <= sample.t <= evidence_end
      and sample.origin == "vehicle_rx"
      and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
      and sample.bus == stock_bus
    ]
    rearm_requests = [sample for sample in self.buttons if start <= sample.t <= end and
                      sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL]
    host_button_requests = [
      sample for sample in self.buttons
      if start <= sample.t <= end and sample.origin == "send_request"
      and sample.address in (CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
    ]
    returned_button_echoes = [
      sample for sample in self.buttons
      if start <= sample.t <= end and sample.origin == "tx_returned"
      and sample.address in (CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
    ]
    boundary_margin_host_button_traffic = [
      sample for sample in self.buttons
      if evidence_start <= sample.t <= evidence_end
      and not start <= sample.t <= end
      and sample.origin != "vehicle_rx"
      and sample.address in (CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
    ]
    unexpected_host_button_requests = [
      sample for sample in host_button_requests
      if sample.address != CRUISE_BUTTONS_ALT_ADDRESS or sample.button != BUTTON_RES_ACCEL
    ]
    unexpected_returned_button_echoes = [
      sample for sample in returned_button_echoes
      if sample.address != CRUISE_BUTTONS_ALT_ADDRESS or sample.button != BUTTON_RES_ACCEL
    ]
    episode_decode_errors = [
      sample for sample in self.decode_error_samples
      if evidence_start <= sample.t <= evidence_end
    ]
    scc_decode_errors = [
      sample for sample in episode_decode_errors
      if sample.origin == "vehicle_rx"
      and sample.address == SCC_CONTROL_ADDRESS
      and (stock_bus is None or sample.bus == stock_bus)
    ]
    raw_alt_decode_errors = [
      sample for sample in episode_decode_errors
      if sample.origin == "vehicle_rx"
      and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
      and (stock_bus is None or sample.bus == stock_bus)
    ]
    rearm_decode_errors = [
      sample for sample in episode_decode_errors
      if sample.origin != "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
    ]
    rejected_button_echoes = [
      sample for sample in self.buttons
      if start <= sample.t <= end
      and sample.origin in ("tx_rejected", "tx_rejected_returned")
      and sample.address in (CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
    ]
    rejected_button_decode_errors = [
      sample for sample in episode_decode_errors
      if sample.origin in ("tx_rejected", "tx_rejected_returned")
      and sample.address in (CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
    ]
    rejected_button_echo_count = len(rejected_button_echoes) + len(rejected_button_decode_errors)
    relevant_transport_entries = [
      (origin, bus, address, stat)
      for (_, origin, _, bus, address), stat in self.streams.items()
      if address in (SCC_CONTROL_ADDRESS, CRUISE_BUTTONS_ALT_ADDRESS, CRUISE_BUTTONS_ADDRESS)
      and stat.last_t >= evidence_start
      and stat.first_t <= evidence_end
    ]
    unexpected_scc_transport_observed = any(
      address == SCC_CONTROL_ADDRESS
      and (origin != "vehicle_rx" or stock_bus is None or bus != stock_bus)
      for origin, bus, address, _ in relevant_transport_entries
    )
    unexpected_raw_alt_transport_observed = any(
      address == CRUISE_BUTTONS_ALT_ADDRESS
      and origin == "vehicle_rx"
      and (stock_bus is None or bus != stock_bus)
      for origin, bus, address, _ in relevant_transport_entries
    )
    raw_standard_transport_observed = any(
      origin == "vehicle_rx"
      and address == CRUISE_BUTTONS_ADDRESS
      for origin, _, address, _ in relevant_transport_entries
    )

    safe_scc = [sample for sample in scc if sample.raw_lead_safe]
    valid_state = [sample for sample in states if sample.can_valid]
    no_interlock = [sample for sample in states if not sample.interlock_active]
    enabled_controls = [sample for sample in controls if sample.enabled]
    control_cancel_clear = bool(controls) and all(not sample.cancel for sample in controls)
    raw_driver_buttons_clear = bool(raw_alt_stock_bus) and not raw_alt_decode_errors and all(
      sample.button == BUTTON_NONE
      and sample.adaptive_main == 0
      and sample.normal_main == 0
      and sample.lfa_button == 0
      for sample in raw_alt_stock_bus
    )
    schedule_mode, info_display_4_periods = self._schedule_mode(scc, start)
    cluster_evidence = self._cluster_episode_evidence(start, end, cluster_tx_status_by_id)
    first_raw_alert5_value5 = cluster_evidence["firstRawAlert5Value5AfterStop"]

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
      "carState": self._continuous_coverage(
        states, start, coverage_through,
        max_gap=CONTROL_SERVICE_PERIOD + CONTROL_SERVICE_CADENCE_TOLERANCE,
        edge_tolerance=CONTROL_SERVICE_PERIOD + CONTROL_SERVICE_CADENCE_TOLERANCE,
      ),
      "carControl": self._continuous_coverage(
        controls, start, coverage_through,
        max_gap=CONTROL_SERVICE_PERIOD + CONTROL_SERVICE_CADENCE_TOLERANCE,
        edge_tolerance=CONTROL_SERVICE_PERIOD + CONTROL_SERVICE_CADENCE_TOLERANCE,
      ),
      "rawScc0x1a0": self._continuous_coverage(
        scc, start, coverage_through,
        max_gap=STOCK_SOURCE_PERIOD + BUTTON_SOURCE_CADENCE_TOLERANCE,
        edge_tolerance=STOCK_SOURCE_PERIOD + BUTTON_SOURCE_CADENCE_TOLERANCE,
      ),
      "rawStockButtons0x1aa": self._continuous_coverage(
        raw_alt_stock_bus, start, coverage_through,
        max_gap=STOCK_SOURCE_PERIOD + BUTTON_SOURCE_CADENCE_TOLERANCE,
        edge_tolerance=STOCK_SOURCE_PERIOD + BUTTON_SOURCE_CADENCE_TOLERANCE,
      ),
    }
    boundary_states = [
      sample for sample in self.states
      if start - STOP_BOUNDARY_STATE_WINDOW <= sample.t <= start
    ]
    boundary_state_integrity = self._time_sequence_integrity(
      boundary_states,
      start - STOP_BOUNDARY_STATE_WINDOW,
      start,
      expected_period=CONTROL_SERVICE_PERIOD,
      cadence_tolerance=CONTROL_SERVICE_CADENCE_TOLERANCE,
    )
    boundary_state_integrity["enoughSamples"] = len(boundary_states) >= 2
    boundary_state_integrity["allCanValid"] = (
      bool(boundary_states) and all(sample.can_valid for sample in boundary_states)
    )
    boundary_state_integrity["allKinematicsFinite"] = (
      bool(boundary_states) and all(sample.kinematics_finite for sample in boundary_states)
    )
    service_cadence_integrity = {
      "carState": self._time_sequence_integrity(
        states, start, end,
        expected_period=CONTROL_SERVICE_PERIOD,
        cadence_tolerance=CONTROL_SERVICE_CADENCE_TOLERANCE,
      ),
      "carControl": self._time_sequence_integrity(
        controls, start, end,
        expected_period=CONTROL_SERVICE_PERIOD,
        cadence_tolerance=CONTROL_SERVICE_CADENCE_TOLERANCE,
      ),
    }
    source_counter_sequence_integrity = {
      "rawScc0x1a0": self._counter_sequence_integrity(
        scc_evidence_samples, evidence_start, evidence_end,
        expected_period=STOCK_SOURCE_PERIOD,
        cadence_tolerance=BUTTON_SOURCE_CADENCE_TOLERANCE,
      ),
      "rawStockButtons0x1aa": self._counter_sequence_integrity(
        raw_alt_evidence_samples, evidence_start, evidence_end,
        expected_period=STOCK_SOURCE_PERIOD,
        cadence_tolerance=BUTTON_SOURCE_CADENCE_TOLERANCE,
      ),
    }
    gate_passed = bool(self.car_params and self.car_params.get("ka4StockSccGatePassed"))
    state_coverage_ok = bool(states) and len(valid_state) == len(states)
    state_kinematics_finite = bool(states) and all(sample.kinematics_finite for sample in states)
    no_interlocks = (
      bool(states)
      and len(no_interlock) == len(states)
      and control_cancel_clear
      and raw_driver_buttons_clear
    )
    control_coverage_ok = bool(controls) and len(enabled_controls) == len(controls)
    raw_scc_coverage_ok = bool(scc) and not scc_decode_errors and len(safe_scc) == len(scc)
    direct_stopped_motion_violation = any(
      sample.kinematics_finite
      and (
        not sample.standstill
        or abs(sample.v_ego) > 0.05
        or abs(sample.v_ego_raw) > 0.03
      )
      for sample in states
    )
    stopped_motion_ok = bool(states) and state_kinematics_finite and not direct_stopped_motion_violation
    exact_duration = duration >= MIN_EXACT_PROOF_DURATION
    all_tx_returned = bool(episode_groups) and all(group["allReturned"] for group in episode_groups)
    returned_button_request_count = sum(
      button_tx_status_by_id.get(id(request)) == "returned" for request in host_button_requests
    )
    button_request_return_one_to_one = (
      bool(host_button_requests)
      and returned_button_request_count == len(host_button_requests) == len(returned_button_echoes)
    )
    any_rejected = (
      any(group["statuses"].get("rejected", 0) for group in episode_groups)
      or rejected_button_echo_count > 0
    )
    all_scc_crc_valid = (
      bool(scc_evidence_samples)
      and not scc_decode_errors
      and all(sample.checksum_valid is True for sample in scc_evidence_samples)
    )
    all_raw_button_crc_valid = bool(raw_alt_evidence_samples) and not raw_alt_decode_errors and all(
      sample.checksum_valid is True for sample in raw_alt_evidence_samples
    )
    all_rearm_crc_valid = (
      bool(episode_groups)
      and not rearm_decode_errors
      and all(group["allRequestChecksumsValid"] for group in episode_groups)
    )
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
    expected_send_bus = (
      stock_bus if canfd_hda2 else
      None if panda_bus_offset is None else panda_bus_offset + 2
    )
    send_bus_matches = (
      expected_send_bus is not None
      and bool(rearm_requests)
      and all(request.bus == expected_send_bus for request in rearm_requests)
    )
    same_safety_panda = (
      stock_bus is not None
      and panda_bus_offset is not None
      and stock_bus // 4 == panda_bus_offset // 4
      and bool(rearm_requests)
      and all(request.bus // 4 == panda_bus_offset // 4 for request in rearm_requests)
    )
    exact_alt_layout = (
      bool(raw_alt_stock_bus)
      and stock_bus == expected_stock_bus
      and not raw_standard_transport_observed
      and not unexpected_raw_alt_transport_observed
      and send_bus_matches
      and same_safety_panda
      and all(request.address == CRUISE_BUTTONS_ALT_ADDRESS for request in rearm_requests)
    )

    # Look just beyond the final standstill sample. Capping this window at
    # ``start + 30`` makes it empty for every proof-length episode (whose end
    # is already later than 30 s), silently hiding a false start at release.
    next_states = [sample for sample in self.states if end < sample.t <= end + 0.250]
    post_stop_observation_available = bool(next_states)
    post_stop_states_finite = bool(next_states) and all(sample.kinematics_finite for sample in next_states)
    post_stop_states_valid = bool(next_states) and all(sample.can_valid for sample in next_states)
    last_safe_scc = scc[-1].raw_lead_safe if scc else False
    potential_false_start = any(
      sample.can_valid
      and sample.kinematics_finite
      and not sample.standstill
      and sample.cruise_enabled
      and (abs(sample.v_ego) > 0.05 or abs(sample.v_ego_raw) > 0.03)
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
      any(sample.checksum_valid is False for sample in scc_evidence_samples)
      or any(sample.checksum_valid is False for sample in raw_alt_evidence_samples)
      or any(not group["allRequestChecksumsValid"] for group in episode_groups)
      or cluster_evidence["integrity"]["invalidSampleCount"] > 0
    )
    any_cluster_rejected = (
      cluster_evidence["integrity"]["rejectedRequestCount"] > 0
      or cluster_evidence["integrity"]["rejectedEchoCount"] > 0
    )
    direct_alert5_value_mutation = (
      not canfd_hda2
      and cluster_evidence["hdaReplacementComparison"]["adrvAlert5ValueMismatches"] > 0
    )
    direct_adrv_stock_owned_field_mutation = (
      not canfd_hda2
      and cluster_evidence["hdaReplacementComparison"]["adrvStockOwnedFieldMismatchFrames"] > 0
    )
    direct_hda_control_state_mutation = (
      not canfd_hda2
      and cluster_evidence["hdaReplacementComparison"]["stateMismatches"] > 0
    )
    direct_lfahda_stock_owned_field_mutation = (
      not canfd_hda2
      and cluster_evidence["hdaReplacementComparison"]["lfaHdaStockOwnedFieldMismatchFrames"] > 0
    )
    direct_unexpected_host_button_action = bool(
      unexpected_host_button_requests or unexpected_returned_button_echoes
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
      "physicalStopBoundaryStateSequenceUnambiguous": (
        boundary_state_integrity["clean"] and boundary_state_integrity["enoughSamples"]
      ),
      "physicalStopBoundaryStatesValidAndFinite": (
        boundary_state_integrity["allCanValid"]
        and boundary_state_integrity["allKinematicsFinite"]
      ),
      "durationAtLeast30_25s": exact_duration,
      "carStateContinuousCoverage": stream_coverage["carState"]["continuous"],
      "carControlContinuousCoverage": stream_coverage["carControl"]["continuous"],
      "carStateCadenceUnambiguous": service_cadence_integrity["carState"]["clean"],
      "carControlCadenceUnambiguous": service_cadence_integrity["carControl"]["clean"],
      "rawScc0x1a0ContinuousCoverage": stream_coverage["rawScc0x1a0"]["continuous"],
      "rawStockButtons0x1aaContinuousCoverage": stream_coverage["rawStockButtons0x1aa"]["continuous"],
      "preferredStockRxBusMatchesCarParamsEcan": (
        expected_stock_bus is not None and stock_bus == expected_stock_bus
      ),
      "stockScc0x1a0TransportOnlyOnPreferredRxBus": not unexpected_scc_transport_observed,
      "rawStockButtons0x1aaTransportOnlyOnPreferredRxBus": not unexpected_raw_alt_transport_observed,
      "rawStandardButtons0x1cfAbsentOnAllRxBuses": not raw_standard_transport_observed,
      "rawScc0x1a0CounterSequenceUnambiguous": source_counter_sequence_integrity[
        "rawScc0x1a0"]["clean"],
      "rawStockButtons0x1aaCounterSequenceUnambiguous": source_counter_sequence_integrity[
        "rawStockButtons0x1aa"]["clean"],
      "carStateCanValidAllSamples": state_coverage_ok,
      "carStateKinematicsFiniteAllSamples": state_kinematics_finite,
      "driverInterlocksClearAllSamples": no_interlocks,
      "carControlEnabledAllSamples": control_coverage_ok,
      "wheelStandstillAndRawSpeedNearZeroAllSamples": stopped_motion_ok,
      "noPotentialFalseStart": not potential_false_start,
      "postStopObservationAvailable": post_stop_observation_available,
      "postStopObservationKinematicsFinite": post_stop_states_finite,
      "postStopObservationCanValidAllSamples": post_stop_states_valid,
      "rawSccLeadGateValidAllSamples": raw_scc_coverage_ok,
      "raw0x1aaStockBusPresentAnd0x1cfAbsent": (
        bool(raw_alt_stock_bus) and not raw_standard_transport_observed
      ),
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
      "buttonRequestsAndReturnedEchoesOneToOne": button_request_return_one_to_one,
      "noHostButtonTrafficInEpisodeBoundaryMargin": not boundary_margin_host_button_traffic,
      "onlyResAccelHostButtonRequestsAndReturns": not direct_unexpected_host_button_action,
      "allProofRelevantCanFramesDecoded": (
        not episode_decode_errors
        and cluster_evidence["integrity"]["decodeErrorCount"] == 0
      ),
      "rawLfaHda0x1e0ContinuousThrough30_25s": cluster_evidence["rawLfaHdaContinuousThrough30_25s"],
      "rawLfaHda0x1e0Observed": cluster_evidence["rawLfaHdaSamples"] > 0,
      "rawLfaHda0x1e0CrcValidAllSamples": cluster_evidence["integrity"]["rawLfaHdaAllCrcValid"],
      "hostClusterReplacementCrcValidAllSamples": cluster_evidence["integrity"]["hostReplacementAllCrcValid"],
      "clusterCanDecodedAllObservedSamples": cluster_evidence["integrity"]["decodeErrorCount"] == 0,
      "adrvAndLfaHdaPathsConsistentWithObservedVariant": cluster_evidence[
        "hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"],
    }
    signal_specific_checks = {
      "adrv0x161": {
        "applicability": cluster_evidence["adrv0x161Applicability"],
        "continuousThrough30_25s": (
          cluster_evidence["rawAdrvContinuousThrough30_25s"]
          if cluster_evidence["rawAdrvSamples"] else None
        ),
        "crcValidAllSamples": cluster_evidence["integrity"]["rawAdrvAllCrcValid"],
        "candidateAlert5Correlation": cluster_evidence["candidateAlert5Correlation"],
        "pathPreservesObservedValue5": cluster_evidence[
          "alert5ValuePreservation"]["pathPreservesObservedValue5"],
      },
    }

    verdict = "INCONCLUSIVE"
    can_evidence_verdict = "INCONCLUSIVE"
    reasons = []
    if any_rejected or any_cluster_rejected:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("Panda safety rejected one or more 0x1AA RES or cluster replacement requests")
    elif direct_unexpected_host_button_action:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("host button traffic contained an action other than the supported 0x1AA RES schedule")
    elif direct_alert5_value_mutation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("HDA1 returned replacement changed an observed ADRV_0x161 ALERTS_5 value")
    elif direct_adrv_stock_owned_field_mutation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      changed_fields = ", ".join(
        cluster_evidence["hdaReplacementComparison"]["adrvStockOwnedFieldMismatchCounts"]
      )
      reasons.append(f"HDA1 returned replacement changed received ADRV fields: {changed_fields}")
    elif direct_hda_control_state_mutation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("HDA1 returned replacement changed the received LFAHDA HDA_CntrlModSta value")
    elif direct_lfahda_stock_owned_field_mutation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      changed_fields = ", ".join(
        cluster_evidence["hdaReplacementComparison"]["lfaHdaStockOwnedFieldMismatchCounts"]
      )
      reasons.append(f"HDA1 returned replacement changed received LFAHDA fields: {changed_fields}")
    elif direct_schedule_violation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("0x1AA RES traffic exceeded the supported schedule or continued after 27.00s")
    elif direct_crc_failure:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("an SCC, button, ADRV, or LFAHDA frame failed the Hyundai CAN-FD CRC")
    elif direct_source_policy_failure:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append(
        "a RES group violated fresh sequential source/emitted counters, source+1, or stock non-button fields"
      )
    elif potential_false_start:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("vehicle moved while the stationary-lead gate was still valid; potential false start")
    elif direct_stopped_motion_violation:
      verdict = "FAIL"
      can_evidence_verdict = "FAIL"
      reasons.append("standstill evidence contained motion above the vEgo/vEgoRaw gate or a false standstill sample")
    elif schedule_mode == SCHEDULE_MODE_MIXED:
      reasons.append(
        "InfoDisplay=4 changed during the pre-boundary stop window, so neither supported schedule can be matched"
      )
    elif schedule_mode == SCHEDULE_MODE_UNKNOWN:
      reasons.append("authoritative SCC status did not cover the physical stop boundary")
    elif all(prerequisites.values()):
      alert5_correlation = cluster_evidence["candidateAlert5Correlation"]
      if alert5_correlation == "MATCHED_AT_30S":
        can_evidence_verdict = "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
        reasons.append(
          " ".join((
            f"the returned {schedule_mode} 0x1AA schedule and raw ADRV ALERTS_5=5 observation aligned",
            "near 30s; this is CAN-field correlation, not a visible-prompt or timer-reset acknowledgement",
          ))
        )
      elif alert5_correlation == "CLEAR_THROUGH_30S":
        can_evidence_verdict = "OBSERVED_SCHEDULE_ALERT5_CLEAR_THROUGH_30S"
        reasons.append(
          "the returned RES schedule matched and a continuously observed 0x161 stream contained no ALERTS_5=5 through 30s"
        )
      elif alert5_correlation == "NOT_OBSERVED":
        can_evidence_verdict = "OBSERVED_RES_SCHEDULE_ONLY"
        reasons.append(
          "the returned RES schedule matched, but the capture contained no raw ADRV_0x161"
        )
      elif alert5_correlation in ("EARLY", "LATE"):
        can_evidence_verdict = "OBSERVED_SCHEDULE_WITH_ALERT5_TIMING_DIFFERENCE"
        assert first_raw_alert5_value5 is not None
        reasons.append(
          f"ADRV_0x161 ALERTS_5 first equaled 5 at {first_raw_alert5_value5:.3f}s; " +
          "its visible-cluster applicability is not established"
        )
      else:
        can_evidence_verdict = "OBSERVED_RES_SCHEDULE_ONLY"
        reasons.append("the returned RES schedule matched, but 0x161 coverage was incomplete")
      reasons.append(
        "vehicle behavior remains inconclusive until a controlled on-car A/B and synchronized cluster video"
      )
    else:
      failed = [name for name, passed in prerequisites.items() if not passed]
      reasons.append("missing CAN-observation prerequisites: " + ", ".join(failed))

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
      "serviceCadenceIntegrity": service_cadence_integrity,
      "physicalStopBoundaryStateIntegrity": boundary_state_integrity,
      "sourceCounterSequenceIntegrity": source_counter_sequence_integrity,
      "decodeErrors": [
        {
          **asdict(sample),
          "t": self._rel(sample.t),
          "addressHex": f"0x{sample.address:X}",
        }
        for sample in episode_decode_errors[:100]
      ],
      "rejectedButtonEchoCount": rejected_button_echo_count,
      "hostButtonTraffic": {
        "requestCount": len(host_button_requests),
        "returnedEchoCount": len(returned_button_echoes),
        "matchedReturnedRequestCount": returned_button_request_count,
        "unexpectedRequestCount": len(unexpected_host_button_requests),
        "unexpectedReturnedEchoCount": len(unexpected_returned_button_echoes),
        "oneToOne": button_request_return_one_to_one,
        "boundaryMarginTrafficCount": len(boundary_margin_host_button_traffic),
      },
      "busLayout": {
        "stockBus": stock_bus,
        "expectedStockBus": expected_stock_bus,
        "pandaBusOffset": panda_bus_offset,
        "canFdHda2": canfd_hda2,
        "expectedSendBus": expected_send_bus,
        "observedSendBuses": sorted({request.bus for request in rearm_requests}),
        "unexpectedSccTransportObserved": unexpected_scc_transport_observed,
        "unexpectedRaw0x1aaTransportObserved": unexpected_raw_alt_transport_observed,
        "raw0x1cfTransportObservedOnAnyRxBus": raw_standard_transport_observed,
      },
      "potentialFalseStart": potential_false_start,
      "infoDisplay4Periods": info_display_4_periods,
      "clusterEvidence": cluster_evidence,
      "alerts": [{**asdict(alert), "t": self._rel(alert.t)} for alert in alerts[:20]],
      "prerequisites": prerequisites,
      "signalSpecificChecks": signal_specific_checks,
      "canEvidenceVerdict": can_evidence_verdict,
      "verdict": verdict,
      "reasons": reasons,
    }

  def report(self) -> dict[str, Any]:
    if not math.isfinite(self.first_t):
      self.first_t = self.last_t = 0.0
    tx_matches, status_by_id = self._match_button_tx()
    cluster_tx_matches, cluster_status_by_id = self._cluster_tx_matches()
    res_groups = self._res_groups(status_by_id)
    episodes = [self._episode_report(start, end, observed, res_groups, status_by_id, cluster_status_by_id)
                for start, end, observed in self._stop_episodes()]

    overall = "FAIL" if any(episode["verdict"] == "FAIL" for episode in episodes) else "INCONCLUSIVE"
    episode_can_verdicts = [episode["canEvidenceVerdict"] for episode in episodes]
    unique_episode_can_verdicts = set(episode_can_verdicts)
    if "FAIL" in unique_episode_can_verdicts:
      can_evidence_verdict = "FAIL"
    elif (
      not episode_can_verdicts
      or "INCONCLUSIVE" in unique_episode_can_verdicts
      or len(unique_episode_can_verdicts) != 1
    ):
      # The top-level field describes the whole capture, not its best episode.
      # Never let one complete stop hide another incomplete or contradictory
      # stop boundary in the same log.
      can_evidence_verdict = "INCONCLUSIVE"
    else:
      can_evidence_verdict = episode_can_verdicts[0]
    vehicle_acceptance_verdict = (
      "BLOCKED_BY_CAN_EVIDENCE" if overall == "FAIL" else "REQUIRES_ON_CAR_A_B"
    )

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
      "canEvidenceVerdict": can_evidence_verdict,
      "overallVerdict": overall,
      "vehicleAcceptanceVerdict": vehicle_acceptance_verdict,
      "interpretation": {
        "FAIL": "Direct on-wire or stock-SCC status evidence contradicts the requested behavior.",
        "INCONCLUSIVE": (
          "A passive single capture cannot prove SCC acceptance, timer reset, or the visible cluster result. " +
          "Use the CAN evidence verdict only as an observation and complete the controlled on-car A/B."
        ),
      },
      "canEvidenceInterpretation": {
        "FAIL": "Direct CAN, safety, integrity, schedule, or motion evidence failed.",
        "OBSERVED_SCHEDULE_WITH_ALERT5_TIMING_DIFFERENCE": (
          "The RES schedule matched, and the observed ALERTS_5=5 timing differed from the modeled boundary; " +
          "the field has no established visible-cluster applicability on this vehicle."
        ),
        "OBSERVED_SCHEDULE_AND_ALERT5_TIMING": (
          "The RES schedule and an ALERTS_5=5 CAN-field transition were both observed near the modeled boundary."
        ),
        "OBSERVED_SCHEDULE_ALERT5_CLEAR_THROUGH_30S": (
          "The RES schedule matched and the continuously captured 0x161 field did not equal 5 through 30 seconds."
        ),
        "OBSERVED_RES_SCHEDULE_ONLY": (
          "The RES schedule matched, but no applicable complete ALERTS_5=5 correlation was available."
        ),
        "INCONCLUSIVE": "The capture lacks one or more CAN-observation prerequisites.",
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
        "SCC_CONTROL InfoDisplay=4 selects the regular/recovery state schedule only; it is not visible-prompt evidence.",
        (
          "ADRV_0x161 ALERTS_5=5 is only a raw CAN-field observation on variants where 0x161 exists; " +
          "it has no established cluster-display or driver-action correlation here."
        ),
        "When 0x161 is present on HDA1, a returned replacement must preserve each observed ALERTS_5 value.",
        "HDA2 raw-path visibility is inferred from current-branch topology and absence of a host replacement, not a cluster display acknowledgement.",
        "Start the capture before the physical stop and keep recording past 31 seconds.",
        "Use a full rlog when possible; qlogs can omit or downsample CAN/sendcan/state evidence.",
        "Final acceptance requires matched OFF, physical-RES, and ON captures plus synchronized cluster video.",
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
      safetyParam=int(
        HyundaiSafetyFlags.CANFD_ALT_BUTTONS
        | (HyundaiSafetyFlags.CANFD_LKA_STEERING if hda2 else 0)
      ),
    )],
  )


def run_demo(outcome: str, bus_offset: int = 0, hda2: bool = False, *,
             schedule_mode: str = SCHEDULE_MODE_REGULAR, button_source_phase_frames: int = 0,
             raw_alert5_value5_time: float | None = None, change_hda1_alert5_value: bool = False,
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
  if raw_alert5_value5_time is None:
    raw_alert5_value5_time = 30.0 if outcome == "pass" else 3.0

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
  analyzer.feed_state(base - CONTROL_SERVICE_PERIOD, moving_state)
  analyzer.feed_control(base - CONTROL_SERVICE_PERIOD, SimpleNamespace(enabled=True))

  stop_duration = 35.0
  stock_bus = bus_offset + (1 if hda2 else 0)
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
      analyzer.feed_can("can", t, stock_bus, scc[0], scc[1])

    if frame >= button_source_phase_frames and (frame - button_source_phase_frames) % 2 == 0:
      stock_button = packer.make_can_msg("CRUISE_BUTTONS_ALT", 0, {
        "COUNTER": (((frame - button_source_phase_frames) // 2) + button_counter_offset) & 0xFF,
        "CRUISE_BUTTONS": BUTTON_NONE,
        "DISTANCE_UNIT": 1,
        "SET_ME_2": 3,
      })
      analyzer.feed_can("can", t, stock_bus, stock_button[0], stock_button[1])

    stopped_state = SimpleNamespace(
      standstill=True, vEgo=0.0, vEgoRaw=0.0, canValid=True, brakePressed=False, gasPressed=False,
      brakeHoldActive=False, parkingBrake=False, accFaulted=False,
      cruiseState=SimpleNamespace(enabled=True, standstill=info_display >= 4),
    )
    analyzer.feed_state(t, stopped_state)
    analyzer.feed_control(t, SimpleNamespace(enabled=True))

  # Real-DBC camera source and host replacement evidence. HDA1 stock-long
  # replaces both messages on ECAN while preserving the synthetic source
  # field values used by this demo.
  # HDA2 stock-long does not synthesize these frames, leaving the raw camera
  # path unmodified. change_hda1_alert5_value generates the explicit field-
  # mutation regression case.
  raw_camera_bus = bus_offset + 2
  for frame in range(round(stop_duration * 20) + 1):
    elapsed = frame / 20.0
    t = base + elapsed
    raw_alert = USE_SWITCH_OR_PEDAL_TO_ACCELERATE if elapsed >= raw_alert5_value5_time else 0
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
        "ALERTS_5": 0 if change_hda1_alert5_value else raw_alert,
      })
      host_hda = packer.make_can_msg("LFAHDA_CLUSTER", bus_offset, {
        "COUNTER": frame & 0xFF,
        "HDA_CntrlModSta": 2,
        "HDA_LFA_SymSta": 2,
      })
      for message in (host_adrv, host_hda):
        analyzer.feed_can("sendcan", t + 0.001, bus_offset, message[0], message[1])
        analyzer.feed_can("can", t + 0.003, bus_offset + PANDA_RETURNED_BUS_OFFSET, message[0], message[1])

  # Close the synthetic stop with one valid, stationary disengaged sample so
  # the probe can verify that the capture did not simply end at the boundary.
  analyzer.feed_state(
    base + stop_duration + CONTROL_SERVICE_PERIOD,
    SimpleNamespace(
      standstill=True, vEgo=0.0, vEgoRaw=0.0, canValid=True, brakePressed=False, gasPressed=False,
      brakeHoldActive=False, parkingBrake=False, accFaulted=False,
      cruiseState=SimpleNamespace(enabled=False, standstill=False),
    ),
  )

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
        send_bus = stock_bus if hda2 else bus_offset + 2
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
      f"  #{index}: {episode['duration']:.3f}s vehicle={episode['verdict']} ",
      f"can={episode['canEvidenceVerdict']} ",
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
  print(f"CAN EVIDENCE: {report['canEvidenceVerdict']}")
  print(report["canEvidenceInterpretation"][report["canEvidenceVerdict"]])
  print(f"OVERALL VEHICLE BEHAVIOR: {report['overallVerdict']}")
  print(report["interpretation"][report["overallVerdict"]])
  print(f"VEHICLE ACCEPTANCE: {report['vehicleAcceptanceVerdict']}")


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Read-only KA4 stock-SCC RES-schedule and CAN-state observation probe",
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
      print(f"warning: {args.duration:.2f}s cannot cover the 30s observation window", file=sys.stderr)
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
  if report["overallVerdict"] == "FAIL":
    return 1
  # CAN correlation is not physical vehicle acceptance and must not look green
  # in automation without the controlled A/B and synchronized cluster video.
  return 2


if __name__ == "__main__":
  raise SystemExit(main())
