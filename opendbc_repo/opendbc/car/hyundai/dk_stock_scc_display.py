"""Passive KA4 stock-SCC display telemetry, with no parser or control mutations."""

import math

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags


SCC_CONTROL_NAME = "SCC_CONTROL"
SCC_CONTROL_ADDRESS = 0x1A0
SCC_CONTROL_SIZE = 32
STOCK_SCC_DISPLAY_TIMEOUT_NS = 200_000_000


def get_stock_scc_display(CP, cp: CANParser):
  """Read only the SCC parser already selected by CarState's stock-cruise path.

  CANParser commits values, raw bytes, and signal timestamps only after its
  checksum/counter checks. Its learned timeout can initially be ten seconds,
  so presentation has an independent short lifetime. Do not add a message or
  query can_valid here: both would change control-parser behavior.
  """
  ret = structs.CarState.DkStockScc()
  if (CP.carFingerprint != CAR.KIA_CARNIVAL_4TH_GEN or CP.openpilotLongitudinalControl or
      not (CP.flags & HyundaiFlags.CANFD)):
    return ret

  values = cp.vl.get(SCC_CONTROL_NAME)
  timestamps = cp.ts_nanos.get(SCC_CONTROL_NAME)
  state = cp.message_states.get(SCC_CONTROL_ADDRESS)
  data = cp.dat.get(SCC_CONTROL_ADDRESS, b"")
  if values is None or timestamps is None or state is None or len(data) != SCC_CONTROL_SIZE:
    return ret

  source_time = timestamps.get("aReqValue", 0)
  age = cp._last_update_nanos - source_time
  # Be stricter than the control parser's tolerance for intermittent counter
  # failures. A counter fault hides the gauge without altering CAN validity.
  if (source_time <= 0 or not 0 <= age <= STOCK_SCC_DISPLAY_TIMEOUT_NS or state.counter_fail != 0 or
      state.ignore_checksum or state.ignore_counter):
    return ret
  if any(timestamps.get(signal) != source_time for signal in ("aReqRaw", "ACCMode", "SysFailState", "StopReq", "TakeOverReq", "COUNTER")):
    return ret
  # The parser updates its counter even on a rejected checksum. A mismatch
  # therefore means a newer untrusted frame arrived; do not keep displaying
  # the previous accepted braking command until the freshness timeout.
  if values.get("COUNTER") != state.counter:
    return ret

  accel = float(values["aReqValue"])
  raw_accel = float(values["aReqRaw"])
  if not all(math.isfinite(value) and -10.23 - 1e-6 <= value <= 10.24 + 1e-6 for value in (accel, raw_accel)):
    return ret

  ret.accelRequest = accel
  ret.rawAccelRequest = raw_accel
  ret.valid = True
  # ACCMode 2 is driver_override, not an unambiguous SCC braking request.
  ret.active = (values["ACCMode"] == 1 and values["SysFailState"] == 0 and
                values["StopReq"] in (0, 1) and values["TakeOverReq"] == 0)
  ret.sourceMonoTime = source_time
  return ret
