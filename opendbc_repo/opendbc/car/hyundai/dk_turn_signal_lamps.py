"""Passive KA4 turn-lamp phase for display, separate from held control blinkers."""

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags


TURN_SIGNAL_LAMP_TIMEOUT_NS = 200_000_000
LAMP_MESSAGES = {"BLINKERS": (0x413, 8), "BLINKERS_ALT": (0x3E3, 16)}
LAMP_SIGNALS = ("LEFT_LAMP", "LEFT_LAMP_ALT", "RIGHT_LAMP", "RIGHT_LAMP_ALT")


def get_turn_signal_lamps(CP, cp: CANParser, message_name: str | None):
  """Read the same already-selected message as CarState without parser mutations.

  These lamp DBC messages do not define a checksum, so this reports only what
  the existing parser accepted, not a new integrity guarantee. If a future
  definition adds counter/checksum checks, retain those checks and reject any
  currently failing counter or deliberately bypassed integrity validation.
  """
  ret = structs.CarState.DkTurnSignalLamps()
  if CP.carFingerprint != CAR.KIA_CARNIVAL_4TH_GEN or not (CP.flags & HyundaiFlags.CANFD):
    return ret
  ret.supported = True
  if message_name not in LAMP_MESSAGES:
    return ret

  address, size = LAMP_MESSAGES[message_name]
  values = cp.vl.get(message_name)
  timestamps = cp.ts_nanos.get(message_name)
  state = cp.message_states.get(address)
  data = cp.dat.get(address, b"")
  if values is None or timestamps is None or state is None or len(data) != size:
    return ret

  source_time = timestamps.get("LEFT_LAMP", 0)
  age = cp._last_update_nanos - source_time
  if (source_time <= 0 or not 0 <= age <= TURN_SIGNAL_LAMP_TIMEOUT_NS or state.counter_fail != 0 or
      state.ignore_checksum or state.ignore_counter):
    return ret
  if any(timestamps.get(signal) != source_time or values.get(signal) not in (0, 1) for signal in LAMP_SIGNALS):
    return ret
  if any(values.get(signal.name) != state.counter for signal in state.signals if signal.type == 1):
    return ret

  ret.valid = True
  ret.left = bool(values["LEFT_LAMP"] or values["LEFT_LAMP_ALT"])
  ret.right = bool(values["RIGHT_LAMP"] or values["RIGHT_LAMP_ALT"])
  ret.sourceMonoTime = source_time
  return ret
