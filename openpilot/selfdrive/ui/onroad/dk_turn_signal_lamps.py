"""Present received lamp phases without changing control-side blinker hold."""

import math


LAMP_MAX_AGE = 0.2


def _fresh_time(now, source) -> bool:
  return type(source) in (int, float) and math.isfinite(source) and source > 0 and 0 <= now - source <= LAMP_MAX_AGE


def border_turn_signal_lamps(sm, *, started: bool, started_frame: int, now: float) -> tuple[bool, bool]:
  """Use OEM lamp on/off bits on supported cars, never a synthetic UI timer."""
  if not started or type(now) not in (int, float) or not math.isfinite(now):
    return False, False
  try:
    if (not sm.alive["carState"] or not sm.valid["carState"] or sm.recv_frame["carState"] <= started_frame
        or not _fresh_time(now, sm.recv_time["carState"])
        or not _fresh_time(now, sm.logMonoTime["carState"] / 1e9)):
      return False, False
    cs = sm["carState"]
    lamps = getattr(cs, "dkTurnSignalLamps", None)
    if lamps is None or not lamps.supported:
      # Older recordings and other vehicles retain their existing presentation.
      return bool(cs.leftBlinker), bool(cs.rightBlinker)
    if not cs.canValid or not lamps.valid or not _fresh_time(now, lamps.sourceMonoTime / 1e9):
      return False, False
    return bool(lamps.left), bool(lamps.right)
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return False, False
