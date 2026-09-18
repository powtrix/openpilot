"""Read-only stock SCC demand display; never measured deceleration or pressure."""

import math


SCC_MAX_AGE = 0.2  # seconds, including source CAN age, not just fresh carState
SCC_FULL_SCALE = 4.0  # m/s² demand; display saturation only, NOT a control limit
SCC_BAR_HEIGHT_SCALE = 2.0  # Extend above the ordinary border for visibility.


def dk_scc_display_enabled(branch) -> bool:
  return isinstance(branch, (str, bytes)) and branch.strip() in ("dkcarrot-wip", b"dkcarrot-wip")


def _finite(value) -> bool:
  return type(value) in (int, float) and math.isfinite(value)


def _fresh(sm, service: str, started_frame: int, now: float) -> bool:
  received, source = sm.recv_time[service], sm.logMonoTime[service]
  return (sm.alive[service] and sm.valid[service] and sm.recv_frame[service] > started_frame
          and _finite(received) and _finite(source) and received > 0 and source > 0
          and 0 <= now - received <= SCC_MAX_AGE
          and 0 <= now - source / 1e9 <= SCC_MAX_AGE)


def stock_scc_braking_fraction(sm, *, started: bool, started_frame: int, now: float) -> float | None:
  """Only display a fresh, unoverridden received SCC request. Missing != zero."""
  if not started or not _finite(now):
    return None
  try:
    if not all(_fresh(sm, service, started_frame, now) for service in ("carState", "selfdriveState")):
      return None
    if sm["selfdriveState"].alertSize != 0:
      return None
    cs = sm["carState"]
    demand = cs.dkStockScc
    if not (cs.canValid and demand.valid and demand.active) or cs.accFaulted or cs.brakePressed or cs.gasPressed:
      return None
    source = demand.sourceMonoTime
    accel = demand.accelRequest
    if (not _finite(source) or source <= 0 or not 0 <= now - source / 1e9 <= SCC_MAX_AGE
        or not _finite(accel) or not -10.23 <= accel < 0):
      return None
    # The stock ECU already limits the command ramp. Extra filtering would
    # delay the braking onset/release the driver wants to observe.
    # Standstill and StopReq alone neither suppress nor invent a command.
    return min(-accel / SCC_FULL_SCALE, 1.0)
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


def braking_bar_geometry(x: float, y: float, width: float, height: float,
                         thickness: float, fraction: float) -> tuple[float, float, float, float] | None:
  """Grow upward at twice the border height; keep the original side insets."""
  if not all(_finite(value) for value in (x, y, width, height, thickness, fraction)):
    return None
  bar_height = thickness * SCC_BAR_HEIGHT_SCALE
  if thickness <= 0 or width <= thickness * 2 or height < bar_height or not 0 < fraction <= 1:
    return None
  fill_width = (width - 2 * thickness) * fraction
  return (x + (width - fill_width) / 2, y + height - bar_height, fill_width, bar_height)
