"""Read-only presentation of wheel-speed-derived deceleration, not brake pressure."""

import math
from dataclasses import dataclass


DECEL_MAX_AGE = 0.5
DECEL_MIN_SPEED = 0.5  # m/s; hide stop/creep noise
DECEL_MIN_MAGNITUDE = 0.1  # m/s²
DECEL_FULL_SCALE = 4.0  # m/s²; display scale only, not a control limit


@dataclass(frozen=True)
class DecelerationDisplay:
  magnitude: float
  fraction: float

  @property
  def text(self) -> str:
    # The fallback display font includes ASCII/Korean, but not superscript ².
    return f"감속 {self.magnitude:.1f} m/s^2"


def _finite_number(value) -> bool:
  return type(value) in (int, float) and math.isfinite(value)


def _fresh(sm, service: str, started_frame: int, now: float) -> bool:
  # Both arrival and source age matter: a newly delivered old sample is stale.
  received = sm.recv_time[service]
  source_ns = sm.logMonoTime[service]
  return (sm.alive[service] and sm.valid[service] and sm.recv_frame[service] > started_frame
          and _finite_number(received) and _finite_number(source_ns)
          and received > 0 and source_ns > 0
          and 0 <= now - received <= DECEL_MAX_AGE
          and 0 <= now - source_ns / 1e9 <= DECEL_MAX_AGE)


def deceleration_display(sm, *, started: bool, started_frame: int, now: float) -> DecelerationDisplay | None:
  """Fail closed for unavailable signals. Never infer zero braking from no data."""
  if not started or not _finite_number(now):
    return None
  try:
    # Hide even before the warning renderer's communication-loss timeout. Its
    # translucent warning background must never contain a red deceleration bar.
    if not all(_fresh(sm, service, started_frame, now) for service in ("carState", "selfdriveState")):
      return None
    if sm["selfdriveState"].alertSize != 0:
      return None
    cs = sm["carState"]
    if (not _finite_number(cs.vEgo) or not _finite_number(cs.aEgo)
        or cs.standstill or cs.vEgo < DECEL_MIN_SPEED or abs(cs.aEgo) > 20.0):
      return None
    magnitude = -cs.aEgo
    if magnitude < DECEL_MIN_MAGNITUDE:
      return None
    # Do not smooth again: aEgo is already filtered by the vehicle speed KF.
    return DecelerationDisplay(magnitude, min(magnitude / DECEL_FULL_SCALE, 1.0))
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None
