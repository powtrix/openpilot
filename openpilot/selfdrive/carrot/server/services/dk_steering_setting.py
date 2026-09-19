"""Fail-closed write policy for the controls-startup-latched experiment."""
from __future__ import annotations

import math
from typing import Any


DK_EXPERIMENTAL_STEERING_PARAM = "DkExperimentalSteering"


def steering_setting_value(value: Any) -> int:
  """Do not silently clamp a malformed experimental-control selection to ON."""
  try:
    numeric = float(value)
  except (TypeError, ValueError, OverflowError):
    raise ValueError("experimental steering accepts only 0 (OFF) or 1 (ON)") from None
  if not math.isfinite(numeric) or numeric not in (0, 1):
    raise ValueError("experimental steering accepts only 0 (OFF) or 1 (ON)")
  return int(numeric)


def require_steering_setting_offroad(params: Any) -> None:
  try:
    offroad = params is not None and params.get_bool("IsOffroad") is True and params.get_bool("IsOnroad") is False
  except Exception:
    offroad = False
  if not offroad:
    raise PermissionError("experimental steering can only be changed while offroad; applies at next controls start")
