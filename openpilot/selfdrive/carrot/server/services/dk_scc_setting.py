"""Fail-closed writes for the startup-latched DK KA4 stock-SCC experiment."""
from __future__ import annotations

import math
from typing import Any


DK_EXPERIMENTAL_SCC_PARAM = "DkExperimentalScc"


def scc_setting_value(value: Any) -> int:
  """Reject malformed values before the generic numeric clamp can enable SCC."""
  try:
    numeric = float(value)
  except (TypeError, ValueError, OverflowError):
    raise ValueError("SCC experiment accepts only 0 (OFF) or 1 (ON)") from None
  if not math.isfinite(numeric) or numeric not in (0, 1):
    raise ValueError("SCC experiment accepts only 0 (OFF) or 1 (ON)")
  return int(numeric)


def require_scc_setting_offroad(params: Any) -> None:
  try:
    offroad = params is not None and params.get_bool("IsOffroad") is True and params.get_bool("IsOnroad") is False
  except Exception:
    offroad = False
  if not offroad:
    raise PermissionError("SCC experiment can only be changed while offroad; applies at next controls start")


def require_scc_setting_scope(params: Any) -> None:
  """Require decoded vehicle/safety topology, never a vehicle-name guess."""
  supported = False
  try:
    from openpilot.cereal import car, messaging
    from openpilot.selfdrive.carrot.dk_scc_scope import dk_scc_scope_supported

    raw = params.get("CarParams") or params.get("CarParamsPersistent")
    cp = messaging.log_from_bytes(raw, car.CarParams)
    supported = dk_scc_scope_supported(cp, params.get("GitBranch"))
  except Exception:
    supported = False
  if not supported:
    raise PermissionError("SCC experiment is available only for the dkcarrot-wip KA4 stock-SCC configuration")


def require_scc_setting_write(params: Any) -> None:
  require_scc_setting_offroad(params)
  require_scc_setting_scope(params)
