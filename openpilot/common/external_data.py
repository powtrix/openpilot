from __future__ import annotations

from typing import Any

from openpilot.common.params import Params


DK_THIRD_PARTY_DATA_SHARING_PARAM = "DkThirdPartyDataSharing"


def third_party_data_sharing_enabled(params: Any | None = None) -> bool:
  """Return the explicit automatic third-party data-sharing consent state.

  Missing, malformed, or unreadable values fail closed. This gate covers
  automatic diagnostics and telemetry sent to services outside the owner's
  device/NAS; it intentionally does not govern local recording or the
  separately consented private-NAS validation uploader.
  """
  try:
    source = params if params is not None else Params()
    value = source.get(DK_THIRD_PARTY_DATA_SHARING_PARAM)
  except Exception:
    return False

  if value is True:
    return True
  if type(value) is int:
    return value == 1
  if isinstance(value, bytes):
    return value == b"1"
  if isinstance(value, str):
    return value == "1"
  return False
