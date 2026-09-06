from __future__ import annotations

from typing import Any

from openpilot.common.params import Params


COMMUNITY_DATA_SHARING_PARAM = "CarrotCommunityDataSharing"


def community_data_sharing_enabled(params: Any | None = None) -> bool:
  """Return the explicit community-data consent state, failing closed."""
  try:
    source = params if params is not None else Params()
    value = source.get(COMMUNITY_DATA_SHARING_PARAM)
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
