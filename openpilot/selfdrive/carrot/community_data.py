from __future__ import annotations

from typing import Any

from openpilot.common.params import Params
from openpilot.common.external_data import third_party_data_sharing_enabled


COMMUNITY_DATA_SHARING_PARAM = "CarrotCommunityDataSharing"


def community_data_sharing_enabled(params: Any | None = None) -> bool:
  """Return both master and Carrot-community consent, failing closed."""
  try:
    source = params if params is not None else Params()
    if not third_party_data_sharing_enabled(source):
      return False
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
