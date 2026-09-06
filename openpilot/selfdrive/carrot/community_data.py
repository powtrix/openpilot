from __future__ import annotations

from typing import Any

from openpilot.common.params import Params
from openpilot.common.external_data import (
  explicit_bool_param_generation,
  third_party_data_sharing_enabled,
  third_party_data_sharing_generation,
)


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


def community_data_sharing_generation(params: Any | None = None) -> str | None:
  """Return a token bound to the current writes of both sharing consents."""
  try:
    source = params if params is not None else Params()
    master_generation = third_party_data_sharing_generation(source)
    community_generation = explicit_bool_param_generation(source, COMMUNITY_DATA_SHARING_PARAM)
  except Exception:
    return None
  if master_generation is None or community_generation is None:
    return None
  return f"{master_generation}:{community_generation}"


def community_data_sharing_generation_matches(expected: str | None, params: Any | None = None) -> bool:
  """Fail closed unless both consents are still the exact expected writes."""
  if expected is None:
    return False
  try:
    return community_data_sharing_generation(params) == expected
  except Exception:
    return False


class CommunityConsentBoundBytes:
  """File-like request body that rechecks consent at every network read."""

  def __init__(self, payload: bytes, params: Any, consent_generation: str):
    self._payload = payload
    self._params = params
    self._consent_generation = consent_generation
    self._offset = 0

  def read(self, size: int = -1) -> bytes:
    if not community_data_sharing_generation_matches(self._consent_generation, self._params):
      raise PermissionError("community data sharing consent changed")
    if self._offset >= len(self._payload):
      return b""
    end = len(self._payload) if size is None or size < 0 else min(len(self._payload), self._offset + size)
    chunk = self._payload[self._offset:end]
    self._offset = end
    return chunk
