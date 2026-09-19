from __future__ import annotations

from typing import Any
import urllib.request

from openpilot.common.params import Params
from openpilot.common.external_data import (
  explicit_bool_param_generation,
  third_party_data_sharing_enabled,
  third_party_data_sharing_generation,
)


COMMUNITY_DATA_SHARING_PARAM = "CarrotCommunityDataSharing"
AUTOMATIC_DIAGNOSTIC_REQUEST_PREFIX = "dkdiag-v1:"
CONSENT_STREAM_CHUNK_SIZE = 64 * 1024


class NoConsentRedirectHandler(urllib.request.HTTPRedirectHandler):
  def redirect_request(self, req, fp, code, msg, headers, newurl):
    return None


def open_url_no_redirect(request: urllib.request.Request, timeout: float):
  return urllib.request.build_opener(NoConsentRedirectHandler()).open(request, timeout=timeout)


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


def format_automatic_diagnostic_request(reason: str, consent_generation: str) -> str:
  """Encode an automatic diagnostic trigger with its exact consent write."""
  normalized_reason = str(reason or "").strip()
  normalized_generation = str(consent_generation or "").strip()
  if not normalized_reason or ":" in normalized_reason or not normalized_generation:
    raise ValueError("invalid automatic diagnostic request")
  return f"{AUTOMATIC_DIAGNOSTIC_REQUEST_PREFIX}{normalized_reason}:{normalized_generation}"


def automatic_diagnostic_request(
  reason: str,
  params: Any | None = None,
) -> str | None:
  """Build a request only while both master and community consent are ON."""
  generation = community_data_sharing_generation(params)
  if generation is None:
    return None
  return format_automatic_diagnostic_request(reason, generation)


def parse_automatic_diagnostic_request(value: Any) -> tuple[str | None, str | None]:
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="replace")
  text = str(value or "").strip()
  if not text.startswith(AUTOMATIC_DIAGNOSTIC_REQUEST_PREFIX):
    return None, None
  reason, separator, generation = text[len(AUTOMATIC_DIAGNOSTIC_REQUEST_PREFIX):].partition(":")
  reason = reason.strip()
  generation = generation.strip()
  if not separator or not reason or not generation:
    return None, None
  return reason, generation


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
    if size is None or size < 0:
      size = CONSENT_STREAM_CHUNK_SIZE
    size = min(int(size), CONSENT_STREAM_CHUNK_SIZE)
    end = min(len(self._payload), self._offset + size)
    chunk = self._payload[self._offset:end]
    self._offset = end
    if not community_data_sharing_generation_matches(self._consent_generation, self._params):
      raise PermissionError("community data sharing consent changed")
    return chunk
