from __future__ import annotations

import hashlib
import os
from typing import Any

from openpilot.common.params import Params


DK_THIRD_PARTY_DATA_SHARING_PARAM = "DkThirdPartyDataSharing"


def _explicit_bool_value(value: Any) -> bool:
  if value is True:
    return True
  if type(value) is int:
    return value == 1
  if isinstance(value, bytes):
    return value == b"1"
  if isinstance(value, str):
    return value == "1"
  return False


def explicit_bool_param_enabled(params: Any, key: str) -> bool:
  """Read an explicit BOOL consent without accepting truthy/malformed values."""
  try:
    return _explicit_bool_value(params.get(key))
  except Exception:
    return False


def explicit_bool_param_generation(params: Any, key: str) -> str | None:
  """Return a token for the current enabled write of a consent parameter.

  Params writes replace the backing file atomically. The inode/timestamps let
  callers distinguish enable -> disable -> enable even if both OFF and ON were
  shorter than their polling interval. Missing or unreadable production state
  fails closed. The in-memory fallback exists only for lightweight test doubles.
  """
  if not explicit_bool_param_enabled(params, key):
    return None

  get_param_path = getattr(params, "get_param_path", None)
  if not callable(get_param_path):
    test_generation = getattr(params, "_dk_consent_generation", None)
    token = str(test_generation) if test_generation is not None else str(id(params))
    return f"memory:{key}:{token}"

  try:
    stat = os.stat(get_param_path(key), follow_symlinks=False)
  except Exception:
    return None

  material = f"{key}\0{stat.st_dev}\0{stat.st_ino}\0{stat.st_mtime_ns}\0{stat.st_ctime_ns}\0{stat.st_size}"
  return hashlib.sha256(material.encode("utf-8")).hexdigest()


def third_party_data_sharing_enabled(params: Any | None = None) -> bool:
  """Return the explicit automatic third-party data-sharing consent state.

  Missing, malformed, or unreadable values fail closed. This gate covers
  automatic diagnostics and telemetry sent to services outside the owner's
  device/NAS; it intentionally does not govern local recording or the
  separately consented private-NAS validation uploader.
  """
  try:
    source = params if params is not None else Params()
  except Exception:
    return False
  return explicit_bool_param_enabled(source, DK_THIRD_PARTY_DATA_SHARING_PARAM)


def third_party_data_sharing_generation(params: Any | None = None) -> str | None:
  """Return the current master-consent generation, or ``None`` when disabled."""
  try:
    source = params if params is not None else Params()
  except Exception:
    return None
  return explicit_bool_param_generation(source, DK_THIRD_PARTY_DATA_SHARING_PARAM)


def third_party_data_sharing_generation_matches(expected: str | None, params: Any | None = None) -> bool:
  """Fail closed unless consent is still the exact enabled write expected."""
  if expected is None:
    return False
  try:
    return third_party_data_sharing_generation(params) == expected
  except Exception:
    return False
