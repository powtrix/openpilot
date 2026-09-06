from __future__ import annotations

import os
from collections.abc import Iterable

from openpilot.common.external_data import third_party_data_sharing_generation
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr_direct, setxattr


PREPARED_GENERATION_PARAM = "DkThirdPartyDataSharingPreparedGeneration"
BLOCKED_UPLOAD_ATTR = "user.dk.third_party_upload_blocked"
BLOCKED_UPLOAD_VALUE = b"1"


def artifact_is_blocked(path: str) -> bool:
  """Fail closed when a path was present before the current consent session."""
  try:
    return getxattr_direct(path, BLOCKED_UPLOAD_ATTR) == BLOCKED_UPLOAD_VALUE
  except OSError:
    return True


def _artifact_roots() -> tuple[str, ...]:
  return Paths.log_root(), Paths.swaglog_root(), Paths.stats_root()


def _mark_existing_files(root: str) -> None:
  try:
    pending = [os.path.abspath(root)]
  except Exception as exc:
    raise RuntimeError(f"invalid automatic-upload artifact root: {root}") from exc

  while pending:
    directory = pending.pop()
    try:
      entries = list(os.scandir(directory))
    except FileNotFoundError:
      continue
    except OSError as exc:
      raise RuntimeError(f"cannot inspect automatic-upload artifacts: {directory}") from exc

    for entry in entries:
      try:
        if entry.is_symlink():
          continue
        if entry.is_dir(follow_symlinks=False):
          pending.append(entry.path)
        elif entry.is_file(follow_symlinks=False):
          setxattr(entry.path, BLOCKED_UPLOAD_ATTR, BLOCKED_UPLOAD_VALUE)
      except FileNotFoundError:
        # Rotation/deletion racing this one-time snapshot is safe: the vanished
        # inode cannot later be uploaded under this path.
        continue
      except OSError as exc:
        raise RuntimeError(f"cannot exclude pre-consent artifact: {entry.path}") from exc


def prepare_consent_session(params: Params, roots: Iterable[str] | None = None) -> str | None:
  """Exclude every existing upload artifact before admitting a new consent.

  The prepared generation is persisted only after the complete snapshot is
  marked. A crash or xattr failure therefore fails closed and retries without
  opening Athena. Files created later in the same enabled generation remain
  eligible; files created while disabled are marked at the next opt-in.
  """
  generation = third_party_data_sharing_generation(params)
  if generation is None:
    return None

  try:
    prepared = params.get(PREPARED_GENERATION_PARAM)
  except Exception:
    return None
  if isinstance(prepared, bytes):
    prepared = prepared.decode("utf-8", errors="replace")
  if prepared == generation:
    return generation

  for root in roots if roots is not None else _artifact_roots():
    _mark_existing_files(root)

  try:
    params.put(PREPARED_GENERATION_PARAM, generation)
  except Exception:
    return None
  return generation


def consent_session_is_prepared(params: Params, generation: str | None) -> bool:
  if generation is None or third_party_data_sharing_generation(params) != generation:
    return False
  try:
    prepared = params.get(PREPARED_GENERATION_PARAM)
  except Exception:
    return False
  if isinstance(prepared, bytes):
    prepared = prepared.decode("utf-8", errors="replace")
  return prepared == generation
