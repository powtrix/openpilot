#!/usr/bin/env python3
import os
import re
import shutil
import threading
import uuid
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import getxattr_direct

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 30

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5
VALIDATION_PRESERVE_ATTR_NAME = 'user.carrot_validation_preserve'
# Queue/active ownership plus the cleanup journal have one combined durable
# 30-marker cap in validation_auto_upload.
VALIDATION_PRESERVE_COUNT = 30
DELETER_TOMBSTONE_RE = re.compile(r"^(?P<original>.+)\.deleter-[0-9a-fA-F]{32}$")


def _preserve_xattr_state(path: str, attr_name: str) -> bool | None:
  try:
    return getxattr_direct(path, attr_name) == PRESERVE_ATTR_VALUE
  except OSError:
    cloudlog.exception(f"failed to read {attr_name} on {path}; stopping deletion pass")
    return None


def _has_preserve_xattr(d: str, attr_name: str) -> bool:
  # Public compatibility helper: an unknown result is protected. The deletion
  # pass uses the tri-state scanner below so errors do not consume marker quota.
  return _preserve_xattr_state(os.path.join(Paths.log_root(), d), attr_name) is not False


def has_preserve_xattr(d: str) -> bool:
  return _has_preserve_xattr(d, PRESERVE_ATTR_NAME)


def has_validation_preserve_xattr(d: str) -> bool:
  return _has_preserve_xattr(d, VALIDATION_PRESERVE_ATTR_NAME)


def _preserved_neighbors(dirs_by_creation: list[str], marked: set[str], count: int) -> set[str]:
  preserved = set()
  for n, d in enumerate(d for d in reversed(dirs_by_creation) if d in marked):
    if n == count:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and two prior
    for _seg_num in range(max(0, seg_num - 2), seg_num + 1):
      preserved.add(f"{date_str}--{_seg_num}")

  return preserved


def _scan_preserved_segments(dirs_by_creation: list[str]) -> tuple[set[str], dict[str, tuple[bool, bool]]] | None:
  marker_states: dict[str, tuple[bool, bool]] = {}
  for directory in dirs_by_creation:
    path = os.path.join(Paths.log_root(), directory)
    user = _preserve_xattr_state(path, PRESERVE_ATTR_NAME)
    validation = _preserve_xattr_state(path, VALIDATION_PRESERVE_ATTR_NAME)
    if user is None or validation is None:
      return None
    marker_states[directory] = (user, validation)

  user_marked = {directory for directory, state in marker_states.items() if state[0]}
  validation_marked = {directory for directory, state in marker_states.items() if state[1]}
  if len(validation_marked) > VALIDATION_PRESERVE_COUNT:
    # More markers than the service-wide durable invariant indicates a crash,
    # legacy state, or corruption. Never choose an "oldest" validation owner
    # to delete without a complete ownership journal; abort this delete pass.
    cloudlog.error("validation preserve marker invariant exceeded; stopping deletion pass")
    return None
  # Keep the driver's latest bookmarks and bounded automatic-validation
  # anchors in separate quotas, so one source cannot evict the other.
  preserved = (
    _preserved_neighbors(dirs_by_creation, user_marked, PRESERVE_COUNT)
    | _preserved_neighbors(dirs_by_creation, validation_marked, VALIDATION_PRESERVE_COUNT)
  )
  return preserved, marker_states


def get_preserved_segments(dirs_by_creation: list[str]) -> set[str]:
  scan = _scan_preserved_segments(dirs_by_creation)
  return set(dirs_by_creation) if scan is None else scan[0]


def _fsync_directory(directory: str) -> None:
  flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
  fd = os.open(directory, flags)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def _tombstone_original_name(name: str) -> str | None:
  match = DELETER_TOMBSTONE_RE.fullmatch(name)
  return match.group("original") if match is not None else None


def _recover_deleter_tombstones(log_root: str) -> set[str] | None:
  """Restore unambiguous directories left behind by an interrupted delete.

  Returned names must not be deleted during this pass. This includes both
  sides of an ambiguous recovery (for example, when the original directory
  already exists) so storage pressure cannot turn ambiguity into data loss.
  """
  try:
    entries = list(os.scandir(log_root))
  except FileNotFoundError:
    return set()
  except OSError:
    cloudlog.exception(f"failed to scan deleter tombstones in {log_root}; stopping deletion pass")
    return None

  tombstones_by_original: dict[str, list[tuple[str, bool]]] = {}
  for entry in entries:
    original_name = _tombstone_original_name(entry.name)
    if original_name is None:
      continue
    try:
      is_directory = entry.is_dir(follow_symlinks=False)
    except OSError:
      is_directory = False
    tombstones_by_original.setdefault(original_name, []).append((entry.name, is_directory))

  protected: set[str] = set()
  for original_name, tombstones in tombstones_by_original.items():
    original_path = os.path.join(log_root, original_name)
    tombstone_names = [name for name, _ in tombstones]

    if len(tombstones) != 1 or os.path.lexists(original_path) or not tombstones[0][1]:
      cloudlog.error(f"ambiguous deleter recovery for {original_path}; keeping all entries")
      protected.add(original_name)
      protected.update(tombstone_names)
      continue

    tombstone_path = os.path.join(log_root, tombstone_names[0])
    try:
      os.rename(tombstone_path, original_path)
      _fsync_directory(log_root)
      cloudlog.info(f"restored interrupted deletion {original_path}")
    except OSError:
      cloudlog.exception(f"failed to restore interrupted deletion {tombstone_path}")
      protected.add(original_name)
      protected.update(tombstone_names)

  return protected


def _restore_tombstone(tombstone_path: str, original_path: str) -> bool:
  if os.path.lexists(original_path):
    cloudlog.error(f"cannot restore protected log directory; keeping {tombstone_path}")
    return False
  os.rename(tombstone_path, original_path)
  _fsync_directory(os.path.dirname(original_path))
  return True


def deleter_thread(exit_event: threading.Event):
  while not exit_event.is_set():
    log_root = Paths.log_root()
    recovery_protected = _recover_deleter_tombstones(log_root)
    if recovery_protected is None:
      exit_event.wait(.1)
      continue

    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      dirs = listdir_by_creation(log_root)
      scan = _scan_preserved_segments(dirs)
      if scan is None:
        exit_event.wait(.1)
        continue
      preserved_dirs, initial_marker_states = scan

      # remove the earliest directory we can
      for delete_dir in sorted(dirs, key=lambda d: (d in DELETE_LAST, d in preserved_dirs)):
        # Selected preserve windows are an absolute retention boundary. Also
        # leave any unresolved recovery state untouched until it is unambiguous.
        if (delete_dir in preserved_dirs or delete_dir in recovery_protected
              or _tombstone_original_name(delete_dir) is not None):
          continue

        delete_path = os.path.join(log_root, delete_dir)
        tombstone_path = f"{delete_path}.deleter-{uuid.uuid4().hex}"

        try:
          if any(name.endswith(".lock") for name in os.listdir(delete_path)):
            continue

          before = (
            _preserve_xattr_state(delete_path, PRESERVE_ATTR_NAME),
            _preserve_xattr_state(delete_path, VALIDATION_PRESERVE_ATTR_NAME),
          )
          if None in before:
            break
          initial = initial_marker_states.get(delete_dir, (False, False))
          if any(current and not original for current, original in zip(before, initial, strict=True)):
            cloudlog.info(f"skipping newly preserved {delete_path}")
            continue

          # Rename is the synchronization point with cross-process setxattr:
          # a marker completed before it moves with the inode; one attempted
          # after it fails against the now-absent original path.
          os.rename(delete_path, tombstone_path)
          _fsync_directory(log_root)
          locked = any(name.endswith(".lock") for name in os.listdir(tombstone_path))
          after = (
            _preserve_xattr_state(tombstone_path, PRESERVE_ATTR_NAME),
            _preserve_xattr_state(tombstone_path, VALIDATION_PRESERVE_ATTR_NAME),
          )
          newly_preserved = None in after or any(
            current and not original for current, original in zip(after, initial, strict=True)
          )
          if locked or newly_preserved:
            cloudlog.info(f"restoring newly protected {delete_path}")
            _restore_tombstone(tombstone_path, delete_path)
            continue

          cloudlog.info(f"deleting {delete_path}")
          shutil.rmtree(tombstone_path)
          _fsync_directory(log_root)
          break
        except FileNotFoundError:
          continue
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
          if os.path.exists(tombstone_path):
            try:
              _restore_tombstone(tombstone_path, delete_path)
            except OSError:
              cloudlog.exception(f"issue restoring {tombstone_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
