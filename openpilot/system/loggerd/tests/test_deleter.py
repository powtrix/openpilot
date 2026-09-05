import time
import threading
from collections import namedtuple
from pathlib import Path
from collections.abc import Sequence

import openpilot.system.loggerd.deleter as deleter
import openpilot.system.loggerd.xattr_cache as xattr_cache
from openpilot.common.timeout import Timeout, TimeoutException
from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase
from openpilot.system.loggerd.xattr_cache import setxattr

Stats = namedtuple("Stats", ['f_bavail', 'f_blocks', 'f_frsize'])


class TestDeleter(UploaderTestCase):
  def fake_statvfs(self, d):
    return self.fake_stats

  def setup_method(self):
    self.f_type = "fcamera.hevc"
    super().setup_method()
    self.fake_stats = Stats(f_bavail=0, f_blocks=10, f_frsize=4096)
    self.real_statvfs = deleter.os.statvfs
    deleter.os.statvfs = self.fake_statvfs

  def teardown_method(self):
    deleter.os.statvfs = self.real_statvfs

  def start_thread(self):
    self.end_event = threading.Event()
    self.del_thread = threading.Thread(target=deleter.deleter_thread, args=[self.end_event])
    self.del_thread.daemon = True
    self.del_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.del_thread.join()

  def test_delete(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type, 1)

    self.start_thread()

    try:
      with Timeout(2, "Timeout waiting for file to be deleted"):
        while f_path.exists():
          time.sleep(0.01)
    finally:
      self.join_thread()

  def assertDeleteOrder(self, f_paths: Sequence[Path], timeout: int = 5) -> None:
    deleted_order = []

    self.start_thread()
    try:
      with Timeout(timeout, "Timeout waiting for files to be deleted"):
        while True:
          for f in f_paths:
            if not f.exists() and f not in deleted_order:
              deleted_order.append(f)
          if len(deleted_order) == len(f_paths):
            break
          time.sleep(0.01)
    except TimeoutException:
      print("Not deleted:", [f for f in f_paths if f not in deleted_order])
      raise
    finally:
      self.join_thread()

    assert deleted_order == f_paths, "Files not deleted in expected order"

  def test_delete_order(self):
    self.assertDeleteOrder([
      self.make_file_with_data(self.seg_format.format(0), self.f_type),
      self.make_file_with_data(self.seg_format.format(1), self.f_type),
      self.make_file_with_data(self.seg_format2.format(0), self.f_type),
    ])

  def test_delete_many_preserved(self):
    deletable = [
      self.make_file_with_data(self.seg_format.format(0), self.f_type),
      self.make_file_with_data(self.seg_format.format(1), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE),
      self.make_file_with_data(self.seg_format.format(2), self.f_type),
    ]
    preserved = [
      self.make_file_with_data(self.seg_format2.format(i), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE)
      for i in range(5)
    ]

    self.assertDeleteOrder(deletable)

    assert all(path.exists() for path in preserved)

  def test_validation_preserved_segment_is_never_deleted_under_pressure(self):
    validation_file = self.make_file_with_data(self.seg_format.format(0), self.f_type)
    setxattr(
      str(validation_file.parent),
      deleter.VALIDATION_PRESERVE_ATTR_NAME,
      deleter.PRESERVE_ATTR_VALUE,
    )
    deletable_file = self.make_file_with_data(self.seg_format.format(3), self.f_type)

    self.start_thread()
    try:
      with Timeout(2, "Timeout waiting for unprotected file to be deleted"):
        while deletable_file.exists():
          time.sleep(0.01)
      time.sleep(0.25)
    finally:
      self.join_thread()

    assert validation_file.exists()

  def test_deleter_thread_restores_interrupted_delete_and_fsyncs_parent(self, monkeypatch):
    validation_file = self.make_file_with_data(self.seg_format.format(0), self.f_type)
    setxattr(
      str(validation_file.parent),
      deleter.VALIDATION_PRESERVE_ATTR_NAME,
      deleter.PRESERVE_ATTR_VALUE,
    )
    original_dir = validation_file.parent
    tombstone_dir = original_dir.with_name(f"{original_dir.name}.deleter-{'a' * 32}")
    original_dir.rename(tombstone_dir)
    deletable_file = self.make_file_with_data(self.seg_format.format(3), self.f_type)

    fsynced = []
    real_fsync_directory = deleter._fsync_directory

    def record_fsync(directory):
      fsynced.append(directory)
      real_fsync_directory(directory)

    monkeypatch.setattr(deleter, "_fsync_directory", record_fsync)

    self.start_thread()
    try:
      with Timeout(2, "Timeout waiting for recovery pass to delete unprotected file"):
        while deletable_file.exists():
          time.sleep(0.01)
      time.sleep(0.25)
    finally:
      self.join_thread()

    assert validation_file.exists()
    assert not tombstone_dir.exists()
    assert fsynced[0] == str(original_dir.parent)

  def test_ambiguous_tombstone_recovery_keeps_both_directories(self):
    original_file = self.make_file_with_data(self.seg_format.format(0), self.f_type)
    tombstone_dir = original_file.parent.with_name(f"{original_file.parent.name}.deleter-{'b' * 32}")
    tombstone_file = tombstone_dir / "rlog.zst"
    tombstone_file.parent.mkdir()
    tombstone_file.write_bytes(b"interrupted deletion")
    deletable_file = self.make_file_with_data(self.seg_format.format(3), self.f_type)

    self.start_thread()
    try:
      with Timeout(2, "Timeout waiting for unambiguous file to be deleted"):
        while deletable_file.exists():
          time.sleep(0.01)
      time.sleep(0.25)
    finally:
      self.join_thread()

    assert original_file.exists()
    assert tombstone_file.exists()

  def test_validation_retention_has_separate_quota_from_user_bookmarks(self):
    validation_dir = self.seg_format.format(0)
    validation_file = self.make_file_with_data(validation_dir, self.f_type)
    setxattr(
      str(validation_file.parent),
      deleter.VALIDATION_PRESERVE_ATTR_NAME,
      deleter.PRESERVE_ATTR_VALUE,
    )
    user_dirs = [self.seg_format2.format(i) for i in range(deleter.PRESERVE_COUNT)]
    for directory in user_dirs:
      self.make_file_with_data(directory, self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE)

    preserved = deleter.get_preserved_segments([validation_dir, *user_dirs])

    assert validation_dir in preserved
    assert all(directory in preserved for directory in user_dirs)

  def test_validation_retention_overflow_aborts_delete_pass(self):
    validation_dirs = [f"{i:08d}--4c4e99b08b--0" for i in range(deleter.VALIDATION_PRESERVE_COUNT + 1)]
    for directory in validation_dirs:
      validation_file = self.make_file_with_data(directory, self.f_type)
      setxattr(
        str(validation_file.parent),
        deleter.VALIDATION_PRESERVE_ATTR_NAME,
        deleter.PRESERVE_ATTR_VALUE,
      )

    preserved = deleter.get_preserved_segments(validation_dirs)

    assert deleter.VALIDATION_PRESERVE_COUNT == 30
    assert all(directory in preserved for directory in validation_dirs)

  def test_xattr_read_error_is_fail_safe(self, monkeypatch):
    def raise_io_error(path, attr_name):
      raise OSError(5, "simulated xattr I/O error", path)

    monkeypatch.setattr(deleter, "getxattr_direct", raise_io_error)

    assert deleter.has_preserve_xattr(self.seg_dir) is True
    assert deleter.has_validation_preserve_xattr(self.seg_dir) is True

  def test_xattr_error_aborts_pass_without_hiding_older_real_marker(self, monkeypatch):
    older_dir = self.seg_format.format(0)
    older_file = self.make_file_with_data(older_dir, self.f_type)
    setxattr(
      str(older_file.parent),
      deleter.VALIDATION_PRESERVE_ATTR_NAME,
      deleter.PRESERVE_ATTR_VALUE,
    )
    newer_dir = self.seg_format.format(1)
    newer_file = self.make_file_with_data(newer_dir, self.f_type)
    real_getxattr = deleter.getxattr_direct

    def fail_newer(path, attr_name):
      if path == str(newer_file.parent):
        raise OSError(5, "simulated xattr I/O error", path)
      return real_getxattr(path, attr_name)

    monkeypatch.setattr(deleter, "getxattr_direct", fail_newer)

    assert deleter._scan_preserved_segments([older_dir, newer_dir]) is None
    assert deleter.get_preserved_segments([older_dir, newer_dir]) == {older_dir, newer_dir}

  def test_deleter_observes_retention_set_by_another_process(self):
    directory = self.seg_format.format(0)
    file_path = self.make_file_with_data(directory, self.f_type)
    segment_path = str(file_path.parent)
    assert xattr_cache.getxattr(segment_path, deleter.VALIDATION_PRESERVE_ATTR_NAME) is None

    # Bypass setxattr()'s local cache invalidation to model loggerd/Carrot Web
    # setting the marker in a different process after the deleter cached None.
    xattr_cache._setxattr(
      segment_path,
      deleter.VALIDATION_PRESERVE_ATTR_NAME,
      deleter.PRESERVE_ATTR_VALUE,
    )

    assert deleter.has_validation_preserve_xattr(directory) is True

  def test_deleter_atomic_rename_preserves_marker_won_during_delete_race(self, monkeypatch):
    first_dir = self.seg_format.format(0)
    second_dir = self.seg_format.format(1)
    first_file = self.make_file_with_data(first_dir, self.f_type)
    second_file = self.make_file_with_data(second_dir, self.f_type)
    real_rename = deleter.os.rename
    marked = False

    def rename_after_marker(src, dst):
      nonlocal marked
      if src == str(first_file.parent) and not marked:
        setxattr(
          str(first_file.parent),
          deleter.VALIDATION_PRESERVE_ATTR_NAME,
          deleter.PRESERVE_ATTR_VALUE,
        )
        marked = True
      return real_rename(src, dst)

    exit_event = threading.Event()
    real_rmtree = deleter.shutil.rmtree

    def remove_and_stop(path):
      real_rmtree(path)
      exit_event.set()

    monkeypatch.setattr(deleter.os, "rename", rename_after_marker)
    monkeypatch.setattr(deleter.shutil, "rmtree", remove_and_stop)

    deleter.deleter_thread(exit_event)

    assert first_file.exists()
    assert not second_file.exists()

  def test_delete_last(self):
    preserved_file = self.make_file_with_data(
      self.seg_format.format(0), self.f_type, preserve_xattr=deleter.PRESERVE_ATTR_VALUE,
    )
    self.assertDeleteOrder([
      self.make_file_with_data(self.seg_format.format(1), self.f_type),
      self.make_file_with_data(self.seg_format2.format(0), self.f_type),
      self.make_file_with_data("boot", self.seg_format[:-4]),
      self.make_file_with_data("crash", self.seg_format2[:-4]),
    ])

    assert preserved_file.exists()

  def test_no_delete_when_available_space(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type)

    block_size = 4096
    available = (10 * 1024 * 1024 * 1024) / block_size  # 10GB free
    self.fake_stats = Stats(f_bavail=available, f_blocks=10, f_frsize=block_size)

    self.start_thread()
    start_time = time.monotonic()
    while f_path.exists() and time.monotonic() - start_time < 2:
      time.sleep(0.01)
    self.join_thread()

    assert f_path.exists(), "File deleted with available space"

  def test_no_delete_with_lock_file(self):
    f_path = self.make_file_with_data(self.seg_dir, self.f_type, lock=True)

    self.start_thread()
    start_time = time.monotonic()
    while f_path.exists() and time.monotonic() - start_time < 2:
      time.sleep(0.01)
    self.join_thread()

    assert f_path.exists(), "File deleted when locked"
