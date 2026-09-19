import time
import threading
import logging
import json
from types import SimpleNamespace
from pathlib import Path
import pytest
from openpilot.system.hardware.hw import Paths

from openpilot.common.external_data import DK_THIRD_PARTY_DATA_SHARING_PARAM, third_party_data_sharing_generation
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd import uploader
from openpilot.system.loggerd.uploader import main, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
from openpilot.system.loggerd.xattr_cache import getxattr_direct

from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase


class FakeLogHandler(logging.Handler):
  def __init__(self):
    logging.Handler.__init__(self)
    self.reset()

  def reset(self):
    self.upload_order = list()
    self.upload_ignored = list()

  def emit(self, record):
    try:
      j = json.loads(record.getMessage())
      if j["event"] == "upload_success":
        self.upload_order.append(j["key"])
      if j["event"] == "upload_ignored":
        self.upload_ignored.append(j["key"])
    except Exception:
      pass

log_handler = FakeLogHandler()
cloudlog.addHandler(log_handler)


class TestUploader(UploaderTestCase):
  def setup_method(self):
    super().setup_method()
    log_handler.reset()

  def start_thread(self):
    self.end_event = threading.Event()
    self.up_thread = threading.Thread(target=main, args=[self.end_event])
    self.up_thread.daemon = True
    self.up_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.up_thread.join()

  def gen_files(self, lock=False, xattr: bytes | None = None, boot=True) -> list[Path]:
    f_paths = []
    for t in ["qlog", "rlog", "dcamera.hevc", "fcamera.hevc"]:
      f_paths.append(self.make_file_with_data(self.seg_dir, t, 1, lock=lock, upload_xattr=xattr))

    if boot:
      f_paths.append(self.make_file_with_data("boot", f"{self.seg_dir}", 1, lock=lock, upload_xattr=xattr))
    return f_paths

  def gen_order(self, seg1: list[int], seg2: list[int], boot=True) -> list[str]:
    keys = []
    if boot:
      keys += [f"boot/{self.seg_format.format(i)}.zst" for i in seg1]
      keys += [f"boot/{self.seg_format2.format(i)}.zst" for i in seg2]
    keys += [f"{self.seg_format.format(i)}/qlog.zst" for i in seg1]
    keys += [f"{self.seg_format2.format(i)}/qlog.zst" for i in seg2]
    return keys

  def test_upload(self):
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert getxattr_direct(str((Path(Paths.log_root()) / f_path).with_suffix("")), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_with_wrong_xattr(self):
    self.gen_files(lock=False, xattr=b'0')

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert getxattr_direct(str((Path(Paths.log_root()) / f_path).with_suffix("")), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_ignored(self):
    self.set_ignore()
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    exp_order = self.gen_order([self.seg_num], [])

    assert len(log_handler.upload_order) == 0, "Some files were not ignored"
    assert not len(log_handler.upload_ignored) < len(exp_order), "Some files failed to ignore"
    assert not len(log_handler.upload_ignored) > len(exp_order), "Some files were ignored twice"
    for f_path in exp_order:
      assert getxattr_direct(str((Path(Paths.log_root()) / f_path).with_suffix("")), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not ignored"

    assert log_handler.upload_ignored == exp_order, "Files ignored in wrong order"

  def test_upload_files_in_create_order(self):
    seg1_nums = [0, 1, 2, 10, 20]
    for i in seg1_nums:
      self.seg_dir = self.seg_format.format(i)
      self.gen_files(boot=False)
    seg2_nums = [5, 50, 51]
    for i in seg2_nums:
      self.seg_dir = self.seg_format2.format(i)
      self.gen_files(boot=False)

    exp_order = self.gen_order(seg1_nums, seg2_nums, boot=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert getxattr_direct(str((Path(Paths.log_root()) / f_path).with_suffix("")), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_no_upload_with_lock_file(self):
    self.start_thread()

    time.sleep(0.25)
    f_paths = self.gen_files(lock=True, boot=False)

    # allow enough time that files should have been uploaded if they would be uploaded
    time.sleep(1)
    self.join_thread()

    for f_path in f_paths:
      fn = f_path.with_suffix(f_path.suffix.replace(".zst", ""))
      uploaded = getxattr_direct(str(fn), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
      assert not uploaded, "File upload when locked"

  def test_no_upload_with_xattr(self):
    self.gen_files(lock=False, xattr=UPLOAD_ATTR_VALUE)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(1)
    self.join_thread()

    assert len(log_handler.upload_order) == 0, "File uploaded again"

  def test_clear_locks_on_startup(self):
    f_paths = self.gen_files(lock=True, boot=False)
    self.start_thread()
    time.sleep(0.25)
    self.join_thread()

    for f_path in f_paths:
      lock_path = f_path.with_suffix(f_path.suffix + ".lock")
      assert not lock_path.is_file(), "File lock not cleared on startup"

  def test_third_party_sharing_off_blocks_upload_url_request(self, monkeypatch):
    self.params.put_bool(DK_THIRD_PARTY_DATA_SHARING_PARAM, False)
    candidate = self.make_file_with_data(self.seg_dir, "qlog", 0.01)
    instance = uploader.Uploader("0000000000000000", str(Paths.log_root()))
    monkeypatch.setattr(
      instance.api,
      "get",
      lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("comma API called")),
    )

    with pytest.raises(uploader.AutomaticDataSharingDisabled):
      instance.do_upload(f"{self.seg_dir}/qlog.zst", str(candidate))

  def test_signed_put_disables_redirects_and_aborts_aba_mid_body(self, monkeypatch):
    candidate = self.make_file_with_data(self.seg_dir, "qlog", 0.01)
    instance = uploader.Uploader("0000000000000000", str(Paths.log_root()))
    state = {"enabled": True, "generation": "generation-1"}
    consent_params = SimpleNamespace(_dk_consent_generation=state["generation"])
    consent_params.get = lambda _key, *args, **kwargs: b"1" if state["enabled"] else b"0"
    instance.params = consent_params
    generation = third_party_data_sharing_generation(consent_params)
    assert generation is not None
    instance.consent_generation = generation
    instance.api.get = lambda *args, **kwargs: SimpleNamespace(
      status_code=200,
      text='{"url": "https://upload.example/file", "headers": {}}',
    )

    request_options = {}

    def put(_url, *, data, **kwargs):
      request_options.update(kwargs)
      assert data.read(1)
      state["enabled"] = False
      consent_params._dk_consent_generation = state["generation"] = "generation-2"
      state["enabled"] = True
      data.read(1)
      raise AssertionError("stale upload body resumed after consent ABA")

    monkeypatch.setattr(uploader, "fake_upload", False)
    monkeypatch.setattr(uploader.requests, "put", put)

    with pytest.raises(uploader.AutomaticDataSharingDisabled):
      instance.do_upload(f"{self.seg_dir}/qlog.zst", str(candidate))

    assert request_options["allow_redirects"] is False

  def test_signed_url_response_does_not_start_old_generation_put(self, monkeypatch):
    candidate = self.make_file_with_data(self.seg_dir, "qlog", 0.01)
    instance = uploader.Uploader("0000000000000000", str(Paths.log_root()))
    state = {"enabled": True, "generation": "generation-1"}
    consent_params = SimpleNamespace(_dk_consent_generation=state["generation"])
    consent_params.get = lambda _key, *args, **kwargs: b"1" if state["enabled"] else b"0"
    instance.params = consent_params
    generation = third_party_data_sharing_generation(consent_params)
    assert generation is not None

    def get_signed_url(*_args, **_kwargs):
      state["enabled"] = False
      consent_params._dk_consent_generation = state["generation"] = "generation-2"
      state["enabled"] = True
      return SimpleNamespace(
        status_code=200,
        text='{"url": "https://upload.example/file", "headers": {}}',
      )

    instance.api.get = get_signed_url
    monkeypatch.setattr(uploader, "fake_upload", False)
    monkeypatch.setattr(
      uploader.requests,
      "put",
      lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("stale signed upload started")),
    )

    with pytest.raises(uploader.AutomaticDataSharingDisabled):
      instance.do_upload(f"{self.seg_dir}/qlog.zst", str(candidate), generation)

  def test_signed_put_rechecks_generation_after_response(self, monkeypatch):
    candidate = self.make_file_with_data(self.seg_dir, "qlog", 0.01)
    instance = uploader.Uploader("0000000000000000", str(Paths.log_root()))
    state = {"enabled": True, "generation": "generation-1"}
    consent_params = SimpleNamespace(_dk_consent_generation=state["generation"])
    consent_params.get = lambda _key, *args, **kwargs: b"1" if state["enabled"] else b"0"
    instance.params = consent_params
    generation = third_party_data_sharing_generation(consent_params)
    assert generation is not None
    instance.consent_generation = generation
    instance.api.get = lambda *args, **kwargs: SimpleNamespace(
      status_code=200,
      text='{"url": "https://upload.example/file", "headers": {}}',
    )

    response = SimpleNamespace(closed=False)
    response.close = lambda: setattr(response, "closed", True)

    def put(_url, *, data, **kwargs):
      assert kwargs["allow_redirects"] is False
      while data.read(1024):
        pass
      state["enabled"] = False
      consent_params._dk_consent_generation = state["generation"] = "generation-2"
      state["enabled"] = True
      return response

    monkeypatch.setattr(uploader, "fake_upload", False)
    monkeypatch.setattr(uploader.requests, "put", put)

    with pytest.raises(uploader.AutomaticDataSharingDisabled):
      instance.do_upload(f"{self.seg_dir}/qlog.zst", str(candidate))

    assert response.closed
