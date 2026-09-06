from __future__ import annotations

import threading

import pytest

from openpilot.common.external_data import DK_THIRD_PARTY_DATA_SHARING_PARAM
from openpilot.system.athena import athenad, manage_athenad


class FakeParams:
  def __init__(self, enabled=False):
    self._dk_consent_generation = "generation-1"
    self.values = {
      DK_THIRD_PARTY_DATA_SHARING_PARAM: b"1" if enabled else b"0",
      "DongleId": "device",
      "AthenadUploadQueue": [{"id": "old"}],
    }

  def get(self, key, *args, **kwargs):
    del args, kwargs
    return self.values.get(key)

  def remove(self, key):
    self.values.pop(key, None)

  def put(self, key, value):
    self.values[key] = value


def test_athenad_main_off_never_initializes_remote_client(monkeypatch):
  params = FakeParams(False)
  monkeypatch.setattr(athenad, "Params", lambda: params)
  monkeypatch.setattr(
    athenad,
    "Api",
    lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("API client initialized")),
  )
  monkeypatch.setattr(
    athenad,
    "create_connection",
    lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("websocket connected")),
  )

  athenad.main(threading.Event())


def test_athenad_direct_upload_boundaries_fail_closed(monkeypatch):
  monkeypatch.setattr(athenad, "third_party_data_sharing_enabled", lambda params=None: False)
  monkeypatch.setattr(
    athenad,
    "get_upload_stream",
    lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("file stream opened")),
  )

  item = athenad.UploadItem(
    path="/tmp/not-opened",
    url="https://example.invalid/upload",
    headers={},
    created_at=0,
    id="blocked",
  )
  with pytest.raises(athenad.AbortTransferException):
    athenad._do_upload(item)
  with pytest.raises(athenad.AbortTransferException):
    athenad.uploadFilesToUrls([])


def test_manage_athenad_off_discards_legacy_queue_without_starting(monkeypatch):
  params = FakeParams(False)
  monkeypatch.setattr(
    manage_athenad,
    "Process",
    lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("athenad started")),
  )

  with pytest.raises(StopIteration):
    manage_athenad.run_athenad_manager(params, sleep=lambda _delay: (_ for _ in ()).throw(StopIteration))

  assert "AthenadUploadQueue" not in params.values


def test_manage_athenad_stops_running_child_when_consent_is_withdrawn(monkeypatch):
  params = FakeParams(True)
  created = []

  class FakeProcess:
    exitcode = None

    def __init__(self, *args, **kwargs):
      del args, kwargs
      self.started = False
      self.terminated = False
      created.append(self)

    def start(self):
      self.started = True

    def is_alive(self):
      return self.started and not self.terminated

    def terminate(self):
      self.terminated = True
      self.exitcode = -15

    def kill(self):
      self.terminated = True
      self.exitcode = -9

    def join(self, timeout=None):
      del timeout

  sleeps = 0

  def sleep(_delay):
    nonlocal sleeps
    sleeps += 1
    if sleeps == 1:
      params.values[DK_THIRD_PARTY_DATA_SHARING_PARAM] = b"0"
    elif sleeps == 2:
      raise StopIteration

  monkeypatch.setattr(manage_athenad, "Process", FakeProcess)
  monkeypatch.setattr(
    manage_athenad,
    "prepare_consent_session",
    lambda source: manage_athenad.third_party_data_sharing_generation(source),
  )
  with pytest.raises(StopIteration):
    manage_athenad.run_athenad_manager(params, sleep=sleep)

  assert len(created) == 1
  assert created[0].started
  assert created[0].terminated
  assert "AthenadUploadQueue" not in params.values


def test_manage_athenad_restarts_and_clears_queue_after_missed_off_on_cycle(monkeypatch):
  params = FakeParams(True)
  created = []

  class FakeProcess:
    exitcode = None

    def __init__(self, *args, **kwargs):
      del args, kwargs
      self.started = False
      self.terminated = False
      created.append(self)

    def start(self):
      self.started = True

    def is_alive(self):
      return self.started and not self.terminated

    def terminate(self):
      self.terminated = True
      self.exitcode = -15

    def kill(self):
      self.terminate()

    def join(self, timeout=None):
      del timeout

  sleeps = 0

  def sleep(_delay):
    nonlocal sleeps
    sleeps += 1
    if sleeps == 1:
      params.values["AthenadUploadQueue"] = [{"id": "stale"}]
      params._dk_consent_generation = "generation-2"
    elif sleeps == 2:
      raise StopIteration

  monkeypatch.setattr(manage_athenad, "Process", FakeProcess)
  monkeypatch.setattr(
    manage_athenad,
    "prepare_consent_session",
    lambda source: manage_athenad.third_party_data_sharing_generation(source),
  )

  with pytest.raises(StopIteration):
    manage_athenad.run_athenad_manager(params, sleep=sleep)

  assert len(created) == 2
  assert created[0].terminated
  assert created[1].started
  assert created[1].terminated
  assert "AthenadUploadQueue" not in params.values
