from __future__ import annotations

import asyncio
import json
import threading
from pathlib import Path
from types import SimpleNamespace

from openpilot.common import external_data
from openpilot.selfdrive.carrot.server.features.dashcam import routes as dashcam_routes
from openpilot.selfdrive.carrot.server.features.dashcam import upload as dashcam_upload
from openpilot.selfdrive.carrot.server.features.dashcam import upload_jobs
from openpilot.selfdrive.carrot.server.services import validation_auto_upload
from openpilot.selfdrive.ui.lib import prime_state
from openpilot.system import sentry
from openpilot.system.manager import process_config


class FakeParams:
  def __init__(self, value=None):
    self.value = value
    self._dk_consent_generation = "generation-1"
    self.values = {
      external_data.DK_THIRD_PARTY_DATA_SHARING_PARAM: value,
      "DongleId": "device",
      "CarrotExceptionSent": False,
    }

  def get(self, key, *args, **kwargs):
    del args, kwargs
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    self.values[key] = value


def test_master_gate_fails_closed_for_missing_malformed_and_errors():
  class ErrorParams:
    def get(self, _key):
      raise OSError("read failed")

  for value in (None, False, 0, 2, b"", b"true", b"garbage", "", "true", "garbage"):
    assert not external_data.third_party_data_sharing_enabled(FakeParams(value))
  assert not external_data.third_party_data_sharing_enabled(ErrorParams())

  for value in (True, 1, b"1", "1"):
    assert external_data.third_party_data_sharing_enabled(FakeParams(value))


def test_master_generation_invalidates_work_across_fast_off_on_cycle():
  params = FakeParams(True)
  first = external_data.third_party_data_sharing_generation(params)
  assert first is not None
  assert external_data.third_party_data_sharing_generation_matches(first, params)

  params._dk_consent_generation = "generation-2"
  second = external_data.third_party_data_sharing_generation(params)
  assert second is not None and second != first
  assert not external_data.third_party_data_sharing_generation_matches(first, params)
  assert external_data.third_party_data_sharing_generation_matches(second, params)


def test_sentry_off_preserves_local_exception_marker_without_network(monkeypatch):
  params = FakeParams(False)
  monkeypatch.setattr(sentry, "Params", lambda: params)
  monkeypatch.setattr(
    sentry.sentry_sdk,
    "capture_exception",
    lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("Sentry called")),
  )

  sentry.capture_exception()

  assert params.values["CarrotException"] == "exception"


def test_sentry_lazy_initializes_after_consent_is_enabled(monkeypatch):
  params = FakeParams(True)
  initialized = []
  captured = []
  flushed = []
  monkeypatch.setattr(sentry, "Params", lambda: params)
  monkeypatch.setattr(sentry.sentry_sdk, "is_initialized", lambda: False)
  monkeypatch.setattr(sentry, "init", lambda project: (initialized.append(project), False)[1])
  # Prove failed lazy init never reaches capture before checking the
  # successful transition below.
  monkeypatch.setattr(sentry.sentry_sdk, "capture_exception", lambda *args, **kwargs: captured.append((args, kwargs)))
  monkeypatch.setattr(sentry.sentry_sdk, "flush", lambda: flushed.append(True))

  sentry.capture_exception(RuntimeError("not initialized"))
  assert initialized == [sentry.SentryProject.SELFDRIVE]
  assert captured == []

  monkeypatch.setattr(sentry, "init", lambda project: (initialized.append(project), True)[1])
  sentry.capture_exception(RuntimeError("initialized"))

  assert initialized == [sentry.SentryProject.SELFDRIVE, sentry.SentryProject.SELFDRIVE]
  assert len(captured) == 1
  assert flushed == [True]


def test_sentry_before_send_rechecks_consent_fail_closed(monkeypatch):
  event = {"message": "private diagnostic"}
  monkeypatch.setattr(sentry, "third_party_data_sharing_generation", lambda *_args, **_kwargs: None)
  assert sentry._before_send(event, {}) is None

  monkeypatch.setattr(sentry, "third_party_data_sharing_generation", lambda *_args, **_kwargs: "generation-1")
  assert sentry._before_send(event, {}) is event
  assert event.pop(sentry.SENTRY_CONSENT_GENERATION_FIELD) == "generation-1"

  def unreadable_consent(*_args, **_kwargs):
    raise OSError("Params unavailable")

  monkeypatch.setattr(sentry, "third_party_data_sharing_generation", unreadable_consent)
  # Sentry invokes hooks inside its own exception shield, but the hook itself
  # must still fail closed if the local consent store becomes unreadable.
  assert sentry._before_send(event, {}) is None


def test_sentry_init_installs_dynamic_event_and_transaction_gates(monkeypatch):
  params = FakeParams(True)
  metadata = SimpleNamespace(
    tested_channel=False,
    channel="dkcarrot-wip",
    openpilot=SimpleNamespace(
      comma_remote=True,
      git_origin="https://github.com/commaai/openpilot",
      is_dirty=False,
      git_commit="commit",
    ),
  )
  init_calls = []
  monkeypatch.setattr(sentry, "Params", lambda: params)
  monkeypatch.setattr(sentry, "PC", False)
  monkeypatch.setattr(sentry, "third_party_data_sharing_enabled", lambda *_args, **_kwargs: True)
  monkeypatch.setattr(sentry, "get_build_metadata", lambda: metadata)
  monkeypatch.setattr(sentry, "get_version", lambda: "version")
  monkeypatch.setattr(sentry, "is_registered_device", lambda: True)
  monkeypatch.setattr(sentry.HARDWARE, "get_device_type", lambda: "device")
  monkeypatch.setattr(sentry.sentry_sdk, "init", lambda *args, **kwargs: init_calls.append((args, kwargs)))
  monkeypatch.setattr(sentry.sentry_sdk, "set_user", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(sentry.sentry_sdk, "set_tag", lambda *_args, **_kwargs: None)

  assert sentry.init(sentry.SentryProject.SELFDRIVE)
  assert len(init_calls) == 1
  _, options = init_calls[0]
  assert options["before_send"] is sentry._before_send
  assert options["before_send_transaction"] is sentry._before_send
  assert options["transport"] is sentry.ConsentHttpTransport


def test_sentry_transport_discards_envelope_from_previous_consent_generation(monkeypatch):
  queued = []
  current_generation = "generation-2"
  monkeypatch.setattr(
    sentry,
    "third_party_data_sharing_generation_matches",
    lambda expected: expected == current_generation,
  )
  monkeypatch.setattr(
    sentry.HttpTransport,
    "capture_envelope",
    lambda _self, envelope: queued.append(envelope),
  )

  event = {sentry.SENTRY_CONSENT_GENERATION_FIELD: "generation-1"}
  envelope = SimpleNamespace(get_event=lambda: event, get_transaction_event=lambda: None)
  transport = object.__new__(sentry.ConsentHttpTransport)
  transport._dk_transport_state = threading.local()
  transport.capture_envelope(envelope)

  assert queued == []
  assert sentry.SENTRY_CONSENT_GENERATION_FIELD not in event


def test_sentry_transport_rechecks_queued_envelope_before_worker_send(monkeypatch):
  sent = []
  monkeypatch.setattr(sentry, "third_party_data_sharing_generation_matches", lambda _expected: False)
  monkeypatch.setattr(sentry.HttpTransport, "_send_envelope", lambda _self, envelope: sent.append(envelope))

  envelope = SimpleNamespace(_dk_consent_generation="generation-1")
  transport = object.__new__(sentry.ConsentHttpTransport)
  transport._dk_transport_state = threading.local()
  transport._send_envelope(envelope)

  assert sent == []


def test_private_ka4_validation_consent_is_independent_of_master_gate(monkeypatch):
  receiver = validation_auto_upload.DK_VALIDATION_UPLOAD_ORIGIN
  params = FakeParams(False)
  params.values.update({
    "CarrotValidationAutoUpload": 1,
    "IsOffroad": True,
    "IsOnroad": False,
  })
  campaign = {
    "id": "campaign",
    "started_at": 1_000,
    "expires_at": 2_000,
    "base_url": receiver,
  }
  monkeypatch.setattr(validation_auto_upload, "VALIDATION_UPLOAD_BASE_URL", receiver)
  monkeypatch.setattr(
    validation_auto_upload,
    "validation_owner_device_gate",
    lambda _params: (True, {"deviceId": "device", "deviceAllowed": True}),
  )
  monkeypatch.setattr(validation_auto_upload.time, "time", lambda: 1_500)

  assert not external_data.third_party_data_sharing_enabled(params)
  assert validation_auto_upload._upload_runtime_safety_allows(
    params,
    campaign,
    device_state_safe=lambda: True,
    network_state_safe=lambda: True,
  ) == (True, "")


def test_personal_nas_manual_upload_target_is_independent_of_master_gate(monkeypatch):
  params = FakeParams(False)
  monkeypatch.setattr(external_data, "Params", lambda: params)
  monkeypatch.setattr(
    dashcam_upload,
    "read_web_settings",
    lambda: {"web_upload_url": "https://my-nas.example/private/"},
  )
  monkeypatch.delenv("CARROT_WEB_UPLOAD_URL", raising=False)
  monkeypatch.delenv("CARROT_WEB_UPLOAD_TOKEN", raising=False)

  assert not external_data.third_party_data_sharing_enabled()
  assert dashcam_upload.upload_target_settings() == ("https://my-nas.example/private", "")


def test_master_gate_off_does_not_block_explicit_manual_upload_start(monkeypatch):
  class Request:
    async def json(self):
      return {"segments": ["route--0"]}

  started = []

  async def selected_segments(_request):
    return ["route--0"]

  params = FakeParams(False)
  monkeypatch.setattr(external_data, "Params", lambda: params)
  monkeypatch.setattr(dashcam_routes, "request_upload_segments", selected_segments)
  monkeypatch.setattr(upload_jobs, "running_job", lambda: None)
  monkeypatch.setattr(
    upload_jobs,
    "create_job",
    lambda segments: {"id": "manual-job", "status": "running", "segments": segments},
  )
  monkeypatch.setattr(upload_jobs, "start_job", lambda job: started.append(job))

  assert not external_data.third_party_data_sharing_enabled()
  response = asyncio.run(dashcam_routes.api_dashcam_upload_start(Request()))

  assert response.status == 200
  assert json.loads(response.text) == {
    "ok": True,
    "job_id": "manual-job",
    "status": "running",
  }
  assert [job["segments"] for job in started] == [["route--0"]]


def test_updater_and_navigation_process_predicates_ignore_master_gate():
  params = FakeParams(False)
  params.values["SoftwareMenu"] = True

  assert process_config.enable_updated(False, params, None)
  assert process_config.enable_updated(True, params, None)
  assert process_config.managed_processes["navd"].should_run(True, params, None)
  assert not process_config.managed_processes["navd"].should_run(False, params, None)
  assert process_config.managed_processes["carrot_navi"].should_run(True, params, None)
  assert process_config.managed_processes["carrot_navi"].should_run(False, params, None)


def test_prime_polling_does_not_call_comma_api_when_off(monkeypatch):
  params = FakeParams(False)
  def fail(*args, **kwargs):
    raise AssertionError("comma API called")

  monkeypatch.setattr(prime_state, "api_get", fail)

  prime = object.__new__(prime_state.PrimeState)
  prime._params = params
  prime._session = object()
  prime._running = False
  prime._thread = None
  prime._fetch_prime_status()


def test_firehose_polling_has_fail_closed_checks_at_network_boundary():
  source = (
    Path(__file__).resolve().parents[2]
    / "selfdrive/ui/mici/layouts/settings/firehose.py"
  ).read_text(encoding="utf-8")
  assert "from openpilot.common.external_data import third_party_data_sharing_enabled" in source
  fetch_body = source.split("def _fetch_firehose_stats(self):", 1)[1].split("def _update_loop(self):", 1)[0]
  assert fetch_body.count("third_party_data_sharing_enabled(self._params)") >= 3
  assert fetch_body.rfind("third_party_data_sharing_enabled") < fetch_body.find("response = api_get")
