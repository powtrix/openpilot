"""Install exception handler for process crash."""
from contextlib import contextmanager
import threading

import sentry_sdk
from enum import Enum
from typing import Any
from sentry_sdk.integrations.threading import ThreadingIntegration
from sentry_sdk.transport import HttpTransport

from openpilot.common.external_data import (
  third_party_data_sharing_enabled,
  third_party_data_sharing_generation,
  third_party_data_sharing_generation_matches,
)
from openpilot.common.params import Params
from openpilot.selfdrive.carrot.community_data import automatic_diagnostic_request
from openpilot.system.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

SENTRY_CONSENT_GENERATION_FIELD = "_dk_consent_generation"
SENTRY_UPLOAD_CHUNK_SIZE = 64 * 1024
_sentry_event_state = threading.local()


class _ConsentBoundBody:
  """File-like Sentry body that rechecks the exact consent write per chunk."""

  def __init__(self, payload: bytes, consent_generation: str):
    self._payload = payload
    self._consent_generation = consent_generation
    self._offset = 0

  def read(self, size: int = -1) -> bytes:
    if not third_party_data_sharing_generation_matches(self._consent_generation):
      raise RuntimeError("Sentry consent generation changed during network write")
    if self._offset >= len(self._payload):
      return b""
    requested = SENTRY_UPLOAD_CHUNK_SIZE if size is None or size < 0 else min(size, SENTRY_UPLOAD_CHUNK_SIZE)
    end = min(len(self._payload), self._offset + requested)
    chunk = self._payload[self._offset:end]
    self._offset = end
    if not third_party_data_sharing_generation_matches(self._consent_generation):
      raise RuntimeError("Sentry consent generation changed during network write")
    return chunk


@contextmanager
def _bind_event_consent_generation(consent_generation: str):
  previous = getattr(_sentry_event_state, "consent_generation", None)
  _sentry_event_state.consent_generation = consent_generation
  try:
    yield
  finally:
    if previous is None:
      try:
        del _sentry_event_state.consent_generation
      except AttributeError:
        pass
    else:
      _sentry_event_state.consent_generation = previous


class SentryProject(Enum):
  # python project
  SELFDRIVE = "https://6f3c7076c1e14b2aa10f5dde6dda0cc4@o33823.ingest.sentry.io/77924"
  # native project
  SELFDRIVE_NATIVE = "https://3e4b586ed21a4479ad5d85083b639bc6@o33823.ingest.sentry.io/157615"


class ConsentHttpTransport(HttpTransport):
  """Bind every queued Sentry envelope to one exact consent generation."""

  def __init__(self, options):
    super().__init__(options)
    self._dk_transport_state = threading.local()

  def capture_envelope(self, envelope) -> None:
    event = envelope.get_event() or envelope.get_transaction_event()
    generation = event.pop(SENTRY_CONSENT_GENERATION_FIELD, None) if isinstance(event, dict) else None
    if not third_party_data_sharing_generation_matches(generation):
      return
    envelope._dk_consent_generation = generation
    super().capture_envelope(envelope)

  def _send_envelope(self, envelope) -> None:
    generation = getattr(envelope, "_dk_consent_generation", None)
    if not third_party_data_sharing_generation_matches(generation):
      return
    self._dk_transport_state.consent_generation = generation
    try:
      super()._send_envelope(envelope)
    finally:
      self._dk_transport_state.consent_generation = None

  def _request(self, method, endpoint_type, body, headers):
    generation = getattr(self._dk_transport_state, "consent_generation", None)
    if not third_party_data_sharing_generation_matches(generation):
      raise RuntimeError("Sentry consent generation changed before network write")
    if not isinstance(body, (bytes, bytearray, memoryview)):
      raise RuntimeError("Sentry request body cannot be consent guarded")

    payload = bytes(body)
    guarded_body = _ConsentBoundBody(payload, generation)
    request_headers = dict(headers)
    request_headers.setdefault("Content-Length", str(len(payload)))
    response = self._pool.request(
      method,
      self._auth.get_api_url(endpoint_type),
      body=guarded_body,
      headers=request_headers,
      redirect=False,
    )
    if not third_party_data_sharing_generation_matches(generation):
      response.close()
      raise RuntimeError("Sentry consent generation changed during network request")
    return response


def _before_send(event: dict[str, Any], _hint: dict[str, Any]) -> dict[str, Any] | None:
  """Recheck consent after event enrichment, immediately before SDK transport."""
  try:
    generation = getattr(_sentry_event_state, "consent_generation", None)
    if generation is None:
      generation = third_party_data_sharing_generation()
    if generation is None:
      return None
    event[SENTRY_CONSENT_GENERATION_FIELD] = generation
    return event
  except Exception:
    return None


def _ensure_initialized(project: SentryProject) -> bool:
  if sentry_sdk.is_initialized():
    return True
  return init(project)


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})
  consent_generation = third_party_data_sharing_generation()
  if consent_generation is None:
    return
  if not _ensure_initialized(SentryProject.SELFDRIVE_NATIVE):
    return
  if not third_party_data_sharing_generation_matches(consent_generation):
    return

  with _bind_event_consent_generation(consent_generation):
    with sentry_sdk.configure_scope() as scope:
      scope.set_extra("tombstone_fn", fn)
      scope.set_extra("tombstone", contents)
      sentry_sdk.capture_message(message=message)
      sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))
  params = Params()
  consent_generation = third_party_data_sharing_generation(params)
  if not params.get_bool("CarrotExceptionSent"):
    params.put(
      "CarrotException",
      automatic_diagnostic_request("exception", params) or "exception",
    )
  if consent_generation is None:
    return
  if not _ensure_initialized(SentryProject.SELFDRIVE):
    return
  if not third_party_data_sharing_generation_matches(consent_generation, params):
    return

  try:
    with _bind_event_consent_generation(consent_generation):
      sentry_sdk.capture_exception(*args, **kwargs)
      sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def init(project: SentryProject) -> bool:
  if not third_party_data_sharing_enabled():
    return False
  build_metadata = get_build_metadata()
  # forks like to mess with this, so double check
  comma_remote = build_metadata.openpilot.comma_remote and "commaai" in build_metadata.openpilot.git_origin
  if not comma_remote or not is_registered_device() or PC:
    return False

  env = "release" if build_metadata.tested_channel else "master"
  dongle_id = Params().get("DongleId")

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  before_send=_before_send,
                  before_send_transaction=_before_send,
                  transport=ConsentHttpTransport,
                  traces_sample_rate=1.0,
                  max_value_length=8192,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_tag("dirty", build_metadata.openpilot.is_dirty)
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", build_metadata.channel)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())

  return True
