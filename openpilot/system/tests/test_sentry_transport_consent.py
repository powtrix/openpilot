from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from openpilot.system import sentry


def _transport(pool, generation: str = "generation-1"):
  transport = object.__new__(sentry.ConsentHttpTransport)
  transport._dk_transport_state = threading.local()
  transport._dk_transport_state.consent_generation = generation
  transport._pool = pool
  transport._auth = SimpleNamespace(get_api_url=lambda _endpoint_type: "https://sentry.example/envelope")
  return transport


def test_before_send_preserves_bound_operation_generation(monkeypatch):
  monkeypatch.setattr(sentry, "third_party_data_sharing_generation", lambda: "generation-2")
  event = {"message": "diagnostic"}

  with sentry._bind_event_consent_generation("generation-1"):
    assert sentry._before_send(event, {}) is event

  assert event[sentry.SENTRY_CONSENT_GENERATION_FIELD] == "generation-1"


def test_capture_exception_drops_old_operation_after_aba(monkeypatch):
  params = SimpleNamespace(
    _dk_consent_generation="generation-1",
    values={"DkThirdPartyDataSharing": True, "CarrotExceptionSent": True},
  )
  params.get = lambda key, *args, **kwargs: params.values.get(key)
  params.get_bool = lambda key: bool(params.values.get(key))
  params.put = lambda key, value: params.values.__setitem__(key, value)
  captured = []

  def initialize(_project):
    params._dk_consent_generation = "generation-2"
    return True

  monkeypatch.setattr(sentry, "Params", lambda: params)
  monkeypatch.setattr(sentry, "_ensure_initialized", initialize)
  monkeypatch.setattr(sentry.sentry_sdk, "capture_exception", lambda *args, **kwargs: captured.append((args, kwargs)))

  sentry.capture_exception(RuntimeError("old operation"))

  assert captured == []


def test_sentry_request_disables_redirects_and_aborts_aba_mid_body(monkeypatch):
  state = {"generation": "generation-1"}
  request_options = {}

  monkeypatch.setattr(
    sentry,
    "third_party_data_sharing_generation_matches",
    lambda expected, *_args: expected == state["generation"],
  )

  class Pool:
    def request(self, _method, _url, **kwargs):
      request_options.update(kwargs)
      assert kwargs["body"].read() == b"x" * sentry.SENTRY_UPLOAD_CHUNK_SIZE
      state["generation"] = "generation-2"
      kwargs["body"].read()
      raise AssertionError("stale Sentry body resumed after consent ABA")

  transport = _transport(Pool())
  payload = b"x" * (sentry.SENTRY_UPLOAD_CHUNK_SIZE + 1)

  with pytest.raises(RuntimeError, match="consent generation changed"):
    transport._request("POST", object(), payload, {})

  assert request_options["redirect"] is False
  assert request_options["headers"]["Content-Length"] == str(len(payload))


def test_sentry_body_rechecks_generation_after_forming_each_chunk(monkeypatch):
  checks = iter((True, False))
  monkeypatch.setattr(sentry, "third_party_data_sharing_generation_matches", lambda *_args: next(checks))
  body = sentry._ConsentBoundBody(b"payload", "generation-1")

  with pytest.raises(RuntimeError, match="during network write"):
    body.read()


def test_sentry_request_rechecks_generation_after_response(monkeypatch):
  state = {"generation": "generation-1"}
  response = SimpleNamespace(closed=False)
  response.close = lambda: setattr(response, "closed", True)

  monkeypatch.setattr(
    sentry,
    "third_party_data_sharing_generation_matches",
    lambda expected, *_args: expected == state["generation"],
  )

  class Pool:
    def request(self, _method, _url, **kwargs):
      assert kwargs["redirect"] is False
      while kwargs["body"].read():
        pass
      state["generation"] = "generation-2"
      return response

  transport = _transport(Pool())

  with pytest.raises(RuntimeError, match="during network request"):
    transport._request("POST", object(), b"payload", {})

  assert response.closed
