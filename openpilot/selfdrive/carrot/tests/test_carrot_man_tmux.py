from types import SimpleNamespace

from openpilot.selfdrive.carrot import carrot_man
from openpilot.selfdrive.carrot.carrot_man import (
  AUTOMATIC_EXCEPTION_TMUX_REASONS,
  CARROT_CAN_ERROR_TMUX_DELAY_SECONDS,
  CARROT_EXCEPTION_TMUX_REASONS,
  carrot_can_error,
  carrot_can_error_sources,
  carrot_can_error_send_ready,
  carrot_tmux_reason_allowed,
)


def test_spi_error_requests_tmux_capture():
  assert "spi_error" in CARROT_EXCEPTION_TMUX_REASONS


def test_egpu_error_requests_tmux_capture():
  assert "egpu_error" in CARROT_EXCEPTION_TMUX_REASONS


def test_can_error_send_waits_for_five_seconds_after_detection():
  detected_at = 100.0

  assert not carrot_can_error_send_ready(None, detected_at + 10.0, True)
  assert not carrot_can_error_send_ready(detected_at, detected_at + CARROT_CAN_ERROR_TMUX_DELAY_SECONDS - 0.01, True)
  assert carrot_can_error_send_ready(detected_at, detected_at + CARROT_CAN_ERROR_TMUX_DELAY_SECONDS, True)


def test_can_error_send_is_canceled_offroad():
  assert not carrot_can_error_send_ready(100.0, 110.0, False)


def test_can_error_ignores_mock_car():
  invalid_car_state = SimpleNamespace(canTimeout=False, canValid=False)
  radar_state = SimpleNamespace(radarErrors=SimpleNamespace(canError=False))

  assert not carrot_can_error("MOCK", True, invalid_car_state, True, radar_state)
  assert not carrot_can_error(b"MOCK", True, invalid_car_state, True, radar_state)


def test_can_error_detects_car_and_radar_errors():
  valid_car_state = SimpleNamespace(canTimeout=False, canValid=True)
  invalid_car_state = SimpleNamespace(canTimeout=False, canValid=False)
  radar_ok = SimpleNamespace(radarErrors=SimpleNamespace(canError=False))
  radar_error = SimpleNamespace(radarErrors=SimpleNamespace(canError=True))

  assert carrot_can_error("HYUNDAI_SONATA", True, invalid_car_state, True, radar_ok)
  assert carrot_can_error("HYUNDAI_SONATA", True, valid_car_state, True, radar_error)
  assert not carrot_can_error("HYUNDAI_SONATA", False, invalid_car_state, False, radar_error)


def test_can_error_ignores_stale_previous_onroad_state():
  stale_invalid_car_state = SimpleNamespace(canTimeout=True, canValid=False)
  stale_radar_error = SimpleNamespace(radarErrors=SimpleNamespace(canError=True))

  assert carrot_can_error_sources(
    "HYUNDAI_SONATA", False, stale_invalid_car_state, False, stale_radar_error,
  ) == (False, False)


class _FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value


def test_automatic_exception_tmux_queue_fails_closed_without_community_consent(monkeypatch):
  params = _FakeParams()
  monkeypatch.setattr(carrot_man, "Params", lambda: params)
  monkeypatch.setattr(carrot_man, "community_data_sharing_enabled", lambda _params=None: False)
  carrot_man.reset_carrot_exception_tmux_send_queue()

  assert not carrot_man.queue_carrot_exception_tmux_send("test")
  assert "CarrotException" not in params.values


def test_automatic_exception_tmux_queue_keeps_automatic_provenance(monkeypatch):
  params = _FakeParams({"CarrotCommunityDataSharing": "1"})
  monkeypatch.setattr(carrot_man, "Params", lambda: params)
  monkeypatch.setattr(carrot_man, "community_data_sharing_enabled", lambda _params=None: True)
  carrot_man.reset_carrot_exception_tmux_send_queue()

  assert carrot_man.queue_carrot_exception_tmux_send("automatic failure")
  assert params.values["CarrotException"] == "exception"
  assert params.values["CarrotException"] in AUTOMATIC_EXCEPTION_TMUX_REASONS
  assert params.values["CarrotException"] != "tmux_send"


def test_explicit_tools_tmux_send_remains_allowed_without_community_consent():
  assert "tmux_send" not in AUTOMATIC_EXCEPTION_TMUX_REASONS
  assert carrot_tmux_reason_allowed("tmux_send", False)
  assert not carrot_tmux_reason_allowed("exception", False)
  assert not carrot_tmux_reason_allowed("can_error", False)
  assert carrot_tmux_reason_allowed("exception", True)


def test_carrot_logs_and_bundled_exception_discord_are_gated(monkeypatch):
  instance = object.__new__(carrot_man.CarrotMan)
  instance.params = _FakeParams()
  monkeypatch.setattr(carrot_man, "community_data_sharing_enabled", lambda _params=None: False)
  for key in (
    "CARROT_EXCEPTION_DISCORD_WEBHOOK_URL",
    "CARROT_DISCORD_WEBHOOK_URL",
    "DISCORD_WEBHOOK_URL",
  ):
    monkeypatch.delenv(key, raising=False)
  monkeypatch.setattr(
    instance,
    "_tmux_upload_payload",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("payload must not be collected")),
  )

  assert instance.send_tmux_carrot_logs("onroad") is None
  assert instance._tmux_discord_webhook_url() == ""


def test_custom_exception_discord_remains_independent(monkeypatch):
  instance = object.__new__(carrot_man.CarrotMan)
  instance.params = _FakeParams()
  monkeypatch.setattr(carrot_man, "community_data_sharing_enabled", lambda _params=None: False)
  monkeypatch.setenv("CARROT_EXCEPTION_DISCORD_WEBHOOK_URL", "https://operator.example/webhook")

  assert instance._tmux_discord_webhook_url() == "https://operator.example/webhook"
