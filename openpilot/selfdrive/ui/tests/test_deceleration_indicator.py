from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.onroad.deceleration_indicator import deceleration_display


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__(carState=SimpleNamespace(vEgo=15.0, aEgo=-1.5, standstill=False),
                     selfdriveState=SimpleNamespace(alertSize=0))
    self.alive = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.recv_frame = dict.fromkeys(self, 101)
    self.recv_time = dict.fromkeys(self, 10.0)
    self.logMonoTime = dict.fromkeys(self, 10_000_000_000)


def read(sm, **kwargs):
  return deceleration_display(sm, **dict(started=True, started_frame=100, now=10.05, **kwargs))


def test_uses_measured_deceleration_without_any_brake_or_control_fields():
  display = read(FakeSubMaster())
  assert display is not None
  assert display.magnitude == 1.5
  assert display.fraction == 0.375
  assert display.text == "감속 1.5 m/s^2"


@pytest.mark.parametrize("accel, fraction", [(-0.1, 0.025), (-0.5, 0.125), (-2, 0.5), (-4, 1.0), (-6, 1.0), (-20, 1.0)])
def test_increasing_deceleration_increases_bar_only_number_remains_uncapped(accel, fraction):
  sm = FakeSubMaster()
  sm["carState"].aEgo = accel
  display = read(sm)
  assert display is not None
  assert display.fraction == fraction
  assert display.magnitude == -accel
  assert display.text == f"감속 {-accel:.1f} m/s^2"


@pytest.mark.parametrize("field, value", [
  ("aEgo", 0), ("aEgo", 1), ("aEgo", -0.099), ("aEgo", -20.01),
  ("aEgo", float("nan")), ("aEgo", float("inf")), ("aEgo", float("-inf")),
  ("aEgo", "-1.5"), ("aEgo", False), ("aEgo", None),
  ("vEgo", 0), ("vEgo", 0.49), ("vEgo", -1), ("vEgo", float("nan")),
  ("vEgo", float("inf")), ("vEgo", "15"), ("vEgo", True), ("standstill", True),
])
def test_invalid_stationary_or_non_decelerating_values_are_hidden(field, value):
  sm = FakeSubMaster()
  setattr(sm["carState"], field, value)
  assert read(sm) is None


@pytest.mark.parametrize("service", ["carState", "selfdriveState"])
@pytest.mark.parametrize("field, value", [
  ("alive", False), ("valid", False), ("recv_frame", 0), ("recv_frame", 99), ("recv_frame", 100),
  ("recv_time", 0), ("recv_time", 9.54), ("recv_time", 10.1), ("recv_time", float("nan")),
  ("recv_time", "10.0"), ("logMonoTime", 0), ("logMonoTime", 9_540_000_000),
  ("logMonoTime", 10_100_000_000), ("logMonoTime", float("inf")), ("logMonoTime", None),
])
def test_fresh_current_drive_source_and_reception_required(service, field, value):
  sm = FakeSubMaster()
  getattr(sm, field)[service] = value
  assert read(sm) is None


@pytest.mark.parametrize("service", ["carState", "selfdriveState"])
def test_missing_service_is_hidden(service):
  sm = FakeSubMaster()
  del sm[service]
  assert read(sm) is None


@pytest.mark.parametrize("size", [1, 2, 3])
def test_every_visible_alert_hides_indicator(size):
  sm = FakeSubMaster()
  sm["selfdriveState"].alertSize = size
  assert read(sm) is None


def test_offroad_missing_or_nonfinite_clock_hides_indicator():
  sm = FakeSubMaster()
  assert deceleration_display(sm, started=False, started_frame=100, now=10.05) is None
  assert deceleration_display(sm, started=True, started_frame=100, now=float("nan")) is None
  assert deceleration_display(None, started=True, started_frame=100, now=10.05) is None


def test_boundary_moving_speed_is_allowed():
  sm = FakeSubMaster()
  sm["carState"].vEgo = 0.5
  assert read(sm) is not None


def test_stale_value_does_not_linger_after_valid_display():
  sm = FakeSubMaster()
  assert read(sm) is not None
  assert deceleration_display(sm, started=True, started_frame=100, now=10.51) is None


def test_real_cereal_values_and_alert_enum():
  from openpilot.cereal import car, log

  sm = FakeSubMaster()
  cs = car.CarState.new_message(vEgo=15.0, aEgo=-1.5, standstill=False)
  ss = log.SelfdriveState.new_message(alertSize="none")
  sm["carState"] = cs.as_reader()
  sm["selfdriveState"] = ss.as_reader()
  assert read(sm).magnitude == 1.5
  ss.alertSize = "small"
  sm["selfdriveState"] = ss.as_reader()
  assert read(sm) is None
