import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.onroad.stock_scc_braking import (
  braking_bar_geometry, dk_scc_display_enabled, stock_scc_braking_fraction,
)


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__(
      carState=SimpleNamespace(canValid=True, accFaulted=False, brakePressed=False, gasPressed=False,
                               dkStockScc=SimpleNamespace(valid=True, active=True, accelRequest=-2.0,
                                                          sourceMonoTime=10_000_000_000)),
      selfdriveState=SimpleNamespace(alertSize=0),
    )
    self.alive = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.recv_frame = dict.fromkeys(self, 101)
    self.recv_time = dict.fromkeys(self, 10.0)
    self.logMonoTime = dict.fromkeys(self, 10_000_000_000)


def read(sm, now=10.05):
  return stock_scc_braking_fraction(sm, started=True, started_frame=100, now=now)


@pytest.mark.parametrize("branch, enabled", [
  ("dkcarrot-wip", True), (b"dkcarrot-wip\n", True), ("carrot-wip", False),
  ("carrot", False), ("origin/dkcarrot-wip", False), (None, False), (b"\xff", False),
])
def test_branch_gate(branch, enabled):
  assert dk_scc_display_enabled(branch) is enabled


@pytest.mark.parametrize("accel, fraction", [(-0.01, 0.0025), (-0.2, 0.05), (-2, 0.5), (-4, 1), (-6, 1), (-10.23, 1)])
def test_received_command_strength_controls_width(accel, fraction):
  sm = FakeSubMaster()
  sm["carState"].dkStockScc.accelRequest = accel
  assert read(sm) == pytest.approx(fraction)


@pytest.mark.parametrize("accel", [0, 0.1, -10.24, float("nan"), float("inf"), "-2", False, None])
def test_no_braking_or_invalid_value_hidden(accel):
  sm = FakeSubMaster()
  sm["carState"].dkStockScc.accelRequest = accel
  assert read(sm) is None


@pytest.mark.parametrize("field, value", [("canValid", False), ("accFaulted", True), ("brakePressed", True), ("gasPressed", True)])
def test_driver_override_and_invalid_can_hidden(field, value):
  sm = FakeSubMaster()
  setattr(sm["carState"], field, value)
  assert read(sm) is None


@pytest.mark.parametrize("field, value", [
  ("valid", False), ("active", False), ("sourceMonoTime", 0),
  ("sourceMonoTime", 9_840_000_000), ("sourceMonoTime", 10_100_000_000),
  ("sourceMonoTime", "10000000000"), ("sourceMonoTime", float("nan")),
])
def test_original_can_freshness_and_active_state_required(field, value):
  sm = FakeSubMaster()
  setattr(sm["carState"].dkStockScc, field, value)
  assert read(sm) is None


@pytest.mark.parametrize("service", ["carState", "selfdriveState"])
@pytest.mark.parametrize("field, value", [
  ("alive", False), ("valid", False), ("recv_frame", 100), ("recv_frame", 99),
  ("recv_time", 9.84), ("recv_time", 10.1), ("recv_time", 0), ("recv_time", None),
  ("logMonoTime", 9_840_000_000), ("logMonoTime", 10_100_000_000), ("logMonoTime", 0),
])
def test_source_and_delivery_freshness_required(service, field, value):
  sm = FakeSubMaster()
  getattr(sm, field)[service] = value
  assert read(sm) is None


def test_no_fallback_to_measured_or_planned_deceleration_even_when_stock_data_missing():
  sm = FakeSubMaster()
  cs = sm["carState"]
  cs.aEgo, cs.vEgo, cs.standstill, cs.brake, cs.brakeLights = -5.0, 0, True, 1, True
  sm["carControl"] = SimpleNamespace(actuators=SimpleNamespace(accel=-6))
  assert read(sm) == 0.5  # speed/standstill do not discard a genuine hold request
  cs.aEgo = 2.0
  assert read(sm) == 0.5  # display command onset before the vehicle responds
  del cs.dkStockScc
  assert read(sm) is None


@pytest.mark.parametrize("size", [1, 2, 3])
def test_alerts_hide_secondary_gauge(size):
  sm = FakeSubMaster()
  sm["selfdriveState"].alertSize = size
  assert read(sm) is None


def test_stale_command_expires_even_if_carstate_keeps_publishing():
  sm = FakeSubMaster()
  assert read(sm) == 0.5
  sm.recv_time = dict.fromkeys(sm, 10.25)
  sm.logMonoTime = dict.fromkeys(sm, 10_250_000_000)
  assert read(sm, now=10.25) is None


def test_unavailable_and_offroad_fail_closed():
  assert read(None) is None
  assert read({}) is None
  assert read(FakeSubMaster(), now=float("nan")) is None
  assert stock_scc_braking_fraction(FakeSubMaster(), started=False, started_frame=100, now=10.05) is None


def test_real_cereal_contract_round_trip_and_missing_default():
  from openpilot.cereal import car, log

  sm = FakeSubMaster()
  cs = car.CarState.new_message(canValid=True)
  sm["carState"] = cs.as_reader()
  sm["selfdriveState"] = log.SelfdriveState.new_message(alertSize="none").as_reader()
  assert read(sm) is None  # older recordings/unset additive field
  cs.dkStockScc.valid = True
  cs.dkStockScc.active = True
  cs.dkStockScc.accelRequest = -2
  cs.dkStockScc.sourceMonoTime = 10_000_000_000
  with car.CarState.from_bytes(cs.to_bytes()) as reader:
    sm["carState"] = reader
    assert read(sm) == 0.5


@pytest.mark.parametrize("fraction", [0.0025, 0.05, 0.5, 1.0])
@pytest.mark.parametrize("x, y, width, height", [(0, 0, 2160, 1080), (300, 20, 1860, 1060), (90, 50, 700, 500)])
def test_geometry_center_out_within_bottom_border(fraction, x, y, width, height):
  left, top, fill, thick = braking_bar_geometry(x, y, width, height, 30, fraction)
  assert left + fill / 2 == pytest.approx(x + width / 2)
  assert top == y + height - 30
  assert thick == 30
  assert fill == pytest.approx((width - 60) * fraction)
  assert left >= x + 30
  assert left + fill <= x + width - 30


@pytest.mark.parametrize("index, value", [(0, float("nan")), (2, 60), (3, 29), (4, 0), (5, 0), (5, -1), (5, 1.01)])
def test_invalid_geometry_hidden(index, value):
  args = [0, 0, 2160, 1080, 30, 0.5]
  args[index] = value
  assert braking_bar_geometry(*args) is None


def _road_method(name):
  path = Path(__file__).parents[1] / "onroad" / "augmented_road_view.py"
  tree = ast.parse(path.read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "AugmentedRoadView")
  return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)


def test_border_layer_order_preserves_camera_clipping_and_text():
  render = ast.unparse(_road_method("_render"))
  assert render.index("rl.end_scissor_mode()") < render.index("self._draw_border_carrot(rect)")
  border = ast.unparse(_road_method("_draw_border_carrot"))
  assert border.index("int(y + h - thickness)") < border.index("self._draw_stock_scc_braking_border(rect)")
  assert border.index("self._draw_stock_scc_braking_border(rect)") < border.index("draw_text_ui_style(bottom,")
  hud = (Path(__file__).parents[1] / "onroad" / "hud_renderer.py").read_text()
  model = (Path(__file__).parents[1] / "onroad" / "model_renderer.py").read_text()
  assert "deceleration_indicator" not in hud
  assert "deceleration_exclusion" not in model


def test_actual_border_draw_uses_real_pyray_rectangle_without_window(monkeypatch):
  import pyray as rl

  sm = FakeSubMaster()
  calls = []
  monkeypatch.setattr(rl, "draw_rectangle_rec", lambda rect, color: calls.append((rect, color)))
  namespace = dict(rl=rl, UI_BORDER_SIZE=30, stock_scc_braking_fraction=stock_scc_braking_fraction,
                   braking_bar_geometry=braking_bar_geometry, time=SimpleNamespace(monotonic=lambda: 10.05),
                   ui_state=SimpleNamespace(sm=sm, started=True, started_frame=100))
  method = _road_method("_draw_stock_scc_braking_border")
  exec(compile(ast.Module(body=[method], type_ignores=[]), "stock_scc_border_draw", "exec"), namespace)
  draw = namespace[method.name]
  view = SimpleNamespace(_dk_scc_braking_enabled=True)
  rect = rl.Rectangle(300, 20, 1860, 1060)
  draw(view, rect)
  assert len(calls) == 1
  drawn, color = calls[0]
  assert (drawn.x, drawn.y, drawn.width, drawn.height) == (780, 1050, 900, 30)
  assert (color.r, color.g, color.b, color.a) == (255, 0, 0, 255)
  view._dk_scc_braking_enabled = False
  draw(view, rect)
  assert len(calls) == 1
