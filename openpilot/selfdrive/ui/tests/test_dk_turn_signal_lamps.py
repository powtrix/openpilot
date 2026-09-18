import ast
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.onroad.dk_turn_signal_lamps import border_turn_signal_lamps


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__(carState=SimpleNamespace(
      canValid=True, leftBlinker=True, rightBlinker=False, steeringPressed=False, logCarrot="",
      dkTurnSignalLamps=SimpleNamespace(supported=True, valid=True, left=True, right=False, sourceMonoTime=10_000_000_000),
    ))
    self.alive = defaultdict(bool, carState=True)
    self.valid = {"carState": True}
    self.recv_frame = {"carState": 101}
    self.recv_time = {"carState": 10.0}
    self.logMonoTime = {"carState": 10_000_000_000}


def read(sm, now=10.05):
  return border_turn_signal_lamps(sm, started=True, started_frame=100, now=now)


@pytest.mark.parametrize("left, right", [(False, False), (True, False), (False, True), (True, True)])
def test_received_phase_not_held_control_state_or_synthetic_timer(left, right):
  sm = FakeSubMaster()
  sm["carState"].dkTurnSignalLamps.left = left
  sm["carState"].dkTurnSignalLamps.right = right
  assert read(sm) == (left, right)
  assert sm["carState"].leftBlinker  # observer never alters lane-change/control state
  assert not sm["carState"].rightBlinker


def test_on_off_on_and_hazards_are_forwarded_without_a_hold_or_phase_delay():
  sm = FakeSubMaster()
  for phase in (True, False, True, False):
    sm["carState"].dkTurnSignalLamps.left = phase
    sm["carState"].dkTurnSignalLamps.right = phase
    assert read(sm) == (phase, phase)


@pytest.mark.parametrize("field, value", [("valid", False), ("sourceMonoTime", 0),
                                        ("sourceMonoTime", 9_800_000_000), ("sourceMonoTime", 10_100_000_000),
                                        ("sourceMonoTime", None), ("sourceMonoTime", float("nan"))])
def test_supported_but_invalid_lamps_hide_without_falling_back_to_held_state(field, value):
  sm = FakeSubMaster()
  setattr(sm["carState"].dkTurnSignalLamps, field, value)
  assert read(sm) == (False, False)


@pytest.mark.parametrize("field, value", [("alive", False), ("valid", False), ("recv_frame", 100),
                                        ("recv_time", 9.8), ("recv_time", 10.1), ("recv_time", None),
                                        ("logMonoTime", 9_800_000_000), ("logMonoTime", 10_100_000_000)])
def test_stale_carstate_never_leaves_indicator_lit(field, value):
  sm = FakeSubMaster()
  getattr(sm, field)["carState"] = value
  assert read(sm) == (False, False)


def test_invalid_can_offroad_and_missing_inputs_hide():
  sm = FakeSubMaster()
  sm["carState"].canValid = False
  assert read(sm) == (False, False)
  assert read(None) == (False, False)
  assert read({}) == (False, False)
  assert read(FakeSubMaster(), float("nan")) == (False, False)
  assert border_turn_signal_lamps(FakeSubMaster(), started=False, started_frame=100, now=10.05) == (False, False)


def test_other_cars_and_older_recordings_keep_existing_indicator_behavior():
  sm = FakeSubMaster()
  sm["carState"].dkTurnSignalLamps.supported = False
  assert read(sm) == (True, False)
  del sm["carState"].dkTurnSignalLamps
  assert read(sm) == (True, False)


def test_real_cereal_contract_round_trip():
  from openpilot.cereal import car

  sm = FakeSubMaster()
  cs = car.CarState.new_message(canValid=True, leftBlinker=True, rightBlinker=True)
  cs.dkTurnSignalLamps.supported = True
  sm["carState"] = cs
  assert read(sm) == (False, False)  # supported but no accepted sample yet
  cs.dkTurnSignalLamps.valid = True
  cs.dkTurnSignalLamps.right = True
  cs.dkTurnSignalLamps.sourceMonoTime = 10_000_000_000
  with car.CarState.from_bytes(cs.to_bytes()) as reader:
    sm["carState"] = reader
    assert read(sm) == (False, True)


@pytest.mark.parametrize("enabled", [False, True])
def test_actual_border_draw_follows_lamps_only_on_dk_branch(monkeypatch, enabled):
  import pyray as rl
  from openpilot.selfdrive.ui.carrot_param_cache import BorderParamSnapshot

  sm = FakeSubMaster()
  sm["carState"].dkTurnSignalLamps.left = False
  sm["carState"].dkTurnSignalLamps.right = True
  calls = []
  monkeypatch.setattr(rl, "draw_rectangle", lambda *args: None)
  monkeypatch.setattr(rl, "draw_rectangle_rounded_lines_ex", lambda *args: None)
  monkeypatch.setattr(rl, "draw_rectangle_rounded", lambda rect, rounding, segments, color: calls.append((rect, color)))
  path = Path(__file__).parents[1] / "onroad" / "augmented_road_view.py"
  cls = next(node for node in ast.parse(path.read_text()).body if isinstance(node, ast.ClassDef) and node.name == "AugmentedRoadView")
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "_draw_border_carrot")
  namespace = dict(rl=rl, UI_BORDER_SIZE=30, UIStatus=SimpleNamespace(ENGAGED=2, DISENGAGED=0, OVERRIDE=1),
                   time=SimpleNamespace(monotonic=lambda: 10.05), border_turn_signal_lamps=border_turn_signal_lamps,
                   ui_state=SimpleNamespace(sm=sm, started=True, started_frame=100, lat_active=True, status=2),
                   draw_text_ui_style=lambda *args, **kwargs: None)
  exec(compile(ast.Module(body=[method], type_ignores=[]), "turn_signal_border_draw", "exec"), namespace)
  view = SimpleNamespace(_dk_turn_signal_lamps_enabled=enabled,
                         _border_params=SimpleNamespace(refresh=lambda _: BorderParamSnapshot()),
                         _get_border_color=lambda _: rl.GREEN, _draw_stock_scc_braking_border=lambda _: None)
  namespace[method.name](view, rl.Rectangle(300, 20, 1860, 1060))
  assert len(calls) == 2
  expected = (rl.BLACK, rl.ORANGE) if enabled else (rl.ORANGE, rl.BLACK)
  assert [color for _, color in calls] == list(expected)
