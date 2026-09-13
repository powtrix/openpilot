import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.tests.test_carrot_model_renderer_lane_visibility import model_renderer_module as model_renderer_module


@pytest.fixture
def renderer(monkeypatch, model_renderer_module):
  model_renderer = model_renderer_module
  value = object.__new__(model_renderer.ModelRenderer)
  value._deceleration_exclusion_rect = None
  value._font_display = object()
  value._carrot_path_x = 1050
  value._carrot_path_y = 900
  value._carrot_soft_hold_active = False
  value._carrot_brake_hold_active = False
  value._carrot_carrot_cruise = False
  value._carrot_long_active = False
  value._carrot_x_state = 0
  value._carrot_radar_dist = 0.0
  value._carrot_vision_dist = 0.0
  value._carrot_radar_track_id = 1
  value._carrot_tf_distance = 0.0
  value._carrot_lead_status = False
  value._carrot_v_ego = 10.0
  value._carrot_traffic_state = 0
  value._draw_text_box_carrot = lambda *args: None
  # Include the real font scale in a deterministic measurement, without GL.
  monkeypatch.setattr(model_renderer, "measure_text_cached", lambda font, text, size: SimpleNamespace(x=len(text) * size * 0.6, y=size * 1.242))
  monkeypatch.setattr(model_renderer, "draw_text_ui_style", lambda *args, **kwargs: None)
  return value


def test_no_drawn_label_leaves_no_exclusion(renderer):
  renderer._draw_path_end_overlay_carrot()
  assert renderer.deceleration_exclusion_rect is None


def test_two_lead_labels_preserve_text_and_box_bounds(renderer):
  renderer._carrot_radar_dist = 12.3
  renderer._carrot_vision_dist = 14.5
  renderer._draw_path_end_overlay_carrot()
  rect = renderer.deceleration_exclusion_rect
  assert rect.x == 1050 - 80 - 64 - 10
  assert rect.width == 160 + 128 + 20
  assert rect.y == pytest.approx(960 - 40 * 1.242 / 2 - 10)
  assert rect.height == pytest.approx(40 * 1.242 + 20)


@pytest.mark.parametrize("state, expected", [(3, "Signal slowing"), (4, "E2E주행중"), (5, "Signal slowing")])
def test_longitudinal_status_label_is_reserved_without_leads(renderer, state, expected):
  renderer._carrot_long_active = True
  renderer._carrot_x_state = state
  renderer._draw_path_end_overlay_carrot()
  rect = renderer.deceleration_exclusion_rect
  assert rect.width == pytest.approx(len(expected) * 50 * 0.6 + 20)
  assert rect.height == pytest.approx(50 * 1.242 + 20)


@pytest.mark.parametrize("brake_hold, expected", [(False, "SOFTHOLD"), (True, "AUTOHOLD")])
def test_hold_label_is_reserved(renderer, brake_hold, expected):
  renderer._carrot_soft_hold_active = True
  renderer._carrot_brake_hold_active = brake_hold
  renderer._draw_path_end_overlay_carrot()
  assert renderer.deceleration_exclusion_rect.width == pytest.approx(len(expected) * 50 * 0.6 + 20)


def test_exclusion_property_does_not_expose_mutable_internal_rect(renderer):
  renderer._record_deceleration_exclusion_text("12.3", 500, 900, 40)
  original = renderer.deceleration_exclusion_rect.x
  rect = renderer.deceleration_exclusion_rect
  object.__setattr__(rect, "x", -1000)
  assert renderer.deceleration_exclusion_rect.x == original


def test_nonfinite_label_position_does_not_poison_bounds(renderer):
  renderer._record_deceleration_exclusion_text("12.3", float("nan"), 900, 40)
  assert renderer.deceleration_exclusion_rect is None


def test_early_return_clears_previous_frame_exclusion(renderer, monkeypatch, model_renderer_module):
  model_renderer = model_renderer_module
  renderer._record_deceleration_exclusion_text("12.3", 500, 900, 40)
  monkeypatch.setattr(model_renderer.ui_state, "sm", SimpleNamespace(recv_frame={"liveCalibration": 0, "modelV2": 0}), raising=False)
  monkeypatch.setattr(model_renderer.ui_state, "started_frame", 1, raising=False)
  renderer._render(object())
  assert renderer.deceleration_exclusion_rect is None


def _road_view_hud_fragment():
  path = Path(__file__).parents[1] / "onroad" / "augmented_road_view.py"
  tree = ast.parse(path.read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "AugmentedRoadView")
  render = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "_render")
  start = next(i for i, node in enumerate(render.body) if isinstance(node, ast.Assign) and any(
    isinstance(target, ast.Attribute) and target.attr == "deceleration_exclusion_rect" for target in node.targets
  ))
  end = next(i for i in range(start, len(render.body)) if isinstance(render.body[i], ast.Expr)
             and isinstance(render.body[i].value, ast.Call)
             and ast.unparse(render.body[i].value.func) == "self._hud_renderer.render")
  render.body = render.body[start:end + 1]
  namespace = {"time": SimpleNamespace(monotonic=lambda: 0.0)}
  exec(compile(ast.Module(body=[render], type_ignores=[]), str(path), "exec"), namespace)
  return namespace["_render"]


@pytest.mark.parametrize("suppressed", [False, True])
def test_road_view_forwards_only_this_frame_rendered_model_exclusion(suppressed):
  calls = []
  bound = object()
  model = SimpleNamespace(deceleration_exclusion_rect=bound, render=lambda rect: calls.append("model"))
  hud = SimpleNamespace(deceleration_exclusion_rect=object())
  hud.render = lambda rect: calls.append(hud.deceleration_exclusion_rect)
  view = SimpleNamespace(_hud_renderer=hud, model_renderer=model, _suppress_camera_for_cluster=suppressed, _content_rect=object())
  _road_view_hud_fragment()(view, object())
  assert calls == ([None] if suppressed else ["model", bound])
