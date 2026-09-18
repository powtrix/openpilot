"""Keep real PyRay constructors in the UI startup regression test.

Most geometry tests use dataclass rectangles. PyRay's Rectangle is instead a
factory function, so eager ``rl.Rectangle | None`` annotations fail at import.
Only the hardware-backed ui_state singleton is replaced in this subprocess.
"""

import os
from pathlib import Path
import subprocess
import sys


def test_model_renderer_import_with_real_pyray():
  repo_root = Path(__file__).resolve().parents[4]
  script = """
import sys
import types
from enum import Enum
from collections import defaultdict

import pyray as rl

# Do not initialize sockets, real Params paths, or a display window. The rest
# of the module imports real application, rendering and PyRay dependencies.
ui_state = types.ModuleType('openpilot.selfdrive.ui.ui_state')
ui_state.ui_state = types.SimpleNamespace()
class UIStatus(Enum):
  DISENGAGED = 'disengaged'
  ENGAGED = 'engaged'
  OVERRIDE = 'override'
ui_state.UIStatus = UIStatus
sys.modules[ui_state.__name__] = ui_state

from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer

# Keep the real dependency even though the now-unneeded exclusion property is
# removed. Rectangle remains a factory, so future UI annotations need this gate.
rect = rl.Rectangle(1, 2, 3, 4)
assert (rect.x, rect.y, rect.width, rect.height) == (1, 2, 3, 4)
assert callable(ModelRenderer._draw_path_end_overlay_carrot)

# Import and execute the actual bottom-border method with real cereal and
# PyRay constructors. Intercept only GPU draws (no window in this test).
from openpilot.cereal import car, log
from openpilot.selfdrive.ui.carrot_param_cache import BorderParamSnapshot
from openpilot.selfdrive.ui.onroad import augmented_road_view as road
class SM(dict):
  pass
cs = car.CarState.new_message(canValid=True)
cs.dkStockScc.valid = cs.dkStockScc.active = True
cs.dkStockScc.accelRequest = -2
cs.dkStockScc.sourceMonoTime = 10_000_000_000
cs.leftBlinker = cs.rightBlinker = True
cs.dkTurnSignalLamps.supported = cs.dkTurnSignalLamps.valid = True
cs.dkTurnSignalLamps.right = True
cs.dkTurnSignalLamps.sourceMonoTime = 10_000_000_000
sm = SM(carState=cs.as_reader(), selfdriveState=log.SelfdriveState.new_message().as_reader())
sm.alive = defaultdict(bool, dict.fromkeys(sm, True))
sm.valid = dict.fromkeys(sm, True)
sm.recv_frame = dict.fromkeys(sm, 101)
sm.recv_time = dict.fromkeys(sm, 10.0)
sm.logMonoTime = dict.fromkeys(sm, 10_000_000_000)
ui_state.ui_state.sm = sm
ui_state.ui_state.started = True
ui_state.ui_state.started_frame = 100
ui_state.ui_state.status = UIStatus.ENGAGED
ui_state.ui_state.lat_active = True
road.time.monotonic = lambda: 10.05
calls = []
rl.draw_rectangle = lambda *args: calls.append(('base', args))
rl.draw_rectangle_rec = lambda r, c: calls.append(('fill', (r.x, r.y, r.width, r.height, c.r, c.g, c.b)))
lamp_colors = []
rl.draw_rectangle_rounded = lambda r, rounding, segments, color: lamp_colors.append(color)
rl.draw_rectangle_rounded_lines_ex = lambda *args: None
road.draw_text_ui_style = lambda *args, **kwargs: calls.append(('text', args))
view = object.__new__(road.AugmentedRoadView)
view._dk_scc_braking_enabled = True
view._dk_turn_signal_lamps_enabled = True
view._border_params = types.SimpleNamespace(refresh=lambda now: BorderParamSnapshot())
view._draw_border_carrot(rl.Rectangle(300, 20, 1860, 1060))
fill = next(i for i, (kind, _) in enumerate(calls) if kind == 'fill')
assert calls[fill][1] == (780, 1020, 900, 60, 255, 0, 0)
assert lamp_colors == [rl.BLACK, rl.ORANGE]
assert calls[fill - 1][0] == 'base'
assert all(kind == 'text' for kind, _ in calls[fill + 1:])
assert len(calls[fill + 1:]) == 6
print('real-pyray ModelRenderer import passed')
print('real-pyray complete stock SCC border draw passed')
"""
  env = dict(os.environ, SCALE="1")
  result = subprocess.run([sys.executable, "-c", script], cwd=repo_root, env=env,
                          capture_output=True, text=True, timeout=30, check=False)
  assert result.returncode == 0, result.stdout + result.stderr
  assert "real-pyray ModelRenderer import passed" in result.stdout
  assert "real-pyray complete stock SCC border draw passed" in result.stdout
