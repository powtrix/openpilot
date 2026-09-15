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

import pyray as rl

# Do not initialize sockets, real Params paths, or a display window. The rest
# of the module imports real application, rendering and PyRay dependencies.
ui_state = types.ModuleType('openpilot.selfdrive.ui.ui_state')
ui_state.ui_state = types.SimpleNamespace()
sys.modules[ui_state.__name__] = ui_state

from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer

# Exercise the runtime constructor and property, not a dataclass replacement.
renderer = object.__new__(ModelRenderer)
renderer._deceleration_exclusion_rect = rl.Rectangle(1, 2, 3, 4)
copied = renderer.deceleration_exclusion_rect
assert (copied.x, copied.y, copied.width, copied.height) == (1, 2, 3, 4)
copied.x = 99
assert renderer.deceleration_exclusion_rect.x == 1
renderer._deceleration_exclusion_rect = None
assert renderer.deceleration_exclusion_rect is None
print('real-pyray ModelRenderer import and rectangle property passed')
"""
  env = dict(os.environ, SCALE="1")
  result = subprocess.run([sys.executable, "-c", script], cwd=repo_root, env=env,
                          capture_output=True, text=True, timeout=30, check=False)
  assert result.returncode == 0, result.stdout + result.stderr
  assert "real-pyray ModelRenderer import and rectangle property passed" in result.stdout
