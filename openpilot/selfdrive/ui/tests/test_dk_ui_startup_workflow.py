"""Keep DK CI coverage from silently falling back to build-only checks."""

from pathlib import Path

import yaml


def test_dk_pushes_and_pull_requests_run_real_startup_import_after_build():
  root = Path(__file__).resolve().parents[4]
  workflow = yaml.load((root / ".github/workflows/tests.yaml").read_text(), Loader=yaml.BaseLoader)
  assert "dkcarrot-wip" in workflow["on"]["push"]["branches"]
  assert "dkcarrot-wip" in workflow["on"]["pull_request"]["branches"]
  steps = workflow["jobs"]["build_release"]["steps"]
  build = next(i for i, step in enumerate(steps) if step.get("run") == "scons")
  startup = next(i for i, step in enumerate(steps) if step.get("name") == "Import the real DK UI startup graph")
  assert startup > build
  step = steps[startup]
  assert "refs/heads/dkcarrot-wip" in step["if"]
  assert "github.base_ref == 'dkcarrot-wip'" in step["if"]
  assert step["env"]["SCALE"] == "1"
  assert step["env"]["BIG"] == "1"
  assert step["env"]["OPENPILOT_PREFIX"].startswith("dk-ui-")
  assert "import openpilot.selfdrive.ui.ui" in step["run"]
  # An environment variable alone does not create the msgq namespace. The
  # real UI subscribes during import, before any vehicle publisher is started.
  assert "from openpilot.common.prefix import OpenpilotPrefix" in step["run"]
  assert "with OpenpilotPrefix():" in step["run"]
  assert "mock" not in step["run"]
  assert "continue-on-error" not in step
  assert 0 < int(step["timeout-minutes"]) <= 2


def test_dk_ci_does_not_skip_real_pyray_or_exact_commit_annotation_checks():
  root = Path(__file__).resolve().parents[4]
  workflow = yaml.load((root / ".github/workflows/tests.yaml").read_text(), Loader=yaml.BaseLoader)
  steps = workflow["jobs"]["build_release"]["steps"]
  step = next(step for step in steps if step.get("name") == "Verify DK startup and display regressions")
  assert "test_dk_model_renderer_runtime_import.py" in step["run"]
  assert "tools/dk/check_ui_annotations.py --commit HEAD" in step["run"]
  assert "test_check_ui_annotations.py" in step["run"]
  assert "test_stock_scc_braking.py" in step["run"]
  assert "test_dk_stock_scc_display.py" in step["run"]
  assert "continue-on-error" not in step
