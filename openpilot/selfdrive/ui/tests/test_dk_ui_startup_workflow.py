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
  assert "import openpilot.selfdrive.controls.controlsd" in step["run"]
  assert "import openpilot.selfdrive.carrot.dk_vehicle_diagnostics" in step["run"]
  assert "import openpilot.selfdrive.carrot.dk_diagnosticsd" in step["run"]
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
  assert "openpilot/selfdrive/ui/tests/test_dk_turn_signal_lamps.py" in step["run"]
  assert "opendbc/car/hyundai/tests/test_dk_turn_signal_lamps.py" in step["run"]
  assert "test_dk_stock_scc_display.py" in step["run"]
  assert "test_dk_vehicle_diagnostics.py" in step["run"]
  assert "test_dk_turn_return.py" in step["run"]
  assert "test_dk_diagnosticsd.py" in step["run"]
  assert "test_dk_log_transfer.py" in step["run"]
  assert "test_dk_lateral_diagnostics.py" in step["run"]
  assert "test_ka4_lateral_sign.py" in step["run"]
  assert "test_dk_experimental_steering.py" in step["run"]
  assert "test_dk_steering_setting.py" in step["run"]
  assert "test_dk_diagnostics_report.py" in step["run"]
  assert "continue-on-error" not in step


def test_can_replay_dependency_is_built_before_test_collection():
  root = Path(__file__).resolve().parents[4]
  workflow = yaml.load((root / ".github/workflows/tests.yaml").read_text(), Loader=yaml.BaseLoader)
  steps = workflow["jobs"]["build_release"]["steps"]
  build = next(i for i, step in enumerate(steps) if step.get("name") == "Build DK CAN replay test library")
  tests = next(i for i, step in enumerate(steps) if step.get("name") == "Verify DK startup and display regressions")
  assert build < tests
  assert steps[build]["run"] == "scons -C tools/dk -f safety_tests.scons"
  assert "continue-on-error" not in steps[build]
  assert "test_stock_scc_can_replay.py" in steps[tests]["run"]
  entry = (root / "tools/dk/safety_tests.scons").read_text()
  assert "opendbc/safety/tests/libsafety/SConscript" in entry
