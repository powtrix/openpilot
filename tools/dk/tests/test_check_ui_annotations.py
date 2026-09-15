import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
import pyray


CHECK_PATH = Path(__file__).resolve().parents[1] / "check_ui_annotations.py"
spec = importlib.util.spec_from_file_location("dk_ui_annotations", CHECK_PATH)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


@pytest.mark.parametrize("annotation", ["rl.Rectangle | None", "None | rl.Color", "list[rl.Vector2 | None]"])
def test_real_function_constructors_reject_eager_unions(annotation):
  assert callable(pyray.Rectangle) and not isinstance(pyray.Rectangle, type)
  failures = check.check_source(f"import pyray as rl\ndef draw() -> {annotation}: pass\n", "hud.py", pyray)
  assert len(failures) == 1
  assert "unsupported operand" in failures[0]


@pytest.mark.parametrize("source", [
  "import pyray as rl\ndef draw() -> rl.Rectangle: pass\n",
  "import pyray as rl\ndef draw() -> 'rl.Rectangle | None': pass\n",
  "from __future__ import annotations\nimport pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n",
  "import pyray as rl\ndef draw():\n  value: rl.Rectangle | None = None\n",
  "import pyray as rl\nfrom typing import Union\ndef draw() -> Union[rl.Rectangle, None]: pass\n",
  "import pyray as rl\ndef draw() -> tuple[str, rl.Rectangle]: pass\n",
])
def test_valid_or_deferred_annotations_are_preserved(source):
  assert check.check_source(source, "hud.py", pyray) == []


@pytest.mark.parametrize("source", [
  "from pyray import Rectangle as Rect\ndef draw(x: Rect | None): pass\n",
  "import pyray as ray\nvalue: ray.Rectangle | None = None\n",
  "import pyray as ray\nclass Hud:\n  value: ray.Rectangle | None = None\n",
  "import pyray as ray\nasync def draw(*args: ray.Rectangle | None): pass\n",
])
def test_aliases_class_module_and_async_annotations_are_checked(source):
  assert len(check.check_source(source, "hud.py", pyray)) == 1


def test_annotation_calls_and_other_module_code_are_never_executed(tmp_path):
  marker = tmp_path / "must-not-exist"
  source = (f"import pyray as rl\nopen({str(marker)!r}, 'w').write('bad')\n"
            + f"def draw() -> (open({str(marker)!r}, 'w'), rl.Rectangle)[1]: pass\n")
  assert check.check_source(source, "hud.py", pyray)
  assert not marker.exists()


@pytest.mark.parametrize("source,reason", [
  ("import pyray as rl\nreturn None\n", "outside function"),
  ("import pyray as rl\ndef draw(x, x) -> rl.Rectangle: pass\n", "duplicate argument"),
  ("value = 1\nfrom __future__ import annotations\nimport pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n",
   "beginning of the file"),
  ("from __future__ import annotations\nimport pyray as rl\ndef draw(x, x) -> rl.Rectangle | None: pass\n",
   "duplicate argument"),
])
def test_compiler_invalid_source_is_rejected_before_annotation_shortcuts(source, reason):
  with pytest.raises(SyntaxError, match=reason):
    check.check_source(source, "hud.py", pyray)


def test_compile_validation_does_not_execute_valid_source(tmp_path):
  marker = tmp_path / "must-not-exist"
  source = (f"from __future__ import annotations\nopen({str(marker)!r}, 'w').write('bad')\n"
            + "import pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n")
  assert check.check_source(source, "hud.py", pyray) == []
  assert not marker.exists()


def run_git(repo, *args):
  return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


@pytest.fixture
def repository(tmp_path):
  run_git(tmp_path, "init", "-q")
  run_git(tmp_path, "config", "user.email", "test@example.invalid")
  run_git(tmp_path, "config", "user.name", "Test")
  path = tmp_path / "openpilot/selfdrive/ui/hud.py"
  path.parent.mkdir(parents=True)
  return tmp_path, path


def test_exact_commit_not_dirty_worktree_is_checked(repository):
  repo, path = repository
  bad = "import pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n"
  path.write_text(bad)
  run_git(repo, "add", ".")
  run_git(repo, "commit", "-qm", "bad annotation")
  bad_commit = run_git(repo, "rev-parse", "HEAD")
  path.write_text("from __future__ import annotations\n" + bad)
  assert check.check_commit(repo, bad_commit, pyray)[1]
  run_git(repo, "add", ".")
  run_git(repo, "commit", "-qm", "defer annotation")
  fixed_commit = run_git(repo, "rev-parse", "HEAD")
  path.write_text(bad)
  assert check.check_commit(repo, fixed_commit, pyray)[1] == []


@pytest.mark.parametrize("bad", [
  "import pyray as rl\nreturn None\n",
  "value = 1\nfrom __future__ import annotations\nimport pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n",
])
def test_compiler_invalid_exact_commit_fails_closed_despite_valid_worktree(repository, bad):
  repo, path = repository
  path.write_text(bad)
  run_git(repo, "add", ".")
  run_git(repo, "commit", "-qm", "compiler-invalid source")
  bad_commit = run_git(repo, "rev-parse", "HEAD")
  path.write_text("from __future__ import annotations\nimport pyray as rl\ndef draw() -> rl.Rectangle | None: pass\n")
  with pytest.raises(SyntaxError):
    check.check_commit(repo, bad_commit, pyray)
  result = subprocess.run([sys.executable, str(CHECK_PATH), "--repo", str(repo), "--commit", bad_commit],
                          capture_output=True, text=True, timeout=30)
  assert result.returncode == 2
  assert "push blocked" in result.stderr
  assert "passed" not in result.stdout


SHA = "1" * 40
ZERO = "0" * 40
RECORD = f"refs/heads/local {SHA} refs/heads/dkcarrot-wip {ZERO}\n".encode()


@pytest.mark.parametrize("name,url", [
  ("powtrix", ""), ("origin", "https://github.com/powtrix/openpilot.git"),
  ("origin", "git@github.com:powtrix/openpilot.git"), ("origin", "ssh://git@github.com/powtrix/openpilot.git"),
])
def test_only_intended_remote_and_branch_are_gated(name, url):
  assert check.pushed_commits(RECORD, name, url) == [SHA]
  assert check.pushed_commits(RECORD.replace(b"refs/heads/dkcarrot-wip", b"refs/heads/carrot-wip"), name, url) == []
  assert check.pushed_commits(RECORD.replace(SHA.encode(), ZERO.encode()), name, url) == []


def test_unrelated_remote_and_empty_updates_do_not_require_a_check():
  assert check.pushed_commits(RECORD, "origin", "https://github.com/ajouatom/openpilot.git") == []
  assert check.pushed_commits(b"", "powtrix", "") == []


@pytest.mark.parametrize("data", [b"invalid\n", b"\xff", RECORD * (check.MAX_PUSH_REFS + 1), b"x" * (check.MAX_PUSH_BYTES + 1)])
def test_malformed_or_oversized_push_records_fail_closed(data):
  with pytest.raises(check.CheckFailure):
    check.pushed_commits(data, "powtrix", "")


def test_dependency_missing_fails_closed():
  # -I -S excludes PYTHONPATH and site-packages, so no fake pyray can be used.
  result = subprocess.run([sys.executable, "-I", "-S", str(CHECK_PATH), "--commit", SHA], capture_output=True, text=True)
  assert result.returncode == 2
  assert "Real pyray is unavailable" in result.stderr


def test_non_target_push_does_not_import_missing_dependency():
  result = subprocess.run([sys.executable, "-I", "-S", str(CHECK_PATH), "--pre-push", "--remote-name", "other"],
                          input=RECORD, capture_output=True)
  assert result.returncode == 0
