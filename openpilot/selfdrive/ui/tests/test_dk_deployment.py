import json

import pytest

from openpilot.selfdrive.ui.dk_deployment import DK_RELEASE_MAX_BYTES, load_dk_deployment_text


def write_metadata(tmp_path, metadata):
  path = tmp_path / "dk_release.json"
  path.write_text(json.dumps(metadata), encoding="utf-8")
  return path


def release_metadata(**overrides):
  return {"schema": 1, "deployed_at": "2026-09-09 01:23 KST", "diagnostics_version": 1, **overrides}


@pytest.mark.parametrize("branch", ["dkcarrot-wip", b"dkcarrot-wip", " dkcarrot-wip\n"])
def test_installed_deployment_metadata_supplies_date(branch, tmp_path):
  path = write_metadata(tmp_path, release_metadata())
  assert load_dk_deployment_text(branch, path) == "DK 배포 2026-09-09"


@pytest.mark.parametrize("branch", [None, "", "carrot-wip", "carrot", "origin/dkcarrot-wip", b"\xff", 1])
def test_other_branches_do_not_even_read_metadata(branch):
  class NoReadPath:
    def open(self, *_args):
      pytest.fail("non-DK branch must not read deployment metadata")

  assert load_dk_deployment_text(branch, NoReadPath()) == ""


@pytest.mark.parametrize("metadata", [
  {}, [], None,
  release_metadata(schema=2), release_metadata(schema=True),
  release_metadata(diagnostics_version=0), release_metadata(diagnostics_version=True),
  release_metadata(deployed_at=None), release_metadata(deployed_at="2026-09-09"),
  release_metadata(deployed_at="2026-09-09 01:23 UTC"),
  release_metadata(deployed_at="2026-02-30 01:23 KST"),
  release_metadata(deployed_at="2026-9-9 01:23 KST"),
  release_metadata(deployed_at="2026-09-09 25:23 KST"),
])
def test_invalid_metadata_has_no_fallback_date(tmp_path, metadata):
  assert load_dk_deployment_text("dkcarrot-wip", write_metadata(tmp_path, metadata)) == ""


@pytest.mark.parametrize("raw", [b"{", b"\xff", b" " * (DK_RELEASE_MAX_BYTES + 1), b"[" * 1500 + b"]" * 1500])
def test_unreadable_or_oversized_metadata_is_hidden(tmp_path, raw):
  path = tmp_path / "dk_release.json"
  path.write_bytes(raw)
  assert load_dk_deployment_text("dkcarrot-wip", path) == ""


def test_missing_metadata_is_hidden(tmp_path):
  assert load_dk_deployment_text("dkcarrot-wip", tmp_path / "missing.json") == ""
