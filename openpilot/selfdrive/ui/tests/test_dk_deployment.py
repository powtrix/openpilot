import json

import pytest

from openpilot.selfdrive.ui.dk_deployment import DK_RELEASE_LABEL_MAX_CHARS, DK_RELEASE_MAX_BYTES, load_dk_deployment_text


def write_metadata(tmp_path, metadata):
  path = tmp_path / "dk_release.json"
  path.write_text(json.dumps(metadata), encoding="utf-8")
  return path


def release_metadata(**overrides):
  return {"schema": 1, "deployed_at": "2026-09-12 01:23 KST", "diagnostics_version": 1, **overrides}


@pytest.mark.parametrize("branch", ["dkcarrot-wip", b"dkcarrot-wip", " dkcarrot-wip\n"])
def test_installed_deployment_metadata_supplies_date(branch, tmp_path):
  path = write_metadata(tmp_path, release_metadata())
  assert load_dk_deployment_text(branch, path) == "0912 개선"


@pytest.mark.parametrize("deployed_at, expected", [
  ("2026-01-02 03:04 KST", "0102 개선"),
  ("2028-11-23 12:34 KST", "1123 개선"),
])
def test_label_uses_installed_release_month_day_not_the_live_date(tmp_path, deployed_at, expected):
  path = write_metadata(tmp_path, release_metadata(deployed_at=deployed_at))
  assert load_dk_deployment_text("dkcarrot-wip", path) == expected


@pytest.mark.parametrize("label", ["감속", "로그 개선", "Brake-v2", "브레이크게이지/핸들복원로그", "가" * DK_RELEASE_LABEL_MAX_CHARS])
def test_release_label_is_configurable_and_date_prefix_tracks_metadata(tmp_path, label):
  path = write_metadata(tmp_path, release_metadata(deployed_at="2027-12-31 23:59 KST", label=label))
  assert load_dk_deployment_text("dkcarrot-wip", path) == f"1231 {label}"


def test_installed_release_displays_braking_and_turn_return_log_label():
  assert load_dk_deployment_text("dkcarrot-wip") == "0917 브레이크게이지/핸들복원로그"


@pytest.mark.parametrize("label", [
  None, False, 1, [], {}, "", " ", " 감속", "감속 ", "가" * (DK_RELEASE_LABEL_MAX_CHARS + 1),
  "감\n속", "감\r속", "감\t속", "감\x00속", "감\x7f속", "감\x85속", "감\u200b속",
  "감\u200e속", "감\u200f속", "감\u202a속", "감\u202e속", "감\u2066속", "감\u2069속",
  "감\u2028속", "감\u2029속", "감\ud800속",
])
def test_invalid_explicit_label_is_hidden_instead_of_using_legacy_fallback(tmp_path, label):
  assert load_dk_deployment_text("dkcarrot-wip", write_metadata(tmp_path, release_metadata(label=label))) == ""


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
