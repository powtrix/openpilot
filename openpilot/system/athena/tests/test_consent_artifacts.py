from __future__ import annotations

from openpilot.common.external_data import DK_THIRD_PARTY_DATA_SHARING_PARAM
from openpilot.system.athena.consent_artifacts import (
  PREPARED_GENERATION_PARAM,
  artifact_is_blocked,
  consent_session_is_prepared,
  prepare_consent_session,
)


class FakeParams:
  def __init__(self) -> None:
    self._dk_consent_generation = "generation-1"
    self.values = {DK_THIRD_PARTY_DATA_SHARING_PARAM: b"1"}

  def get(self, key, *args, **kwargs):
    del args, kwargs
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value


def test_pre_consent_and_off_created_artifacts_never_become_uploadable(tmp_path):
  params = FakeParams()
  old_file = tmp_path / "old.rlog"
  old_file.write_bytes(b"old")

  first_generation = prepare_consent_session(params, (str(tmp_path),))
  assert first_generation is not None
  assert artifact_is_blocked(str(old_file))
  assert consent_session_is_prepared(params, first_generation)

  current_file = tmp_path / "current.rlog"
  current_file.write_bytes(b"current")
  assert not artifact_is_blocked(str(current_file))

  # This represents a short OFF -> ON cycle that a polling manager did not
  # observe. The Params-file generation changes even though its value is ON at
  # both observations, and every artifact from the old session is excluded.
  params._dk_consent_generation = "generation-2"
  second_generation = prepare_consent_session(params, (str(tmp_path),))

  assert second_generation is not None and second_generation != first_generation
  assert artifact_is_blocked(str(old_file))
  assert artifact_is_blocked(str(current_file))
  assert params.values[PREPARED_GENERATION_PARAM] == second_generation
  assert not consent_session_is_prepared(params, first_generation)
  assert consent_session_is_prepared(params, second_generation)


def test_failed_snapshot_does_not_mark_generation_prepared(monkeypatch, tmp_path):
  params = FakeParams()
  artifact = tmp_path / "artifact"
  artifact.write_bytes(b"private")

  monkeypatch.setattr(
    "openpilot.system.athena.consent_artifacts.setxattr",
    lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("xattr unavailable")),
  )

  try:
    prepare_consent_session(params, (str(tmp_path),))
  except RuntimeError:
    pass
  else:
    raise AssertionError("failed artifact snapshot must fail closed")

  assert PREPARED_GENERATION_PARAM not in params.values
