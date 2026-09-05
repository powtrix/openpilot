import asyncio
import hashlib
import json
from pathlib import Path

import pytest
from aiohttp import web

from openpilot.selfdrive.carrot import web_upload
from openpilot.selfdrive.carrot.server.features.dashcam import catalog
from openpilot.selfdrive.carrot.server.features.dashcam import upload
from openpilot.selfdrive.carrot.server.features.dashcam import upload_jobs
from openpilot.selfdrive.carrot.server.services import dashcam_upload_report
from openpilot.selfdrive.carrot.server.services import web_settings


def clear_upload_env(monkeypatch):
  for key in ("CARROT_WEB_UPLOAD_URL", "CARROT_WEB_UPLOAD_TOKEN", "CARROT_TMUX_WEB_UPLOAD_URL"):
    monkeypatch.delenv(key, raising=False)


class FakeUploadTask:
  def __init__(self, *, done=False):
    self._done = done
    self.cancel_called = False

  def done(self):
    return self._done

  def cancel(self):
    self.cancel_called = True


def test_dashcam_stale_upload_job_is_failed_and_released():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])
  task = FakeUploadTask()
  job["_task"] = task
  job["_activity_at"] = 100.0

  upload_jobs.expire_stale_jobs(now=100.0 + upload_jobs.UPLOAD_JOB_STALE_SECONDS)

  assert task.cancel_called is True
  assert job["status"] == "failed"
  assert job["error"] == "upload job expired after 30 minutes without activity"
  assert upload_jobs.running_job() is None
  upload_jobs.jobs().clear()


def test_dashcam_finished_task_cannot_leave_running_job():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])
  job["_task"] = FakeUploadTask(done=True)

  upload_jobs.expire_stale_jobs(now=job["_activity_at"])

  assert job["status"] == "failed"
  assert job["error"] == "upload task ended without a final state"
  upload_jobs.jobs().clear()


def test_dashcam_upload_job_exposes_stable_phase_codes():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])

  assert upload_jobs.snapshot(job)["phase"] == "queued"

  upload_jobs.progress(job, current=1, total=1, phase=upload_jobs.UPLOAD_PHASE_UPLOADING)
  assert upload_jobs.snapshot(job)["phase"] == "uploading"

  upload_jobs.finish(job, ok=True, result={"ok": True})
  snapshot = upload_jobs.snapshot(job)
  assert snapshot["status"] == "done"
  assert snapshot["phase"] == "complete"
  upload_jobs.jobs().clear()


def test_dashcam_upload_progress_is_monotonic_and_revisioned():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0", "route--1"])
  assert upload_jobs.snapshot(job)["revision"] == 0

  upload_jobs.progress(
    job,
    phase=upload_jobs.UPLOAD_PHASE_PREPARING,
    current=1,
    total=2,
    phase_current=1,
    phase_total=2,
    percent=4,
  )
  preparing = upload_jobs.snapshot(job)
  assert preparing["progress"] == 4
  assert preparing["phase_current"] == 1
  assert preparing["phase_total"] == 2
  assert preparing["revision"] == 1

  upload_jobs.progress(
    job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=2,
    bytes_current=128,
    bytes_total=1024,
    bytes_per_second=512,
  )
  uploading = upload_jobs.snapshot(job)
  assert uploading["progress"] == 4
  assert uploading["bytes_current"] == 128
  assert uploading["bytes_total"] == 1024
  assert uploading["bytes_per_second"] == 512
  assert uploading["revision"] == 2

  upload_jobs.finish(job, ok=True, result={"ok": True})
  complete = upload_jobs.snapshot(job)
  assert complete["progress"] == 100
  assert complete["revision"] == 3
  upload_jobs.jobs().clear()


def test_dashcam_upload_cancel_and_failure_keep_visible_progress():
  upload_jobs.jobs().clear()

  canceled_job = upload_jobs.create_job(["route--0"])
  upload_jobs.progress(
    canceled_job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=41,
  )
  canceled = upload_jobs.cancel_job(canceled_job["id"])
  assert canceled["phase"] == "canceling"
  assert canceled["progress"] == 41
  upload_jobs.finish(
    canceled_job,
    ok=False,
    status="canceled",
    result={"ok": False, "canceled": True},
  )
  canceled = upload_jobs.snapshot(canceled_job)
  assert canceled["phase"] == "canceled"
  assert canceled["progress"] == 41

  failed_job = upload_jobs.create_job(["route--1"])
  upload_jobs.progress(
    failed_job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=62,
  )
  upload_jobs.finish(
    failed_job,
    ok=False,
    error="network failed",
    result={"ok": False, "error": "network failed"},
  )
  failed = upload_jobs.snapshot(failed_job)
  assert failed["phase"] == "failed"
  assert failed["progress"] == 62
  upload_jobs.jobs().clear()


def test_dashcam_start_job_finalizes_unhandled_task_failure(monkeypatch):
  upload_jobs.jobs().clear()

  async def scenario():
    async def crash(_job):
      raise RuntimeError("unexpected task failure")

    monkeypatch.setattr(upload_jobs, "run_job", crash)
    job = upload_jobs.create_job(["route--0"])
    task = upload_jobs.start_job(job)
    with pytest.raises(RuntimeError, match="unexpected task failure"):
      await task
    await asyncio.sleep(0)
    return job

  job = asyncio.run(scenario())
  assert job["status"] == "failed"
  assert job["error"] == "unexpected task failure"
  upload_jobs.jobs().clear()


def test_carrot_runtime_contains_no_legacy_ftp_code():
  carrot_root = Path(__file__).resolve().parents[2]
  legacy_terms = (
    "ft" + "plib",
    "carrot_" + "ftp",
    "ftp" + "://",
    "ftp" + "_ok",
    "upload_folder_to_" + "ftp",
  )
  findings = []
  for path in carrot_root.rglob("*"):
    if not path.is_file() or path.suffix.lower() not in {".py", ".js", ".sh"}:
      continue
    if "tests" in path.parts or "generated" in path.parts or "vendor" in path.parts:
      continue
    text = path.read_text(encoding="utf-8", errors="ignore").lower()
    if any(term in text for term in legacy_terms):
      findings.append(str(path.relative_to(carrot_root)))
  assert findings == []


def test_carrot_man_sends_diagnostics_to_dsm_and_carrot_logs():
  carrot_man = (Path(__file__).resolve().parents[2] / "carrot_man.py").read_text(encoding="utf-8")
  assert "def send_tmux_web(" in carrot_man
  assert "def send_tmux_carrot_logs(" in carrot_man
  assert 'self.send_tmux_carrot_logs("onroad", send_settings = True)' in carrot_man
  assert "self.send_tmux_carrot_logs(pending_tmux_reason, send_settings = False)" in carrot_man
  assert 'self.send_tmux_carrot_logs("tmux_send")' in carrot_man
  assert "using tmux web fallback" not in carrot_man


def test_web_upload_settings_support_legacy_values_and_environment_override(monkeypatch):
  clear_upload_env(monkeypatch)
  assert web_upload.web_upload_settings({
    "toss_upload_url": "https://legacy.example/",
    "toss_upload_token": "legacy-token",
  }) == ("https://legacy.example", "")

  monkeypatch.setenv("CARROT_WEB_UPLOAD_URL", "https://env.example/root/")
  monkeypatch.setenv("CARROT_WEB_UPLOAD_TOKEN", "env-token")
  assert web_upload.web_upload_settings({
    "web_upload_url": "https://setting.example",
    "web_upload_token": "setting-token",
  }) == ("https://env.example/root", "env-token")


def test_web_api_url_quotes_every_path_component():
  assert web_upload.api_url(
    "https://upload.example/",
    "upload",
    "car name/id",
    "route|0",
    "qlog.zst",
  ) == "https://upload.example/api/v1/upload/car%20name%2Fid/route%7C0/qlog.zst"


def test_dashcam_upload_report_links_public_segment_and_quotes_storage_directory():
  payload = {
    "uploadedAt": "2026-07-23 11:17:18",
    "remoteBasePath": "https://upload.example/routes/HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3/",
    "meta": {
      "carName": "HYUNDAI_IONIQ_5_PE",
      "dongleId": "8b06424f3adf2bd3",
      "commit": "79a2a542",
    },
    "results": [{
      "segment": "00000cfb--69588de3d7--10",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": 10,
      "ok": True,
      "remotePath": "https://upload.example/routes/HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3/00000cfb--69588de3d7--10",
    }],
  }

  report = dashcam_upload_report.upload_share_text(payload)
  assert "HYUNDAI_IONIQ_5_PE%208b06424f3adf2bd3" in report
  assert "[00000cfb--69588de3d7--10 OK · Open](https://upload.example/routes/" in report
  assert "### Open & Analyze" not in report


def test_dashcam_upload_report_adds_one_slice_link_for_consecutive_segments():
  base = "https://upload.example/routes/TEST CAR 0123456789abcdef"
  results = [
    {
      "segment": f"00000cfb--69588de3d7--{index}",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": index,
      "ok": True,
      "remotePath": f"{base}/00000cfb--69588de3d7--{index}",
    }
    for index in (10, 11, 12)
  ]

  report = dashcam_upload_report.upload_share_text({"remoteBasePath": f"{base}/", "results": results})
  assert "### Open & Analyze" in report
  assert "Segments 10–12 (3 logs) · Web/Video/Tools" in report
  assert "https://upload.example/routes/TEST%20CAR%200123456789abcdef/00000cfb--69588de3d7--10:13" in report
  assert report.count(" OK · Open]") == 3


def test_dashcam_upload_report_does_not_merge_nonconsecutive_segments():
  base = "https://upload.example/routes/TEST CAR 0123456789abcdef"
  results = [
    {
      "segment": f"00000cfb--69588de3d7--{index}",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": index,
      "ok": True,
      "remotePath": f"{base}/00000cfb--69588de3d7--{index}",
    }
    for index in (10, 12)
  ]

  report = dashcam_upload_report.upload_share_text({"remoteBasePath": f"{base}/", "results": results})
  assert "### Open & Analyze" not in report


def test_dashcam_upload_completion_notifies_web_server_and_discord(monkeypatch):
  segment = "00000cfb--69588de3d7--10"
  notifications = []
  uploaded_files = []
  progress_snapshots = []

  async def fake_upload_folder(*args, **kwargs):
    uploaded_files.extend(kwargs["filenames"])
    on_progress = kwargs.get("on_progress")
    if on_progress:
      on_progress("qcamera.ts", 12, 12, 12)
      on_progress("rlog.zst", 34, 34, 34)
    return True

  async def fake_web_complete(base_url, token, payload):
    notifications.append(("web", base_url, token, payload["results"][0]["segment"]))
    return {"ok": True, "status": 200}

  async def fake_discord(webhook_url, payload):
    notifications.append(("discord", webhook_url, payload["shareText"]))
    return {"configured": True, "ok": True, "status": 204}

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "session-token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda params: {
    "carName": "TEST_CAR",
    "dongleId": "0123456789abcdef",
  })
  monkeypatch.setattr(upload, "upload_share_text", lambda payload: "shared upload report")
  monkeypatch.setattr(upload, "discord_webhook_url", lambda params: "https://discord.example/webhook")
  monkeypatch.setattr(upload, "send_discord_webhook", fake_discord)
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda value: "/tmp/segment")
  monkeypatch.setattr(upload_jobs, "segment_file_summary", lambda value: [
    {"kind": "qcamera", "name": "qcamera.ts", "size": 12},
    {"kind": "rlog", "name": "rlog.zst", "size": 34},
  ])
  monkeypatch.setattr(upload_jobs, "upload_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_web_upload_complete", fake_web_complete)
  real_progress = upload_jobs.progress

  def capture_progress(job, **kwargs):
    real_progress(job, **kwargs)
    progress_snapshots.append(upload_jobs.snapshot(job))

  monkeypatch.setattr(upload_jobs, "progress", capture_progress)

  upload_jobs.jobs().clear()
  job = upload_jobs.create_job([segment])
  result = asyncio.run(upload_jobs.run_upload_segments([segment], job))

  assert result["ok"] is True
  assert result["webComplete"] == {"ok": True, "status": 200}
  assert result["discord"] == {"configured": True, "ok": True, "status": 204}
  assert notifications == [
    ("web", "https://upload.example", "session-token", segment),
    ("discord", "https://discord.example/webhook", "shared upload report"),
  ]
  assert uploaded_files == ["qcamera.ts", "rlog.zst"]
  snapshot = upload_jobs.snapshot(job)
  assert snapshot["bytes_current"] == 46
  assert snapshot["bytes_total"] == 46
  assert snapshot["bytes_per_second"] >= 0
  assert snapshot["step_current"] == 1
  assert snapshot["step_total"] == 1
  assert snapshot["progress"] == 99
  assert snapshot["phase"] == "notifying"
  assert snapshot["phase_current"] == 2
  assert snapshot["phase_total"] == 2
  assert [item["progress"] for item in progress_snapshots] == sorted(
    item["progress"] for item in progress_snapshots
  )
  assert {"preparing", "uploading", "notifying"} <= {
    item["phase"] for item in progress_snapshots
  }
  assert any(
    item["phase"] == "preparing" and 0 < item["progress"] <= upload_jobs.UPLOAD_PREPARING_END_PERCENT
    for item in progress_snapshots
  )
  upload_jobs.jobs().clear()


def test_validation_upload_serializes_segments_even_with_parallel_override(monkeypatch):
  segments = [f"00000cfb--69588de3d7--{index}" for index in range(3)]
  manifest = [
    {"segment": segment, "name": "rlog.zst", "size": 8, "sha256": "a" * 64}
    for segment in segments
  ]
  active = 0
  peak = 0
  issued_sessions = []
  upload_tokens = []

  def safety_check():
    return True

  def fake_summary(_path, *, artifact_kinds=None):
    assert set(artifact_kinds or ()) == {"rlog"}
    return [{"kind": "rlog", "name": "rlog.zst", "size": 8}]

  async def fake_validation_session(_base_url, metadata, *, should_continue=None):
    assert metadata["dongleId"] == "device"
    assert should_continue is safety_check
    token = f"validation-session-{len(issued_sessions) + 1}"
    issued_sessions.append(token)
    return token

  async def fake_upload_folder(
    _path, segment, capture_id, _base_url, token, expected_files, should_cancel, **kwargs,
  ):
    nonlocal active, peak
    assert capture_id == "capture"
    upload_tokens.append(token)
    assert expected_files == manifest
    assert any(item["segment"] == segment for item in expected_files)
    assert should_cancel() is False
    active += 1
    peak = max(peak, active)
    await asyncio.sleep(0)
    active -= 1
    return True

  async def fake_complete(_base_url, token, payload, *, should_continue=None):
    # The completion session must be issued after all segment sessions. This
    # models transfers that outlive every earlier short-lived bearer token.
    assert token == issued_sessions[-1]
    assert token not in upload_tokens
    assert should_continue is safety_check
    return {"status": 200, **durable_validation_receipt(payload)}

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda _params: {"carName": "KA4", "dongleId": "device"})
  monkeypatch.setattr(upload, "upload_share_text", lambda _payload: "unused")
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda segment: f"/tmp/{segment}")
  monkeypatch.setattr(upload_jobs, "segment_file_summary", fake_summary)
  monkeypatch.setattr(upload_jobs, "create_validation_upload_session", fake_validation_session)
  monkeypatch.setattr(upload_jobs, "upload_validation_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_validation_upload_complete", fake_complete)
  monkeypatch.setattr(
    upload,
    "discord_webhook_url",
    lambda _params: pytest.fail("Discord configuration must not be read for validation uploads"),
  )

  result = asyncio.run(upload_jobs.run_upload_segments(
    segments,
    artifact_kinds={"rlog"},
    notify_discord=False,
    concurrency_override=3,
    completion_metadata={"captureId": "capture"},
    safety_check=safety_check,
    validation_capture_id="capture",
    validation_files=manifest,
  ))

  assert result["ok"] is True
  assert result["uploaded"] == 3
  assert peak == 1
  assert result["discord"]["skipped"] is True
  assert upload_tokens == issued_sessions[:-1]
  assert len(issued_sessions) == len(segments) + 1


def test_validation_upload_refreshes_rejected_file_and_completion_sessions(monkeypatch):
  segment = "00000cfb--69588de3d7--0"
  manifest = [{"segment": segment, "name": "rlog.zst", "size": 8, "sha256": "a" * 64}]
  issued_sessions = []
  upload_tokens = []
  completion_tokens = []

  async def fake_validation_session(_base_url, _metadata, *, should_continue=None):
    token = f"validation-session-{len(issued_sessions) + 1}"
    issued_sessions.append(token)
    return token

  async def fake_upload_folder(
    _path, _segment, _capture_id, _base_url, token, _expected_files, _should_cancel, **_kwargs,
  ):
    upload_tokens.append(token)
    if len(upload_tokens) == 1:
      raise RuntimeError("rlog.zst: validation upload HTTP 401: expired session")
    return True

  async def fake_complete(_base_url, token, payload, *, should_continue=None):
    completion_tokens.append(token)
    if len(completion_tokens) == 1:
      return {"ok": False, "status": 401, "error": "expired session"}
    return {"status": 200, **durable_validation_receipt(payload)}

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda _params: {"carName": "KA4", "dongleId": "device"})
  monkeypatch.setattr(upload, "upload_share_text", lambda _payload: "unused")
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda _segment: "/tmp/segment")
  monkeypatch.setattr(
    upload_jobs,
    "segment_file_summary",
    lambda _path, **_kwargs: [{"kind": "rlog", "name": "rlog.zst", "size": 8}],
  )
  monkeypatch.setattr(upload_jobs, "create_validation_upload_session", fake_validation_session)
  monkeypatch.setattr(upload_jobs, "upload_validation_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_validation_upload_complete", fake_complete)

  result = asyncio.run(upload_jobs.run_upload_segments(
    [segment],
    artifact_kinds={"rlog"},
    notify_discord=False,
    concurrency_override=3,
    completion_metadata={"captureId": "capture"},
    validation_capture_id="capture",
    validation_files=manifest,
  ))

  assert result["ok"] is True
  assert upload_tokens == ["validation-session-1", "validation-session-2"]
  assert completion_tokens == ["validation-session-3", "validation-session-4"]
  assert issued_sessions == [
    "validation-session-1", "validation-session-2", "validation-session-3", "validation-session-4",
  ]


def test_validation_upload_rechecks_consent_bound_destination_before_network(monkeypatch):
  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://changed.example", "token"))
  monkeypatch.setattr(
    upload,
    "upload_metadata",
    lambda _params: pytest.fail("destination mismatch must fail before collecting or uploading metadata"),
  )

  with pytest.raises(RuntimeError, match="destination changed after consent"):
    asyncio.run(upload_jobs.run_upload_segments(
      ["00000cfb--69588de3d7--0"],
      artifact_kinds={"rlog"},
      notify_discord=False,
      concurrency_override=1,
      base_url_override="https://consented.example",
    ))


def test_validation_upload_honors_cancellation_after_completion_request(monkeypatch):
  segment = "00000cfb--69588de3d7--0"
  manifest = [{"segment": segment, "name": "rlog.zst", "size": 8, "sha256": "a" * 64}]
  discord_calls = 0

  def fake_summary(_path, *, artifact_kinds=None):
    assert set(artifact_kinds or ()) == {"rlog"}
    return [{"kind": "rlog", "name": "rlog.zst", "size": 8}]

  async def fake_validation_session(_base_url, _metadata, *, should_continue=None):
    assert should_continue is None
    return "validation-session"

  async def fake_upload_folder(*_args, **_kwargs):
    return True

  async def fake_complete(_base_url, _token, payload, *, should_continue=None):
    assert should_continue is None
    upload_jobs.cancel_job(job["id"])
    return {"status": 200, **durable_validation_receipt(payload)}

  async def fail_discord(*_args, **_kwargs):
    nonlocal discord_calls
    discord_calls += 1
    return {"ok": True}

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda _params: {"carName": "KA4", "dongleId": "device"})
  monkeypatch.setattr(upload, "upload_share_text", lambda _payload: "unused")
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda _segment: "/tmp/segment")
  monkeypatch.setattr(upload_jobs, "segment_file_summary", fake_summary)
  monkeypatch.setattr(upload_jobs, "create_validation_upload_session", fake_validation_session)
  monkeypatch.setattr(upload_jobs, "upload_validation_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_validation_upload_complete", fake_complete)
  monkeypatch.setattr(upload, "send_discord_webhook", fail_discord)

  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(
    [segment],
    action="validation_auto_upload",
    run_options={
      "artifact_kinds": {"rlog"},
      "notify_discord": False,
      "validation_capture_id": "capture",
      "validation_files": manifest,
    },
  )
  asyncio.run(upload_jobs.run_job(job))

  assert job["status"] == "canceled"
  assert job["result"]["canceled"] is True
  assert discord_calls == 0
  upload_jobs.jobs().clear()


def test_tmux_target_uses_automatic_session_token(monkeypatch):
  clear_upload_env(monkeypatch)
  url, headers = web_upload.tmux_web_target({
    "web_upload_url": "https://upload.example",
  }, "automatic-session")
  assert url == "https://upload.example/api/v1/tmux/upload"
  assert headers == {"Authorization": "Bearer automatic-session"}


def test_tmux_target_falls_back_to_direct_web_endpoint_without_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  assert web_upload.tmux_web_target({}) == ("https://tmux.example/upload", {})


def test_carrot_logs_target_is_independent_from_dsm_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_WEB_UPLOAD_TOKEN", "dsm-token")
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  assert web_upload.carrot_logs_web_target() == ("https://tmux.example/upload", {})


def test_sync_session_is_issued_automatically_from_device_metadata():
  captured = {}

  class Response:
    status_code = 200
    text = '{"ok":true}'

    @staticmethod
    def json():
      return {"ok": True, "token": "short-lived-session"}

  def fake_post(url, *, json, timeout):
    captured.update({"url": url, "json": json, "timeout": timeout})
    return Response()

  token = web_upload.create_web_upload_session_sync(
    "https://upload.example",
    {"dongle_id": "0123456789abcdef", "car_name": "TEST"},
    fake_post,
  )
  assert token == "short-lived-session"
  assert captured == {
    "url": "https://upload.example/api/v1/session",
    "json": {
      "dongle_id": "0123456789abcdef",
      "car_name": "TEST",
      "deviceId": "0123456789abcdef",
      "purpose": "tmux",
    },
    "timeout": 12,
  }


def test_async_session_is_issued_automatically(monkeypatch):
  captured = {}

  class Response:
    status = 200

    async def text(self):
      return '{"ok":true,"token":"dashcam-session"}'

  class Context:
    async def __aenter__(self):
      return Response()

    async def __aexit__(self, exc_type, exc, tb):
      return False

  class Session:
    def __init__(self, *args, **kwargs):
      pass

    async def __aenter__(self):
      return self

    async def __aexit__(self, exc_type, exc, tb):
      return False

    def post(self, url, *, json):
      captured.update({"url": url, "json": json})
      return Context()

  monkeypatch.setattr(web_upload, "ClientSession", Session)
  token = asyncio.run(web_upload.create_web_upload_session(
    "https://upload.example", {"dongleId": "0123456789abcdef"}, "dashcam",
  ))
  assert token == "dashcam-session"
  assert captured["json"]["deviceId"] == "0123456789abcdef"
  assert captured["json"]["purpose"] == "dashcam"


def test_tmux_web_post_sends_multipart_and_closes_files(tmp_path: Path):
  tmux_path = tmp_path / "tmux.log"
  settings_path = tmp_path / "toggle_values.json"
  tmux_path.write_bytes(b"tmux-data")
  settings_path.write_bytes(b'{"enabled": true}')
  captured = {}

  def fake_post(url, *, headers, data, files, timeout):
    captured.update({"url": url, "headers": headers, "data": data, "files": files, "timeout": timeout})
    captured["contents"] = [item[1][1].read() for item in files]
    return "response"

  response = web_upload.post_tmux_web(
    "https://upload.example/api/v1/tmux/upload",
    {"Authorization": "Bearer token"},
    {"tmux_why": "exception"},
    str(tmux_path),
    str(settings_path),
    fake_post,
  )

  assert response == "response"
  assert captured["headers"] == {"Authorization": "Bearer token"}
  assert captured["data"] == {"tmux_why": "exception"}
  assert [item[0] for item in captured["files"]] == ["files[0]", "files[1]"]
  assert captured["contents"] == [b"tmux-data", b'{"enabled": true}']
  assert captured["timeout"] == 30
  assert all(item[1][1].closed for item in captured["files"])


def test_web_settings_migrate_previous_upload_keys():
  settings = web_settings.sanitize_web_settings({
    "toss_upload_url": "https://legacy.example/",
    "toss_upload_token": "legacy-token",
  })
  assert settings["web_upload_url"] == "https://legacy.example"
  assert "web_upload_token" not in settings
  assert "toss_upload_url" not in settings
  assert "toss_upload_token" not in settings


@pytest.mark.parametrize("previous_url", [
  "https://op.wjcloud.kr",
  "https://shind0.synology.me",
  "https://SHIND0.synology.me",
])
def test_web_settings_migrate_previous_default_server(previous_url):
  settings = web_settings.sanitize_web_settings({"web_upload_url": previous_url})
  assert settings["web_upload_url"] == web_upload.DEFAULT_WEB_UPLOAD_URL


class FakeResponse:
  def __init__(self, status: int, payload: dict):
    self.status = status
    self._payload = payload

  async def text(self):
    return json.dumps(self._payload)


class FakeCompleteResponse:
  def __init__(self, status: int, text: str):
    self.status = status
    self._text = text

  async def text(self):
    return self._text


class FakeCompleteRequestContext:
  def __init__(self, response):
    self.response = response

  async def __aenter__(self):
    return self.response

  async def __aexit__(self, exc_type, exc, tb):
    return False


class FakeCompleteSession:
  response = FakeCompleteResponse(200, '{"ok":true}')
  requests = []

  def __init__(self, *args, **kwargs):
    pass

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    return False

  def post(self, url, *, json, headers):
    type(self).requests.append((url, json, headers))
    return FakeCompleteRequestContext(type(self).response)


def test_web_upload_complete_requires_explicit_positive_ack(monkeypatch):
  FakeCompleteSession.response = FakeCompleteResponse(
    200,
    '{"ok":false,"error":"manifest rejected","receiptId":"rejected-123"}',
  )
  FakeCompleteSession.requests = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeCompleteSession)

  result = asyncio.run(web_upload.send_web_upload_complete(
    "https://upload.example", "token", {"results": []},
  ))

  assert result == {
    "ok": False,
    "error": "manifest rejected",
    "receiptId": "rejected-123",
    "status": 200,
  }


def test_web_upload_complete_preserves_success_receipt_fields(monkeypatch):
  FakeCompleteSession.response = FakeCompleteResponse(
    201,
    '{"ok":true,"receiptId":"receipt-123","storedSegments":2}',
  )
  FakeCompleteSession.requests = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeCompleteSession)

  result = asyncio.run(web_upload.send_web_upload_complete(
    "https://upload.example", "token", {"results": [{"segment": "route--0"}]},
  ))

  assert result == {
    "ok": True,
    "receiptId": "receipt-123",
    "storedSegments": 2,
    "status": 201,
  }
  assert FakeCompleteSession.requests == [(
    "https://upload.example/api/v1/complete",
    {"results": [{"segment": "route--0"}]},
    {"Authorization": "Bearer token"},
  )]


@pytest.mark.parametrize(("status", "body"), [
  (500, '{"ok":true,"receiptId":"server-error"}'),
  (204, ""),
])
def test_web_upload_complete_rejects_transport_or_json_without_ack(monkeypatch, status, body):
  FakeCompleteSession.response = FakeCompleteResponse(status, body)
  FakeCompleteSession.requests = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeCompleteSession)

  result = asyncio.run(web_upload.send_web_upload_complete(
    "https://upload.example", "token", {"results": []},
  ))

  assert result["ok"] is False
  assert result["status"] == status


class FakeRequestContext:
  def __init__(self, session, url, data, headers):
    self.session = session
    self.url = url
    self.data = data
    self.headers = headers

  async def __aenter__(self):
    content = bytearray()
    async for chunk in self.data:
      content.extend(chunk)
    self.session.requests.append((self.url, bytes(content), self.headers))
    size_delta = self.session.size_deltas.pop(0) if self.session.size_deltas else 0
    return FakeResponse(200, {"ok": True, "size": len(content) + size_delta})

  async def __aexit__(self, exc_type, exc, tb):
    return False


class FakeSession:
  instances = []
  size_deltas = []

  def __init__(self, *args, headers=None, **kwargs):
    self.headers = headers or {}
    self.requests = []
    self.size_deltas = list(type(self).size_deltas)
    type(self).instances.append(self)

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    return False

  def put(self, url, data, headers=None):
    return FakeRequestContext(self, url, data, headers or {})


def test_dashcam_web_upload_streams_and_verifies_every_file(tmp_path: Path, monkeypatch):
  (tmp_path / "fcamera.hevc").write_bytes(b"camera-data")
  (tmp_path / "qlog.zst").write_bytes(b"log-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "car name dongle/id",
    "2026-07-20--00-00-00|0",
    "https://upload.example",
    "token",
  ))

  session = FakeSession.instances[-1]
  assert session.headers["Authorization"] == "Bearer token"
  assert [request[0] for request in session.requests] == [
    "https://upload.example/api/v1/upload/car%20name%20dongle%2Fid/2026-07-20--00-00-00%7C0/fcamera.hevc",
    "https://upload.example/api/v1/upload/car%20name%20dongle%2Fid/2026-07-20--00-00-00%7C0/qlog.zst",
  ]
  assert [request[1] for request in session.requests] == [b"camera-data", b"log-data"]
  assert [request[2]["X-File-Size"] for request in session.requests] == ["11", "8"]


def test_dashcam_web_upload_reports_chunk_progress(tmp_path: Path, monkeypatch):
  (tmp_path / "qcamera.ts").write_bytes(b"camera-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)
  updates = []

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
    on_progress=lambda *args: updates.append(args),
  ))

  assert updates == [
    ("qcamera.ts", 0, 11, 0),
    ("qcamera.ts", 11, 11, 11),
  ]


def test_dashcam_upload_summary_selects_only_original_qcamera_and_rlog(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"original-video")
  (tmp_path / "qcamera.mp4").write_bytes(b"converted-video")
  (tmp_path / "rlog.zst").write_bytes(b"original-rlog")
  (tmp_path / "rlog.bz2").write_bytes(b"fallback-rlog")
  (tmp_path / "qlog.zst").write_bytes(b"reduced-log")
  (tmp_path / "fcamera.hevc").write_bytes(b"auxiliary-video")

  files = catalog.segment_file_summary(str(tmp_path))

  assert [(item["kind"], item["name"], item["size"]) for item in files] == [
    ("qcamera", "qcamera.ts", len(b"original-video")),
    ("rlog", "rlog.zst", len(b"original-rlog")),
  ]


def test_dashcam_upload_summary_allows_rlog_without_qcamera(tmp_path: Path):
  (tmp_path / "rlog.bz2").write_bytes(b"original-rlog")

  files = catalog.segment_file_summary(str(tmp_path))

  assert [(item["kind"], item["name"]) for item in files] == [("rlog", "rlog.bz2")]


def test_dashcam_upload_summary_can_select_full_rlog_only(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"road-video")
  (tmp_path / "rlog.zst").write_bytes(b"full-log")

  files = catalog.segment_file_summary(str(tmp_path), artifact_kinds={"rlog"})

  assert [(item["kind"], item["name"]) for item in files] == [("rlog", "rlog.zst")]


def test_dashcam_upload_summary_rejects_selection_without_rlog(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"road-video")
  (tmp_path / "rlog.zst").write_bytes(b"full-log")

  with pytest.raises(ValueError, match="unsupported upload artifact selection"):
    catalog.segment_file_summary(str(tmp_path), artifact_kinds={"qcamera"})


def test_dashcam_upload_summary_requires_rlog(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"original-video")

  with pytest.raises(web.HTTPNotFound) as exc_info:
    catalog.segment_file_summary(str(tmp_path))
  assert exc_info.value.text == "rlog not found"


def test_dashcam_web_upload_honors_explicit_file_selection(tmp_path: Path, monkeypatch):
  (tmp_path / "qcamera.ts").write_bytes(b"camera-data")
  (tmp_path / "rlog.zst").write_bytes(b"log-data")
  (tmp_path / "qlog.zst").write_bytes(b"excluded-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
    filenames=("qcamera.ts", "rlog.zst"),
  ))

  session = FakeSession.instances[-1]
  assert [request[0].rsplit("/", 1)[-1] for request in session.requests] == [
    "qcamera.ts",
    "rlog.zst",
  ]


def test_dashcam_web_upload_retries_size_mismatch(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"retry-me")
  FakeSession.instances = []
  FakeSession.size_deltas = [1, 0]
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
  ))
  assert len(FakeSession.instances[-1].requests) == 2


def test_dashcam_web_upload_requires_session_before_network(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"data")
  FakeSession.instances = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  with pytest.raises(RuntimeError, match="session is not configured"):
    asyncio.run(web_upload.upload_folder_to_web(
      str(tmp_path),
      "device",
      "route|0",
      "https://upload.example",
      "",
    ))
  assert FakeSession.instances == []


def test_dashcam_web_upload_honors_cancellation_before_network(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"data")
  FakeSession.instances = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  with pytest.raises(RuntimeError, match="upload canceled"):
    asyncio.run(web_upload.upload_folder_to_web(
      str(tmp_path),
      "device",
      "route|0",
      "https://upload.example",
      "token",
      lambda: True,
    ))
  assert FakeSession.instances == []


def validation_challenge(**overrides):
  return {
    "ok": True,
    "challengeId": "challenge-123",
    "nonce": "nonce-456",
    "audience": "carrot-validation",
    "deviceAuthVersion": 1,
    "receiptVersion": 1,
    **overrides,
  }


def validation_session(**overrides):
  return {
    "ok": True,
    "token": "validation-session-token",
    "verifiedDeviceId": "device-123",
    "deviceAuthVersion": 1,
    "receiptVersion": 1,
    **overrides,
  }


class ScriptedJsonContext:
  def __init__(self, response):
    self.response = response

  async def __aenter__(self):
    return self.response

  async def __aexit__(self, exc_type, exc, tb):
    return False


class ScriptedJsonSession:
  responses = []
  requests = []
  timeouts = []

  def __init__(self, *args, **kwargs):
    type(self).timeouts.append(kwargs.get("timeout"))

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    return False

  def post(self, url, *, json, headers=None, allow_redirects=True):
    type(self).requests.append({
      "url": url,
      "json": json,
      "headers": headers,
      "allow_redirects": allow_redirects,
    })
    return ScriptedJsonContext(type(self).responses.pop(0))


class RecordingDeviceApi:
  calls = []

  def __init__(self, device_id):
    self.device_id = device_id

  def get_token(self, **kwargs):
    type(self).calls.append({"deviceId": self.device_id, **kwargs})
    return "signed-device-identity"


def scripted_json_responses(*payloads):
  ScriptedJsonSession.requests = []
  ScriptedJsonSession.timeouts = []
  ScriptedJsonSession.responses = [
    FakeCompleteResponse(status, json.dumps(body))
    for status, body in payloads
  ]


def test_validation_session_challenge_and_short_lived_device_jwt_contract(monkeypatch):
  scripted_json_responses(
    (200, validation_challenge()),
    (201, validation_session()),
  )
  RecordingDeviceApi.calls = []
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)
  monkeypatch.setattr(web_upload, "Api", RecordingDeviceApi)

  token = asyncio.run(web_upload.create_validation_upload_session(
    "https://upload.example",
    {"dongleId": "device-123", "carName": "KA4", "branch": "carrot-wip"},
  ))

  assert token == "validation-session-token"
  assert ScriptedJsonSession.requests == [
    {
      "url": "https://upload.example/api/v1/validation/challenge",
      "json": {"deviceId": "device-123"},
      "headers": None,
      "allow_redirects": False,
    },
    {
      "url": "https://upload.example/api/v1/validation/session",
      "json": {
        "dongleId": "device-123",
        "carName": "KA4",
        "branch": "carrot-wip",
        "deviceId": "device-123",
        "purpose": "validation",
        "challengeId": "challenge-123",
        "identityToken": "signed-device-identity",
      },
      "headers": None,
      "allow_redirects": False,
    },
  ]
  assert RecordingDeviceApi.calls == [{
    "deviceId": "device-123",
    "payload_extra": {
      "carrotUploadChallenge": "challenge-123",
      "carrotUploadNonce": "nonce-456",
      "carrotUploadPurpose": "validation",
      "carrotUploadAudience": "carrot-validation",
    },
    "expiry_hours": pytest.approx(web_upload.VALIDATION_IDENTITY_TOKEN_TTL_SECONDS / 3600),
  }]


@pytest.mark.parametrize(("challenge", "session"), [
  (validation_challenge(deviceAuthVersion=2), None),
  (validation_challenge(receiptVersion=2), None),
  (validation_challenge(), validation_session(deviceAuthVersion=2)),
  (validation_challenge(), validation_session(receiptVersion=2)),
  (validation_challenge(), validation_session(verifiedDeviceId="different-device")),
])
def test_validation_session_rejects_unknown_protocol_capability_versions(monkeypatch, challenge, session):
  payloads = [(200, challenge)]
  if session is not None:
    payloads.append((200, session))
  scripted_json_responses(*payloads)
  RecordingDeviceApi.calls = []
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)
  monkeypatch.setattr(web_upload, "Api", RecordingDeviceApi)

  with pytest.raises(RuntimeError, match=r"validation (challenge|session) HTTP 200"):
    asyncio.run(web_upload.create_validation_upload_session(
      "https://upload.example", {"dongleId": "device-123"},
    ))


def test_validation_session_rechecks_live_safety_before_second_request(monkeypatch):
  scripted_json_responses(
    (200, validation_challenge()),
    (201, validation_session()),
  )
  RecordingDeviceApi.calls = []
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)
  monkeypatch.setattr(web_upload, "Api", RecordingDeviceApi)
  checks = 0

  def safety_check():
    nonlocal checks
    checks += 1
    return checks < 3

  with pytest.raises(RuntimeError, match="safety policy changed"):
    asyncio.run(web_upload.create_validation_upload_session(
      "https://upload.example",
      {"dongleId": "device-123"},
      should_continue=safety_check,
    ))

  assert len(ScriptedJsonSession.requests) == 1
  assert ScriptedJsonSession.requests[0]["url"].endswith("/validation/challenge")


class ValidationPutContext:
  def __init__(self, session, url, data, headers, allow_redirects):
    self.session = session
    self.url = url
    self.data = data
    self.headers = headers
    self.allow_redirects = allow_redirects

  async def __aenter__(self):
    content = bytearray()
    async for chunk in self.data:
      content.extend(chunk)
    self.session.requests.append({
      "url": self.url,
      "content": bytes(content),
      "headers": self.headers,
      "allow_redirects": self.allow_redirects,
    })
    status, body = self.session.responses.pop(0)
    return FakeCompleteResponse(status, json.dumps(body))

  async def __aexit__(self, exc_type, exc, tb):
    return False


class ValidationPutSession:
  instances = []
  response_script = []

  def __init__(self, *args, headers=None, **kwargs):
    self.default_headers = headers or {}
    self.requests = []
    self.responses = list(type(self).response_script)
    type(self).instances.append(self)

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    return False

  def put(self, url, *, data, headers, allow_redirects=True):
    return ValidationPutContext(self, url, data, headers, allow_redirects)


def validation_file_manifest(segment, data):
  return [{
    "segment": segment,
    "name": "rlog.zst",
    "size": len(data),
    "sha256": hashlib.sha256(data).hexdigest(),
  }]


def test_validation_file_upload_sends_and_verifies_size_and_sha256(tmp_path, monkeypatch):
  segment = "2026-09-05--12-34-56--0"
  data = b"immutable-validation-rlog"
  digest = hashlib.sha256(data).hexdigest()
  (tmp_path / "rlog.zst").write_bytes(data)
  ValidationPutSession.instances = []
  ValidationPutSession.response_script = [(200, {
    "ok": True,
    "size": len(data),
    "sha256": digest,
  })]
  monkeypatch.setattr(web_upload, "ClientSession", ValidationPutSession)

  assert asyncio.run(web_upload.upload_validation_folder_to_web(
    str(tmp_path),
    segment,
    "capture-123",
    "https://upload.example",
    "validation-token",
    validation_file_manifest(segment, data),
  ))

  session = ValidationPutSession.instances[-1]
  assert session.default_headers == {
    "Authorization": "Bearer validation-token",
    "Content-Type": "application/octet-stream",
  }
  assert session.requests == [{
    "url": "https://upload.example/api/v1/validation/upload/capture-123/2026-09-05--12-34-56--0/rlog.zst",
    "content": data,
    "headers": {
      "X-File-Size": str(len(data)),
      "X-Content-SHA256": digest,
    },
    "allow_redirects": False,
  }]


@pytest.mark.parametrize("response_override", [
  {"size": 1},
  {"sha256": "0" * 64},
])
def test_validation_file_upload_rejects_mismatched_server_receipt(
  tmp_path, monkeypatch, response_override,
):
  segment = "2026-09-05--12-34-56--0"
  data = b"validation-rlog"
  digest = hashlib.sha256(data).hexdigest()
  (tmp_path / "rlog.zst").write_bytes(data)
  response = {"ok": True, "size": len(data), "sha256": digest, **response_override}
  ValidationPutSession.instances = []
  ValidationPutSession.response_script = [(200, response), (200, response)]
  monkeypatch.setattr(web_upload, "ClientSession", ValidationPutSession)

  with pytest.raises(RuntimeError, match="validation upload HTTP 200"):
    asyncio.run(web_upload.upload_validation_folder_to_web(
      str(tmp_path),
      segment,
      "capture-123",
      "https://upload.example",
      "validation-token",
      validation_file_manifest(segment, data),
    ))

  assert len(ValidationPutSession.instances[-1].requests) == 2


def test_validation_file_upload_checks_live_safety_between_chunks(tmp_path, monkeypatch):
  segment = "2026-09-05--12-34-56--0"
  data = b"x" * (2 * 1024 * 1024)
  (tmp_path / "rlog.zst").write_bytes(data)
  ValidationPutSession.instances = []
  ValidationPutSession.response_script = []
  monkeypatch.setattr(web_upload, "ClientSession", ValidationPutSession)
  checks = 0

  def should_cancel():
    nonlocal checks
    checks += 1
    return checks >= 4

  with pytest.raises(RuntimeError, match="upload canceled"):
    asyncio.run(web_upload.upload_validation_folder_to_web(
      str(tmp_path),
      segment,
      "capture-123",
      "https://upload.example",
      "validation-token",
      validation_file_manifest(segment, data),
      should_cancel,
    ))

  assert checks >= 4
  assert ValidationPutSession.instances[-1].requests == []


def durable_validation_receipt(payload, **overrides):
  manifest_sha256 = web_upload.validation_manifest_sha256(
    payload["deviceId"],
    payload["captureId"],
    payload.get("files") or [],
    payload.get("validationCapture"),
  )
  return {
    "ok": True,
    "receiptVersion": 1,
    "receiptId": web_upload.validation_receipt_id(manifest_sha256),
    "manifestSha256": manifest_sha256,
    "deviceId": payload["deviceId"],
    "verifiedDeviceId": payload["deviceId"],
    "captureId": payload["captureId"],
    "files": list(payload.get("files") or []),
    **overrides,
  }


def test_validation_completion_requires_and_preserves_durable_receipt(monkeypatch):
  payload = {
    "deviceId": "device-123",
    "captureId": "capture-123",
    "files": [],
    "validationCapture": {"condition": "standstill_off"},
    "shareText": "must stay local",
    "meta": {"private": "must stay local"},
  }
  receipt = durable_validation_receipt(payload)
  scripted_json_responses((201, receipt))
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)

  result = asyncio.run(web_upload.send_validation_upload_complete(
    "https://upload.example",
    "validation-token",
    payload,
  ))

  assert result == {**receipt, "status": 201}
  assert ScriptedJsonSession.requests == [{
    "url": "https://upload.example/api/v1/validation/complete",
    "json": {
      "deviceId": "device-123",
      "captureId": "capture-123",
      "files": [],
      "validationCapture": {"condition": "standstill_off"},
    },
    "headers": {"Authorization": "Bearer validation-token"},
    "allow_redirects": False,
  }]
  timeout = ScriptedJsonSession.timeouts[-1]
  assert timeout.total is None
  assert timeout.connect == web_upload.VALIDATION_COMPLETION_CONNECT_TIMEOUT_SECONDS
  assert timeout.sock_read == web_upload.VALIDATION_COMPLETION_READ_TIMEOUT_SECONDS
  assert timeout.sock_read >= 15 * 60


def test_validation_completion_checks_live_safety_before_request(monkeypatch):
  payload = {"deviceId": "device-123", "captureId": "capture-123", "files": []}
  scripted_json_responses((201, durable_validation_receipt(payload)))
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)

  result = asyncio.run(web_upload.send_validation_upload_complete(
    "https://upload.example",
    "validation-token",
    payload,
    should_continue=lambda: False,
  ))

  assert result["ok"] is False
  assert "safety policy changed" in result["error"]
  assert ScriptedJsonSession.requests == []


@pytest.mark.parametrize("missing_field", [
  "receiptVersion",
  "receiptId",
  "manifestSha256",
  "deviceId",
  "verifiedDeviceId",
  "captureId",
])
def test_validation_completion_rejects_incomplete_durable_receipt(monkeypatch, missing_field):
  payload = {"deviceId": "device-123", "captureId": "capture-123", "files": []}
  receipt = durable_validation_receipt(payload)
  receipt.pop(missing_field)
  scripted_json_responses((200, receipt))
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)

  result = asyncio.run(web_upload.send_validation_upload_complete(
    "https://upload.example", "validation-token", payload,
  ))

  assert result["ok"] is False
  assert result["status"] == 200
  assert "error" in result


@pytest.mark.parametrize(("field", "replacement"), [
  ("receiptId", "0" * 64),
  ("receiptId", "A" * 64),
  ("receiptId", "short"),
  ("manifestSha256", "0" * 64),
  ("deviceId", "different-device"),
  ("verifiedDeviceId", "different-device"),
  ("captureId", "different-capture"),
])
def test_validation_completion_rejects_mismatched_receipt_identity(
  monkeypatch, field, replacement,
):
  payload = {"deviceId": "device-123", "captureId": "capture-123", "files": []}
  scripted_json_responses((200, durable_validation_receipt(payload, **{field: replacement})))
  monkeypatch.setattr(web_upload, "ClientSession", ScriptedJsonSession)

  result = asyncio.run(web_upload.send_validation_upload_complete(
    "https://upload.example", "validation-token", payload,
  ))

  assert result["ok"] is False
  assert result["status"] == 200


def test_validation_receipt_id_uses_protocol_v1_domain_separator():
  manifest_sha256 = "a" * 64

  assert web_upload.validation_receipt_id(manifest_sha256) == hashlib.sha256(
    b"carrot-validation-receipt-v1\0" + manifest_sha256.encode("ascii"),
  ).hexdigest()
  assert web_upload.validation_receipt_id(manifest_sha256.upper()) == ""
  assert web_upload.validation_receipt_id("a" * 63) == ""


@pytest.mark.parametrize(("capture_id", "files"), [
  (None, None),
  ("capture-123", None),
  (None, []),
])
def test_validation_auto_upload_never_falls_back_to_legacy_upload(
  monkeypatch, capture_id, files,
):
  async def fail_legacy(*args, **kwargs):
    pytest.fail("validation auto upload must not use the legacy upload protocol")

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "legacy-token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda _params: {"carName": "KA4", "dongleId": "device-123"})
  monkeypatch.setattr(upload_jobs, "create_web_upload_session", fail_legacy)
  monkeypatch.setattr(upload_jobs, "upload_folder_to_web", fail_legacy)

  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["2026-09-05--12-34-56--0"], action="validation_auto_upload")
  try:
    with pytest.raises(RuntimeError, match="requires authenticated receipt mode"):
      asyncio.run(upload_jobs.run_upload_segments(
        job["segments"],
        job,
        artifact_kinds={"rlog"},
        notify_discord=False,
        validation_capture_id=capture_id,
        validation_files=files,
      ))
  finally:
    upload_jobs.jobs().clear()
