import asyncio
import json
import os
import sqlite3
from dataclasses import replace
from pathlib import Path
from typing import Any

from aiohttp import FormData, web
from aiohttp.test_utils import TestClient, TestServer
import pytest

from .. import server as receiver_server
from ..server import UPLOAD_SERVICE_KEY, Config, create_app


DEVICE = "0123456789abcdef"
OTHER_DEVICE = "fedcba9876543210"
CLIENT_IP = "203.0.113.10"
DEVICE_KEY_SHA256 = "1" * 64


def config(tmp_path: Path, *, quota: int = 1024 * 1024) -> Config:
  return Config(
    storage_root=tmp_path / "uploads",
    db_path=tmp_path / "state" / "uploads.sqlite3",
    allowed_device_ids=frozenset({DEVICE}),
    allowed_device_public_key_sha256={DEVICE: DEVICE_KEY_SHA256},
    legacy_uploads_enabled=True,
    daily_device_quota=quota,
    daily_ip_quota=quota * 4,
    max_file_bytes=1024 * 1024,
    max_tmux_bytes=1024 * 1024,
    min_free_bytes=0,
    validation_free_space_reserve_bytes=0,
    session_ttl_seconds=600,
    concurrent_per_device=3,
    concurrent_global=16,
  )


async def session(
  client: TestClient,
  *,
  purpose: str = "dashcam",
  ip: str = CLIENT_IP,
  metadata: dict[str, str] | None = None,
) -> str:
  response = await client.post(
    "/api/v1/session",
    json={"deviceId": DEVICE, "purpose": purpose, "carName": "TEST", **(metadata or {})},
    headers={"X-Forwarded-For": ip},
  )
  assert response.status == 200, await response.text()
  return (await response.json())["token"]


def test_health_session_stream_upload_and_completion(tmp_path: Path):
  async def run():
    async with TestClient(TestServer(create_app(config(tmp_path), start_cleanup=False))) as client:
      health = await client.get("/api/v1/health")
      assert health.status == 200
      health_body = await health.json()
      assert health_body["ok"] is True
      assert health_body["bandwidthLimit"] is None

      token = await session(client)
      content = b"camera-data"
      headers = {
        "Authorization": f"Bearer {token}",
        "X-Forwarded-For": CLIENT_IP,
        "X-File-Size": str(len(content)),
      }
      upload = await client.put(
        f"/api/v1/upload/{DEVICE}/2026-07-20--00-00-00--0/qlog.zst",
        data=content,
        headers=headers,
      )
      assert upload.status == 200, await upload.text()
      assert await upload.json() == {"ok": True, "size": len(content)}
      assert (
        tmp_path / "uploads" / "routes" / f"TEST {DEVICE}" / "2026-07-20--00-00-00--0" / "qlog.zst"
      ).read_bytes() == content

      complete = await client.post(
        "/api/v1/complete",
        json={"deviceId": DEVICE, "ok": True, "meta": {"dongleId": DEVICE}},
        headers={"Authorization": f"Bearer {token}", "X-Forwarded-For": CLIENT_IP},
      )
      assert complete.status == 200
      manifests = list((tmp_path / "state" / "manifests" / DEVICE).glob("*.json"))
      assert len(manifests) == 1

  asyncio.run(run())


def test_security_policy_defaults_fail_closed(tmp_path: Path):
  async def run():
    cfg = Config(
      storage_root=tmp_path / "uploads",
      db_path=tmp_path / "state" / "uploads.sqlite3",
      min_free_bytes=0,
      validation_free_space_reserve_bytes=0,
    )
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      health = await client.get("/api/v1/health")
      assert health.status == 503
      assert await health.json() == {
        "ok": False,
        "service": "dk-upload",
        "deviceAllowlistConfigured": False,
        "deviceKeyPinsConfigured": False,
        "legacyUploadsEnabled": False,
        "storageWritable": True,
        "dailyQuotaBytes": 1024 * 1024 * 1024,
        "maxFileBytes": 512 * 1024 * 1024,
        "bandwidthLimit": None,
      }

      legacy = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert legacy.status == 403
      assert "disabled" in (await legacy.json())["error"]

      challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert challenge.status == 403
      assert "not allowed" in (await challenge.json())["error"]

  asyncio.run(run())


def test_health_fails_closed_when_validation_store_is_not_writable(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(config(tmp_path), legacy_uploads_enabled=False)
    app = create_app(cfg, start_cleanup=False)
    monkeypatch.setattr(app[UPLOAD_SERVICE_KEY], "_validation_storage_writable", lambda: False)
    async with TestClient(TestServer(app)) as client:
      health = await client.get("/api/v1/health")
      body = await health.json()

      assert health.status == 507
      assert body["ok"] is False
      assert body["deviceAllowlistConfigured"] is True
      assert body["deviceKeyPinsConfigured"] is True
      assert body["storageWritable"] is False

  asyncio.run(run())


def test_validation_readiness_and_challenge_fail_closed_without_exact_key_pins(tmp_path: Path):
  async def run():
    for pins in ({}, {DEVICE: DEVICE_KEY_SHA256, OTHER_DEVICE: "2" * 64}):
      cfg = replace(config(tmp_path), allowed_device_public_key_sha256=pins)
      async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
        health = await client.get("/api/v1/health")
        body = await health.json()
        assert health.status == 503
        assert body["ok"] is False
        assert body["deviceAllowlistConfigured"] is True
        assert body["deviceKeyPinsConfigured"] is False
        assert DEVICE_KEY_SHA256 not in json.dumps(body)

        challenge = await client.post(
          "/api/v1/validation/challenge",
          json={"deviceId": DEVICE},
          headers={"X-Forwarded-For": CLIENT_IP},
        )
        if pins:
          assert challenge.status == 200
        else:
          assert challenge.status == 503
          assert "key pin" in (await challenge.json())["error"]

  asyncio.run(run())


def test_allowlist_and_legacy_switch_cover_existing_sessions_and_routes(tmp_path: Path):
  async def run():
    enabled = config(tmp_path)
    async with TestClient(TestServer(create_app(enabled, start_cleanup=False))) as client:
      token = await session(client)

      disallowed_session = await client.post(
        "/api/v1/session",
        json={"deviceId": OTHER_DEVICE, "purpose": "dashcam"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert disallowed_session.status == 403

      disallowed_challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": OTHER_DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert disallowed_challenge.status == 403

    disabled = replace(enabled, legacy_uploads_enabled=False)
    async with TestClient(TestServer(create_app(disabled, start_cleanup=False))) as client:
      health = await client.get("/api/v1/health")
      assert health.status == 200
      assert (await health.json())["legacyUploadsEnabled"] is False

      headers = {
        "Authorization": f"Bearer {token}",
        "X-Forwarded-For": CLIENT_IP,
        "X-File-Size": "1",
      }
      upload = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog", data=b"x", headers=headers,
      )
      complete = await client.post(
        "/api/v1/complete", json={"deviceId": DEVICE}, headers=headers,
      )
      tmux = await client.post("/api/v1/tmux/upload", data=b"ignored", headers=headers)
      new_session = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert [upload.status, complete.status, tmux.status, new_session.status] == [403] * 4
      for response in (upload, complete, tmux, new_session):
        assert "disabled" in (await response.json())["error"]

      allowed_challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert allowed_challenge.status == 200

    revoked = replace(enabled, allowed_device_ids=frozenset({OTHER_DEVICE}))
    async with TestClient(TestServer(create_app(revoked, start_cleanup=False))) as client:
      revoked_upload = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog", data=b"x", headers=headers,
      )
      assert revoked_upload.status == 403
      assert "not allowed" in (await revoked_upload.json())["error"]

  asyncio.run(run())


def test_security_policy_environment_parsing(tmp_path: Path, monkeypatch):
  monkeypatch.setenv("CARROT_UPLOAD_ROOT", str(tmp_path / "uploads"))
  monkeypatch.setenv("CARROT_UPLOAD_DB", str(tmp_path / "state" / "uploads.sqlite3"))
  monkeypatch.setenv("CARROT_ALLOWED_DEVICE_IDS", f"{DEVICE}, {OTHER_DEVICE}")
  monkeypatch.setenv(
    "CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256",
    f"{DEVICE}={'1' * 64}, {OTHER_DEVICE}={'2' * 64}",
  )
  monkeypatch.setenv("CARROT_LEGACY_UPLOADS_ENABLED", "true")
  cfg = Config.from_env()
  assert cfg.allowed_device_ids == frozenset({DEVICE, OTHER_DEVICE})
  assert cfg.allowed_device_public_key_sha256 == {DEVICE: "1" * 64, OTHER_DEVICE: "2" * 64}
  assert cfg.legacy_uploads_enabled is True

  monkeypatch.setenv("CARROT_ALLOWED_DEVICE_IDS", "not/valid")
  with pytest.raises(ValueError, match="invalid device ID"):
    Config.from_env()

  monkeypatch.setenv("CARROT_ALLOWED_DEVICE_IDS", DEVICE)
  monkeypatch.setenv("CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256", f"{DEVICE}={'A' * 64}")
  with pytest.raises(ValueError, match="device key pin"):
    Config.from_env()

  monkeypatch.setenv("CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256", f"{DEVICE}={'1' * 64}")
  monkeypatch.setenv("CARROT_LEGACY_UPLOADS_ENABLED", "sometimes")
  with pytest.raises(ValueError, match="boolean"):
    Config.from_env()


def test_session_is_bound_to_ip_and_device(tmp_path: Path):
  async def run():
    async with TestClient(TestServer(create_app(config(tmp_path), start_cleanup=False))) as client:
      token = await session(client)
      headers = {
        "Authorization": f"Bearer {token}",
        "X-Forwarded-For": "203.0.113.99",
        "X-File-Size": "1",
      }
      wrong_ip = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog", data=b"x", headers=headers,
      )
      assert wrong_ip.status == 403

      headers["X-Forwarded-For"] = CLIENT_IP
      wrong_device = await client.put(
        "/api/v1/upload/fedcba9876543210/route--0/qlog", data=b"x", headers=headers,
      )
      assert wrong_device.status == 403

      test_token = await session(client, purpose="test")
      test_session_upload = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog",
        data=b"x",
        headers={
          "Authorization": f"Bearer {test_token}",
          "X-Forwarded-For": CLIENT_IP,
          "X-File-Size": "1",
        },
      )
      assert test_session_upload.status == 403

      spoofed_session = await session(client, ip=f"198.51.100.77, {CLIENT_IP}")
      real_ip_upload = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog",
        data=b"x",
        headers={
          "Authorization": f"Bearer {spoofed_session}",
          "X-Forwarded-For": CLIENT_IP,
          "X-File-Size": "1",
        },
      )
      assert real_ip_upload.status == 200

  asyncio.run(run())


def test_quota_and_file_validation(tmp_path: Path):
  async def run():
    async with TestClient(TestServer(create_app(config(tmp_path, quota=8), start_cleanup=False))) as client:
      token = await session(client)

      async def upload(filename: str, content: bytes):
        return await client.put(
          f"/api/v1/upload/{DEVICE}/route--0/{filename}",
          data=content,
          headers={
            "Authorization": f"Bearer {token}",
            "X-Forwarded-For": CLIENT_IP,
            "X-File-Size": str(len(content)),
          },
        )

      first = await upload("qlog", b"123456")
      assert first.status == 200
      empty = await upload("empty", b"")
      assert empty.status == 400
      # Replacing an existing remote path still consumes daily transfer quota.
      # Otherwise a client could evade the limit by overwriting one filename.
      over_quota = await upload("qlog", b"7890")
      assert over_quota.status == 413
      blocked = await upload("run.py", b"x")
      assert blocked.status == 400
      assert not list((tmp_path / "uploads").rglob("*.part"))

  asyncio.run(run())


def test_tmux_multipart_upload(tmp_path: Path, monkeypatch):
  async def run():
    async with TestClient(TestServer(create_app(config(tmp_path), start_cleanup=False))) as client:
      token = await session(
        client,
        purpose="tmux",
        metadata={"git_branch": "carrot/wip", "tmux_why": "can_error"},
      )
      form = FormData()
      form.add_field("tmux_why", "can_error")
      form.add_field("files[0]", b"tmux output", filename="tmux.log", content_type="text/plain")
      form.add_field("files[1]", b'{"enabled":true}', filename="toggle_values.json", content_type="application/json")
      service = client.app[UPLOAD_SERVICE_KEY]
      original_progress = service._record_stream_progress
      progress: list[tuple[int, int, int]] = []

      async def record_progress(
        reservation: Any,
        state: Any,
        **kwargs: Any,
      ) -> None:
        progress.append((
          kwargs["received_bytes"],
          kwargs["disk_written_bytes"],
          kwargs.get("stored_bytes", 0),
        ))
        await original_progress(reservation, state, **kwargs)

      monkeypatch.setattr(service, "_record_stream_progress", record_progress)
      response = await client.post(
        "/api/v1/tmux/upload",
        data=form,
        headers={"Authorization": f"Bearer {token}", "X-Forwarded-For": CLIENT_IP},
      )
      assert response.status == 200, await response.text()
      body = await response.json()
      assert body["ok"] is True
      assert body["files"] == 2
      tmux_logs = list(
        (tmp_path / "uploads" / "carrot__wip" / f"TEST {DEVICE}").glob(
          "can_error-*-carrot__wip.txt",
        ),
      )
      assert len(tmux_logs) == 1
      assert tmux_logs[0].read_bytes() == b"tmux output"
      assert not (tmp_path / "uploads" / "tmux" / "carrot__wip").exists()
      assert progress
      assert all(0 <= disk <= received and stored <= disk for received, disk, stored in progress)
      assert any(received > disk for received, disk, _stored in progress)
      for previous, current in zip(progress, progress[1:], strict=False):
        if current[1] > previous[1]:
          assert current[0] == previous[0]

  asyncio.run(run())


def test_received_progress_keeps_free_space_reserved_until_write(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(config(tmp_path), min_free_bytes=10)
    monkeypatch.setattr(
      receiver_server.shutil,
      "disk_usage",
      lambda _path: type("DiskUsage", (), {"free": 15})(),
    )
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      token = await session(client)
      service = client.app[UPLOAD_SERVICE_KEY]
      original_progress = service._record_reservation_progress
      before_write = asyncio.Event()
      release_write = asyncio.Event()
      first_progress: tuple[int, int] | None = None

      async def interleave_progress(
        reservation: Any,
        received_bytes: int,
        stored_bytes: int = 0,
        *,
        disk_written_bytes: int,
      ) -> None:
        nonlocal first_progress
        await original_progress(
          reservation,
          received_bytes,
          stored_bytes,
          disk_written_bytes=disk_written_bytes,
        )
        if first_progress is None and reservation.device_id == DEVICE:
          first_progress = (received_bytes, disk_written_bytes)
          before_write.set()
          await release_write.wait()

      monkeypatch.setattr(service, "_record_reservation_progress", interleave_progress)
      request = asyncio.create_task(client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=b"abcd",
        headers={
          "Authorization": f"Bearer {token}",
          "X-Forwarded-For": CLIENT_IP,
          "X-File-Size": "4",
        },
      ))
      await asyncio.wait_for(before_write.wait(), timeout=1)
      assert first_progress == (4, 0)
      parts = list(cfg.storage_root.rglob("*.part"))
      assert len(parts) == 1
      assert parts[0].stat().st_size == 0
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute(
          "SELECT received_bytes, disk_written_bytes FROM upload_reservations",
        ).fetchone() == (4, 0)

      unexpected = None
      try:
        unexpected = await service._reserve(OTHER_DEVICE, "203.0.113.11", 4)
      except web.HTTPInsufficientStorage:
        pass
      finally:
        if unexpected is not None:
          await service._finish_reservation(unexpected, 0)
        release_write.set()
      assert unexpected is None
      response = await asyncio.wait_for(request, timeout=2)
      assert response.status == 200, await response.text()

  asyncio.run(run())


def test_partial_write_crash_reconciliation_charges_network_only(tmp_path: Path):
  async def run():
    cfg = replace(config(tmp_path), reservation_lease_seconds=1, stale_part_seconds=1)
    service = create_app(cfg, start_cleanup=False)[UPLOAD_SERVICE_KEY]
    reservation = await service._reserve(DEVICE, CLIENT_IP, 4)
    await service._record_reservation_progress(
      reservation,
      4,
      disk_written_bytes=2,
    )
    part = cfg.storage_root / "routes" / "TEST" / ".qlog.aaaaaaaaaaaaaaaa.part"
    part.parent.mkdir(parents=True)
    part.write_bytes(b"ab")
    os.utime(part, (0, 0))
    with sqlite3.connect(cfg.db_path) as connection:
      connection.execute(
        "UPDATE upload_reservations SET touched_at=0 WHERE reservation_id=?",
        (reservation.reservation_id,),
      )

    result = await service.cleanup()
    assert result["staleReservations"] == 1
    assert result["staleParts"] == 1
    assert not part.exists()
    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0
      assert connection.execute(
        "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(4, 0, 0), (4, 0, 0)]

  asyncio.run(run())


def test_stream_timeout_charges_partial_body_and_releases_slot(tmp_path: Path, monkeypatch):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      token = await session(client)
      service = client.app[UPLOAD_SERVICE_KEY]
      original_body_chunks = service._body_chunks
      first_call = True

      async def timeout_first_body(content: Any, chunk_size: int, **kwargs: Any):
        nonlocal first_call
        if first_call:
          first_call = False
          yield b"a"
          raise web.HTTPRequestTimeout(text="upload body idle deadline exceeded")
        async for chunk in original_body_chunks(content, chunk_size, **kwargs):
          yield chunk

      monkeypatch.setattr(service, "_body_chunks", timeout_first_body)
      headers = {
        "Authorization": f"Bearer {token}",
        "X-Forwarded-For": CLIENT_IP,
        "X-File-Size": "2",
      }
      timed_out = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=b"ab",
        headers=headers,
      )
      assert timed_out.status == 408
      assert service._active_global == 0
      assert not list(cfg.storage_root.rglob("*.part"))
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0
        assert connection.execute(
          "SELECT committed_bytes FROM daily_usage ORDER BY scope",
        ).fetchall() == [(1,), (1,)]

      retry = await client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=b"ab",
        headers=headers,
      )
      assert retry.status == 200, await retry.text()

  asyncio.run(run())


def test_tmux_body_deadline_releases_legacy_capacity(tmp_path: Path, monkeypatch):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      token = await session(client, purpose="tmux")
      service = client.app[UPLOAD_SERVICE_KEY]
      original_await_body = service._await_body_operation
      first_call = True

      async def timeout_first_operation(operation: Any, **kwargs: Any):
        nonlocal first_call
        if first_call:
          first_call = False
          raise web.HTTPRequestTimeout(text="upload body idle deadline exceeded")
        return await original_await_body(operation, **kwargs)

      monkeypatch.setattr(service, "_await_body_operation", timeout_first_operation)

      def form() -> FormData:
        value = FormData()
        value.add_field("files[0]", b"tmux output", filename="tmux.log", content_type="text/plain")
        return value

      headers = {"Authorization": f"Bearer {token}", "X-Forwarded-For": CLIENT_IP}
      timed_out = await client.post("/api/v1/tmux/upload", data=form(), headers=headers)
      assert timed_out.status == 408
      assert service._active_global == 0
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0

      retry = await client.post("/api/v1/tmux/upload", data=form(), headers=headers)
      assert retry.status == 200, await retry.text()

  asyncio.run(run())


def test_slow_legacy_json_admission_covers_full_handler_and_isolates_validation(
  tmp_path: Path,
  monkeypatch,
):
  async def run():
    cfg = replace(
      config(tmp_path),
      request_concurrent_per_ip=1,
      request_concurrent_global=1,
      validation_request_concurrent_per_ip=1,
      validation_request_concurrent_global=1,
    )
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      token = await session(client)
      service = client.app[UPLOAD_SERVICE_KEY]
      original_json_body = service._json_body
      body_started = asyncio.Event()
      release_body = asyncio.Event()
      session_body_calls = 0

      async def blocking_json_body(request: web.Request, limit: int, **kwargs: Any) -> Any:
        nonlocal session_body_calls
        if request.path == "/api/v1/session":
          session_body_calls += 1
          if session_body_calls == 1:
            body_started.set()
            await release_body.wait()
        return await original_json_body(request, limit, **kwargs)

      monkeypatch.setattr(service, "_json_body", blocking_json_body)
      slow_session = asyncio.create_task(client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam"},
        headers={"X-Forwarded-For": CLIENT_IP},
      ))
      await asyncio.wait_for(body_started.wait(), timeout=1)

      blocked_complete = await client.post(
        "/api/v1/complete",
        json={"deviceId": DEVICE},
        headers={"Authorization": f"Bearer {token}", "X-Forwarded-For": CLIENT_IP},
      )
      assert blocked_complete.status in {429, 503}
      assert session_body_calls == 1
      assert service._request_active_global == 1

      validation = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert validation.status == 200, await validation.text()

      release_body.set()
      completed_session = await asyncio.wait_for(slow_session, timeout=2)
      assert completed_session.status == 200, await completed_session.text()
      assert service._request_active_global == 0
      assert service._validation_request_active_global == 0
      assert not service._request_active_by_ip
      assert not service._validation_request_active_by_ip

  asyncio.run(run())


def test_cleanup_never_deletes_existing_openpilot_files(tmp_path: Path):
  async def run():
    existing = tmp_path / "uploads" / "carrot-wip" / "README.md"
    existing.parent.mkdir(parents=True)
    existing.write_text("keep", encoding="utf-8")
    service_app = create_app(config(tmp_path), start_cleanup=False)
    await service_app[UPLOAD_SERVICE_KEY].cleanup()
    assert existing.read_text(encoding="utf-8") == "keep"

  asyncio.run(run())
