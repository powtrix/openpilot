import asyncio
from types import SimpleNamespace
from io import BytesIO
import json
import threading
import zipfile

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.dk_diagnosticsd import CaptureStore
from openpilot.selfdrive.carrot.server.features import dk_diagnostics as feature


def request(peer="192.168.50.2", host="192.168.50.95:7000", origin=None, **match):
  headers = {"Host": host}
  if origin:
    headers["Origin"] = origin
  return SimpleNamespace(scheme="http", headers=headers, remote=peer, transport=None, match_info=match)


@pytest.fixture
def captured(tmp_path, monkeypatch):
  logs = tmp_path / "realdata"
  segment = logs / "route--0"
  segment.mkdir(parents=True)
  (segment / "rlog.zst").write_bytes(b"log")
  store = CaptureStore(logs, tmp_path / "diagnostics", min_free_bytes=0)
  cap = store.accept({"event": "dk_vehicle_diag", "schema": 1, "kind": "sample", "topics": ["curve"]}, "route", 1000)
  store.tick(1181)
  store.tick(1186)
  monkeypatch.setattr(feature, "diagnostics_root", lambda: store.root)
  return store, cap


def test_local_only_policy():
  feature._require_local(request())
  for req in (request(peer="8.8.8.8"), request(host="attacker.example"), request(origin="http://attacker.example"),
              request(origin="http://[malformed")):
    with pytest.raises(web.HTTPForbidden):
      feature._require_local(req)


@pytest.mark.asyncio
async def test_read_only_download_and_allowlist(captured):
  store, cap = captured
  assert feature._list_captures()[0]["capture_id"] == cap
  assert isinstance(await feature.download(request(capture_id=cap, name="manifest.json")), web.FileResponse)
  assert isinstance(await feature.download(request(capture_id=cap, name="0-rlog.zst")), web.FileResponse)
  for cid, name in (("..", "manifest.json"), (cap, "../../secret"), (cap, "1-rlog.zst"), (cap, "manifest.json.tmp")):
    with pytest.raises(web.HTTPNotFound):
      await feature.download(request(capture_id=cid, name=name))
  copied = store.root / cap / "0-rlog.zst"
  copied.unlink()
  copied.symlink_to(store.log_root / "route--0" / "rlog.zst")
  with pytest.raises(web.HTTPNotFound):
    await feature.download(request(capture_id=cap, name="0-rlog.zst"))


def test_only_get_routes_are_registered():
  app = web.Application()
  feature.register(app)
  assert {route.method for route in app.router.routes()} == {"GET", "HEAD"}


@pytest.mark.asyncio
async def test_phone_page_and_native_bundle_round_trip(captured, monkeypatch):
  store, cap = captured
  monkeypatch.setattr(feature.shutil, "disk_usage", lambda _path: SimpleNamespace(free=20 * 1024 ** 3))
  app = web.Application()
  feature.register(app)
  async with TestClient(TestServer(app)) as client:
    page = await client.get("/dk-logs")
    assert page.status == 200
    assert "frame-ancestors 'none'" in page.headers["Content-Security-Policy"]
    assert "dk 로그전달" in await page.text()
    assert (await client.get("/dk-logs/assets/dk-log-transfer.js")).status == 200
    assert (await client.get("/dk-logs/assets/secret.txt")).status == 404
    response = await client.get("/api/dk/diagnostics/bundle", params={"capture": cap})
    assert response.status == 200
    assert response.headers["Content-Type"] == "application/zip"
    assert ".dklog.zip" in response.headers["Content-Disposition"]
    with zipfile.ZipFile(BytesIO(await response.read())) as archive:
      index = json.loads(archive.read("dk-transfer.json"))
      assert index["format"] == "dk-log-transfer-v1"
      assert archive.read(f"captures/{cap}/0-rlog.zst") == b"log"
      assert json.loads(archive.read(f"captures/{cap}/manifest.json"))["status"] == "partial"
    assert (store.root / cap / "0-rlog.zst").read_bytes() == b"log"
    assert not list(store.root.parent.glob("dk-transfer-*"))
    assert (await client.head("/api/dk/diagnostics/bundle")).status == 405
    assert (await client.post("/api/dk/diagnostics/bundle")).status == 405


@pytest.mark.asyncio
async def test_bundle_rejects_bad_selection_origin_busy_and_low_space(captured, monkeypatch):
  _, cap = captured
  monkeypatch.setattr(feature.shutil, "disk_usage", lambda _path: SimpleNamespace(free=20 * 1024 ** 3))
  app = web.Application()
  feature.register(app)
  async with TestClient(TestServer(app)) as client:
    for query in ("capture=../", f"capture={cap}&capture={cap}", "unknown=1"):
      assert (await client.get(f"/api/dk/diagnostics/bundle?{query}")).status == 400
    for headers in ({"Origin": "http://attacker.example"}, {"Sec-Fetch-Site": "cross-site"}, {"Host": "attacker.example"}):
      assert (await client.get("/api/dk/diagnostics/bundle", headers=headers)).status == 403
    await app[feature.EXPORT_LOCK].acquire()
    try:
      assert (await client.get("/api/dk/diagnostics/bundle")).status == 409
    finally:
      app[feature.EXPORT_LOCK].release()
    monkeypatch.setattr(feature.shutil, "disk_usage", lambda _path: SimpleNamespace(free=0))
    assert (await client.get("/api/dk/diagnostics/bundle")).status == 507
    assert not app[feature.EXPORT_LOCK].locked()


@pytest.mark.asyncio
async def test_failed_bundle_is_not_downloaded_and_releases_resources(captured, monkeypatch):
  store, _ = captured
  monkeypatch.setattr(feature.shutil, "disk_usage", lambda _path: SimpleNamespace(free=20 * 1024 ** 3))
  def failed(_root, output, _ids):
    output.write(b"incomplete")
    raise feature.TransferError("private detail must not reach response")
  monkeypatch.setattr(feature, "build_bundle", failed)
  app = web.Application()
  feature.register(app)
  async with TestClient(TestServer(app)) as client:
    response = await client.get("/api/dk/diagnostics/bundle")
    assert response.status == 409
    assert "Content-Disposition" not in response.headers
    assert "private detail" not in await response.text()
    assert not app[feature.EXPORT_LOCK].locked()
    assert not list(store.root.parent.glob("dk-transfer-*"))


@pytest.mark.asyncio
@pytest.mark.parametrize("phase", ["build", "read"])
@pytest.mark.parametrize("worker_fails", [False, True])
async def test_repeated_cancel_joins_file_worker_before_close_and_unlock(captured, monkeypatch, phase, worker_fails):
  store, cap = captured
  monkeypatch.setattr(feature.shutil, "disk_usage", lambda _path: SimpleNamespace(free=20 * 1024 ** 3))
  entered = threading.Event()
  release = threading.Event()
  observed = []
  owned = []
  original_temporary_file = feature.tempfile.TemporaryFile
  app = web.Application()
  feature.register(app)

  def blocked(file):
    entered.set()
    if not release.wait(5):
      raise AssertionError("test worker was not released")
    observed.append((file.closed, app[feature.EXPORT_LOCK].locked()))
    if worker_fails:
      raise feature.TransferError("private worker failure after cancellation")

  class OwnedFile:
    def __init__(self, **kwargs):
      self.file = original_temporary_file(**kwargs)
      owned.append(self.file)

    def __enter__(self):
      return self

    def __exit__(self, *_args):
      self.file.close()

    def __getattr__(self, name):
      return getattr(self.file, name)

    def read(self, size):
      if phase == "read":
        blocked(self.file)
      return self.file.read(size)

  class Response:
    def __init__(self, **_kwargs):
      pass

    async def prepare(self, _request):
      pass

    async def write(self, _data):
      raise AssertionError("cancelled download must not publish a chunk")

    async def write_eof(self):
      raise AssertionError("cancelled download must not finish successfully")

  def build(_root, output, _ids):
    if phase == "build":
      blocked(output)
    output.write(b"bounded test archive")
    return {"transfer_id": cap}

  monkeypatch.setattr(feature.tempfile, "TemporaryFile", OwnedFile)
  monkeypatch.setattr(feature.web, "StreamResponse", Response)
  monkeypatch.setattr(feature, "build_bundle", build)
  req = request()
  req.app = app
  # download_bundle also checks the complete query key set.
  class Query(dict):
    def getall(self, _key, default):
      return default
  req.query = Query()
  task = asyncio.create_task(feature.download_bundle(req))
  try:
    assert await asyncio.wait_for(asyncio.to_thread(entered.wait, 2), 3)
    for _ in range(3):
      task.cancel()
      await asyncio.sleep(0)
      assert not task.done()
      assert owned and not owned[0].closed
      assert app[feature.EXPORT_LOCK].locked()
  finally:
    release.set()
  with pytest.raises(asyncio.CancelledError):
    await asyncio.wait_for(task, 3)
  assert observed == [(False, True)]
  assert owned[0].closed
  assert not app[feature.EXPORT_LOCK].locked()
  assert not list(store.root.parent.glob("dk-transfer-*"))
