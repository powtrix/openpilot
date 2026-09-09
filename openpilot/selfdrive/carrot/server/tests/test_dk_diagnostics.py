from types import SimpleNamespace

import pytest
from aiohttp import web

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
