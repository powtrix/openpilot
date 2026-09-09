"""Read-only access to local DK diagnostic captures. No upload/start actions."""
import asyncio

from aiohttp import web

from openpilot.selfdrive.carrot.dk_diagnosticsd import ARTIFACT_NAME, CAPTURE_ID, diagnostics_root, read_manifest
from ..services.web_consent import _is_private_web_host, _origin_scope, _request_peer, _request_scope


def _require_local(request):
  scope = _request_scope(request)
  peer = _request_peer(request)
  if scope is None or peer is None or not _is_private_web_host(peer):
    raise web.HTTPForbidden(text="Local network access required")
  origin = request.headers.get("Origin")
  if origin:
    from urllib.parse import urlsplit
    try:
      parsed = urlsplit(origin)
    except ValueError as exc:
      raise web.HTTPForbidden(text="Invalid origin") from exc
    if _origin_scope(parsed.scheme, parsed.netloc) != scope:
      raise web.HTTPForbidden(text="Same-origin access required")


def _list_captures():
  root = diagnostics_root()
  captures = []
  if root.is_dir() and not root.is_symlink():
    for directory in root.iterdir():
      if CAPTURE_ID.fullmatch(directory.name):
        manifest = read_manifest(directory)
        if manifest:
          captures.append({key: manifest[key] for key in ("capture_id", "status", "created_at", "expires_at", "topics", "files", "missing")})
  return sorted(captures, key=lambda item: item["created_at"], reverse=True)


async def list_captures(request):
  _require_local(request)
  return web.json_response({"schema": 1, "local_only": True, "captures": await asyncio.to_thread(_list_captures)},
                           headers={"Cache-Control": "no-store"})


async def download(request):
  _require_local(request)
  capture_id = request.match_info["capture_id"]
  name = request.match_info["name"]
  if not CAPTURE_ID.fullmatch(capture_id) or (name != "manifest.json" and not ARTIFACT_NAME.fullmatch(name)):
    raise web.HTTPNotFound()
  root = diagnostics_root()
  directory = root / capture_id
  manifest = read_manifest(directory)
  if root.is_symlink() or manifest is None:
    raise web.HTTPNotFound()
  allowed = {item.get("name") for item in manifest.get("files", ())}
  if name != "manifest.json" and name not in allowed:
    raise web.HTTPNotFound()
  path = directory / name
  if path.is_symlink() or not path.is_file():
    raise web.HTTPNotFound()
  return web.FileResponse(path, headers={"Cache-Control": "no-store", "X-Content-Type-Options": "nosniff",
                                        "Content-Disposition": f'attachment; filename="{capture_id}-{name}"'})


def register(app):
  app.router.add_get("/api/dk/diagnostics", list_captures)
  app.router.add_get("/api/dk/diagnostics/{capture_id}/{name}", download)
