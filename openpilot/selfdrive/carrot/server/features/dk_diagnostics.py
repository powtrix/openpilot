"""Local read-only captures and manual exports. No uploads or vehicle actions."""
import asyncio
from contextvars import copy_context
import shutil
import tempfile
from pathlib import Path

from aiohttp import web

from openpilot.selfdrive.carrot.dk_diagnosticsd import ARTIFACT_NAME, CAPTURE_ID, diagnostics_root, read_manifest
from openpilot.selfdrive.carrot.dk_log_transfer import MAX_BUNDLE_BYTES, TransferError, build_bundle
from ..config import WEB_DIR
from ..services.web_consent import _is_private_web_host, _origin_scope, _request_peer, _request_scope

EXPORT_LOCK = web.AppKey("dk_log_export_lock", asyncio.Lock)
MIN_EXPORT_FREE = 5 * 1024 ** 3
TRANSFER_HEADERS = {"Cache-Control": "no-store", "X-Content-Type-Options": "nosniff", "Referrer-Policy": "no-referrer"}


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


async def transfer_page(request):
  _require_local(request)
  content = await asyncio.to_thread((Path(WEB_DIR) / "dk-log-transfer.html").read_text, encoding="utf-8")
  return web.Response(text=content, content_type="text/html", headers={**TRANSFER_HEADERS,
    "Content-Security-Policy": ("default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; "
                              + "form-action 'self'; base-uri 'none'; frame-ancestors 'none'")})


async def transfer_asset(request):
  _require_local(request)
  asset = request.match_info["asset"]
  if asset not in ("dk-log-transfer.js", "dk-log-transfer.css"):
    raise web.HTTPNotFound()
  return web.FileResponse(Path(WEB_DIR) / ("js" if asset.endswith(".js") else "css") / asset, headers=TRANSFER_HEADERS)


async def _thread_owned(function, *args):
  """Join file work before propagating even repeated request cancellation.

  Use an executor Future, not a separate Task: shutdown's cancel-all-tasks must
  not cancel a worker wrapper while the underlying thread still owns the file.
  """
  work = asyncio.get_running_loop().run_in_executor(None, copy_context().run, function, *args)
  cancelled = False
  while True:
    try:
      result = await asyncio.shield(work)
    except asyncio.CancelledError:
      cancelled = True
      if work.cancelled():
        raise
    except Exception:
      if cancelled:
        raise asyncio.CancelledError from None
      raise
    else:
      if cancelled:
        raise asyncio.CancelledError
      return result


async def download_bundle(request):
  """Manual, local attachment only. Never upload, remove a capture, or buffer it in phone JS."""
  _require_local(request)
  if request.headers.get("Sec-Fetch-Site") == "cross-site":
    raise web.HTTPForbidden(text="Open dk 로그전달 on this device first.")
  ids = request.query.getall("capture", [])
  if (set(request.query) - {"capture"} or len(ids) > 10 or len(set(ids)) != len(ids)
      or any(not CAPTURE_ID.fullmatch(cid) for cid in ids)):
    raise web.HTTPBadRequest(text="Invalid capture selection.")
  lock = request.app[EXPORT_LOCK]
  if lock.locked():
    raise web.HTTPConflict(text="다른 로그 파일을 준비/전송 중입니다. 완료 후 다시 시도하세요. / Another download is in progress.")
  async with lock:
    root = diagnostics_root()
    if not root.is_dir() or root.is_symlink():
      raise web.HTTPNotFound(text="아직 보관된 DK 진단 로그가 없습니다. / No retained DK captures.")
    # Reserve the maximum bounded archive space without consuming the device's
    # existing 5 GiB log-retention floor. The temporary file is unlinked on close.
    if (await asyncio.to_thread(shutil.disk_usage, root.parent)).free < MAX_BUNDLE_BYTES + MIN_EXPORT_FREE:
      raise web.HTTPInsufficientStorage(text="로그 묶음을 준비할 여유 공간이 부족합니다. / Insufficient space for export.")
    with tempfile.TemporaryFile(prefix="dk-transfer-", dir=root.parent) as output:
      try:
        index = await _thread_owned(build_bundle, root, output, ids or None)
      except (TransferError, OSError) as exc:
        raise web.HTTPConflict(text="로그가 변경됐거나 묶음 검증에 실패했습니다. 새로고침 후 다시 시도하세요. "
                                   + "/ Export unavailable; refresh and retry.") from exc
      size = output.tell()
      if not 0 < size <= MAX_BUNDLE_BYTES:
        raise web.HTTPConflict(text="Export size limit exceeded.")
      output.seek(0)
      response = web.StreamResponse(headers={**TRANSFER_HEADERS, "Content-Type": "application/zip",
        "Content-Length": str(size), "Content-Disposition": f'attachment; filename="dk-logs-{index["transfer_id"]}.dklog.zip"'})
      await response.prepare(request)
      while chunk := await _thread_owned(output.read, 1024 * 1024):
        await response.write(chunk)
      await response.write_eof()
      return response


def register(app):
  app[EXPORT_LOCK] = asyncio.Lock()
  app.router.add_get("/dk-logs", transfer_page)
  app.router.add_get("/dk-logs/assets/{asset}", transfer_asset)
  app.router.add_get("/api/dk/diagnostics", list_captures)
  app.router.add_get("/api/dk/diagnostics/bundle", download_bundle, allow_head=False)
  app.router.add_get("/api/dk/diagnostics/{capture_id}/{name}", download)
