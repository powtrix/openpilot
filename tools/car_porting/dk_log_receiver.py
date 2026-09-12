#!/usr/bin/env python3
"""Manually receive a phone-carried DK log bundle on a trusted home LAN.

Run only when needed; there is no background service, public endpoint, or upload
to any third party. A fresh secret is printed in the URL fragment on each run.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import secrets
import shutil
import sys
import tempfile
import threading
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
  sys.path.insert(0, str(ROOT))

DEFAULT_OUTPUT = ROOT / "local_comma3x" / "dk-phone-inbox"
CHUNK_BYTES = 1024 * 1024
FREE_RESERVE_BYTES = 256 * 1024 * 1024
IDLE_TIMEOUT_SECONDS = 20
UPLOAD_TIMEOUT_SECONDS = 15 * 60
MAX_CONNECTIONS = 8
PRIVATE_NETWORKS = tuple(ipaddress.ip_network(value) for value in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "127.0.0.0/8"))

STYLE = """
:root{color-scheme:light dark;font-family:system-ui,sans-serif}body{max-width:42rem;margin:2rem auto;padding:0 1rem;line-height:1.6}
h1{font-size:1.6rem}button,input{font:inherit;margin:.5rem 0;max-width:100%}button{padding:.6rem 1rem}
progress{display:block;width:100%;height:1.2rem}#status{white-space:pre-wrap;overflow-wrap:anywhere}.note{opacity:.8;font-size:.95rem}
"""
SCRIPT = """
'use strict';
(() => {
  const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
  history.replaceState(null, '', location.pathname);
  const fileInput = document.getElementById('bundle');
  const button = document.getElementById('send');
  const progress = document.getElementById('progress');
  const status = document.getElementById('status');
  const invitation = document.getElementById('invitation');
  let busy = false;
  const tokenValid = /^[A-Za-z0-9_-]{43}$/.test(token);
  if (tokenValid) {
    document.getElementById('share').hidden = false;
    invitation.value = location.origin + '/#token=' + token;
    invitation.addEventListener('click', () => invitation.select());
  }
  const setStatus = (message) => { status.textContent = message; };
  const refresh = () => { button.disabled = busy || !tokenValid || !fileInput.files.length; fileInput.disabled = busy; };
  if (!tokenValid) setStatus('수신기를 시작한 컴퓨터에 표시된 전체 링크로 접속하세요. 새로고침했다면 링크를 다시 여세요.');
  fileInput.addEventListener('change', refresh);
  button.addEventListener('click', () => {
    const file = fileInput.files[0];
    if (busy || !tokenValid || !file) return;
    if (!file.name.endsWith('.dklog.zip')) { setStatus('웹당근에서 받은 .dklog.zip 파일을 선택하세요.'); return; }
    if (!file.size || file.size > Number(document.body.dataset.maxBytes)) { setStatus('파일이 비어 있거나 허용 용량을 넘었습니다.'); return; }
    busy = true; refresh(); progress.value = 0;
    setStatus('휴대폰 → 이 컴퓨터로 전송 중입니다. 이 화면을 열어 두세요.');
    const xhr = new XMLHttpRequest();
    xhr.open('POST', '/api/import');
    xhr.timeout = 16 * 60 * 1000;
    xhr.setRequestHeader('Content-Type', 'application/octet-stream');
    xhr.setRequestHeader('X-DK-Transfer-Token', token);
    xhr.upload.addEventListener('progress', (event) => {
      if (event.lengthComputable) progress.value = event.loaded / event.total * 100;
      if (event.lengthComputable && event.loaded === event.total) setStatus('전송 완료. 원본 크기와 SHA-256을 검증하고 보관하는 중입니다.');
    });
    const failed = () => setStatus('전송을 완료하지 못했습니다. 수신기가 켜져 있는지 확인하고 같은 파일을 다시 보내세요. 휴대폰 원본은 유지됩니다.');
    xhr.addEventListener('error', failed);
    xhr.addEventListener('timeout', failed);
    xhr.addEventListener('abort', failed);
    xhr.addEventListener('load', () => {
      let result;
      try { result = JSON.parse(xhr.responseText); } catch (_) { failed(); return; }
      if (xhr.status !== 200 || !result.ok) { setStatus(result.error || '파일 검증 또는 보관에 실패했습니다.'); return; }
      const receipt = result.receipt;
      progress.value = 100;
      setStatus((receipt.duplicate ? '이미 검증·보관된 동일한 묶음입니다.' : '검증 및 보관 완료.') +
        '\\n전달 ID: ' + receipt.transfer_id + '\\n캡처: ' + receipt.capture_count + '개 / 불완전 캡처: ' + receipt.partial_count +
        '개\\n보관 위치: ' + receipt.destination + '\\n불완전 캡처가 있으면 원인 판정 전에 누락 파일을 확인해야 합니다.');
    });
    xhr.addEventListener('loadend', () => { busy = false; refresh(); });
    xhr.send(file);
  });
  refresh();
})();
"""
PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>dk 로그 전달</title><style>__STYLE__</style></head><body data-max-bytes="__MAX_BYTES__">
<h1>dk 로그 전달</h1><p>휴대폰에 받은 로그 파일을 집의 이 컴퓨터로 보관합니다. 두 기기가 같은 신뢰할 수 있는 Wi-Fi에 있어야 합니다.</p>
<div id="share" hidden><label for="invitation">휴대폰에서 열 링크 (접속 키 포함, 다른 사람에게 공유하지 마세요)</label>
<input id="invitation" type="text" readonly size="64" autocomplete="off" spellcheck="false"></div>
<ol><li>차량에서 웹당근으로 받은 <b>.dklog.zip</b> 파일을 선택하세요.</li><li>아래 버튼을 누르고 검증·보관 완료가 나올 때까지 이 화면을 열어 두세요.</li></ol>
<label for="bundle">휴대폰 다운로드 폴더의 로그 묶음</label><br><input id="bundle" type="file" accept=".zip,.dklog.zip,application/zip">
<br><button id="send" type="button" disabled>이 컴퓨터로 전달</button><progress id="progress" max="100" value="0"></progress>
<p id="status" role="status" aria-live="polite">파일을 선택하세요.</p>
<p class="note">공식 당근 서버나 외부 서비스로 보내지 않습니다. 차량·휴대폰 원본을 삭제하지 않습니다.
이 방식은 백그라운드 전송이나 끊긴 위치부터 재개를 지원하지 않으며, 끊기면 같은 파일을 다시 보내면 됩니다.
HTTP는 암호화되지 않으므로 공용 Wi-Fi에서 사용하거나 인터넷에 공개하지 마세요. 완료 후 컴퓨터에서 Ctrl+C로 수신기를 종료하세요.</p>
<script>__SCRIPT__</script></body></html>""".replace("__STYLE__", STYLE).replace("__SCRIPT__", SCRIPT)


def _hash_source(value: str) -> str:
  return "'sha256-" + base64.b64encode(hashlib.sha256(value.encode()).digest()).decode() + "'"


CSP = ("default-src 'none'; script-src " + _hash_source(SCRIPT) + "; style-src " + _hash_source(STYLE) +
       "; connect-src 'self'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'; object-src 'none'")


def is_local_address(value: str) -> bool:
  try:
    address = ipaddress.IPv4Address(value)
    return any(address in network for network in PRIVATE_NETWORKS)
  except ipaddress.AddressValueError:
    return False


def validate_bind(value: str) -> str:
  value = "127.0.0.1" if value == "localhost" else value
  if not is_local_address(value):
    raise ValueError("--bind must be an explicit private IPv4 address or localhost; wildcard/public addresses are not allowed")
  return value


class UploadError(Exception):
  def __init__(self, status: int, message: str):
    self.status = status
    self.message = message


class ReceiverServer(ThreadingHTTPServer):
  """Ephemeral LAN-only server; authenticated writes, no stored-log browse API."""
  daemon_threads = False
  block_on_close = True
  request_queue_size = MAX_CONNECTIONS

  def __init__(self, bind: str, port: int, output: Path, *, importer=None, max_bundle_bytes=None):
    bind = validate_bind(bind)
    if not 0 <= port <= 65535:
      raise ValueError("port out of range")
    # Lazy import also lets the HTTP boundary be tested without vehicle libraries.
    if importer is None or max_bundle_bytes is None:
      from openpilot.selfdrive.carrot.dk_log_transfer import MAX_BUNDLE_BYTES, import_bundle
      importer = importer or import_bundle
      max_bundle_bytes = MAX_BUNDLE_BYTES if max_bundle_bytes is None else max_bundle_bytes
    self.importer = importer
    self.max_bundle_bytes = max_bundle_bytes
    self.output = Path(output).expanduser().resolve()
    self.output.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not self.output.is_dir():
      raise ValueError("output must be a directory")
    self.token = secrets.token_urlsafe(32)
    self.upload_lock = threading.Lock()
    self.request_slots = threading.BoundedSemaphore(MAX_CONNECTIONS)
    self.stopping = threading.Event()
    self.idle_timeout = IDLE_TIMEOUT_SECONDS
    self.upload_timeout = UPLOAD_TIMEOUT_SECONDS
    super().__init__((bind, port), ReceiverHandler)
    host, bound_port = self.server_address
    self.authorities = {f"{host}:{bound_port}"}
    if host == "127.0.0.1":
      self.authorities.add(f"localhost:{bound_port}")
    self.base_url = f"http://{host}:{bound_port}"

  @property
  def invitation_url(self) -> str:
    return self.base_url + "/#token=" + self.token

  def verify_request(self, request, client_address):
    return not self.stopping.is_set() and is_local_address(client_address[0])

  def process_request(self, request, client_address):
    if not self.request_slots.acquire(blocking=False):
      self.shutdown_request(request)
      return
    try:
      super().process_request(request, client_address)
    except BaseException:
      self.request_slots.release()
      raise

  def process_request_thread(self, request, client_address):
    try:
      super().process_request_thread(request, client_address)
    finally:
      self.request_slots.release()

  def handle_error(self, request, client_address):
    # Never dump request headers, tokens, archive paths or imported data to stderr.
    pass

  def server_close(self):
    self.stopping.set()
    super().server_close()


class ReceiverHandler(BaseHTTPRequestHandler):
  server: ReceiverServer
  server_version = "dk-log-receiver"
  sys_version = ""
  protocol_version = "HTTP/1.1"

  def setup(self):
    self.request.settimeout(self.server.idle_timeout)
    super().setup()

  def log_message(self, message_format, *args):
    pass

  def _reply(self, status: int, data: bytes, content_type: str):
    self.close_connection = True
    self.send_response(status)
    for key, value in (
      ("Content-Type", content_type), ("Content-Length", str(len(data))), ("Cache-Control", "no-store"),
      ("Referrer-Policy", "no-referrer"), ("X-Content-Type-Options", "nosniff"),
      ("X-Frame-Options", "DENY"), ("Content-Security-Policy", CSP), ("Connection", "close"),
    ):
      self.send_header(key, value)
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(data)

  def _json(self, status: int, **data):
    self._reply(status, json.dumps(data, ensure_ascii=False, allow_nan=False).encode(), "application/json; charset=utf-8")

  def send_error(self, code, message=None, explain=None):
    # BaseHTTPRequestHandler's default error page may quote a malformed request.
    self._json(code, ok=False, error="요청을 처리할 수 없습니다.")

  def _single_header(self, name: str) -> str | None:
    values = self.headers.get_all(name, [])
    return values[0] if len(values) == 1 else None

  def _local_request(self) -> bool:
    host = self._single_header("Host")
    if not is_local_address(self.client_address[0]) or host not in self.server.authorities:
      self._json(403, ok=False, error="허용되지 않은 로컬 주소입니다.")
      return False
    return True

  def do_GET(self):
    if not self._local_request():
      return
    if self.path != "/":
      self._json(404, ok=False, error="해당 페이지가 없습니다.")
      return
    data = PAGE.replace("__MAX_BYTES__", str(self.server.max_bundle_bytes)).encode()
    self._reply(200, data, "text/html; charset=utf-8")

  def do_HEAD(self):
    self.do_GET()

  def handle_expect_100(self):
    # The supported File/XHR client does not need Expect; avoid a body preflight
    # before the same-origin, token and bounded-length checks have passed.
    self._json(417, ok=False, error="Expect 요청은 지원하지 않습니다.")
    return False

  def do_POST(self):
    if not self._local_request():
      return
    if self.path != "/api/import":
      self._json(404, ok=False, error="해당 기능이 없습니다.")
      return
    origin = self._single_header("Origin")
    if origin != "http://" + self._single_header("Host"):
      self._json(403, ok=False, error="수신기 페이지에서 파일을 선택해 전달하세요.")
      return
    token = self._single_header("X-DK-Transfer-Token") or ""
    if not token.isascii() or not secrets.compare_digest(token, self.server.token):
      self._json(403, ok=False, error="접속 키가 올바르지 않습니다. 컴퓨터의 전체 링크를 다시 여세요.")
      return
    length_header = self._single_header("Content-Length")
    if (self.headers.get_all("Transfer-Encoding") or self.headers.get_all("Content-Encoding") or
        length_header is None or not length_header.isascii() or not length_header.isdigit() or len(length_header) > 12):
      self._json(400, ok=False, error="길이가 명시된 압축하지 않은 파일 전송만 지원합니다.")
      return
    length = int(length_header)
    if not 0 < length <= self.server.max_bundle_bytes:
      self._json(413, ok=False, error="파일이 비어 있거나 허용 용량을 넘었습니다.")
      return
    if self._single_header("Content-Type") not in ("application/octet-stream", "application/zip"):
      self._json(415, ok=False, error="웹당근에서 받은 로그 묶음 파일을 선택하세요.")
      return
    if not self.server.upload_lock.acquire(blocking=False):
      self._json(409, ok=False, error="다른 파일을 처리하고 있습니다. 완료 후 다시 보내세요.")
      return
    archive = None
    response_status, response = 500, {"ok": False, "error": "파일 처리에 실패했습니다. 같은 파일을 다시 보내세요."}
    try:
      self._require_space(2 * length + FREE_RESERVE_BYTES)
      fd, temporary_name = tempfile.mkstemp(prefix=".dk-incoming-", suffix=".zip", dir=self.server.output)
      archive = Path(temporary_name)
      deadline = time.monotonic() + self.server.upload_timeout
      with os.fdopen(fd, "wb") as stream:
        remaining = length
        next_space_check = length - 16 * CHUNK_BYTES
        while remaining:
          if self.server.stopping.is_set() or time.monotonic() > deadline:
            raise UploadError(408, "전송 제한 시간이 지났습니다. 같은 파일을 다시 보내세요.")
          chunk = self.rfile.read1(min(CHUNK_BYTES, remaining))
          if not chunk:
            raise UploadError(400, "전송이 중단되었습니다. 같은 파일을 다시 보내세요.")
          stream.write(chunk)
          remaining -= len(chunk)
          if remaining <= next_space_check:
            self._require_space(remaining + length + FREE_RESERVE_BYTES)
            next_space_check = remaining - 16 * CHUNK_BYTES
        stream.flush()
        os.fsync(stream.fileno())
      self._require_space(length + FREE_RESERVE_BYTES)
      if self.server.stopping.is_set():
        raise UploadError(503, "수신기가 종료 중입니다. 다시 실행한 후 보내세요.")
      receipt = self.server.importer(archive, self.server.output)
      response_status, response = 200, {"ok": True, "receipt": receipt}
    except UploadError as exc:
      response_status, response = exc.status, {"ok": False, "error": exc.message}
    except TimeoutError:
      response_status, response = 408, {"ok": False, "error": "전송이 오래 중단되었습니다. 같은 파일을 다시 보내세요."}
    except ValueError:
      # TransferError is a ValueError. Do not expose parser paths or untrusted
      # archive text in errors, and never acknowledge before import validation.
      response_status, response = 422, {"ok": False, "error": "로그 묶음의 구조·크기·SHA-256 검증에 실패했습니다. 차량에서 묶음을 다시 받으세요."}
    except OSError:
      response_status, response = 500, {"ok": False, "error": "파일 보관에 실패했습니다. 컴퓨터의 저장 공간과 폴더 권한을 확인하세요."}
    except Exception:
      # Do not leak unexpected parser exception details or strand the upload lock.
      response_status, response = 500, {"ok": False, "error": "파일 처리에 실패했습니다. 수신기 버전을 확인한 후 다시 보내세요."}
    finally:
      try:
        if archive is not None:
          archive.unlink(missing_ok=True)
      except OSError:
        response_status, response = 500, {"ok": False, "error": "임시 파일 정리에 실패했습니다. 컴퓨터의 폴더 권한을 확인하세요."}
      finally:
        self.server.upload_lock.release()
    self._json(response_status, **response)

  def _require_space(self, minimum: int):
    if shutil.disk_usage(self.server.output).free < minimum:
      raise UploadError(507, "컴퓨터의 여유 공간이 부족합니다. 파일 복사·검증 공간을 확보한 후 다시 보내세요.")


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--bind", default="127.0.0.1", help="explicit private LAN IPv4; default localhost only")
  parser.add_argument("--port", type=int, default=8766)
  parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="private local/NAS mounted folder for verified logs")
  parser.add_argument("--open-browser", action="store_true", help="open the private receiver link in this computer's browser")
  args = parser.parse_args(argv)
  try:
    server = ReceiverServer(args.bind, args.port, args.output)
  except (ValueError, OSError) as exc:
    parser.error(str(exc))
  with server:
    print("dk 로그 전달 수신기 — 신뢰할 수 있는 집 Wi-Fi에서만 사용하세요. HTTP는 암호화되지 않습니다.", flush=True)
    print("휴대폰에서 열 전체 링크 (접속 키 포함, 공유하지 마세요):", flush=True)
    print(server.invitation_url, flush=True)
    print(f"검증 후 보관 위치: {server.output}", flush=True)
    if server.server_address[0].startswith("127."):
      print("현재 컴퓨터에서만 접속됩니다. 휴대폰 수신은 --bind에 이 컴퓨터의 집 Wi-Fi IPv4 주소를 지정하세요.", flush=True)
    print("전달하는 동안 실행해 두고, 완료 후 Ctrl+C로 종료하세요. 외부 서버에는 전송하지 않습니다.", flush=True)
    if args.open_browser:
      webbrowser.open(server.invitation_url)
    try:
      server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
      print("\ndk 로그 수신기를 종료합니다.", flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
