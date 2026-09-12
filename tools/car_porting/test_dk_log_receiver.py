from contextlib import contextmanager
import hashlib
import http.client
import json
from pathlib import Path
import re
import socket
import threading
import time
from types import SimpleNamespace

import pytest

from tools.car_porting import dk_log_receiver as receiver  # noqa: TID251


@contextmanager
def running_server(tmp_path, *, importer=None, maximum=1024):
  calls = []

  def accept(archive, destination):
    calls.append((archive.read_bytes(), destination, archive.stat().st_mode & 0o777))
    return {"transfer_id": "test-transfer", "capture_count": 1, "partial_count": 0, "total_bytes": archive.stat().st_size,
            "destination": str(destination / "test-transfer"), "duplicate": False}

  server = receiver.ReceiverServer("127.0.0.1", 0, tmp_path / "inbox", importer=importer or accept, max_bundle_bytes=maximum)
  worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
  worker.start()
  try:
    yield server, calls
  finally:
    server.stopping.set()
    server.shutdown()
    server.server_close()
    worker.join(timeout=2)
    assert not worker.is_alive()


@pytest.fixture
def endpoint(tmp_path):
  with running_server(tmp_path) as value:
    yield value


def request(server, *, method="POST", path="/api/import", body=b"bundle", headers=None):
  fields = {"Origin": server.base_url, "X-DK-Transfer-Token": server.token, "Content-Type": "application/octet-stream"}
  if headers:
    for key, value in headers.items():
      if value is None:
        fields.pop(key, None)
      else:
        fields[key] = value
  connection = http.client.HTTPConnection(*server.server_address, timeout=2)
  try:
    connection.request(method, path, body=body, headers=fields)
    response = connection.getresponse()
    result = response.status, dict(response.getheaders()), response.read()
  finally:
    connection.close()
  return result


def test_page_is_local_only_static_and_contains_no_secret_or_log_listing(endpoint):
  server, calls = endpoint
  status, headers, data = request(server, method="GET", path="/", body=None)
  text = data.decode()
  assert status == 200
  assert server.token not in text
  assert str(server.output) not in text
  assert "dk 로그 전달" in text
  assert 'data-max-bytes="1024"' in text
  assert ".dklog.zip" in text
  assert "xhr.send(file)" in text
  assert "readAsArrayBuffer" not in text
  assert "localStorage" not in text
  assert "history.replaceState" in text
  assert "location.hash" in text
  assert "http://" not in text and "https://" not in text
  assert calls == []
  assert headers["Referrer-Policy"] == "no-referrer"
  assert headers["Cache-Control"] == "no-store"
  assert headers["X-Content-Type-Options"] == "nosniff"
  assert headers["X-Frame-Options"] == "DENY"
  assert "Access-Control-Allow-Origin" not in headers
  assert "default-src 'none'" in headers["Content-Security-Policy"]
  assert "frame-ancestors 'none'" in headers["Content-Security-Policy"]
  assert receiver._hash_source(receiver.SCRIPT) in headers["Content-Security-Policy"]
  assert receiver._hash_source(receiver.STYLE) in headers["Content-Security-Policy"]


def test_valid_import_is_streamed_private_verified_and_temp_cleaned(endpoint):
  server, calls = endpoint
  status, _, data = request(server)
  result = json.loads(data)
  assert status == 200
  assert result["ok"] is True
  assert result["receipt"]["transfer_id"] == "test-transfer"
  assert calls == [(b"bundle", server.output, 0o600)]
  assert not list(server.output.iterdir())
  assert not server.upload_lock.locked()


def test_head_does_not_send_body(endpoint):
  server, _ = endpoint
  status, headers, data = request(server, method="HEAD", path="/", body=None)
  assert status == 200
  assert int(headers["Content-Length"]) > 1000
  assert not data


@pytest.mark.parametrize("headers,status", [
  ({"Origin": None}, 403),
  ({"Origin": "null"}, 403),
  ({"Origin": "https://evil.example"}, 403),
  ({"Origin": "http://127.0.0.1"}, 403),
  ({"X-DK-Transfer-Token": None}, 403),
  ({"X-DK-Transfer-Token": "invalid"}, 403),
  ({"X-DK-Transfer-Token": "é"}, 403),
  ({"Host": "evil.example:8766"}, 403),
  ({"Host": "127.0.0.1"}, 403),
  ({"Content-Length": "1025"}, 413),
  ({"Content-Length": "0"}, 413),
  ({"Content-Length": "-1"}, 400),
  ({"Content-Length": "6,6"}, 400),
  ({"Content-Length": "10000000000000"}, 400),
  ({"Transfer-Encoding": "chunked"}, 400),
  ({"Content-Encoding": "gzip"}, 400),
  ({"Content-Type": "multipart/form-data; boundary=not-accepted"}, 415),
  ({"Content-Type": "text/plain"}, 415),
])
def test_bad_boundary_requests_never_import_or_leave_files(endpoint, headers, status):
  server, calls = endpoint
  result, _, data = request(server, headers=headers)
  assert result == status
  assert json.loads(data)["ok"] is False
  assert calls == []
  assert not list(server.output.iterdir())


@pytest.mark.parametrize("path", ["/api/import?token=secret", "/inbox", "/../", "/?token=secret", "/logs", "/api/list"])
def test_no_log_reading_or_query_token_endpoints(endpoint, path):
  server, calls = endpoint
  status, _, data = request(server, method="GET", path=path, body=None)
  assert status == 404
  assert b"secret" not in data
  assert calls == []


def test_cross_origin_preflight_is_not_enabled(endpoint):
  server, _ = endpoint
  status, headers, _ = request(server, method="OPTIONS", headers={"Origin": "https://evil.example"})
  assert status == 501
  assert "Access-Control-Allow-Origin" not in headers


def test_expect_body_preflight_is_rejected_before_continue(endpoint):
  server, calls = endpoint
  status, _, _ = request(server, headers={"Expect": "100-continue"})
  assert status == 417
  assert calls == []


@pytest.mark.parametrize("duplicate", ["Content-Length", "Host", "Origin", "X-DK-Transfer-Token", "Content-Type"])
def test_duplicate_critical_headers_are_rejected(endpoint, duplicate):
  server, calls = endpoint
  fields = {"Host": server.base_url.removeprefix("http://"), "Origin": server.base_url,
            "X-DK-Transfer-Token": server.token, "Content-Type": "application/octet-stream", "Content-Length": "6"}
  lines = ["POST /api/import HTTP/1.1", *[f"{key}: {value}" for key, value in fields.items()], f"{duplicate}: {fields[duplicate]}"]
  with socket.create_connection(server.server_address, timeout=2) as connection:
    connection.sendall(("\r\n".join(lines) + "\r\n\r\nbundle").encode())
    data = connection.recv(4096)
  assert b" 200 " not in data
  assert calls == []
  assert not list(server.output.iterdir())


def test_missing_length_rejected_without_waiting_for_body(endpoint):
  server, calls = endpoint
  text = (f"POST /api/import HTTP/1.1\r\nHost: {server.server_address[0]}:{server.server_address[1]}\r\n" +
          f"Origin: {server.base_url}\r\nX-DK-Transfer-Token: {server.token}\r\nContent-Type: application/octet-stream\r\n\r\n")
  with socket.create_connection(server.server_address, timeout=2) as connection:
    connection.sendall(text.encode())
    assert b" 400 " in connection.recv(4096)
  assert calls == []


def test_insufficient_space_prevents_body_temp_creation(endpoint, monkeypatch):
  server, calls = endpoint
  monkeypatch.setattr(receiver.shutil, "disk_usage", lambda _: SimpleNamespace(free=receiver.FREE_RESERVE_BYTES + 11))
  status, _, _ = request(server)
  assert status == 507
  assert calls == []
  assert not list(server.output.iterdir())


def test_low_space_after_receive_removes_temp_without_import(endpoint, monkeypatch):
  server, calls = endpoint
  checks = iter((receiver.FREE_RESERVE_BYTES + 12, 0))
  monkeypatch.setattr(receiver.shutil, "disk_usage", lambda _: SimpleNamespace(free=next(checks)))
  status, _, _ = request(server)
  assert status == 507
  assert calls == []
  assert not list(server.output.iterdir())


@pytest.mark.parametrize("failure,status", [(ValueError("secret/private/path"), 422), (OSError("secret/private/path"), 500)])
def test_failed_import_never_reports_success_and_cleans_only_own_temp(tmp_path, failure, status):
  def fail(archive, destination):
    assert archive.is_file()
    raise failure

  with running_server(tmp_path, importer=fail) as (server, _):
    original = server.output / "unrelated-original"
    original.write_bytes(b"keep")
    actual_status, _, data = request(server)
    assert actual_status == status
    assert b"secret" not in data
    assert not json.loads(data)["ok"]
    assert list(server.output.iterdir()) == [original]
    assert original.read_bytes() == b"keep"
    assert not server.upload_lock.locked()


def test_only_one_upload_is_accepted(endpoint):
  server, calls = endpoint
  server.upload_lock.acquire()
  try:
    status, _, _ = request(server)
  finally:
    server.upload_lock.release()
  assert status == 409
  assert calls == []
  assert not list(server.output.iterdir())


def _partial_upload(server, body=b"a", length=20):
  connection = socket.create_connection(server.server_address, timeout=2)
  text = (f"POST /api/import HTTP/1.1\r\nHost: {server.server_address[0]}:{server.server_address[1]}\r\n" +
          f"Origin: {server.base_url}\r\nX-DK-Transfer-Token: {server.token}\r\nContent-Type: application/octet-stream\r\n" +
          f"Content-Length: {length}\r\n\r\n")
  connection.sendall(text.encode() + body)
  return connection


def _wait_until(predicate, timeout=2):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if predicate():
      return
    time.sleep(0.01)
  raise AssertionError("condition did not become true")


def test_disconnect_cleans_partial_and_accepts_retry(endpoint):
  server, calls = endpoint
  connection = _partial_upload(server)
  _wait_until(server.upload_lock.locked)
  connection.close()
  _wait_until(lambda: not server.upload_lock.locked())
  assert not list(server.output.iterdir())
  assert calls == []
  assert request(server)[0] == 200


def test_idle_timeout_cleans_partial_and_releases_upload_slot(endpoint):
  server, calls = endpoint
  server.idle_timeout = 0.1
  with _partial_upload(server) as connection:
    data = connection.recv(4096)
  _wait_until(lambda: not server.upload_lock.locked())
  assert b" 408 " in data
  assert not list(server.output.iterdir())
  assert calls == []


def test_total_deadline_cleans_partial(endpoint):
  server, calls = endpoint
  server.upload_timeout = -1
  status, _, _ = request(server)
  assert status == 408
  assert not server.upload_lock.locked()
  assert not list(server.output.iterdir())
  assert calls == []


def test_invitation_key_is_strong_per_run_and_only_in_fragment(tmp_path):
  with running_server(tmp_path / "a") as (first, _), running_server(tmp_path / "b") as (second, _):
    assert re.fullmatch("[A-Za-z0-9_-]{43}", first.token)
    assert first.token != second.token
    assert first.invitation_url == first.base_url + "/#token=" + first.token
    assert "?" not in first.invitation_url


@pytest.mark.parametrize("value", ["0.0.0.0", "::", "::1", "8.8.8.8", "169.254.1.1", "100.64.1.1", "192.0.2.1", "evil.example", "127.1"])
def test_public_wildcard_and_hostname_bind_rejected_without_network(value, tmp_path):
  with pytest.raises(ValueError, match="explicit private IPv4"):
    receiver.ReceiverServer(value, 0, tmp_path / "unused", importer=lambda *_: {}, max_bundle_bytes=1024)
  assert not (tmp_path / "unused").exists()


@pytest.mark.parametrize("value", ["127.0.0.1", "10.0.0.5", "172.16.0.2", "172.31.255.254", "192.168.50.2"])
def test_private_ipv4_bind_validation(value):
  assert receiver.validate_bind(value) == value
  assert receiver.is_local_address(value)


def test_localhost_validation():
  assert receiver.validate_bind("localhost") == "127.0.0.1"


def test_public_peer_is_rejected_even_with_good_host(endpoint):
  server, _ = endpoint
  assert not server.verify_request(None, ("8.8.8.8", 1234))
  assert server.verify_request(None, ("192.168.50.3", 1234))


def test_no_http_request_or_token_in_default_logging(endpoint, capsys):
  server, _ = endpoint
  request(server, method="GET", path="/private?token=" + server.token)
  output = capsys.readouterr()
  assert output.out == output.err == ""


def test_byte_stream_is_read_in_bounded_chunks(tmp_path, monkeypatch):
  monkeypatch.setattr(receiver, "CHUNK_BYTES", 7)
  payload = bytes(range(256)) * 4
  with running_server(tmp_path, maximum=len(payload)) as (server, calls):
    status, _, data = request(server, body=payload)
    assert status == 200
    assert calls[0][0] == payload
    assert json.loads(data)["receipt"]["total_bytes"] == len(payload)


def test_default_destination_is_local_gitignored_workspace_inbox():
  assert receiver.DEFAULT_OUTPUT == receiver.ROOT / "local_comma3x" / "dk-phone-inbox"


def test_server_shutdown_rejects_new_peer(endpoint):
  server, _ = endpoint
  server.stopping.set()
  assert not server.verify_request(None, ("127.0.0.1", 1234))


def test_static_csp_hashes_match_html_script_and_style():
  scripts = re.findall(r"<script>(.*?)</script>", receiver.PAGE, re.DOTALL)
  styles = re.findall(r"<style>(.*?)</style>", receiver.PAGE, re.DOTALL)
  assert scripts == [receiver.SCRIPT]
  assert styles == [receiver.STYLE]
  assert len(hashlib.sha256(scripts[0].encode()).digest()) == 32


def test_cli_help_has_manual_lan_and_destination_options(capsys):
  with pytest.raises(SystemExit) as exc:
    receiver.main(["--help"])
  assert exc.value.code == 0
  output = capsys.readouterr().out
  assert "--bind" in output
  assert "--output" in output
  assert "--port" in output
  assert "--open-browser" in output


def test_cli_rejects_unsafe_binding_without_starting_server(tmp_path, capsys):
  with pytest.raises(SystemExit) as exc:
    receiver.main(["--bind", "0.0.0.0", "--output", str(tmp_path / "unused")])
  assert exc.value.code == 2
  assert "explicit private IPv4" in capsys.readouterr().err
  assert not (tmp_path / "unused").exists()


@pytest.mark.parametrize("status", ["ready", "partial"])
def test_real_bundle_through_http_is_verified_preserved_and_idempotent(tmp_path, status):
  from openpilot.selfdrive.carrot import dk_log_transfer as transfer
  from openpilot.selfdrive.carrot.tests.test_dk_log_transfer import make_archive

  archive, index, source, _ = make_archive(tmp_path, status=status)
  original = archive.read_bytes()
  with running_server(tmp_path, importer=transfer.import_bundle, maximum=transfer.MAX_BUNDLE_BYTES) as (server, _):
    code, _, data = request(server, body=original)
    assert code == 200
    receipt = json.loads(data)["receipt"]
    assert receipt["transfer_id"] == index["transfer_id"]
    assert receipt["partial_count"] == int(status == "partial")
    assert receipt["capture_count"] == 1
    assert receipt["duplicate"] is False
    for path in source.iterdir():
      destination = Path(receipt["destination"]) / "captures" / source.name / path.name
      assert hashlib.sha256(destination.read_bytes()).digest() == hashlib.sha256(path.read_bytes()).digest()
    assert archive.read_bytes() == original
    assert len(list(server.output.iterdir())) == 1
    code, _, data = request(server, body=original)
    assert code == 200
    assert json.loads(data)["receipt"]["duplicate"] is True
    assert len(list(server.output.iterdir())) == 1


def test_real_corrupt_bundle_is_never_imported_or_acknowledged(tmp_path):
  from openpilot.selfdrive.carrot import dk_log_transfer as transfer
  from openpilot.selfdrive.carrot.tests.test_dk_log_transfer import make_archive, rewrite_archive

  archive, *_ = make_archive(tmp_path)
  rewrite_archive(archive, mutate_index=lambda index: index["files"][1].update(sha256="0" * 64))
  original = archive.read_bytes()
  with running_server(tmp_path, importer=transfer.import_bundle, maximum=transfer.MAX_BUNDLE_BYTES) as (server, _):
    status, _, data = request(server, body=original)
    assert status == 422
    assert json.loads(data)["ok"] is False
    assert not list(server.output.iterdir())
    assert archive.read_bytes() == original


def test_receiver_default_importer_uses_shared_limits_and_canonical_output(tmp_path):
  from openpilot.selfdrive.carrot import dk_log_transfer as transfer

  with receiver.ReceiverServer("localhost", 0, tmp_path / "inbox") as server:
    assert server.importer is transfer.import_bundle
    assert server.max_bundle_bytes == transfer.MAX_BUNDLE_BYTES
    assert server.output == (tmp_path / "inbox").resolve()


def test_request_thread_count_is_bounded(endpoint):
  server, _ = endpoint
  for _ in range(receiver.MAX_CONNECTIONS):
    assert server.request_slots.acquire(blocking=False)
  try:
    with pytest.raises((ConnectionError, http.client.RemoteDisconnected)):
      request(server, method="GET", path="/")
  finally:
    for _ in range(receiver.MAX_CONNECTIONS):
      server.request_slots.release()
  assert request(server, method="GET", path="/", body=None)[0] == 200


def test_unexpected_import_failure_cleans_and_does_not_leak_details(tmp_path):
  def fail(*_):
    raise RuntimeError("private untrusted parser content")

  with running_server(tmp_path, importer=fail) as (server, _):
    status, _, data = request(server)
    assert status == 500
    assert b"private" not in data
    assert not server.upload_lock.locked()
    assert not list(server.output.iterdir())


def test_disk_write_failure_cleans_temp_and_leaves_originals(endpoint, monkeypatch):
  server, calls = endpoint
  original = server.output / "prior-log"
  original.write_bytes(b"untouched")

  def fail(_):
    raise OSError("private fsync details")

  monkeypatch.setattr(receiver.os, "fsync", fail)
  status, _, data = request(server)
  assert status == 500
  assert b"private" not in data
  assert calls == []
  assert list(server.output.iterdir()) == [original]
  assert original.read_bytes() == b"untouched"
  assert not server.upload_lock.locked()


def test_open_browser_is_explicit_and_uses_same_private_fragment_link(tmp_path, monkeypatch, capsys):
  opened = []
  monkeypatch.setattr(receiver.webbrowser, "open", opened.append)
  monkeypatch.setattr(receiver.ReceiverServer, "serve_forever", lambda *_args, **_kwargs: None)
  assert receiver.main(["--port", "0", "--output", str(tmp_path / "first")]) == 0
  assert opened == []
  capsys.readouterr()
  assert receiver.main(["--port", "0", "--output", str(tmp_path / "second"), "--open-browser"]) == 0
  assert len(opened) == 1
  assert opened[0].startswith("http://127.0.0.1:")
  assert "/#token=" in opened[0]
  assert opened[0] in capsys.readouterr().out
