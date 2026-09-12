import hashlib
import io
import json
from pathlib import Path
import stat
import struct
import zipfile

import pytest

from openpilot.selfdrive.carrot import dk_log_transfer as transfer


def create_capture(root, cid="a" * 32, *, status="ready", payload=b"original full cereal rlog", extension=".zst"):
  directory = root / cid
  directory.mkdir(parents=True)
  name = f"2-rlog{extension}"
  (directory / name).write_bytes(payload)
  manifest = {"schema": 1, "capture_id": cid, "route": "route-sample", "center_segment": 2,
              "status": status, "created_at": 1000, "expires_at": 605800, "topics": ["braking"],
              "event": {"kind": "sample", "lead": {"status": False}},
              "session": {"branch": "dkcarrot-wip", "commit": "test-commit", "initial_params": {"PathOffset": 10}},
              "files": [{"name": name, "segment": "route-sample--2", "segment_index": 2,
                         "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}],
              "missing": [] if status == "ready" else [{"segment": "route-sample--3", "reason": "not_available"}]}
  (directory / "manifest.json").write_text(json.dumps(manifest, indent=2))
  return directory, manifest


def make_archive(tmp_path, **kwargs):
  root = tmp_path / "captures"
  directory, manifest = create_capture(root, **kwargs)
  archive = tmp_path / "logs.dklog.zip"
  with archive.open("wb") as out:
    index = transfer.build_bundle(root, out)
  return archive, index, directory, manifest


def rewrite_archive(path, mutate_index=None, mutate_members=None, compression=zipfile.ZIP_STORED):
  with zipfile.ZipFile(path) as archive:
    entries = {info.filename: archive.read(info) for info in archive.infolist()}
  index = json.loads(entries[transfer.INDEX_NAME])
  if mutate_index:
    mutate_index(index)
  entries[transfer.INDEX_NAME] = json.dumps(index).encode()
  if mutate_members:
    mutate_members(entries)
  with zipfile.ZipFile(path, "w", compression=compression) as archive:
    for name, content in entries.items():
      archive.writestr(name, content)


@pytest.mark.parametrize("extension", [".zst", ".bz2", ""])
@pytest.mark.parametrize("status", ["ready", "partial"])
def test_roundtrip_preserves_full_originals_and_metadata(tmp_path, extension, status):
  archive, index, directory, _ = make_archive(tmp_path, extension=extension, status=status)
  result = transfer.import_bundle(archive, tmp_path / "received")
  target = Path(result["destination"])
  assert result == {"transfer_id": index["transfer_id"], "capture_count": 1, "partial_count": int(status == "partial"),
                    "total_bytes": sum(item["size"] for item in index["files"]), "destination": str(target), "duplicate": False}
  for source in directory.iterdir():
    copied = target / "captures" / directory.name / source.name
    assert copied.read_bytes() == source.read_bytes()
    assert stat.S_IMODE(copied.stat().st_mode) == 0o600
  assert (target / transfer.INDEX_NAME).is_file()
  assert not list((tmp_path / "received").glob(".dk-import-*"))
  assert archive.is_file()
  assert transfer.import_bundle(archive, tmp_path / "received")["duplicate"] is True


def test_pending_excluded_and_explicit_selection_fails(tmp_path):
  root = tmp_path / "captures"
  ready, _ = create_capture(root)
  pending, _ = create_capture(root, "b" * 32, status="pending")
  assert transfer.build_bundle(root, io.BytesIO())["captures"] == [{"capture_id": ready.name, "status": "ready"}]
  with pytest.raises(transfer.TransferError):
    transfer.build_bundle(root, io.BytesIO(), [pending.name])
  with pytest.raises(transfer.TransferError):
    transfer.build_bundle(root, io.BytesIO(), ["../escape"])
  with pytest.raises(transfer.TransferError):
    transfer.build_bundle(root, io.BytesIO(), [ready.name, ready.name])


def test_no_ready_capture_is_not_empty_success(tmp_path):
  root = tmp_path / "captures"
  create_capture(root, status="pending")
  with pytest.raises(transfer.TransferError, match="no finalized"):
    transfer.build_bundle(root, io.BytesIO())


@pytest.mark.parametrize("kind", ["root", "capture", "manifest", "artifact"])
def test_export_rejects_symlinks(tmp_path, kind):
  root = tmp_path / "captures"
  directory, _ = create_capture(root)
  source = {"root": root, "capture": directory, "manifest": directory / "manifest.json", "artifact": directory / "2-rlog.zst"}[kind]
  moved = source.with_name(source.name + "-original")
  source.rename(moved)
  source.symlink_to(moved)
  with pytest.raises(transfer.TransferError):
    transfer.build_bundle(root, io.BytesIO())


@pytest.mark.parametrize("kind", ["size", "hash", "manifest_oversize", "file_missing"])
def test_export_rejects_corrupt_sources(tmp_path, kind):
  root = tmp_path / "captures"
  directory, _ = create_capture(root)
  source = directory / "2-rlog.zst"
  if kind == "size":
    source.write_bytes(b"short")
  elif kind == "hash":
    source.write_bytes(b"x" * source.stat().st_size)
  elif kind == "manifest_oversize":
    (directory / "manifest.json").write_bytes(b" " * (transfer.MAX_MANIFEST_BYTES + 1))
  else:
    source.unlink()
  with pytest.raises(transfer.TransferError):
    transfer.build_bundle(root, io.BytesIO())


def test_export_rechecks_source_after_copy(tmp_path, monkeypatch):
  root = tmp_path / "captures"
  directory, _ = create_capture(root)
  original = transfer._unchanged
  mutated = False

  def mutate(path, stream, before):
    nonlocal mutated
    if not mutated and path.name == "2-rlog.zst":
      path.write_bytes(b"x" * before.st_size)
      mutated = True
    original(path, stream, before)

  monkeypatch.setattr(transfer, "_unchanged", mutate)
  with pytest.raises(transfer.TransferError, match="source changed"):
    transfer.build_bundle(root, io.BytesIO())
  assert (directory / "manifest.json").is_file()


@pytest.mark.parametrize("mutate", [
  lambda index: index.update(transfer_id="../escape"),
  lambda index: index.update(format="other-format"),
  lambda index: index.update(created_at="not-a-date"),
  lambda index: index.update(created_at="2026-09-12T12:00:00"),
  lambda index: index["files"][0].update(name="../secret"),
  lambda index: index["files"][0].update(name="/absolute"),
  lambda index: index["files"][0].update(name="captures/" + "a" * 32 + "/../../escape"),
  lambda index: index["files"][0].update(size=True),
  lambda index: index["files"][0].update(size=transfer.MAX_MANIFEST_BYTES + 1),
  lambda index: index["files"][1].update(sha256="0" * 64),
  lambda index: index["captures"][0].update(status="pending"),
  lambda index: index["captures"].append(index["captures"][0].copy()),
  lambda index: index["files"].append(index["files"][0].copy()),
])
def test_import_rejects_invalid_index(tmp_path, mutate):
  archive, *_ = make_archive(tmp_path)
  rewrite_archive(archive, mutate_index=mutate)
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(archive, tmp_path / "received")
  assert not list((tmp_path / "received").iterdir())


@pytest.mark.parametrize("kind", ["extra", "missing", "hash", "compressed", "duplicate", "symlink", "directory", "truncated"])
def test_import_rejects_unsafe_archive_without_partial_success(tmp_path, kind):
  archive, index, _, _ = make_archive(tmp_path)
  artifact = index["files"][1]["name"]
  if kind == "extra":
    rewrite_archive(archive, mutate_members=lambda entries: entries.update({"arbitrary.txt": b"secret"}))
  elif kind == "missing":
    rewrite_archive(archive, mutate_members=lambda entries: entries.pop(artifact))
  elif kind == "hash":
    rewrite_archive(archive, mutate_members=lambda entries: entries.update({artifact: b"x" * len(entries[artifact])}))
  elif kind == "compressed":
    rewrite_archive(archive, compression=zipfile.ZIP_DEFLATED)
  elif kind in ("duplicate", "symlink", "directory"):
    with zipfile.ZipFile(archive, "a") as out:
      if kind == "duplicate":
        with pytest.warns(UserWarning):
          out.writestr(artifact, b"duplicate")
      else:
        info = zipfile.ZipInfo("link" if kind == "symlink" else "directory/")
        info.create_system = 3
        info.external_attr = ((stat.S_IFLNK if kind == "symlink" else stat.S_IFDIR) | 0o777) << 16
        out.writestr(info, b"target" if kind == "symlink" else b"")
  else:
    archive.write_bytes(archive.read_bytes()[:-20])
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(archive, tmp_path / "received")
  assert not list((tmp_path / "received").iterdir())


def test_import_rejects_manifest_inconsistent_with_index_even_if_rehashed(tmp_path):
  archive, *_ = make_archive(tmp_path)
  with zipfile.ZipFile(archive) as bundle:
    entries = {info.filename: bundle.read(info) for info in bundle.infolist()}
  index = json.loads(entries[transfer.INDEX_NAME])
  name = index["files"][0]["name"]
  manifest = json.loads(entries[name])
  manifest["files"][0]["sha256"] = "0" * 64
  entries[name] = json.dumps(manifest).encode()
  index["files"][0].update(size=len(entries[name]), sha256=hashlib.sha256(entries[name]).hexdigest())
  entries[transfer.INDEX_NAME] = json.dumps(index).encode()
  with zipfile.ZipFile(archive, "w") as bundle:
    for name, raw in entries.items():
      bundle.writestr(name, raw)
  with pytest.raises(transfer.TransferError, match="artifact does not match"):
    transfer.import_bundle(archive, tmp_path / "received")


def test_duplicate_must_be_complete_and_cannot_overwrite(tmp_path):
  archive, index, *_ = make_archive(tmp_path)
  result = transfer.import_bundle(archive, tmp_path / "received")
  target = Path(result["destination"])
  source = target / index["files"][1]["name"]
  source.write_bytes(b"existing private contents")
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(archive, tmp_path / "received")
  assert source.read_bytes() == b"existing private contents"
  assert list((tmp_path / "received").iterdir()) == [target]


def test_same_id_different_index_is_conflict_not_overwrite(tmp_path):
  archive, *_ = make_archive(tmp_path)
  result = transfer.import_bundle(archive, tmp_path / "received")
  target = Path(result["destination"])
  original = (target / transfer.INDEX_NAME).read_bytes()
  rewrite_archive(archive, mutate_index=lambda index: index.update(created_at="2026-09-12T00:00:00+00:00"))
  with pytest.raises(transfer.TransferError, match="conflicts"):
    transfer.import_bundle(archive, tmp_path / "received")
  assert (target / transfer.INDEX_NAME).read_bytes() == original


def test_destination_and_archive_links_rejected(tmp_path):
  archive, *_ = make_archive(tmp_path)
  destination = tmp_path / "received"
  destination.mkdir()
  linked = tmp_path / "linked"
  linked.symlink_to(destination)
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(archive, linked)
  alias = tmp_path / "alias.zip"
  alias.symlink_to(archive)
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(alias, destination)


def test_limits_are_enforced_before_import_copy(tmp_path, monkeypatch):
  archive, *_ = make_archive(tmp_path)
  monkeypatch.setattr(transfer.shutil, "disk_usage", lambda _: type("Usage", (), {"free": 1})())
  with pytest.raises(transfer.TransferError, match="free space"):
    transfer.import_bundle(archive, tmp_path / "received")
  assert not list((tmp_path / "received").iterdir())


def test_payload_limit_export_and_import(tmp_path, monkeypatch):
  archive, *_ = make_archive(tmp_path)
  monkeypatch.setattr(transfer, "MAX_PAYLOAD_BYTES", 1)
  with pytest.raises(transfer.TransferError, match="payload limit"):
    transfer.build_bundle(tmp_path / "captures", io.BytesIO())
  with pytest.raises(transfer.TransferError, match="payload limit"):
    transfer.import_bundle(archive, tmp_path / "received")


def test_import_lock_prevents_concurrent_publish(tmp_path):
  archive, index, *_ = make_archive(tmp_path)
  destination = tmp_path / "received"
  destination.mkdir()
  lock = destination / f".dk-import-{index['transfer_id']}.lock"
  lock.write_bytes(b"other importer")
  with pytest.raises(transfer.TransferError, match="already being imported"):
    transfer.import_bundle(archive, destination)
  assert list(destination.iterdir()) == [lock]
  assert lock.read_bytes() == b"other importer"


def test_nonseekable_export_is_importable(tmp_path):
  class NonSeekable(io.BytesIO):
    def seek(self, *args):
      raise io.UnsupportedOperation("not seekable")

  root = tmp_path / "captures"
  create_capture(root)
  output = NonSeekable()
  transfer.build_bundle(root, output)
  archive = tmp_path / "streamed.dklog.zip"
  archive.write_bytes(output.getvalue())
  assert transfer.import_bundle(archive, tmp_path / "received")["capture_count"] == 1


def test_capture_count_limit_and_explicit_subset(tmp_path):
  root = tmp_path / "captures"
  for number in range(11):
    create_capture(root, f"{number:032x}")
  with pytest.raises(transfer.TransferError, match="capture count"):
    transfer.build_bundle(root, io.BytesIO())
  index = transfer.build_bundle(root, io.BytesIO(), [f"{0:032x}"])
  assert len(index["captures"]) == 1


@pytest.mark.parametrize("raw", [b'{"schema":1,"schema":1}', b'{"extra":NaN}', b'{"extra":1e999}', b'{"extra":Infinity}'])
def test_metadata_rejects_duplicate_keys_and_nonfinite_values(raw):
  with pytest.raises(transfer.TransferError):
    transfer._json(raw)


@pytest.mark.parametrize("kind", ["excessive_entries", "excessive_directory", "archive_size", "index_compressed_size"])
def test_zip_metadata_limits_checked_before_unbounded_read(tmp_path, kind, monkeypatch):
  archive, *_ = make_archive(tmp_path)
  raw = bytearray(archive.read_bytes())
  end = raw.rfind(b"PK\x05\x06")
  if kind == "excessive_entries":
    struct.pack_into("<HH", raw, end + 8, 1000, 1000)
  elif kind == "excessive_directory":
    struct.pack_into("<L", raw, end + 12, transfer.MAX_INDEX_BYTES + 1)
  elif kind == "index_compressed_size":
    central = raw.index(b"PK\x01\x02")
    struct.pack_into("<L", raw, central + 20, transfer.MAX_BUNDLE_BYTES)
  else:
    monkeypatch.setattr(transfer, "MAX_BUNDLE_BYTES", 32)
  archive.write_bytes(raw)
  with pytest.raises(transfer.TransferError):
    transfer.import_bundle(archive, tmp_path / "received")
  assert not list((tmp_path / "received").iterdir())
