"""Portable, local-only DK diagnostic bundles. No network or vehicle imports.

The archive contains selected diagnostic candidates, not every drive or video.
Hashes detect incomplete/changed copies; they do not authenticate a device or
establish that a capture's observations are a vehicle diagnosis.
"""
from __future__ import annotations

from contextlib import ExitStack
from datetime import UTC, datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import tempfile
import uuid
import zipfile

FORMAT = "dk-log-transfer-v1"
INDEX_NAME = "dk-transfer.json"
MAX_CAPTURES = 10
MAX_MEMBERS = 40
MAX_PAYLOAD_BYTES = 1024 ** 3
MAX_BUNDLE_BYTES = MAX_PAYLOAD_BYTES + 4 * 1024 ** 2
MAX_FILE_BYTES = 256 * 1024 ** 2
MAX_MANIFEST_BYTES = 192 * 1024
MAX_INDEX_BYTES = 128 * 1024
MIN_FREE_BYTES = 64 * 1024 ** 2
CHUNK_BYTES = 1024 * 1024
CAPTURE_ID = re.compile(r"[0-9a-f]{32}\Z")
ARTIFACT = re.compile(r"([0-9]{1,10})-rlog(?:\.zst|\.bz2)?\Z")
MEMBER_NAME = re.compile(r"captures/([0-9a-f]{32})/(manifest\.json|[0-9]{1,10}-rlog(?:\.zst|\.bz2)?)\Z")
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
ROUTE = re.compile(r"[A-Za-z0-9_-]{1,100}\Z")


class TransferError(ValueError):
  """The bundle is invalid, incomplete, unsafe, or exceeds local limits."""


def _integer(value, minimum=0, maximum=MAX_PAYLOAD_BYTES):
  return type(value) is int and minimum <= value <= maximum


def _json(raw: bytes):
  def unique_pairs(items):
    result = {}
    for key, value in items:
      if key in result:
        raise TransferError("duplicate JSON key")
      result[key] = value
    return result

  def invalid_constant(_value):
    raise TransferError("non-finite JSON value")

  def finite_float(raw):
    value = float(raw)
    if not math.isfinite(value):
      raise TransferError("non-finite JSON value")
    return value

  try:
    value = json.loads(raw, object_pairs_hook=unique_pairs, parse_constant=invalid_constant, parse_float=finite_float)
    if not isinstance(value, dict):
      raise TransferError("JSON object required")
    return value
  except (UnicodeError, ValueError, RecursionError) as exc:
    raise TransferError("invalid JSON metadata") from exc


def _manifest(raw: bytes, capture_id: str):
  if not 0 < len(raw) <= MAX_MANIFEST_BYTES:
    raise TransferError("manifest size limit")
  value = _json(raw)
  if (type(value.get("schema")) is not int or value["schema"] != 1 or value.get("capture_id") != capture_id
      or value.get("status") not in ("ready", "partial")
      or not isinstance(value.get("route"), str) or not ROUTE.fullmatch(value["route"])
      or not _integer(value.get("center_segment"))
      or not isinstance(value.get("files"), list) or not 1 <= len(value["files"]) <= 3
      or not isinstance(value.get("missing"), list) or len(value["missing"]) > 3
      or (value["status"] == "ready" and value["missing"])):
    raise TransferError("capture is not a finalized diagnostic capture")
  names = set()
  segments = set()
  for item in value["files"]:
    if not isinstance(item, dict):
      raise TransferError("invalid artifact metadata")
    name = item.get("name")
    match = ARTIFACT.fullmatch(name) if isinstance(name, str) else None
    segment = item.get("segment_index")
    if (not match or name in names or not _integer(segment) or segment in segments
        or int(match[1]) != segment or item.get("segment") != f"{value['route']}--{segment}"
        or not _integer(item.get("bytes"), 1, MAX_FILE_BYTES)
        or not isinstance(item.get("sha256"), str) or not SHA256.fullmatch(item["sha256"])):
      raise TransferError("invalid artifact metadata")
    names.add(name)
    segments.add(segment)
  return value


def _signature(info):
  return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def _regular_open(path: Path, stack: ExitStack):
  try:
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode):
      raise TransferError("regular files required")
    stream = stack.enter_context(path.open("rb") if not hasattr(os, "O_NOFOLLOW") else
                                 os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb"))
    if _signature(before) != _signature(os.fstat(stream.fileno())):
      raise TransferError("source changed while opening")
    return stream, before
  except OSError as exc:
    raise TransferError("source file is unavailable") from exc


def _unchanged(path: Path, stream, before):
  try:
    if _signature(before) != _signature(path.lstat()) or _signature(before) != _signature(os.fstat(stream.fileno())):
      raise TransferError("source changed during transfer")
  except OSError as exc:
    raise TransferError("source changed during transfer") from exc


def _safe_directory(path: Path):
  # Do not follow a symlink at any existing component, including the root.
  absolute = path.absolute()
  for item in (absolute, *absolute.parents):
    if item.is_symlink():
      raise TransferError("symlink directories are not allowed")
  if not absolute.is_dir():
    raise TransferError("directory is unavailable")
  return absolute


def _entry(name: str):
  info = zipfile.ZipInfo(name)
  info.compress_type = zipfile.ZIP_STORED
  info.create_system = 3
  info.external_attr = (stat.S_IFREG | 0o600) << 16
  return info


def build_bundle(root: Path, output, capture_ids: list[str] | None = None) -> dict:
  """Write a bounded ZIP_STORED archive; caller removes output on any failure.

  Selected pending/empty captures fail explicitly. With no selection these are
  excluded, since their canonical manifest is still changing. No source is
  deleted and no capture is marked delivered by this function.
  """
  root = _safe_directory(Path(root))
  explicit = capture_ids is not None
  if explicit:
    if (not isinstance(capture_ids, list) or not 1 <= len(capture_ids) <= MAX_CAPTURES
        or any(not isinstance(cid, str) or not CAPTURE_ID.fullmatch(cid) for cid in capture_ids)
        or len(set(capture_ids)) != len(capture_ids)):
      raise TransferError("invalid capture selection")
    selected = sorted(capture_ids)
  else:
    selected = sorted(path.name for path in root.iterdir() if CAPTURE_ID.fullmatch(path.name))
    if len(selected) > MAX_CAPTURES:
      raise TransferError("capture count limit; select up to ten captures")
  index = {"format": FORMAT, "schema": 1, "transfer_id": uuid.uuid4().hex,
           "created_at": datetime.now(UTC).isoformat(), "captures": [], "files": []}
  total = 0
  with ExitStack() as stack:
    sources = []
    for cid in selected:
      directory = _safe_directory(root / cid)
      path = directory / "manifest.json"
      stream, before = _regular_open(path, stack)
      raw = stream.read(MAX_MANIFEST_BYTES + 1)
      candidate = _json(raw) if len(raw) <= MAX_MANIFEST_BYTES else {}
      if not explicit and (candidate.get("status") == "pending" or candidate.get("files") == []):
        continue
      manifest = _manifest(raw, cid)
      if len(index["captures"]) >= MAX_CAPTURES:
        raise TransferError("capture count limit")
      index["captures"].append({"capture_id": cid, "status": manifest["status"]})
      items = [(path, stream, before, len(raw), hashlib.sha256(raw).hexdigest())]
      for item in manifest["files"]:
        source = directory / item["name"]
        source_stream, source_stat = _regular_open(source, stack)
        if source_stat.st_size != item["bytes"]:
          raise TransferError("source size does not match manifest")
        items.append((source, source_stream, source_stat, item["bytes"], item["sha256"]))
      for source, source_stream, source_stat, size, digest in items:
        if source_stat.st_size != size:
          raise TransferError("source size does not match manifest")
        total += size
        if total > MAX_PAYLOAD_BYTES or len(index["files"]) >= MAX_MEMBERS:
          raise TransferError("bundle payload limit")
        name = f"captures/{cid}/{source.name}"
        index["files"].append({"name": name, "size": size, "sha256": digest})
        source_stream.seek(0)
        sources.append((source, source_stream, source_stat, name, size, digest))
    if not index["captures"]:
      raise TransferError("no finalized captures available")
    encoded = json.dumps(index, ensure_ascii=True, separators=(",", ":"), allow_nan=False).encode()
    if len(encoded) > MAX_INDEX_BYTES:
      raise TransferError("index size limit")
    try:
      with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED, allowZip64=False) as archive:
        archive.writestr(_entry(INDEX_NAME), encoded)
        for source, stream, before, name, size, expected in sources:
          count = 0
          digest = hashlib.sha256()
          with archive.open(_entry(name), "w") as target:
            while chunk := stream.read(CHUNK_BYTES):
              count += len(chunk)
              if count > size:
                raise TransferError("source grew during transfer")
              digest.update(chunk)
              target.write(chunk)
          if count != size or digest.hexdigest() != expected:
            raise TransferError("source hash does not match manifest")
          _unchanged(source, stream, before)
        # Recheck earlier files too: a capture can be pruned while another is read.
        for source, stream, before, *_ in sources:
          _unchanged(source, stream, before)
    except (OSError, zipfile.BadZipFile, zipfile.LargeZipFile) as exc:
      raise TransferError("bundle creation failed") from exc
  return index


def _zip_directory(stream):
  """Bound the central directory before ZipFile allocates attacker metadata."""
  size = os.fstat(stream.fileno()).st_size
  if not 22 <= size <= MAX_BUNDLE_BYTES:
    raise TransferError("archive size limit")
  stream.seek(max(0, size - 65557))
  tail = stream.read(65557)
  offset = tail.rfind(b"PK\x05\x06")
  if offset < 0 or len(tail) - offset < 22:
    raise TransferError("ZIP end record missing")
  _, disk, directory_disk, disk_count, count, directory_size, directory_offset, comment = struct.unpack(
    "<4s4H2LH", tail[offset:offset + 22])
  end_offset = size - len(tail) + offset
  if (disk or directory_disk or disk_count != count or not 2 <= count <= MAX_MEMBERS + 1
      or not 0 < directory_size <= MAX_INDEX_BYTES or directory_offset + directory_size != end_offset
      or offset + 22 + comment != len(tail) or comment):
    raise TransferError("unsupported ZIP layout")
  stream.seek(0)


def _index(raw: bytes):
  if not 0 < len(raw) <= MAX_INDEX_BYTES:
    raise TransferError("index size limit")
  value = _json(raw)
  if (set(value) != {"format", "schema", "transfer_id", "created_at", "captures", "files"}
      or value.get("format") != FORMAT or type(value.get("schema")) is not int or value["schema"] != 1
      or not isinstance(value.get("transfer_id"), str) or not CAPTURE_ID.fullmatch(value["transfer_id"])
      or not isinstance(value.get("created_at"), str) or len(value["created_at"]) > 64
      or not isinstance(value.get("captures"), list) or not 1 <= len(value["captures"]) <= MAX_CAPTURES
      or not isinstance(value.get("files"), list) or not 2 <= len(value["files"]) <= MAX_MEMBERS):
    raise TransferError("invalid transfer index")
  try:
    timestamp = datetime.fromisoformat(value["created_at"])
    if timestamp.utcoffset() is None or timestamp.utcoffset().total_seconds() != 0:
      raise ValueError("UTC required")
  except (ValueError, OverflowError) as exc:
    raise TransferError("invalid creation timestamp") from exc
  captures = {}
  for item in value["captures"]:
    if (not isinstance(item, dict) or set(item) != {"capture_id", "status"}
        or not isinstance(item["capture_id"], str) or not CAPTURE_ID.fullmatch(item["capture_id"])
        or item["capture_id"] in captures or item["status"] not in ("ready", "partial")):
      raise TransferError("invalid capture index")
    captures[item["capture_id"]] = item["status"]
  names = set()
  total = 0
  for item in value["files"]:
    if not isinstance(item, dict) or set(item) != {"name", "size", "sha256"}:
      raise TransferError("invalid file index")
    name = item["name"]
    match = MEMBER_NAME.fullmatch(name) if isinstance(name, str) else None
    maximum = MAX_MANIFEST_BYTES if match and match[2] == "manifest.json" else MAX_FILE_BYTES
    if (not match or match[1] not in captures or name in names or not _integer(item["size"], 1, maximum)
        or not isinstance(item["sha256"], str) or not SHA256.fullmatch(item["sha256"])):
      raise TransferError("invalid file index")
    names.add(name)
    total += item["size"]
  if total > MAX_PAYLOAD_BYTES:
    raise TransferError("bundle payload limit")
  if any(f"captures/{cid}/manifest.json" not in names for cid in captures):
    raise TransferError("capture manifest missing from index")
  return value, total


def _validate_members(archive, index):
  expected = {item["name"]: item["size"] for item in index["files"]}
  expected[INDEX_NAME] = None
  infos = archive.infolist()
  if len(infos) != len(expected) or len({item.filename for item in infos}) != len(infos):
    raise TransferError("duplicate, missing, or extra archive members")
  for info in infos:
    mode = info.external_attr >> 16
    if (info.filename not in expected or info.is_dir() or info.compress_type != zipfile.ZIP_STORED
        or info.compress_size != info.file_size or info.flag_bits & ~0x808
        or (stat.S_IFMT(mode) not in (0, stat.S_IFREG)) or info.extra or info.comment
        or (expected[info.filename] is not None and info.file_size != expected[info.filename])):
      raise TransferError("unsupported or mismatched archive member")


def _manifest_consistency(staging: Path, index):
  indexed = {item["name"]: item for item in index["files"]}
  expected = set()
  for capture in index["captures"]:
    cid = capture["capture_id"]
    prefix = f"captures/{cid}/"
    manifest_name = prefix + "manifest.json"
    manifest = _manifest((staging / manifest_name).read_bytes(), cid)
    if manifest["status"] != capture["status"]:
      raise TransferError("capture status does not match index")
    expected.add(manifest_name)
    for item in manifest["files"]:
      name = prefix + item["name"]
      if name not in indexed or indexed[name]["size"] != item["bytes"] or indexed[name]["sha256"] != item["sha256"]:
        raise TransferError("artifact does not match capture manifest")
      expected.add(name)
  if expected != set(indexed):
    raise TransferError("unlisted capture artifact")


def _existing_matches(directory: Path, index_raw: bytes, index):
  _safe_directory(directory)
  with ExitStack() as stack:
    stream, before = _regular_open(directory / INDEX_NAME, stack)
    if before.st_size != len(index_raw) or stream.read(MAX_INDEX_BYTES + 1) != index_raw:
      raise TransferError("transfer ID conflicts with an existing import")
    for item in index["files"]:
      path = directory / item["name"]
      _safe_directory(path.parent)
      stream, before = _regular_open(path, stack)
      if before.st_size != item["size"]:
        raise TransferError("existing import is incomplete")
      digest = hashlib.sha256()
      count = 0
      while chunk := stream.read(CHUNK_BYTES):
        count += len(chunk)
        if count > item["size"]:
          raise TransferError("existing import changed during verification")
        digest.update(chunk)
      if count != item["size"] or digest.hexdigest() != item["sha256"]:
        raise TransferError("existing import hash mismatch")
      _unchanged(path, stream, before)


def import_bundle(archive: Path, destination: Path) -> dict:
  """Validate and atomically import a manual phone bundle without overwriting.

  Caller owns/deletes the uploaded archive. Failed validation removes only this
  call's private staging directory. Existing imports and source files survive.
  """
  destination = Path(destination).absolute()
  for item in (destination, *destination.parents):
    if item.is_symlink():
      raise TransferError("symlink directories are not allowed")
  destination.mkdir(parents=True, exist_ok=True, mode=0o700)
  destination = _safe_directory(destination)
  staging = None
  try:
    with ExitStack() as stack:
      _safe_directory(Path(archive).parent)
      stream, before = _regular_open(Path(archive), stack)
      _zip_directory(stream)
      bundle = stack.enter_context(zipfile.ZipFile(stream))
      try:
        info = bundle.getinfo(INDEX_NAME)
      except KeyError as exc:
        raise TransferError("transfer index missing") from exc
      if (not 0 < info.file_size <= MAX_INDEX_BYTES or info.compress_type != zipfile.ZIP_STORED
          or info.compress_size != info.file_size or info.flag_bits & ~0x808):
        raise TransferError("invalid transfer index member")
      raw = bundle.read(info)
      index, total = _index(raw)
      _validate_members(bundle, index)
      if shutil.disk_usage(destination).free < total + len(raw) + MIN_FREE_BYTES:
        raise TransferError("insufficient free space for verified import")
      staging = Path(tempfile.mkdtemp(prefix=".dk-import-", dir=destination))
      for item in index["files"]:
        target = staging / item["name"]
        target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        count = 0
        digest = hashlib.sha256()
        with bundle.open(item["name"]) as source, target.open("xb") as out:
          os.chmod(target, 0o600)
          while chunk := source.read(CHUNK_BYTES):
            count += len(chunk)
            if count > item["size"]:
              raise TransferError("artifact exceeds indexed size")
            digest.update(chunk)
            out.write(chunk)
          out.flush()
          os.fsync(out.fileno())
        if count != item["size"] or digest.hexdigest() != item["sha256"]:
          raise TransferError("artifact hash mismatch")
      _manifest_consistency(staging, index)
      index_path = staging / INDEX_NAME
      with index_path.open("xb") as out:
        os.chmod(index_path, 0o600)
        out.write(raw)
        out.flush()
        os.fsync(out.fileno())
      _unchanged(Path(archive), stream, before)
      target = destination / index["transfer_id"]
      duplicate = target.exists() or target.is_symlink()
      if duplicate:
        _existing_matches(target, raw, index)
      else:
        # Serializes this transfer ID even when callers use different processes.
        lock = destination / f".dk-import-{index['transfer_id']}.lock"
        try:
          lock_fd = os.open(lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError as exc:
          raise TransferError("this transfer is already being imported") from exc
        try:
          os.close(lock_fd)
          if target.exists() or target.is_symlink():
            _existing_matches(target, raw, index)
            duplicate = True
          else:
            staging.rename(target)
            staging = None
        finally:
          lock.unlink()
      return {"transfer_id": index["transfer_id"], "capture_count": len(index["captures"]),
              "partial_count": sum(item["status"] == "partial" for item in index["captures"]),
              "total_bytes": total, "destination": str(target), "duplicate": duplicate}
  except (OSError, zipfile.BadZipFile, zipfile.LargeZipFile, RuntimeError, EOFError) as exc:
    raise TransferError("bundle import failed") from exc
  finally:
    if staging is not None:
      shutil.rmtree(staging)
