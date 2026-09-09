"""Bounded local-only retention of passive DK diagnostic candidate windows.

This process never publishes CAN, changes Params, or uploads anything. It copies
only closed full rlogs. Missing context and failed copies remain explicit in the
manifest; a candidate is not a diagnosis or an ECU acknowledgement.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import time
import uuid
from pathlib import Path

TOPICS = frozenset(("resume", "engage_warning", "curve", "unwind", "braking"))
MAX_CAPTURES = 10
MAX_PER_TOPIC = 2
MAX_BYTES = 1024 ** 3
MAX_FILE_BYTES = 256 * 1024 ** 2
MAX_RECORD_BYTES = 64 * 1024
TTL_SECONDS = 7 * 86400
TOPIC_COOLDOWN = 120
FINALIZE_AFTER = 180
MIN_FREE_BYTES = 5 * 1024 ** 3
CAPTURE_ID = re.compile(r"^[0-9a-f]{32}$")
ROUTE_ID = re.compile(r"^[A-Za-z0-9_-]{1,100}$")
ARTIFACT_NAME = re.compile(r"^[0-9]+-rlog(?:\.zst|\.bz2)?$")


def diagnostics_root() -> Path:
  from openpilot.system.hardware.hw import Paths
  return Path(Paths.log_root()).parent / "dk-diagnostics"


def parse_record(raw: str | dict) -> dict | None:
  try:
    if isinstance(raw, str):
      if len(raw) > MAX_RECORD_BYTES * 2:
        return None
      raw = json.loads(raw)
    record = raw.get("msg", raw)
    if isinstance(record, dict) and record.get("event") == "dk_vehicle_diag" and record.get("schema") == 1:
      if len(json.dumps(record, allow_nan=False)) <= MAX_RECORD_BYTES:
        return record
  except (TypeError, ValueError, AttributeError, RecursionError):
    pass
  return None


def read_manifest(directory: Path) -> dict | None:
  try:
    path = directory / "manifest.json"
    if directory.is_symlink() or path.is_symlink() or path.stat().st_size > MAX_RECORD_BYTES * 3:
      return None
    obj = json.loads(path.read_text())
    if (obj.get("schema") == 1 and obj.get("capture_id") == directory.name
        and CAPTURE_ID.fullmatch(directory.name) and ROUTE_ID.fullmatch(obj.get("route", ""))
        and isinstance(obj.get("created_at"), (int, float))
        and math.isfinite(obj["created_at"])
        and isinstance(obj.get("expires_at"), (int, float)) and math.isfinite(obj["expires_at"])
        and isinstance(obj.get("center_segment"), int) and obj["center_segment"] >= 0
        and obj.get("status") in ("pending", "ready", "partial")
        and isinstance(obj.get("files"), list) and len(obj["files"]) <= 3
        and all(isinstance(item, dict) and ARTIFACT_NAME.fullmatch(item.get("name", ""))
                and isinstance(item.get("segment_index"), int) and item["segment_index"] >= 0
                for item in obj["files"])
        and isinstance(obj.get("topics"), list) and all(isinstance(t, str) and t in TOPICS for t in obj["topics"])
        and isinstance(obj.get("missing"), list) and len(obj["missing"]) <= 3):
      return obj
  except (OSError, ValueError, TypeError, AttributeError, RecursionError):
    pass
  return None


def write_manifest(directory: Path, manifest: dict) -> None:
  temporary = directory / "manifest.json.tmp"
  with temporary.open("w") as stream:
    json.dump(manifest, stream, ensure_ascii=False, allow_nan=False)
    stream.flush()
    os.fsync(stream.fileno())
  temporary.replace(directory / "manifest.json")


class CaptureStore:
  def __init__(self, log_root: Path, root: Path, *, max_bytes=MAX_BYTES, min_free_bytes=MIN_FREE_BYTES):
    self.log_root = Path(log_root)
    self.root = Path(root)
    self.max_bytes = max_bytes
    self.min_free_bytes = min_free_bytes
    if self.root.is_symlink():
      raise ValueError("diagnostic root must not be a symlink")
    self.root.mkdir(parents=True, exist_ok=True)
    self.session = None
    self.last_topic = {}
    self.source_stats = {}
    self.pending_seen = {}

  def directories(self):
    return [directory for directory in self.root.iterdir()
            if CAPTURE_ID.fullmatch(directory.name) and directory.is_dir() and not directory.is_symlink()]

  def entries(self):
    entries = []
    for directory in self.directories():
      manifest = read_manifest(directory)
      if manifest is not None:
        entries.append((directory, manifest))
    return sorted(entries, key=lambda item: item[1]["created_at"])

  def size_bytes(self) -> int:
    # Account for incomplete and malformed capture directories too, so failed
    # writes cannot evade the byte budget. Never follow a link outside our root.
    total = 0
    for directory in self.root.iterdir():
      if not CAPTURE_ID.fullmatch(directory.name) or directory.is_symlink() or not directory.is_dir():
        continue
      for path in directory.iterdir():
        if path.is_file() and not path.is_symlink():
          total += path.stat().st_size
    return total

  def _remove(self, directory: Path):
    if directory.parent != self.root or not CAPTURE_ID.fullmatch(directory.name) or directory.is_symlink():
      raise ValueError("invalid diagnostic deletion target")
    shutil.rmtree(directory)

  def prune(self, now: float, *, max_captures=MAX_CAPTURES):
    # A crash before the first manifest rename must not strand an uncounted
    # directory. Include malformed/orphan UUID directories using their mtime.
    entries = []
    for directory in self.directories():
      manifest = read_manifest(directory)
      created = manifest["created_at"] if manifest else directory.stat().st_mtime
      if now - created > TTL_SECONDS:
        self._remove(directory)
      else:
        entries.append((directory, created))
    entries.sort(key=lambda item: item[1])
    while entries and (len(entries) > max_captures or self.size_bytes() > self.max_bytes):
      self._remove(entries.pop(0)[0])
    kept = {directory.name for directory, _ in entries}
    self.pending_seen = {key: value for key, value in self.pending_seen.items() if key in kept}

  def accept(self, record: dict, route: str, now: float) -> str | None:
    record = parse_record(record)
    if record is None:
      return None
    if record.get("kind") == "session":
      self.session = record
      return None
    if record.get("kind") != "sample" or not ROUTE_ID.fullmatch(route or ""):
      return None
    raw_topics = record.get("topics")
    if not isinstance(raw_topics, list) or not all(isinstance(t, str) for t in raw_topics):
      return None
    topics = sorted(set(raw_topics) & TOPICS)
    if not topics:
      return None
    entries = self.entries()
    for _, manifest in entries:
      for topic in manifest["topics"]:
        self.last_topic[topic] = max(self.last_topic.get(topic, 0), manifest["created_at"])
    topics = [t for t in topics if now - self.last_topic.get(t, -TOPIC_COOLDOWN) >= TOPIC_COOLDOWN]
    if not topics:
      return None
    candidates = []
    for path in self.log_root.glob(f"{route}--*"):
      suffix = path.name[len(route) + 2:]
      if suffix.isdigit() and path.is_dir() and not path.is_symlink():
        candidates.append(int(suffix))
    if not candidates:
      return None
    self.prune(now)
    entries = self.entries()
    # Keep at most two candidate windows per topic and ten in total. Eviction
    # only removes our copies, never original loggerd data or driver bookmarks.
    for topic in topics:
      matches = [entry for entry in entries if topic in entry[1]["topics"]]
      while len(matches) >= MAX_PER_TOPIC:
        old = matches.pop(0)
        self._remove(old[0])
        entries.remove(old)
    while len(entries) >= MAX_CAPTURES:
      self._remove(entries.pop(0)[0])
    self.prune(now, max_captures=MAX_CAPTURES - 1)
    if self.size_bytes() + MAX_RECORD_BYTES * 3 > self.max_bytes:
      return None
    capture_id = uuid.uuid4().hex
    directory = self.root / capture_id
    directory.mkdir(mode=0o700)
    manifest = {
      "schema": 1, "capture_id": capture_id, "status": "pending", "created_at": now,
      "expires_at": now + TTL_SECONDS, "route": route, "center_segment": max(candidates),
      "segment_selection": "latest_logger_segment_at_event_receipt_with_previous_and_next",
      "topics": topics, "event": record, "session": self.session, "files": [], "missing": [],
      "interpretation": "candidate_observation_not_diagnosis", "network_upload": False,
    }
    write_manifest(directory, manifest)
    for topic in topics:
      self.last_topic[topic] = now
    return capture_id

  def _copy(self, source: Path, target: Path, *, budget: int) -> dict:
    # Existing .part files are included in the budget; a retry can remove only
    # its own partial copy before opening a new one.
    temporary = target.with_name(target.name + ".part")
    if temporary.is_symlink() or source.is_symlink() or source.parent.is_symlink():
      raise ValueError("symlink not allowed")
    if temporary.exists():
      temporary.unlink()
    fd = os.open(source, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    try:
      size = os.fstat(fd).st_size
      if not 0 < size <= min(MAX_FILE_BYTES, budget):
        raise ValueError("file or capture budget exceeded")
      if shutil.disk_usage(self.root).free - size < self.min_free_bytes:
        raise ValueError("insufficient free space")
      digest = hashlib.sha256()
      copied = 0
      with os.fdopen(fd, "rb", closefd=False) as src, temporary.open("wb") as dst:
        while chunk := src.read(1024 * 1024):
          copied += len(chunk)
          if copied > size:
            raise ValueError("source changed during copy")
          digest.update(chunk)
          dst.write(chunk)
        dst.flush()
        os.fsync(dst.fileno())
      if copied != size or os.fstat(fd).st_size != size or (source.parent / "rlog.lock").exists():
        raise ValueError("source not finalized")
      temporary.replace(target)
      return {"name": target.name, "bytes": copied, "sha256": digest.hexdigest()}
    except Exception:
      if temporary.exists() and not temporary.is_symlink():
        temporary.unlink()
      raise
    finally:
      os.close(fd)

  def tick(self, now: float):
    self.prune(now)
    observed_sources = set()
    for directory, manifest in self.entries():
      if manifest["status"] != "pending":
        continue
      first_tick = self.pending_seen.setdefault(directory.name, now)
      center = manifest["center_segment"]
      expected = list(range(max(0, center - 1), center + 2))
      files = {item["segment_index"]: item for item in manifest["files"]}
      missing = []
      for index in expected:
        if index in files:
          continue
        segment = f"{manifest['route']}--{index}"
        segment_path = self.log_root / segment
        if segment_path.is_symlink() or (segment_path / "rlog.lock").exists():
          missing.append({"segment": segment, "reason": "not_finalized"})
          continue
        source = next((segment_path / name for name in ("rlog.zst", "rlog.bz2", "rlog")
                       if (segment_path / name).is_file() and not (segment_path / name).is_symlink()), None)
        if source is None:
          missing.append({"segment": segment, "reason": "not_available"})
          continue
        # loggerd removes rlog.lock just before closing its compression writer.
        # Wait for size/mtime to be unchanged across ticks, not merely unlocked.
        observed_sources.add(source)
        stat = source.stat()
        signature = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
        previous = self.source_stats.get(source)
        if previous is None or previous[0] != signature:
          self.source_stats[source] = (signature, now)
          missing.append({"segment": segment, "reason": "waiting_for_stable_file"})
          continue
        if now - previous[1] < 5:
          missing.append({"segment": segment, "reason": "waiting_for_stable_file"})
          continue
        name = f"{index}-{source.name}"
        try:
          info = self._copy(source, directory / name, budget=self.max_bytes - self.size_bytes() - MAX_RECORD_BYTES)
          files[index] = {**info, "segment": segment, "segment_index": index}
        except (OSError, ValueError) as exc:
          missing.append({"segment": segment, "reason": type(exc).__name__})
      manifest["files"] = list(files.values())
      manifest["missing"] = missing
      if not missing:
        manifest["status"] = "ready"
      elif now - manifest["created_at"] >= FINALIZE_AFTER and now - first_tick >= 5:
        manifest["status"] = "partial"
      write_manifest(directory, manifest)
    self.source_stats = {key: value for key, value in self.source_stats.items() if key in observed_sources}
    self.prune(now)


def main():
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.system.hardware.hw import Paths

  try:
    os.nice(10)
  except OSError:
    pass
  params = Params()
  store = CaptureStore(Path(Paths.log_root()), diagnostics_root())
  sock = messaging.sub_sock("logMessage", conflate=False, timeout=100)
  last_tick = 0.0
  while True:
    try:
      msg = messaging.recv_one(sock)
      if msg is not None:
        record = parse_record(msg.logMessage)
        if record is not None:
          route = params.get("CurrentRoute") or b""
          if isinstance(route, bytes):
            route = route.decode("utf-8", errors="replace")
          store.accept(record, route, time.time())  # noqa: TID251 - persisted expiry must survive reboots
      if time.monotonic() - last_tick >= 5.0:
        store.tick(time.time())  # noqa: TID251 - persisted expiry must survive reboots
        last_tick = time.monotonic()
    except Exception as exc:
      # Independent best-effort recorder: failure cannot affect card/controlsd.
      # Do not recursively emit a diagnostic event into our own subscription.
      print(f"dk_diagnosticsd: {type(exc).__name__}", flush=True)
      time.sleep(1)


if __name__ == "__main__":
  main()
