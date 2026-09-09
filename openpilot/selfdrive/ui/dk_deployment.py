"""Read the installed DK release label without consulting Git or the network."""

import json
from datetime import datetime
from pathlib import Path

from openpilot.common.basedir import BASEDIR


DK_RELEASE_PATH = Path(BASEDIR) / "dk_release.json"
DK_RELEASE_MAX_BYTES = 4096


def load_dk_deployment_text(branch: str | bytes | None, path: Path = DK_RELEASE_PATH) -> str:
  if isinstance(branch, bytes):
    try:
      branch = branch.decode("utf-8")
    except UnicodeDecodeError:
      return ""
  if not isinstance(branch, str) or branch.strip() != "dkcarrot-wip":
    return ""

  try:
    with path.open("rb") as metadata_file:
      raw = metadata_file.read(DK_RELEASE_MAX_BYTES + 1)
    if len(raw) > DK_RELEASE_MAX_BYTES:
      return ""
    metadata = json.loads(raw)
    if not isinstance(metadata, dict) or type(metadata.get("schema")) is not int or metadata["schema"] != 1:
      return ""
    if type(metadata.get("diagnostics_version")) is not int or metadata["diagnostics_version"] < 1:
      return ""
    deployed_at = metadata.get("deployed_at")
    if not isinstance(deployed_at, str):
      return ""
    timestamp_format = "%Y-%m-%d %H:%M KST"
    deployed = datetime.strptime(deployed_at, timestamp_format)
    if deployed.strftime(timestamp_format) != deployed_at:
      return ""
  except (OSError, ValueError, UnicodeError, RecursionError):
    return ""

  # A deployment date is not the device's installation date or Git commit date.
  return f"DK 배포 {deployed:%Y-%m-%d}"
