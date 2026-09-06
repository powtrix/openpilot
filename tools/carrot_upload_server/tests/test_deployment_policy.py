import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_compose_keeps_receiver_on_one_internal_loopback_only_network():
  compose = yaml.safe_load((ROOT / "compose.dsm.yml").read_text())
  service = compose["services"]["dk-upload"]
  network = compose["networks"]["dk-upload-internal"]

  assert service["platform"] == "linux/amd64"
  assert service["ports"] == ["127.0.0.1:18080:8080"]
  assert service["networks"] == ["dk-upload-internal"]
  assert network == {
    "name": "dk-upload-internal",
    "driver": "bridge",
    "internal": True,
    "labels": {"dk.openpilot.receiver-network": "true"},
  }


def test_dsm_deploy_verifies_network_policy_ingress_and_blocked_egress():
  deploy = (ROOT / "deploy_dsm.sh").read_text()
  assert "--network \"$NETWORK\"" in deploy
  assert "--publish 127.0.0.1:18080:8080" in deploy
  assert "{{.Driver}}|{{.Internal}}" in deploy
  assert "{{len .NetworkSettings.Networks}}" in deploy
  assert "{{.Architecture}}" in deploy
  assert 'socket.create_connection(("1.1.1.1", 443), timeout=2)' in deploy


@pytest.mark.parametrize("script", ["deploy_dsm.sh", "verify_dsm_proxy.sh"])
def test_dsm_shell_assets_parse(script: str):
  subprocess.run(["sh", "-n", str(ROOT / script)], check=True)


def test_receiver_dependencies_are_exact_wheel_hash_locked():
  requirement_lines = [
    line.strip()
    for line in (ROOT / "requirements.txt").read_text().splitlines()
    if line.strip() and not line.lstrip().startswith("#")
  ]
  locked = re.compile(
    r"[A-Za-z0-9_-]+==[A-Za-z0-9_.+-]+ --hash=sha256:[0-9a-f]{64}",
  )
  assert len(requirement_lines) == 13
  assert len(set(requirement_lines)) == len(requirement_lines)
  assert all(locked.fullmatch(line) for line in requirement_lines)

  dockerfile = (ROOT / "Dockerfile").read_text()
  assert "--only-binary=:all:" in dockerfile
  assert "--require-hashes" in dockerfile


def test_proxy_verifier_uses_dedicated_nas_port_and_standard_public_https():
  verifier = (ROOT / "verify_dsm_proxy.sh").read_text()
  assert "HOST=adot.synology.me" in verifier
  assert "SOURCE_PORT=18443" in verifier
  assert '--resolve "$HOST:$SOURCE_PORT:$NAS_IP"' in verifier
  assert 'BASE_URL="https://$HOST"' in verifier
  for blocked_port in (18080, 18443, 5000, 5001):
    assert f":{blocked_port}/" in verifier


def test_compose_cli_accepts_the_hardened_project_when_available():
  docker = shutil.which("docker")
  if docker is None:
    pytest.skip("Docker CLI is unavailable on this workstation")
  environment = {
    **os.environ,
    "DK_UPLOAD_ALLOWED_DEVICE_ID": "0123456789abcdef",
    "DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256": "a" * 64,
  }
  subprocess.run(
    [docker, "compose", "-f", str(ROOT / "compose.dsm.yml"), "config", "--quiet"],
    cwd=ROOT,
    env=environment,
    check=True,
  )
