import asyncio
import base64
import hashlib
import json
import os
import sqlite3
import threading
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, padding, rsa
import pytest

from opendbc.car.hyundai.values import CAR, HyundaiFlags

from .. import server as receiver_server
from ..server import (
  DEVICE_AUTH_VERSION,
  DEVICE_PROOF_DOMAIN,
  RECEIPT_VERSION,
  VALIDATION_NAMESPACE,
  Config,
  DeviceProofVerificationRejected,
  DeviceProofVerificationUnavailable,
  PinnedDeviceProofVerifier,
  UploadService,
  create_app,
)
from openpilot.selfdrive.carrot import web_upload as client_web_upload
from openpilot.selfdrive.carrot.web_upload import (
  validation_manifest_sha256 as client_validation_manifest_sha256,
  validation_receipt_id as client_validation_receipt_id,
)
from openpilot.selfdrive.carrot.server.services.validation_auto_upload import (
  _sanitize_capture_metadata as client_sanitize_capture_metadata,
)


DEVICE = "0123456789abcdef"
OTHER_DEVICE = "fedcba9876543210"
CLIENT_IP = "203.0.113.40"
CAPTURE = "0123456789abcdef0123456789abcdef"
RSA_PRIVATE_KEY = rsa.generate_private_key(public_exponent=65537, key_size=2048)
EC_PRIVATE_KEY = ec.generate_private_key(ec.SECP256R1())


def _public_key_pem(private_key: Any) -> str:
  return private_key.public_key().public_bytes(
    serialization.Encoding.PEM,
    serialization.PublicFormat.SubjectPublicKeyInfo,
  ).decode("ascii")


def _public_key_fingerprint(private_key: Any) -> str:
  public_der = private_key.public_key().public_bytes(
    serialization.Encoding.DER,
    serialization.PublicFormat.SubjectPublicKeyInfo,
  )
  return hashlib.sha256(public_der).hexdigest()


def config(tmp_path: Path, *, quota: int = 1024 * 1024) -> Config:
  return Config(
    storage_root=tmp_path / "uploads",
    db_path=tmp_path / "state" / "uploads.sqlite3",
    allowed_device_ids=frozenset({DEVICE}),
    allowed_device_public_key_sha256={DEVICE: _public_key_fingerprint(RSA_PRIVATE_KEY)},
    legacy_uploads_enabled=True,
    daily_device_quota=quota,
    daily_ip_quota=quota * 4,
    max_file_bytes=1024 * 1024,
    max_tmux_bytes=1024 * 1024,
    min_free_bytes=0,
    validation_free_space_reserve_bytes=0,
    session_ttl_seconds=600,
    concurrent_per_device=3,
    concurrent_global=16,
    validation_challenge_ttl_seconds=300,
    validation_session_ttl_seconds=300,
    validation_audience="https://uploads.example.test/api/v1/validation",
  )


class FakeVerifier:
  def __init__(self, *, result: str | None = None, error: Exception | None = None):
    self.result = result
    self.error = error
    self.calls: list[dict[str, Any]] = []

  async def verify(self, **kwargs: Any) -> str:
    self.calls.append(kwargs)
    if self.error is not None:
      raise self.error
    return self.result or str(kwargs["expected_device_id"])


def _proof(
  challenge: dict[str, Any],
  *,
  private_key: Any = RSA_PRIVATE_KEY,
  algorithm: str = "RS256",
  device: str = DEVICE,
  challenge_id: str | None = None,
  nonce: str | None = None,
  audience: str | None = None,
) -> dict[str, str]:
  message = DEVICE_PROOF_DOMAIN + b"\0".join(value.encode("utf-8") for value in (
    device,
    challenge_id or str(challenge["challengeId"]),
    nonce or str(challenge["nonce"]),
    audience or str(challenge["audience"]),
  ))
  if algorithm == "RS256":
    signature = private_key.sign(message, padding.PKCS1v15(), hashes.SHA256())
  else:
    signature = private_key.sign(message, ec.ECDSA(hashes.SHA256()))
  return {
    "deviceKeyAlgorithm": algorithm,
    "devicePublicKey": _public_key_pem(private_key),
    "deviceProof": base64.urlsafe_b64encode(signature).rstrip(b"=").decode("ascii"),
  }


async def _challenge(client: TestClient, *, device: str = DEVICE) -> dict[str, Any]:
  response = await client.post(
    "/api/v1/validation/challenge",
    json={"deviceId": device},
    headers={"X-Forwarded-For": CLIENT_IP},
  )
  assert response.status == 200, await response.text()
  return await response.json()


async def _validation_session(
  client: TestClient,
  challenge: dict[str, Any],
  *,
  proof: dict[str, str] | None = None,
) -> tuple[int, dict[str, Any]]:
  response = await client.post(
    "/api/v1/validation/session",
    json={
      "deviceId": DEVICE,
      "challengeId": challenge["challengeId"],
      **(proof or _proof(challenge)),
      "carName": "KIA CARNIVAL 4TH GEN",
      "branch": "carrot-wip",
    },
    headers={"X-Forwarded-For": CLIENT_IP},
  )
  return response.status, await response.json()


async def _token(client: TestClient) -> str:
  status, body = await _validation_session(client, await _challenge(client))
  assert status == 200, body
  return str(body["token"])


def _auth(token: str) -> dict[str, str]:
  return {"Authorization": f"Bearer {token}", "X-Forwarded-For": CLIENT_IP}


def _file_headers(token: str, content: bytes, *, sha256: str | None = None) -> dict[str, str]:
  return {
    **_auth(token),
    "X-File-Size": str(len(content)),
    "X-Content-SHA256": sha256 or hashlib.sha256(content).hexdigest(),
  }


def _capture_metadata(
  files: list[dict[str, Any]],
  *,
  condition: str = "standstill_on",
  capture: str = CAPTURE,
) -> dict[str, Any]:
  segments = sorted(str(item["segment"]) for item in files)
  route, _, anchor_text = segments[0].rpartition("--")
  trigger = {
    "lane_offset_0": "stable_lane_control",
    "lane_offset_10": "stable_lane_control",
    "stock_scc_close_accel": "accelerating_while_closing",
    "standstill_on_no_request": "standstill_observed",
  }.get(condition, "keepalive_requested")
  raw = {
    "anchorSegment": int(anchor_text),
    "duration": 31.0,
    "trigger": trigger,
    "keepaliveRequestDelta": 1 if condition == "standstill_on" else None,
    "qualified": True if condition.startswith("standstill_") else None,
    "controllerStoppedSec": 31.0 if condition.startswith("standstill_") else None,
    "vEgo": 12.0 if condition == "stock_scc_close_accel" else None,
    "aEgo": 0.8 if condition == "stock_scc_close_accel" else None,
    "leadDRel": 18.0 if condition == "stock_scc_close_accel" else None,
    "leadVRel": -2.0 if condition == "stock_scc_close_accel" else None,
    "timeGap": 1.5 if condition == "stock_scc_close_accel" else None,
    "ttc": 9.0 if condition == "stock_scc_close_accel" else None,
    "detectedAt": 1_800_000_000,
    "settingsEpoch": 0,
    "routeSettings": {
      "Ka4StockSccStandstillRearm": 0,
      "PathOffset": 10 if condition == "lane_offset_10" else 0,
      "AdjustLaneOffset": 0,
    },
    "git": {
      "branch": "carrot-wip",
      "commit": "b" * 40,
      "dirty": False,
      "topology": {
        "carFingerprint": "KIA_CARNIVAL_4TH_GEN",
        "pcmCruise": True,
        "openpilotLongitudinalControl": False,
        "flags": (1 << 13) | (1 << 14),
        "alternativeExperience": 0,
        "safetyConfigs": [{"model": "hyundaiCanfd", "param": 0}],
        "gatePassed": True,
      },
    },
  }
  return client_sanitize_capture_metadata(
    raw,
    campaign_id="a" * 24,
    capture_id=capture,
    condition=condition,
    route=route,
    segments=segments,
  )


def test_receiver_ka4_topology_wire_constants_match_the_vehicle_gate():
  assert receiver_server.KA4_STOCK_SCC_CAR_FINGERPRINT == str(CAR.KIA_CARNIVAL_4TH_GEN)
  assert receiver_server.HYUNDAI_FLAG_CANFD_HDA2 == int(HyundaiFlags.CANFD_HDA2)
  assert receiver_server.HYUNDAI_FLAG_CAMERA_SCC == int(HyundaiFlags.CAMERA_SCC)
  assert receiver_server.HYUNDAI_FLAG_CANFD == int(HyundaiFlags.CANFD)
  assert receiver_server.HYUNDAI_FLAG_RADAR_SCC == int(HyundaiFlags.RADAR_SCC)


def _completion_body(files: list[dict[str, Any]]) -> dict[str, Any]:
  return {
    "deviceId": DEVICE,
    "captureId": CAPTURE,
    "files": files,
    "validationCapture": _capture_metadata(files),
  }


def _json_bytes(value: Any) -> bytes:
  return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def test_challenge_device_proof_session_is_one_time_and_persistent(tmp_path: Path):
  async def run():
    verifier = FakeVerifier()
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False, validation_verifier=verifier))) as client:
      challenge = await _challenge(client)
      assert challenge.keys() == {
        "ok", "challengeId", "nonce", "audience", "expiresAt", "deviceAuthVersion", "receiptVersion",
      }
      assert challenge["deviceAuthVersion"] == DEVICE_AUTH_VERSION
      assert challenge["receiptVersion"] == RECEIPT_VERSION
      duplicate_challenge = await _challenge(client)
      assert duplicate_challenge["challengeId"] == challenge["challengeId"]
      assert duplicate_challenge["nonce"] == challenge["nonce"]
      device_proof = _proof(challenge)
      status, body = await _validation_session(client, challenge, proof=device_proof)
      assert status == 200
      assert body["verifiedDeviceId"] == DEVICE
      assert body["deviceAuthVersion"] == DEVICE_AUTH_VERSION
      assert len(verifier.calls) == 1
      assert verifier.calls[0]["expected_device_id"] == DEVICE
      assert verifier.calls[0]["algorithm"] == "RS256"
      assert verifier.calls[0]["proof"] == device_proof["deviceProof"]

      replay_status, replay = await _validation_session(client, challenge, proof=device_proof)
      assert replay_status == 409
      assert "consumed" in replay["error"]

    # Both the consumed challenge and issued purpose-scoped session survive a
    # process restart in the same SQLite database.
    with sqlite3.connect(cfg.db_path) as connection:
      challenge_row = connection.execute(
        "SELECT consumed_at, session_token_hash FROM validation_challenges WHERE challenge_id=?",
        (challenge["challengeId"],),
      ).fetchone()
      assert challenge_row[0] is not None
      assert challenge_row[1]
      purpose, device_key_sha256 = connection.execute(
        "SELECT purpose, device_key_sha256 FROM sessions WHERE token_hash=?", (challenge_row[1],),
      ).fetchone()
      assert purpose == "validation"
      assert device_key_sha256 == _public_key_fingerprint(RSA_PRIVATE_KEY)
      assert connection.execute("PRAGMA synchronous").fetchone()[0] == 2

  asyncio.run(run())


def test_allowlist_revocation_blocks_validation_entrypoints_and_existing_session(tmp_path: Path):
  async def run():
    verifier = FakeVerifier()
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      token = await _token(client)
      pending_challenge = await _challenge(client)

    revoked = replace(cfg, allowed_device_ids=frozenset({OTHER_DEVICE}))
    async with TestClient(TestServer(create_app(
      revoked, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert challenge.status == 403

      validation_session = await client.post(
        "/api/v1/validation/session",
        json={
          "deviceId": DEVICE,
          "challengeId": pending_challenge["challengeId"],
          **_proof(pending_challenge),
        },
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert validation_session.status == 403

      content = b"must not be accepted after allowlist revocation"
      upload = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      complete = await client.post(
        "/api/v1/validation/complete",
        json=_completion_body([{
          "segment": "route--0",
          "name": "rlog.zst",
          "size": len(content),
          "sha256": hashlib.sha256(content).hexdigest(),
        }]),
        headers=_auth(token),
      )
      assert upload.status == 403
      assert complete.status == 403
      for response in (challenge, validation_session, upload, complete):
        assert "not allowed" in (await response.json())["error"]

    assert len(verifier.calls) == 1
    assert verifier.calls[0]["expected_device_id"] == DEVICE
    assert not (tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE).exists()

  asyncio.run(run())


def test_device_key_pin_rotation_revokes_existing_validation_session(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)

    rotated = replace(cfg, allowed_device_public_key_sha256={DEVICE: "2" * 64})
    async with TestClient(TestServer(create_app(
      rotated, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      content = b"must not survive a receiver pin change"
      upload = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert upload.status == 403
      assert "key pin changed" in (await upload.json())["error"]

  asyncio.run(run())


def test_mismatched_identity_timeout_and_failed_verification_do_not_consume(tmp_path: Path):
  async def run():
    verifier = FakeVerifier(result=OTHER_DEVICE)
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      wrong_claim_challenge = await _challenge(client)
      status, _body = await _validation_session(client, wrong_claim_challenge)
      assert status == 401

      # Definitive authentication failures cool the challenge down before any
      # further proof verification is allowed.
      status, _body = await _validation_session(client, wrong_claim_challenge)
      assert status == 429
      assert len(verifier.calls) == 1
      with sqlite3.connect(cfg.db_path) as connection:
        connection.execute(
          "UPDATE validation_challenges SET retry_after=0 WHERE challenge_id=?",
          (wrong_claim_challenge["challengeId"],),
        )

      verifier.result = None
      verifier.error = DeviceProofVerificationUnavailable("timeout")
      status, _body = await _validation_session(client, wrong_claim_challenge)
      assert status == 503

      # A verifier outage or rejected identity must not burn the one-time
      # challenge. A later authenticated attempt can consume it exactly once.
      status, _body = await _validation_session(client, wrong_claim_challenge)
      assert status == 429
      with sqlite3.connect(cfg.db_path) as connection:
        connection.execute(
          "UPDATE validation_challenges SET retry_after=0 WHERE challenge_id=?",
          (wrong_claim_challenge["challengeId"],),
        )
      verifier.error = None
      status, body = await _validation_session(client, wrong_claim_challenge)
      assert status == 200, body

      expired_challenge = await _challenge(client)
      with sqlite3.connect(cfg.db_path) as connection:
        connection.execute(
          "UPDATE validation_challenges SET expires_at=1 WHERE challenge_id=?",
          (expired_challenge["challengeId"],),
        )
      status, _body = await _validation_session(client, expired_challenge)
      assert status == 401

  asyncio.run(run())


def test_pinned_device_proof_verifier_accepts_rsa_and_es256_and_rejects_mismatches():
  async def run():
    verifier = PinnedDeviceProofVerifier()
    challenge = {
      "challengeId": "challenge_01234567890123456789",
      "nonce": "nonce_012345678901234567890123",
      "audience": "https://uploads.example.test/api/v1/validation",
    }
    assert UploadService._validation_device_proof_message(
      DEVICE,
      challenge["challengeId"],
      challenge["nonce"],
      challenge["audience"],
    ) == (
      b"dk-carrot-validation-device-proof-v2\0"
      + DEVICE.encode()
      + b"\0challenge_01234567890123456789"
      + b"\0nonce_012345678901234567890123"
      + b"\0https://uploads.example.test/api/v1/validation"
    )

    async def verify(private_key: Any, algorithm: str, **overrides: Any) -> str:
      signed = _proof(challenge, private_key=private_key, algorithm=algorithm)
      message = DEVICE_PROOF_DOMAIN + b"\0".join(value.encode() for value in (
        DEVICE, challenge["challengeId"], challenge["nonce"], challenge["audience"],
      ))
      return await verifier.verify(
        algorithm=algorithm,
        public_key_pem=signed["devicePublicKey"],
        proof=signed["deviceProof"],
        message=message,
        expected_fingerprint=_public_key_fingerprint(private_key),
        expected_device_id=DEVICE,
        **overrides,
      )

    assert await verify(RSA_PRIVATE_KEY, "RS256") == DEVICE
    assert await verify(EC_PRIVATE_KEY, "ES256") == DEVICE

    rsa_proof = _proof(challenge)
    canonical = DEVICE_PROOF_DOMAIN + b"\0".join(value.encode() for value in (
      DEVICE, challenge["challengeId"], challenge["nonce"], challenge["audience"],
    ))
    common = {
      "algorithm": "RS256",
      "public_key_pem": rsa_proof["devicePublicKey"],
      "proof": rsa_proof["deviceProof"],
      "message": canonical,
      "expected_fingerprint": _public_key_fingerprint(RSA_PRIVATE_KEY),
      "expected_device_id": DEVICE,
    }
    with pytest.raises(DeviceProofVerificationRejected, match="receiver pin"):
      await verifier.verify(**{**common, "expected_fingerprint": "0" * 64})
    with pytest.raises(DeviceProofVerificationRejected, match="signature"):
      await verifier.verify(**{**common, "message": canonical + b"tampered"})
    with pytest.raises(DeviceProofVerificationRejected, match="P-256"):
      await verifier.verify(**{**common, "algorithm": "ES256"})
    with pytest.raises(DeviceProofVerificationRejected, match="base64url"):
      await verifier.verify(**{**common, "proof": rsa_proof["deviceProof"] + "="})
    pkcs1_pem = RSA_PRIVATE_KEY.public_key().public_bytes(
      serialization.Encoding.PEM,
      serialization.PublicFormat.PKCS1,
    ).decode("ascii")
    with pytest.raises(DeviceProofVerificationRejected, match="PEM"):
      await verifier.verify(**{**common, "public_key_pem": pkcs1_pem})

  asyncio.run(run())


def test_default_validation_session_verifies_the_pinned_proof_locally(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(cfg, start_cleanup=False))) as client:
      challenge = await _challenge(client)
      status, body = await _validation_session(client, challenge)
      assert status == 200, body
      assert body["verifiedDeviceId"] == DEVICE

      tampered_challenge = await _challenge(client)
      legacy_jwt = await client.post(
        "/api/v1/validation/session",
        json={
          "deviceId": DEVICE,
          "challengeId": tampered_challenge["challengeId"],
          "identityToken": "generic.comma.jwt",
        },
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert legacy_jwt.status == 400
      assert "not accepted" in (await legacy_jwt.json())["error"]

      status, body = await _validation_session(
        client,
        tampered_challenge,
        proof=_proof(tampered_challenge, nonce="not-the-server-nonce"),
      )
      assert status == 401
      assert "proof" in body["error"]

  asyncio.run(run())


@pytest.mark.parametrize(("algorithm", "private_key"), [
  ("RS256", RSA_PRIVATE_KEY),
  ("ES256", EC_PRIVATE_KEY),
])
def test_actual_client_v2_proof_upload_and_manifest_contract(
  tmp_path: Path,
  monkeypatch,
  algorithm: str,
  private_key: Any,
):
  """Exercise the production client and receiver contract over loopback only."""
  private_pem = private_key.private_bytes(
    serialization.Encoding.PEM,
    serialization.PrivateFormat.PKCS8,
    serialization.NoEncryption(),
  ).decode("ascii")
  public_pem = _public_key_pem(private_key)
  monkeypatch.setattr(
    client_web_upload,
    "get_key_pair",
    lambda: (algorithm, private_pem, public_pem),
  )
  # The production client now requires private HTTPS. This contract test owns
  # an in-process loopback HTTP server, so bypass only the destination-policy
  # predicate while exercising the real proof/upload/receipt implementation.
  monkeypatch.setattr(client_web_upload, "is_private_validation_upload_url", lambda _: True)

  async def run():
    cfg = replace(
      config(tmp_path),
      allowed_device_public_key_sha256={DEVICE: _public_key_fingerprint(private_key)},
    )
    async with TestServer(create_app(cfg, start_cleanup=False)) as server:
      base_url = str(server.make_url("/")).rstrip("/")
      token = await client_web_upload.create_validation_upload_session(
        base_url,
        {
          "dongleId": DEVICE,
          "carName": "KIA CARNIVAL 4TH GEN",
          "branch": "dkcarrot-wip",
        },
      )
      assert token

      segment = "2026-09-06--12-34-56--0"
      content = f"actual-{algorithm}-client-rlog".encode()
      segment_dir = tmp_path / f"client-{algorithm}"
      segment_dir.mkdir()
      (segment_dir / "rlog.zst").write_bytes(content)
      files = [{
        "segment": segment,
        "name": "rlog.zst",
        "size": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
      }]
      assert await client_web_upload.upload_validation_folder_to_web(
        str(segment_dir),
        segment,
        CAPTURE,
        base_url,
        token,
        files,
      ) is True

      validation_capture = _capture_metadata(files)
      receipt = await client_web_upload.send_validation_upload_complete(
        base_url,
        token,
        {
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": files,
          "validationCapture": validation_capture,
        },
      )
      expected_manifest_sha256 = client_web_upload.validation_manifest_sha256(
        DEVICE,
        CAPTURE,
        files,
        validation_capture,
      )
      assert receipt["ok"] is True, receipt
      assert receipt["manifestSha256"] == expected_manifest_sha256
      assert receipt["receiptId"] == client_web_upload.validation_receipt_id(
        expected_manifest_sha256,
      )

      manifest_path = (
        cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "manifest.json"
      )
      assert hashlib.sha256(manifest_path.read_bytes()).hexdigest() == expected_manifest_sha256

  asyncio.run(run())


def test_identical_hardlink_orphan_recovery_charges_storage_once(tmp_path: Path):
  async def run():
    verifier = FakeVerifier()
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      token = await _token(client)
      content = b"already-atomically-published"
      target = (
        tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "route--0" / "rlog.zst"
      )
      target.parent.mkdir(parents=True)
      target.write_bytes(content)

      response = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert response.status == 200, await response.text()
      assert (await response.json())["alreadyReceived"] is True

    with sqlite3.connect(cfg.db_path) as connection:
      receipt = connection.execute(
        "SELECT size, sha256 FROM validation_files WHERE device_id=? AND capture_id=?",
        (DEVICE, CAPTURE),
      ).fetchone()
      assert receipt == (len(content), hashlib.sha256(content).hexdigest())
      usage = connection.execute(
        "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall()
      assert usage == [(len(content), 0, len(content)), (len(content), 0, len(content))]

  asyncio.run(run())


def test_concurrent_identical_validation_uploads_charge_one_storage_copy(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    content = b"same-concurrent-rlog"
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      url = f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst"
      first, second = await asyncio.gather(
        client.put(url, data=content, headers=_file_headers(token, content)),
        client.put(url, data=content, headers=_file_headers(token, content)),
      )
      assert first.status == 200, await first.text()
      assert second.status == 200, await second.text()
      assert sorted([
        (await first.json())["alreadyReceived"],
        (await second.json())["alreadyReceived"],
      ]) == [False, True]

    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [
        (2 * len(content), len(content)),
        (2 * len(content), len(content)),
      ]

  asyncio.run(run())


def test_validation_upload_records_received_before_disk_progress(tmp_path: Path, monkeypatch):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_progress = service._record_stream_progress
      progress: list[tuple[int, int, int]] = []

      async def record_progress(
        reservation: Any,
        state: Any,
        **kwargs: Any,
      ) -> None:
        progress.append((
          kwargs["received_bytes"],
          kwargs["disk_written_bytes"],
          kwargs.get("stored_bytes", 0),
        ))
        await original_progress(reservation, state, **kwargs)

      monkeypatch.setattr(service, "_record_stream_progress", record_progress)
      content = b"ordered-rlog-write"
      response = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert response.status == 200, await response.text()
      assert progress[0] == (len(content), 0, 0)
      assert progress[1] == (len(content), len(content), 0)
      assert progress[-1] == (len(content), len(content), len(content))

  asyncio.run(run())


def test_validation_receipt_insert_and_storage_accounting_are_atomic(tmp_path: Path, monkeypatch):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_progress = service._record_stream_progress
      transaction_visible = asyncio.Event()
      release_response = asyncio.Event()
      paused = False
      content = b"atomic-rlog-accounting"

      async def pause_after_transaction(
        reservation: Any,
        state: Any,
        **kwargs: Any,
      ) -> None:
        nonlocal paused
        if kwargs.get("stored_bytes") == len(content) and not paused:
          paused = True
          transaction_visible.set()
          await release_response.wait()
        await original_progress(reservation, state, **kwargs)

      monkeypatch.setattr(service, "_record_stream_progress", pause_after_transaction)
      request = asyncio.create_task(client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      ))
      await asyncio.wait_for(transaction_visible.wait(), timeout=1)
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM validation_files").fetchone()[0] == 1
        assert connection.execute(
          "SELECT received_bytes, disk_written_bytes, stored_bytes FROM upload_reservations",
        ).fetchone() == (len(content), len(content), len(content))

      release_response.set()
      response = await asyncio.wait_for(request, timeout=2)
      assert response.status == 200, await response.text()
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute(
          "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
        ).fetchall() == [
          (len(content), len(content)),
          (len(content), len(content)),
        ]

  asyncio.run(run())


def test_validation_upload_hash_size_rlog_only_and_immutable_retry(tmp_path: Path):
  async def run():
    verifier = FakeVerifier()
    async with TestClient(TestServer(create_app(
      config(tmp_path), start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      token = await _token(client)
      content = b"full-rlog-content"
      url = f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst"

      wrong_name = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/qlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert wrong_name.status == 400

      wrong_hash = await client.put(
        url,
        data=content,
        headers=_file_headers(token, content, sha256="0" * 64),
      )
      assert wrong_hash.status == 400

      wrong_size_headers = _file_headers(token, content)
      wrong_size_headers["X-File-Size"] = str(len(content) + 1)
      wrong_size = await client.put(url, data=content, headers=wrong_size_headers)
      assert wrong_size.status == 400

      upload = await client.put(url, data=content, headers=_file_headers(token, content))
      assert upload.status == 200, await upload.text()
      first_body = await upload.json()
      assert first_body["alreadyReceived"] is False
      target = tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "route--0" / "rlog.zst"
      assert target.read_bytes() == content

      retry = await client.put(url, data=content, headers=_file_headers(token, content))
      assert retry.status == 200
      assert (await retry.json())["alreadyReceived"] is True

      conflict_content = b"other-rlog-content"
      conflict = await client.put(url, data=conflict_content, headers=_file_headers(token, conflict_content))
      assert conflict.status == 409
      assert target.read_bytes() == content
      assert not list(target.parent.glob("*.part"))

      # Tokens issued by unauthenticated legacy v1 cannot write the protected
      # validation namespace, and validation tokens cannot use legacy routes.
      legacy_session = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam", "carName": "TEST"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      legacy_token = (await legacy_session.json())["token"]
      legacy_on_validation = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--1/rlog.zst",
        data=content,
        headers=_file_headers(legacy_token, content),
      )
      assert legacy_on_validation.status == 403
      validation_on_legacy = await client.put(
        f"/api/v1/upload/{DEVICE}/route--1/rlog.zst",
        data=content,
        headers={**_file_headers(token, content), "X-File-Size": str(len(content))},
      )
      assert validation_on_legacy.status == 403

  asyncio.run(run())


def test_existing_validation_upload_flood_is_rejected_before_second_hash(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(config(tmp_path), validation_concurrent_per_device=1)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      content = b"immutable-rlog"
      url = f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst"
      uploaded = await client.put(url, data=content, headers=_file_headers(token, content))
      assert uploaded.status == 200, await uploaded.text()

      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_hash = service._hash_file_bounded
      hash_started = asyncio.Event()
      release_hash = asyncio.Event()
      hash_calls = 0

      async def blocking_hash(path: Path, **kwargs: Any) -> tuple[int, str]:
        nonlocal hash_calls
        hash_calls += 1
        if hash_calls == 1:
          hash_started.set()
          await release_hash.wait()
        return await original_hash(path, **kwargs)

      monkeypatch.setattr(service, "_hash_file_bounded", blocking_hash)
      first = asyncio.create_task(client.put(url, data=content, headers=_file_headers(token, content)))
      await asyncio.wait_for(hash_started.wait(), timeout=1)
      try:
        second = await asyncio.wait_for(
          client.put(url, data=content, headers=_file_headers(token, content)),
          timeout=1,
        )
        assert second.status == 429, await second.text()
        assert "concurrent" in (await second.json())["error"]
        # The rejected request must not queue behind the bounded hash worker.
        assert hash_calls == 1
      finally:
        release_hash.set()

      first_response = await asyncio.wait_for(first, timeout=2)
      assert first_response.status == 200, await first_response.text()

  asyncio.run(run())


def test_cancelled_hash_holds_worker_and_validation_capacity_until_thread_exits(
  tmp_path: Path,
  monkeypatch,
):
  async def run():
    cfg = replace(
      config(tmp_path),
      validation_hash_concurrent=1,
      validation_concurrent_per_device=1,
      validation_concurrent_global=1,
    )
    service = UploadService(cfg, validation_verifier=FakeVerifier())
    first_path = tmp_path / "first-rlog"
    second_path = tmp_path / "second-rlog"
    first_path.write_bytes(b"first")
    second_path.write_bytes(b"second")
    first_started = threading.Event()
    second_started = threading.Event()
    release_first = threading.Event()
    original_hash = service._hash_file
    loop = asyncio.get_running_loop()
    orphan_errors: list[dict[str, Any]] = []
    previous_exception_handler = loop.get_exception_handler()
    loop.set_exception_handler(lambda _loop, context: orphan_errors.append(context))

    def blocking_hash(path: Path) -> tuple[int, str]:
      if path == first_path:
        first_started.set()
        assert release_first.wait(timeout=5)
        raise OSError("late failure from cancelled hash worker")
      else:
        second_started.set()
      return original_hash(path)

    try:
      monkeypatch.setattr(service, "_hash_file", blocking_hash)
      await service._enter_upload(DEVICE, validation=True)
      first = asyncio.create_task(service._hash_file_bounded(first_path, device=DEVICE))
      assert await asyncio.wait_for(asyncio.to_thread(first_started.wait, 1), timeout=2)
      first.cancel()
      with pytest.raises(asyncio.CancelledError):
        await first
      assert not release_first.is_set()

      # Mirror the request handler's finally block. The detached synchronous
      # hash retains one replacement operation slot until its thread exits.
      await service._leave_upload(DEVICE, validation=True)
      assert service._validation_active_global == 1
      with pytest.raises(web.HTTPServiceUnavailable):
        await service._enter_upload(OTHER_DEVICE, validation=True)

      second = asyncio.create_task(service._hash_file_bounded(second_path, device=OTHER_DEVICE))
      await asyncio.sleep(0)
      assert service._validation_hash_semaphore.locked()
      assert service._validation_hash_semaphore._waiters
      assert not second_started.is_set()

      release_first.set()
      result = await asyncio.wait_for(second, timeout=2)
      assert result == (len(b"second"), hashlib.sha256(b"second").hexdigest())
      assert second_started.is_set()
      assert service._validation_active_global == 0
      assert not service._validation_active_by_device
      assert not orphan_errors
    finally:
      release_first.set()
      loop.set_exception_handler(previous_exception_handler)

  asyncio.run(run())


def test_validation_completion_exact_manifest_deterministic_and_immutable(tmp_path: Path):
  async def run():
    verifier = FakeVerifier()
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      token = await _token(client)
      uploads = [("route--1", b"second"), ("route--0", b"first")]
      files = []
      for segment, content in uploads:
        response = await client.put(
          f"/api/v1/validation/upload/{CAPTURE}/{segment}/rlog.zst",
          data=content,
          headers=_file_headers(token, content),
        )
        assert response.status == 200, await response.text()
        files.append({
          "segment": segment,
          "name": "rlog.zst",
          "size": len(content),
          "sha256": hashlib.sha256(content).hexdigest(),
        })

      missing = await client.post(
        "/api/v1/validation/complete",
        json={
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": files[:1],
          "validationCapture": _capture_metadata(files[:1]),
        },
        headers=_auth(token),
      )
      assert missing.status == 409

      mismatched_files = [dict(item) for item in files]
      mismatched_files[0]["sha256"] = "0" * 64
      mismatch = await client.post(
        "/api/v1/validation/complete",
        json={
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": mismatched_files,
          "validationCapture": _capture_metadata(mismatched_files),
        },
        headers=_auth(token),
      )
      assert mismatch.status == 409

      # Receipt creation re-hashes the durable files rather than trusting only
      # their recorded size. Same-size corruption must block completion.
      first_path = (
        tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "route--0" / "rlog.zst"
      )
      first_path.write_bytes(b"FIRST")
      corrupt = await client.post(
        "/api/v1/validation/complete",
        json={
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": files,
          "validationCapture": _capture_metadata(files),
        },
        headers=_auth(token),
      )
      assert corrupt.status == 409
      first_path.write_bytes(b"first")

      body = {
        "deviceId": DEVICE,
        "captureId": CAPTURE,
        "files": files,
        "validationCapture": _capture_metadata(files),
      }
      complete = await client.post("/api/v1/validation/complete", json=body, headers=_auth(token))
      assert complete.status == 200, await complete.text()
      receipt = await complete.json()
      assert receipt["verifiedDeviceId"] == DEVICE
      assert receipt["captureId"] == CAPTURE
      assert receipt["files"] == sorted(files, key=lambda item: (item["segment"], item["name"]))
      assert len(receipt["receiptId"]) == 64
      assert len(receipt["manifestSha256"]) == 64
      client_manifest_sha256 = client_validation_manifest_sha256(
        DEVICE,
        CAPTURE,
        files,
        body["validationCapture"],
      )
      assert receipt["manifestSha256"] == client_manifest_sha256
      assert receipt["receiptId"] == client_validation_receipt_id(client_manifest_sha256)

      manifest_path = tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "manifest.json"
      manifest_bytes = manifest_path.read_bytes()
      assert hashlib.sha256(manifest_bytes).hexdigest() == receipt["manifestSha256"]
      assert manifest_bytes.endswith(b"\n")

      retry = await client.post("/api/v1/validation/complete", json=body, headers=_auth(token))
      assert retry.status == 200
      assert await retry.json() == receipt

      # An idempotent completion is only acknowledged after the exact current
      # DB set and every immutable rlog are revalidated, not just manifest.json.
      first_path.write_bytes(b"FIRST")
      corrupt_retry = await client.post("/api/v1/validation/complete", json=body, headers=_auth(token))
      assert corrupt_retry.status == 409
      first_path.write_bytes(b"first")

      rogue_content = b"rogue"
      rogue_path = (
        tmp_path / "uploads" / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "route--9" / "rlog.zst"
      )
      rogue_path.parent.mkdir(parents=True)
      rogue_path.write_bytes(rogue_content)
      rogue_relative = rogue_path.relative_to((tmp_path / "uploads").resolve()).as_posix()
      with sqlite3.connect(cfg.db_path) as connection:
        connection.execute(
          """INSERT INTO validation_files(
               device_id, capture_id, segment, filename, size, sha256, relative_path, received_at
             ) VALUES(?,?,?,?,?,?,?,?)""",
          (
            DEVICE, CAPTURE, "route--9", "rlog.zst", len(rogue_content),
            hashlib.sha256(rogue_content).hexdigest(), rogue_relative,
            int(datetime.now(UTC).timestamp()),
          ),
        )
      extra_db_file = await client.post("/api/v1/validation/complete", json=body, headers=_auth(token))
      assert extra_db_file.status == 409
      with sqlite3.connect(cfg.db_path) as connection:
        connection.execute(
          "DELETE FROM validation_files WHERE device_id=? AND capture_id=? AND segment=?",
          (DEVICE, CAPTURE, "route--9"),
        )
      rogue_path.unlink()

      conflict_body = dict(body)
      conflict_body["validationCapture"] = _capture_metadata(files, condition="lane_offset_10")
      conflict = await client.post(
        "/api/v1/validation/complete", json=conflict_body, headers=_auth(token),
      )
      assert conflict.status == 409
      assert manifest_path.read_bytes() == manifest_bytes

      late_content = b"late"
      late = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--2/rlog.zst",
        data=late_content,
        headers=_file_headers(token, late_content),
      )
      assert late.status == 409

    # A crash or external deletion after the durable DB commit can be repaired
    # from the immutable manifest bytes stored in SQLite on restart.
    manifest_path.unlink()
    UploadService(cfg, validation_verifier=verifier)
    assert manifest_path.read_bytes() == manifest_bytes

  asyncio.run(run())


def test_validation_completion_charges_body_and_two_new_manifest_copies_once(tmp_path: Path):
  async def run():
    content = b"accounted-rlog"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    body = _completion_body(files)
    encoded_body = _json_bytes(body)
    cfg = config(tmp_path, quota=len(content) + 2 * len(encoded_body) - 1)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      uploaded = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert uploaded.status == 200, await uploaded.text()

      complete = await client.post(
        "/api/v1/validation/complete",
        data=encoded_body,
        headers={**_auth(token), "Content-Type": "application/json"},
      )
      assert complete.status == 200, await complete.text()
      manifest_path = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "manifest.json"
      manifest_size = len(manifest_path.read_bytes())

      with sqlite3.connect(cfg.db_path) as connection:
        usage = connection.execute(
          "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
        ).fetchall()
      assert usage == [
        (len(content) + len(encoded_body), len(content) + 2 * manifest_size),
        (len(content) + len(encoded_body), len(content) + 2 * manifest_size),
      ]

      # The retry would exceed the remaining network quota by one byte. The
      # existing manifest consumes no second storage delta and cannot be used
      # to bypass the body-byte quota.
      retry = await client.post(
        "/api/v1/validation/complete",
        data=encoded_body,
        headers={**_auth(token), "Content-Type": "application/json"},
      )
      assert retry.status == 413
      with sqlite3.connect(cfg.db_path) as connection:
        unchanged = connection.execute(
          "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
        ).fetchall()
      assert unchanged == usage

  asyncio.run(run())


def test_completion_retry_after_hardlink_before_commit_accounts_both_manifest_copies(tmp_path: Path):
  async def run():
    content = b"crash-residue-rlog"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    body = _completion_body(files)
    encoded_body = _json_bytes(body)
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      uploaded = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert uploaded.status == 200, await uploaded.text()

      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      canonical = service._canonical_json({
        "captureId": CAPTURE,
        "deviceAuthVersion": DEVICE_AUTH_VERSION,
        "deviceId": DEVICE,
        "files": files,
        "receiptVersion": RECEIPT_VERSION,
        "validationCapture": body["validationCapture"],
      })
      # This is the durable state left by a process crash after link+directory
      # fsync but before the SQLite completion transaction commits.
      manifest_path = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "manifest.json"
      manifest_path.write_bytes(canonical)
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM validation_completions").fetchone()[0] == 0

      completed = await client.post(
        "/api/v1/validation/complete",
        data=encoded_body,
        headers={**_auth(token), "Content-Type": "application/json"},
      )
      assert completed.status == 200, await completed.text()

    with sqlite3.connect(cfg.db_path) as connection:
      usage = connection.execute(
        "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall()
      assert usage == [
        (len(content) + len(encoded_body), len(content) + 2 * len(canonical)),
        (len(content) + len(encoded_body), len(content) + 2 * len(canonical)),
      ]
      assert connection.execute("SELECT COUNT(*) FROM validation_completions").fetchone()[0] == 1

  asyncio.run(run())


def test_idempotent_completion_charges_network_but_no_storage_delta(tmp_path: Path):
  async def run():
    content = b"retry-rlog"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    body_bytes = _json_bytes(_completion_body(files))
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      uploaded = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert uploaded.status == 200, await uploaded.text()
      headers = {**_auth(token), "Content-Type": "application/json"}
      first = await client.post("/api/v1/validation/complete", data=body_bytes, headers=headers)
      assert first.status == 200, await first.text()
      with sqlite3.connect(cfg.db_path) as connection:
        before = connection.execute(
          "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
        ).fetchall()

      retry = await client.post("/api/v1/validation/complete", data=body_bytes, headers=headers)
      assert retry.status == 200, await retry.text()
      with sqlite3.connect(cfg.db_path) as connection:
        after = connection.execute(
          "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
        ).fetchall()
      assert after == [(network + len(body_bytes), stored) for network, stored in before]

  asyncio.run(run())


def test_invalid_and_amplifying_completion_metadata_is_bounded_and_charged(tmp_path: Path):
  async def run():
    content = b"unreceived"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    invalid_body = _completion_body(files)
    invalid_body["validationCapture"]["git"]["topology"]["unexpected"] = {
      "nested": [["x" * 64] for _ in range(200)],
    }
    invalid_bytes = _json_bytes(invalid_body)
    assert len(invalid_bytes) < receiver_server.VALIDATION_COMPLETION_BODY_MAX
    truncated_json = b'{"deviceId":"0123456789abcdef","validationCapture":'
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      headers = {**_auth(token), "Content-Type": "application/json"}
      invalid = await client.post("/api/v1/validation/complete", data=invalid_bytes, headers=headers)
      assert invalid.status == 400
      assert "git.topology" in (await invalid.json())["error"]
      truncated = await client.post(
        "/api/v1/validation/complete",
        data=truncated_json,
        headers=headers,
      )
      assert truncated.status == 400

    with sqlite3.connect(cfg.db_path) as connection:
      usage = connection.execute(
        "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall()
      assert usage == [
        (len(invalid_bytes) + len(truncated_json), 0, 0),
        (len(invalid_bytes) + len(truncated_json), 0, 0),
      ]
      assert connection.execute("SELECT COUNT(*) FROM validation_completions").fetchone()[0] == 0
    assert not (cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE).exists()

  asyncio.run(run())


def test_validation_completion_rejects_unknown_top_level_fields_and_charges_body(tmp_path: Path):
  async def run():
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": 1,
      "sha256": hashlib.sha256(b"x").hexdigest(),
    }]
    body = _completion_body(files)
    body["unexpectedEnvelopeField"] = {"ignoredByOldReceiver": True}
    encoded = _json_bytes(body)
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      response = await client.post(
        "/api/v1/validation/complete",
        data=encoded,
        headers={**_auth(token), "Content-Type": "application/json"},
      )
      assert response.status == 400
      assert "protocol v1" in (await response.json())["error"]

    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(len(encoded), 0), (len(encoded), 0)]

  asyncio.run(run())


def test_validation_capture_rejects_wrong_types_bounds_and_nested_shape(tmp_path: Path):
  async def run():
    content = b"not-uploaded"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    base = _completion_body(files)
    invalid_bodies: list[dict[str, Any]] = []

    missing = json.loads(json.dumps(base))
    del missing["validationCapture"]["settingsEpoch"]
    invalid_bodies.append(missing)
    wrong_type = json.loads(json.dumps(base))
    wrong_type["validationCapture"]["duration"] = True
    invalid_bodies.append(wrong_type)
    out_of_bounds = json.loads(json.dumps(base))
    out_of_bounds["validationCapture"]["routeSettings"]["PathOffset"] = 151
    invalid_bodies.append(out_of_bounds)
    nested_shape = json.loads(json.dumps(base))
    nested_shape["validationCapture"]["git"]["topology"]["safetyConfigs"] = [
      {"model": "hyundaiCanfd", "param": 0} for _ in range(9)
    ]
    invalid_bodies.append(nested_shape)

    cfg = config(tmp_path)
    charged = 0
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      for body in invalid_bodies:
        encoded = _json_bytes(body)
        charged += len(encoded)
        response = await client.post(
          "/api/v1/validation/complete",
          data=encoded,
          headers={**_auth(token), "Content-Type": "application/json"},
        )
        assert response.status == 400, await response.text()

    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(charged,), (charged,)]

  asyncio.run(run())


def test_validation_capture_rejects_every_non_ka4_stock_scc_gate_variant(tmp_path: Path):
  async def run():
    content = b"not-uploaded"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    base = _completion_body(files)

    invalid_variants: list[tuple[str, tuple[str, ...], Any]] = [
      ("different fingerprint", ("git", "topology", "carFingerprint"), "KIA CARNIVAL 4TH GEN"),
      ("pcmCruise false", ("git", "topology", "pcmCruise"), False),
      ("openpilot longitudinal enabled", ("git", "topology", "openpilotLongitudinalControl"), True),
      ("CAN-FD missing", ("git", "topology", "flags"), 1 << 14),
      ("radar SCC missing", ("git", "topology", "flags"), 1 << 13),
      ("camera SCC present", ("git", "topology", "flags"), (1 << 3) | (1 << 13) | (1 << 14)),
      ("HDA2 present", ("git", "topology", "flags"), (1 << 0) | (1 << 13) | (1 << 14)),
      ("topology gate false", ("git", "topology", "gatePassed"), False),
      (
        "Hyundai CAN-FD safety missing",
        ("git", "topology", "safetyConfigs"),
        [{"model": "noOutput", "param": 0}],
      ),
      ("automatic rearm value invalid", ("routeSettings", "Ka4StockSccStandstillRearm"), 2),
    ]
    invalid_bodies: list[tuple[str, dict[str, Any]]] = []
    for name, path, value in invalid_variants:
      body = json.loads(json.dumps(base))
      target = body["validationCapture"]
      for key in path[:-1]:
        target = target[key]
      target[path[-1]] = value
      invalid_bodies.append((name, body))

    cfg = config(tmp_path)
    charged = 0
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      for name, body in invalid_bodies:
        encoded = _json_bytes(body)
        charged += len(encoded)
        response = await client.post(
          "/api/v1/validation/complete",
          data=encoded,
          headers={**_auth(token), "Content-Type": "application/json"},
        )
        assert response.status == 400, f"{name}: {await response.text()}"

    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(charged,), (charged,)]

  asyncio.run(run())


def test_canonical_manifest_cap_blocks_storage_amplification(tmp_path: Path, monkeypatch):
  async def run():
    content = b"not-uploaded"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    body = _completion_body(files)
    encoded_body = _json_bytes(body)
    monkeypatch.setattr(receiver_server, "VALIDATION_MANIFEST_MAX", 64)
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      response = await client.post(
        "/api/v1/validation/complete",
        data=encoded_body,
        headers={**_auth(token), "Content-Type": "application/json"},
      )
      assert response.status == 413

    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(len(encoded_body), 0), (len(encoded_body), 0)]
      assert connection.execute("SELECT COUNT(*) FROM validation_completions").fetchone()[0] == 0

  asyncio.run(run())


def test_completion_reserves_both_manifest_copies_before_publish(tmp_path: Path, monkeypatch):
  async def run():
    content = b"free-space-rlog"
    files = [{
      "segment": "route--0",
      "name": "rlog.zst",
      "size": len(content),
      "sha256": hashlib.sha256(content).hexdigest(),
    }]
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      uploaded = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert uploaded.status == 200, await uploaded.text()

      free = 1
      monkeypatch.setattr(
        receiver_server.shutil,
        "disk_usage",
        lambda _path: type("DiskUsage", (), {"free": free})(),
      )
      body = _completion_body(files)
      rejected = await client.post("/api/v1/validation/complete", json=body, headers=_auth(token))
      assert rejected.status == 507
      manifest_path = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE / "manifest.json"
      assert not manifest_path.exists()
      with sqlite3.connect(cfg.db_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM validation_completions").fetchone()[0] == 0
        assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0

  asyncio.run(run())


def test_validation_capture_accepts_only_three_unique_segment_rlogs(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      files = []
      for index in range(3):
        content = f"rlog-{index}".encode()
        segment = f"route--{index}"
        response = await client.put(
          f"/api/v1/validation/upload/{CAPTURE}/{segment}/rlog.zst",
          data=content,
          headers=_file_headers(token, content),
        )
        assert response.status == 200, await response.text()
        files.append({
          "segment": segment,
          "name": "rlog.zst",
          "size": len(content),
          "sha256": hashlib.sha256(content).hexdigest(),
        })

      duplicate_segment = b"alternate-compression"
      duplicate = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.bz2",
        data=duplicate_segment,
        headers=_file_headers(token, duplicate_segment),
      )
      assert duplicate.status == 409
      fourth = b"fourth"
      over_limit = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--3/rlog.zst",
        data=fourth,
        headers=_file_headers(token, fourth),
      )
      assert over_limit.status == 409

      too_many = [*files, {
        "segment": "route--3",
        "name": "rlog.zst",
        "size": len(fourth),
        "sha256": hashlib.sha256(fourth).hexdigest(),
      }]
      completion = await client.post(
        "/api/v1/validation/complete",
        json={
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": too_many,
          "validationCapture": _capture_metadata(files),
        },
        headers=_auth(token),
      )
      assert completion.status == 400

  asyncio.run(run())


def test_validation_completion_flood_is_rejected_before_body_and_hash(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(config(tmp_path), validation_concurrent_per_device=1)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      content = b"completion-rlog"
      segment = "route--0"
      uploaded = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/{segment}/rlog.zst",
        data=content,
        headers=_file_headers(token, content),
      )
      assert uploaded.status == 200, await uploaded.text()

      completion_body = {
        "deviceId": DEVICE,
        "captureId": CAPTURE,
        "files": [{
          "segment": segment,
          "name": "rlog.zst",
          "size": len(content),
          "sha256": hashlib.sha256(content).hexdigest(),
        }],
        "validationCapture": _capture_metadata([{
          "segment": segment,
          "name": "rlog.zst",
          "size": len(content),
          "sha256": hashlib.sha256(content).hexdigest(),
        }]),
      }
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_json_body = service._json_body
      original_hash = service._hash_file_bounded
      hash_started = asyncio.Event()
      release_hash = asyncio.Event()
      body_calls = 0
      hash_calls = 0

      async def counting_json_body(request: web.Request, limit: int, **kwargs: Any) -> Any:
        nonlocal body_calls
        body_calls += 1
        return await original_json_body(request, limit, **kwargs)

      async def blocking_hash(path: Path, **kwargs: Any) -> tuple[int, str]:
        nonlocal hash_calls
        hash_calls += 1
        hash_started.set()
        await release_hash.wait()
        return await original_hash(path, **kwargs)

      monkeypatch.setattr(service, "_json_body", counting_json_body)
      monkeypatch.setattr(service, "_hash_file_bounded", blocking_hash)
      first = asyncio.create_task(client.post(
        "/api/v1/validation/complete", json=completion_body, headers=_auth(token),
      ))
      await asyncio.wait_for(hash_started.wait(), timeout=1)
      try:
        second = await asyncio.wait_for(
          client.post("/api/v1/validation/complete", json=completion_body, headers=_auth(token)),
          timeout=1,
        )
        assert second.status == 429, await second.text()
        assert "concurrent" in (await second.json())["error"]
        # Authentication is permitted, but completion parsing and durable-file
        # verification must never queue behind another operation for this device.
        assert body_calls == 1
        assert hash_calls == 1
      finally:
        release_hash.set()

      first_response = await asyncio.wait_for(first, timeout=2)
      assert first_response.status == 200, await first_response.text()
      assert body_calls == 1
      assert hash_calls == 1

  asyncio.run(run())


def test_slow_legacy_body_cannot_consume_validation_capacity(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(
      config(tmp_path),
      concurrent_per_device=1,
      concurrent_global=1,
      validation_concurrent_per_device=1,
      validation_concurrent_global=1,
    )
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      validation_token = await _token(client)
      legacy_session = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam", "carName": "TEST"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      legacy_token = str((await legacy_session.json())["token"])
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_body_chunks = service._body_chunks
      legacy_started = asyncio.Event()
      release_legacy = asyncio.Event()
      calls = 0

      async def block_first_body(content: Any, chunk_size: int, **kwargs: Any):
        nonlocal calls
        calls += 1
        if calls == 1:
          legacy_started.set()
          await release_legacy.wait()
        async for chunk in original_body_chunks(content, chunk_size, **kwargs):
          yield chunk

      monkeypatch.setattr(service, "_body_chunks", block_first_body)
      legacy_content = b"legacy"
      legacy = asyncio.create_task(client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=legacy_content,
        headers={
          "Authorization": f"Bearer {legacy_token}",
          "X-Forwarded-For": CLIENT_IP,
          "X-File-Size": str(len(legacy_content)),
        },
      ))
      await asyncio.wait_for(legacy_started.wait(), timeout=1)
      validation_content = b"validation"
      validation = await asyncio.wait_for(client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--1/rlog.zst",
        data=validation_content,
        headers=_file_headers(validation_token, validation_content),
      ), timeout=1)
      assert validation.status == 200, await validation.text()
      assert service._active_global == 1
      assert service._validation_active_global == 0

      release_legacy.set()
      legacy_response = await asyncio.wait_for(legacy, timeout=1)
      assert legacy_response.status == 200, await legacy_response.text()
      assert service._active_global == 0

  asyncio.run(run())


def test_legacy_preauth_admission_bounds_malformed_flood_without_blocking_validation(
  tmp_path: Path,
  monkeypatch,
):
  async def run():
    cfg = replace(
      config(tmp_path),
      request_concurrent_per_ip=1,
      request_concurrent_global=1,
      validation_request_concurrent_per_ip=2,
      validation_request_concurrent_global=2,
    )
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      validation_token = await _token(client)
      legacy_session = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam", "carName": "TEST"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert legacy_session.status == 200, await legacy_session.text()
      legacy_token = (await legacy_session.json())["token"]
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_authenticate = service.authenticate
      auth_started = asyncio.Event()
      release_auth = asyncio.Event()
      legacy_auth_calls = 0

      async def blocking_authenticate(request: web.Request, **kwargs: Any) -> Any:
        nonlocal legacy_auth_calls
        if request.path.startswith("/api/v1/upload/"):
          legacy_auth_calls += 1
          if legacy_auth_calls == 1:
            auth_started.set()
            await release_auth.wait()
        return await original_authenticate(request, **kwargs)

      monkeypatch.setattr(service, "authenticate", blocking_authenticate)
      malformed_headers = {
        "Authorization": f"Bearer {legacy_token}",
        "X-Forwarded-For": CLIENT_IP,
      }
      first = asyncio.create_task(client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=b"x",
        headers=malformed_headers,
      ))
      await asyncio.wait_for(auth_started.wait(), timeout=1)
      rejected = await client.put(
        f"/api/v1/upload/{DEVICE}/route--1/qlog.zst",
        data=b"x",
        headers=malformed_headers,
      )
      assert rejected.status in {429, 503}
      assert legacy_auth_calls == 1
      assert service._request_active_global == 1

      challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert challenge.status == 200, await challenge.text()
      content = b"validation-still-responsive"
      validation = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=content,
        headers=_file_headers(validation_token, content),
      )
      assert validation.status == 200, await validation.text()

      release_auth.set()
      first_response = await asyncio.wait_for(first, timeout=2)
      assert first_response.status == 411
      assert service._request_active_global == 0
      assert not service._request_active_by_ip

  asyncio.run(run())


def test_fragmented_legacy_body_coalesces_db_progress_and_validation_stays_responsive(
  tmp_path: Path,
  monkeypatch,
):
  async def run():
    cfg = replace(
      config(tmp_path),
      progress_commit_bytes=1024,
      progress_commit_interval_seconds=60,
    )
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      validation_token = await _token(client)
      legacy_session = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam", "carName": "TEST"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      legacy_token = (await legacy_session.json())["token"]
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      original_progress = service._record_reservation_progress
      progress_calls = {receiver_server.LEGACY_QUOTA_NAMESPACE: 0, receiver_server.VALIDATION_PURPOSE: 0}
      first_legacy_progress = asyncio.Event()
      release_fragments = asyncio.Event()

      async def count_progress(reservation: Any, *args: Any, **kwargs: Any) -> None:
        progress_calls[reservation.quota_namespace] += 1
        if reservation.quota_namespace == receiver_server.LEGACY_QUOTA_NAMESPACE:
          first_legacy_progress.set()
        await original_progress(reservation, *args, **kwargs)

      async def fragmented_body():
        for index in range(100):
          yield b"x"
          if index == 0:
            await release_fragments.wait()
          await asyncio.sleep(0)

      monkeypatch.setattr(service, "_record_reservation_progress", count_progress)
      legacy = asyncio.create_task(client.put(
        f"/api/v1/upload/{DEVICE}/route--0/qlog.zst",
        data=fragmented_body(),
        headers={
          "Authorization": f"Bearer {legacy_token}",
          "X-Forwarded-For": CLIENT_IP,
          "X-File-Size": "100",
        },
      ))
      await asyncio.wait_for(first_legacy_progress.wait(), timeout=1)
      validation_content = b"v"
      validation = await asyncio.wait_for(client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=validation_content,
        headers=_file_headers(validation_token, validation_content),
      ), timeout=1)
      assert validation.status == 200, await validation.text()

      release_fragments.set()
      legacy_response = await asyncio.wait_for(legacy, timeout=2)
      assert legacy_response.status == 200, await legacy_response.text()
      assert 1 <= progress_calls[receiver_server.LEGACY_QUOTA_NAMESPACE] <= 3

  asyncio.run(run())


def test_body_idle_timeout_cancels_read_and_total_deadline_is_immediate(tmp_path: Path):
  async def run():
    cfg = replace(
      config(tmp_path),
      upload_idle_timeout_seconds=0.01,
      upload_total_timeout_seconds=60.0,
    )
    service = UploadService(cfg, validation_verifier=FakeVerifier())
    started = asyncio.Event()
    cancelled = asyncio.Event()
    never = asyncio.Event()

    async def blocked_read() -> bytes:
      started.set()
      try:
        await never.wait()
      finally:
        cancelled.set()
      return b"unused"

    deadline = asyncio.get_running_loop().time() + 60.0
    with pytest.raises(web.HTTPRequestTimeout) as idle_error:
      await service._await_body_operation(
        blocked_read,
        deadline=deadline,
        idle_timeout=cfg.upload_idle_timeout_seconds,
      )
    assert "idle" in idle_error.value.text
    assert started.is_set()
    assert cancelled.is_set()
    with pytest.raises(web.HTTPRequestTimeout) as total_error:
      service._body_wait_timeout(asyncio.get_running_loop().time() - 1.0, 60.0)
    assert "total" in total_error.value.text

  asyncio.run(run())


def test_rejected_completion_does_not_create_capture_directories(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      content = b"not-uploaded"
      rejected = await client.post(
        "/api/v1/validation/complete",
        json={
          "deviceId": DEVICE,
          "captureId": CAPTURE,
          "files": [{
            "segment": "route--0",
            "name": "rlog.zst",
            "size": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
          }],
          "validationCapture": _capture_metadata([{
            "segment": "route--0",
            "name": "rlog.zst",
            "size": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
          }]),
        },
        headers=_auth(token),
      )
      assert rejected.status == 409
      assert not (cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE).exists()

  asyncio.run(run())


class BlockingVerifier(FakeVerifier):
  def __init__(self):
    super().__init__()
    self.started = asyncio.Event()
    self.release = asyncio.Event()

  async def verify(self, **kwargs: Any) -> str:
    self.calls.append(kwargs)
    self.started.set()
    await self.release.wait()
    return str(kwargs["expected_device_id"])


def test_validation_verification_is_single_flight_per_challenge(tmp_path: Path):
  async def run():
    verifier = BlockingVerifier()
    async with TestClient(TestServer(create_app(
      config(tmp_path), start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      challenge = await _challenge(client)
      first = asyncio.create_task(_validation_session(client, challenge))
      await verifier.started.wait()
      second_status, second_body = await _validation_session(client, challenge)
      assert second_status == 409
      assert "progress" in second_body["error"]
      assert len(verifier.calls) == 1
      verifier.release.set()
      first_status, first_body = await first
      assert first_status == 200, first_body

  asyncio.run(run())


def test_validation_verification_attempt_limit_is_persistent(tmp_path: Path):
  async def run():
    cfg = replace(
      config(tmp_path),
      validation_verify_attempt_limit=2,
      validation_verify_cooldown_seconds=1,
    )
    verifier = FakeVerifier(error=DeviceProofVerificationRejected("bad signature"))
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as client:
      challenge = await _challenge(client)
      for _attempt in range(2):
        status, _body = await _validation_session(client, challenge)
        assert status == 401
        with sqlite3.connect(cfg.db_path) as connection:
          connection.execute(
            "UPDATE validation_challenges SET retry_after=0 WHERE challenge_id=?",
            (challenge["challengeId"],),
          )
      limited = await client.post(
        "/api/v1/validation/session",
        json={
          "deviceId": DEVICE,
          "challengeId": challenge["challengeId"],
          **_proof(challenge),
        },
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert limited.status == 429
      assert int(limited.headers["Retry-After"]) >= 1
      assert "attempt limit" in (await limited.json())["error"]
      assert len(verifier.calls) == 2

    # The lockout is in SQLite, not process-local state.
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=verifier,
    ))) as restarted:
      status, _body = await _validation_session(restarted, challenge)
      assert status == 429
      assert len(verifier.calls) == 2

  asyncio.run(run())


def test_failed_validation_body_consumes_network_quota(tmp_path: Path):
  async def run():
    cfg = config(tmp_path, quota=8)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      failed_content = b"123456"
      failed = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=failed_content,
        headers=_file_headers(token, failed_content, sha256="0" * 64),
      )
      assert failed.status == 400

      # Six bytes were received even though integrity validation failed, so a
      # further three-byte request exceeds the eight-byte network quota.
      later_content = b"789"
      later = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=later_content,
        headers=_file_headers(token, later_content),
      )
      assert later.status == 413

    with sqlite3.connect(cfg.db_path) as connection:
      usage = connection.execute(
        "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall()
      assert usage == [(6, 0, 0), (6, 0, 0)]
      assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0

  asyncio.run(run())


def test_truncated_validation_body_charges_received_not_declared_bytes(tmp_path: Path):
  async def run():
    cfg = config(tmp_path)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)

      async def truncated_body():
        yield b"abc"

      declared = b"abcdef"
      response = await client.put(
        f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
        data=truncated_body(),
        headers=_file_headers(token, declared),
      )
      assert response.status == 400

    with sqlite3.connect(cfg.db_path) as connection:
      usage = connection.execute(
        "SELECT committed_bytes, reserved_bytes FROM daily_usage ORDER BY scope",
      ).fetchall()
      assert usage == [(3, 0), (3, 0)]

  asyncio.run(run())


def test_startup_reconciles_stale_parts_leases_and_missing_incomplete_rows(tmp_path: Path):
  cfg = replace(config(tmp_path), reservation_lease_seconds=1, stale_part_seconds=1)
  UploadService(cfg, validation_verifier=FakeVerifier())
  now = int(datetime.now(UTC).timestamp())
  day = datetime.now(UTC).strftime("%Y-%m-%d")
  missing_relative = f"{VALIDATION_NAMESPACE}/{DEVICE}/{CAPTURE}/route--0/rlog.zst"
  with sqlite3.connect(cfg.db_path) as connection:
    for scope, scope_id in (("device", DEVICE), ("ip", CLIENT_IP)):
      connection.execute(
        """INSERT INTO daily_usage(
             usage_day, scope, scope_id, committed_bytes, reserved_bytes, stored_bytes
           ) VALUES(?,?,?,?,?,?)""",
        (day, scope, scope_id, 2, 99, 1),
      )
    connection.execute(
      """INSERT INTO upload_reservations(
           reservation_id, usage_day, device_id, source_ip, reserved_bytes,
           received_bytes, stored_bytes, created_at, touched_at
         ) VALUES(?,?,?,?,?,?,?,?,?)""",
      ("stale-reservation", day, DEVICE, CLIENT_IP, 10, 7, 3, now - 20, now - 20),
    )
    connection.execute(
      """INSERT INTO validation_files(
           device_id, capture_id, segment, filename, size, sha256, relative_path, received_at
         ) VALUES(?,?,?,?,?,?,?,?)""",
      (DEVICE, CAPTURE, "route--0", "rlog.zst", 5, "0" * 64, missing_relative, now - 20),
    )

  stale_part = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / f".rlog.{('a' * 32)}.part"
  stale_part.parent.mkdir(parents=True)
  stale_part.write_bytes(b"partial")
  os.utime(stale_part, (now - 20, now - 20))

  UploadService(cfg, validation_verifier=FakeVerifier())
  assert not stale_part.exists()
  with sqlite3.connect(cfg.db_path) as connection:
    assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0
    usage = connection.execute(
      "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
    ).fetchall()
    assert usage == [(9, 0, 4), (9, 0, 4)]
    assert connection.execute("SELECT COUNT(*) FROM validation_files").fetchone()[0] == 0


def test_validation_only_cleanup_never_scans_legacy_routes(tmp_path: Path):
  cfg = replace(config(tmp_path), legacy_uploads_enabled=False)
  service = UploadService(cfg, validation_verifier=FakeVerifier())
  validation_part = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / f".rlog.{('a' * 32)}.part"
  legacy_part = cfg.storage_root / "routes" / DEVICE / f".qlog.{('b' * 32)}.part"
  validation_part.parent.mkdir(parents=True)
  legacy_part.parent.mkdir(parents=True)
  validation_part.write_bytes(b"validation")
  legacy_part.write_bytes(b"legacy-must-not-be-scanned")

  assert service._cleanup_stale_parts_sync(int(datetime.now(UTC).timestamp()), startup=True) == 1
  assert not validation_part.exists()
  assert legacy_part.read_bytes() == b"legacy-must-not-be-scanned"


def test_restart_reconciles_fresh_reservation_and_owned_parts_immediately(tmp_path: Path):
  async def run():
    cfg = config(tmp_path, quota=4)
    first_service = UploadService(cfg, validation_verifier=FakeVerifier())
    reservation = await first_service._reserve(
      DEVICE,
      CLIENT_IP,
      4,
      validation=True,
    )
    await first_service._record_reservation_progress(
      reservation,
      3,
      1,
      disk_written_bytes=2,
    )
    capture_root = cfg.storage_root / VALIDATION_NAMESPACE / DEVICE / CAPTURE
    capture_root.mkdir(parents=True)
    fresh_owned_part = capture_root / f".rlog.zst.{('a' * 32)}.part"
    fresh_owned_part.write_bytes(b"ab")
    published = capture_root / "published-rlog.zst"
    published.write_bytes(b"x")
    unowned_part = capture_root / "operator-note.part"
    unowned_part.write_bytes(b"keep")

    periodic = await first_service.cleanup()
    assert periodic["staleReservations"] == 0
    assert periodic["staleParts"] == 0
    assert fresh_owned_part.exists()
    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 1

    restarted = UploadService(cfg, validation_verifier=FakeVerifier())
    assert not fresh_owned_part.exists()
    assert published.read_bytes() == b"x"
    assert unowned_part.read_bytes() == b"keep"
    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0
      assert connection.execute(
        "SELECT scope, committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [
        ("validation_device", 3, 0, 1),
        ("validation_ip", 3, 0, 1),
      ]

    retry = await restarted._reserve(DEVICE, CLIENT_IP, 1, validation=True)
    await restarted._finish_reservation(retry, 1)
    with sqlite3.connect(cfg.db_path) as connection:
      assert connection.execute(
        "SELECT committed_bytes, reserved_bytes FROM daily_usage ORDER BY scope",
      ).fetchall() == [(4, 0), (4, 0)]

  asyncio.run(run())


def test_reservation_schema_migration_and_startup_reconciliation_preserve_accounting(tmp_path: Path):
  db_path = tmp_path / "state" / "uploads.sqlite3"
  db_path.parent.mkdir(parents=True)
  now = int(datetime.now(UTC).timestamp())
  with sqlite3.connect(db_path) as connection:
    connection.execute(
      """CREATE TABLE upload_reservations (
           reservation_id TEXT PRIMARY KEY,
           usage_day TEXT NOT NULL,
           device_id TEXT NOT NULL,
           source_ip TEXT NOT NULL,
           reserved_bytes INTEGER NOT NULL,
           received_bytes INTEGER NOT NULL DEFAULT 0,
           stored_bytes INTEGER NOT NULL DEFAULT 0,
           created_at INTEGER NOT NULL,
           touched_at INTEGER NOT NULL
         )""",
    )
    connection.execute(
      """CREATE TABLE daily_usage (
           usage_day TEXT NOT NULL,
           scope TEXT NOT NULL,
           scope_id TEXT NOT NULL,
           committed_bytes INTEGER NOT NULL DEFAULT 0,
           reserved_bytes INTEGER NOT NULL DEFAULT 0,
           stored_bytes INTEGER NOT NULL DEFAULT 0,
           PRIMARY KEY (usage_day, scope, scope_id)
         )""",
    )
    connection.executemany(
      """INSERT INTO daily_usage(
           usage_day, scope, scope_id, committed_bytes, reserved_bytes, stored_bytes
         ) VALUES(?,?,?,?,?,?)""",
      (
        ("2099-01-01", "device", DEVICE, 0, 100, 0),
        ("2099-01-01", "ip", CLIENT_IP, 0, 100, 0),
      ),
    )
    connection.execute(
      """INSERT INTO upload_reservations(
           reservation_id, usage_day, device_id, source_ip, reserved_bytes,
           received_bytes, stored_bytes, created_at, touched_at
         ) VALUES(?,?,?,?,?,?,?,?,?)""",
      ("legacy", "2099-01-01", DEVICE, CLIENT_IP, 100, 40, 0, now, now),
    )

  cfg = replace(config(tmp_path), db_path=db_path)
  UploadService(cfg, validation_verifier=FakeVerifier())
  with sqlite3.connect(db_path) as connection:
    columns = {row[1] for row in connection.execute("PRAGMA table_info(upload_reservations)")}
    assert {"reserved_storage_bytes", "disk_written_bytes", "quota_namespace"} <= columns
    assert connection.execute("SELECT COUNT(*) FROM upload_reservations").fetchone()[0] == 0
    assert connection.execute(
      "SELECT committed_bytes, reserved_bytes, stored_bytes FROM daily_usage ORDER BY scope",
    ).fetchall() == [(40, 0, 0), (40, 0, 0)]


def test_free_space_reservations_are_atomic_across_in_flight_uploads(tmp_path: Path, monkeypatch):
  async def run():
    cfg = replace(config(tmp_path), min_free_bytes=10)
    monkeypatch.setattr(
      receiver_server.shutil,
      "disk_usage",
      lambda _path: type("DiskUsage", (), {"free": 15})(),
    )
    service = UploadService(cfg, validation_verifier=FakeVerifier())
    first = await service._reserve(DEVICE, CLIENT_IP, 4)
    try:
      try:
        await service._reserve(OTHER_DEVICE, "203.0.113.41", 4)
        raise AssertionError("concurrent reservations must not overcommit free space")
      except web.HTTPInsufficientStorage:
        pass
    finally:
      await service._finish_reservation(first, 0)

  asyncio.run(run())


def test_legacy_reserve_cannot_consume_validation_quota_or_disk_headroom(
  tmp_path: Path,
  monkeypatch,
):
  async def run():
    cfg = replace(
      config(tmp_path, quota=4),
      daily_ip_quota=16,
      min_free_bytes=10,
      validation_free_space_reserve_bytes=4,
    )
    monkeypatch.setattr(
      receiver_server.shutil,
      "disk_usage",
      lambda _path: type("DiskUsage", (), {"free": 18})(),
    )
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      token = await _token(client)
      service = client.app[receiver_server.UPLOAD_SERVICE_KEY]
      legacy = await service._reserve(DEVICE, CLIENT_IP, 4)
      try:
        with pytest.raises(web.HTTPInsufficientStorage):
          await service._reserve(OTHER_DEVICE, "203.0.113.99", 1)

        content = b"safe"
        validation = await client.put(
          f"/api/v1/validation/upload/{CAPTURE}/route--0/rlog.zst",
          data=content,
          headers=_file_headers(token, content),
        )
        assert validation.status == 200, await validation.text()
        with sqlite3.connect(cfg.db_path) as connection:
          rows = connection.execute(
            "SELECT scope, committed_bytes, reserved_bytes FROM daily_usage ORDER BY scope",
          ).fetchall()
        assert rows == [
          ("device", 0, 4),
          ("ip", 0, 4),
          ("validation_device", 4, 0),
          ("validation_ip", 4, 0),
        ]
      finally:
        await service._finish_reservation(legacy, 0)

  asyncio.run(run())


def test_legacy_session_rate_saturation_does_not_block_validation_challenge(tmp_path: Path):
  async def run():
    cfg = replace(config(tmp_path), session_issue_limit=2)
    async with TestClient(TestServer(create_app(
      cfg, start_cleanup=False, validation_verifier=FakeVerifier(),
    ))) as client:
      for _ in range(2):
        response = await client.post(
          "/api/v1/session",
          json={"deviceId": DEVICE, "purpose": "dashcam"},
          headers={"X-Forwarded-For": CLIENT_IP},
        )
        assert response.status == 200, await response.text()
      saturated = await client.post(
        "/api/v1/session",
        json={"deviceId": DEVICE, "purpose": "dashcam"},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert saturated.status == 429

      challenge = await client.post(
        "/api/v1/validation/challenge",
        json={"deviceId": DEVICE},
        headers={"X-Forwarded-For": CLIENT_IP},
      )
      assert challenge.status == 200, await challenge.text()

  asyncio.run(run())


def test_session_rate_bucket_count_is_bounded_pruned_and_namespaced(tmp_path: Path, monkeypatch):
  async def run():
    clock = 1000.0
    monkeypatch.setattr(receiver_server.time, "monotonic", lambda: clock)
    cfg = replace(config(tmp_path), session_rate_bucket_limit=4, session_issue_window_seconds=10)
    service = UploadService(cfg, validation_verifier=FakeVerifier())
    for index in range(4):
      service._check_session_rate(f"2001:db8::{index}", validation=False)
    with pytest.raises(web.HTTPTooManyRequests):
      service._check_session_rate("2001:db8::ffff", validation=False)
    service._check_session_rate("2001:db8::ffff", validation=True)
    assert len(service._session_issues[receiver_server.LEGACY_QUOTA_NAMESPACE]) == 4
    assert len(service._session_issues[receiver_server.VALIDATION_PURPOSE]) == 1

    clock += 11
    result = await service.cleanup()
    assert result["sessionRateBuckets"] == 5
    assert not service._session_issues[receiver_server.LEGACY_QUOTA_NAMESPACE]
    assert not service._session_issues[receiver_server.VALIDATION_PURPOSE]

  asyncio.run(run())
