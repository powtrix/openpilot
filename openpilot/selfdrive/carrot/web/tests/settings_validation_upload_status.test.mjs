import assert from "node:assert/strict";
import test from "node:test";

import {
  normalizeValidationUploadStatus,
  VALIDATION_UPLOAD_STATUS_LABEL_KEYS,
} from "../src/features/settings/validation_upload_status.js";

test("public validation states collapse into localized display groups", () => {
  const cases = {
    armed: "armed",
    waiting_for_route_finalize: "recording",
    recording: "recording",
    queued: "queued",
    cleanup_pending: "queued",
    uploading: "uploading",
    retry_wait: "retry",
    upload_blocked: "retry",
    queue_limit: "full",
    vehicle_not_supported: "unsupported",
    complete: "complete",
    expired: "expired",
    parked_consent_required: "error",
    reconsent_required: "error",
    state_invalid: "error",
  };

  for (const [statusCode, expected] of Object.entries(cases)) {
    assert.equal(
      normalizeValidationUploadStatus({ enabled: true, serviceRunning: true, statusCode }).status,
      expected,
      statusCode,
    );
  }
  assert.deepEqual(
    Object.keys(VALIDATION_UPLOAD_STATUS_LABEL_KEYS).sort(),
    ["armed", "complete", "disabled", "enabled", "error", "expired", "full", "queued", "recording", "retry", "unsupported", "uploading"],
  );
});

test("disabled, stopped, and unknown states fail closed", () => {
  assert.equal(normalizeValidationUploadStatus({ enabled: false, statusCode: "uploading" }).status, "disabled");
  assert.equal(normalizeValidationUploadStatus({ enabled: false, statusCode: "parked_consent_required" }).status, "error");
  assert.equal(normalizeValidationUploadStatus({ enabled: true, statusCode: "disabled" }).status, "enabled");
  assert.equal(normalizeValidationUploadStatus({ enabled: true, serviceRunning: false, statusCode: "armed" }).status, "error");
  assert.equal(normalizeValidationUploadStatus({ enabled: true, statusCode: "future_internal_state" }).status, "error");
});

test("the UI model copies only bounded display-safe fields", () => {
  const result = normalizeValidationUploadStatus({
    enabled: true,
    serviceRunning: true,
    statusCode: "queued",
    pendingCaptures: 5000.9,
    expiresAt: 2_000_000_000.7,
    lastUploadedAt: -1,
    route: "secret-route",
    receiverUrl: "https://secret.invalid",
    lastError: "raw stack trace",
    deviceId: "secret-device",
  });

  assert.deepEqual({ ...result }, {
    enabled: true,
    status: "queued",
    pendingCaptures: 999,
    expiresAt: 2_000_000_000,
    lastUploadedAt: 0,
  });
  assert.equal(Object.isFrozen(result), true);
});
