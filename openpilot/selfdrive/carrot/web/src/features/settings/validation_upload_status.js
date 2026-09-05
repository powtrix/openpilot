const STATUS_GROUP_BY_CODE = Object.freeze({
  disabled: "disabled",
  idle: "enabled",
  parked_consent_required: "error",
  reconsent_required: "error",
  armed: "armed",
  waiting_for_route_finalize: "recording",
  recording: "recording",
  queued: "queued",
  cleanup_pending: "queued",
  uploading: "uploading",
  retry_wait: "retry",
  upload_blocked: "retry",
  manual_upload_active: "retry",
  destination_changed: "error",
  https_required: "error",
  queue_limit: "full",
  vehicle_not_supported: "unsupported",
  capture_unavailable: "unsupported",
  complete: "complete",
  expired: "expired",
  clock_rollback: "error",
  complete_disable_failed: "error",
  expired_disable_failed: "error",
  preserve_failed: "error",
  preserve_recovery_failed: "error",
  service_error: "error",
  state_invalid: "error",
  state_write_failed: "error",
  error: "error",
});

function safeCount(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return 0;
  return Math.max(0, Math.min(999, Math.floor(number)));
}

function safeEpochSeconds(value) {
  const number = Number(value);
  if (!Number.isFinite(number) || number <= 0) return 0;
  // Keep dates inside the ECMAScript Date range and discard sub-second input.
  return Math.min(8_640_000_000, Math.floor(number));
}

/**
 * Reduce the status endpoint to the small, display-safe contract used by the
 * settings page. Unknown server states deliberately become a generic error;
 * paths, routes, identifiers, receiver details and error messages are never
 * copied into the returned object.
 */
export function normalizeValidationUploadStatus(payload) {
  const source = payload && typeof payload === "object" ? payload : {};
  const enabled = source.enabled === true;
  const rawCode = typeof source.statusCode === "string" ? source.statusCode : "";
  let status = STATUS_GROUP_BY_CODE[rawCode] || "error";

  if (!enabled && !["complete", "expired", "error", "unsupported"].includes(status)) status = "disabled";
  if (enabled && status === "disabled") status = "enabled";
  if (enabled && source.serviceRunning === false) status = "error";

  return Object.freeze({
    enabled,
    status,
    pendingCaptures: safeCount(source.pendingCaptures),
    expiresAt: safeEpochSeconds(source.expiresAt),
    lastUploadedAt: safeEpochSeconds(source.lastUploadedAt),
  });
}

export const VALIDATION_UPLOAD_STATUS_LABEL_KEYS = Object.freeze({
  disabled: "validation_upload_status_disabled",
  enabled: "validation_upload_status_enabled",
  armed: "validation_upload_status_armed",
  recording: "validation_upload_status_recording",
  queued: "validation_upload_status_queued",
  uploading: "validation_upload_status_uploading",
  retry: "validation_upload_status_retry",
  full: "validation_upload_status_full",
  unsupported: "validation_upload_status_unsupported",
  complete: "validation_upload_status_complete",
  expired: "validation_upload_status_expired",
  error: "validation_upload_status_error",
});
