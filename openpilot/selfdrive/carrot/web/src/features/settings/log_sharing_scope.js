function consentEnabled(value) {
  return value === true || value === 1 || value === "1";
}

export const LOG_SHARING_SCOPE_LABEL_KEYS = Object.freeze({
  allBlocked: "log_sharing_scope_all_blocked",
  privateNasOnly: "log_sharing_scope_private_nas_only",
  otherAllowed: "log_sharing_scope_other_allowed",
  communityBlocked: "community_data_sharing_blocked_by_master",
});

/**
 * Reduce the two independent consent switches to the three user-visible
 * transfer scopes. KA4 validation deliberately remains independent from the
 * master switch; every other log or diagnostic destination is below it.
 */
export function normalizeLogSharingScope({ thirdPartyEnabled, validationEnabled } = {}) {
  const otherAllowed = consentEnabled(thirdPartyEnabled);
  const validationAllowed = consentEnabled(validationEnabled);
  const scope = otherAllowed
    ? "other_allowed"
    : (validationAllowed ? "private_nas_only" : "all_blocked");

  return Object.freeze({
    scope,
    scopeLabelKey: scope === "other_allowed"
      ? LOG_SHARING_SCOPE_LABEL_KEYS.otherAllowed
      : (scope === "private_nas_only"
        ? LOG_SHARING_SCOPE_LABEL_KEYS.privateNasOnly
        : LOG_SHARING_SCOPE_LABEL_KEYS.allBlocked),
    communityBlocked: !otherAllowed,
    communityBlockedLabelKey: LOG_SHARING_SCOPE_LABEL_KEYS.communityBlocked,
  });
}
