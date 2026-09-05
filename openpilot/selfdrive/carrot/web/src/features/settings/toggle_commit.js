/**
 * Commit a toggle without letting its optimistic DOM state become authoritative.
 * The caller owns rendering; this helper guarantees that cancel/rejection puts
 * the control back on the last server-confirmed value.
 */
export async function commitSettingToggle(options = {}) {
  const commit = options.commit;
  const restore = options.restore;
  if (typeof commit !== "function" || typeof restore !== "function") {
    throw new TypeError("commit and restore functions are required");
  }

  if (options.requiresConfirmation === true) {
    if (typeof options.confirm !== "function") throw new TypeError("a confirmation function is required");
    if (!await options.confirm()) {
      restore(options.previous);
      return Object.freeze({ committed: false, cancelled: true });
    }
  }

  try {
    const accepted = await commit(options.next);
    if (accepted === false) {
      restore(options.previous);
      return Object.freeze({ committed: false, cancelled: false });
    }
    return Object.freeze({ committed: true, cancelled: false });
  } catch (error) {
    restore(options.previous);
    throw error;
  }
}
