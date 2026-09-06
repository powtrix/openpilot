"use strict";

/* ── Params helpers ───────────────────────────────────── */
async function bulkGet(names) {
  const q = encodeURIComponent(names.join(","));
  const j = await getJson("/api/params_bulk?names=" + q);
  return j.values || {};
}

// Every parameter write in the app goes through here, so this is where the
// shared commit rules live: the value the server reports back is the one that
// wins, and each write says where it came from so the change history can tell
// a mis-tap apart from a profile apply or a steering-wheel button.
let paramCommitter = null;

function getParamCommitter() {
  if (!paramCommitter) {
    paramCommitter = window.CarrotParamCommit.create({
      postJson,
      dispatchEvent: (result) => {
        window.dispatchEvent(new CustomEvent("carrot:paramchange", {
          detail: { name: result.name, value: result.value, source: result.source },
        }));
      },
    });
  }
  return paramCommitter;
}

async function setParam(name, value, options) {
  return getParamCommitter().commit(name, value, options);
}


/* ── Generic fetch wrappers ─────────────────────────── */
function buildApiError(response, payload) {
  const friendly = (typeof friendlyError === "function") ? friendlyError(payload) : null;
  const message = friendly || payload?.error || payload?.out || `HTTP ${response.status}`;
  const error = new Error(message);
  error.status = response.status;
  error.payload = payload || null;
  error.errorCode = payload?.error_code || null;
  error.errorDetail = payload?.error_detail || null;
  return error;
}

async function readJsonResponse(response) {
  const text = await response.text();
  if (!text) return {};
  try {
    return JSON.parse(text);
  } catch (e) {
    return {
      ok: false,
      error: text.slice(0, 500) || `HTTP ${response.status}`,
      error_code: "INVALID_JSON_RESPONSE",
    };
  }
}

async function requestJson(url, options = {}) {
  const response = await fetch(url, options);
  const payload = await readJsonResponse(response);
  if (!response.ok || payload?.ok === false) {
    throw buildApiError(response, payload);
  }
  return payload;
}

async function getWebConsentSessionToken() {
  const payload = await requestJson("/api/web-consent/session", {
    cache: "no-store",
    credentials: "same-origin",
  });
  const token = String(payload?.token || "").trim();
  if (!token) throw new Error("Web consent session is unavailable");
  return token;
}

async function postJson(url, bodyObj, options = {}) {
  const headers = {
    "Content-Type": "application/json",
    "X-Carrot-Web-Request": "1",
  };
  // Fetch only after the user accepts one of the two high-risk dialogs. The
  // server issues it only to the active tether-host peer and binds the
  // short-lived, one-use token to both that TCP peer and the exact origin.
  if (options.webConsent === true) {
    headers["X-Carrot-Web-Consent"] = await getWebConsentSessionToken();
  }
  return requestJson(url, {
    method: "POST",
    headers,
    credentials: "same-origin",
    body: JSON.stringify(bodyObj || {})
  });
}

async function getJson(url) {
  return requestJson(url, { cache: "no-store" });
}

function waitMs(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
