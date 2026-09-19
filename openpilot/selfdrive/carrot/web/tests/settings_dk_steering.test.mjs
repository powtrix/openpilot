import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const source = readFileSync(new URL("../js/pages/setting.js", import.meta.url), "utf8");
const begin = source.indexOf("    async function commitSettingValue(");
const end = source.indexOf("    function bindPopularDetailRows()", begin);
assert.ok(begin >= 0 && end > begin);

function commitRuntime(previous, { confirm = true, fail = false } = {}) {
  const calls = [];
  const val = { dataset: { committedValue: String(previous), rawValue: String(previous) } };
  const env = {
    name: "DkExperimentalSteering", title: "실험용 조향개선", p: { default: 0 }, val,
    profile: null, el: {}, group: "STEER", originGroup: "STEER", validationUploadStatus: null,
    VALIDATION_AUTO_UPLOAD_PARAM: "CarrotValidationAutoUpload",
    COMMUNITY_DATA_SHARING_PARAM: "CarrotCommunityDataSharing",
    THIRD_PARTY_DATA_SHARING_PARAM: "DkThirdPartyDataSharing",
    renderedLogSharingValues: {}, LANG: "en", UI_STRINGS: { en: { set_failed: "failed: " } },
    getUIText: (_key, fallback) => fallback,
    appConfirm: async (message) => { calls.push(["confirm", message]); return confirm; },
    syncSettingControlState: (_el, value) => calls.push(["sync", value]),
    setParam: async (key, value) => {
      calls.push(["write", key, value]);
      if (fail) throw new Error("offroad required");
      return { ok: true, value, applies_at: "controls_start", restart_recommended: true };
    },
    showAppToast: (message) => calls.push(["toast", message]),
    cacheSettingValue() {}, refreshSettingHistory() {}, syncLogSharingPolicyUi() {},
  };
  vm.createContext(env);
  vm.runInContext(`${source.slice(begin, end)}; globalThis.commit = commitSettingValue;`, env);
  return { calls, val, commit: env.commit };
}

for (const next of [0, 1]) {
  test(`steering ${next ? "ON" : "OFF"} is confirmed and explicitly applies at next controls start`, async () => {
    const runtime = commitRuntime(1 - next);
    assert.equal(await runtime.commit(next), true);
    assert.equal(runtime.calls[0][0], "confirm");
    assert.match(runtime.calls[0][1], /ON and OFF apply when controls next starts/);
    assert.deepEqual(runtime.calls.find(c => c[0] === "write"), ["write", "DkExperimentalSteering", next]);
    assert.match(runtime.calls.find(c => c[0] === "toast")[1], /current controller has not changed/);
    assert.equal(runtime.val.dataset.committedValue, String(next));
  });
}

test("cancelled experimental selection writes nothing and restores the previous choice", async () => {
  const runtime = commitRuntime(0, { confirm: false });
  assert.equal(await runtime.commit(1), false);
  assert.equal(runtime.calls.some(c => c[0] === "write"), false);
  assert.deepEqual(runtime.calls.at(-1), ["sync", "0"]);
  assert.equal(runtime.val.dataset.committedValue, "0");
});

test("server onroad rejection restores stored value and never claims restart-ready success", async () => {
  const runtime = commitRuntime(1, { fail: true });
  assert.equal(await runtime.commit(0), false);
  assert.deepEqual(runtime.calls.find(c => c[0] === "sync"), ["sync", "1"]);
  assert.equal(runtime.val.dataset.committedValue, "1");
  assert.match(runtime.calls.find(c => c[0] === "toast")[1], /offroad required/);
});

test("all Web locales explicitly explain delayed ON/OFF application", () => {
  for (const locale of ["ko", "en", "zh"]) {
    let strings;
    const context = { window: { CarrotTranslations: { register: (_locale, data) => { strings = data.strings; } } } };
    vm.runInNewContext(readFileSync(new URL(`../js/translations/${locale}.js`, import.meta.url), "utf8"), context);
    assert.ok(strings.setting_dk_steering_confirm.length > 20);
    assert.ok(strings.setting_dk_steering_restart.length > 20);
  }
});
