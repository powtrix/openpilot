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
    name: "DkExperimentalScc", title: "SCC 실험", p: { default: 1 }, val,
    profile: null, el: {}, group: "START_AUTO", originGroup: "START_AUTO", validationUploadStatus: null,
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
  test(`SCC ${next ? "ON" : "OFF"} explicitly applies at next controls start`, async () => {
    const runtime = commitRuntime(1 - next);
    assert.equal(await runtime.commit(next), true);
    assert.equal(runtime.calls.some(c => c[0] === "confirm"), false);
    assert.deepEqual(runtime.calls.find(c => c[0] === "write"), ["write", "DkExperimentalScc", next]);
    assert.match(runtime.calls.find(c => c[0] === "toast")[1], /current controller has not changed/);
    assert.equal(runtime.val.dataset.committedValue, String(next));
  });
}

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
    assert.ok(strings.setting_dk_scc_restart.length > 20);
    assert.equal(strings.setting_dk_steering_confirm, undefined);
    assert.equal(strings.setting_dk_steering_restart, undefined);
  }
});

test("retired steering confirmation is absent from the setting flow", () => {
  assert.equal(source.includes("DkExperimentalSteering"), false);
  assert.equal(source.includes("setting_dk_steering_confirm"), false);
});
