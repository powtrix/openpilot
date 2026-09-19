import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import {
  LOG_SHARING_SCOPE_LABEL_KEYS,
  normalizeLogSharingScope,
} from "../src/features/settings/log_sharing_scope.js";

const webRoot = new URL("../", import.meta.url);

async function read(relativePath) {
  return readFile(new URL(relativePath, webRoot), "utf8");
}

test("the consent matrix has exactly three plain-language transfer scopes", () => {
  assert.deepEqual(
    { ...normalizeLogSharingScope({ thirdPartyEnabled: 0, validationEnabled: 0 }) },
    {
      scope: "all_blocked",
      scopeLabelKey: LOG_SHARING_SCOPE_LABEL_KEYS.allBlocked,
      communityBlocked: true,
      communityBlockedLabelKey: LOG_SHARING_SCOPE_LABEL_KEYS.communityBlocked,
    },
  );
  assert.equal(
    normalizeLogSharingScope({ thirdPartyEnabled: 0, validationEnabled: 1 }).scope,
    "private_nas_only",
  );
  assert.equal(
    normalizeLogSharingScope({ thirdPartyEnabled: 1, validationEnabled: 0 }).scope,
    "other_allowed",
  );
  assert.equal(
    normalizeLogSharingScope({ thirdPartyEnabled: "1", validationEnabled: "1" }).scope,
    "other_allowed",
  );
  assert.equal(
    normalizeLogSharingScope({ thirdPartyEnabled: 0, validationEnabled: 1 }).communityBlocked,
    true,
  );
  assert.equal(
    normalizeLogSharingScope({ thirdPartyEnabled: 1, validationEnabled: 0 }).communityBlocked,
    false,
  );
});

test("settings UI renders the scope and blocks the community switch under master off", async () => {
  const source = await read("js/pages/setting.js");
  assert.match(source, /carrotSettingsRuntime\?\.logSharing/);
  assert.match(source, /function syncLogSharingPolicyUi\(\)/);
  assert.match(source, /log-sharing-scope-status/);
  assert.match(source, /community-sharing-policy-status/);
  assert.match(source, /communityToggle\.disabled = state\.communityBlocked/);
  assert.match(
    source,
    /name === THIRD_PARTY_DATA_SHARING_PARAM[\s\S]*?\{ \[COMMUNITY_DATA_SHARING_PARAM\]: 0 \}/,
  );
  assert.match(source, /syncLogSharingPolicyUi\(\)/);
});

test("all supported languages explain every scope and the community block", async () => {
  const requiredFragments = {
    en: ["All log transfers blocked", "Only my DK NAS validation logs", "Other logs and diagnostics allowed", "Blocked by DK Other Logs"],
    ko: ["모든 로그 전송 차단", "내 DK NAS 검증 로그만", "기타 로그·진단 외부 전송 허용", "DK 기타 로그·진단 외부 전송이 꺼져 있어 차단됨"],
    zh: ["阻止所有日志传输", "仅发送我的 DK NAS 验证日志", "允许发送其他日志与诊断", "DK 其他日志与诊断传输已关闭"],
  };

  for (const [language, fragments] of Object.entries(requiredFragments)) {
    const source = await read(`js/translations/${language}.js`);
    for (const fragment of fragments) assert.ok(source.includes(fragment), `${language}: ${fragment}`);
  }
});
