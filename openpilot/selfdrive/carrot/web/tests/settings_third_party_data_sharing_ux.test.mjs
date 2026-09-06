import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";

import { commitSettingToggle } from "../src/features/settings/toggle_commit.js";

const webRoot = new URL("../", import.meta.url);

async function read(relativePath) {
  return readFile(new URL(relativePath, webRoot), "utf8");
}

async function translatedStrings(language) {
  const source = await read(`js/translations/${language}.js`);
  let registered = null;
  vm.runInNewContext(source, {
    window: {
      CarrotTranslations: {
        register(actualLanguage, payload) {
          registered = { actualLanguage, payload };
        },
      },
    },
  }, { filename: `${language}.js` });
  assert.equal(registered?.actualLanguage, language);
  assert.ok(registered?.payload?.strings);
  return registered.payload.strings;
}

test("the third-party master toggle requires its own 0-to-1 confirmation and consent proof", async () => {
  const source = await read("js/pages/setting.js");
  assert.match(source, /THIRD_PARTY_DATA_SHARING_PARAM = "DkThirdPartyDataSharing"/);
  assert.match(source, /function confirmThirdPartyDataSharingEnable\(\)/);
  assert.match(source, /third_party_data_sharing_enable_confirm/);
  assert.match(source, /name === THIRD_PARTY_DATA_SHARING_PARAM[\s\S]{0,180}!thirdPartyConsentConfirmed/);
  assert.match(source, /requiresThirdPartyConfirmation = name === THIRD_PARTY_DATA_SHARING_PARAM\s*&& next === 1\s*&& String\(previous\) !== "1"/);
  assert.match(source, /requiresThirdPartyConfirmation\s*\? confirmThirdPartyDataSharingEnable/);
  assert.match(source, /thirdPartyConsentConfirmed: requiresThirdPartyConfirmation/);
  assert.match(source, /name === THIRD_PARTY_DATA_SHARING_PARAM && thirdPartyConsentConfirmed/);
  assert.match(source, /paramCommitOptions\.webConsent = true/);
  assert.match(
    source,
    /third_party_data_sharing_enable_confirm[\s\S]{0,600}personal-NAS\/manual dashcam uploads, Git updates, and navigation\/maps remain independent/,
  );
});

test("cancelling the third-party consent restores the committed value without writing", async () => {
  const events = [];
  const result = await commitSettingToggle({
    next: 1,
    previous: 0,
    requiresConfirmation: true,
    confirm: async () => {
      events.push("confirm-third-party");
      return false;
    },
    commit: async () => events.push("write"),
    restore: (value) => events.push(["restore", value]),
  });

  assert.deepEqual({ ...result }, { committed: false, cancelled: true });
  assert.deepEqual(events, ["confirm-third-party", ["restore", 0]]);
});

test("all supported languages disclose the third-party transfer scope and independent paths", async () => {
  const requiredFragments = {
    en: [
      "Athena", "cloudlogs", "location", "rlog/qlog/qcamera", "remote SSH", "Prime/Firehose",
      "Sentry", "stock uploader", "Carrot community sharing", "old Athena upload queue",
      "KA4 validation", "private NAS", "personal-NAS", "dashcam uploads", "Git updates",
      "online routing/maps", "remain independent", "mobile data", "not deleted automatically",
    ],
    ko: [
      "Athena", "cloudlog", "위치", "rlog/qlog/qcamera", "원격 SSH", "Prime/Firehose",
      "Sentry", "stock uploader", "Carrot 커뮤니티 공유", "기존 Athena 업로드 큐",
      "KA4 자동 검증", "고정 개인 NAS", "개인 NAS", "대시캠 전송", "Git 업데이트",
      "온라인 길찾기·지도", "독립적으로", "테더링 데이터", "자동 삭제되지 않습니다",
    ],
    zh: [
      "Athena", "cloudlog", "位置", "rlog/qlog/qcamera", "远程 SSH", "Prime/Firehose",
      "Sentry", "原生 uploader", "Carrot 社区共享", "旧 Athena 上传队列",
      "KA4 自动验证", "固定私人 NAS", "个人 NAS", "行车日志上传", "Git 更新",
      "在线路线/地图", "独立", "移动数据", "不会自动删除",
    ],
  };

  for (const [language, fragments] of Object.entries(requiredFragments)) {
    const strings = await translatedStrings(language);
    assert.equal(typeof strings.third_party_data_sharing_enable_title, "string", language);
    const confirmation = strings.third_party_data_sharing_enable_confirm;
    assert.equal(typeof confirmation, "string", language);
    for (const fragment of fragments) {
      assert.ok(confirmation.includes(fragment), `${language} confirmation: ${fragment}`);
    }
  }
});
