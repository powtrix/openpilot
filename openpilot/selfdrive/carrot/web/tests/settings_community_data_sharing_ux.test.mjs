import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const webRoot = new URL("../", import.meta.url);

async function read(relativePath) {
  return readFile(new URL(relativePath, webRoot), "utf8");
}

test("the community data toggle requires its own explicit confirmation", async () => {
  const source = await read("js/pages/setting.js");
  assert.match(source, /COMMUNITY_DATA_SHARING_PARAM = "CarrotCommunityDataSharing"/);
  assert.match(source, /function confirmCommunityDataSharingEnable\(\)/);
  assert.match(source, /name === COMMUNITY_DATA_SHARING_PARAM[\s\S]{0,180}!communityConsentConfirmed/);
  assert.match(source, /requiresCommunityConfirmation = name === COMMUNITY_DATA_SHARING_PARAM/);
  assert.match(source, /confirmCommunityDataSharingEnable/);
  assert.match(source, /communityConsentConfirmed: requiresCommunityConfirmation/);
  assert.match(source, /name === COMMUNITY_DATA_SHARING_PARAM && communityConsentConfirmed/);
  assert.match(source, /paramCommitOptions\.webConsent = true/);
});

test("all supported languages disclose the community transfer scope and KA4 independence", async () => {
  const requiredFragments = {
    en: [
      "device identifiers", "local network address", "all setting values", "automatic onroad and exception tmux",
      "bundled Discord", "mobile data", "user-configured NAS", "KA4 automatic validation upload",
      "private WPA2/WPA3",
    ],
    ko: [
      "장치 식별자", "로컬 네트워크 주소", "전체 설정값", "자동 onroad·예외 tmux", "기본 Discord",
      "모바일 데이터", "직접 지정한 NAS", "KA4 자동 검증 로그 전송", "WPA2/WPA3",
    ],
    zh: [
      "设备标识", "局域网地址", "全部设置值", "自动行驶中/异常 tmux", "内置 Discord", "移动数据",
      "自行配置的 NAS", "KA4 自动验证日志", "WPA2/WPA3",
    ],
  };

  for (const [language, fragments] of Object.entries(requiredFragments)) {
    const source = await read(`js/translations/${language}.js`);
    assert.match(source, /community_data_sharing_enable_title:/, language);
    assert.match(source, /community_data_sharing_enable_confirm:/, language);
    for (const fragment of fragments) assert.ok(source.includes(fragment), `${language}: ${fragment}`);
  }
});
