import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const webRoot = new URL("../", import.meta.url);

async function read(relativePath) {
  return readFile(new URL(relativePath, webRoot), "utf8");
}

test("the validation toggle confirms consent and reads only the sanitized status endpoint", async () => {
  const source = await read("js/pages/setting.js");
  assert.match(source, /VALIDATION_AUTO_UPLOAD_PARAM = "CarrotValidationAutoUpload"/);
  assert.match(source, /validation_upload_enable_confirm/);
  assert.match(source, /\/api\/dashcam\/validation-upload\/status/);
  assert.match(source, /settingToggleRuntime\.commit/);
  assert.match(source, /name === VALIDATION_AUTO_UPLOAD_PARAM[\s\S]{0,180}!validationConsentConfirmed/);
  assert.match(source, /confirmValidationUploadEnable/);
  assert.match(source, /syncSettingControlState\(el, previous\)/);
  assert.doesNotMatch(source, /validationUploadStatus[^\n]*(route|receiver|deviceId|captureId|lastError)/i);
});

test("an authoritative live refresh makes an automatic disable the next consent baseline", async () => {
  const source = await read("js/pages/setting.js");
  const functionSource = source.match(
    /function applyRestoredSettingValuesToRenderedItems\(values, options = \{\}\) \{[\s\S]*?\n\}/,
  )?.[0];
  assert.ok(functionSource, "live-refresh helper is present");

  const valueButton = { dataset: { rawValue: "1", committedValue: "1" } };
  const row = {
    dataset: { settingName: "CarrotValidationAutoUpload" },
    classList: { add() {}, remove() {} },
    querySelector(selector) {
      return selector === ".val" ? valueButton : null;
    },
  };
  const document = {
    querySelectorAll() {
      return [row];
    },
  };
  const syncSettingControlState = (_row, value) => {
    valueButton.dataset.rawValue = String(value);
  };
  const applyLiveValues = Function(
    "document",
    "syncSettingControlState",
    `return (${functionSource});`,
  )(document, syncSettingControlState);

  // The service completes or expires the campaign and changes Params 1 -> 0.
  assert.equal(applyLiveValues({ CarrotValidationAutoUpload: 0 }, { animate: false }), true);
  assert.equal(valueButton.dataset.rawValue, "0");
  assert.equal(valueButton.dataset.committedValue, "0");

  // The toggle's actual condition must now recognize the next 0 -> 1 edge.
  const next = 1;
  const previous = valueButton.dataset.committedValue ?? valueButton.dataset.rawValue;
  const requiresConfirmation = row.dataset.settingName === "CarrotValidationAutoUpload"
    && next === 1
    && String(previous) !== "1";
  assert.equal(requiresConfirmation, true);
});

test("all supported languages carry the complete one-time consent warning", async () => {
  const requiredFragments = {
    en: ["up to 3 full rlogs per event capture", "14 captures / 42 full rlogs", "5 captures / 750 MiB", "cumulative campaign uploads and retry traffic can exceed", "1 GiB per-device daily limit", "next day", "precise location", "not be asked again for each log", "WPA2/WPA3", "mobile data", "not deleted automatically"],
    ko: ["이벤트 캡처마다 full rlog 최대 3개", "14개 캡처/42개 full rlog", "5개 캡처/750 MiB", "누적 전송량이나 재시도 데이터 사용량", "장치별 일일 한도는 1 GiB", "다음 날", "정확한 위치", "로그마다 다시 확인하지 않습니다", "WPA2/WPA3", "모바일 데이터", "자동 삭제되지 않습니다"],
    zh: ["每个符合条件的事件捕获", "最多 3 个完整 rlog", "14 个捕获/42 个完整 rlog", "5 个捕获/750 MiB", "累计上传量和重试流量", "每日 1 GiB", "第二天", "精确位置", "不会逐个日志再次询问", "WPA2/WPA3", "移动数据", "不会自动删除"],
  };

  for (const [language, fragments] of Object.entries(requiredFragments)) {
    const source = await read(`js/translations/${language}.js`);
    assert.match(source, /validation_upload_enable_confirm:/, language);
    assert.match(source, /validation_upload_status_uploading:/, language);
    for (const fragment of fragments) assert.ok(source.includes(fragment), `${language}: ${fragment}`);
  }
});

test("confirmed consent writes fetch and attach a one-use same-origin session", async () => {
  const source = await read("js/shared/api.js");
  assert.match(source, /"Content-Type": "application\/json"/);
  assert.match(source, /"X-Carrot-Web-Request": "1"/);
  assert.match(source, /\/api\/web-consent\/session/);
  assert.match(source, /headers\["X-Carrot-Web-Consent"\]/);
  assert.match(source, /options\.webConsent === true/);

  const settingsSource = await read("js/pages/setting.js");
  assert.match(settingsSource, /paramCommitOptions\.webConsent = true/);
});
