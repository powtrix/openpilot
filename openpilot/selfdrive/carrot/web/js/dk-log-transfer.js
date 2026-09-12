// Manual, same-origin downloads. No upload, background task, storage, or Blob buffering.
const strings = {
  ko: {
    back: "← 웹당근", title: "dk 로그전달", intro: "차량에서 받아 두고, 집에서 전달하세요. 별도 앱이나 백그라운드 실행 없이 사용합니다.",
    step1: "1. 차량 → 휴대폰", scope: "현재 보관된 문제 전후 원본 rlog와 설정·커밋 정보를 ZIP 하나로 받습니다. 전체 주행 백업이나 영상은 포함하지 않습니다.",
    privacy: "로그에는 위치·장치 식별정보가 포함될 수 있습니다. 본인의 핫스팟에서 정차 후 사용하고, 파일을 공개하거나 공식 서버에 올리지 마세요.",
    refresh: "목록 새로고침", selection: "전달할 진단 기록", download: "선택 로그를 휴대폰에 저장",
    downloadHelp: "묶음 준비 후 브라우저가 다운로드합니다. 완료 여부는 브라우저 다운로드 목록에서 확인하세요. 완료 전에는 디바이스 전원과 핫스팟을 유지하세요. 중단되면 다시 다운로드해야 합니다.",
    step2: "2. 휴대폰 → 집의 맥미니 / NAS", homeHelp: "집 Wi-Fi에 연결한 뒤 dk 수신 페이지를 열고, 다운로드 폴더의 .dklog.zip 파일을 선택하세요. 수신 주소는 맥미니에서 dk 수신기를 실행하면 표시됩니다. NAS에서는 별도 실행 환경 설정이 필요합니다.",
    verification: "수신기가 파일 검증 후 ‘수신 완료’를 표시해야 전달이 끝난 것입니다. 확인 전에는 휴대폰 파일을 지우지 마세요.",
    limits: "준비 중인 기록은 아직 받을 수 없습니다. ‘일부 구간 누락’ 기록도 확보된 원본은 전달하지만, 누락된 구간이 복구되는 것은 아닙니다. 현재 진단 보관 정책은 최대 10건·1 GiB·7일이며 더 일찍 교체될 수 있습니다.",
    localOnly: "이 페이지는 파일을 휴대폰에 내려줄 뿐, 외부 서버로 전송하거나 차량 제어·설정을 변경하지 않습니다.",
    loading: "기록 목록을 확인 중입니다…", empty: "현재 보관된 기록이 없습니다. 문제가 없었다는 뜻은 아닙니다.",
    error: "목록을 받지 못했습니다. 디바이스와 핫스팟 연결을 확인하고 다시 시도하세요.",
    loaded: "받을 기록을 선택하세요. 일부 구간 누락 여부를 확인해 주세요.",
    requested: "다운로드를 요청했습니다. 새 탭의 오류 또는 브라우저 다운로드 목록에서 완료 여부를 확인하세요.",
    choose: "파일이 있는 기록을 하나 이상 선택하세요.", selected: "선택", count: "건", ready: "확보 완료", partial: "일부 구간 누락", pending: "준비 중", noFiles: "확보된 파일 없음", missing: "누락 구간 수", unknown: "시각 미상",
    resume: "자동재출발", engage_warning: "인게이지 경고", curve: "커브", unwind: "조향 복원", braking: "제동",
  },
  en: {
    back: "← Carrot Web", title: "dk Log transfer", intro: "Save logs in the car and deliver them at home. No app installation or background service required.",
    step1: "1. Vehicle → phone", scope: "Download retained diagnostic-window full rlogs and settings/commit metadata in one ZIP. This is not an entire-drive backup and contains no video.",
    privacy: "Logs may contain location and device identifiers. Use your own hotspot while parked. Do not publish the file or upload it to an official server.",
    refresh: "Refresh list", selection: "Diagnostic captures to deliver", download: "Save selected logs to phone",
    downloadHelp: "The browser starts downloading after the bundle is prepared. Check completion in browser Downloads. Keep device power and the hotspot on until complete. Interrupted downloads must be restarted.",
    step2: "2. Phone → home Mac mini / NAS", homeHelp: "Join home Wi-Fi, open the dk receiver page, and choose the .dklog.zip file from Downloads. The receiver displays its address when started on your Mac mini. Running it on a NAS requires separate environment setup.",
    verification: "Delivery is complete only when the receiver verifies the files and reports success. Keep the phone copy until then.",
    limits: "Pending captures are not downloadable. Partial captures preserve available originals but cannot recover missing segments. Current retention: up to 10 captures, 1 GiB, 7 days; replacement may happen earlier.",
    localOnly: "This page only downloads to your phone. It never uploads to an external server or changes vehicle controls/settings.",
    loading: "Loading captures…", empty: "No retained captures. This does not mean there were no issues.", error: "Could not load captures. Check the device and hotspot connection, then retry.",
    loaded: "Choose captures and review any missing segments.", requested: "Download requested. Check the new tab for errors or browser Downloads for completion.",
    choose: "Select at least one capture containing files.", selected: "Selected", count: "captures", ready: "Files retained", partial: "Missing segments", pending: "Pending", noFiles: "No retained files", missing: "Missing segments", unknown: "Unknown time",
    resume: "Auto resume", engage_warning: "Engage warning", curve: "Curve", unwind: "Steering unwind", braking: "Braking",
  },
};

export function normalizeCaptures(payload) {
  if (payload?.schema !== 1 || payload?.local_only !== true || !Array.isArray(payload.captures) || payload.captures.length > 10) throw Error("Invalid list");
  const seen = new Set();
  return payload.captures.map((item) => {
    if (!/^[0-9a-f]{32}$/.test(item?.capture_id) || seen.has(item.capture_id) || !["ready", "partial", "pending"].includes(item.status)
        || !Array.isArray(item.files) || item.files.length > 3 || !Array.isArray(item.topics) || !Array.isArray(item.missing)) throw Error("Invalid capture");
    seen.add(item.capture_id);
    const bytes = item.files.reduce((total, file) => {
      if (!Number.isSafeInteger(file?.bytes) || file.bytes < 0 || file.bytes > 256 * 1024 ** 2) throw Error("Invalid size");
      return total + file.bytes;
    }, 0);
    return { id: item.capture_id, status: item.status, bytes, hasFiles: item.files.length > 0, available: item.status !== "pending" && item.files.length > 0,
      time: Number.isFinite(item.created_at) ? item.created_at : null,
      topics: item.topics.filter((topic) => ["resume", "engage_warning", "curve", "unwind", "braking"].includes(topic)), missing: item.missing.length };
  });
}

export function formatBytes(bytes) {
  return bytes >= 1024 ** 3 ? `${(bytes / 1024 ** 3).toFixed(2)} GiB` : `${(bytes / 1024 ** 2).toFixed(1)} MiB`;
}

export function mount(doc, fetcher = globalThis.fetch) {
  let language = (doc.defaultView.navigator.language || "").startsWith("ko") ? "ko" : "en";
  let captures = [];
  let selected = new Set();
  let busy = false;
  let statusKey = "loading";
  const byId = (id) => doc.getElementById(id);
  const text = (key) => strings[language][key];
  function selection() {
    const items = captures.filter((cap) => selected.has(cap.id) && cap.available);
    byId("selection-summary").textContent = `${text("selected")}: ${items.length} ${text("count")} · ${formatBytes(items.reduce((sum, cap) => sum + cap.bytes, 0))}`;
    byId("download").disabled = busy || items.length === 0;
  }
  function render() {
    doc.documentElement.lang = language;
    doc.querySelectorAll("[data-text]").forEach((element) => { element.textContent = text(element.dataset.text); });
    byId("language").textContent = language === "ko" ? "English" : "한국어";
    byId("status").textContent = text(statusKey);
    byId("refresh").disabled = busy;
    const list = byId("capture-list");
    list.replaceChildren();
    for (const cap of captures) {
      const label = doc.createElement("label");
      label.className = "capture";
      const input = doc.createElement("input");
      input.type = "checkbox"; input.name = "capture"; input.value = cap.id;
      input.checked = selected.has(cap.id); input.disabled = busy || !cap.available;
      input.addEventListener("change", () => { if (input.checked) selected.add(cap.id); else selected.delete(cap.id); selection(); });
      const detail = doc.createElement("span");
      const date = cap.time === null ? text("unknown") : new Date(cap.time * 1000).toLocaleString(language === "ko" ? "ko-KR" : "en-US");
      detail.textContent = `${date}\n${cap.topics.map(text).join(" · ")}\n${text(cap.status)} · ${formatBytes(cap.bytes)}${cap.missing ? ` · ${text("missing")}: ${cap.missing}` : ""}${cap.hasFiles ? "" : ` · ${text("noFiles")}`}`;
      label.append(input, detail); list.append(label);
    }
    selection();
  }
  async function refresh() {
    if (busy) return;
    busy = true; statusKey = "loading"; render();
    try {
      const response = await fetcher("/api/dk/diagnostics", { cache: "no-store", credentials: "same-origin" });
      if (!response.ok) throw Error("List unavailable");
      captures = normalizeCaptures(await response.json());
      selected = new Set(captures.filter((cap) => cap.available).map((cap) => cap.id));
      statusKey = captures.length ? "loaded" : "empty";
    } catch { captures = []; selected.clear(); statusKey = "error"; }
    finally { busy = false; render(); }
  }
  byId("language").addEventListener("click", () => { language = language === "ko" ? "en" : "ko"; render(); });
  byId("refresh").addEventListener("click", refresh);
  byId("download-form").addEventListener("submit", (event) => {
    if (busy || !captures.some((cap) => cap.available && selected.has(cap.id))) {
      event.preventDefault(); statusKey = "choose";
    } else statusKey = "requested";
    byId("status").textContent = text(statusKey);
  });
  return refresh();
}

if (typeof document !== "undefined") mount(document);
