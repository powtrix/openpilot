import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { formatBytes, mount, normalizeCaptures } from "../js/dk-log-transfer.js";

const id = "a".repeat(32);
const capture = (extra = {}) => ({ capture_id: id, status: "ready", created_at: 1000, files: [{ bytes: 3 }], topics: ["braking"], missing: [], ...extra });
const payload = (captures = [capture()]) => ({ schema: 1, local_only: true, captures });

test("list separates available partial, pending, and empty data", () => {
  const values = normalizeCaptures(payload([capture(), capture({ capture_id: "b".repeat(32), status: "partial", missing: [2] }),
    capture({ capture_id: "c".repeat(32), status: "pending" }), capture({ capture_id: "d".repeat(32), files: [] })]));
  assert.deepEqual(values.map((item) => item.available), [true, true, false, false]);
  assert.equal(values[1].missing, 1);
  assert.equal(values[2].hasFiles, true);
});

test("hostile or oversized list entries are rejected", () => {
  for (const input of [payload(Array(11).fill(capture())), payload([capture(), capture()]), payload([capture({ capture_id: "../" })]),
    payload([capture({ files: [{ bytes: -1 }] })]), payload([capture({ files: [{ bytes: 257 * 1024 ** 2 }] })]),
    payload([capture({ status: "normal" })]), { ...payload(), local_only: false }]) assert.throws(() => normalizeCaptures(input));
  assert.equal(formatBytes(1024 ** 3), "1.00 GiB");
  assert.equal(formatBytes(1024 ** 2), "1.0 MiB");
});

function fakeDocument() {
  class Element {
    constructor() { this.children = []; this.listeners = {}; this.dataset = {}; this.disabled = false; }
    addEventListener(name, callback) { this.listeners[name] = callback; }
    append(...children) { this.children.push(...children); }
    replaceChildren(...children) { this.children = children; }
  }
  const elements = new Map();
  return { elements, documentElement: {}, defaultView: { navigator: { language: "ko-KR" } },
    getElementById(name) { if (!elements.has(name)) elements.set(name, new Element()); return elements.get(name); },
    querySelectorAll() { return []; }, createElement() { return new Element(); } };
}

test("manual submit uses browser download and never marks it complete", async () => {
  const doc = fakeDocument();
  const requests = [];
  await mount(doc, async (...args) => { requests.push(args); return { ok: true, json: async () => payload() }; });
  assert.equal(requests.length, 1);
  assert.equal(requests[0][0], "/api/dk/diagnostics");
  assert.equal(doc.getElementById("download").disabled, false);
  let blocked = false;
  doc.getElementById("download-form").listeners.submit({ preventDefault() { blocked = true; } });
  assert.equal(blocked, false);
  assert.match(doc.getElementById("status").textContent, /요청했습니다/);
  const input = doc.getElementById("capture-list").children[0].children[0];
  input.checked = false; input.listeners.change();
  assert.equal(doc.getElementById("download").disabled, true);
  doc.getElementById("download-form").listeners.submit({ preventDefault() { blocked = true; } });
  assert.equal(blocked, true);
  assert.equal(requests.length, 1);
});

test("failed requests clear old selections and show an explicit error", async () => {
  const doc = fakeDocument();
  let failed = false;
  await mount(doc, async () => { if (failed) throw Error("offline"); return { ok: true, json: async () => payload() }; });
  failed = true;
  await doc.getElementById("refresh").listeners.click();
  assert.equal(doc.getElementById("download").disabled, true);
  assert.equal(doc.getElementById("capture-list").children.length, 0);
  assert.match(doc.getElementById("status").textContent, /받지 못했습니다/);
});

test("no-file and pending-only captures cannot submit all-captures by accident", async () => {
  const doc = fakeDocument();
  await mount(doc, async () => ({ ok: true, json: async () => payload([capture({ status: "pending" })]) }));
  assert.equal(doc.getElementById("download").disabled, true);
  assert.doesNotMatch(doc.getElementById("capture-list").children[0].children[1].textContent, /확보된 파일 없음/);
});

test("page is discoverable and contains no third-party or in-memory archive upload code", async () => {
  const html = await readFile(new URL("../dk-log-transfer.html", import.meta.url), "utf8");
  const js = await readFile(new URL("../js/dk-log-transfer.js", import.meta.url), "utf8");
  const index = await readFile(new URL("../index.html", import.meta.url), "utf8");
  assert.match(index, /id="dkLogTransferLink"[^>]*href="\/dk-logs"/);
  assert.match(html, /action="\/api\/dk\/diagnostics\/bundle" method="get"/);
  assert.doesNotMatch(html + js, /https?:\/\/|\.blob\(|localStorage|serviceWorker|setInterval|sendBeacon/);
  assert.match(html, /전체 주행 백업이나 영상은 포함하지 않습니다/);
  assert.match(html, /중단되면 다시 다운로드/);
});
