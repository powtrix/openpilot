import assert from "node:assert/strict";
import test from "node:test";

import { commitSettingToggle } from "../src/features/settings/toggle_commit.js";

test("a cancelled consent restores the committed toggle without writing", async () => {
  const events = [];
  const result = await commitSettingToggle({
    next: 1,
    previous: 0,
    requiresConfirmation: true,
    confirm: async () => false,
    commit: async () => events.push("commit"),
    restore: (value) => events.push(["restore", value]),
  });

  assert.deepEqual({ ...result }, { committed: false, cancelled: true });
  assert.deepEqual(events, [["restore", 0]]);
});

test("a rejected write restores the last committed toggle", async () => {
  const restored = [];
  const result = await commitSettingToggle({
    next: 1,
    previous: 0,
    commit: async () => false,
    restore: (value) => restored.push(value),
  });

  assert.deepEqual({ ...result }, { committed: false, cancelled: false });
  assert.deepEqual(restored, [0]);
});

test("a thrown write restores before propagating the failure", async () => {
  const restored = [];
  await assert.rejects(() => commitSettingToggle({
    next: 0,
    previous: 1,
    commit: async () => { throw new Error("HTTP 403"); },
    restore: (value) => restored.push(value),
  }), /HTTP 403/);
  assert.deepEqual(restored, [1]);
});

test("an accepted write leaves the rendered value alone", async () => {
  let restored = false;
  const result = await commitSettingToggle({
    next: 1,
    previous: 0,
    commit: async () => true,
    restore: () => { restored = true; },
  });

  assert.deepEqual({ ...result }, { committed: true, cancelled: false });
  assert.equal(restored, false);
});
