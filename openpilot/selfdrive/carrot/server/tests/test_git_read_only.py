import asyncio
import json
from types import SimpleNamespace

from openpilot.selfdrive.carrot.server.features.tools import dispatcher, jobs
from openpilot.selfdrive.carrot.server.services import git_status


def test_background_git_status_queries_remote_without_fetch(monkeypatch):
  calls = []
  responses = {
    ("rev-parse", "--is-inside-work-tree"): (0, "true"),
    ("rev-parse", "HEAD"): (0, "local-head"),
    ("branch", "--show-current"): (0, "dkcarrot-wip"),
    ("config", "--get", "branch.dkcarrot-wip.remote"): (0, "powtrix"),
    ("config", "--get", "branch.dkcarrot-wip.merge"): (0, "refs/heads/dkcarrot-wip"),
    ("ls-remote", "--heads", "powtrix", "refs/heads/dkcarrot-wip"): (
      0,
      "remote-head\trefs/heads/dkcarrot-wip",
    ),
    ("cat-file", "-e", "remote-head^{commit}"): (0, ""),
    ("rev-list", "--left-right", "--count", "HEAD...remote-head"): (0, "0 2"),
  }

  async def fake_git(args, timeout=git_status.GIT_TIMEOUT):
    del timeout
    calls.append(args)
    return responses[tuple(args)]

  monkeypatch.setattr(git_status, "_git", fake_git)

  status = asyncio.run(git_status._read_status())

  assert status["state"] == "ok"
  assert status["available"] is True
  assert status["behind"] == 2
  assert status["counts_exact"] is True
  assert status["target_head"] == "remote-head"
  assert any(args[0] == "ls-remote" for args in calls)
  assert not any(args[0] == "fetch" for args in calls)


def test_background_git_status_reports_unfetched_remote_change(monkeypatch):
  calls = []
  responses = {
    ("rev-parse", "--is-inside-work-tree"): (0, "true"),
    ("rev-parse", "HEAD"): (0, "local-head"),
    ("branch", "--show-current"): (0, "dkcarrot-wip"),
    ("config", "--get", "branch.dkcarrot-wip.remote"): (0, "origin"),
    ("config", "--get", "branch.dkcarrot-wip.merge"): (0, "refs/heads/dkcarrot-wip"),
    ("ls-remote", "--heads", "origin", "refs/heads/dkcarrot-wip"): (
      0,
      "new-head\trefs/heads/dkcarrot-wip",
    ),
    ("cat-file", "-e", "new-head^{commit}"): (1, "missing"),
  }

  async def fake_git(args, timeout=git_status.GIT_TIMEOUT):
    del timeout
    calls.append(args)
    return responses[tuple(args)]

  monkeypatch.setattr(git_status, "_git", fake_git)

  status = asyncio.run(git_status._read_status())

  assert status["state"] == "ok"
  assert status["behind"] == 1
  assert status["counts_exact"] is False
  assert status["target_head"] == "new-head"
  assert not any(args[0] == "fetch" for args in calls)


def test_async_branch_list_reads_remote_heads_without_fetch(monkeypatch):
  captured = []
  finished = []

  async def fake_capture_exec(cmd, *, cwd=None, timeout=None):
    del cwd, timeout
    captured.append(cmd)
    outputs = {
      ("git", "for-each-ref", "--format=%(refname:short)", "refs/heads"): "dkcarrot-wip",
      ("git", "branch", "--show-current"): "dkcarrot-wip",
      ("git", "remote"): "powtrix",
      ("git", "ls-remote", "--heads", "powtrix"): "old-head\trefs/heads/dkcarrot-wip\nnew-head\trefs/heads/new-remote-branch",
      ("git", "remote", "-v"): "powtrix https://example.invalid/repo.git (fetch)",
    }
    return 0, outputs[tuple(cmd)]

  async def fail_stream_exec(*args, **kwargs):
    raise AssertionError("branch listing must not run a mutating command")

  monkeypatch.setattr(jobs, "capture_exec", fake_capture_exec)
  monkeypatch.setattr(jobs, "stream_exec", fail_stream_exec)
  monkeypatch.setattr(jobs, "progress", lambda *args, **kwargs: None)
  monkeypatch.setattr(jobs, "append", lambda *args, **kwargs: None)
  monkeypatch.setattr(jobs, "finish", lambda job, **kwargs: finished.append(kwargs))
  monkeypatch.setattr(jobs, "get_branch_prefix", lambda: "c3")
  monkeypatch.setattr(dispatcher, "HARDWARE", SimpleNamespace(get_device_type=lambda: "tici"))

  asyncio.run(dispatcher.run_tool_job({"action": "git_branch_list", "payload": {}, "log": ""}))

  assert finished[0]["ok"] is True
  assert finished[0]["result"]["branches"] == ["dkcarrot-wip", "powtrix/dkcarrot-wip", "powtrix/new-remote-branch"]
  assert finished[0]["result"]["fetch"] == ""
  assert ["git", "ls-remote", "--heads", "powtrix"] in captured
  assert not any("fetch" in cmd for cmd in captured)


def test_sync_branch_list_reads_remote_heads_without_fetch(monkeypatch):
  commands = []

  def fake_run(cmd, **kwargs):
    del kwargs
    commands.append(cmd)
    outputs = {
      ("git", "for-each-ref", "--format=%(refname:short)", "refs/heads"): "dkcarrot-wip",
      ("git", "branch", "--show-current"): "dkcarrot-wip",
      ("git", "remote"): "powtrix",
      ("git", "ls-remote", "--heads", "powtrix"): "old-head\trefs/heads/dkcarrot-wip\nnew-head\trefs/heads/new-remote-branch",
      ("git", "remote", "-v"): "powtrix https://example.invalid/repo.git (fetch)",
    }
    return SimpleNamespace(returncode=0, stdout=outputs[tuple(cmd)], stderr="")

  monkeypatch.setattr(dispatcher.subprocess, "run", fake_run)
  monkeypatch.setattr(jobs, "get_branch_prefix", lambda: "c3")
  monkeypatch.setattr(dispatcher, "HARDWARE", SimpleNamespace(get_device_type=lambda: "tici"))

  response = asyncio.run(dispatcher.dispatch_sync(None, {"action": "git_branch_list"}))
  body = json.loads(response.body)

  assert body["ok"] is True
  assert body["branches"] == ["dkcarrot-wip", "powtrix/dkcarrot-wip", "powtrix/new-remote-branch"]
  assert body["fetch"] == ""
  assert ["git", "ls-remote", "--heads", "powtrix"] in commands
  assert not any("fetch" in cmd for cmd in commands)


def test_async_remote_checkout_fetches_only_selected_branch(monkeypatch):
  commands = []
  finished = []

  async def fake_capture_exec(cmd, *, cwd=None, timeout=None):
    del cwd, timeout
    assert cmd == ["git", "remote"]
    return 0, "powtrix"

  async def fake_stream_exec(job, cmd, *, cwd=None, timeout=None):
    del job, cwd, timeout
    commands.append(cmd)
    return 0

  monkeypatch.setattr(jobs, "capture_exec", fake_capture_exec)
  monkeypatch.setattr(jobs, "stream_exec", fake_stream_exec)
  monkeypatch.setattr(jobs, "progress", lambda *args, **kwargs: None)
  monkeypatch.setattr(jobs, "finish", lambda job, **kwargs: finished.append(kwargs))

  asyncio.run(dispatcher.run_tool_job({
    "action": "git_checkout",
    "payload": {
      "branch": "powtrix/new-remote-branch",
      "kind": "remote",
      "remote": "powtrix",
      "name": "new-remote-branch",
    },
    "log": "",
  }))

  assert commands[0] == [
    "git", "fetch", "powtrix",
    "+refs/heads/new-remote-branch:refs/remotes/powtrix/new-remote-branch",
  ]
  assert commands[1][0:2] == ["bash", "-lc"]
  assert finished[0]["ok"] is True


def test_sync_remote_checkout_fetches_only_selected_branch(monkeypatch):
  commands = []

  def fake_run(cmd, **kwargs):
    del kwargs
    commands.append(cmd)
    outputs = {
      ("git", "remote"): (0, "powtrix"),
      (
        "git", "fetch", "powtrix",
        "+refs/heads/new-remote-branch:refs/remotes/powtrix/new-remote-branch",
      ): (0, ""),
      ("git", "show-ref", "--verify", "--quiet", "refs/heads/new-remote-branch"): (1, ""),
      ("git", "switch", "-c", "new-remote-branch", "--track", "powtrix/new-remote-branch"): (0, ""),
    }
    rc, out = outputs[tuple(cmd)]
    return SimpleNamespace(returncode=rc, stdout=out, stderr="")

  monkeypatch.setattr(dispatcher.subprocess, "run", fake_run)

  response = asyncio.run(dispatcher.dispatch_sync(None, {
    "action": "git_checkout",
    "branch": "powtrix/new-remote-branch",
    "kind": "remote",
    "remote": "powtrix",
    "name": "new-remote-branch",
  }))
  body = json.loads(response.body)

  assert body["ok"] is True
  assert [
    "git", "fetch", "powtrix",
    "+refs/heads/new-remote-branch:refs/remotes/powtrix/new-remote-branch",
  ] in commands
  assert not any(cmd[:3] == ["git", "fetch", "--all"] for cmd in commands)
