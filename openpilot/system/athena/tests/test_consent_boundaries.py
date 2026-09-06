from __future__ import annotations

import json
import queue
import threading

from websocket import ABNF

from openpilot.system.athena import athenad


def _drain(target: queue.Queue) -> None:
  while True:
    try:
      target.get_nowait()
    except queue.Empty:
      return


def test_jsonrpc_revoked_after_dequeue_is_not_dispatched(monkeypatch):
  _drain(athenad.recv_queue)
  called = []
  athenad.dispatcher["dkConsentTest"] = lambda: called.append(True)
  monkeypatch.setattr(athenad, "third_party_data_sharing_generation_matches", lambda *_args: False)
  end_event = threading.Event()
  athenad.recv_queue.put_nowait(json.dumps({"method": "dkConsentTest", "jsonrpc": "2.0", "id": 1}))

  athenad.jsonrpc_handler(end_event, "generation-1")

  assert end_event.is_set()
  assert called == []


def test_jsonrpc_response_is_not_enqueued_after_revoke_during_dispatch(monkeypatch):
  _drain(athenad.recv_queue)
  _drain(athenad.send_queue)
  state = {"allowed": True}

  def revoke_during_dispatch():
    state["allowed"] = False
    return "private result"

  athenad.dispatcher["dkConsentTest"] = revoke_during_dispatch
  monkeypatch.setattr(
    athenad,
    "third_party_data_sharing_generation_matches",
    lambda *_args: state["allowed"],
  )
  end_event = threading.Event()
  athenad.recv_queue.put_nowait(json.dumps({"method": "dkConsentTest", "jsonrpc": "2.0", "id": 1}))

  athenad.jsonrpc_handler(end_event, "generation-1")

  assert end_event.is_set()
  assert athenad.send_queue.empty()


def test_ws_recv_does_not_enqueue_command_revoked_during_receive(monkeypatch):
  _drain(athenad.recv_queue)
  state = {"allowed": True}

  class FakeWebsocket:
    def recv_data(self, control_frame=True):
      del control_frame
      state["allowed"] = False
      return ABNF.OPCODE_TEXT, b'{"method":"echo"}'

  monkeypatch.setattr(
    athenad,
    "third_party_data_sharing_generation_matches",
    lambda *_args: state["allowed"],
  )
  end_event = threading.Event()

  athenad.ws_recv(FakeWebsocket(), end_event, "generation-1")

  assert end_event.is_set()
  assert athenad.recv_queue.empty()


def test_ws_send_stops_before_next_frame_after_revoke(monkeypatch):
  _drain(athenad.send_queue)
  _drain(athenad.low_priority_send_queue)
  state = {"allowed": True}
  frames = []

  class FakeWebsocket:
    def send_frame(self, frame):
      frames.append(frame)
      state["allowed"] = False

  monkeypatch.setattr(
    athenad,
    "third_party_data_sharing_generation_matches",
    lambda *_args: state["allowed"],
  )
  athenad.send_queue.put_nowait("x" * (athenad.WS_FRAME_SIZE * 2))
  end_event = threading.Event()

  athenad.ws_send(FakeWebsocket(), end_event, "generation-1")

  assert end_event.is_set()
  assert len(frames) == 1


def test_ws_proxy_recv_blocks_remote_bytes_revoked_during_receive(monkeypatch):
  state = {"allowed": True}

  class FakeWebsocket:
    sock = object()

    def recv(self):
      state["allowed"] = False
      return b"remote command"

    def close(self):
      pass

  class FakeSocket:
    def __init__(self):
      self.sent = []

    def sendall(self, data):
      self.sent.append(data)

    def close(self):
      pass

  local_sock = FakeSocket()
  signal_sock = FakeSocket()
  monkeypatch.setattr(athenad.select, "select", lambda *_args, **_kwargs: ([object()], [], []))
  monkeypatch.setattr(
    athenad,
    "third_party_data_sharing_generation_matches",
    lambda *_args: state["allowed"],
  )
  end_event = threading.Event()

  athenad.ws_proxy_recv(
    FakeWebsocket(), local_sock, signal_sock, end_event, threading.Event(), "generation-1",
  )

  assert end_event.is_set()
  assert local_sock.sent == []
