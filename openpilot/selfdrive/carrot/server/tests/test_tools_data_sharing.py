from openpilot.selfdrive.carrot.server.features.tools import dispatcher


class _FakeParams:
  def __init__(self) -> None:
    self.writes: list[tuple[str, str]] = []

  def put_nonblocking(self, key: str, value: str) -> None:
    self.writes.append((key, value))


def test_manual_tmux_upload_is_not_queued_while_master_sharing_is_off(monkeypatch):
  params = _FakeParams()
  monkeypatch.setattr(dispatcher, "third_party_data_sharing_generation", lambda _params: None)

  assert dispatcher.queue_server_tmux_upload(params) is None
  assert params.writes == []


def test_manual_tmux_upload_request_is_bound_to_exact_master_generation(monkeypatch):
  params = _FakeParams()
  monkeypatch.setattr(
    dispatcher,
    "third_party_data_sharing_generation",
    lambda _params: "master-generation-1",
  )

  assert dispatcher.queue_server_tmux_upload(params) == "master-generation-1"
  assert params.writes == [
    ("CarrotException", "tmux_send:master-generation-1"),
  ]
