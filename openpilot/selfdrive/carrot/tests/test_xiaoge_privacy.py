from __future__ import annotations

from openpilot.selfdrive.carrot import xiaoge_data
from openpilot.system.manager import process_config


class FakeParams:
  def __init__(self, *, master: bool = True, share: bool = True) -> None:
    self._dk_consent_generation = "generation-1"
    self.values = {
      "DkThirdPartyDataSharing": b"1" if master else b"0",
      "ShareData": share,
    }

  def get(self, key, *args, **kwargs):
    del args, kwargs
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))


def test_xiaoge_process_requires_share_data_and_master_consent():
  assert process_config.enable_xiaoge_data(False, FakeParams(master=True, share=True), None)
  assert not process_config.enable_xiaoge_data(False, FakeParams(master=False, share=True), None)
  assert not process_config.enable_xiaoge_data(False, FakeParams(master=True, share=False), None)


def test_xiaoge_packet_stops_before_payload_after_mid_send_revoke():
  params = FakeParams()
  broadcaster = object.__new__(xiaoge_data.XiaogeDataBroadcaster)
  broadcaster.params = params
  writes = []

  class FakeConnection:
    def sendall(self, data):
      writes.append(data)
      params._dk_consent_generation = "generation-2"

  generation = xiaoge_data.third_party_data_sharing_generation(params)
  assert generation is not None
  assert not broadcaster.send_packet_to_client(FakeConnection(), b"private-payload", generation)
  assert len(writes) == 1
  assert b"private-payload" not in writes


def test_xiaoge_packet_stops_between_bounded_payload_chunks_after_revoke():
  params = FakeParams()
  broadcaster = object.__new__(xiaoge_data.XiaogeDataBroadcaster)
  broadcaster.params = params
  writes = []

  class FakeConnection:
    def sendall(self, data):
      writes.append(bytes(data))
      if len(writes) == 2:
        params._dk_consent_generation = "generation-2"

  packet = b"x" * (xiaoge_data.CONSENT_STREAM_CHUNK_SIZE * 2 + 1)
  generation = xiaoge_data.third_party_data_sharing_generation(params)
  assert generation is not None
  assert not broadcaster.send_packet_to_client(FakeConnection(), packet, generation)
  assert len(writes) == 2
  assert len(writes[1]) == xiaoge_data.CONSENT_STREAM_CHUNK_SIZE
  assert sum(map(len, writes[1:])) < len(packet)
