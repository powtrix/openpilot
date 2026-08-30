from dataclasses import replace
from types import SimpleNamespace

from tools.car_porting.ka4_stock_scc_probe import (  # noqa: TID251
  ProbeAnalyzer,
  classify_can_source,
  run_demo,
)


def test_classify_panda_tx_echo_sources() -> None:
  assert classify_can_source("can", 0x02) == ("vehicle_rx", 2)
  assert classify_can_source("can", 0x82) == ("tx_returned", 2)
  assert classify_can_source("can", 0xC2) == ("tx_rejected", 2)
  assert classify_can_source("can", 0x86) == ("tx_returned", 6)
  assert classify_can_source("can", 0xC6) == ("tx_rejected", 6)
  assert classify_can_source("sendcan", 0x02) == ("send_request", 2)


def test_demo_pass_proves_exact_30_second_behavior() -> None:
  report = run_demo("pass").report()

  assert report["overallVerdict"] == "PASS"
  episode = report["stopEpisodes"][0]
  assert 29.9 <= episode["warningPeriods"][0]["startAfterStop"] <= 30.1
  assert episode["prerequisites"]["exact11GroupRearmSchedule"]
  assert episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]
  assert episode["prerequisites"]["send0x1aaUsesExpectedBus"]
  assert episode["prerequisites"]["stockAndSendBusesUseSafetyPanda"]
  assert episode["prerequisites"]["allScc0x1a0CrcValid"]
  assert episode["prerequisites"]["allRaw0x1aaCrcValid"]
  assert episode["prerequisites"]["allRearm0x1aaCrcValid"]
  assert episode["prerequisites"]["latestStockNonButtonFieldsPreserved"]
  assert episode["prerequisites"]["sourcePlusOneCounterAllFrames"]
  assert episode["prerequisites"]["wheelStandstillAndVEgoZeroAllSamples"]
  assert report["carParams"]["ka4StockSccGate"]["canFdAltButtons"]
  assert report["carParams"]["ka4StockSccGate"]["hyundaiCanfdSafetyAltButtons"]
  assert all(group["allReturned"] for group in episode["resGroups"])
  assert all(group["counterPatternMatchesCurrentController"] for group in episode["resGroups"])
  assert sum(match["status"] == "returned" for match in report["txMatches"]) == 33
  assert sum(match["status"] == "rejected" for match in report["txMatches"]) == 0


def test_demo_pass_accepts_second_panda_global_bus_offset() -> None:
  report = run_demo("pass", bus_offset=4).report()

  assert report["overallVerdict"] == "PASS"
  assert report["carParams"]["pandaBusOffset"] == 4
  episode = report["stopEpisodes"][0]
  assert episode["busLayout"] == {
    "stockBus": 4,
    "pandaBusOffset": 4,
    "canFdHda2": False,
    "expectedSendBus": 6,
    "observedSendBuses": [6],
  }
  assert episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]
  assert episode["prerequisites"]["send0x1aaUsesExpectedBus"]
  assert episode["prerequisites"]["stockAndSendBusesUseSafetyPanda"]


def test_hda1_reports_raw_warning_and_masked_return_as_distinct_dbc_frames() -> None:
  report = run_demo("pass").report()
  cluster = report["clusterCanEvidence"]
  streams = cluster["streams"]

  raw_adrv = next(stream for stream in streams
                  if stream["addressHex"] == "0x161" and stream["origin"] == "vehicle_rx")
  sent_adrv = next(stream for stream in streams
                   if stream["addressHex"] == "0x161" and stream["origin"] == "send_request")
  returned_adrv = next(stream for stream in streams
                       if stream["addressHex"] == "0x161" and stream["origin"] == "tx_returned")
  assert (raw_adrv["bus"], sent_adrv["bus"], returned_adrv["bus"]) == (2, 0, 0)
  assert raw_adrv["valueCounts"]["5"] > 0
  assert sent_adrv["valueCounts"] == {"0": sent_adrv["count"]}
  assert returned_adrv["valueCounts"] == {"0": returned_adrv["count"]}
  assert raw_adrv["transitions"][-1]["value"] == 5
  assert raw_adrv["transitions"][-1]["dataHex"] != sent_adrv["transitions"][0]["dataHex"]

  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["warningPathObservation"] == "hostReplacementMaskedRawWarning"
  assert evidence["warningOutputPairCounts"]["maskedByReturnedFrame"] > 0
  assert evidence["hdaReplacementComparison"]["stateMismatches"] == 0
  assert evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]
  res_times = [event["t"] for event in report["correlatedTimeline"] if event["event"] == "resHostRequest"]
  raw_warning = next(event for event in report["correlatedTimeline"]
                     if event["event"] == "clusterCanTransition"
                     and event["origin"] == "vehicle_rx" and event["signal"] == "ALERTS_5"
                     and event["value"] == 5)
  assert max(res_times) < raw_warning["t"]


def test_hda2_second_panda_reports_unmodified_raw_warning_topology() -> None:
  report = run_demo("pass", bus_offset=4, hda2=True).report()
  assert report["overallVerdict"] == "PASS"
  topology = report["clusterCanEvidence"]["topology"]
  assert topology["canFdHda2"]
  assert topology["expectedRawCameraBus"] == 6
  assert topology["expectedHostReplacementBus"] is None

  streams = report["clusterCanEvidence"]["streams"]
  raw_addresses = {(stream["addressHex"], stream["bus"]) for stream in streams
                   if stream["origin"] == "vehicle_rx"}
  assert ("0x161", 6) in raw_addresses
  assert ("0x1E0", 6) in raw_addresses
  assert not any(stream["addressHex"] in ("0x161", "0x1E0") and stream["origin"] == "send_request"
                 for stream in streams)
  evidence = report["stopEpisodes"][0]["clusterEvidence"]
  assert evidence["warningPathObservation"] == "hda2RawUnmodifiedForwardingTopology"
  assert evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]


def test_cluster_rejection_preserves_raw_dbc_value_and_global_bus() -> None:
  from opendbc.can import CANPacker

  analyzer = ProbeAnalyzer("test", "cluster-reject")
  message = CANPacker("hyundai_canfd_generated").make_can_msg("ADRV_0x161", 4, {
    "COUNTER": 7,
    "ALERTS_5": 5,
  })
  analyzer.feed_can("sendcan", 10.0, 4, message[0], message[1])
  analyzer.feed_can("can", 10.002, 0xC4, message[0], message[1])
  report = analyzer.report()

  match = report["clusterCanEvidence"]["txMatches"][0]
  assert match["status"] == "rejected"
  assert match["bus"] == 4
  assert match["ALERTS_5"] == 5
  assert match["dataHex"] == message[1].hex()
  rejected_stream = next(stream for stream in report["clusterCanEvidence"]["streams"]
                         if stream["origin"] == "tx_rejected")
  assert rejected_stream["bus"] == 4
  assert rejected_stream["valueCounts"] == {"5": 1}


def test_demo_rejected_tx_and_early_warning_fails() -> None:
  report = run_demo("fail").report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["warningPeriods"][0]["startAfterStop"] < 3.1
  assert any("rejected" in reason for reason in episode["reasons"])
  assert sum(match["status"] == "rejected" for match in report["txMatches"]) == 33


def test_demo_missing_tx_is_inconclusive() -> None:
  report = run_demo("missing-tx").report()

  assert report["overallVerdict"] == "FAIL"
  # The synthetic missing-TX trace also exposes the OEM warning at 3 seconds,
  # so it is direct negative evidence, not merely an absent request.
  assert not report["stopEpisodes"][0]["prerequisites"]["exact11GroupRearmSchedule"]


def test_empty_analyzer_is_inconclusive() -> None:
  report = ProbeAnalyzer("test", "empty").report()
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["stopEpisodes"] == []


def test_extra_res_after_27_seconds_is_direct_failure() -> None:
  analyzer = run_demo("pass")
  original = next(sample for sample in analyzer.buttons if sample.origin == "send_request")
  request_t = 1_000.0 + 28.0
  data = bytes.fromhex(original.data_hex)
  analyzer.feed_can("sendcan", request_t, 2, original.address, data)
  analyzer.feed_can("can", request_t + 0.002, 0x82, original.address, data)

  report = analyzer.report()
  assert report["overallVerdict"] == "FAIL"
  assert "continued after 27.00s" in report["stopEpisodes"][0]["reasons"][0]


def test_crc_failure_is_direct_failure() -> None:
  analyzer = run_demo("pass")
  original = analyzer.scc[500]
  invalid = bytearray.fromhex(original.data_hex)
  invalid[0] ^= 1
  analyzer.feed_can("can", original.t + 0.001, 0, 0x1A0, bytes(invalid))

  report = analyzer.report()
  assert report["overallVerdict"] == "FAIL"
  assert "CRC" in report["stopEpisodes"][0]["reasons"][0]


def test_nonzero_vego_during_reported_standstill_is_direct_failure() -> None:
  analyzer = run_demo("pass")
  index = next(index for index, state in enumerate(analyzer.states) if state.stop_active and state.t > 1_010.0)
  analyzer.states[index] = replace(analyzer.states[index], v_ego=0.2)

  report = analyzer.report()
  assert report["overallVerdict"] == "FAIL"
  assert "nonzero vEgo" in report["stopEpisodes"][0]["reasons"][0]


def test_vehicle_motion_after_long_stop_with_stationary_lead_is_direct_failure() -> None:
  analyzer = run_demo("pass")
  final_stop_state = analyzer.states[-1]
  analyzer.feed_state(
    final_stop_state.t + 0.02,
    SimpleNamespace(
      standstill=False,
      vEgo=0.2,
      canValid=True,
      brakePressed=False,
      gasPressed=False,
      brakeHoldActive=False,
      parkingBrake=False,
      accFaulted=False,
      cruiseState=SimpleNamespace(enabled=True, standstill=False),
    ),
  )

  report = analyzer.report()
  assert report["overallVerdict"] == "FAIL"
  assert report["stopEpisodes"][0]["potentialFalseStart"]
  assert "potential false start" in report["stopEpisodes"][0]["reasons"][0]


def test_changed_non_button_field_cannot_pass() -> None:
  analyzer = run_demo("pass")
  index = next(index for index, sample in enumerate(analyzer.buttons) if sample.origin == "send_request")
  request = analyzer.buttons[index]
  analyzer.buttons[index] = replace(
    request,
    non_button_values=request.non_button_values + (("NOT_FROM_STOCK", 1),),
  )

  report = analyzer.report()
  assert report["overallVerdict"] == "FAIL"
  assert "non-button field" in report["stopEpisodes"][0]["reasons"][0]
