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
