from dataclasses import replace
from types import SimpleNamespace

import pytest

from tools.car_porting.ka4_stock_scc_probe import (  # noqa: TID251
  ADRV_0X161_ADDRESS,
  BUTTON_RES_ACCEL,
  CRUISE_BUTTONS_ALT_ADDRESS,
  LFAHDA_CLUSTER_ADDRESS,
  ProbeAnalyzer,
  RECOVERY_REARM_GROUP_STARTS,
  REGULAR_REARM_GROUP_STARTS,
  SCHEDULE_MODE_INITIAL_RECOVERY,
  SCHEDULE_MODE_MIXED,
  SCHEDULE_MODE_REGULAR,
  classify_can_source,
  run_demo,
  summarize_car_params,
)


def test_classify_panda_tx_echo_sources() -> None:
  assert classify_can_source("can", 0x02) == ("vehicle_rx", 2)
  assert classify_can_source("can", 0x82) == ("tx_returned", 2)
  assert classify_can_source("can", 0xC2) == ("tx_rejected", 2)
  assert classify_can_source("can", 0x86) == ("tx_returned", 6)
  assert classify_can_source("can", 0xC6) == ("tx_rejected", 6)
  assert classify_can_source("sendcan", 0x02) == ("send_request", 2)


@pytest.mark.parametrize("fingerprint", ("KIA_CARNIVAL_4TH_GEN", "KIA CARNIVAL 4TH GEN"))
def test_probe_accepts_current_and_legacy_ka4_fingerprint_values(fingerprint: str) -> None:
  report = summarize_car_params(SimpleNamespace(carFingerprint=fingerprint))
  assert report["ka4StockSccGate"]["fingerprintIsKa4"]


@pytest.mark.parametrize(
  "schedule_mode,button_source_phase_frames,expected_schedules",
  [
    pytest.param(SCHEDULE_MODE_REGULAR, 0, REGULAR_REARM_GROUP_STARTS, id="regular-phase-0"),
    pytest.param(SCHEDULE_MODE_REGULAR, 1, REGULAR_REARM_GROUP_STARTS, id="regular-phase-1"),
    pytest.param(SCHEDULE_MODE_INITIAL_RECOVERY, 0, RECOVERY_REARM_GROUP_STARTS, id="recovery-phase-0"),
    pytest.param(SCHEDULE_MODE_INITIAL_RECOVERY, 1, RECOVERY_REARM_GROUP_STARTS, id="recovery-phase-1"),
  ],
)
def test_demo_pass_proves_exact_supported_30_second_behavior(
    schedule_mode: str,
    button_source_phase_frames: int,
    expected_schedules: tuple[tuple[float, ...], ...],
) -> None:
  report = run_demo(
    "pass",
    schedule_mode=schedule_mode,
    button_source_phase_frames=button_source_phase_frames,
  ).report()

  assert report["overallVerdict"] == "PASS"
  episode = report["stopEpisodes"][0]
  assert episode["scheduleMode"] == schedule_mode
  assert episode["clusterEvidence"]["firstRawAccelerateWarningAfterStop"] == 30.0
  first_info_display_4 = episode["infoDisplay4Periods"][0]["startAfterStop"]
  if schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY:
    assert first_info_display_4 == 0.0
  else:
    assert first_info_display_4 == 30.0
  schedule = episode["returnedSchedule"]
  assert schedule["actualGroupStarts"] == list(expected_schedules[button_source_phase_frames])
  assert schedule["matchingButtonSourcePhaseFrames"] == [button_source_phase_frames]
  assert schedule["matchedButtonSourcePhaseFrames"] == button_source_phase_frames
  assert schedule["exactSupportedRearmSchedule"]
  assert episode["prerequisites"]["exactSupportedRearmSchedule"]
  assert episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]
  assert episode["prerequisites"]["send0x1aaUsesExpectedBus"]
  assert episode["prerequisites"]["stockAndSendBusesUseSafetyPanda"]
  assert episode["prerequisites"]["allScc0x1a0CrcValid"]
  assert episode["prerequisites"]["allRaw0x1aaCrcValid"]
  assert episode["prerequisites"]["allRearm0x1aaCrcValid"]
  assert episode["prerequisites"]["latestStockNonButtonFieldsPreserved"]
  assert episode["prerequisites"]["sourcePlusOneCounterAllFrames"]
  assert episode["prerequisites"]["sourceTimestampsFreshSequentialAllGroups"]
  assert episode["prerequisites"]["sourceCountersFreshSequentialAllGroups"]
  assert episode["prerequisites"]["emittedCountersFreshSequentialAllGroups"]
  assert episode["prerequisites"][
    "rawPostBurstSameCounterAndNextReleaseCandidatesObservedAllGroups"
  ]
  assert all(group["postBurstSameCounterThenNextCounterObserved"] for group in episode["resGroups"])
  assert episode["prerequisites"]["wheelStandstillAndRawSpeedNearZeroAllSamples"]
  assert all(coverage["continuous"] for coverage in episode["streamCoverage"].values())
  assert report["carParams"]["ka4StockSccGate"]["canFdAltButtons"]
  assert report["carParams"]["ka4StockSccGate"]["hyundaiCanfdSafetyAltButtons"]
  assert all(group["allReturned"] for group in episode["resGroups"])
  assert all(group["counterPatternMatchesCurrentController"] for group in episode["resGroups"])
  assert all(group["frameSpacingMs"] == [20.0, 20.0] for group in episode["resGroups"])
  expected_request_count = len(expected_schedules[button_source_phase_frames]) * 3
  assert sum(match["status"] == "returned" for match in report["txMatches"]) == expected_request_count
  assert sum(match["status"] == "rejected" for match in report["txMatches"]) == 0


@pytest.mark.parametrize(
  "schedule_mode,button_source_phase_frames,expected_schedules",
  [
    pytest.param(SCHEDULE_MODE_REGULAR, 0, REGULAR_REARM_GROUP_STARTS, id="regular-phase-0"),
    pytest.param(SCHEDULE_MODE_REGULAR, 1, REGULAR_REARM_GROUP_STARTS, id="regular-phase-1"),
    pytest.param(SCHEDULE_MODE_INITIAL_RECOVERY, 0, RECOVERY_REARM_GROUP_STARTS, id="recovery-phase-0"),
    pytest.param(SCHEDULE_MODE_INITIAL_RECOVERY, 1, RECOVERY_REARM_GROUP_STARTS, id="recovery-phase-1"),
  ],
)
def test_probe_schedule_constants_track_real_controller_replay(
    schedule_mode: str,
    button_source_phase_frames: int,
    expected_schedules: tuple[tuple[float, ...], ...],
) -> None:
  from opendbc.car.hyundai.tests.test_stock_scc_can_replay import Ka4StockSccReplay, pulse_groups

  replay = Ka4StockSccReplay(alt_buttons=True, button_phase_frames=button_source_phase_frames)
  if schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY:
    replay.warning_deadline = 0

  for frame in range(2701):
    replay.step(frame)

  groups = pulse_groups([message.frame for message in replay.injected])
  assert tuple(group[0] / 100.0 for group in groups) == expected_schedules[button_source_phase_frames]
  assert all(group == [group[0], group[0] + 2, group[0] + 4] for group in groups)


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


def test_hda1_host_replacement_preserves_raw_warning_in_returned_dbc_frames() -> None:
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
  assert sent_adrv["valueCounts"]["5"] == raw_adrv["valueCounts"]["5"]
  assert returned_adrv["valueCounts"]["5"] == raw_adrv["valueCounts"]["5"]
  assert raw_adrv["transitions"][-1]["value"] == 5
  assert sent_adrv["transitions"][-1]["value"] == 5
  assert returned_adrv["transitions"][-1]["value"] == 5

  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["warningPathObservation"] == "hostReplacementForwardedRawWarning"
  assert evidence["warningOutputPairCounts"]["maskedByReturnedFrame"] == 0
  assert evidence["warningOutputPairCounts"]["unpairedRawWarningFrames"] == 0
  assert evidence["warningOutputPairCounts"]["forwardedByReturnedFrame"] > 0
  assert evidence["warningPreservation"]["pathPreservesRawWarning"]
  assert episode["prerequisites"]["rawAdrvWarningPreservedAcrossActiveTopology"]
  assert evidence["hdaReplacementComparison"]["stateMismatches"] == 0
  assert evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]
  res_times = [event["t"] for event in report["correlatedTimeline"] if event["event"] == "resHostRequest"]
  raw_warning = next(event for event in report["correlatedTimeline"]
                     if event["event"] == "clusterCanTransition"
                     and event["origin"] == "vehicle_rx" and event["signal"] == "ALERTS_5"
                     and event["value"] == 5)
  assert max(res_times) < raw_warning["t"]


def test_sparse_hda1_host_replacement_streams_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.standstill)

  # Keep one LFAHDA host request/return pair near the stop boundary and only
  # the warning-era ADRV replacements. The raw camera streams remain complete,
  # so a presence-only check would incorrectly certify this capture.
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can
    if not (
      sample.origin in ("send_request", "tx_returned")
      and (
        (sample.address == LFAHDA_CLUSTER_ADDRESS and sample.t > stop_start + 0.01)
        or (sample.address == ADRV_0X161_ADDRESS and sample.t < stop_start + 29.99)
      )
    )
  ]

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert not evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]
  assert not all(coverage["continuous"] for coverage in evidence["hostReplacementCoverage"].values())
  assert not evidence["hostReplacementPairing"]["exactAdrvRawRequestReturnedPairing"]
  assert not evidence["hostReplacementPairing"]["exactLfaHdaRawRequestReturnedPairing"]
  assert not episode["prerequisites"]["hda1HostReplacementStreamsContinuousOrHda2NoReplacement"]
  assert not episode["prerequisites"][
    "hda1RawAdrvExactlyPairedWithHostRequestAndReturnOrHda2NoReplacement"
  ]
  assert not episode["prerequisites"][
    "hda1RawLfaHdaExactlyPairedWithHostRequestAndReturnOrHda2NoReplacement"
  ]


def test_extra_duplicate_hda1_host_request_and_return_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.standstill)
  request = min(
    (sample for sample in analyzer.cluster_can
     if sample.origin == "send_request" and sample.address == ADRV_0X161_ADDRESS),
    key=lambda sample: abs(sample.t - (stop_start + 10.0)),
  )
  returned = min(
    (sample for sample in analyzer.cluster_can
     if sample.origin == "tx_returned" and sample.address == ADRV_0X161_ADDRESS
     and sample.data_hex == request.data_hex),
    key=lambda sample: abs(sample.t - request.t),
  )
  analyzer.cluster_can.extend((
    replace(request, t=request.t + 0.0004),
    replace(returned, t=returned.t + 0.0004),
  ))

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  request_pairing = evidence["hostReplacementPairing"]["adrvRawToRequest"]
  returned_pairing = evidence["hostReplacementPairing"]["adrvRawToReturned"]
  assert evidence["hdaReplacementComparison"]["allAdrvRequestsReturned"]
  assert request_pairing["allSourceFramesPairedByCounterAndTime"]
  assert returned_pairing["allSourceFramesPairedByCounterAndTime"]
  assert request_pairing["unusedOutputFrames"] == 1
  assert returned_pairing["unusedOutputFrames"] == 1
  assert not request_pairing["exactOneToOneByCounterAndTime"]
  assert not returned_pairing["exactOneToOneByCounterAndTime"]
  assert not episode["prerequisites"][
    "hda1RawAdrvExactlyPairedWithHostRequestAndReturnOrHda2NoReplacement"
  ]


def test_hda1_returned_host_replacement_masking_raw_warning_is_direct_failure() -> None:
  report = run_demo("pass", mask_hda1_warning=True).report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["warningPathObservation"] == "hostReplacementMaskedRawWarning"
  assert evidence["warningOutputPairCounts"]["maskedByReturnedFrame"] > 0
  assert not evidence["warningPreservation"]["pathPreservesRawWarning"]
  assert not episode["prerequisites"]["rawAdrvWarningPreservedAcrossActiveTopology"]
  assert not episode["prerequisites"]["hdaPathConsistentWithCurrentStockLongTopology"]
  assert "returned host replacement masked raw ADRV_0x161 ALERTS_5=5" in episode["reasons"][0]


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
  assert evidence["warningPreservation"]["topology"] == "hda2_raw_no_host_replacement"
  assert evidence["warningPreservation"]["pathPreservesRawWarning"]
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


def test_invalid_raw_cluster_checksum_cannot_produce_pass() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  message = CANPacker("hyundai_canfd_generated").make_can_msg("ADRV_0x161", 2, {
    "COUNTER": 9,
    "ALERTS_5": 0,
  })
  invalid = bytearray(message[1])
  invalid[0] ^= 0x01
  analyzer.feed_can("can", 1_010.0, 2, message[0], bytes(invalid))

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["clusterEvidence"]["integrity"]["invalidSampleCount"] == 1
  assert not episode["prerequisites"]["rawAdrv0x161CrcValidAllSamples"]
  assert "failed the Hyundai CAN-FD CRC" in episode["reasons"][0]


def test_demo_rejected_tx_and_early_warning_fails() -> None:
  report = run_demo("fail").report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 30.0
  assert episode["clusterEvidence"]["firstRawAccelerateWarningAfterStop"] < 3.1
  assert any("rejected" in reason for reason in episode["reasons"])
  assert sum(match["status"] == "rejected" for match in report["txMatches"]) == 33


def test_demo_missing_tx_is_inconclusive() -> None:
  report = run_demo("missing-tx").report()

  assert report["overallVerdict"] == "FAIL"
  # The synthetic missing-TX trace also exposes the OEM warning at 3 seconds,
  # so it is direct negative evidence, not merely an absent request.
  assert not report["stopEpisodes"][0]["prerequisites"]["exactSupportedRearmSchedule"]


def test_mid_stop_info_display_4_is_inconclusive_even_with_raw_warning_at_30_seconds() -> None:
  report = run_demo("pass", schedule_mode=SCHEDULE_MODE_MIXED).report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["scheduleMode"] == SCHEDULE_MODE_MIXED
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 1.0
  assert episode["clusterEvidence"]["firstRawAccelerateWarningAfterStop"] == 30.0
  assert not episode["returnedSchedule"]["exactSupportedRearmSchedule"]
  assert not episode["prerequisites"]["exactSupportedRearmSchedule"]
  assert "neither supported schedule can be proven" in episode["reasons"][0]


def test_initial_info_display_4_does_not_hide_an_early_raw_adrv_warning() -> None:
  report = run_demo(
    "pass",
    schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY,
    raw_warning_time=3.0,
  ).report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["scheduleMode"] == SCHEDULE_MODE_INITIAL_RECOVERY
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 0.0
  assert episode["returnedSchedule"]["exactSupportedRearmSchedule"]
  assert episode["clusterEvidence"]["firstRawAccelerateWarningAfterStop"] == 3.0
  assert "raw ADRV_0x161 ALERTS_5=5 appeared early" in episode["reasons"][0]


def test_empty_analyzer_is_inconclusive() -> None:
  report = ProbeAnalyzer("test", "empty").report()
  assert report["schemaVersion"] == 4
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["stopEpisodes"] == []
  assert report["stateEvidence"]["sampleCount"] == 0


def test_state_evidence_explains_standstill_with_stock_cruise_disabled() -> None:
  analyzer = ProbeAnalyzer("test", "disabled-cruise-stop")
  for frame in range(101):
    analyzer.feed_state(
      10.0 + frame * 0.01,
      SimpleNamespace(
        standstill=True,
        vEgo=0.0,
        vEgoRaw=0.0,
        canValid=True,
        brakePressed=False,
        gasPressed=False,
        brakeHoldActive=False,
        parkingBrake=False,
        accFaulted=False,
        cruiseState=SimpleNamespace(enabled=False, standstill=True),
      ),
    )

  report = analyzer.report()
  state = report["stateEvidence"]
  assert report["stopEpisodes"] == []
  assert state["standstillSamples"] == 101
  assert state["cruiseEnabledSamples"] == 0
  assert state["eligibleStopSamples"] == 0
  assert state["maxContinuousStandstill"] == 1.0
  assert state["maxContinuousEligibleStop"] == 0.0
  assert state["gateTransitions"] == [{
    "t": 0.0,
    "standstill": True,
    "cruiseEnabled": False,
    "cruiseStandstill": True,
    "vEgo": 0.0,
    "vEgoRaw": 0.0,
    "canValid": True,
    "brakePressed": False,
    "gasPressed": False,
    "brakeHoldActive": False,
    "parkingBrake": False,
    "accFaulted": False,
    "eligibleStop": False,
  }]


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
  assert "vEgo/vEgoRaw gate" in report["stopEpisodes"][0]["reasons"][0]


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


def test_single_car_control_sample_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  analyzer.controls = [next(sample for sample in analyzer.controls if sample.t >= 1_000.0)]

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  coverage = episode["streamCoverage"]["carControl"]
  assert coverage["startEdgeCovered"]
  assert not coverage["endEdgeCovered"]
  assert not coverage["continuous"]
  assert episode["prerequisites"]["carControlEnabledAllSamples"]
  assert not episode["prerequisites"]["carControlContinuousCoverage"]


def test_two_sparse_scc_samples_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  analyzer.scc = [
    analyzer.scc[0],
    next(sample for sample in analyzer.scc if sample.info_display == 4),
  ]

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  coverage = episode["streamCoverage"]["rawScc0x1a0"]
  assert coverage["startEdgeCovered"]
  assert coverage["gapsOverLimit"] == 1
  assert not coverage["continuous"]
  assert episode["prerequisites"]["rawSccLeadGateValidAllSamples"]
  assert not episode["prerequisites"]["rawScc0x1a0ContinuousCoverage"]


def test_sparse_raw_buttons_with_every_res_source_still_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  requests = [sample for sample in analyzer.buttons
              if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL]
  stock = [sample for sample in analyzer.buttons
           if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS]
  required_stock_ids = {
    id(min(stock, key=lambda sample: abs(sample.t - request.t)))
    for request in requests
  }
  required_stock_ids.update((id(stock[0]), id(min(stock, key=lambda sample: abs(sample.t - 1_030.25)))))
  analyzer.buttons = [
    sample for sample in analyzer.buttons
    if sample.origin != "vehicle_rx"
    or sample.address != CRUISE_BUTTONS_ALT_ADDRESS
    or id(sample) in required_stock_ids
  ]

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  coverage = episode["streamCoverage"]["rawStockButtons0x1aa"]
  assert coverage["startEdgeCovered"]
  assert coverage["endEdgeCovered"]
  assert coverage["gapsOverLimit"] > 0
  assert not coverage["continuous"]
  assert episode["prerequisites"]["sourcePlusOneCounterAllFrames"]
  assert episode["prerequisites"]["sourceTimestampsFreshSequentialAllGroups"]
  assert episode["prerequisites"]["sourceCountersFreshSequentialAllGroups"]
  assert episode["prerequisites"]["emittedCountersFreshSequentialAllGroups"]
  assert not episode["prerequisites"]["rawStockButtons0x1aaContinuousCoverage"]


def test_missing_post_burst_release_candidate_cannot_produce_pass() -> None:
  analyzer = run_demo("pass")
  requests = sorted(
    (sample for sample in analyzer.buttons
     if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL),
    key=lambda sample: sample.t,
  )
  first_group = [request for request in requests if request.t - requests[0].t <= 0.100]
  final_request = first_group[-1]
  stock = sorted(
    (sample for sample in analyzer.buttons
     if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS),
    key=lambda sample: sample.t,
  )
  final_source = min(
    (sample for sample in stock
     if -0.002 <= final_request.t - sample.t <= 0.025
     and sample.counter == ((final_request.counter - 1) & 0xFF)),
    key=lambda sample: abs(final_request.t - sample.t),
  )
  release_candidate = min(
    (sample for sample in stock
     if sample.t > final_source.t and sample.counter == ((final_request.counter + 1) & 0xFF)),
    key=lambda sample: sample.t,
  )
  analyzer.buttons.remove(release_candidate)

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["streamCoverage"]["rawStockButtons0x1aa"]["continuous"]
  assert not episode["resGroups"][0]["postBurstSameCounterThenNextCounterObserved"]
  assert not episode["prerequisites"][
    "rawPostBurstSameCounterAndNextReleaseCandidatesObservedAllGroups"
  ]


@pytest.mark.parametrize("missing_source_index", (0, 1, 2))
def test_missing_one_raw_source_is_inconclusive_instead_of_stale_source_failure(
    missing_source_index: int,
) -> None:
  analyzer = run_demo("pass")
  first_requests = [sample for sample in analyzer.buttons
                    if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL][:3]
  missing_time = first_requests[missing_source_index].t
  analyzer.buttons = [
    sample for sample in analyzer.buttons
    if not (
      sample.origin == "vehicle_rx"
      and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
      and sample.t == missing_time
    )
  ]

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  group = report["stopEpisodes"][0]["resGroups"][0]
  assert not group["sourceSequenceFullyObserved"]
  assert not group["sourceTimestampsFreshSequential"]
  assert group["emittedCountersFreshSequential"]
  assert "fresh sequential source/emitted counters" not in report["stopEpisodes"][0]["reasons"][0]


def test_stale_sources_and_duplicate_res_counters_are_direct_failure() -> None:
  analyzer = run_demo("pass")
  request_indices = [index for index, sample in enumerate(analyzer.buttons)
                     if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL][:3]
  request_times = [analyzer.buttons[index].t for index in request_indices]
  source_indices = [next(
    index for index, sample in enumerate(analyzer.buttons)
    if sample.origin == "vehicle_rx"
    and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
    and sample.t == request_time
  ) for request_time in request_times]
  source_counter = analyzer.buttons[source_indices[0]].counter
  emitted_counter = analyzer.buttons[request_indices[0]].counter
  for index in source_indices[1:]:
    analyzer.buttons[index] = replace(analyzer.buttons[index], counter=source_counter)
  for index in request_indices[1:]:
    analyzer.buttons[index] = replace(analyzer.buttons[index], counter=emitted_counter)

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  group = report["stopEpisodes"][0]["resGroups"][0]
  assert group["sourceSequenceFullyObserved"]
  assert group["sourceTimestampsFreshSequential"]
  assert group["sourcePlusOneAllFrames"]
  assert group["sourceCounterDeltasMod256"] == [0, 0]
  assert group["emittedCounterDeltasMod256"] == [0, 0]
  assert not group["sourceCountersFreshSequential"]
  assert not group["emittedCountersFreshSequential"]
  assert "fresh sequential source/emitted counters" in report["stopEpisodes"][0]["reasons"][0]


def test_duplicate_res_counter_with_fresh_sources_is_direct_failure() -> None:
  analyzer = run_demo("pass")
  request_indices = [index for index, sample in enumerate(analyzer.buttons)
                     if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL][:3]
  analyzer.buttons[request_indices[1]] = replace(
    analyzer.buttons[request_indices[1]],
    counter=analyzer.buttons[request_indices[0]].counter,
  )

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  group = report["stopEpisodes"][0]["resGroups"][0]
  assert group["sourceSequenceFullyObserved"]
  assert group["sourceTimestampsFreshSequential"]
  assert group["sourceCountersFreshSequential"]
  assert group["emittedCounterDeltasMod256"][0] == 0
  assert not group["emittedCountersFreshSequential"]
  assert not group["sourcePlusOneAllFrames"]
  assert "fresh sequential source/emitted counters" in report["stopEpisodes"][0]["reasons"][0]


def test_fresh_source_and_emitted_counter_sequences_accept_mod256_wraparound() -> None:
  report = run_demo("pass", button_counter_offset=129).report()

  assert report["overallVerdict"] == "PASS"
  group = report["stopEpisodes"][0]["resGroups"][0]
  assert group["sourceCountersPerFrame"] == [254, 255, 0]
  assert group["sourceCounterDeltasMod256"] == [1, 1]
  assert group["sourceTimestampDeltasMs"] == [20.0, 20.0]
  assert group["counters"] == [255, 0, 1]
  assert group["emittedCounterDeltasMod256"] == [1, 1]
  assert group["sourceCountersFreshSequential"]
  assert group["sourceTimestampsFreshSequential"]
  assert group["sourceSequenceFullyObserved"]
  assert group["emittedCountersFreshSequential"]
