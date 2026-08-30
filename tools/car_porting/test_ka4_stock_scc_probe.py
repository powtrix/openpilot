from dataclasses import replace
from types import SimpleNamespace

import pytest

from tools.car_porting.ka4_stock_scc_probe import (  # noqa: TID251
  ADRV_0X161_ADDRESS,
  BUTTON_RES_ACCEL,
  CRUISE_BUTTONS_ALT_ADDRESS,
  CRUISE_BUTTONS_ADDRESS,
  LFAHDA_CLUSTER_ADDRESS,
  ProbeAnalyzer,
  RECOVERY_REARM_GROUP_STARTS,
  REGULAR_REARM_GROUP_STARTS,
  SCC_CONTROL_ADDRESS,
  SCHEDULE_MODE_INITIAL_RECOVERY,
  SCHEDULE_MODE_MIXED,
  SCHEDULE_MODE_REGULAR,
  classify_can_source,
  _demo_car_params,
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
def test_demo_models_exact_supported_schedule_without_claiming_vehicle_acceptance(
    schedule_mode: str,
    button_source_phase_frames: int,
    expected_schedules: tuple[tuple[float, ...], ...],
) -> None:
  report = run_demo(
    "pass",
    schedule_mode=schedule_mode,
    button_source_phase_frames=button_source_phase_frames,
  ).report()

  assert report["schemaVersion"] == 6
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  episode = report["stopEpisodes"][0]
  assert episode["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
  assert episode["signalSpecificChecks"]["adrv0x161"]["candidateAlert5Correlation"] == "MATCHED_AT_30S"
  assert episode["scheduleMode"] == schedule_mode
  assert episode["clusterEvidence"]["firstRawAlert5Value5AfterStop"] == 30.0
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


def test_probe_reports_tcs_active_panda_and_ordered_transition_evidence() -> None:
  report = run_demo("pass").report()

  assert report["tcsEvidence"]["buses"][0]["accRequestCounts"] == {1: 1751}
  panda = report["pandaEvidence"]
  assert panda["activeSafetyPandaIndex"] == 0
  assert panda["activeSafetyPandaIndexObserved"]
  assert panda["activeSafetyConfigObserved"]
  assert panda["pandas"][0]["safetyTxBlocked"]["delta"] == 0
  assert panda["pandas"][0]["safetyTxBlocked"]["deltaExact"]

  transition = report["stopTransitionEvidence"][0]
  assert transition["firstInfoDisplay4AfterStop"] == 30.0
  assert transition["firstRawAlert5Value5AfterStop"] == 30.0
  assert transition["controllerGateEligibility"]["status"] == "ELIGIBLE"
  assert transition["controllerGateEligibility"]["firstEligibleAfterPhysicalStop"] == 0.3
  assert transition["firstHostResRequestAfterStop"] == 2.5
  assert transition["firstHostResTxStatus"] == "returned"
  assert transition["promptCandidateOrderingVsSafeRearm"] == "AFTER"
  assert transition["promptCandidateBeforeObservedSafeRearm"] is False
  assert transition["blockingGateClosedAtFirstHostRes"] is False
  assert not transition["destinationEcuAcceptanceProven"]
  assert transition["classifications"] == ["DESTINATION_UNPROVEN"]

  timeline_events = {item["event"] for item in report["correlatedTimeline"]}
  assert {
    "sccEngagementTransition",
    "tcsStateTransition",
    "carStateCruiseTransition",
    "carControlTransition",
    "pandaSafetyTransition",
  } <= timeline_events


def test_probe_flags_early_prompt_candidate_and_gate_close_before_first_res() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  close_t = stop_start + 0.20
  analyzer.scc = [
    replace(sample, acc_mode=4) if sample.t >= close_t else sample
    for sample in analyzer.scc
  ]
  analyzer.panda_states = [
    replace(sample, controls_allowed=False, safety_tx_blocked=1)
    if sample.t >= close_t else sample
    for sample in analyzer.panda_states
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["firstInfoDisplay4AfterStop"] == 0.0
  assert transition["promptCandidateWithin300msOfPhysicalStop"]
  assert transition["promptCandidateOrderingVsSafeRearm"] == "SAFE_REARM_NOT_OBSERVED"
  assert transition["promptCandidateBeforeObservedSafeRearm"] is None
  assert transition["blockingGateEvidence"]["sccAccMode"]["firstFallingEdgeAfterStop"] == 0.2
  assert transition["blockingGateEvidence"]["sccAccMode"]["stateAtFirstHostRes"] == "CLOSED"
  assert transition["blockingGateEvidence"]["pandaControlsAllowed"]["firstFallingEdgeAfterStop"] == 0.2
  assert transition["blockingGateEvidence"]["pandaControlsAllowed"]["stateAtFirstHostRes"] == "CLOSED"
  assert transition["activePandaSafetyTxBlocked"]["delta"] == 1
  assert transition["blockingGateClosedAtFirstHostRes"] is True
  assert "PROMPT_CANDIDATE_WITHIN_300MS_OF_PHYSICAL_STOP" in transition["classifications"]
  assert "BLOCKING_GATE_CLOSED_AT_FIRST_HOST_RES" in transition["classifications"]


def test_nonblocking_main_mode_and_tcs_request_transitions_do_not_create_gate_blocker() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  close_t = stop_start + 0.20
  analyzer.scc = [
    replace(sample, main_mode_acc=0) if sample.t >= close_t else sample
    for sample in analyzer.scc
  ]
  analyzer.tcs = [
    replace(sample, acc_request=0) if sample.t >= close_t else sample
    for sample in analyzer.tcs
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["observedNonBlockingTransitions"]["sccMainModeAcc"]["firstFallingEdgeAfterStop"] == 0.2
  assert transition["observedNonBlockingTransitions"]["tcsAccRequest"]["firstFallingEdgeAfterStop"] == 0.2
  assert transition["blockingGateClosedAtFirstHostRes"] is False
  assert "BLOCKING_GATE_CLOSED_AT_FIRST_HOST_RES" not in transition["classifications"]


def test_gate_already_closed_at_physical_stop_boundary_is_not_missed() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  analyzer.scc = [replace(sample, acc_mode=4) for sample in analyzer.scc]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  acc_mode = transition["blockingGateEvidence"]["sccAccMode"]
  assert acc_mode["status"] == "CLOSED_THROUGH_REQUEST_WINDOW"
  assert acc_mode["firstClosedObservationAfterStop"] == 0.0
  assert acc_mode["stateAtFirstHostRes"] == "CLOSED"
  assert transition["blockingGateClosedAtFirstHostRes"] is True


def test_gate_change_across_unobserved_can_gap_is_not_claimed_as_ordered_edge() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.scc = [
    replace(sample, acc_mode=4) if sample.t >= stop_start + 0.20 else sample
    for sample in analyzer.scc
    if not stop_start + 0.01 <= sample.t < stop_start + 0.20
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  acc_mode = transition["blockingGateEvidence"]["sccAccMode"]
  assert acc_mode["status"] == "UNKNOWN_GAP"
  assert acc_mode["firstFallingEdgeAfterStop"] is None
  assert acc_mode["stateAtFirstHostRes"] == "CLOSED"
  assert transition["blockingGateClosedAtFirstHostRes"] is True


def test_gate_change_at_same_time_as_first_res_is_ordering_ambiguous() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  first_res = stop_start + 0.30
  analyzer.scc = [
    replace(sample, acc_mode=4) if sample.t >= first_res else sample
    for sample in analyzer.scc
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["blockingGateEvidence"]["sccAccMode"]["stateAtFirstHostRes"] == "AMBIGUOUS_TRANSITION"
  assert transition["blockingGateClosedAtFirstHostRes"] is None


def test_gate_closed_early_then_reopened_before_res_is_not_a_request_blocker() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_REGULAR)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.scc = [
    replace(sample, lead_info=1) if sample.t < stop_start + 0.20 else sample
    for sample in analyzer.scc
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  raw_gate = transition["blockingGateEvidence"]["rawSccLeadSafe"]
  assert raw_gate["firstClosedObservationAfterStop"] == 0.0
  assert raw_gate["firstReopeningEdgeAfterStop"] == 0.2
  assert raw_gate["stateAtFirstHostRes"] == "OPEN"
  assert transition["controllerGateEligibility"]["firstEligibleAfterPhysicalStop"] == 0.5
  assert transition["blockingGateClosedAtFirstHostRes"] is False
  assert not transition["hostResBeforeObservedSafeRearmEligibility"]


def test_res_after_physical_stop_end_is_not_attributed_by_grace_window() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.states = [
    replace(sample, standstill=False, v_ego=0.2, v_ego_raw=0.2)
    if sample.t >= stop_start + 0.25 else sample
    for sample in analyzer.states
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["end"] - transition["start"] < 0.30
  assert transition["firstHostResRequestAfterStop"] is None
  assert "NO_HOST_RES_REQUEST" in transition["classifications"]


@pytest.mark.parametrize("post_reset_counter", [0, 9], ids=["decrease", "increase"])
def test_panda_counter_reset_is_unknown_regardless_of_counter_direction(post_reset_counter: int) -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.panda_states = [
    replace(sample, uptime=100, safety_tx_blocked=5)
    if sample.t < stop_start + 0.50 else
    replace(sample, uptime=1, safety_tx_blocked=post_reset_counter)
    for sample in analyzer.panda_states
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  counter = transition["activePandaSafetyTxBlocked"]
  assert counter["delta"] is None
  assert not counter["deltaExact"]
  assert counter["resetCount"] == 1
  assert "PANDA_COUNTER_DISCONTINUITY" in transition["classifications"]


def test_panda_counter_wrap_and_ambiguous_decrease_are_distinguished() -> None:
  analyzer = run_demo("pass")
  first, second = analyzer.panda_states[:2]
  wrap = analyzer._counter_evidence([
    replace(first, uptime=100, safety_tx_blocked=(1 << 32) - 2),
    replace(second, uptime=101, safety_tx_blocked=3),
  ], "safety_tx_blocked")
  assert wrap["delta"] == 5
  assert wrap["deltaExact"]
  assert wrap["wrapCount"] == 1

  ambiguous = analyzer._counter_evidence([
    replace(first, uptime=100, safety_tx_blocked=100),
    replace(second, uptime=101, safety_tx_blocked=3),
  ], "safety_tx_blocked")
  assert ambiguous["delta"] is None
  assert not ambiguous["deltaExact"]
  assert ambiguous["ambiguousDecreaseCount"] == 1


def test_duplicate_stock_signal_bus_makes_controller_bus_evidence_unresolved() -> None:
  analyzer = run_demo("pass")
  analyzer.scc.append(replace(analyzer.scc[1], bus=1))

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["busEvidence"]["unexpectedStockSignalBuses"] == [1]
  assert not transition["busEvidence"]["resolvedForControllerEvidence"]
  assert "BUS_UNRESOLVED" in transition["classifications"]


def test_stock_signal_on_other_bus_outside_stop_does_not_contaminate_stop_bus_evidence() -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.scc.append(replace(analyzer.scc[0], t=stop_start - 0.01, bus=1))

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["busEvidence"]["unexpectedStockSignalBuses"] == []
  assert transition["busEvidence"]["resolvedForControllerEvidence"]


def test_sparse_tcs_stream_cannot_claim_controller_bus_evidence_resolved() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.tcs = [min(
    (sample for sample in analyzer.tcs if sample.t >= stop_start),
    key=lambda sample: sample.t,
  )]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  tcs_gate = transition["blockingGateEvidence"]["tcsAccEnableNoFault"]
  assert tcs_gate["status"] == "UNKNOWN_GAP"
  assert tcs_gate["coverageGapCount"] > 0
  assert not transition["busEvidence"]["tcsGateCoverageComplete"]
  assert not transition["busEvidence"]["resolvedForControllerEvidence"]
  assert "BUS_UNRESOLVED" in transition["classifications"]


def test_wrong_active_panda_safety_config_is_not_used_as_controls_allowed_gate() -> None:
  analyzer = run_demo("pass")
  analyzer.panda_states = [
    replace(sample, safety_param=sample.safety_param + 1)
    for sample in analyzer.panda_states
  ]

  report = analyzer.report()
  assert not report["pandaEvidence"]["activeSafetyConfigObserved"]
  transition = report["stopTransitionEvidence"][0]
  assert not transition["activePandaStateObserved"]
  assert transition["activePandaIndexStateObserved"]
  assert transition["blockingGateEvidence"]["pandaControlsAllowed"]["status"] == "CLOSED_THROUGH_REQUEST_WINDOW"
  assert transition["blockingGateEvidence"]["pandaControlsAllowed"]["stateAtFirstHostRes"] == "CLOSED"
  assert "ACTIVE_PANDA_CONFIG_UNOBSERVED" in transition["classifications"]


def test_temporary_panda_config_change_is_preserved_as_discontinuity_segment() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.panda_states = [
    replace(sample, safety_model="noOutput", safety_param=0)
    if stop_start + 0.10 <= sample.t < stop_start + 0.20 else sample
    for sample in analyzer.panda_states
  ]

  report = analyzer.report()
  assert not report["pandaEvidence"]["activeSafetyConfigContinuous"]
  transition = report["stopTransitionEvidence"][0]
  panda_gate = transition["blockingGateEvidence"]["pandaControlsAllowed"]
  assert panda_gate["firstFallingEdgeAfterStop"] == 0.1
  assert panda_gate["firstReopeningEdgeAfterStop"] == 0.2
  assert panda_gate["stateAtFirstHostRes"] == "OPEN"
  assert transition["activePandaConfigMismatchSamples"] > 0
  assert transition["activePandaSafetyTxBlocked"]["delta"] is None
  assert "ACTIVE_PANDA_CONFIG_DISCONTINUITY" in transition["classifications"]


def test_crc_invalid_causal_can_sample_is_excluded_and_marks_integrity_unresolved() -> None:
  analyzer = run_demo("pass")
  analyzer.scc[1] = replace(analyzer.scc[1], checksum_valid=False)

  transition = analyzer.report()["stopTransitionEvidence"][0]
  integrity = transition["busEvidence"]["causalCanIntegrity"]
  assert integrity["sccInvalidOrUnavailableChecksumSamples"] == 1
  assert not transition["busEvidence"]["resolvedForControllerEvidence"]
  assert "CAUSAL_CAN_INTEGRITY_UNRESOLVED" in transition["classifications"]


@pytest.mark.parametrize(
  "wrong_address,wrong_bus,expected_address_hex",
  [
    pytest.param(CRUISE_BUTTONS_ADDRESS, 2, "0x1CF", id="wrong-address"),
    pytest.param(CRUISE_BUTTONS_ALT_ADDRESS, 0, "0x1AA", id="wrong-bus"),
  ],
)
def test_wrong_address_or_bus_res_does_not_become_causal_first_host_request(
    wrong_address: int, wrong_bus: int, expected_address_hex: str,
) -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  expected = next(
    sample for sample in analyzer.buttons
    if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL
  )
  analyzer.buttons.append(replace(
    expected,
    t=stop_start + 0.10,
    address=wrong_address,
    bus=wrong_bus,
  ))

  transition = analyzer.report()["stopTransitionEvidence"][0]
  assert transition["firstHostResRequestAfterStop"] == 0.3
  assert transition["unexpectedHostResRequests"] == [{
    "t": 0.1,
    "addressHex": expected_address_hex,
    "bus": wrong_bus,
    "txStatus": "unobserved",
  }]
  assert "UNEXPECTED_HOST_RES_REQUEST" in transition["classifications"]


@pytest.mark.parametrize("bus_offset,expected_bus", [(0, 1), (4, 5)])
def test_hda2_causal_host_res_uses_car_params_derived_ecan_bus(
    bus_offset: int, expected_bus: int,
) -> None:
  transition = run_demo(
    "pass",
    bus_offset=bus_offset,
    hda2=True,
    schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY,
  ).report()["stopTransitionEvidence"][0]

  assert transition["expectedHostRes"] == {
    "addressHex": "0x1AA",
    "bus": expected_bus,
  }
  assert transition["firstHostResRequestAfterStop"] == 0.3
  assert transition["unexpectedHostResRequests"] == []


def test_sparse_adrv_stream_marks_prompt_candidate_timing_unresolved() -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  first_alert_t = min(
    sample.t for sample in analyzer.cluster_can
    if sample.origin == "vehicle_rx"
    and sample.address == ADRV_0X161_ADDRESS
    and sample.alert_5 == 5
  )
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can
    if not (
      sample.origin == "vehicle_rx"
      and sample.address == ADRV_0X161_ADDRESS
      and sample.t not in (stop_start, first_alert_t)
    )
  ]

  transition = analyzer.report()["stopTransitionEvidence"][0]
  bus_evidence = transition["busEvidence"]
  assert transition["firstRawAlert5Value5AfterStop"] == 30.0
  assert bus_evidence["adrvPromptStreamObserved"]
  assert not bus_evidence["adrvPromptStreamCoverage"]["continuous"]
  assert bus_evidence["adrvPromptStreamCoverage"]["gapsOverLimit"] == 1
  assert not bus_evidence["resolvedForAdrvPromptCandidate"]
  assert "PROMPT_CANDIDATE_STREAM_UNRESOLVED" in transition["classifications"]


def test_safe_rearm_eligibility_tracks_delayed_raw_lead_qualification_not_fixed_stop_plus_300ms() -> None:
  analyzer = run_demo("pass", schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY)
  stop_start = min(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.scc = [
    replace(sample, lead_info=1) if sample.t < stop_start + 0.20 else sample
    for sample in analyzer.scc
  ]

  qualification = analyzer.report()["stopTransitionEvidence"][0]["controllerGateEligibility"]
  assert qualification["status"] == "ELIGIBLE"
  assert qualification["firstEligibleAfterPhysicalStop"] == 0.5


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
  from opendbc.car.structs import CarParams
  from opendbc.car.hyundai.values import HyundaiSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.car.hyundai.tests.test_stock_scc_can_replay import Ka4StockSccReplay, pulse_groups

  replay = Ka4StockSccReplay(alt_buttons=True, button_phase_frames=button_source_phase_frames)
  if schedule_mode == SCHEDULE_MODE_INITIAL_RECOVERY:
    replay.modeled_state_deadline = 0

  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(
    CarParams.SafetyModel.hyundaiCanfd, int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS),
  ) == 0
  safety.init_tests()

  for frame in range(2701):
    replay.step(frame, safety=safety)
    assert all(replay.last_safety_tx_results)

  groups = pulse_groups([message.frame for message in replay.injected])
  assert tuple(group[0] / 100.0 for group in groups) == expected_schedules[button_source_phase_frames]
  assert all(group == [group[0], group[0] + 2, group[0] + 4] for group in groups)


def test_demo_observation_accepts_second_panda_global_bus_offset() -> None:
  report = run_demo("pass", bus_offset=4).report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  assert report["carParams"]["pandaBusOffset"] == 4
  episode = report["stopEpisodes"][0]
  assert episode["busLayout"] == {
    "stockBus": 4,
    "expectedStockBus": 4,
    "pandaBusOffset": 4,
    "canFdHda2": False,
    "expectedSendBus": 6,
    "observedSendBuses": [6],
    "unexpectedSccTransportObserved": False,
    "unexpectedRaw0x1aaTransportObserved": False,
    "raw0x1cfTransportObservedOnAnyRxBus": False,
  }
  assert episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]
  assert episode["prerequisites"]["send0x1aaUsesExpectedBus"]
  assert episode["prerequisites"]["stockAndSendBusesUseSafetyPanda"]


def test_hda1_host_replacement_preserves_observed_alert5_value_in_returned_dbc_frames() -> None:
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
  assert evidence["alert5ValuePathObservation"] == "hostReplacementPreservedAlert5Value5"
  assert evidence["alert5ValuePairCounts"]["changedByReturnedFrame"] == 0
  assert evidence["alert5ValuePairCounts"]["unpairedRawAlert5Value5Frames"] == 0
  assert evidence["alert5ValuePairCounts"]["preservedByReturnedFrame"] > 0
  assert evidence["alert5ValuePreservation"]["pathPreservesObservedValue5"]
  assert episode["signalSpecificChecks"]["adrv0x161"]["pathPreservesObservedValue5"]
  assert evidence["hdaReplacementComparison"]["stateMismatches"] == 0
  assert evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]
  res_times = [event["t"] for event in report["correlatedTimeline"] if event["event"] == "resHostRequest"]
  raw_value5 = next(event for event in report["correlatedTimeline"]
                     if event["event"] == "clusterCanTransition"
                     and event["origin"] == "vehicle_rx" and event["signal"] == "ALERTS_5"
                     and event["value"] == 5)
  assert max(res_times) < raw_value5["t"]


def test_complete_schedule_without_0x161_is_observed_schedule_only() -> None:
  analyzer = run_demo("pass")
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can if sample.address != ADRV_0X161_ADDRESS
  ]
  analyzer.streams = {
    key: stat for key, stat in analyzer.streams.items() if key[-1] != ADRV_0X161_ADDRESS
  }

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_RES_SCHEDULE_ONLY"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  episode = report["stopEpisodes"][0]
  assert episode["canEvidenceVerdict"] == "OBSERVED_RES_SCHEDULE_ONLY"
  assert all(episode["prerequisites"].values())
  adrv = episode["signalSpecificChecks"]["adrv0x161"]
  assert adrv["applicability"] == "NOT_OBSERVED_IN_CAPTURE"
  assert adrv["continuousThrough30_25s"] is None
  assert adrv["crcValidAllSamples"] is None
  assert adrv["candidateAlert5Correlation"] == "NOT_OBSERVED"
  assert adrv["pathPreservesObservedValue5"] is None


def test_observed_but_undecodable_0x161_is_not_treated_as_variant_absence() -> None:
  analyzer = run_demo("pass")
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can if sample.address != ADRV_0X161_ADDRESS
  ]
  analyzer.streams = {
    key: stat for key, stat in analyzer.streams.items() if key[-1] != ADRV_0X161_ADDRESS
  }
  analyzer.feed_can("can", 1_010.0, 2, ADRV_0X161_ADDRESS, b"\x00")

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  assert report["decodeErrors"]
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["rawAdrvTransportObserved"]
  assert evidence["adrv0x161Applicability"] == "OBSERVED_BUT_UNDECODED"
  assert evidence["candidateAlert5Correlation"] == "UNAVAILABLE_DUE_TO_DECODE_ERRORS"
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_undecodable_host_0x161_without_raw_source_is_not_treated_as_variant_absence() -> None:
  analyzer = run_demo("pass")
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can if sample.address != ADRV_0X161_ADDRESS
  ]
  analyzer.streams = {
    key: stat for key, stat in analyzer.streams.items() if key[-1] != ADRV_0X161_ADDRESS
  }
  analyzer.feed_can("sendcan", 1_010.010, 0, ADRV_0X161_ADDRESS, b"\x00")
  analyzer.feed_can("can", 1_010.012, 0x80, ADRV_0X161_ADDRESS, b"\x00")

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert not evidence["rawAdrvTransportObserved"]
  assert evidence["hostAdrvTransportObserved"]
  assert evidence["adrv0x161Applicability"] == "EXPECTED_RAW_NOT_OBSERVED_OTHER_TRAFFIC_PRESENT"
  assert evidence["candidateAlert5Correlation"] == "UNAVAILABLE_DUE_TO_DECODE_ERRORS"
  assert evidence["integrity"]["decodeErrorCount"] == 2
  assert not episode["prerequisites"]["clusterCanDecodedAllObservedSamples"]
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_wrong_bus_0x161_transport_is_not_reported_as_variant_absence() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  analyzer.cluster_can = [
    sample for sample in analyzer.cluster_can if sample.address != ADRV_0X161_ADDRESS
  ]
  analyzer.streams = {
    key: stat for key, stat in analyzer.streams.items() if key[-1] != ADRV_0X161_ADDRESS
  }
  message = CANPacker("hyundai_canfd_generated").make_can_msg("ADRV_0x161", 0, {
    "COUNTER": 1,
    "ALERTS_5": 0,
  })
  analyzer.feed_can("can", 1_010.0, 0, message[0], message[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["anyAdrvTransportObserved"]
  assert evidence["unexpectedAdrvTransportObserved"]
  assert evidence["adrv0x161Applicability"] == "EXPECTED_RAW_NOT_OBSERVED_OTHER_TRAFFIC_PRESENT"
  assert not evidence["hdaReplacementComparison"]["adrvPathConsistentWithObservedVariant"]


def test_undecodable_raw_stock_button_frame_cannot_hide_in_decoded_coverage() -> None:
  analyzer = run_demo("pass")
  analyzer.feed_can("can", 1_011.0, 0, CRUISE_BUTTONS_ALT_ADDRESS, b"\x00")

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["streamCoverage"]["rawStockButtons0x1aa"]["continuous"]
  assert not episode["prerequisites"]["allRaw0x1aaCrcValid"]
  assert not episode["prerequisites"]["allProofRelevantCanFramesDecoded"]
  assert episode["decodeErrors"][0]["addressHex"] == "0x1AA"


def test_undecodable_standard_button_transport_is_not_reported_absent() -> None:
  analyzer = run_demo("pass")
  analyzer.feed_can("can", 1_010.0, 0, CRUISE_BUTTONS_ADDRESS, b"\x00")

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]
  assert not episode["prerequisites"]["exactAltBusAndAddressLayout"]
  assert not episode["prerequisites"]["allProofRelevantCanFramesDecoded"]


def test_one_undecodable_raw_0x161_among_decoded_frames_keeps_evidence_inconclusive() -> None:
  analyzer = run_demo("pass")
  analyzer.feed_can("can", 1_010.025, 2, ADRV_0X161_ADDRESS, b"\x00")

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["adrv0x161Applicability"] == "OBSERVED_WITH_DECODE_ERRORS"
  assert evidence["candidateAlert5Correlation"] == "UNAVAILABLE_DUE_TO_DECODE_ERRORS"
  assert not evidence["integrity"]["rawAdrvAllCrcValid"]
  assert not episode["prerequisites"]["clusterCanDecodedAllObservedSamples"]
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_sparse_hda1_host_replacement_streams_keep_can_evidence_inconclusive() -> None:
  analyzer = run_demo("pass")
  stop_start = min(sample.t for sample in analyzer.states if sample.standstill)

  # Keep one LFAHDA host request/return pair near the stop boundary and only
  # the ALERTS_5=5-era ADRV replacements. The raw camera streams remain complete,
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
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_extra_duplicate_hda1_host_request_and_return_keep_can_evidence_inconclusive() -> None:
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
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_pre_stop_raw_cluster_source_cannot_consume_in_window_extra_output() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  packer = CANPacker("hyundai_canfd_generated")
  raw = packer.make_can_msg("LFAHDA_CLUSTER", 2, {
    "COUNTER": 255,
    "HDA_CntrlModSta": 2,
    "HDA_LFA_SymSta": 2,
  })
  extra = packer.make_can_msg("LFAHDA_CLUSTER", 0, {
    "COUNTER": 255,
    "HDA_CntrlModSta": 2,
    "HDA_LFA_SymSta": 2,
  })
  analyzer.feed_can("can", 999.950, 2, raw[0], raw[1])
  analyzer.feed_can("sendcan", 1_000.001, 0, extra[0], extra[1])
  analyzer.feed_can("can", 1_000.003, 0x80, extra[0], extra[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  comparison = episode["clusterEvidence"]["hdaReplacementComparison"]
  assert comparison["unexpectedLfaHdaReturnedFrames"] == 1
  assert not comparison["lfaHdaPathConsistent"]
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]


def test_hda1_returned_host_replacement_changing_observed_alert5_value_is_direct_failure() -> None:
  report = run_demo("pass", change_hda1_alert5_value=True).report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["alert5ValuePathObservation"] == "hostReplacementChangedAlert5Value5"
  assert evidence["alert5ValuePairCounts"]["changedByReturnedFrame"] > 0
  assert not evidence["alert5ValuePreservation"]["pathPreservesObservedValue5"]
  assert not episode["signalSpecificChecks"]["adrv0x161"]["pathPreservesObservedValue5"]
  assert not episode["prerequisites"]["adrvAndLfaHdaPathsConsistentWithObservedVariant"]
  assert "changed an observed ADRV_0x161 ALERTS_5 value" in episode["reasons"][0]


def test_hda1_alert5_value_mutation_after_proof_window_is_direct_failure() -> None:
  report = run_demo(
    "pass", raw_alert5_value5_time=33.0, change_hda1_alert5_value=True,
  ).report()

  assert report["overallVerdict"] == "FAIL"
  assert report["canEvidenceVerdict"] == "FAIL"
  assert report["vehicleAcceptanceVerdict"] == "BLOCKED_BY_CAN_EVIDENCE"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["alert5ValuePairCounts"]["changedByReturnedFrame"] > 0
  assert evidence["hdaReplacementComparison"]["adrvAlert5ValueMismatches"] > 0
  assert "changed an observed ADRV_0x161 ALERTS_5 value" in episode["reasons"][0]


def test_extra_mutated_adrv_return_after_proof_window_is_direct_failure() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass", raw_alert5_value5_time=33.0)
  raw = next(
    sample for sample in analyzer.cluster_can
    if sample.origin == "vehicle_rx"
    and sample.address == ADRV_0X161_ADDRESS
    and sample.alert_5 == 5
  )
  extra = CANPacker("hyundai_canfd_generated").make_can_msg("ADRV_0x161", 0, {
    "COUNTER": raw.counter,
    "ALERTS_5": 0,
  })
  analyzer.feed_can("sendcan", raw.t + 0.010, 0, extra[0], extra[1])
  analyzer.feed_can("can", raw.t + 0.012, 0x80, extra[0], extra[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  assert report["canEvidenceVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  evidence = episode["clusterEvidence"]
  assert evidence["hdaReplacementComparison"]["unexpectedAdrvReturnedFrames"] == 1
  assert evidence["alert5ValuePairCounts"]["allReturnedValueMismatches"] == 1
  assert evidence["alert5ValuePairCounts"]["allReturnedValue5Mismatches"] == 1
  assert evidence["alert5ValuePathObservation"] == "hostReplacementChangedAlert5Value5"
  assert not evidence["alert5ValuePreservation"]["pathPreservesObservedValue5"]
  assert "changed an observed ADRV_0x161 ALERTS_5 value" in episode["reasons"][0]


def test_extra_mutated_adrv_stock_owned_field_is_direct_failure() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  raw = min(
    (
      sample for sample in analyzer.cluster_can
      if sample.origin == "vehicle_rx" and sample.address == ADRV_0X161_ADDRESS
    ),
    key=lambda sample: abs(sample.t - 1_010.0),
  )
  values = dict(raw.adrv_stock_owned_values)
  values.update({"COUNTER": raw.counter, "ALERTS_2": int(values["ALERTS_2"]) + 1})
  extra = CANPacker("hyundai_canfd_generated").make_can_msg("ADRV_0x161", 0, values)
  analyzer.feed_can("sendcan", raw.t + 0.010, 0, extra[0], extra[1])
  analyzer.feed_can("can", raw.t + 0.012, 0x80, extra[0], extra[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  comparison = episode["clusterEvidence"]["hdaReplacementComparison"]
  assert comparison["adrvAlert5ValueMismatches"] == 0
  assert comparison["adrvStockOwnedFieldMismatchFrames"] == 1
  assert comparison["adrvStockOwnedFieldMismatchCounts"] == {"ALERTS_2": 1}
  assert "changed received ADRV fields: ALERTS_2" in episode["reasons"][0]


def test_extra_mutated_lfahda_state_after_proof_window_is_direct_failure() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  raw = min(
    (
      sample for sample in analyzer.cluster_can
      if sample.origin == "vehicle_rx" and sample.address == LFAHDA_CLUSTER_ADDRESS
    ),
    key=lambda sample: abs(sample.t - 1_033.0),
  )
  extra = CANPacker("hyundai_canfd_generated").make_can_msg("LFAHDA_CLUSTER", 0, {
    "COUNTER": raw.counter,
    "HDA_CntrlModSta": 0,
    "HDA_LFA_SymSta": 2,
  })
  analyzer.feed_can("sendcan", raw.t + 0.010, 0, extra[0], extra[1])
  analyzer.feed_can("can", raw.t + 0.012, 0x80, extra[0], extra[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  comparison = episode["clusterEvidence"]["hdaReplacementComparison"]
  assert comparison["stateMismatches"] == 1
  assert comparison["unexpectedLfaHdaReturnedFrames"] == 1
  assert not comparison["lfaHdaPathConsistent"]
  assert "changed the received LFAHDA HDA_CntrlModSta value" in episode["reasons"][0]


def test_extra_mutated_lfahda_stock_owned_field_is_direct_failure() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  raw = min(
    (
      sample for sample in analyzer.cluster_can
      if sample.origin == "vehicle_rx" and sample.address == LFAHDA_CLUSTER_ADDRESS
    ),
    key=lambda sample: abs(sample.t - 1_010.0),
  )
  values = dict(raw.lfahda_stock_owned_values)
  values.update({"COUNTER": raw.counter, "HDA_LFA_SymSta": 2, "HDA_InfoPUDis1": 1})
  extra = CANPacker("hyundai_canfd_generated").make_can_msg("LFAHDA_CLUSTER", 0, values)
  analyzer.feed_can("sendcan", raw.t + 0.010, 0, extra[0], extra[1])
  analyzer.feed_can("can", raw.t + 0.012, 0x80, extra[0], extra[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  comparison = episode["clusterEvidence"]["hdaReplacementComparison"]
  assert comparison["stateMismatches"] == 0
  assert comparison["lfaHdaStockOwnedFieldMismatchFrames"] == 1
  assert comparison["lfaHdaStockOwnedFieldMismatchCounts"] == {"HDA_InfoPUDis1": 1}
  assert "changed received LFAHDA fields: HDA_InfoPUDis1" in episode["reasons"][0]


def test_extra_cluster_rejection_echo_is_direct_failure_even_with_normal_return() -> None:
  analyzer = run_demo("pass")
  request = min(
    (
      sample for sample in analyzer.cluster_can
      if sample.origin == "send_request" and sample.address == ADRV_0X161_ADDRESS
    ),
    key=lambda sample: abs(sample.t - 1_010.0),
  )
  analyzer.feed_can("can", request.t + 0.004, 0xC0, request.address, bytes.fromhex(request.data_hex))

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["clusterEvidence"]["integrity"]["rejectedEchoCount"] == 1
  assert "Panda safety rejected" in episode["reasons"][0]


def test_extra_button_rejection_echo_is_direct_failure_even_with_normal_return() -> None:
  analyzer = run_demo("pass")
  request = min(
    (
      sample for sample in analyzer.buttons
      if sample.origin == "send_request" and sample.button == BUTTON_RES_ACCEL
    ),
    key=lambda sample: abs(sample.t - 1_010.0),
  )
  analyzer.feed_can(
    "can", request.t + 0.004, request.bus + 0xC0, request.address, bytes.fromhex(request.data_hex),
  )

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["rejectedButtonEchoCount"] == 1
  assert "Panda safety rejected" in episode["reasons"][0]


def test_non_res_host_button_request_is_direct_failure() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  message = CANPacker("hyundai_canfd_generated").make_can_msg("CRUISE_BUTTONS_ALT", 2, {
    "COUNTER": 0,
    "CRUISE_BUTTONS": 4,
    "DISTANCE_UNIT": 1,
    "SET_ME_2": 3,
  })
  analyzer.feed_can("sendcan", 1_010.510, 2, message[0], message[1])
  analyzer.feed_can("can", 1_010.512, 0x82, message[0], message[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["hostButtonTraffic"]["unexpectedRequestCount"] == 1
  assert episode["hostButtonTraffic"]["unexpectedReturnedEchoCount"] == 1
  assert not episode["prerequisites"]["onlyResAccelHostButtonRequestsAndReturns"]
  assert "action other than the supported 0x1AA RES schedule" in episode["reasons"][0]


def test_unexplained_res_return_echo_keeps_evidence_inconclusive() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  message = CANPacker("hyundai_canfd_generated").make_can_msg("CRUISE_BUTTONS_ALT", 2, {
    "COUNTER": 0,
    "CRUISE_BUTTONS": BUTTON_RES_ACCEL,
    "DISTANCE_UNIT": 1,
    "SET_ME_2": 3,
  })
  analyzer.feed_can("can", 1_010.512, 0x82, message[0], message[1])

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["hostButtonTraffic"]["requestCount"] + 1 == episode["hostButtonTraffic"]["returnedEchoCount"]
  assert not episode["hostButtonTraffic"]["oneToOne"]
  assert not episode["prerequisites"]["buttonRequestsAndReturnedEchoesOneToOne"]


def test_partially_unpaired_alert5_value5_does_not_claim_preservation() -> None:
  analyzer = run_demo("pass")
  raw = next(
    sample for sample in analyzer.cluster_can
    if sample.origin == "vehicle_rx"
    and sample.address == ADRV_0X161_ADDRESS
    and sample.alert_5 == 5
  )
  returned = min(
    (
      sample for sample in analyzer.cluster_can
      if sample.origin == "tx_returned"
      and sample.address == ADRV_0X161_ADDRESS
      and sample.counter == raw.counter
    ),
    key=lambda sample: abs(sample.t - raw.t),
  )
  analyzer.cluster_can.remove(returned)

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  evidence = report["stopEpisodes"][0]["clusterEvidence"]
  assert evidence["alert5ValuePairCounts"]["preservedByReturnedFrame"] > 0
  assert evidence["alert5ValuePairCounts"]["unpairedRawAlert5Value5Frames"] == 1
  assert evidence["alert5ValuePathObservation"] == "hostReplacementMissingAlert5Value5Pair"
  assert evidence["alert5ValuePreservation"]["assessment"] == "CHANGED_OR_UNPAIRED"
  assert not evidence["alert5ValuePreservation"]["pathPreservesObservedValue5"]


def test_hda2_second_panda_reports_unmodified_observed_alert5_topology() -> None:
  report = run_demo("pass", bus_offset=4, hda2=True).report()
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
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
  assert evidence["alert5ValuePathObservation"] == "hda2RawAlert5Value5NoHostReplacement"
  assert evidence["alert5ValuePreservation"]["assessment"] == "RAW_PATH_WITHOUT_HOST_REPLACEMENT"
  assert evidence["alert5ValuePreservation"]["pathPreservesObservedValue5"]
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


def test_invalid_raw_cluster_checksum_is_direct_failure() -> None:
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
  assert not episode["signalSpecificChecks"]["adrv0x161"]["crcValidAllSamples"]
  assert "failed the Hyundai CAN-FD CRC" in episode["reasons"][0]


def test_demo_rejected_tx_fails_independently_of_early_alert5_value() -> None:
  report = run_demo("fail").report()

  assert report["overallVerdict"] == "FAIL"
  assert report["vehicleAcceptanceVerdict"] == "BLOCKED_BY_CAN_EVIDENCE"
  episode = report["stopEpisodes"][0]
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 30.0
  assert episode["clusterEvidence"]["firstRawAlert5Value5AfterStop"] < 3.1
  assert episode["clusterEvidence"]["candidateAlert5Correlation"] == "EARLY"
  assert any("rejected" in reason for reason in episode["reasons"])
  assert sum(match["status"] == "rejected" for match in report["txMatches"]) == 33


def test_demo_missing_tx_is_inconclusive() -> None:
  report = run_demo("missing-tx").report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  # A missing request lacks the schedule prerequisite. The unrelated early
  # ALERTS_5 field value is not established as a visible-prompt failure.
  assert not report["stopEpisodes"][0]["prerequisites"]["exactSupportedRearmSchedule"]


def test_mid_stop_info_display_4_is_inconclusive_even_with_alert5_value_at_30_seconds() -> None:
  report = run_demo("pass", schedule_mode=SCHEDULE_MODE_MIXED).report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["scheduleMode"] == SCHEDULE_MODE_MIXED
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 1.0
  assert episode["clusterEvidence"]["firstRawAlert5Value5AfterStop"] == 30.0
  assert not episode["returnedSchedule"]["exactSupportedRearmSchedule"]
  assert not episode["prerequisites"]["exactSupportedRearmSchedule"]
  assert "neither supported schedule can be matched" in episode["reasons"][0]


def test_initial_info_display_4_with_early_alert5_value_is_observation_not_failure() -> None:
  report = run_demo(
    "pass",
    schedule_mode=SCHEDULE_MODE_INITIAL_RECOVERY,
    raw_alert5_value5_time=3.0,
  ).report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_WITH_ALERT5_TIMING_DIFFERENCE"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  episode = report["stopEpisodes"][0]
  assert episode["scheduleMode"] == SCHEDULE_MODE_INITIAL_RECOVERY
  assert episode["infoDisplay4Periods"][0]["startAfterStop"] == 0.0
  assert episode["returnedSchedule"]["exactSupportedRearmSchedule"]
  assert episode["clusterEvidence"]["firstRawAlert5Value5AfterStop"] == 3.0
  assert episode["signalSpecificChecks"]["adrv0x161"]["candidateAlert5Correlation"] == "EARLY"
  assert "visible-cluster applicability is not established" in episode["reasons"][0]


def test_late_alert5_value_is_observation_not_failure() -> None:
  report = run_demo("pass", raw_alert5_value5_time=33.0).report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_WITH_ALERT5_TIMING_DIFFERENCE"
  assert report["vehicleAcceptanceVerdict"] == "REQUIRES_ON_CAR_A_B"
  episode = report["stopEpisodes"][0]
  assert episode["clusterEvidence"]["firstRawAlert5Value5AfterStop"] == 33.0
  assert episode["signalSpecificChecks"]["adrv0x161"]["candidateAlert5Correlation"] == "LATE"
  assert "visible-cluster applicability is not established" in episode["reasons"][0]


def test_empty_analyzer_is_inconclusive() -> None:
  report = ProbeAnalyzer("test", "empty").report()
  assert report["schemaVersion"] == 6
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["stopEpisodes"] == []
  assert report["stateEvidence"]["sampleCount"] == 0


def test_invalid_pre_stop_boundary_state_cannot_establish_physical_stop_start() -> None:
  analyzer = run_demo("pass")
  index = next(index for index, sample in enumerate(analyzer.states) if not sample.stop_active)
  analyzer.states[index] = replace(analyzer.states[index], can_valid=False)

  report = analyzer.report()

  assert report["stateEvidence"]["canInvalidSamples"] == 1
  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["startObserved"]
  assert not episode["prerequisites"]["physicalStopStartObserved"]


def test_nonfinite_stop_kinematics_cannot_be_normalized_into_zero_speed_proof() -> None:
  analyzer = run_demo("pass")
  analyzer.feed_state(
    1_010.025,
    SimpleNamespace(
      standstill=True,
      vEgo=float("nan"),
      vEgoRaw=float("inf"),
      canValid=True,
      brakePressed=False,
      gasPressed=False,
      brakeHoldActive=False,
      parkingBrake=False,
      accFaulted=False,
      cruiseState=SimpleNamespace(enabled=True, standstill=True),
    ),
  )

  report = analyzer.report()

  assert report["stateEvidence"]["kinematicsNonFiniteSamples"] == 1
  assert report["overallVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["prerequisites"]["carStateKinematicsFiniteAllSamples"]
  assert not episode["prerequisites"]["wheelStandstillAndRawSpeedNearZeroAllSamples"]
  assert "carStateKinematicsFiniteAllSamples" in episode["reasons"][0]


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
    "kinematicsFinite": True,
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


def test_changed_non_button_field_is_direct_failure() -> None:
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


def test_single_car_control_sample_keeps_can_evidence_inconclusive() -> None:
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


def test_two_sparse_scc_samples_keep_can_evidence_inconclusive() -> None:
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


def test_sparse_raw_buttons_with_every_res_source_keep_can_evidence_inconclusive() -> None:
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


def test_missing_post_burst_release_candidate_keeps_can_evidence_inconclusive() -> None:
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
  assert not episode["streamCoverage"]["rawStockButtons0x1aa"]["continuous"]
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

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "OBSERVED_SCHEDULE_AND_ALERT5_TIMING"
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


@pytest.mark.parametrize(
  "address,prerequisite",
  (
    (SCC_CONTROL_ADDRESS, "stockScc0x1a0TransportOnlyOnPreferredRxBus"),
    (CRUISE_BUTTONS_ALT_ADDRESS, "rawStockButtons0x1aaTransportOnlyOnPreferredRxBus"),
  ),
)
def test_valid_stock_source_frame_on_extra_rx_bus_keeps_evidence_inconclusive(
    address: int, prerequisite: str,
) -> None:
  analyzer = run_demo("pass")
  samples = analyzer.scc if address == SCC_CONTROL_ADDRESS else analyzer.buttons
  source = next(sample for sample in samples
                if sample.bus == 0 and getattr(sample, "address", address) == address)
  analyzer.feed_can("can", source.t + 0.001, 1, address, bytes.fromhex(source.data_hex))

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["prerequisites"][prerequisite]


def test_valid_standard_button_frame_on_any_rx_bus_disproves_alt_only_layout() -> None:
  from opendbc.can import CANPacker

  analyzer = run_demo("pass")
  message = CANPacker("hyundai_canfd_generated").make_can_msg("CRUISE_BUTTONS", 1, {
    "COUNTER": 1,
    "CRUISE_BUTTONS": 0,
  })
  analyzer.feed_can("can", 1_010.0, 1, message[0], message[1])

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["busLayout"]["raw0x1cfTransportObservedOnAnyRxBus"]
  assert not episode["prerequisites"]["rawStandardButtons0x1cfAbsentOnAllRxBuses"]
  assert not episode["prerequisites"]["raw0x1aaStockBusPresentAnd0x1cfAbsent"]


def test_consistently_relocated_stock_sources_do_not_redefine_expected_ecan() -> None:
  analyzer = run_demo("pass")
  analyzer.scc = [replace(sample, src=1, bus=1) for sample in analyzer.scc]
  analyzer.buttons = [
    replace(sample, src=1, bus=1)
    if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
    else sample
    for sample in analyzer.buttons
  ]
  analyzer.streams = {
    (
      service,
      origin,
      1 if origin == "vehicle_rx" and address in (SCC_CONTROL_ADDRESS, CRUISE_BUTTONS_ALT_ADDRESS) else src,
      1 if origin == "vehicle_rx" and address in (SCC_CONTROL_ADDRESS, CRUISE_BUTTONS_ALT_ADDRESS) else bus,
      address,
    ): stat
    for (service, origin, src, bus, address), stat in analyzer.streams.items()
  }

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["busLayout"]["stockBus"] == 1
  assert episode["busLayout"]["expectedStockBus"] == 0
  assert not episode["prerequisites"]["preferredStockRxBusMatchesCarParamsEcan"]


def test_safety_panda_offset_comes_from_matching_config_index() -> None:
  analyzer = run_demo("pass", bus_offset=4)
  cp = _demo_car_params(bus_offset=4)
  cp.safetyConfigs = list(reversed(cp.safetyConfigs))
  analyzer.set_car_params(cp, "reversed-safety-configs")

  report = analyzer.report()

  assert report["carParams"]["hyundaiCanfdSafetyAltButtonsConfigIndices"] == [0]
  assert report["carParams"]["pandaBusOffset"] == 0
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["prerequisites"]["preferredStockRxBusMatchesCarParamsEcan"]
  assert not episode["prerequisites"]["send0x1aaUsesExpectedBus"]


@pytest.mark.parametrize("hda2", (False, True))
def test_safety_hda2_bit_must_match_car_params_topology(hda2: bool) -> None:
  from opendbc.car.hyundai.values import HyundaiSafetyFlags

  analyzer = run_demo("pass", hda2=hda2)
  cp = _demo_car_params(hda2=hda2)
  cp.safetyConfigs[-1].safetyParam ^= int(HyundaiSafetyFlags.CANFD_LKA_STEERING)
  analyzer.set_car_params(cp, "mismatched-hda2-safety-bit")

  report = analyzer.report()

  gate = report["carParams"]["ka4StockSccGate"]
  assert not gate["hyundaiCanfdSafetyHda2MatchesCarParams"]
  assert not report["carParams"]["ka4StockSccGatePassed"]
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"


@pytest.mark.parametrize(
  "safety_flag,gate_name",
  (
    ("LONG", "hyundaiCanfdSafetyStockLongitudinal"),
    ("CAMERA_SCC", "hyundaiCanfdSafetyNotCameraScc"),
  ),
)
def test_stock_scc_gate_rejects_incompatible_safety_longitudinal_flags(
    safety_flag: str, gate_name: str,
) -> None:
  from opendbc.car.hyundai.values import HyundaiSafetyFlags

  analyzer = run_demo("pass")
  cp = _demo_car_params()
  cp.safetyConfigs[-1].safetyParam |= int(getattr(HyundaiSafetyFlags, safety_flag))
  analyzer.set_car_params(cp, f"incompatible-{safety_flag.lower()}-safety-bit")

  report = analyzer.report()

  gate = report["carParams"]["ka4StockSccGate"]
  assert not gate[gate_name]
  assert not report["carParams"]["ka4StockSccGatePassed"]
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"


@pytest.mark.parametrize("address", (SCC_CONTROL_ADDRESS, CRUISE_BUTTONS_ALT_ADDRESS))
def test_duplicate_same_tick_stock_source_is_not_complete_evidence(address: int) -> None:
  analyzer = run_demo("pass")
  samples = analyzer.scc if address == SCC_CONTROL_ADDRESS else analyzer.buttons
  source = next(sample for sample in samples
                if sample.bus == 0 and getattr(sample, "address", address) == address
                and sample.t >= 1_010.0)
  analyzer.feed_can("can", source.t, source.src, address, bytes.fromhex(source.data_hex))

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  integrity_name = (
    "rawScc0x1a0" if address == SCC_CONTROL_ADDRESS else "rawStockButtons0x1aa"
  )
  integrity = report["stopEpisodes"][0]["sourceCounterSequenceIntegrity"][integrity_name]
  assert integrity["tooCloseFramePairs"] > 0
  assert integrity["counterStepViolations"] > 0
  assert not integrity["clean"]


def test_duplicate_raw_request_and_return_cluster_tick_is_not_complete_evidence() -> None:
  analyzer = run_demo("pass")
  originals = [
    sample for sample in analyzer.cluster_can
    if sample.address == ADRV_0X161_ADDRESS and 1_010.0 <= sample.t <= 1_010.003
  ]
  assert {sample.origin for sample in originals} == {"vehicle_rx", "send_request", "tx_returned"}
  for sample in originals:
    service = "sendcan" if sample.origin == "send_request" else "can"
    analyzer.feed_can(service, sample.t, sample.src, sample.address, bytes.fromhex(sample.data_hex))

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  evidence = report["stopEpisodes"][0]["clusterEvidence"]
  for name in ("rawAdrv", "adrvRequests", "adrvReturned"):
    assert not evidence["counterSequenceIntegrity"][name]["clean"]
  assert not evidence["hdaReplacementComparison"]["adrvPathConsistentWithObservedVariant"]


def test_button_tx_return_before_request_is_not_matched() -> None:
  analyzer = run_demo("pass")
  requests = [sample for sample in analyzer.buttons if sample.origin == "send_request"]
  moved = []
  for sample in analyzer.buttons:
    if sample.origin == "tx_returned":
      request = min(
        (candidate for candidate in requests
         if (candidate.address, candidate.bus, candidate.data_hex) ==
         (sample.address, sample.bus, sample.data_hex)),
        key=lambda candidate: abs(candidate.t - sample.t),
      )
      sample = replace(sample, t=request.t - 0.002)
    moved.append(sample)
  analyzer.buttons = moved

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["hostButtonTraffic"]["matchedReturnedRequestCount"] == 0
  assert not episode["prerequisites"]["allObservedRearmRequestsReturned"]
  assert not episode["prerequisites"]["buttonRequestsAndReturnedEchoesOneToOne"]


def test_cluster_tx_return_before_request_is_not_matched() -> None:
  analyzer = run_demo("pass")
  requests = [sample for sample in analyzer.cluster_can if sample.origin == "send_request"]
  moved = []
  for sample in analyzer.cluster_can:
    if sample.origin == "tx_returned":
      request = min(
        (candidate for candidate in requests
         if (candidate.address, candidate.bus, candidate.data_hex) ==
         (sample.address, sample.bus, sample.data_hex)),
        key=lambda candidate: abs(candidate.t - sample.t),
      )
      sample = replace(sample, t=request.t - 0.002)
    moved.append(sample)
  analyzer.cluster_can = moved

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  evidence = report["stopEpisodes"][0]["clusterEvidence"]
  assert not evidence["hdaReplacementComparison"]["allAdrvRequestsReturned"]
  assert not evidence["hdaReplacementComparison"]["allLfaHdaRequestsReturned"]
  assert not evidence["hdaReplacementComparison"]["pathConsistentWithCurrentStockLongTopology"]


@pytest.mark.parametrize("address", (SCC_CONTROL_ADDRESS, CRUISE_BUTTONS_ALT_ADDRESS))
@pytest.mark.parametrize("boundary_time", (999.999, 1_035.001))
def test_boundary_decode_error_is_included_in_episode_evidence(
    address: int, boundary_time: float,
) -> None:
  analyzer = run_demo("pass")
  analyzer.feed_can("can", boundary_time, 0, address, b"\x00")

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["decodeErrors"]
  assert not episode["prerequisites"]["allProofRelevantCanFramesDecoded"]


def test_host_res_in_stop_boundary_margin_keeps_episode_inconclusive() -> None:
  analyzer = run_demo("pass")
  request = next(sample for sample in analyzer.buttons if sample.origin == "send_request")
  analyzer.feed_can(
    "sendcan", 999.999, request.src, request.address, bytes.fromhex(request.data_hex),
  )
  analyzer.feed_can(
    "can", 1_000.001, request.src + 0x80, request.address, bytes.fromhex(request.data_hex),
  )

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert episode["hostButtonTraffic"]["boundaryMarginTrafficCount"] == 1
  assert not episode["prerequisites"]["noHostButtonTrafficInEpisodeBoundaryMargin"]


def test_incomplete_second_stop_makes_capture_level_can_evidence_inconclusive() -> None:
  analyzer = run_demo("pass")
  last = max((sample for sample in analyzer.states if sample.stop_active), key=lambda sample: sample.t)
  analyzer.states.extend((
    replace(last, t=last.t + 0.300, standstill=False, v_ego=1.0, v_ego_raw=1.0),
    replace(last, t=last.t + 0.400),
    replace(last, t=last.t + 0.440),
  ))

  report = analyzer.report()

  assert [episode["canEvidenceVerdict"] for episode in report["stopEpisodes"]] == [
    "OBSERVED_SCHEDULE_AND_ALERT5_TIMING",
    "INCONCLUSIVE",
  ]
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"


def test_zero_speed_nonstandstill_sample_does_not_prove_physical_stop_boundary() -> None:
  analyzer = run_demo("pass")
  pre_stop_index = min(range(len(analyzer.states)), key=lambda index: analyzer.states[index].t)
  analyzer.states[pre_stop_index] = replace(
    analyzer.states[pre_stop_index], v_ego=0.0, v_ego_raw=0.0,
  )

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["startObserved"]
  assert not episode["prerequisites"]["physicalStopStartObserved"]


def test_stale_moving_sample_does_not_prove_physical_stop_boundary() -> None:
  analyzer = run_demo("pass")
  pre_stop_index = min(range(len(analyzer.states)), key=lambda index: analyzer.states[index].t)
  analyzer.states[pre_stop_index] = replace(analyzer.states[pre_stop_index], t=999.600)

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["startObserved"]
  assert not episode["prerequisites"]["physicalStopStartObserved"]


@pytest.mark.parametrize("speed", (0.0, 0.2))
def test_invalid_post_stop_state_is_inconclusive_regardless_of_speed(speed: float) -> None:
  analyzer = run_demo("pass")
  last = max(analyzer.states, key=lambda sample: sample.t)
  analyzer.states.append(replace(
    last,
    t=last.t + 0.020,
    standstill=False,
    cruise_standstill=False,
    v_ego=speed,
    v_ego_raw=speed,
    can_valid=False,
  ))

  report = analyzer.report()

  assert report["overallVerdict"] == "INCONCLUSIVE"
  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["potentialFalseStart"]
  assert not episode["prerequisites"]["postStopObservationCanValidAllSamples"]


def test_post_stop_raw_speed_over_raw_gate_is_direct_false_start_evidence() -> None:
  analyzer = run_demo("pass")
  final_stop = max((sample for sample in analyzer.states if sample.stop_active), key=lambda sample: sample.t)
  analyzer.states.append(replace(
    final_stop,
    t=final_stop.t + 0.020,
    standstill=False,
    cruise_standstill=False,
    v_ego=0.0,
    v_ego_raw=0.04,
  ))

  report = analyzer.report()

  assert report["overallVerdict"] == "FAIL"
  episode = report["stopEpisodes"][0]
  assert episode["potentialFalseStart"]
  assert "potential false start" in episode["reasons"][0]


@pytest.mark.parametrize("contradiction", ("invalid", "zero-speed"))
def test_duplicate_pre_stop_tick_cannot_hide_contradictory_state(contradiction: str) -> None:
  analyzer = run_demo("pass")
  pre_stop_index = min(range(len(analyzer.states)), key=lambda index: analyzer.states[index].t)
  pre_stop = analyzer.states[pre_stop_index]
  duplicate = (
    replace(pre_stop, can_valid=False)
    if contradiction == "invalid" else
    replace(pre_stop, v_ego=0.0, v_ego_raw=0.0)
  )
  analyzer.states.insert(pre_stop_index, duplicate)

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  integrity = episode["physicalStopBoundaryStateIntegrity"]
  assert integrity["nonIncreasingTimestampPairs"] > 0
  assert not episode["prerequisites"]["physicalStopBoundaryStateSequenceUnambiguous"]
  if contradiction == "invalid":
    assert not episode["prerequisites"]["physicalStopBoundaryStatesValidAndFinite"]


@pytest.mark.parametrize(
  "stream_name,coverage_name,missing_count",
  (
    ("controls", "carControl", 10),
    ("scc", "rawScc0x1a0", 3),
    ("buttons", "rawStockButtons0x1aa", 5),
  ),
)
def test_missing_proof_start_ticks_fail_source_specific_edge_coverage(
    stream_name: str, coverage_name: str, missing_count: int,
) -> None:
  analyzer = run_demo("pass")
  samples = getattr(analyzer, stream_name)
  if stream_name == "buttons":
    candidates = [
      sample for sample in samples
      if sample.origin == "vehicle_rx" and sample.address == CRUISE_BUTTONS_ALT_ADDRESS
    ][:missing_count]
  else:
    candidates = [sample for sample in samples if sample.t >= 1_000.0][:missing_count]
  candidate_ids = {id(sample) for sample in candidates}
  setattr(analyzer, stream_name, [sample for sample in samples if id(sample) not in candidate_ids])

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  coverage = report["stopEpisodes"][0]["streamCoverage"][coverage_name]
  assert not coverage["startEdgeCovered"]
  assert not coverage["continuous"]


def test_missing_post_stop_observation_keeps_capture_inconclusive() -> None:
  analyzer = run_demo("pass")
  final_stop_t = max(sample.t for sample in analyzer.states if sample.stop_active)
  analyzer.states = [sample for sample in analyzer.states if sample.t <= final_stop_t]

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  episode = report["stopEpisodes"][0]
  assert not episode["prerequisites"]["postStopObservationAvailable"]
  assert not episode["prerequisites"]["postStopObservationKinematicsFinite"]
  assert not episode["prerequisites"]["postStopObservationCanValidAllSamples"]


def test_scc_99ms_gap_is_not_accepted_as_complete_50hz_evidence() -> None:
  analyzer = run_demo("pass")
  analyzer.scc = [
    replace(sample, t=sample.t + 0.079) if sample.t >= 1_010.0 else sample
    for sample in analyzer.scc
  ]

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  integrity = report["stopEpisodes"][0]["sourceCounterSequenceIntegrity"]["rawScc0x1a0"]
  assert integrity["maximumObservedGapMs"] == 99.0
  assert integrity["cadenceViolationPairs"] > 0
  assert not integrity["clean"]


@pytest.mark.parametrize("stream_name", ("states", "controls"))
def test_missing_100hz_service_sample_makes_capture_inconclusive(stream_name: str) -> None:
  analyzer = run_demo("pass")
  samples = getattr(analyzer, stream_name)
  missing = min((sample for sample in samples if sample.t >= 1_010.0), key=lambda sample: sample.t)
  setattr(analyzer, stream_name, [sample for sample in samples if sample is not missing])

  report = analyzer.report()

  assert report["canEvidenceVerdict"] == "INCONCLUSIVE"
  service = "carState" if stream_name == "states" else "carControl"
  integrity = report["stopEpisodes"][0]["serviceCadenceIntegrity"][service]
  assert integrity["maximumObservedGapMs"] == 20.0
  assert integrity["cadenceViolationPairs"] > 0
  assert not integrity["clean"]
