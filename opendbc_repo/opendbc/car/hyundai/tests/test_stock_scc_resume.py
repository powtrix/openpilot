from collections import deque
from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carcontroller import (
  KA4_STOCK_SCC_BUTTON_SOURCE_PERIOD_FRAMES,
  KA4_STOCK_SCC_MAX_NEAR_ZERO_SPEED,
  CarController,
  _dk_ka4_runtime_branch,
)
from opendbc.car.hyundai.hyundaicanfd import create_lfa_icon_non_camera_scc, create_lfahda_cluster
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags


class FakePacker:
  @staticmethod
  def make_can_msg(name, bus, values, **kwargs):
    return name, bus, values.copy(), kwargs


def build_controller(*, alt_buttons=True):
  controller = CarController.__new__(CarController)
  flags = HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC
  if alt_buttons:
    flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  controller.CP = SimpleNamespace(
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    flags=flags,
  )
  controller.frame = 0
  controller.stock_scc_stop_start_frame = None
  controller.stock_scc_near_zero_frames = 0
  controller.stock_scc_near_zero_start_frame = None
  controller.stock_scc_stopped_lead_frames = 0
  controller.stock_scc_keepalive_pending = False
  controller.stock_scc_keepalive_pending_frame = None
  controller.stock_scc_keepalive_press_frames = 0
  controller.stock_scc_keepalive_warning_recovery = False
  controller.stock_scc_warning_recovery_requested = False
  controller.stock_scc_keepalive_requested = False
  controller.stock_scc_keepalive_request_count = 0
  controller.stock_scc_last_keepalive_frame = None
  controller.stock_scc_button_source_counter = None
  controller.stock_scc_alert_stop_start_frame = None
  controller.stock_scc_alert_abort_latched = False
  controller.stock_scc_resume_alert_suppressed = False
  controller.ka4_stock_scc_standstill_rearm = True
  controller.dk_ka4_runtime_branch = True
  controller.activateCruise = 0
  controller.last_button_frame = 0
  controller.last_cancel_frame = -1_000_000
  controller.button_wait = 12
  controller.button_spam1 = 8
  controller.button_spam2 = 30
  controller.button_spamming_count = 0
  controller.prev_clu_speed = 0
  controller.speed_from_pcm = 0
  controller.button_spam3 = 1
  controller.cruise_buttons_msg_cnt = 0
  controller.cruise_buttons_msg_values = None
  controller.packer = FakePacker()
  controller.CAN = SimpleNamespace(ECAN=0, CAM=2)
  return controller


def build_state(*, info_display=0, acc_mode=1, acc_obj_dist=5.0, acc_obj_rel_spd=0.0,
                hud_lead_info=2, sys_fail_state=0, take_over_req=0, brake_pressed=False, gas_pressed=False,
                brake_hold_active=False, parking_brake=False, standstill=True, v_ego=0.0, v_ego_raw=0.0,
                can_valid=True, adrv_0x161=None):
  return SimpleNamespace(
    is_metric=True,
    scc_control={
      "InfoDisplay": info_display,
      "ACCMode": acc_mode,
      "ACC_ObjDist": acc_obj_dist,
      "ACC_ObjRelSpd": acc_obj_rel_spd,
      "HUD_LEAD_INFO": hud_lead_info,
      "SysFailState": sys_fail_state,
      "TakeOverReq": take_over_req,
    },
    adrv_0x161=adrv_0x161,
    buttons_counter=0,
    cruise_buttons_msg={
      "COUNTER": 0,
      "CRUISE_BUTTONS": Buttons.NONE,
      "ADAPTIVE_CRUISE_MAIN_BTN": 0,
      "NORMAL_CRUISE_MAIN_BTN": 0,
      "LFA_BTN": 0,
    },
    cruise_buttons=deque([Buttons.NONE]),
    main_buttons=deque([Buttons.NONE]),
    out=SimpleNamespace(
      standstill=standstill,
      brakePressed=brake_pressed,
      gasPressed=gas_pressed,
      brakeHoldActive=brake_hold_active,
      parkingBrake=parking_brake,
      accFaulted=False,
      canValid=can_valid,
      activateCruise=False,
      latEnabled=True,
      vEgo=v_ego,
      vEgoRaw=v_ego_raw,
      cruiseState=SimpleNamespace(enabled=True, speed=80 / 3.6),
    ),
  )


def build_control(*, lead_visible=True, lead_radar=1, lead_distance=5.0, lead_rel_speed=0.0):
  return SimpleNamespace(
    enabled=True,
    latActive=True,
    cruiseControl=SimpleNamespace(resume=False, cancel=False),
    hudControl=SimpleNamespace(
      setSpeed=80 / 3.6,
      leadVisible=lead_visible,
      leadRadar=lead_radar,
      leadDistance=lead_distance,
      leadRelSpeed=lead_rel_speed,
    ),
  )


def enter_standstill_resume_state(controller, CC, CS, state_frame=300):
  for frame in range(30):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)
  controller.frame = state_frame
  CS.scc_control["InfoDisplay"] = 4
  controller._update_ka4_stock_scc_keepalive(CC, CS)


def step_controller(controller, CC, CS, frame):
  if frame % KA4_STOCK_SCC_BUTTON_SOURCE_PERIOD_FRAMES == 0:
    if controller.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS and CS.cruise_buttons_msg is not None:
      counter = CS.cruise_buttons_msg.get("COUNTER", 0)
      if isinstance(counter, (list, tuple, deque)):
        next_counter = (int(counter[0]) + 1) & 0xFF
        CS.cruise_buttons_msg["COUNTER"] = [next_counter]
      else:
        CS.cruise_buttons_msg["COUNTER"] = (int(counter) + 1) & 0xFF
    else:
      CS.buttons_counter = (CS.buttons_counter + 1) & 0xF
  controller.frame = frame
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  return controller.create_button_messages(CC, CS, use_clu11=False)


def resume_message_requested(messages):
  return any(message[2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL for message in messages)


def encoded_resume_alert(controller, CC, CS):
  return create_lfa_icon_non_camera_scc(
    FakePacker(), CS, controller.CAN, CC,
    openpilot_longitudinal=False,
    suppress_stock_scc_resume_alert=controller.stock_scc_resume_alert_suppressed,
  )[0][2]["ALERTS_5"]


def pulse_group_starts(pulse_frames):
  return [frame for index, frame in enumerate(pulse_frames)
          if index == 0 or frame != pulse_frames[index - 1] + KA4_STOCK_SCC_BUTTON_SOURCE_PERIOD_FRAMES]


@pytest.mark.parametrize(("value", "expected"), [
  ("dkcarrot-wip", True),
  (b"dkcarrot-wip", True),
  ("carrot-wip", False),
  ("carrot", False),
  (None, False),
])
def test_ka4_behavior_is_runtime_gated_to_dk_branch(value, expected):
  params = SimpleNamespace(get=lambda _key: value)
  assert _dk_ka4_runtime_branch(params) is expected


@pytest.mark.parametrize(("target_kph", "ordinary_button"), [
  (30, Buttons.SET_DECEL),
  (100, Buttons.RES_ACCEL),
])
def test_ka4_physical_stop_blocks_ordinary_set_speed_sync_buttons(target_kph, ordinary_button):
  controller = build_controller()
  controller.frame = 100
  CC = build_control()
  CC.hudControl.setSpeed = target_kph / 3.6
  CS = build_state()

  assert controller.make_spam_button(CC, CS) == Buttons.NONE

  # Comparison/recovery branches retain their existing behavior even with the
  # same vehicle fingerprint.
  controller.dk_ka4_runtime_branch = False
  assert controller.make_spam_button(CC, CS) == ordinary_button


def test_ka4_physical_stop_blocks_automatic_cruise_activation():
  controller = build_controller()
  controller.frame = 100
  CC = build_control()
  CS = build_state()
  CS.out.cruiseState.enabled = False

  assert controller.make_spam_button(CC, CS) == Buttons.NONE
  assert controller.activateCruise == 0


def test_ka4_physical_stop_still_allows_planner_qualified_departure_resume():
  controller = build_controller()
  controller.frame = 100
  CC = build_control()
  CC.cruiseControl.resume = True
  CS = build_state()

  assert controller.make_spam_button(CC, CS) == Buttons.RES_ACCEL


@pytest.mark.parametrize(("standstill", "v_ego_raw"), [
  (True, 0.0),
  (True, KA4_STOCK_SCC_MAX_NEAR_ZERO_SPEED + 0.001),
  (False, 0.2),
])
@pytest.mark.parametrize("interlock", [
  "brake", "gas", "auto_hold", "parking_brake", "driver_set", "main", "raw_lfa",
  "acc_fault", "can_invalid", "scc_missing", "scc_failure", "takeover", "cancel",
])
def test_ka4_planner_departure_resume_obeys_every_current_frame_interlock(interlock, standstill, v_ego_raw):
  controller = build_controller()
  controller.frame = 100
  CC = build_control()
  CC.cruiseControl.resume = True
  CS = build_state(standstill=standstill, v_ego=v_ego_raw, v_ego_raw=v_ego_raw)

  if interlock == "brake":
    CS.out.brakePressed = True
  elif interlock == "gas":
    CS.out.gasPressed = True
  elif interlock == "auto_hold":
    CS.out.brakeHoldActive = True
  elif interlock == "parking_brake":
    CS.out.parkingBrake = True
  elif interlock == "driver_set":
    CS.cruise_buttons[-1] = Buttons.SET_DECEL
  elif interlock == "main":
    CS.main_buttons[-1] = 1
  elif interlock == "raw_lfa":
    controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
    CS.cruise_buttons_msg = {"CRUISE_BUTTONS": 0, "LFA_BTN": 1}
  elif interlock == "acc_fault":
    CS.out.accFaulted = True
  elif interlock == "can_invalid":
    CS.out.canValid = False
  elif interlock == "scc_missing":
    CS.scc_control = None
  elif interlock == "scc_failure":
    CS.scc_control["SysFailState"] = 1
  elif interlock == "takeover":
    CS.scc_control["TakeOverReq"] = 1
  elif interlock == "cancel":
    CC.cruiseControl.cancel = True

  assert controller.make_spam_button(CC, CS, stock_scc_source_fresh=True) == Buttons.NONE


def test_ka4_stock_scc_resume_state_requests_short_resume_press():
  controller = build_controller()
  CC = build_control()
  CS = build_state()

  enter_standstill_resume_state(controller, CC, CS)

  assert controller.stock_scc_keepalive_pending
  pulse_frames = [frame for frame in range(300, 306)
                  if resume_message_requested(step_controller(controller, CC, CS, frame))]

  assert pulse_frames == [300, 302, 304]
  assert controller.stock_scc_keepalive_request_count == 3
  assert not controller.stock_scc_keepalive_pending
  assert not resume_message_requested(step_controller(controller, CC, CS, 306))


def test_ka4_stock_scc_rearm_internal_gate_disables_experimental_behavior():
  controller = build_controller()
  controller.ka4_stock_scc_standstill_rearm = False
  CC = build_control()
  CS = build_state()

  enter_standstill_resume_state(controller, CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert not controller.stock_scc_keepalive_pending
  assert not resume_message_requested(controller.create_button_messages(CC, CS, use_clu11=False))


@pytest.mark.parametrize("state_frame, expected", [(2695, True), (2696, False), (2700, False), (3000, False)])
def test_ka4_stock_scc_rearm_stops_before_thirty_seconds(state_frame, expected):
  controller = build_controller()
  CC = build_control()
  CS = build_state()

  enter_standstill_resume_state(controller, CC, CS, state_frame)

  assert controller.stock_scc_keepalive_pending is expected


@pytest.mark.parametrize("state_kwargs", [
  {"brake_pressed": True},
  {"gas_pressed": True},
  {"brake_hold_active": True},
  {"parking_brake": True},
  {"acc_mode": 0},
  {"acc_obj_dist": 0.0},
  {"acc_obj_dist": 25.0},
  {"acc_obj_rel_spd": 0.6},
  {"hud_lead_info": 0},
  {"hud_lead_info": 1},
  {"hud_lead_info": 4},
  {"sys_fail_state": 1},
  {"take_over_req": 1},
])
def test_ka4_stock_scc_rearm_requires_safe_stationary_lead(state_kwargs):
  controller = build_controller()
  CC = build_control()
  CS = build_state(**state_kwargs)

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("relative_speed", [-0.5, 0.3, 0.5])
def test_ka4_stock_scc_rearm_allows_non_departing_relative_speed(relative_speed):
  controller = build_controller()
  CC = build_control()
  CS = build_state(acc_obj_rel_spd=relative_speed)

  enter_standstill_resume_state(controller, CC, CS)

  assert controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("hud_lead_info", [2, 3])
def test_ka4_stock_scc_rearm_accepts_both_supported_hda_lead_states(hud_lead_info):
  controller = build_controller()
  CC = build_control()
  CS = build_state(hud_lead_info=hud_lead_info)

  enter_standstill_resume_state(controller, CC, CS)

  assert controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("relative_speed", [-2.0, -0.6, 0.6, 2.0])
def test_ka4_stock_scc_rearm_rejects_moving_lead_in_either_direction(relative_speed):
  controller = build_controller()
  CC = build_control()
  CS = build_state(acc_obj_rel_spd=relative_speed)

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_rearm_cancels_on_driver_button():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  CS.cruise_buttons[-1] = Buttons.SET_DECEL

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_rearm_cancels_on_main_button():
  controller = build_controller()
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})
  CS.main_buttons[-1] = 1

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


def test_ka4_stock_scc_rearm_cancels_on_acc_fault():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  CS.out.accFaulted = True

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


@pytest.mark.parametrize(("signal", "value"), [
  ("CRUISE_BUTTONS", Buttons.SET_DECEL),
  ("CRUISE_BUTTONS", Buttons.CANCEL),
  ("ADAPTIVE_CRUISE_MAIN_BTN", 1),
  ("NORMAL_CRUISE_MAIN_BTN", 1),
  ("LFA_BTN", 1),
])
def test_ka4_alt_rearm_vetoes_raw_source_button_even_if_deque_is_stale(signal, value):
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})
  CS.cruise_buttons_msg = {
    "COUNTER": 17,
    "CRUISE_BUTTONS": Buttons.NONE,
    "ADAPTIVE_CRUISE_MAIN_BTN": 0,
    "NORMAL_CRUISE_MAIN_BTN": 0,
    "LFA_BTN": 0,
    signal: value,
  }

  enter_standstill_resume_state(controller, CC, CS)

  assert CS.cruise_buttons[-1] == Buttons.NONE
  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


@pytest.mark.parametrize("hud_lead_info", [0, 1, 4])
def test_ka4_stock_scc_rearm_cancels_if_lead_control_state_changes(hud_lead_info):
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)
  assert controller.stock_scc_keepalive_pending

  CS.scc_control["HUD_LEAD_INFO"] = hud_lead_info
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("abort_case", [
  "brake", "gas", "auto_hold", "parking_brake", "driver_button", "cruise_disabled",
  "can_invalid", "control_cancel", "movement", "scc_failure", "takeover", "lead_state", "lead_distance",
  "lead_departure", "info_display",
])
def test_ka4_stock_scc_short_press_aborts_on_every_safety_interlock(abort_case):
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)

  assert resume_message_requested(controller.create_button_messages(CC, CS, use_clu11=False))
  assert controller.stock_scc_keepalive_pending

  if abort_case == "brake":
    CS.out.brakePressed = True
  elif abort_case == "gas":
    CS.out.gasPressed = True
  elif abort_case == "auto_hold":
    CS.out.brakeHoldActive = True
  elif abort_case == "parking_brake":
    CS.out.parkingBrake = True
  elif abort_case == "driver_button":
    CS.cruise_buttons[-1] = Buttons.SET_DECEL
  elif abort_case == "cruise_disabled":
    CC.enabled = False
    CS.out.cruiseState.enabled = False
  elif abort_case == "can_invalid":
    CS.out.canValid = False
  elif abort_case == "control_cancel":
    CC.cruiseControl.cancel = True
  elif abort_case == "movement":
    CS.out.standstill = False
    CS.out.vEgo = 0.2
  elif abort_case == "scc_failure":
    CS.scc_control["SysFailState"] = 1
  elif abort_case == "takeover":
    CS.scc_control["TakeOverReq"] = 1
  elif abort_case == "lead_state":
    CS.scc_control["HUD_LEAD_INFO"] = 1
  elif abort_case == "lead_distance":
    CS.scc_control["ACC_ObjDist"] = 25.0
  elif abort_case == "lead_departure":
    CS.scc_control["ACC_ObjRelSpd"] = 0.6
  elif abort_case == "info_display":
    CS.scc_control["InfoDisplay"] = 5

  messages = step_controller(controller, CC, CS, controller.frame + 1)

  assert not resume_message_requested(messages)
  assert not controller.stock_scc_keepalive_pending
  assert controller.stock_scc_keepalive_press_frames == 0
  assert not controller.stock_scc_resume_alert_suppressed


def test_ka4_stock_scc_ignores_front_vehicle_departure_notice():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  controller.frame = 300
  CS.scc_control["InfoDisplay"] = 5
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_front_departure_notice_does_not_arm_stale_resume_state_edge():
  controller = build_controller()
  CC = build_control()
  CS = build_state()

  controller._update_ka4_stock_scc_keepalive(CC, CS)
  CS.scc_control["InfoDisplay"] = 5
  for frame in range(1, 300):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  controller.frame = 300
  CS.scc_control["InfoDisplay"] = 4
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_info_display_4_at_stop_start_waits_for_dwell_then_requests_exact_burst():
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=4)
  pulse_frames = []

  for frame in range(100):
    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)

  assert pulse_frames == [30, 32, 34]
  assert not any(frame < 30 for frame in pulse_frames)


def test_ka4_stock_scc_early_info_display_recovery_is_blocked_when_acc_mode_closes_before_dwell():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  pulse_frames = []

  for frame in range(300):
    if frame == 10:
      CS.scc_control["InfoDisplay"] = 4
    if frame == 20:
      # Keep the higher-level cruiseState fixture enabled so this assertion
      # specifically proves that raw SCC ACCMode closes the controller gate.
      CS.scc_control["ACCMode"] = 4
    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)

  assert pulse_frames == []
  assert controller.stock_scc_stop_start_frame is None
  assert controller.stock_scc_near_zero_frames == 0
  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_alerts_5_resume_prompt_triggers_early_recovery():
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=0, adrv_0x161={"ALERTS_5": 5})

  pulse_frames = [frame for frame in range(250)
                  if resume_message_requested(step_controller(controller, CC, CS, frame))]

  assert pulse_frames == [30, 32, 34]
  assert controller.stock_scc_warning_recovery_requested


@pytest.mark.parametrize("interlock", [
  "none", "brake", "gas", "auto_hold", "parking_brake", "cancel", "fault", "can_invalid",
])
def test_ka4_stock_scc_always_preserves_oem_resume_alert(interlock):
  controller = build_controller()
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})

  if interlock == "brake":
    CS.out.brakePressed = True
  elif interlock == "gas":
    CS.out.gasPressed = True
  elif interlock == "auto_hold":
    CS.out.brakeHoldActive = True
  elif interlock == "parking_brake":
    CS.out.parkingBrake = True
  elif interlock == "cancel":
    CC.cruiseControl.cancel = True
  elif interlock == "fault":
    CS.out.accFaulted = True
  elif interlock == "can_invalid":
    CS.out.canValid = False

  controller.frame = 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  assert not controller.stock_scc_resume_alert_suppressed
  assert controller.stock_scc_alert_stop_start_frame is None
  assert not controller.stock_scc_alert_abort_latched
  assert encoded_resume_alert(controller, CC, CS) == 5


def test_ka4_stock_scc_hda2_does_not_replace_resume_alert():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_HDA2
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})

  for frame in range(31):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


def test_ka4_stock_scc_info_display_4_at_096_seconds_triggers_immediate_recovery():
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=0, acc_mode=1)
  pulse_frames = []

  for frame in range(105):
    if frame == 96:
      CS.scc_control["InfoDisplay"] = 4
    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)

  assert pulse_frames == [96, 98, 100]
  assert controller.stock_scc_warning_recovery_requested


@pytest.mark.parametrize("info_display", [5, 6, 7])
def test_ka4_stock_scc_info_display_5_through_7_block_raw_lead_gate(info_display):
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=info_display)

  pulse_frames = [frame for frame in range(300)
                  if resume_message_requested(step_controller(controller, CC, CS, frame))]

  assert pulse_frames == []
  assert controller.stock_scc_stop_start_frame is None
  assert controller.stock_scc_near_zero_frames == 0
  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_active_resume_state_gets_one_fast_response_then_normal_cadence():
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=4)
  pulse_frames = []

  for frame in range(600):
    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)

  starts = pulse_group_starts(pulse_frames)
  assert starts == [30, 284, 538]
  assert all(next_start - start >= 250 for start, next_start in zip(starts, starts[1:], strict=False))


def test_ka4_stock_scc_enabled_transient_does_not_reset_thirty_second_epoch():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  for frame in range(31):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  controller.frame = 2600
  CC.enabled = False
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  CC.enabled = True
  CS.scc_control["InfoDisplay"] = 4
  controller.frame = 5000
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame == 0
  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_schedule_epoch_starts_when_scc_engages_after_manual_stop():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  CC.enabled = False
  CS.out.cruiseState.enabled = False

  for frame in range(6000):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None

  CC.enabled = True
  CS.out.cruiseState.enabled = True
  for frame in range(6000, 6300):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)
  CS.scc_control["InfoDisplay"] = 4
  controller.frame = 6300
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame == 6000
  assert controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_epoch_does_not_start_during_generic_standstill_crawl():
  controller = build_controller()
  CC = build_control()
  CS = build_state(v_ego=0.1, v_ego_raw=0.1)

  # 0.1 m/s is below CarState's generic 0.375 km/h standstill threshold,
  # but it is still a crawl and must not consume the finite stopped-lead epoch.
  for frame in range(400):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert controller.stock_scc_near_zero_frames == 0


def test_ka4_stock_scc_epoch_does_not_start_above_near_zero_boundary():
  controller = build_controller()
  CC = build_control()
  CS = build_state(v_ego_raw=KA4_STOCK_SCC_MAX_NEAR_ZERO_SPEED + 1e-6)

  for frame in range(31):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert controller.stock_scc_near_zero_frames == 0


@pytest.mark.parametrize("v_ego_raw", [0.0, KA4_STOCK_SCC_MAX_NEAR_ZERO_SPEED])
def test_ka4_stock_scc_epoch_backdates_to_first_stable_near_zero_sample(v_ego_raw):
  controller = build_controller()
  CC = build_control()
  CS = build_state(v_ego_raw=v_ego_raw)

  for frame in range(31):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame == 0


def test_ka4_stock_scc_epoch_waits_for_stable_safe_lead_before_starting():
  controller = build_controller()
  CC = build_control()
  CS = build_state(hud_lead_info=0)

  for frame in range(500):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None

  CS.scc_control["HUD_LEAD_INFO"] = 3
  for frame in range(500, 531):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame == 500


@pytest.mark.parametrize(("fingerprint", "pcm_cruise", "openpilot_longitudinal"), [
  (CAR.KIA_EV6, True, False),
  (CAR.KIA_CARNIVAL_4TH_GEN, False, False),
  (CAR.KIA_CARNIVAL_4TH_GEN, True, True),
])
def test_stock_scc_rearm_is_limited_to_ka4_stock_longitudinal(fingerprint, pcm_cruise, openpilot_longitudinal):
  controller = build_controller()
  controller.CP.carFingerprint = fingerprint
  controller.CP.pcmCruise = pcm_cruise
  controller.CP.openpilotLongitudinalControl = openpilot_longitudinal
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


def test_stock_scc_rearm_excludes_camera_scc_stock_longitudinal():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CAMERA_SCC
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


@pytest.mark.parametrize("missing_flag", [HyundaiFlags.CANFD, HyundaiFlags.RADAR_SCC, HyundaiFlags.CANFD_ALT_BUTTONS])
def test_stock_scc_rearm_requires_all_exact_topology_flags(missing_flag):
  controller = build_controller()
  controller.CP.flags &= ~missing_flag
  CC = build_control()
  CS = build_state(adrv_0x161={"ALERTS_5": 5})

  enter_standstill_resume_state(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending
  assert not controller.stock_scc_resume_alert_suppressed
  assert encoded_resume_alert(controller, CC, CS) == 5


def test_keepalive_button_is_not_duplicated_by_button_spam_setting():
  controller = build_controller()
  controller.button_spam3 = 20
  CC = build_control()
  CC.cruiseControl.cancel = False
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)

  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  assert len(messages) == 1
  assert messages[0][0] == "CRUISE_BUTTONS_ALT"
  assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
  assert not controller.stock_scc_keepalive_requested


def test_software_cancel_preempts_keepalive_quiet_period():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)

  assert resume_message_requested(controller.create_button_messages(CC, CS, use_clu11=False))
  controller.frame += 1
  CC.cruiseControl.cancel = True
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  cancel_messages = [message for message in messages if message[2]["CRUISE_BUTTONS"] == Buttons.CANCEL]
  assert not resume_message_requested(messages)
  assert len(cancel_messages) == 1
  assert controller.last_button_frame == controller.frame


def test_software_cancel_preempts_keepalive_after_movement_reset():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)

  assert resume_message_requested(controller.create_button_messages(CC, CS, use_clu11=False))
  controller.frame += 1
  CS.out.standstill = False
  CS.out.vEgo = 0.2
  CC.cruiseControl.cancel = True
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  cancel_messages = [message for message in messages if message[2]["CRUISE_BUTTONS"] == Buttons.CANCEL]
  assert not resume_message_requested(messages)
  assert len(cancel_messages) == 1
  assert controller.last_cancel_frame == controller.frame


def test_software_cancel_repeat_rate_is_independent_of_keepalive_state():
  controller = build_controller(alt_buttons=False)
  CC = build_control()
  CS = build_state()
  CC.cruiseControl.cancel = True

  controller.frame = 1
  first_messages = controller.create_button_messages(CC, CS, use_clu11=False)
  assert len(first_messages) == 20

  for frame in range(2, 12):
    controller.frame = frame
    assert controller.create_button_messages(CC, CS, use_clu11=False) == []

  controller.frame = 12
  assert len(controller.create_button_messages(CC, CS, use_clu11=False)) == 20


@pytest.mark.parametrize("list_values", [False, True])
def test_ka4_alt_button_keepalive_accepts_cached_scalar_or_list_values(list_values):
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  controller.button_spam3 = 20
  CC = build_control()
  CC.cruiseControl.cancel = False
  CS = build_state()
  button_values = {"COUNTER": 17, "CRUISE_BUTTONS": Buttons.NONE, "LFA_BTN": 0}
  CS.cruise_buttons_msg = {key: [value] for key, value in button_values.items()} if list_values else button_values
  enter_standstill_resume_state(controller, CC, CS)

  emitted_counters = []
  for frame in range(300, 305):
    messages = step_controller(controller, CC, CS, frame)
    if not messages:
      continue
    assert len(messages) == 1
    assert messages[0][0] == "CRUISE_BUTTONS_ALT"
    assert messages[0][1] == controller.CAN.CAM
    assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
    emitted_counters.append(messages[0][2]["COUNTER"])

  assert emitted_counters == [19, 20, 21]
  assert not controller.stock_scc_keepalive_pending


def test_ka4_alt_button_keepalive_waits_for_source_message():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  CC = build_control()
  CC.cruiseControl.cancel = False
  CS = build_state()
  CS.cruise_buttons_msg = None
  enter_standstill_resume_state(controller, CC, CS)

  assert controller.create_button_messages(CC, CS, use_clu11=False) == []
  assert controller.stock_scc_keepalive_pending

  controller.frame += 2
  CS.cruise_buttons_msg = {"COUNTER": 17, "CRUISE_BUTTONS": Buttons.NONE, "LFA_BTN": 0}
  pulse_frames = []
  for frame in range(controller.frame, controller.frame + 5):
    messages = step_controller(controller, CC, CS, frame)
    if not messages:
      continue
    assert len(messages) == 1
    assert messages[0][0] == "CRUISE_BUTTONS_ALT"
    assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
    pulse_frames.append(frame)

  assert pulse_frames == [302, 304, 306]
  assert not controller.stock_scc_keepalive_pending


def test_ka4_alt_button_resume_state_response_is_retried_if_source_arrives_late():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  CC = build_control()
  CS = build_state(info_display=4)
  CS.cruise_buttons_msg = None

  for frame in range(81):
    assert not resume_message_requested(step_controller(controller, CC, CS, frame))

  assert not controller.stock_scc_warning_recovery_requested
  CS.cruise_buttons_msg = {"COUNTER": 17, "CRUISE_BUTTONS": Buttons.NONE, "LFA_BTN": 0}
  messages = step_controller(controller, CC, CS, 81)

  assert resume_message_requested(messages)
  assert controller.stock_scc_warning_recovery_requested


def test_ka4_stock_scc_rearm_resets_after_vehicle_moves():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_resume_state(controller, CC, CS)
  assert controller.stock_scc_keepalive_pending

  CS.out.standstill = False
  CS.out.vEgo = 0.2
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert controller.stock_scc_alert_stop_start_frame is None
  assert not controller.stock_scc_alert_abort_latched
  assert not controller.stock_scc_keepalive_pending
  assert controller.stock_scc_last_keepalive_frame is None


def test_ka4_stock_scc_synthetic_schedule_delays_modeled_resume_state_until_thirty_seconds():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  modeled_state_deadline = 300
  modeled_state_frames = []
  pulse_frames = []

  for frame in range(3050):
    modeled_state_active = frame >= modeled_state_deadline
    CS.scc_control["InfoDisplay"] = 4 if modeled_state_active else 0
    if modeled_state_active:
      modeled_state_frames.append(frame)

    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)
      # Advance only this test's synthetic InfoDisplay schedule. This does not
      # prove SCC acceptance, an OEM timing change, or a visible cluster state.
      modeled_state_deadline = frame + 300

  starts = pulse_group_starts(pulse_frames)
  assert starts[0] == 250
  assert starts[-1] == 2696
  assert all([frame for frame in pulse_frames if start <= frame <= start + 4] == [start, start + 2, start + 4]
             for start in starts)
  assert modeled_state_frames[0] == 3000
  assert all(frame >= 3000 for frame in modeled_state_frames)
  assert not any(frame > 2700 for frame in pulse_frames)


@pytest.mark.parametrize("state_frame", [2695, 2697])
def test_ka4_stock_scc_reserves_a_distinct_final_press_window(state_frame):
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  pulse_frames = []

  for frame in range(2701):
    CS.scc_control["InfoDisplay"] = 4 if frame >= state_frame else 0
    if resume_message_requested(step_controller(controller, CC, CS, frame)):
      pulse_frames.append(frame)

  assert pulse_group_starts(pulse_frames)[-1] == 2696
  assert pulse_frames[-3:] == [2696, 2698, 2700]


@pytest.mark.parametrize("oem_hda_state", [0, 1, 2])
def test_stock_longitudinal_preserves_oem_hda_state(oem_hda_state):
  original = {
    "COUNTER": 17,
    "HDA_CntrlModSta": oem_hda_state,
    "HDA_LFA_SymSta": 1,
    "LFA_OptUsmSta": 2,
    "HDA_OptUsmSta": 2,
  }
  original_copy = deepcopy(original)
  CS = SimpleNamespace(lfahda_cluster=original)
  CAN = SimpleNamespace(ECAN=0)

  msg = create_lfahda_cluster(
    FakePacker(), CS, CAN, long_active=False, lat_active=True, openpilot_longitudinal=False,
  )[0]

  assert msg[2]["HDA_CntrlModSta"] == oem_hda_state
  assert msg[2]["HDA_LFA_SymSta"] == 2
  assert msg[3]["rx_counter"] == 17
  assert original == original_copy


@pytest.mark.parametrize("alerts_5", [3, 4, 5])
def test_stock_longitudinal_preserves_received_oem_alert_sound_daw_and_mute_fields(alerts_5):
  original = {
    "COUNTER": 17,
    "LFA_ICON": 0,
    "LKA_ICON": 0,
    "ALERTS_1": 0,
    "ALERTS_2": 21,
    "ALERTS_3": 26,
    "ALERTS_5": alerts_5,
    "MUTE": 1,
    "DAW_ICON": 2,
    "SOUNDS_1": 1,
    "SOUNDS_2": 2,
    "SOUNDS_3": 3,
    "SOUNDS_4": 4,
  }
  original_copy = deepcopy(original)
  CS = SimpleNamespace(adrv_0x161=original, out=SimpleNamespace(latEnabled=True))
  CC = SimpleNamespace(latActive=True)
  CAN = SimpleNamespace(ECAN=0)

  msg = create_lfa_icon_non_camera_scc(
    FakePacker(), CS, CAN, CC, openpilot_longitudinal=False,
  )[0]

  assert msg[2]["LFA_ICON"] == 2
  assert msg[2]["LKA_ICON"] == 4
  assert msg[3]["rx_counter"] == 17
  for signal in ("ALERTS_1", "ALERTS_2", "ALERTS_3", "ALERTS_5", "MUTE", "DAW_ICON",
                 "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4"):
    assert msg[2][signal] == original[signal]
  assert original == original_copy


@pytest.mark.parametrize(("alerts_5", "expected"), [(3, 3), (4, 4), (5, 0)])
def test_stock_longitudinal_masks_only_the_qualified_resume_alert(alerts_5, expected):
  original = {
    "COUNTER": 17,
    "LFA_ICON": 0,
    "LKA_ICON": 0,
    "ALERTS_1": 1,
    "ALERTS_2": 21,
    "ALERTS_3": 26,
    "ALERTS_5": alerts_5,
    "MUTE": 1,
    "DAW_ICON": 2,
    "SOUNDS_1": 1,
    "SOUNDS_2": 2,
    "SOUNDS_3": 3,
    "SOUNDS_4": 4,
  }
  CS = SimpleNamespace(adrv_0x161=original, out=SimpleNamespace(latEnabled=True))
  CC = SimpleNamespace(latActive=True)
  CAN = SimpleNamespace(ECAN=0)

  msg = create_lfa_icon_non_camera_scc(
    FakePacker(), CS, CAN, CC,
    openpilot_longitudinal=False,
    suppress_stock_scc_resume_alert=True,
  )[0]

  assert msg[2]["ALERTS_5"] == expected
  for signal in ("ALERTS_1", "ALERTS_2", "ALERTS_3", "MUTE", "DAW_ICON",
                 "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4"):
    assert msg[2][signal] == original[signal]


def test_openpilot_longitudinal_retains_existing_adrv_field_suppression():
  original = {
    "COUNTER": 4,
    "LFA_ICON": 0,
    "LKA_ICON": 0,
    "ALERTS_1": 0,
    "ALERTS_2": 21,
    "ALERTS_3": 26,
    "ALERTS_5": 5,
    "DAW_ICON": 2,
    "SOUNDS_1": 1,
    "SOUNDS_2": 2,
    "SOUNDS_3": 3,
    "SOUNDS_4": 4,
  }
  CS = SimpleNamespace(adrv_0x161=original, out=SimpleNamespace(latEnabled=False))
  CC = SimpleNamespace(latActive=False)
  CAN = SimpleNamespace(ECAN=0)

  msg = create_lfa_icon_non_camera_scc(
    FakePacker(), CS, CAN, CC, openpilot_longitudinal=True,
  )[0]

  for signal in ("ALERTS_2", "ALERTS_3", "ALERTS_5", "DAW_ICON",
                 "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4"):
    assert msg[2][signal] == 0


@pytest.mark.parametrize(("long_active", "expected_hda_state"), [(False, 0), (True, 2)])
def test_openpilot_longitudinal_synthesizes_hda_state(long_active, expected_hda_state):
  CS = SimpleNamespace(lfahda_cluster={"COUNTER": 4, "HDA_CntrlModSta": 1, "HDA_LFA_SymSta": 1})
  CAN = SimpleNamespace(ECAN=0)

  msg = create_lfahda_cluster(
    FakePacker(), CS, CAN, long_active=long_active, lat_active=False, openpilot_longitudinal=True,
  )[0]

  assert msg[2]["HDA_CntrlModSta"] == expected_hda_state
  assert msg[2]["HDA_LFA_SymSta"] == 0


def test_lfahda_cluster_requires_oem_source_message():
  CS = SimpleNamespace(lfahda_cluster=None)
  CAN = SimpleNamespace(ECAN=0)

  assert create_lfahda_cluster(
    FakePacker(), CS, CAN, long_active=False, lat_active=False, openpilot_longitudinal=False,
  ) == []
