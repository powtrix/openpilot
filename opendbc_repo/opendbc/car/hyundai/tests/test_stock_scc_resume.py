from collections import deque
from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import create_lfahda_cluster
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags


class FakePacker:
  @staticmethod
  def make_can_msg(name, bus, values, **kwargs):
    return name, bus, values.copy(), kwargs


def build_controller():
  controller = CarController.__new__(CarController)
  controller.CP = SimpleNamespace(
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    flags=HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC,
  )
  controller.frame = 0
  controller.stock_scc_stop_start_frame = None
  controller.stock_scc_info_display_active_prev = False
  controller.stock_scc_info_display_inactive_frames = 0
  controller.stock_scc_stopped_lead_frames = 0
  controller.stock_scc_keepalive_pending = False
  controller.stock_scc_keepalive_pending_frame = None
  controller.stock_scc_keepalive_sent = False
  controller.stock_scc_last_keepalive_frame = None
  controller.activateCruise = 0
  controller.last_button_frame = 0
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
                brake_hold_active=False, parking_brake=False):
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
    buttons_counter=0,
    cruise_buttons_msg=None,
    cruise_buttons=deque([Buttons.NONE]),
    out=SimpleNamespace(
      standstill=True,
      brakePressed=brake_pressed,
      gasPressed=gas_pressed,
      brakeHoldActive=brake_hold_active,
      parkingBrake=parking_brake,
      vEgo=0.0,
      cruiseState=SimpleNamespace(enabled=True, speed=80 / 3.6),
    ),
  )


def build_control(*, lead_visible=True, lead_radar=1, lead_distance=5.0, lead_rel_speed=0.0):
  return SimpleNamespace(
    enabled=True,
    cruiseControl=SimpleNamespace(resume=False),
    hudControl=SimpleNamespace(
      setSpeed=80 / 3.6,
      leadVisible=lead_visible,
      leadRadar=lead_radar,
      leadDistance=lead_distance,
      leadRelSpeed=lead_rel_speed,
    ),
  )


def enter_standstill_warning(controller, CC, CS, warning_frame=300):
  for frame in range(30):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)
  controller.frame = warning_frame
  CS.scc_control["InfoDisplay"] = 4
  controller._update_ka4_stock_scc_keepalive(CC, CS)


def test_ka4_stock_scc_warning_requests_one_resume_pulse():
  controller = build_controller()
  CC = build_control()
  CS = build_state()

  enter_standstill_warning(controller, CC, CS)

  assert controller.stock_scc_keepalive_pending
  assert controller.make_spam_button(CC, CS) == Buttons.RES_ACCEL
  assert not controller.stock_scc_keepalive_pending
  assert controller.stock_scc_keepalive_sent
  assert controller.last_button_frame == controller.frame

  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  assert not controller.stock_scc_keepalive_pending
  assert controller.make_spam_button(CC, CS) == Buttons.NONE


@pytest.mark.parametrize("warning_frame, expected", [(2700, True), (2701, False), (3000, False)])
def test_ka4_stock_scc_rearm_stops_before_thirty_seconds(warning_frame, expected):
  controller = build_controller()
  CC = build_control()
  CS = build_state()

  enter_standstill_warning(controller, CC, CS, warning_frame)

  assert controller.stock_scc_keepalive_pending is expected


@pytest.mark.parametrize("state_kwargs", [
  {"brake_pressed": True},
  {"gas_pressed": True},
  {"brake_hold_active": True},
  {"parking_brake": True},
  {"acc_mode": 0},
  {"acc_obj_dist": 0.0},
  {"acc_obj_dist": 25.0},
  {"acc_obj_rel_spd": 0.3},
  {"hud_lead_info": 0},
  {"hud_lead_info": 1},
  {"hud_lead_info": 3},
  {"sys_fail_state": 1},
  {"take_over_req": 1},
])
def test_ka4_stock_scc_rearm_requires_safe_stationary_lead(state_kwargs):
  controller = build_controller()
  CC = build_control()
  CS = build_state(**state_kwargs)

  enter_standstill_warning(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_rearm_cancels_on_driver_button():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  CS.cruise_buttons[-1] = Buttons.SET_DECEL

  enter_standstill_warning(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("hud_lead_info", [0, 1, 3])
def test_ka4_stock_scc_rearm_cancels_if_lead_control_state_changes(hud_lead_info):
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_warning(controller, CC, CS)
  assert controller.stock_scc_keepalive_pending

  CS.scc_control["HUD_LEAD_INFO"] = hud_lead_info
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_ignores_front_vehicle_departure_notice():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  controller.frame = 300
  CS.scc_control["InfoDisplay"] = 5
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_front_departure_notice_does_not_arm_warning_edge():
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


def test_ka4_stock_scc_requires_a_real_inactive_to_warning_edge():
  controller = build_controller()
  CC = build_control()
  CS = build_state(info_display=4)

  for frame in range(300):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_rejects_bouncing_warning_edges():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_warning(controller, CC, CS)
  assert controller.make_spam_button(CC, CS) == Buttons.RES_ACCEL

  CS.scc_control["InfoDisplay"] = 0
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)
  CS.scc_control["InfoDisplay"] = 4
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_enabled_transient_does_not_reset_thirty_second_epoch():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  for frame in range(30):
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


def test_ka4_stock_scc_timer_starts_when_scc_engages_after_manual_stop():
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
  CS = build_state()

  enter_standstill_warning(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_stock_scc_rearm_excludes_camera_scc_stock_longitudinal():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CAMERA_SCC
  CC = build_control()
  CS = build_state()

  enter_standstill_warning(controller, CC, CS)

  assert not controller.stock_scc_keepalive_pending


def test_keepalive_button_is_not_duplicated_by_button_spam_setting():
  controller = build_controller()
  controller.button_spam3 = 20
  CC = build_control()
  CC.cruiseControl.cancel = False
  CS = build_state()
  enter_standstill_warning(controller, CC, CS)

  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  assert len(messages) == 1
  assert messages[0][0] == "CRUISE_BUTTONS"
  assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
  assert not controller.stock_scc_keepalive_sent


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
  enter_standstill_warning(controller, CC, CS)

  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  assert len(messages) == 1
  assert messages[0][0] == "CRUISE_BUTTONS_ALT"
  assert messages[0][1] == controller.CAN.CAM
  assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
  assert messages[0][2]["COUNTER"] == 18
  assert not controller.stock_scc_keepalive_pending


def test_ka4_alt_button_keepalive_waits_for_source_message():
  controller = build_controller()
  controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
  CC = build_control()
  CC.cruiseControl.cancel = False
  CS = build_state()
  enter_standstill_warning(controller, CC, CS)

  assert controller.create_button_messages(CC, CS, use_clu11=False) == []
  assert controller.stock_scc_keepalive_pending

  controller.frame += 1
  CS.cruise_buttons_msg = {"COUNTER": 17, "CRUISE_BUTTONS": Buttons.NONE, "LFA_BTN": 0}
  messages = controller.create_button_messages(CC, CS, use_clu11=False)

  assert len(messages) == 1
  assert messages[0][0] == "CRUISE_BUTTONS_ALT"
  assert messages[0][2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
  assert not controller.stock_scc_keepalive_pending


def test_ka4_stock_scc_rearm_resets_after_vehicle_moves():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  enter_standstill_warning(controller, CC, CS)
  assert controller.stock_scc_keepalive_pending

  CS.out.standstill = False
  CS.out.vEgo = 0.2
  controller.frame += 1
  controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert controller.stock_scc_stop_start_frame is None
  assert not controller.stock_scc_keepalive_pending
  assert controller.stock_scc_last_keepalive_frame is None


def test_ka4_stock_scc_rearms_every_oem_interval_only_until_thirty_seconds():
  controller = build_controller()
  CC = build_control()
  CS = build_state()
  pulse_frames = []

  for frame in range(300):
    controller.frame = frame
    controller._update_ka4_stock_scc_keepalive(CC, CS)

  for warning_frame in range(300, 3001, 300):
    controller.frame = warning_frame
    CS.scc_control["InfoDisplay"] = 4
    controller._update_ka4_stock_scc_keepalive(CC, CS)
    if controller.stock_scc_keepalive_pending:
      assert controller.make_spam_button(CC, CS) == Buttons.RES_ACCEL
      pulse_frames.append(warning_frame)

    CS.scc_control["InfoDisplay"] = 0
    for frame in range(warning_frame + 1, min(warning_frame + 300, 3001)):
      controller.frame = frame
      controller._update_ka4_stock_scc_keepalive(CC, CS)

  assert pulse_frames == list(range(300, 2701, 300))


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
