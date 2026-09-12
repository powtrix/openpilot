from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.hyundaicanfd import alt_cruise_buttons
from opendbc.car.hyundai.tests.test_stock_scc_resume import FakePacker, build_control, build_controller, build_state
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags


@pytest.mark.parametrize(("hda2", "expected_bus"), [(False, 2), (True, 0)])
@pytest.mark.parametrize("offset", [0, 2])
def test_alt_button_encoder_preserves_source_and_repeated_call_is_deterministic(hda2, expected_bus, offset):
  source = {"COUNTER": 254, "CRUISE_BUTTONS": Buttons.NONE, "LFA_BTN": 0, "ADAPTIVE_CRUISE_MAIN_BTN": 0}
  original = deepcopy(source)
  cp = SimpleNamespace(flags=HyundaiFlags.CANFD_HDA2 if hda2 else 0)
  can = SimpleNamespace(ECAN=0, CAM=2)

  first = alt_cruise_buttons(FakePacker(), cp, can, Buttons.RES_ACCEL, source, offset)
  second = alt_cruise_buttons(FakePacker(), cp, can, Buttons.RES_ACCEL, source, offset)

  assert source == original
  assert first == second
  assert first[1] == expected_bus  # This integrity change must not reroute traffic.
  assert first[2]["COUNTER"] == (254 + 1 + offset) % 256
  assert first[2]["CRUISE_BUTTONS"] == Buttons.RES_ACCEL


@pytest.mark.parametrize("button_request", ["planner_resume", "moving_set_sync"])
@pytest.mark.parametrize("stale_source", ["same_counter", "missing_message", "empty_counter"])
def test_ka4_alt_non_cancel_request_waits_for_next_oem_counter(button_request, stale_source):
  controller = build_controller()
  cc = build_control()
  cs = build_state(standstill=False, v_ego=15.0, v_ego_raw=15.0)
  controller.prev_clu_speed = 80
  if button_request == "planner_resume":
    cc.cruiseControl.resume = True
  else:
    cc.hudControl.setSpeed = 70 / 3.6
  cs.cruise_buttons_msg["COUNTER"] = 254
  original = deepcopy(cs.cruise_buttons_msg)
  controller.frame = 100

  first = controller.create_button_messages(cc, cs, use_clu11=False)
  assert first
  assert first[0][1] == 2
  assert first[0][2]["COUNTER"] == 255

  if stale_source == "missing_message":
    cs.cruise_buttons_msg = None
  elif stale_source == "empty_counter":
    cs.cruise_buttons_msg["COUNTER"] = []
  for frame in (101, 102):
    controller.frame = frame
    assert controller.create_button_messages(cc, cs, use_clu11=False) == []

  cs.cruise_buttons_msg = {**original, "COUNTER": 255}
  controller.frame = 103
  following = controller.create_button_messages(cc, cs, use_clu11=False)
  assert following
  assert following[0][1] == 2
  assert following[0][2]["COUNTER"] == 0


@pytest.mark.parametrize("outside_scope", ["non_dk", "other_car"])
@pytest.mark.parametrize("button_request", ["planner_resume", "moving_set_sync"])
def test_fresh_counter_gate_does_not_change_other_branches_or_cars(outside_scope, button_request):
  controller = build_controller()
  cc = build_control()
  cs = build_state(standstill=False, v_ego=15.0, v_ego_raw=15.0)
  controller.prev_clu_speed = 80
  if outside_scope == "non_dk":
    controller.dk_ka4_runtime_branch = False
  else:
    controller.CP.carFingerprint = CAR.KIA_EV6
  if button_request == "planner_resume":
    cc.cruiseControl.resume = True
  else:
    cc.hudControl.setSpeed = 70 / 3.6
  for frame in (100, 101):
    controller.frame = frame
    assert controller.create_button_messages(cc, cs, use_clu11=False)


def test_ka4_cancel_is_not_delayed_by_stale_oem_counter():
  controller = build_controller()
  cc = build_control()
  cs = build_state(standstill=False, v_ego=15.0, v_ego_raw=15.0)
  controller.prev_clu_speed = 80
  cc.cruiseControl.resume = True
  controller.frame = 100
  assert controller.create_button_messages(cc, cs, use_clu11=False)

  cc.cruiseControl.cancel = True
  controller.frame = 101
  messages = controller.create_button_messages(cc, cs, use_clu11=False)
  assert any(message[2]["CRUISE_BUTTONS"] == Buttons.CANCEL for message in messages)
