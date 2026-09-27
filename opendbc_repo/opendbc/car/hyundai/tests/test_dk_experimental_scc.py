from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car import Bus
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.tests.test_stock_scc_resume import build_controller, build_control, build_state, step_controller
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags, HyundaiSafetyFlags
from openpilot.cereal import car


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("target,expected", [(70, Buttons.SET_DECEL), (80, Buttons.NONE), (90, Buttons.RES_ACCEL)])
def test_only_mode2_automatic_speed_increase_is_suppressed(enabled, mode, target, expected):
  controller, cc = build_controller(), build_control()
  cs = build_state(standstill=False, v_ego=20.0, v_ego_raw=20.0)
  controller.dk_experimental_scc = enabled
  controller.speed_from_pcm = mode
  controller.frame, controller.prev_clu_speed = 100, 80
  cc.hudControl.setSpeed = target / 3.6
  if mode == 1 or (enabled and mode == 2 and target > 80):
    expected = Buttons.NONE
  assert controller.make_spam_button(cc, cs, stock_scc_source_fresh=True) == expected


@pytest.mark.parametrize("kind", ["departure", "keepalive", "new_engagement", "activate", "manual_res", "manual_set", "cancel"])
def test_existing_button_and_departure_sequences_are_identical(kind):
  def sequence(enabled):
    controller, cc = build_controller(), build_control()
    cs = build_state(standstill=False, v_ego=20.0, v_ego_raw=20.0)
    controller.dk_experimental_scc = enabled
    controller.speed_from_pcm = 2
    controller.prev_clu_speed = 80
    if kind in ("departure", "keepalive"):
      cs.out.standstill = True
      cs.out.vEgo = cs.out.vEgoRaw = 0.0
      if kind == "departure":
        cc.cruiseControl.resume = True
    elif kind == "new_engagement":
      cs.out.cruiseState.enabled = False
    elif kind == "activate":
      cc.enabled = False
      cs.out.activateCruise = True
    elif kind == "cancel":
      cc.enabled = False
      cc.cruiseControl.cancel = True
    else:
      cs.cruise_buttons[-1] = Buttons.RES_ACCEL if kind == "manual_res" else Buttons.SET_DECEL
      cc.hudControl.setSpeed = 70 / 3.6
    results = []
    for frame in range(400):
      if kind == "keepalive" and frame == 300:
        cs.scc_control["InfoDisplay"] = 4
      before = deepcopy(cs.cruise_buttons_msg)
      messages = step_controller(controller, cc, cs, frame)
      # Source buttons are never overwritten by the experiment.
      assert cs.cruise_buttons_msg["CRUISE_BUTTONS"] == before["CRUISE_BUTTONS"]
      results.append(messages)
    return results

  off, on = sequence(False), sequence(True)
  assert on == off
  if kind not in ("manual_res", "manual_set"):
    assert any(off), "The preserved sequence must exercise actual outgoing messages"


@pytest.mark.parametrize("initial", [False, True, None])
def test_real_controller_initializes_selection_once(monkeypatch, initial):
  import opendbc.car.hyundai.carcontroller as module

  selected = [initial]
  reads = []

  def get(key):
    reads.append(key)
    return selected[0] if key == "DkExperimentalScc" else "dkcarrot-wip"

  params = SimpleNamespace(get=get, get_int=lambda _: 0, get_bool=lambda _: False, put_int_nonblocking=lambda *args: None)
  monkeypatch.setattr(module, "Params", lambda: params)
  cp = car.CarParams.new_message(
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
    pcmCruise=True,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS),
    safetyConfigs=[{"safetyModel": "hyundaiCanfd", "safetyParam": int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)}],
  )
  controller = CarController({Bus.pt: "hyundai_canfd_generated"}, cp)
  expected = initial is not False
  assert controller.dk_experimental_scc == expected
  selected[0] = not expected
  controller.speed_from_pcm = 2
  controller.frame, controller.prev_clu_speed = 100, 80
  cs, cc = build_state(standstill=False, v_ego=20.0, v_ego_raw=20.0), build_control()
  cc.hudControl.setSpeed = 90 / 3.6
  assert controller.make_spam_button(cc, cs, stock_scc_source_fresh=True) == (Buttons.NONE if expected else Buttons.RES_ACCEL)
  assert reads.count("DkExperimentalScc") == 1
