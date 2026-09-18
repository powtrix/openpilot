from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState
import opendbc.car.hyundai.carstate as carstate_module
from opendbc.car.hyundai.dk_turn_signal_lamps import (
  LAMP_MESSAGES, LAMP_SIGNALS, TURN_SIGNAL_LAMP_TIMEOUT_NS, get_turn_signal_lamps,
)
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params


DBC_NAME = "hyundai_canfd_generated"
START_NS = 1_000_000_000


def make_params(**overrides):
  return SimpleNamespace(**(dict(carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN, flags=HyundaiFlags.CANFD) | overrides))


def make_parser(message_name="BLINKERS", bus=0):
  return CANParser(DBC_NAME, [(message_name, 50)], bus)


def make_frame(message_name="BLINKERS", bus=0, **values):
  return CANPacker(DBC_NAME).make_can_msg(message_name, bus, values)


def make_carstate():
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, gen_empty_fingerprint(), [], False, False, False)
  CP.openpilotLongitudinalControl = False
  return CarState(CP)


def test_schema_defaults_and_accepted_source_round_trip():
  ret = structs.CarState()
  assert not ret.dkTurnSignalLamps.supported
  assert not ret.dkTurnSignalLamps.valid
  assert not ret.dkTurnSignalLamps.left and not ret.dkTurnSignalLamps.right
  assert ret.dkTurnSignalLamps.sourceMonoTime == 0
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  ret.dkTurnSignalLamps = get_turn_signal_lamps(make_params(), parser, "BLINKERS")
  with structs.CarState.from_bytes(ret.to_bytes()) as decoded:
    assert decoded.dkTurnSignalLamps.supported and decoded.dkTurnSignalLamps.valid
    assert decoded.dkTurnSignalLamps.left and not decoded.dkTurnSignalLamps.right
    assert decoded.dkTurnSignalLamps.sourceMonoTime == START_NS


@pytest.mark.parametrize("message_name", LAMP_MESSAGES)
@pytest.mark.parametrize("left,left_alt,right,right_alt", (
  (0, 0, 0, 0), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1),
  (1, 0, 1, 0), (0, 1, 0, 1), (1, 1, 1, 1),
))
def test_raw_left_right_alternate_and_hazard_phases(message_name, left, left_alt, right, right_alt):
  parser = make_parser(message_name)
  parser.update([START_NS, [make_frame(message_name, LEFT_LAMP=left, LEFT_LAMP_ALT=left_alt,
                                     RIGHT_LAMP=right, RIGHT_LAMP_ALT=right_alt)]])
  lamps = get_turn_signal_lamps(make_params(), parser, message_name)
  assert lamps.supported and lamps.valid
  assert lamps.left == bool(left or left_alt)
  assert lamps.right == bool(right or right_alt)


@pytest.mark.parametrize("message_name", LAMP_MESSAGES)
def test_lamp_off_phase_does_not_clear_held_control_blinker(message_name):
  state = make_carstate()
  parsers = state.get_can_parsers(state.CP)
  parser = parsers[Bus.pt]
  setattr(state, "blinkers" if message_name == "BLINKERS" else "blinkers_alt", parser.vl[message_name])
  parser.update([START_NS, [make_frame(message_name, bus=parser.bus, LEFT_LAMP=1, RIGHT_LAMP_ALT=1)]])
  on = state.update_canfd(parsers)
  assert on.leftBlinker and on.rightBlinker
  assert on.dkTurnSignalLamps.valid and on.dkTurnSignalLamps.left and on.dkTurnSignalLamps.right
  parser.update([START_NS + 10_000_000, [make_frame(message_name, bus=parser.bus)]])
  off = state.update_canfd(parsers)
  assert off.leftBlinker and off.rightBlinker
  assert off.dkTurnSignalLamps.valid
  assert not off.dkTurnSignalLamps.left and not off.dkTurnSignalLamps.right
  assert state.left_blinker_cnt == 49 and state.right_blinker_cnt == 49


def test_existing_selected_primary_message_has_priority_over_alternate():
  state = make_carstate()
  parsers = state.get_can_parsers(state.CP)
  parser = parsers[Bus.pt]
  state.blinkers = parser.vl["BLINKERS"]
  state.blinkers_alt = parser.vl["BLINKERS_ALT"]
  parser.update([START_NS, [make_frame(bus=parser.bus, LEFT_LAMP=1),
                            make_frame("BLINKERS_ALT", bus=parser.bus, RIGHT_LAMP=1)]])
  ret = state.update_canfd(parsers)
  assert ret.leftBlinker and not ret.rightBlinker
  assert ret.dkTurnSignalLamps.left and not ret.dkTurnSignalLamps.right

  # Do not switch the display to a different source than the control reader,
  # even if only the alternate message is still arriving.
  parser.update([START_NS + TURN_SIGNAL_LAMP_TIMEOUT_NS + 1,
                 [make_frame("BLINKERS_ALT", bus=parser.bus, RIGHT_LAMP=1)]])
  ret = state.update_canfd(parsers)
  assert ret.leftBlinker and not ret.rightBlinker
  assert ret.dkTurnSignalLamps.supported and not ret.dkTurnSignalLamps.valid


@pytest.mark.parametrize("age,valid", ((-1, False), (0, True), (TURN_SIGNAL_LAMP_TIMEOUT_NS, True),
                                     (TURN_SIGNAL_LAMP_TIMEOUT_NS + 1, False)))
def test_cached_source_expiration_and_future_timestamp(age, valid):
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  parser.update([START_NS + age, []])
  lamps = get_turn_signal_lamps(make_params(), parser, "BLINKERS")
  assert lamps.supported and lamps.valid == valid
  assert lamps.left == valid
  assert lamps.sourceMonoTime == (START_NS if valid else 0)


@pytest.mark.parametrize("message_name", (None, "BLINKERS", "BLINKERS_ALT", "BLINKER_STALKS", "unexpected"))
def test_missing_or_unsupported_message_is_not_registered(message_name):
  parser = CANParser(DBC_NAME, [], 0)
  lamps = get_turn_signal_lamps(make_params(), parser, message_name)
  assert lamps.supported and not lamps.valid
  assert not parser.addresses and not parser.vl and not parser.message_states


@pytest.mark.parametrize("overrides", ({"carFingerprint": CAR.KIA_SORENTO_4TH_GEN}, {"flags": 0}))
def test_other_vehicle_or_non_canfd_is_unsupported(overrides):
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  lamps = get_turn_signal_lamps(make_params(**overrides), parser, "BLINKERS")
  assert not lamps.supported and not lamps.valid and not lamps.left


@pytest.mark.parametrize("message_name", LAMP_MESSAGES)
def test_wrong_bus_and_wrong_payload_size_cannot_supply_lamp_state(message_name):
  parser = make_parser(message_name, bus=1)
  parser.update([START_NS, [make_frame(message_name, bus=2, LEFT_LAMP=1)]])
  assert not get_turn_signal_lamps(make_params(), parser, message_name).valid
  address, data, bus = make_frame(message_name, bus=1, LEFT_LAMP=1)
  parser.update([START_NS + 10_000_000, [(address, data[:-1], bus)]])
  assert not get_turn_signal_lamps(make_params(), parser, message_name).valid


@pytest.mark.parametrize("signal", LAMP_SIGNALS)
def test_inconsistent_signal_timestamps_are_not_current_phase(signal):
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  parser.ts_nanos["BLINKERS"][signal] -= 1
  assert not get_turn_signal_lamps(make_params(), parser, "BLINKERS").valid


@pytest.mark.parametrize("attribute,value", (("counter_fail", 1), ("ignore_checksum", True), ("ignore_counter", True)))
def test_parser_integrity_failures_or_bypasses_hide_lamp_phase(attribute, value):
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  setattr(parser.message_states[0x413], attribute, value)
  assert not get_turn_signal_lamps(make_params(), parser, "BLINKERS").valid


def test_reader_does_not_modify_parser_or_ask_for_control_can_valid():
  parser = make_parser()
  parser.update([START_NS, [make_frame(LEFT_LAMP=1)]])
  before = deepcopy((dict(parser.vl), parser.ts_nanos, parser.dat, parser.message_states, parser.addresses, parser.can_invalid_cnt))
  assert get_turn_signal_lamps(make_params(), parser, "BLINKERS").valid
  after = (dict(parser.vl), parser.ts_nanos, parser.dat, parser.message_states, parser.addresses, parser.can_invalid_cnt)
  assert before == after


def test_optional_display_exception_leaves_all_existing_vehicle_state_unchanged(monkeypatch):
  normal_state = make_carstate()
  failure_state = CarState(normal_state.CP)
  parsers = normal_state.get_can_parsers(normal_state.CP)
  parser = parsers[Bus.pt]
  normal_state.blinkers = failure_state.blinkers = parser.vl["BLINKERS"]
  parser.update([START_NS, [make_frame(bus=parser.bus, LEFT_LAMP=1)]])
  normal = normal_state.update_canfd(parsers).to_dict()
  calls = []

  def failing_display(*args):
    calls.append(args)
    raise RuntimeError("optional lamp telemetry failure")

  monkeypatch.setattr(carstate_module, "get_turn_signal_lamps", failing_display)
  failed = failure_state.update_canfd(parsers).to_dict()
  assert len(calls) == 1
  assert normal.pop("dkTurnSignalLamps")["valid"]
  lamps = failed.pop("dkTurnSignalLamps")
  assert lamps == dict(supported=True, valid=False, left=False, right=False, sourceMonoTime=0)
  assert normal == failed


def test_other_vehicle_does_not_evaluate_optional_lamp_reader(monkeypatch):
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(CAR.KIA_SORENTO_4TH_GEN, gen_empty_fingerprint(), [], False, False, False)
  state = CarState(CP)
  parsers = state.get_can_parsers(CP)
  calls = []

  def unexpected_call(*args):
    calls.append(args)
    raise AssertionError("another vehicle must retain its existing presentation")

  monkeypatch.setattr(carstate_module, "get_turn_signal_lamps", unexpected_call)
  ret = state.update_canfd(parsers)
  assert not calls
  assert not ret.dkTurnSignalLamps.supported and not ret.dkTurnSignalLamps.valid
