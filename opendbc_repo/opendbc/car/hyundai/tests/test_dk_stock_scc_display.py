from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState
import opendbc.car.hyundai.carstate as carstate_module
from opendbc.car.hyundai.dk_stock_scc_display import (
  SCC_CONTROL_ADDRESS, SCC_CONTROL_NAME, STOCK_SCC_DISPLAY_TIMEOUT_NS, get_stock_scc_display,
)
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params


DBC_NAME = "hyundai_canfd_generated"
START_NS = 1_000_000_000


def make_params(**overrides):
  values = dict(carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN, openpilotLongitudinalControl=False, flags=HyundaiFlags.CANFD)
  return SimpleNamespace(**(values | overrides))


def make_parser(bus=0):
  return CANParser(DBC_NAME, [(SCC_CONTROL_NAME, 50)], bus)


def make_frame(*, bus=0, counter=1, **overrides):
  values = dict(COUNTER=counter, ACCMode=1, MainMode_ACC=1, aReqValue=-1.25, aReqRaw=-2.5)
  return CANPacker(DBC_NAME).make_can_msg(SCC_CONTROL_NAME, bus, values | overrides)


def test_schema_defaults_are_unavailable_and_round_trip_preserves_source():
  ret = structs.CarState()
  assert not ret.dkStockScc.valid
  assert not ret.dkStockScc.active
  assert ret.dkStockScc.sourceMonoTime == 0
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  ret.dkStockScc = get_stock_scc_display(make_params(), parser)
  with structs.CarState.from_bytes(ret.to_bytes()) as decoded:
    assert decoded.dkStockScc.valid
    assert decoded.dkStockScc.active
    assert decoded.dkStockScc.accelRequest == pytest.approx(-1.25)
    assert decoded.dkStockScc.rawAccelRequest == pytest.approx(-2.5)
    assert decoded.dkStockScc.sourceMonoTime == START_NS


@pytest.mark.parametrize("accel", (-10.23, -4.0, -1.25, -0.01, 0.0, 0.01, 2.0, 10.24))
def test_received_command_preserves_sign_zero_and_dbc_range(accel):
  parser = make_parser()
  parser.update([START_NS, [make_frame(aReqValue=accel, aReqRaw=-3.0)]])
  ret = get_stock_scc_display(make_params(), parser)
  assert ret.valid and ret.active
  assert ret.accelRequest == pytest.approx(accel, abs=1e-6)
  assert ret.rawAccelRequest == pytest.approx(-3.0)


@pytest.mark.parametrize("mode", range(8))
def test_only_enabled_mode_is_display_active(mode):
  parser = make_parser()
  parser.update([START_NS, [make_frame(ACCMode=mode)]])
  ret = get_stock_scc_display(make_params(), parser)
  assert ret.valid
  assert ret.active == (mode == 1)


@pytest.mark.parametrize(("signal", "value", "active"), (
  ("StopReq", 0, True), ("StopReq", 1, True), ("StopReq", 2, False), ("StopReq", 3, False),
  ("SysFailState", 1, False), ("SysFailState", 2, False), ("SysFailState", 3, False),
  ("TakeOverReq", 1, False), ("TakeOverReq", 2, False), ("TakeOverReq", 3, False),
))
def test_fault_and_stop_status_do_not_invent_or_misattribute_commands(signal, value, active):
  parser = make_parser()
  parser.update([START_NS, [make_frame(**{signal: value}, aReqValue=0.0)]])
  ret = get_stock_scc_display(make_params(), parser)
  assert ret.valid
  assert ret.active == active
  assert ret.accelRequest == pytest.approx(0.0)


@pytest.mark.parametrize("overrides", (
  {"carFingerprint": CAR.KIA_SORENTO_4TH_GEN},
  {"openpilotLongitudinalControl": True},
  {"flags": 0},
))
def test_unsupported_vehicle_or_openpilot_longitudinal_is_unavailable(overrides):
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  ret = get_stock_scc_display(make_params(**overrides), parser)
  assert not ret.valid and not ret.active
  assert ret.sourceMonoTime == 0


def test_missing_message_is_not_registered_or_made_valid_by_display():
  parser = CANParser(DBC_NAME, [], 0)
  assert not get_stock_scc_display(make_params(), parser).valid
  assert not parser.addresses
  assert not parser.vl
  assert not parser.message_states


@pytest.mark.parametrize("bus", (0, 1, 2, 4, 6))
def test_parser_selected_bus_is_respected(bus):
  parser = make_parser(bus)
  parser.update([START_NS, [make_frame(bus=bus + 1)]])
  assert not get_stock_scc_display(make_params(), parser).valid
  parser.update([START_NS + 10_000_000, [make_frame(bus=bus)]])
  assert get_stock_scc_display(make_params(), parser).valid


def test_bad_checksum_does_not_refresh_previous_accepted_command():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  address, data, bus = make_frame(counter=2, aReqValue=-5.0)
  corrupted = bytearray(data)
  corrupted[0] ^= 1
  assert not parser.update([START_NS + 20_000_000, [(address, bytes(corrupted), bus)]])
  ret = get_stock_scc_display(make_params(), parser)
  assert not ret.valid and not ret.active
  assert parser.vl[SCC_CONTROL_NAME]["aReqValue"] == pytest.approx(-1.25)
  assert parser.ts_nanos[SCC_CONTROL_NAME]["aReqValue"] == START_NS
  parser.update([START_NS + STOCK_SCC_DISPLAY_TIMEOUT_NS + 1, []])
  assert not get_stock_scc_display(make_params(), parser).valid


def test_bad_checksum_cancellation_hides_immediately_and_next_good_frame_recovers():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  address, data, bus = make_frame(counter=2, ACCMode=4, aReqValue=0.0)
  corrupted = bytearray(data)
  corrupted[0] ^= 1
  parser.update([START_NS + 20_000_000, [(address, bytes(corrupted), bus)]])
  assert not get_stock_scc_display(make_params(), parser).valid
  parser.update([START_NS + 40_000_000, [make_frame(counter=3, aReqValue=-0.5)]])
  ret = get_stock_scc_display(make_params(), parser)
  assert ret.valid and ret.active
  assert ret.accelRequest == pytest.approx(-0.5)
  assert ret.sourceMonoTime == START_NS + 40_000_000


@pytest.mark.parametrize("flag", ("ignore_counter", "ignore_checksum"))
def test_display_requires_counter_and_checksum_validation_not_bypassed(flag):
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  setattr(parser.message_states[SCC_CONTROL_ADDRESS], flag, True)
  assert not get_stock_scc_display(make_params(), parser).valid


def test_bad_checksum_before_first_accepted_frame_is_unavailable():
  parser = make_parser()
  address, data, bus = make_frame()
  corrupted = bytearray(data)
  corrupted[-1] ^= 1
  assert not parser.update([START_NS, [(address, bytes(corrupted), bus)]])
  assert not get_stock_scc_display(make_params(), parser).valid


def test_one_counter_failure_hides_display_even_before_parser_rejects_frames():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  # The control parser tolerates up to four bad counters; the gauge does not.
  assert parser.update([START_NS + 20_000_000, [make_frame(counter=1, aReqValue=-6.0)]])
  assert not get_stock_scc_display(make_params(), parser).valid
  assert parser.update([START_NS + 40_000_000, [make_frame(counter=2, aReqValue=-0.75)]])
  assert get_stock_scc_display(make_params(), parser).accelRequest == pytest.approx(-0.75)


def test_repeated_bad_counter_is_unavailable_and_does_not_refresh_rejected_timestamp():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  for i in range(1, 7):
    parser.update([START_NS + i * 20_000_000, [make_frame(counter=1)]])
    assert not get_stock_scc_display(make_params(), parser).valid
  assert parser.ts_nanos[SCC_CONTROL_NAME]["aReqValue"] == START_NS + 80_000_000


@pytest.mark.parametrize(("age", "valid"), (
  (-1, False), (0, True), (STOCK_SCC_DISPLAY_TIMEOUT_NS, True), (STOCK_SCC_DISPLAY_TIMEOUT_NS + 1, False),
))
def test_signal_lifetime_uses_current_update_time_not_last_nonempty_bus_time(age, valid):
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  parser.update([START_NS + age, []])
  ret = get_stock_scc_display(make_params(), parser)
  assert ret.valid == valid
  if valid:
    assert ret.sourceMonoTime == START_NS


@pytest.mark.parametrize("size", (8, 16, 24, 48, 64))
def test_even_checksum_accepted_wrong_length_cannot_drive_display(size):
  parser = make_parser()
  address, data, bus = make_frame()
  payload = bytearray(data[:size].ljust(size, b"\x00"))
  checksum = parser.dbc.name_to_msg[SCC_CONTROL_NAME].sigs["CHECKSUM"]
  payload[:2] = checksum.calc_checksum(address, checksum, payload).to_bytes(2, "little")
  parser.update([START_NS, [(address, bytes(payload), bus)]])
  assert not get_stock_scc_display(make_params(), parser).valid


@pytest.mark.parametrize("value", (float("nan"), float("inf"), float("-inf"), -11.0, 11.0))
def test_nonfinite_or_out_of_dbc_range_cache_is_unavailable(value):
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  parser.vl[SCC_CONTROL_NAME]["aReqValue"] = value
  assert not get_stock_scc_display(make_params(), parser).valid


def test_signal_timestamps_cannot_be_mixed_between_frames():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  parser.ts_nanos[SCC_CONTROL_NAME]["aReqRaw"] -= 1
  assert not get_stock_scc_display(make_params(), parser).valid


def test_display_never_changes_parser_validity_state_or_values():
  parser = make_parser()
  parser.update([START_NS, [make_frame()]])
  before = deepcopy({
    "addresses": parser.addresses, "values": dict(parser.vl), "timestamps": parser.ts_nanos,
    "data": parser.dat, "states": parser.message_states, "invalid_count": parser.can_invalid_cnt,
  })
  for _ in range(10):
    assert get_stock_scc_display(make_params(), parser).valid
  assert before == {
    "addresses": parser.addresses, "values": dict(parser.vl), "timestamps": parser.ts_nanos,
    "data": parser.dat, "states": parser.message_states, "invalid_count": parser.can_invalid_cnt,
  }


@pytest.mark.parametrize("camera_scc", (False, True))
def test_actual_carstate_update_uses_existing_selected_stock_scc_parser(camera_scc):
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, gen_empty_fingerprint(), [], False, False, False)
  CP.openpilotLongitudinalControl = False
  if camera_scc:
    CP.flags = int(CP.flags | HyundaiFlags.CANFD_CAMERA_SCC)
  else:
    CP.flags = int(CP.flags & ~HyundaiFlags.CANFD_CAMERA_SCC)
  carstate = CarState(CP)
  parsers = carstate.get_can_parsers(CP)
  # Register using the existing stock-cruise field access, not the new helper.
  for key in (Bus.pt, Bus.cam):
    _ = parsers[key].vl[SCC_CONTROL_NAME]
    accel = -1.25 if key == Bus.pt else -2.5
    parsers[key].update([START_NS, [make_frame(bus=parsers[key].bus, aReqValue=accel)]])
  ret = carstate.update_canfd(parsers)
  assert ret.dkStockScc.valid and ret.dkStockScc.active
  assert ret.dkStockScc.accelRequest == pytest.approx(-2.5 if camera_scc else -1.25)
  assert ret.cruiseState.enabled


@pytest.mark.parametrize(("candidate", "op_long"), (
  (CAR.KIA_CARNIVAL_4TH_GEN, True), (CAR.KIA_SORENTO_4TH_GEN, False),
))
def test_actual_carstate_does_not_call_display_helper_outside_ka4_stock_scope(monkeypatch, candidate, op_long):
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
  CP.openpilotLongitudinalControl = op_long
  carstate = CarState(CP)
  parsers = carstate.get_can_parsers(CP)
  unexpected_calls = []

  def unexpected_call(*_args):
    unexpected_calls.append(_args)
    raise AssertionError("display telemetry must not be evaluated on another vehicle or OP longitudinal")

  monkeypatch.setattr(carstate_module, "get_stock_scc_display", unexpected_call)
  ret = carstate.update_canfd(parsers)
  assert not ret.dkStockScc.valid and not ret.dkStockScc.active
  assert not unexpected_calls


def test_actual_carstate_all_existing_fields_unchanged_by_telemetry(monkeypatch):
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, gen_empty_fingerprint(), [], False, False, False)
  CP.openpilotLongitudinalControl = False
  enabled_state = CarState(CP)
  disabled_state = CarState(CP)
  parsers = enabled_state.get_can_parsers(CP)
  selected = parsers[Bus.cam] if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else parsers[Bus.pt]
  _ = selected.vl[SCC_CONTROL_NAME]
  selected.update([START_NS, [make_frame(bus=selected.bus)]])
  with_telemetry = enabled_state.update_canfd(parsers).to_dict()
  monkeypatch.setattr(carstate_module, "get_stock_scc_display", lambda *_args: structs.CarState.DkStockScc())
  without_telemetry = disabled_state.update_canfd(parsers).to_dict()
  assert with_telemetry.pop("dkStockScc")["valid"]
  without_telemetry.pop("dkStockScc")
  assert with_telemetry == without_telemetry


def test_optional_display_exception_leaves_vehicle_state_unchanged_and_display_invalid(monkeypatch):
  Params().put("FingerPrints", str(dict(gen_empty_fingerprint())))
  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, gen_empty_fingerprint(), [], False, False, False)
  CP.openpilotLongitudinalControl = False
  normal_state = CarState(CP)
  failure_state = CarState(CP)
  parsers = normal_state.get_can_parsers(CP)
  selected = parsers[Bus.cam] if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else parsers[Bus.pt]
  _ = selected.vl[SCC_CONTROL_NAME]
  selected.update([START_NS, [make_frame(bus=selected.bus)]])
  normal = normal_state.update_canfd(parsers).to_dict()
  failed_calls = []

  def failing_display(*args):
    failed_calls.append(args)
    raise RuntimeError("unexpected optional display failure")

  monkeypatch.setattr(carstate_module, "get_stock_scc_display", failing_display)
  failed = failure_state.update_canfd(parsers)
  assert len(failed_calls) == 1
  assert not failed.dkStockScc.valid and not failed.dkStockScc.active
  assert failed.dkStockScc.sourceMonoTime == 0
  failed_dict = failed.to_dict()
  normal.pop("dkStockScc")
  failed_dict.pop("dkStockScc", None)
  assert normal == failed_dict
