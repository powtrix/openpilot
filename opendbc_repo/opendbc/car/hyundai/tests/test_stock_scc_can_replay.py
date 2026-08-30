from dataclasses import dataclass

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from opendbc.car import DT_CTRL, gen_empty_fingerprint
import opendbc.car.hyundai.interface as hyundai_interface
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_stock_scc_resume import build_control, build_controller, build_state
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags, HyundaiSafetyFlags
import opendbc.car.interfaces as car_interfaces
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


DBC_NAME = "hyundai_canfd_generated"
SCC_CONTROL_FREQUENCY = 50
CRUISE_BUTTONS_FREQUENCY = 50
SCC_CONTROL_ADDRESS = 0x1A0
CRUISE_BUTTONS_ADDRESS = 0x1CF
CRUISE_BUTTONS_ALT_ADDRESS = 0x1AA

# Stock stopped 0x1AA fields captured from the public KA4 reference route
# de59124955b921d8|2023-06-24--00-12-50/0. CHECKSUM, COUNTER, and the button
# itself are supplied per frame below.
KA4_ALT_BUTTON_STOCK_VALUES = {
  "NEW_SIGNAL_1": 0,
  "SET_ME_1": 0,
  "DISTANCE_UNIT": 1,
  "NEW_SIGNAL_2": 0,
  "ADAPTIVE_CRUISE_MAIN_BTN": 0,
  "NEW_SIGNAL_3": 0,
  "LFA_BTN": 0,
  "NEW_SIGNAL_4": 0,
  "NORMAL_CRUISE_MAIN_BTN": 0,
  "NEW_SIGNAL_5": 2,
  "SET_ME_2": 3,
  "NEW_SIGNAL_6": 0,
  "BYTE6": 0,
  "BYTE7": 0,
  "CLU_SPEED": 0,
  "BYTE9": 0,
  "BYTE10": 0,
  "BYTE11": 0,
  "BYTE12": 0,
  "BYTE13": 0,
  "BYTE14": 0,
  "BYTE15": 0,
}
KA4_ALT_BUTTON_PUBLIC_SAMPLE = bytes.fromhex("63b2a040003800000000000000000000")


@dataclass(frozen=True)
class InjectedButton:
  frame: int
  address: int
  bus: int
  data: bytes
  oem_counter: int


def decode_message(dbc: DBC, message_name: str, data: bytes) -> dict[str, int]:
  message = dbc.name_to_msg[message_name]
  return {name: get_raw_value(data, signal) for name, signal in message.sigs.items()}


class Ka4StockSccReplay:
  """50 Hz stock-CAN inputs driving the real 100 Hz button output path."""

  def __init__(self, *, alt_buttons: bool = False):
    self.alt_buttons = alt_buttons
    self.button_message_name = "CRUISE_BUTTONS_ALT" if alt_buttons else "CRUISE_BUTTONS"
    self.dbc = DBC(DBC_NAME)
    self.scc_packer = CANPacker(DBC_NAME)
    self.oem_button_packer = CANPacker(DBC_NAME)
    self.controller = build_controller()
    if alt_buttons:
      self.controller.CP.flags |= HyundaiFlags.CANFD_ALT_BUTTONS
    self.controller.packer = CANPacker(DBC_NAME)
    self.CC = build_control()
    self.CS = build_state()
    self.scc_parser = CANParser(DBC_NAME, [("SCC_CONTROL", SCC_CONTROL_FREQUENCY)], self.controller.CAN.ECAN)
    self.warning_deadline = round(3.0 / DT_CTRL)
    self.warning_frames: list[int] = []
    self.scc_packets: list[tuple[int, bytes, int]] = []
    self.injected: list[InjectedButton] = []

  def _update_stock_inputs(self, frame: int) -> None:
    if frame % round(1.0 / (SCC_CONTROL_FREQUENCY * DT_CTRL)) != 0:
      return

    stock_counter = (frame // 2) & 0xFF
    info_display = 4 if frame >= self.warning_deadline else 0
    if info_display == 4:
      self.warning_frames.append(frame)

    scc_packet = self.scc_packer.make_can_msg("SCC_CONTROL", self.controller.CAN.ECAN, {
      "COUNTER": stock_counter,
      "ACCMode": 1,
      "ACC_ObjDist": 5.0,
      "ACC_ObjRelSpd": 0.0,
      "HUD_LEAD_INFO": 2,
      "InfoDisplay": info_display,
      "SysFailState": 0,
      "TakeOverReq": 0,
    })
    self.scc_packets.append(scc_packet)
    self.scc_parser.update([round(frame * DT_CTRL * 1e9), [scc_packet]])
    assert self.scc_parser.can_valid
    self.CS.scc_control = dict(self.scc_parser.vl["SCC_CONTROL"])

    # CarState reads this value from the stock 0x1CF/0x1AA received at 50 Hz.
    # Build and decode that packet rather than assigning an invented
    # controller-only counter sequence.
    button_period_frames = round(1.0 / (CRUISE_BUTTONS_FREQUENCY * DT_CTRL))
    oem_counter = (frame // button_period_frames) & (0xFF if self.alt_buttons else 0xF)
    oem_values = dict(KA4_ALT_BUTTON_STOCK_VALUES) if self.alt_buttons else {"SET_ME_1": 1}
    oem_values.update({
      "COUNTER": oem_counter,
      "CRUISE_BUTTONS": Buttons.NONE,
    })
    oem_button = self.oem_button_packer.make_can_msg(self.button_message_name, self.controller.CAN.ECAN, oem_values)
    decoded_button = decode_message(self.dbc, self.button_message_name, oem_button[1])
    self.CS.buttons_counter = decoded_button["COUNTER"]
    self.CS.cruise_buttons_msg = decoded_button if self.alt_buttons else None

  def step(self, frame: int) -> list[tuple[int, bytes, int]]:
    self._update_stock_inputs(frame)
    self.controller.frame = frame
    self.controller._update_ka4_stock_scc_keepalive(self.CC, self.CS)
    messages = self.controller.create_button_messages(self.CC, self.CS, use_clu11=False)
    for address, data, bus in messages:
      decoded = decode_message(self.dbc, self.button_message_name, data)
      if decoded["CRUISE_BUTTONS"] == Buttons.RES_ACCEL:
        self.injected.append(InjectedButton(frame, address, bus, data, self.CS.buttons_counter))
        # Model the contract assumed by the production design: an accepted RES
        # frame restarts the SCC's three-second driver-action timer. The public
        # KA4 reference routes contain no synthetic 0x1AA RES at an engaged
        # stop, so hardware acceptance is outside this deterministic replay.
        self.warning_deadline = frame + round(3.0 / DT_CTRL)
    return messages


def pulse_groups(frames: list[int]) -> list[list[int]]:
  groups: list[list[int]] = []
  for frame in frames:
    if not groups or frame != groups[-1][-1] + 1:
      groups.append([])
    groups[-1].append(frame)
  return groups


def test_ka4_public_reference_uses_crc_protected_alt_button_payload():
  dbc = DBC(DBC_NAME)
  message = dbc.name_to_msg["CRUISE_BUTTONS_ALT"]
  values = decode_message(dbc, "CRUISE_BUTTONS_ALT", KA4_ALT_BUTTON_PUBLIC_SAMPLE)

  assert message.address == CRUISE_BUTTONS_ALT_ADDRESS
  assert len(KA4_ALT_BUTTON_PUBLIC_SAMPLE) == 16
  assert values["COUNTER"] == 160
  assert values["CRUISE_BUTTONS"] == Buttons.NONE
  for name, value in KA4_ALT_BUTTON_STOCK_VALUES.items():
    assert values[name] == value

  checksum = message.sigs["CHECKSUM"]
  assert values["CHECKSUM"] == checksum.calc_checksum(message.address, checksum, bytearray(KA4_ALT_BUTTON_PUBLIC_SAMPLE))


def test_public_ka4_route_shape_selects_stock_long_alt_buttons_and_safety(monkeypatch):
  class ZeroParams:
    def get_bool(self, _key):
      return False

    def get_int(self, _key):
      return 0

    def put_nonblocking(self, _key, _value):
      raise AssertionError("zero-valued Params must not write during get_params")

  # Isolate this capability check from device toggles. Both modules import
  # Params directly, so patch the references used by Hyundai and the base
  # interface rather than mutating a real Params database.
  monkeypatch.setattr(hyundai_interface, "Params", ZeroParams)
  monkeypatch.setattr(car_interfaces, "Params", ZeroParams)

  # Address/DLC shape observed on ECAN in the same public KA4 route as the
  # payload fixture above: 0x1AA is present and 0x1CF is absent. This tests the
  # complete fingerprint -> interface flags -> Panda safety-param path.
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][CRUISE_BUTTONS_ALT_ADDRESS] = len(KA4_ALT_BUTTON_PUBLIC_SAMPLE)
  fingerprint[0][SCC_CONTROL_ADDRESS] = 32
  assert CRUISE_BUTTONS_ALT_ADDRESS in fingerprint[0]
  assert CRUISE_BUTTONS_ADDRESS not in fingerprint[0]

  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, fingerprint, [], False, False, False)

  assert CP.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN
  assert CP.flags & HyundaiFlags.CANFD
  assert CP.flags & HyundaiFlags.RADAR_SCC
  assert CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS
  assert not CP.flags & HyundaiFlags.CAMERA_SCC
  assert CP.pcmCruise
  assert not CP.openpilotLongitudinalControl
  assert len(CP.safetyConfigs) == 1
  assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.hyundaiCanfd
  assert CP.safetyConfigs[0].safetyParam == int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["standard-0x1cf", "ka4-alt-0x1aa"])
def test_ka4_stock_scc_real_can_replay_emits_schedule_under_assumed_timer_contract(alt_buttons):
  replay = Ka4StockSccReplay(alt_buttons=alt_buttons)

  safety = libsafety_py.libsafety
  safety_param = HyundaiSafetyFlags.CANFD_ALT_BUTTONS if alt_buttons else 0
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(safety_param)) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)

  for frame in range(3051):
    messages = replay.step(frame)
    for address, data, bus in messages:
      assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))

  frames = [message.frame for message in replay.injected]
  groups = pulse_groups(frames)
  assert groups == [
    [250, 251, 252],
    [502, 503, 504],
    [754, 755, 756],
    [1006, 1007, 1008],
    [1258, 1259, 1260],
    [1510, 1511, 1512],
    [1762, 1763, 1764],
    [2014, 2015, 2016],
    [2266, 2267, 2268],
    [2518, 2519, 2520],
    [2698, 2699, 2700],
  ]

  # Every generated frame uses its real DBC layout and physical bus, follows
  # the latest 50 Hz stock counter, and contains no stale controls from the
  # rest of the steering-wheel switch message.
  for message in replay.injected:
    values = decode_message(replay.dbc, replay.button_message_name, message.data)
    if alt_buttons:
      assert message.address == CRUISE_BUTTONS_ALT_ADDRESS
      assert message.bus == replay.controller.CAN.CAM
      assert len(message.data) == 16
      assert values["COUNTER"] == (message.oem_counter + 1) & 0xFF
      assert values["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
      for name, value in KA4_ALT_BUTTON_STOCK_VALUES.items():
        assert values[name] == value
    else:
      assert message.address == CRUISE_BUTTONS_ADDRESS
      assert message.bus == replay.controller.CAN.ECAN
      assert len(message.data) == 8
      assert values == {
        "_CHECKSUM": 0,
        "COUNTER": (message.oem_counter + 1) & 0xF,
        "CRUISE_BUTTONS": Buttons.RES_ACCEL,
        "ADAPTIVE_CRUISE_MAIN_BTN": 0,
        "NORMAL_CRUISE_MAIN_BTN": 0,
        "LFA_BTN": 0,
        "RIGHT_PADDLE": 0,
        "LEFT_PADDLE": 0,
        "SET_ME_1": 1,
        "SET_ME_1_": 0,
      }

  # The controller runs at 100 Hz while the stock switch counter advances at
  # 50 Hz. Therefore each three-frame injection is intentionally encoded as
  # source+1, source+1, next-source+1. This proves what reaches Panda; it must
  # not be mistaken for proof that an SCC ECU accepts the duplicate counter.
  counter_mask = 0xFF if alt_buttons else 0xF
  counters = [decode_message(replay.dbc, replay.button_message_name, message.data)["COUNTER"]
              for message in replay.injected]
  for index in range(0, len(counters), 3):
    first, second, third = counters[index:index + 3]
    assert first == second
    assert third == (second + 1) & counter_mask

  # The controller creates three-frame presses and releases by ceasing its
  # injection. No injected RES is present in any quiet interval, after the
  # final frame at 27.00 s, or when the OEM warning appears at 30.00 s.
  emitted = set(frames)
  assert all(len(group) == 3 for group in groups)
  assert all(not any(frame in emitted for frame in range(group[-1] + 1, next_group[0]))
             for group, next_group in zip(groups[:-1], groups[1:], strict=True))
  assert max(emitted) == 2700
  assert not any(frame > 2700 for frame in emitted)
  assert replay.warning_frames[0] == 3000

  # SCC_CONTROL uses and passes the Hyundai CAN-FD CRC. The KA4 standard
  # 0x1CF button definition instead names its byte `_CHECKSUM`, which means
  # neither CANPacker nor CANParser treats it as a verified checksum. Record
  # that protocol limitation explicitly rather than claiming CRC coverage.
  scc_message = replay.dbc.name_to_msg["SCC_CONTROL"]
  scc_checksum = scc_message.sigs["CHECKSUM"]
  assert scc_checksum.calc_checksum is not None
  for address, data, bus in replay.scc_packets:
    assert address == SCC_CONTROL_ADDRESS
    assert bus == replay.controller.CAN.ECAN
    assert get_raw_value(data, scc_checksum) == scc_checksum.calc_checksum(address, scc_checksum, bytearray(data))

  button_message = replay.dbc.name_to_msg[replay.button_message_name]
  if alt_buttons:
    button_checksum = button_message.sigs["CHECKSUM"]
    assert button_checksum.calc_checksum is not None
    for message in replay.injected:
      assert get_raw_value(message.data, button_checksum) == \
             button_checksum.calc_checksum(message.address, button_checksum, bytearray(message.data))
  else:
    button_checksum = button_message.sigs["_CHECKSUM"]
    assert button_checksum.calc_checksum is None


INTERLOCKS = (
  "brake",
  "gas",
  "auto_hold",
  "parking_brake",
  "driver_button",
  "cruise_disabled",
  "control_cancel",
  "movement",
  "scc_failure",
  "takeover",
  "lead_state",
  "lead_distance",
  "lead_departure",
  "info_display",
)


def activate_interlock(replay: Ka4StockSccReplay, interlock: str) -> None:
  if interlock == "brake":
    replay.CS.out.brakePressed = True
  elif interlock == "gas":
    replay.CS.out.gasPressed = True
  elif interlock == "auto_hold":
    replay.CS.out.brakeHoldActive = True
  elif interlock == "parking_brake":
    replay.CS.out.parkingBrake = True
  elif interlock == "driver_button":
    replay.CS.cruise_buttons[-1] = Buttons.SET_DECEL
  elif interlock == "cruise_disabled":
    replay.CC.enabled = False
    replay.CS.out.cruiseState.enabled = False
  elif interlock == "control_cancel":
    replay.CC.cruiseControl.cancel = True
  elif interlock == "movement":
    replay.CS.out.standstill = False
    replay.CS.out.vEgo = 0.2
  elif interlock == "scc_failure":
    replay.CS.scc_control["SysFailState"] = 1
  elif interlock == "takeover":
    replay.CS.scc_control["TakeOverReq"] = 1
  elif interlock == "lead_state":
    replay.CS.scc_control["HUD_LEAD_INFO"] = 1
  elif interlock == "lead_distance":
    replay.CS.scc_control["ACC_ObjDist"] = 25.0
  elif interlock == "lead_departure":
    replay.CS.scc_control["ACC_ObjRelSpd"] = 0.6
  elif interlock == "info_display":
    replay.CS.scc_control["InfoDisplay"] = 5
  else:
    raise AssertionError(f"unhandled interlock: {interlock}")


@pytest.mark.parametrize("interlock", INTERLOCKS)
def test_ka4_real_can_replay_aborts_an_active_press_on_every_interlock(interlock):
  replay = Ka4StockSccReplay(alt_buttons=True)
  for frame in range(251):
    replay.step(frame)

  assert [message.frame for message in replay.injected] == [250]
  activate_interlock(replay, interlock)
  messages = replay.step(251)

  if interlock == "control_cancel":
    assert len(messages) == 1
    assert decode_message(replay.dbc, replay.button_message_name, messages[0][1])["CRUISE_BUTTONS"] == Buttons.CANCEL
  else:
    assert messages == []
  assert [message.frame for message in replay.injected] == [250]
  assert not replay.controller.stock_scc_keepalive_pending
  assert replay.controller.stock_scc_keepalive_press_frames == 0


SUPPORT_GATE_MUTATIONS = (
  "fingerprint",
  "pcm_cruise",
  "openpilot_longitudinal",
  "canfd",
  "radar_scc",
  "camera_scc",
)


def invalidate_support_gate(replay: Ka4StockSccReplay, gate: str) -> None:
  if gate == "fingerprint":
    replay.controller.CP.carFingerprint = CAR.KIA_EV6
  elif gate == "pcm_cruise":
    replay.controller.CP.pcmCruise = False
  elif gate == "openpilot_longitudinal":
    replay.controller.CP.openpilotLongitudinalControl = True
  elif gate == "canfd":
    replay.controller.CP.flags &= ~HyundaiFlags.CANFD
  elif gate == "radar_scc":
    replay.controller.CP.flags &= ~HyundaiFlags.RADAR_SCC
  elif gate == "camera_scc":
    replay.controller.CP.flags |= HyundaiFlags.CAMERA_SCC
  else:
    raise AssertionError(f"unhandled support gate: {gate}")


@pytest.mark.parametrize("gate", SUPPORT_GATE_MUTATIONS)
def test_ka4_real_can_replay_aborts_if_vehicle_support_gate_changes_mid_press(gate):
  replay = Ka4StockSccReplay(alt_buttons=True)
  for frame in range(251):
    replay.step(frame)

  assert [message.frame for message in replay.injected] == [250]
  invalidate_support_gate(replay, gate)
  messages = replay.step(251)

  assert messages == []
  assert not replay.controller.stock_scc_keepalive_pending
  assert replay.controller.stock_scc_stop_start_frame is None


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["standard-0x1cf", "ka4-alt-0x1aa"])
def test_ka4_software_cancel_preempts_keepalive_on_the_next_frame(alt_buttons):
  replay = Ka4StockSccReplay(alt_buttons=alt_buttons)
  for frame in range(30):
    replay.step(frame)

  # Jump to the first stock warning, matching the warning-level recovery path
  # used by enter_standstill_warning in the focused state-machine tests.
  first_messages = replay.step(300)
  assert len(first_messages) == 1
  first_values = decode_message(replay.dbc, replay.button_message_name, first_messages[0][1])
  assert first_values["CRUISE_BUTTONS"] == Buttons.RES_ACCEL

  replay.CC.cruiseControl.cancel = True
  cancel_messages = replay.step(301)

  # CANCEL is safety-critical and must not inherit the 250 ms post-keepalive
  # quiet period. Preserve each existing encoding policy (20 standard copies,
  # one ALT copy), with no remaining RES frame.
  assert len(cancel_messages) == (1 if alt_buttons else 20)
  assert all(decode_message(replay.dbc, replay.button_message_name, message[1])["CRUISE_BUTTONS"] == Buttons.CANCEL
             for message in cancel_messages)


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["standard-0x1cf", "ka4-alt-0x1aa"])
def test_ka4_movement_reset_cannot_delay_software_cancel(alt_buttons):
  replay = Ka4StockSccReplay(alt_buttons=alt_buttons)
  for frame in range(30):
    replay.step(frame)

  first_messages = replay.step(300)
  assert len(first_messages) == 1
  assert decode_message(replay.dbc, replay.button_message_name, first_messages[0][1])["CRUISE_BUTTONS"] == Buttons.RES_ACCEL

  replay.CS.out.standstill = False
  replay.CS.out.vEgo = 0.2
  replay.CC.cruiseControl.cancel = True
  cancel_messages = replay.step(301)

  assert len(cancel_messages) == (1 if alt_buttons else 20)
  assert all(decode_message(replay.dbc, replay.button_message_name, message[1])["CRUISE_BUTTONS"] == Buttons.CANCEL
             for message in cancel_messages)
