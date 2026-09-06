from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from opendbc.car import Bus, DT_CTRL, gen_empty_fingerprint
import opendbc.car.hyundai.carcontroller as hyundai_carcontroller
import opendbc.car.hyundai.interface as hyundai_interface
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_stock_scc_resume import build_control, build_controller, build_state
from opendbc.car.hyundai.values import Buttons, CAR, HyundaiFlags, HyundaiSafetyFlags
import opendbc.car.interfaces as car_interfaces
from opendbc.car.structs import CarControl, CarParams, CarState
from opendbc.safety.tests.libsafety import libsafety_py


DBC_NAME = "hyundai_canfd_generated"
SCC_CONTROL_FREQUENCY = 50
CRUISE_BUTTONS_FREQUENCY = 50
SCC_CONTROL_ADDRESS = 0x1A0
ADRV_0X161_ADDRESS = 0x161
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


def build_full_hda1_ka4_controller(monkeypatch):
  class TestParams:
    @staticmethod
    def get(key):
      return "dkcarrot-wip" if key == "GitBranch" else ""

    @staticmethod
    def get_int(key):
      return {
        "MaxAngleFrames": 89,
        "CruiseButtonTest1": 8,
        "CruiseButtonTest2": 30,
        "CruiseButtonTest3": 1,
        # Simulate the stale value written by the superseded automatic build.
        "Ka4StockSccStandstillRearm": 1,
      }.get(key, 0)

    @staticmethod
    def get_bool(_key):
      return False

    @staticmethod
    def get_float(_key):
      return 0.0

    @staticmethod
    def put_int_nonblocking(_key, _value):
      pass

  monkeypatch.setattr(hyundai_carcontroller, "Params", TestParams)
  monkeypatch.setattr(hyundai_carcontroller.hyundaicanfd, "Params", TestParams)

  CP = CarParams.new_message()
  CP.carFingerprint = CAR.KIA_CARNIVAL_4TH_GEN
  CP.flags = int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS)
  CP.pcmCruise = True
  CP.openpilotLongitudinalControl = False
  CP.wheelbase = 3.09
  CP.steerRatio = 14.23
  CP.init("safetyConfigs", 1)
  CP.safetyConfigs[0].safetyModel = CarParams.SafetyModel.hyundaiCanfd
  CP.safetyConfigs[0].safetyParam = int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)
  controller = hyundai_carcontroller.CarController({Bus.pt: DBC_NAME}, CP.as_reader())

  dbc = DBC(DBC_NAME)
  packer = CANPacker(DBC_NAME)
  raw_adrv = packer.make_can_msg("ADRV_0x161", controller.CAN.CAM, {
    "COUNTER": 17,
    "LFA_ICON": 0,
    "LKA_ICON": 0,
    "ALERTS_1": 1,
    "ALERTS_2": 21,
    "ALERTS_3": 26,
    "ALERTS_5": 5,
    "MUTE": 1,
    "DAW_ICON": 2,
    "SOUNDS_1": 1,
    "SOUNDS_2": 2,
    "SOUNDS_3": 3,
    "SOUNDS_4": 4,
  })
  adrv_values = decode_message(dbc, "ADRV_0x161", raw_adrv[1])

  raw_button = packer.make_can_msg("CRUISE_BUTTONS_ALT", controller.CAN.ECAN, {
    "COUNTER": 4,
    "CRUISE_BUTTONS": Buttons.NONE,
    "DISTANCE_UNIT": 1,
    "NEW_SIGNAL_5": 2,
    "SET_ME_2": 3,
  })
  button_values = decode_message(dbc, "CRUISE_BUTTONS_ALT", raw_button[1])

  car_state = CarState.new_message()
  car_state.standstill = True
  car_state.canValid = True
  car_state.latEnabled = True
  car_state.cruiseState.enabled = True
  car_state.cruiseState.speed = 80 / 3.6
  CS = SimpleNamespace(
    out=car_state.as_reader(),
    is_metric=True,
    modelV2=None,
    scc_control={
      "InfoDisplay": 0,
      "ACCMode": 1,
      "ACC_ObjDist": 5.0,
      "ACC_ObjRelSpd": 0.0,
      "HUD_LEAD_INFO": 2,
      "SysFailState": 0,
      "TakeOverReq": 0,
    },
    adrv_0x161=adrv_values,
    lfahda_cluster=None,
    buttons_counter=button_values["COUNTER"],
    cruise_buttons_msg=button_values,
    cruise_buttons=deque([Buttons.NONE]),
    main_buttons=deque([Buttons.NONE]),
  )

  car_control = CarControl.new_message()
  car_control.enabled = True
  car_control.latActive = True
  car_control.hudControl.setSpeed = 80 / 3.6
  car_control.hudControl.leadVisible = True
  car_control.hudControl.leadRadar = 1
  car_control.hudControl.leadDistance = 5.0
  CC = car_control.as_reader()
  return controller, CC, CS, car_state, dbc, raw_adrv


def get_adrv_0x161(messages, dbc):
  matches = [message for message in messages if message[0] == ADRV_0X161_ADDRESS]
  assert len(matches) == 1
  assert matches[0][2] == 0
  return matches[0], decode_message(dbc, "ADRV_0x161", matches[0][1])


class Ka4StockSccReplay:
  """50 Hz stock-CAN inputs driving the real 100 Hz button output path."""

  def __init__(self, *, alt_buttons: bool = False, hda2: bool = False,
               panda_bus_offset: int = 0, button_phase_frames: int = 0):
    self.alt_buttons = alt_buttons
    self.hda2 = hda2
    self.panda_bus_offset = panda_bus_offset
    self.button_phase_frames = button_phase_frames
    self.button_message_name = "CRUISE_BUTTONS_ALT" if alt_buttons else "CRUISE_BUTTONS"
    self.dbc = DBC(DBC_NAME)
    self.scc_packer = CANPacker(DBC_NAME)
    self.oem_button_packer = CANPacker(DBC_NAME)
    self.controller = build_controller(alt_buttons=alt_buttons)
    self.controller.CAN = SimpleNamespace(ECAN=panda_bus_offset + (1 if hda2 else 0), CAM=panda_bus_offset + 2)
    if hda2:
      self.controller.CP.flags |= HyundaiFlags.CANFD_HDA2
    self.controller.packer = CANPacker(DBC_NAME)
    self.CC = build_control()
    self.CS = build_state()
    self.scc_parser = CANParser(DBC_NAME, [("SCC_CONTROL", SCC_CONTROL_FREQUENCY)], self.controller.CAN.ECAN)
    self.modeled_state_deadline = round(3.0 / DT_CTRL)
    self.info_display_override: int | None = None
    self.acc_mode = 1
    self.modeled_state_frames: list[int] = []
    self.scc_packets: list[tuple[int, bytes, int]] = []
    self.oem_button_packets: list[tuple[int, bytes, int]] = []
    # ``injected`` records host requests. A request is not proof that Panda
    # accepted it, so keep the safety result in a separate list.
    self.injected: list[InjectedButton] = []
    self.safety_accepted: list[InjectedButton] = []
    self.last_safety_tx_results: list[bool | None] = []

  def _update_stock_inputs(self, frame: int) -> None:
    scc_period_frames = round(1.0 / (SCC_CONTROL_FREQUENCY * DT_CTRL))
    if frame % scc_period_frames == 0:
      stock_counter = (frame // scc_period_frames) & 0xFF
      info_display = (
        self.info_display_override
        if self.info_display_override is not None
        else 4 if frame >= self.modeled_state_deadline else 0
      )
      if info_display == 4:
        self.modeled_state_frames.append(frame)

      scc_packet = self.scc_packer.make_can_msg("SCC_CONTROL", self.controller.CAN.ECAN, {
        "COUNTER": stock_counter,
        "ACCMode": self.acc_mode,
        "MainMode_ACC": 1,
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
      # Mirror the production CarState stock-SCC engagement contract. The
      # parser above supplies the real DBC value; ACCMode 1/2 are the only
      # enabled states accepted by CarState and Panda stock-long safety.
      self.CS.out.cruiseState.enabled = self.acc_mode in (1, 2)

    # CarState reads this value from the stock 0x1CF/0x1AA received at 50 Hz.
    # Build and decode that packet rather than assigning an invented
    # controller-only counter sequence. Exercise both possible phases relative
    # to the 100 Hz controller; SCC_CONTROL remains on its independent phase.
    button_period_frames = round(1.0 / (CRUISE_BUTTONS_FREQUENCY * DT_CTRL))
    if frame < self.button_phase_frames or (frame - self.button_phase_frames) % button_period_frames != 0:
      return

    oem_counter = ((frame - self.button_phase_frames) // button_period_frames) & (0xFF if self.alt_buttons else 0xF)
    oem_values = dict(KA4_ALT_BUTTON_STOCK_VALUES) if self.alt_buttons else {"SET_ME_1": 1}
    oem_values.update({
      "COUNTER": oem_counter,
      "CRUISE_BUTTONS": Buttons.NONE,
    })
    oem_button = self.oem_button_packer.make_can_msg(self.button_message_name, self.controller.CAN.ECAN, oem_values)
    self.oem_button_packets.append(oem_button)
    decoded_button = decode_message(self.dbc, self.button_message_name, oem_button[1])
    self.CS.buttons_counter = decoded_button["COUNTER"]
    self.CS.cruise_buttons_msg = decoded_button if self.alt_buttons else None

  def step(self, frame: int, *, safety=None) -> list[tuple[int, bytes, int]]:
    previous_scc_count = len(self.scc_packets)
    previous_button_count = len(self.oem_button_packets)
    self._update_stock_inputs(frame)
    if safety is not None:
      safety.set_timer(round(frame * DT_CTRL * 1e6))
      for address, data, bus in (
        self.scc_packets[previous_scc_count:] + self.oem_button_packets[previous_button_count:]
      ):
        local_bus = bus - self.panda_bus_offset
        assert 0 <= local_bus <= 2
        assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, local_bus, data))
    self.controller.frame = frame
    self.controller._update_ka4_stock_scc_keepalive(self.CC, self.CS)
    messages = self.controller.create_button_messages(self.CC, self.CS, use_clu11=False)
    self.last_safety_tx_results = []
    for address, data, bus in messages:
      decoded = decode_message(self.dbc, self.button_message_name, data)
      accepted = None
      if safety is not None:
        local_bus = bus - self.panda_bus_offset
        assert 0 <= local_bus <= 2
        accepted = bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(address, local_bus, data)))
      self.last_safety_tx_results.append(accepted)
      if decoded["CRUISE_BUTTONS"] == Buttons.RES_ACCEL:
        request = InjectedButton(frame, address, bus, data, self.CS.buttons_counter)
        self.injected.append(request)
        if accepted:
          self.safety_accepted.append(request)
          # Advance the synthetic InfoDisplay schedule only after Panda
          # accepts the frame. This still does not prove arbitration or SCC
          # ECU acceptance, but a rejected host request must not advance even
          # the replay's synthetic model.
          self.modeled_state_deadline = frame + round(3.0 / DT_CTRL)
    return messages


def pulse_groups(frames: list[int]) -> list[list[int]]:
  groups: list[list[int]] = []
  for frame in frames:
    if not groups or frame != groups[-1][-1] + round(1.0 / (CRUISE_BUTTONS_FREQUENCY * DT_CTRL)):
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


def test_ka4_hda1_full_controller_update_arms_bounded_rearm_and_preserves_oem_alert(monkeypatch):
  controller, CC, CS, car_state, dbc, raw_adrv = build_full_hda1_ka4_controller(monkeypatch)
  assert controller.ka4_stock_scc_standstill_rearm
  assert controller.CAN.ECAN == 0
  assert controller.CAN.CAM == 2

  _, messages = controller.update(CC, CS, 0)
  replacement_message, replacement = get_adrv_0x161(messages, dbc)
  assert replacement["ALERTS_5"] == 5
  assert replacement["COUNTER"] == 18
  checksum = dbc.name_to_msg["ADRV_0x161"].sigs["CHECKSUM"]
  assert replacement["CHECKSUM"] == checksum.calc_checksum(
    replacement_message[0], checksum, bytearray(replacement_message[1]),
  )
  for signal in (
    "ALERTS_1", "ALERTS_2", "ALERTS_3", "MUTE", "DAW_ICON",
    "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4",
  ):
    assert replacement[signal] == CS.adrv_0x161[signal]

  # The accepted ECAN replacement suppresses the raw camera-side ADRV frame
  # inside Panda's 20 Hz forwarding timeout. The experiment must preserve the
  # stock driver instruction because a transmitted RES does not prove that the
  # SCC ECU accepted it or reset its timer.
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(
    CarParams.SafetyModel.hyundaiCanfd, int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS),
  ) == 0
  safety.init_tests()
  safety.set_timer(1_000_000)
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(
    replacement_message[0], replacement_message[2], replacement_message[1],
  ))
  safety.set_timer(1_069_999)
  assert safety.safety_fwd_hook(libsafety_py.make_CANPacket(raw_adrv[0], raw_adrv[2], raw_adrv[1])) == -1
  safety.set_timer(1_070_000)
  assert safety.safety_fwd_hook(libsafety_py.make_CANPacket(raw_adrv[0], raw_adrv[2], raw_adrv[1])) == 0

  # With no fresh OEM button counter after the first frame, the request must
  # fail closed instead of reusing an alive counter. A driver-brake stop also
  # cannot arm or transmit the periodic RES path.
  for _ in range(310):
    controller.update(CC, CS, 0)
  assert controller.stock_scc_stop_start_frame is not None
  assert controller.stock_scc_keepalive_request_count == 0

  car_state.brakePressed = True
  CS.out = car_state.as_reader()
  controller.frame = 315
  _, messages = controller.update(CC, CS, 0)
  _, braking = get_adrv_0x161(messages, dbc)
  assert braking["ALERTS_5"] == 5
  assert not controller.stock_scc_keepalive_pending


@pytest.mark.parametrize("ecan_bus", [0, 4], ids=["single-panda", "second-panda-offset"])
def test_public_ka4_route_shape_selects_stock_long_alt_buttons_and_safety(monkeypatch, ecan_bus):
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
  fingerprint[ecan_bus][CRUISE_BUTTONS_ALT_ADDRESS] = len(KA4_ALT_BUTTON_PUBLIC_SAMPLE)
  fingerprint[ecan_bus][SCC_CONTROL_ADDRESS] = 32
  assert CRUISE_BUTTONS_ALT_ADDRESS in fingerprint[ecan_bus]
  assert CRUISE_BUTTONS_ADDRESS not in fingerprint[ecan_bus]

  CP = CarInterface.get_params(CAR.KIA_CARNIVAL_4TH_GEN, fingerprint, [], False, False, False)

  assert CP.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN
  assert CP.flags & HyundaiFlags.CANFD
  assert CP.flags & HyundaiFlags.RADAR_SCC
  assert CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS
  assert not CP.flags & HyundaiFlags.CAMERA_SCC
  assert CP.pcmCruise
  assert not CP.openpilotLongitudinalControl
  assert len(CP.safetyConfigs) == (1 if ecan_bus == 0 else 2)
  if ecan_bus == 4:
    assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput
  assert CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd
  assert CP.safetyConfigs[-1].safetyParam == int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)


@pytest.mark.parametrize(
  "alt_buttons,panda_bus_offset,button_phase_frames",
  [
    (True, 0, 0), (True, 4, 0),
    (True, 0, 1), (True, 4, 1),
  ],
  ids=[
    "ka4-alt-0x1aa-phase-0", "ka4-alt-second-panda-phase-0",
    "ka4-alt-0x1aa-phase-1", "ka4-alt-second-panda-phase-1",
  ],
)
def test_ka4_stock_scc_real_can_replay_emits_schedule_under_synthetic_state_model(
    alt_buttons, panda_bus_offset, button_phase_frames,
):
  replay = Ka4StockSccReplay(
    alt_buttons=alt_buttons, hda2=False,
    panda_bus_offset=panda_bus_offset, button_phase_frames=button_phase_frames,
  )

  safety = libsafety_py.libsafety
  safety_param = 0
  if alt_buttons:
    safety_param |= HyundaiSafetyFlags.CANFD_ALT_BUTTONS
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(safety_param)) == 0
  safety.init_tests()
  assert not safety.get_controls_allowed()

  for frame in range(3051):
    messages = replay.step(frame, safety=safety)
    assert len(replay.last_safety_tx_results) == len(messages)
    assert all(replay.last_safety_tx_results)

  frames = [message.frame for message in replay.injected]
  groups = pulse_groups(frames)
  expected_starts = (
    [250, 504, 758, 1012, 1266, 1520, 1774, 2028, 2282, 2536, 2696]
    if button_phase_frames == 0 else
    [251, 505, 759, 1013, 1267, 1521, 1775, 2029, 2283, 2537, 2695]
  )
  assert groups == [[start, start + 2, start + 4] for start in expected_starts]

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

  # Each three-frame injection replaces three fresh 50 Hz OEM switch frames.
  # The injected counters therefore advance once per 20 ms source transition;
  # this proves what reaches Panda, not that SCC accepts or acts on those RES
  # frames or that they reset any OEM standstill timer.
  counter_mask = 0xFF if alt_buttons else 0xF
  for group_index in range(0, len(replay.injected), 3):
    group = replay.injected[group_index:group_index + 3]
    assert [message.frame for message in group] == [group[0].frame, group[0].frame + 2, group[0].frame + 4]
    counters = [decode_message(replay.dbc, replay.button_message_name, message.data)["COUNTER"] for message in group]
    assert counters == [counters[0], (counters[0] + 1) & counter_mask, (counters[0] + 2) & counter_mask]

  # The controller creates three-frame presses and releases by ceasing its
  # injection. No injected RES is present in any quiet interval, after the
  # final frame at 27.00 s, or when modeled InfoDisplay=4 begins at 30.00 s.
  emitted = set(frames)
  assert all(len(group) == 3 for group in groups)
  assert all(not any(frame in emitted for frame in range(group[-1] + 1, next_group[0]))
             for group, next_group in zip(groups[:-1], groups[1:], strict=True))
  assert max(emitted) == (2700 if button_phase_frames == 0 else 2699)
  assert not any(frame > 2700 for frame in emitted)
  assert replay.modeled_state_frames[0] == 3000


@pytest.mark.parametrize("panda_bus_offset", [0, 4], ids=["single-panda", "second-panda-offset"])
@pytest.mark.parametrize("button_phase_frames", [0, 1], ids=["phase-0", "phase-1"])
def test_standard_0x1cf_ka4_variant_never_emits_owner_rearm_experiment(panda_bus_offset, button_phase_frames):
  replay = Ka4StockSccReplay(
    alt_buttons=False, hda2=False,
    panda_bus_offset=panda_bus_offset, button_phase_frames=button_phase_frames,
  )

  for frame in range(3051):
    assert replay.step(frame) == []

  assert replay.injected == []


@pytest.mark.parametrize("alt_buttons", [False, True])
def test_ka4_hda2_replay_never_emits_experimental_rearm(alt_buttons):
  replay = Ka4StockSccReplay(alt_buttons=alt_buttons, hda2=True)

  for frame in range(3051):
    replay.step(frame)

  assert replay.injected == []

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


@pytest.mark.parametrize("closing_acc_mode", [0, 4], ids=["off", "cancelled"])
def test_ka4_real_scc_rx_aborts_keepalive_and_panda_rejects_generic_reactivation(closing_acc_mode):
  replay = Ka4StockSccReplay(alt_buttons=True)
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(
    CarParams.SafetyModel.hyundaiCanfd, int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS),
  ) == 0
  safety.init_tests()

  # Qualify the physical stop with ACCMode=1. At the 0.30 s boundary, the
  # observed InfoDisplay=4 state starts a recovery burst and Panda permits its
  # first frame because the received SCC state is still engaged.
  for frame in range(30):
    assert replay.step(frame, safety=safety) == []
  replay.info_display_override = 4
  first = replay.step(30, safety=safety)
  assert len(first) == 1
  assert decode_message(replay.dbc, replay.button_message_name, first[0][1])["CRUISE_BUTTONS"] == Buttons.RES_ACCEL
  assert replay.last_safety_tx_results == [True]
  assert [request.frame for request in replay.safety_accepted] == [30]
  assert safety.get_controls_allowed()

  # The next 50 Hz SCC frame closes ACC before the second fresh button source.
  # Feed that real RX transition to Panda before running the controller. The
  # dedicated keepalive aborts. The KA4 physical-stop gate must also suppress
  # generic cruise reactivation instead of relying on Panda to reject a stale
  # host RES request.
  assert replay.step(31, safety=safety) == []
  replay.acc_mode = closing_acc_mode
  after_transition = replay.step(32, safety=safety)
  assert after_transition == []
  assert not replay.CS.out.cruiseState.enabled
  assert not replay.controller.stock_scc_keepalive_pending
  assert replay.controller.stock_scc_keepalive_press_frames == 0
  assert not replay.controller.stock_scc_keepalive_requested
  assert not safety.get_controls_allowed()
  assert replay.last_safety_tx_results == []
  assert [request.frame for request in replay.injected] == [30]
  assert [request.frame for request in replay.safety_accepted] == [30]
  assert replay.modeled_state_deadline == 30 + round(3.0 / DT_CTRL)


INTERLOCKS = (
  "brake",
  "gas",
  "auto_hold",
  "parking_brake",
  "driver_button",
  "cruise_disabled",
  "can_invalid",
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
  elif interlock == "can_invalid":
    replay.CS.out.canValid = False
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


def test_ka4_software_cancel_preempts_keepalive_on_the_next_frame():
  replay = Ka4StockSccReplay(alt_buttons=True)
  for frame in range(30):
    replay.step(frame)

  # Jump to the first modeled InfoDisplay=4 state, matching the state-triggered
  # path used by enter_standstill_resume_state in the focused tests.
  first_messages = replay.step(300)
  assert len(first_messages) == 1
  first_values = decode_message(replay.dbc, replay.button_message_name, first_messages[0][1])
  assert first_values["CRUISE_BUTTONS"] == Buttons.RES_ACCEL

  replay.CC.cruiseControl.cancel = True
  cancel_messages = replay.step(301)

  # CANCEL is safety-critical and must not inherit the 250 ms post-keepalive
  # quiet period. The owner's ALT layout emits one fresh-counter copy, with no
  # remaining RES frame.
  assert len(cancel_messages) == 1
  assert all(decode_message(replay.dbc, replay.button_message_name, message[1])["CRUISE_BUTTONS"] == Buttons.CANCEL
             for message in cancel_messages)


def test_ka4_movement_reset_cannot_delay_software_cancel():
  replay = Ka4StockSccReplay(alt_buttons=True)
  for frame in range(30):
    replay.step(frame)

  first_messages = replay.step(300)
  assert len(first_messages) == 1
  assert decode_message(replay.dbc, replay.button_message_name, first_messages[0][1])["CRUISE_BUTTONS"] == Buttons.RES_ACCEL

  replay.CS.out.standstill = False
  replay.CS.out.vEgo = 0.2
  replay.CC.cruiseControl.cancel = True
  cancel_messages = replay.step(301)

  assert len(cancel_messages) == 1
  assert all(decode_message(replay.dbc, replay.button_message_name, message[1])["CRUISE_BUTTONS"] == Buttons.CANCEL
             for message in cancel_messages)
