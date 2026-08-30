import pytest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car.hyundai.values import Buttons, HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


def make_alt_button_packet(packer, bus, counter, button):
  address, data, bus = packer.make_can_msg("CRUISE_BUTTONS_ALT", bus, {
    "COUNTER": counter,
    "CRUISE_BUTTONS": button,
    "SET_ME_1": 1,
  })
  return libsafety_py.make_CANPacket(address, bus, data)


def make_button_packet(packer, *, alt_buttons, bus, counter, button, extra_values=None):
  message = "CRUISE_BUTTONS_ALT" if alt_buttons else "CRUISE_BUTTONS"
  values = {
    "COUNTER": counter,
    "CRUISE_BUTTONS": button,
    "SET_ME_1": 1,
  }
  if extra_values is not None:
    values.update(extra_values)
  address, data, bus = packer.make_can_msg(message, bus, values)
  return libsafety_py.make_CANPacket(address, bus, data)


def make_rx_packet(packer, message, bus, values):
  address, data, bus = packer.make_can_msg(message, bus, values)
  return libsafety_py.make_CANPacket(address, bus, data)


def make_steering_packet(packer, message, bus, torque, steer_req):
  address, data, bus = packer.make_can_msg(message, bus, {
    "TORQUE_REQUEST": torque,
    "STEER_REQ": steer_req,
  })
  return libsafety_py.make_CANPacket(address, bus, data)


BUTTON_TX_TOPOLOGIES = (
  ("hda1-standard", 0, False, 0),
  ("hda1-alt", HyundaiSafetyFlags.CANFD_ALT_BUTTONS, True, 2),
  ("hda2-standard", HyundaiSafetyFlags.CANFD_LKA_STEERING, False, 1),
  ("hda2-alt", HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_ALT_BUTTONS, True, 1),
)

PHYSICAL_CONTROL_TOPOLOGIES = BUTTON_TX_TOPOLOGIES

CAMERA_SCC_BUTTON_PASSTHROUGH_TOPOLOGIES = (
  ("hda1-standard", HyundaiSafetyFlags.CAMERA_SCC, False, 2),
  ("hda1-alt", HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ALT_BUTTONS, True, 2),
  ("hda2-standard", HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LKA_STEERING, False, 2),
  (
    "hda2-alt",
    HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_ALT_BUTTONS,
    True,
    2,
  ),
)

AUXILIARY_CONTROL_BITS = {
  False: {
    "ADAPTIVE_CRUISE_MAIN_BTN": 19,
    "NORMAL_CRUISE_MAIN_BTN": 21,
    "LFA_BTN": 23,
    "RIGHT_PADDLE": 25,
    "LEFT_PADDLE": 27,
  },
  True: {
    "ADAPTIVE_CRUISE_MAIN_BTN": 34,
    "LFA_BTN": 39,
    "NORMAL_CRUISE_MAIN_BTN": 41,
  },
}


def test_canfd_auxiliary_control_bits_match_generated_dbc():
  dbc = DBC("hyundai_canfd_generated")
  for alt_buttons, signals in AUXILIARY_CONTROL_BITS.items():
    message = dbc.name_to_msg["CRUISE_BUTTONS_ALT" if alt_buttons else "CRUISE_BUTTONS"]
    for signal, expected_bit in signals.items():
      assert message.sigs[signal].start_bit == expected_bit


def test_canfd_button_tx_is_fail_closed_for_controls_cruise_and_button_value():
  safety = libsafety_py.libsafety
  packer = CANPacker("hyundai_canfd_generated")

  for _name, safety_param, alt_buttons, bus in BUTTON_TX_TOPOLOGIES:
    assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
    safety.init_tests()

    for controls_allowed in (False, True):
      safety.set_controls_allowed(controls_allowed)
      safety.set_cruise_engaged_prev(False)
      # SET is deliberately valid in Carrot because make_spam_button uses it
      # to lower stock SCC's set speed. It still requires controls_allowed.
      for button in (Buttons.RES_ACCEL, Buttons.SET_DECEL):
        packet = make_button_packet(
          packer, alt_buttons=alt_buttons, bus=bus, counter=1, button=button,
        )
        assert safety.safety_tx_hook(packet) == controls_allowed

    for cruise_engaged in (False, True):
      safety.set_controls_allowed(False)
      safety.set_cruise_engaged_prev(cruise_engaged)
      packet = make_button_packet(
        packer, alt_buttons=alt_buttons, bus=bus, counter=2, button=Buttons.CANCEL,
      )
      assert safety.safety_tx_hook(packet) == cruise_engaged

    safety.set_controls_allowed(True)
    safety.set_cruise_engaged_prev(True)
    for button in (Buttons.NONE, Buttons.GAP_DIST, Buttons.LFA_BUTTON, 6, 7):
      packet = make_button_packet(
        packer, alt_buttons=alt_buttons, bus=bus, counter=3, button=button,
      )
      assert not safety.safety_tx_hook(packet)


@pytest.mark.parametrize(
  "_name,safety_param,alt_buttons,bus",
  CAMERA_SCC_BUTTON_PASSTHROUGH_TOPOLOGIES,
)
def test_camera_scc_preserves_full_physical_button_passthrough(_name, safety_param, alt_buttons, bus):
  safety = libsafety_py.libsafety
  packer = CANPacker("hyundai_canfd_generated")
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(False)
  safety.set_cruise_engaged_prev(False)

  # Camera-SCC clones the complete physical switch frame to CAM every 20 ms,
  # so NONE, unknown/reserved values, and standalone auxiliary controls must
  # retain the existing broad passthrough behavior.
  for counter, button in enumerate(range(8)):
    packet = make_button_packet(
      packer, alt_buttons=alt_buttons, bus=bus, counter=counter, button=button,
    )
    assert safety.safety_tx_hook(packet)

  for signal in AUXILIARY_CONTROL_BITS[alt_buttons]:
    packet = make_button_packet(
      packer,
      alt_buttons=alt_buttons,
      bus=bus,
      counter=9,
      button=Buttons.NONE,
      extra_values={signal: 1},
    )
    assert safety.safety_tx_hook(packet)


@pytest.mark.parametrize("alt_steering", [False, True], ids=["standard-steer", "alt-steer"])
@pytest.mark.parametrize("alt_buttons", [False, True], ids=["standard-buttons", "alt-buttons"])
def test_stock_long_hda2_non_camera_tx_allowlist_matches_controller_paths(alt_steering, alt_buttons):
  safety = libsafety_py.libsafety
  safety_param = HyundaiSafetyFlags.CANFD_LKA_STEERING
  if alt_steering:
    safety_param |= HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT
  if alt_buttons:
    safety_param |= HyundaiSafetyFlags.CANFD_ALT_BUTTONS
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  button_bus = 1
  # create_steering_messages chooses the steering address from the flag;
  # create_suppress_lfa can clone either cached camera-lane encoding.
  expected_non_buttons = {
    (0x2A4, 0, 24), (0x362, 0, 32),
  }

  for addr, bus, length in expected_non_buttons:
    assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(length)))
  steer_message = "LKAS_ALT" if alt_steering else "LKAS"
  steering = make_steering_packet(packer, steer_message, 0, 0, 0)
  assert safety.safety_tx_hook(steering)
  allowed_button = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=button_bus, counter=1, button=Buttons.RES_ACCEL,
  )
  assert safety.safety_tx_hook(allowed_button)

  # Frames generated only after disabling stock longitudinal/ADAS must remain
  # unavailable in every stock-long HDA2 topology.
  for addr, bus, length in ((0x1A0, 1, 32), (0x160, 1, 16), (0x1EA, 1, 32)):
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(length)))

  # The other cruise encoding and a valid payload on the wrong topology bus
  # must both fail the generic allowlist before reaching the platform hook.
  other_alt = not alt_buttons
  wrong_encoding = make_button_packet(
    packer, alt_buttons=other_alt, bus=button_bus, counter=2, button=Buttons.RES_ACCEL,
  )
  wrong_bus = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=2,
    counter=3, button=Buttons.RES_ACCEL,
  )
  assert not safety.safety_tx_hook(wrong_encoding)
  assert not safety.safety_tx_hook(wrong_bus)


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["standard-buttons", "alt-buttons"])
def test_stock_long_hda1_radar_tx_allowlist_matches_controller_paths(alt_buttons):
  safety = libsafety_py.libsafety
  safety_param = HyundaiSafetyFlags.CANFD_ALT_BUTTONS if alt_buttons else 0
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  for addr, bus, length in ((0x1E0, 0, 16), (0x161, 0, 32)):
    assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(length)))
  assert safety.safety_tx_hook(make_steering_packet(packer, "LFA", 0, 0, 0))
  button_bus = 2 if alt_buttons else 0
  allowed_button = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=button_bus, counter=1, button=Buttons.RES_ACCEL,
  )
  assert safety.safety_tx_hook(allowed_button)

  # These frames are emitted only by openpilot-long or a camera-harness path.
  for addr, bus, length in (
    (0x1A0, 0, 32), (0x160, 0, 16), (0x200, 0, 8),
    (0x0EA, 2, 24), (0x2AF, 2, 8),
  ):
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(length)))

  other_alt = not alt_buttons
  wrong_encoding = make_button_packet(
    packer, alt_buttons=other_alt, bus=button_bus, counter=2, button=Buttons.RES_ACCEL,
  )
  wrong_bus = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=0 if alt_buttons else 2,
    counter=3, button=Buttons.RES_ACCEL,
  )
  assert not safety.safety_tx_hook(wrong_encoding)
  assert not safety.safety_tx_hook(wrong_bus)


TORQUE_STEERING_TOPOLOGIES = (
  ("hda1-standard-buttons", 0, "LFA"),
  ("hda1-alt-buttons", HyundaiSafetyFlags.CANFD_ALT_BUTTONS, "LFA"),
  ("hda2-standard", HyundaiSafetyFlags.CANFD_LKA_STEERING, "LKAS"),
  (
    "hda2-alt-buttons",
    HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_ALT_BUTTONS,
    "LKAS",
  ),
  (
    "hda2-alt-steering",
    HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT,
    "LKAS_ALT",
  ),
  (
    "hda2-alt-steering-buttons",
    HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT |
    HyundaiSafetyFlags.CANFD_ALT_BUTTONS,
    "LKAS_ALT",
  ),
)


@pytest.mark.parametrize(
  "_name,safety_param,message", TORQUE_STEERING_TOPOLOGIES,
  ids=[topology[0] for topology in TORQUE_STEERING_TOPOLOGIES],
)
@pytest.mark.parametrize("controls_allowed", [False, True], ids=["aol", "controls-allowed"])
def test_canfd_torque_steering_enforces_limits_with_and_without_controls(
    _name, safety_param, message, controls_allowed,
):
  safety = libsafety_py.libsafety
  packer = CANPacker("hyundai_canfd_generated")

  def reset_safety():
    assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
    safety.init_tests()
    safety.set_controls_allowed(controls_allowed)

  # Preserve Carrot's always-on-lateral contract for a legal low command.
  reset_safety()
  assert safety.safety_tx_hook(make_steering_packet(packer, message, 0, 2, 1))

  # Absolute and per-frame rate limits must be enforced in both engagement
  # states. The CAN-FD controller's default ramp-up contract is +/-2 per frame.
  reset_safety()
  assert not safety.safety_tx_hook(make_steering_packet(packer, message, 0, 513, 1))
  for sign in (-1, 1):
    reset_safety()
    assert safety.safety_tx_hook(make_steering_packet(packer, message, 0, sign * 2, 1))
    reset_safety()
    assert not safety.safety_tx_hook(make_steering_packet(packer, message, 0, sign * 3, 1))

  # Opposing driver torque must reduce the permitted command below MAX_STEER.
  reset_safety()
  safety.set_torque_driver(-251, -251)
  safety.set_desired_torque_last(512)
  safety.set_rt_torque_last(512)
  assert not safety.safety_tx_hook(make_steering_packet(packer, message, 0, 512, 1))

  # TorqueDriverLimited's down-rate is a required wind-down under driver
  # opposition, not a cap on how quickly torque may be released toward zero.
  # Exactly three counts of wind-down is accepted; two is insufficient.
  reset_safety()
  safety.set_torque_driver(-1_000, -1_000)
  safety.set_desired_torque_last(512)
  safety.set_rt_torque_last(512)
  assert safety.safety_tx_hook(make_steering_packet(packer, message, 0, 509, 1))
  reset_safety()
  safety.set_torque_driver(-1_000, -1_000)
  safety.set_desired_torque_last(512)
  safety.set_rt_torque_last(512)
  assert not safety.safety_tx_hook(make_steering_packet(packer, message, 0, 510, 1))

  # A non-zero torque cannot drop STEER_REQ before the tolerance window is earned.
  reset_safety()
  assert not safety.safety_tx_hook(make_steering_packet(packer, message, 0, 2, 0))


@pytest.mark.parametrize("button", [Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL])
@pytest.mark.parametrize(
  "controls_allowed,cruise_engaged",
  [(False, False), (False, True), (True, False), (True, True)],
)
@pytest.mark.parametrize(
  "_name,safety_param,alt_buttons,bus", PHYSICAL_CONTROL_TOPOLOGIES,
  ids=[topology[0] for topology in PHYSICAL_CONTROL_TOPOLOGIES],
)
def test_canfd_button_tx_rejects_every_combined_physical_control(
    _name, safety_param, alt_buttons, bus, controls_allowed, cruise_engaged, button,
):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(controls_allowed)
  safety.set_cruise_engaged_prev(cruise_engaged)
  packer = CANPacker("hyundai_canfd_generated")

  for signal in AUXILIARY_CONTROL_BITS[alt_buttons]:
    combined = make_button_packet(
      packer, alt_buttons=alt_buttons, bus=bus, counter=1, button=button,
      extra_values={signal: 1},
    )
    assert not safety.safety_tx_hook(combined)


@pytest.mark.parametrize("button", [Buttons.RES_ACCEL, Buttons.SET_DECEL])
@pytest.mark.parametrize(
  "pedal_message,pedal_signal,pedal_state_getter",
  [
    ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED", "get_gas_pressed_prev"),
    ("TCS", "DriverBraking", "get_brake_pressed_prev"),
  ],
  ids=["gas-held", "brake-held"],
)
def test_pedal_held_blocks_alt_res_set_after_scc_rx_reenables_controls(
    button, pedal_message, pedal_signal, pedal_state_getter,
):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(
    CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS,
  ) == 0
  safety.init_tests()
  packer = CANPacker("hyundai_canfd_generated")

  safety.set_controls_allowed(True)
  pedal = make_rx_packet(packer, pedal_message, 0, {
    "COUNTER": 1,
    pedal_signal: 1,
  })
  assert safety.safety_rx_hook(pedal)
  assert getattr(safety, pedal_state_getter)()
  assert not safety.get_controls_allowed()

  # This fork's stock-long cruise-state path currently re-enables controls on
  # any engaged ACCMode. The TX gate must still fail closed while the driver's
  # latched pedal state remains active.
  scc = make_rx_packet(packer, "SCC_CONTROL", 0, {
    "COUNTER": 1,
    "ACCMode": 1,
  })
  assert safety.safety_rx_hook(scc)
  assert safety.get_cruise_engaged_prev()
  assert safety.get_controls_allowed()

  injected = make_button_packet(
    packer, alt_buttons=True, bus=2, counter=2, button=button,
  )
  assert not safety.safety_tx_hook(injected)

  # CANCEL remains available while stock cruise is engaged, regardless of the
  # pedal interlock used for RES/SET.
  cancel = make_button_packet(
    packer, alt_buttons=True, bus=2, counter=3, button=Buttons.CANCEL,
  )
  assert safety.safety_tx_hook(cancel)


def test_camera_scc_physical_alt_button_uses_buffered_forward_queue():
  safety = libsafety_py.libsafety
  safety_param = HyundaiSafetyFlags.CANFD_ALT_BUTTONS | HyundaiSafetyFlags.CAMERA_SCC
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  safety.set_cruise_engaged_prev(True)
  safety.set_timer(1_000_000)
  packer = CANPacker("hyundai_canfd_generated")

  # Camera-SCC clones physical switch frames, including GAP. Two packets start
  # the existing jitter buffer; the next raw source frame supplies the alive
  # counter for the forwarded replacement.
  for counter in (10, 11):
    physical = make_button_packet(
      packer, alt_buttons=True, bus=2, counter=counter, button=Buttons.GAP_DIST,
    )
    assert safety.safety_tx_hook(physical)

  stock = make_button_packet(
    packer, alt_buttons=True, bus=0, counter=12, button=Buttons.NONE,
  )
  assert safety.safety_fwd_hook(stock) == 2
  assert (stock.data[4] >> 4) & 0x7 == Buttons.GAP_DIST
  assert stock.data[2] == 12


def test_topology_rejected_button_does_not_change_forward_replacement_state():
  safety = libsafety_py.libsafety
  # Stock-long HDA2 permits standard buttons on bus 1, not bus 2. Bus 2 is
  # nevertheless a tracked replacement TX bus for HDA1, so this exercises the
  # generic allowlist gate rather than relying on a missing per-address state.
  assert safety.set_safety_hooks(
    CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING,
  ) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  rejected = make_button_packet(
    packer, alt_buttons=False, bus=2, counter=9, button=Buttons.RES_ACCEL,
  )
  safety.set_timer(1_000_000)
  assert not safety.safety_tx_hook(rejected)

  # A rejected TX must not suppress an otherwise forwarded raw frame with the
  # same counter.
  raw = make_button_packet(
    packer, alt_buttons=False, bus=0, counter=9, button=Buttons.NONE,
  )
  assert safety.safety_fwd_hook(raw) == 2


def test_hda1_alt_button_tx_temporarily_replaces_forwarded_stock_frame():
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  # KA4's stock 0x1AA originates on E-CAN (bus 0) and normally crosses to
  # the camera bus. A synthetic button is transmitted directly on camera bus
  # 2, so Panda suppresses only the stock frame carrying the same alive counter.
  start_time_us = 1_000_000
  stock = make_alt_button_packet(packer, 0, 125, Buttons.NONE)
  safety.set_timer(start_time_us)
  assert safety.safety_fwd_hook(stock) == 2

  injected = make_alt_button_packet(packer, 2, 126, Buttons.RES_ACCEL)
  assert safety.safety_tx_hook(injected)
  duplicate = make_alt_button_packet(packer, 0, 126, Buttons.NONE)
  assert safety.safety_fwd_hook(duplicate) == -1

  safety.set_timer(start_time_us + 39_999)
  assert safety.safety_fwd_hook(duplicate) == -1
  # A new counter is the release frame and must not wait for the timeout.
  release = make_alt_button_packet(packer, 0, 127, Buttons.NONE)
  assert safety.safety_fwd_hook(release) == 2
  safety.set_timer(start_time_us + 40_000)
  assert safety.safety_fwd_hook(duplicate) == 2


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["0x1cf", "0x1aa"])
@pytest.mark.parametrize("host_latency_us", [5_000, 10_000], ids=["5ms", "10ms"])
@pytest.mark.parametrize("wrap_counter", [False, True], ids=["ordinary", "wrap"])
def test_hda1_three_button_burst_preserves_release_counter_continuity(
    alt_buttons, host_latency_us, wrap_counter,
):
  safety = libsafety_py.libsafety
  # Radar-SCC HDA1 standard buttons are sent on E-CAN directly. Exercise the
  # cross-bus 0x1CF replacement path on the existing camera-SCC topology;
  # KA4's 0x1AA radar-SCC path remains the primary target here.
  safety_param = (HyundaiSafetyFlags.CANFD_ALT_BUTTONS if alt_buttons else
                  HyundaiSafetyFlags.CAMERA_SCC)
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  modulus = 256 if alt_buttons else 16
  start_counter = modulus - 2 if wrap_counter else 6
  start_time_us = 1_000_000
  camera_counters = []
  release_forwarded = False

  for source_index in range(6):
    source_counter = (start_counter + source_index) % modulus
    raw = make_button_packet(
      packer, alt_buttons=alt_buttons, bus=0, counter=source_counter, button=Buttons.NONE,
    )
    safety.set_timer(start_time_us + source_index * 20_000)
    if safety.safety_fwd_hook(raw) == 2:
      camera_counters.append(source_counter)
      if source_index == 4:
        release_forwarded = True

    if source_index < 3:
      injected_counter = (source_counter + 1) % modulus
      injected = make_button_packet(
        packer, alt_buttons=alt_buttons, bus=2, counter=injected_counter,
        button=Buttons.RES_ACCEL,
      )
      safety.set_timer(start_time_us + source_index * 20_000 + host_latency_us)
      assert safety.safety_tx_hook(injected)
      camera_counters.append(injected_counter)

  assert release_forwarded
  assert camera_counters == [
    (start_counter + offset) % modulus for offset in range(6)
  ]


@pytest.mark.parametrize("alt_buttons", [False, True], ids=["0x1cf", "0x1aa"])
def test_hda1_button_forward_replacement_state_expires_and_resets(alt_buttons):
  safety = libsafety_py.libsafety
  safety_param = (HyundaiSafetyFlags.CANFD_ALT_BUTTONS if alt_buttons else
                  HyundaiSafetyFlags.CAMERA_SCC)
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  injected = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=2, counter=7, button=Buttons.RES_ACCEL,
  )
  same_counter = make_button_packet(
    packer, alt_buttons=alt_buttons, bus=0, counter=7, button=Buttons.NONE,
  )
  safety.set_timer(1_000_000)
  assert safety.safety_tx_hook(injected)
  assert safety.safety_fwd_hook(same_counter) == -1

  safety.set_timer(1_040_000)
  assert safety.safety_fwd_hook(same_counter) == 2

  # Reinitializing the safety mode must not retain an old replacement counter.
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, safety_param) == 0
  safety.init_tests()
  safety.set_timer(1_000_001)
  assert safety.safety_fwd_hook(same_counter) == 2
