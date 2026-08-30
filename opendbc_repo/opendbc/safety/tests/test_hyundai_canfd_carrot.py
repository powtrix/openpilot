from opendbc.can import CANPacker
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


def test_hda1_alt_button_tx_temporarily_replaces_forwarded_stock_frame():
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_canfd_generated")

  # KA4's stock 0x1AA originates on E-CAN (bus 0) and normally crosses to
  # the camera bus. A synthetic button is transmitted directly on camera bus
  # 2, so Panda suppresses the competing stock frame for one 50 Hz period plus
  # its 20 ms timing allowance.
  start_time_us = 1_000_000
  stock = make_alt_button_packet(packer, 0, 125, Buttons.NONE)
  safety.set_timer(start_time_us)
  assert safety.safety_fwd_hook(stock) == 2

  injected = make_alt_button_packet(packer, 2, 126, Buttons.RES_ACCEL)
  assert safety.safety_tx_hook(injected)
  assert safety.safety_fwd_hook(stock) == -1

  safety.set_timer(start_time_us + 39_999)
  assert safety.safety_fwd_hook(stock) == -1
  safety.set_timer(start_time_us + 40_000)
  assert safety.safety_fwd_hook(stock) == 2
