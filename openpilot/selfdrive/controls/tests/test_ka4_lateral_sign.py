from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, apply_driver_steer_torque_limits, gen_empty_fingerprint
from opendbc.car.hyundai import hyundaicanfd
import opendbc.car.hyundai.interface as hyundai_interface
from opendbc.car.hyundai.values import CAR, DBC, CarControllerParams, HyundaiFlags
import opendbc.car.interfaces as car_interfaces
from opendbc.car.vehicle_model import VehicleModel

from openpilot.cereal import car, log
import openpilot.selfdrive.controls.lib.latcontrol_torque as latcontrol_torque_module
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque


RIGHT_CURVATURE = 0.001
TEST_SPEED_MS = 20.0
KA4_ALT_BUTTON_ADDRESS = 0x1AA
KA4_SCC_CONTROL_ADDRESS = 0x1A0


class ZeroParams:
  """Keep this sign test independent of device-specific lateral toggles."""

  def get_bool(self, _key):
    return False

  def get_int(self, _key):
    return 0

  def get_float(self, _key):
    return 0.0


def test_ka4_right_curvature_reaches_negative_can_torque(monkeypatch):
  monkeypatch.setattr(hyundai_interface, "Params", ZeroParams)
  monkeypatch.setattr(car_interfaces, "Params", ZeroParams)
  monkeypatch.setattr(hyundaicanfd, "Params", ZeroParams)
  monkeypatch.setattr(latcontrol_torque_module, "Params", ZeroParams)

  # Match the relevant CAN-FD shape of the public 2023 KA4 route while using
  # the production interface to construct its torque-control CarParams.
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][KA4_ALT_BUTTON_ADDRESS] = 16
  fingerprint[0][KA4_SCC_CONTROL_ADDRESS] = 32
  CP = hyundai_interface.CarInterface.get_params(
    CAR.KIA_CARNIVAL_4TH_GEN,
    fingerprint,
    [],
    False,
    False,
    False,
  )
  assert CP.steerControlType == car.CarParams.SteerControlType.torque
  assert CP.lateralTuning.which() == "torque"
  assert CP.flags & HyundaiFlags.CANFD
  assert not CP.flags & HyundaiFlags.ANGLE_CONTROL

  # Full CarInterface construction starts CAN parsers and CarState. The torque
  # controller only needs the production lateral-acceleration mapping and NN
  # feature flags, so retain the real interface class without device I/O.
  CI = hyundai_interface.CarInterface.__new__(hyundai_interface.CarInterface)
  CI.use_nnff = False
  CI.use_nnff_lite = False
  vehicle_model = VehicleModel(CP)
  lateral_control = LatControlTorque(CP.as_reader(), CI)

  car_state = car.CarState.new_message()
  car_state.vEgo = TEST_SPEED_MS
  car_state.steeringAngleDeg = 0.0
  car_state.steeringRateDeg = 0.0
  car_state.steeringPressed = False
  live_params = log.LiveParametersData.new_message()
  car_control = car.CarControl.new_message()

  actuator_torque, desired_angle_deg, torque_log = lateral_control.update(
    True,
    car_state,
    vehicle_model,
    live_params,
    False,
    RIGHT_CURVATURE,
    car_control,
    False,
  )

  assert torque_log.desiredLateralAccel > 0.0
  assert actuator_torque < 0.0
  assert desired_angle_deg < 0.0

  controller_params = CarControllerParams(CP)
  requested_torque = int(round(actuator_torque * controller_params.STEER_MAX))
  applied_torque = apply_driver_steer_torque_limits(
    requested_torque,
    0,
    0,
    controller_params,
  )
  assert requested_torque < 0
  assert applied_torque < 0

  dbc_name = DBC[CP.carFingerprint][Bus.pt]
  packer = CANPacker(dbc_name)
  can_bus = hyundaicanfd.CanBus(CP, fingerprint)
  messages = hyundaicanfd.create_steering_messages(
    packer,
    CP,
    can_bus,
    True,
    True,
    applied_torque,
    desired_angle_deg,
    0,
    False,
  )
  parser = CANParser(dbc_name, [("LFA", 100)], can_bus.ECAN)
  parser.update([0, [message for message in messages if message[2] == can_bus.ECAN]])

  assert parser.can_valid
  assert parser.vl["LFA"]["STEER_REQ"] == 1
  assert parser.vl["LFA"]["TORQUE_REQUEST"] == applied_torque
  assert parser.vl["LFA"]["TORQUE_REQUEST"] < 0

  # This proves software command-sign propagation through the KA4 DBC only;
  # EPS response and physical rightward displacement still require route A/B.
