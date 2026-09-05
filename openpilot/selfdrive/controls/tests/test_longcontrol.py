from types import SimpleNamespace

from openpilot.cereal import car
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longcontrol import long_control_state_trans as production_state_transition


def long_control_state_trans(CP, active, current_state, v_ego, should_stop,
                             brake_pressed, cruise_standstill, *, a_ego=0.0,
                             stopping_accel=-0.5, lead_status=False, lead_distance=100.0):
  radar_state = SimpleNamespace(
    leadOne=SimpleNamespace(status=lead_status, dRel=lead_distance),
  )
  return production_state_transition(
    CP, active, current_state, v_ego, should_stop, brake_pressed,
    cruise_standstill, a_ego, stopping_accel, radar_state,
  )




class TestLongControlStateTransition:

  def test_stay_stopped(self):
    CP = car.CarParams.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

def test_engage():
  CP = car.CarParams.new_message()
  active = True
  current_state = LongCtrlState.off
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid

def test_starting():
  CP = car.CarParams.new_message(startingState=True, vEgoStarting=0.5)
  active = True
  current_state = LongCtrlState.starting
  next_state = long_control_state_trans(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.starting
  next_state = long_control_state_trans(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid


def test_pid_stop_waits_for_acceleration_threshold_unless_lead_is_close():
  CP = car.CarParams.new_message()
  current_state = LongCtrlState.pid

  next_state = long_control_state_trans(
    CP, True, current_state, v_ego=1.0, should_stop=True,
    brake_pressed=False, cruise_standstill=False,
    a_ego=-1.0, stopping_accel=-0.5,
  )
  assert next_state == LongCtrlState.pid

  next_state = long_control_state_trans(
    CP, True, current_state, v_ego=1.0, should_stop=True,
    brake_pressed=False, cruise_standstill=False,
    a_ego=-1.0, stopping_accel=-0.5, lead_status=True, lead_distance=3.9,
  )
  assert next_state == LongCtrlState.stopping
