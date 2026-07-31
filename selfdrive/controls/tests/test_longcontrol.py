from cereal import car
from openpilot.selfdrive.controls.lib.longcontrol import CONTROL_N_T_IDX, LongCtrlState, get_velocity_pid_target, long_control_state_trans




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


class TestVelocityPidTarget:
  def test_uses_actuator_delay(self):
    speeds = [float(t) for t in CONTROL_N_T_IDX]
    action_t = 0.35
    assert abs(get_velocity_pid_target(speeds, action_t, fallback=9.0) - action_t) < 1e-6

  def test_incomplete_plan_uses_current_speed(self):
    assert get_velocity_pid_target([], action_t=0.35, fallback=4.2) == 4.2
    assert get_velocity_pid_target([1.0, 2.0], action_t=0.35, fallback=4.2) == 4.2
    assert get_velocity_pid_target([], action_t=float('nan'), fallback=4.2) == 4.2

  def test_e2e_source_uses_selected_acceleration(self):
    assert get_velocity_pid_target([], action_t=0.2, fallback=5.0, a_target=-2.0, e2e_source=True) == 4.6
    assert get_velocity_pid_target([], action_t=1.0, fallback=0.2, a_target=-2.0, e2e_source=True) == 0.0
