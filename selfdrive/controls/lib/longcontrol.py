import numpy as np
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


def get_velocity_pid_target(speeds, action_t: float, fallback: float,
                            a_target: float = 0.0, e2e_source: bool = False) -> float:
  """Return a velocity target consistent with the acceleration command's time/source.

  MPC speeds are sampled at the actuator horizon. In experimental mode the selected aTarget may
  come from e2e while the published speed trajectory still comes from MPC, so integrate that
  selected acceleration over the short actuator horizon instead of mixing two plan sources.
  """
  if not np.isfinite(fallback):
    return 0.0
  if not np.isfinite(action_t):
    return float(fallback)
  if e2e_source:
    return max(float(fallback + a_target * max(action_t, 0.0)), 0.0) if np.isfinite(a_target) else float(fallback)
  if len(speeds) != len(CONTROL_N_T_IDX):
    return float(fallback)
  v_target = float(np.interp(action_t, CONTROL_N_T_IDX, speeds))
  return v_target if np.isfinite(v_target) else float(fallback)


# Velocity-tracking PID (ported from CarrotPilot). Tesla's accel PID gains are 0, i.e. the accel
# command is pure feedforward with no closed-loop correction, so it overshoots stops. Tracking the
# planned velocity with a closed loop (kp on velocity error, plus the plan's accel as feedforward)
# instead lands the stop where the plan wants it.
VELOCITY_PID_KP = 1.0

# Precise-stop (CarrotPilot) tuning, only used when velocity_pid is on. While stopping, stay in the
# velocity PID (which decelerates accurately) until either nearly stopped or close behind a lead,
# then commit to the brake-and-hold ramp. fcw_stop is the "don't coast into a close lead" guard.
STOPPING_ACCEL_TH = -0.5   # m/s^2; above this (gently braking) switch to the stopping ramp
FCW_STOP_DIST = 4.0        # m; commit to the stopping ramp this close behind a lead

# The velocity target comes from longitudinalPlan.speeds, which the planner anchors on its own
# filtered state (v_desired_filter, RC=2.0s) rather than on vEgo. Under hard braking that anchor
# sits above the real speed, so the tracking error turns positive and the correction *releases*
# the brake. Measured on route 0000001b (9.1 engaged min): with aTarget <= -2.0 the delivered
# command came out 1.11 m/s^2 short of the plan -- 33% of the braking demand -- while at
# aTarget > -1.0 the error was 0.01-0.03, i.e. the loop behaves exactly where the precise-stop
# benefit lives. So fade out only the brake-releasing half of the correction as the plan leans on
# the brakes; extra braking still passes through untouched.
BRAKE_RELEASE_FADE_BP = [-2.0, -1.0]   # aTarget: no release allowed at/below -2.0, full above -1.0

LongCtrlState = car.CarControl.Actuators.LongControlState


def limit_brake_release(output_accel: float, a_target: float) -> float:
  """Stop velocity tracking from undoing the plan's braking (see BRAKE_RELEASE_FADE_BP)."""
  if not np.isfinite(output_accel) or not np.isfinite(a_target):
    return output_accel
  release = output_accel - a_target
  if release <= 0.0:
    return output_accel      # the loop is asking for more braking, never limit that
  allowed = float(np.interp(a_target, BRAKE_RELEASE_FADE_BP, [0.0, 1.0]))
  return a_target + release * allowed


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill,
                             a_ego=0.0, radar_state=None, precise_stop=False):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        if precise_stop:
          # Keep velocity-tracking (accurate decel) until nearly stopped, but commit to the ramp
          # early when close behind a lead so the car doesn't coast into it (CarrotPilot fcw_stop).
          fcw_stop = radar_state is not None and radar_state.leadOne.status and radar_state.leadOne.dRel < FCW_STOP_DIST
          if a_ego > STOPPING_ACCEL_TH or fcw_stop:
            long_control_state = LongCtrlState.stopping
        else:
          long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    # Velocity-tracking PID for a precise stop (Tesla only, opt-in). Otherwise the stock accel PID.
    self.velocity_pid = CP.brand == "tesla" and Params().get_bool("TeslaVelocityPid")
    if self.velocity_pid:
      self.pid = PIDController(([0.], [VELOCITY_PID_KP]), ([0.], [0.]), rate=1 / DT_CTRL)
    else:
      self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                               (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                               rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, v_target=None, radar_state=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill,
                                                       a_ego=CS.aEgo, radar_state=radar_state,
                                                       precise_stop=self.velocity_pid)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      # Velocity PID closes the loop on the planned speed so the car lands the stop instead of
      # coasting past it on feedforward alone; feedforward is still the plan's accel.
      target_v = CS.vEgo if v_target is None or not np.isfinite(v_target) else v_target
      error = (target_v - CS.vEgo) if self.velocity_pid else (a_target - CS.aEgo)
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)
      if self.velocity_pid:
        output_accel = limit_brake_release(output_accel, a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
