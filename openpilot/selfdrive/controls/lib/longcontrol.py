import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# How fast the command may climb away from the stopping hold, m/s^2 per second.
#
# Coming out of a stop the state goes stopping -> pid in one frame, and the output goes with it:
# the hold sits at StoppingAccel (-0.50 here) while the PID, reset on every stopping frame, starts
# again from its feedforward. On the 09-09 evening route that was a 0.56 m/s^2 step inside one
# 20 ms tick, on a car that holds the stop with regen -- the first half of the lurch the driver
# reported in stop-and-go traffic. Ramping instead spreads it over ~0.3 s. Only the release is
# rate-limited; braking harder is never delayed by this.
STOP_RELEASE_RATE = 2.0


# 0.11.2 retired startingState, vEgoStarting, startAccel and stoppingDecelRate into car.capnp's
# deprecated group, and no port fills them any more. That makes LongCtrlState.starting
# unreachable -- it was only ever entered behind CP.startingState -- so the branches that led
# into it are gone. What stays is CarrotPilot's own contribution: the gate on when the stopping
# ramp is allowed to take the stop away from the PID.
def long_control_state_trans(CP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill,
                             a_ego=0.0, stopping_accel=0.0, lead_d_rel=None):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid if starting_condition else LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        if stopping_accel < 0.0:
          # CarrotPilot: hand over to the stopping ramp only while braking is still gentler than
          # this. The ramp walks output down at a fixed rate toward stopAccel; taking it over
          # from the PID mid-way through a firm stop replaces braking that was already tracking
          # the plan with a slower one, and the shortfall has to be made up later. Below the
          # threshold the PID keeps the stop. A lead inside 4m overrides it either way.
          fcw_stop = lead_d_rel is not None and lead_d_rel < 4.0
          if a_ego > stopping_accel or fcw_stop:
            long_control_state = LongCtrlState.stopping
        else:
          long_control_state = LongCtrlState.stopping
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    # 0.11.2 deprecated the longitudinal kp table -- kpBP/kpV moved into the schema's deprecated
    # group and no port fills them any more, so the proportional term starts at zero the way
    # upstream leaves it. CarrotPilot still sets one below when its own tuning is enabled.
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self._releasing = False
    # 1.0 keeps a_target going through untouched, which is what this tree did before.
    self._k_f = 1.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits,
             stopping_accel: float = 0.0, lead_d_rel: float | None = None,
             gains: tuple[float, float, float] | None = None):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # CarrotPilot exposes the longitudinal PID. Only meaningful where the port gives a single
    # gain point, as this car does (kiV is [0.], so the loop is feedforward-only until something
    # sets it); ports with speed-dependent tables keep their own. kp is speed-independent here
    # because the table it used to ride on is gone from the schema.
    if gains is not None and len(self.CP.longitudinalTuning.kiBP) == 1:
      kp, ki, self._k_f = gains
      self.pid._k_p = ([0], [kp])
      self.pid._k_i = (self.CP.longitudinalTuning.kiBP, [ki])

    was_stopping = self.long_control_state == LongCtrlState.stopping
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill,
                                                       CS.aEgo, stopping_accel, lead_d_rel)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self._releasing = False
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      # A user-set stopping accel replaces the port's, which is a per-car constant.
      stop_accel = stopping_accel if stopping_accel < 0.0 else self.CP.stopAccel
      if output_accel > stop_accel:
        output_accel = min(output_accel, 0.0)
        # stoppingDecelRate is retired on 0.11.2, so this is upstream's fixed 1.0 m/s^2/s.
        output_accel -= 1.0 * DT_CTRL
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target * self._k_f)

      # Walk out of the stopping hold rather than stepping off it. The ramp ends the moment the
      # PID asks for no more than the ramp allows, so it costs nothing once under way.
      if was_stopping:
        self._releasing = True
      if self._releasing:
        release_limit = self.last_output_accel + STOP_RELEASE_RATE * DT_CTRL
        if output_accel <= release_limit:
          self._releasing = False
        else:
          output_accel = release_limit

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
