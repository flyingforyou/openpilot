"""Leaving the stopping hold must not step the command.

Reproduces the 09-09 evening stop-and-go release: the hold sits at StoppingAccel and the state
flips to pid in one frame, at which point the PID -- reset on every stopping frame -- returns its
feedforward. Without a ramp the command jumped 0.56 m/s^2 inside one 20 ms tick.
"""
import numpy as np
from opendbc.car.structs import car

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import (
  STOP_RELEASE_RATE,
  LongControl,
  LongCtrlState,
)

ACCEL_LIMITS = (-3.5, 2.0)
STOPPING_ACCEL = -0.50


class FakeCS:
  def __init__(self, v_ego=0.0, a_ego=0.0, brake=False, standstill=False):
    self.vEgo = v_ego
    self.aEgo = a_ego
    self.brakePressed = brake
    self.cruiseState = type("cs", (), {"standstill": standstill})()


def make_lc():
  CP = car.CarParams.new_message()
  CP.stopAccel = STOPPING_ACCEL
  # 0.11.2 dropped the kp table from the schema; kiBP is the single gain point this port has.
  CP.longitudinalTuning.kiBP = [0.]
  CP.longitudinalTuning.kiV = [0.]
  lc = LongControl(CP)
  lc.pid._k_p = ([0], [1.0])
  return lc


def hold_at_stop(lc, frames=100):
  """Drive it into the stopping hold and let the ramp settle on StoppingAccel."""
  out = 0.0
  for _ in range(frames):
    out = lc.update(True, FakeCS(v_ego=0.0), 0.0, True, ACCEL_LIMITS,
                    stopping_accel=STOPPING_ACCEL)
  assert lc.long_control_state == LongCtrlState.stopping
  return out


def release(lc, frames, a_target=0.4):
  outs = []
  for _ in range(frames):
    outs.append(lc.update(True, FakeCS(v_ego=0.05), a_target, False, ACCEL_LIMITS,
                          stopping_accel=STOPPING_ACCEL))
  return outs


def test_hold_settles_just_past_stopping_accel():
  """The ramp steps by 1.0 m/s^2/s until it is no longer above the target, so it lands one step
  below it -- -0.51 for a -0.50 setting, which is exactly what the logs show."""
  held = hold_at_stop(make_lc())
  assert STOPPING_ACCEL - 1.0 * DT_CTRL - 1e-9 <= held <= STOPPING_ACCEL


def test_release_has_no_step():
  lc = make_lc()
  held = hold_at_stop(lc)
  outs = release(lc, 60)
  steps = np.abs(np.diff([held] + outs))
  assert steps.max() <= STOP_RELEASE_RATE * DT_CTRL + 1e-9, steps.max()


def test_release_step_would_have_been_large_without_the_ramp():
  """Guards the premise: the PID really does want a big jump on the first frame.

  Entered straight into pid with the hold still on the output, so was_stopping is False and the
  ramp never arms -- which is what the code did before this change.
  """
  lc = make_lc()
  held = hold_at_stop(lc)
  bare = make_lc()
  bare.long_control_state = LongCtrlState.pid
  bare.last_output_accel = held
  first = bare.update(True, FakeCS(v_ego=0.05), 0.4, False, ACCEL_LIMITS,
                      stopping_accel=STOPPING_ACCEL)
  assert not bare._releasing
  assert first - held > 0.2, (held, first)


def test_ramp_reaches_the_target_and_then_lets_go():
  lc = make_lc()
  hold_at_stop(lc)
  outs = release(lc, 200, a_target=0.4)
  assert outs[-1] > 0.3
  assert not lc._releasing            # ramp released control once the PID caught up


def test_ramp_does_not_delay_braking():
  """A stop that resumes into braking must not be held back by the release limiter."""
  lc = make_lc()
  hold_at_stop(lc)
  out = lc.update(True, FakeCS(v_ego=0.05), -2.0, False, ACCEL_LIMITS,
                  stopping_accel=STOPPING_ACCEL)
  assert out <= STOPPING_ACCEL + 1e-9, out


def test_release_flag_clears_when_disengaged():
  lc = make_lc()
  hold_at_stop(lc)
  release(lc, 3)
  assert lc._releasing
  lc.update(False, FakeCS(v_ego=0.05), 0.4, False, ACCEL_LIMITS, stopping_accel=STOPPING_ACCEL)
  assert lc.long_control_state == LongCtrlState.off
  assert not lc._releasing
