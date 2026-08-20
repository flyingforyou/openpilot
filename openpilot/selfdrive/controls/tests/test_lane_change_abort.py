"""Once a lane change is committed, what can still call it off.

Nothing could, before: laneChangeStarting only ended when the model said the move was finished,
which on route 00000087 segment 13 took 5.6 s with no check of any kind in between. These pin the
narrow window in which the blindspot can now stop it, and -- as much to the point -- that the
window really is narrow, because reversing out of a change already half made is its own hazard.
"""
import pytest

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  LANE_CHANGE_ABORT_S,
  DesireHelper,
)

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

CRUISE = 60 * CV.MPH_TO_MS


class CS:
  def __init__(self, *, right_blinker=False, left_blinker=False, pressed=False, torque=0.0,
               right_bs=False, left_bs=False, v=CRUISE):
    self.vEgo = v
    self.rightBlinker = right_blinker
    self.leftBlinker = left_blinker
    self.steeringPressed = pressed
    self.steeringTorque = torque
    self.rightBlindspot = right_bs
    self.leftBlindspot = left_bs
    self.brakePressed = False


def commit_right(dh, **kw):
  """Blinker, then a nudge, until the change is committed. Returns frames spent."""
  dh.update(CS(right_blinker=True, **kw), True, 1.0)
  n = 1
  while dh.lane_change_state != LaneChangeState.laneChangeStarting and n < 200:
    dh.update(CS(right_blinker=True, pressed=True, torque=-1.5, **kw), True, 1.0)
    n += 1
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  return n


def hold(dh, seconds, **kw):
  for _ in range(int(seconds / DT_MDL)):
    dh.update(CS(right_blinker=True, **kw), True, 1.0)


class TestAborts:
  def test_a_car_arriving_alongside_early_calls_it_off(self):
    dh = DesireHelper()
    commit_right(dh)
    dh.update(CS(right_blinker=True, right_bs=True), True, 1.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_the_desire_stops_with_it(self):
    """Reverting the state is only half of it -- the desire is what the lateral planner acts on."""
    dh = DesireHelper()
    commit_right(dh)
    assert dh.desire == log.Desire.laneChangeRight
    dh.update(CS(right_blinker=True, right_bs=True), True, 1.0)
    assert dh.desire == log.Desire.none

  def test_the_automatic_start_has_to_earn_its_delay_again(self):
    """Otherwise a blindspot that blinks off re-commits on the very next frame."""
    dh = DesireHelper()
    dh.auto_lane_change_delay = 2.0
    commit_right(dh)
    dh.update(CS(right_blinker=True, right_bs=True), True, 1.0)
    assert dh.auto_lane_change_timer == 0.0


class TestDoesNotAbort:
  def test_not_once_the_car_is_committed(self):
    """Past the window the move stands, because coming back is worse than finishing."""
    dh = DesireHelper()
    commit_right(dh)
    hold(dh, LANE_CHANGE_ABORT_S + 0.2)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    dh.update(CS(right_blinker=True, right_bs=True), True, 1.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_not_for_the_other_side(self):
    """A car to the left is no reason to stop moving right."""
    dh = DesireHelper()
    commit_right(dh)
    dh.update(CS(right_blinker=True, left_bs=True), True, 1.0)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_an_ordinary_change_still_completes(self):
    dh = DesireHelper()
    commit_right(dh)
    hold(dh, 2.0)
    # the model reports the move done
    for _ in range(3):
      dh.update(CS(right_blinker=True), True, 0.0)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


class TestWindow:
  @pytest.mark.parametrize("elapsed, aborts", [(0.0, True), (0.5, True), (1.5, False)])
  def test_where_the_boundary_sits(self, elapsed, aborts):
    dh = DesireHelper()
    commit_right(dh)
    hold(dh, elapsed)
    before = dh.lane_change_state
    dh.update(CS(right_blinker=True, right_bs=True), True, 1.0)
    assert before == LaneChangeState.laneChangeStarting
    assert (dh.lane_change_state == LaneChangeState.preLaneChange) is aborts

  def test_the_window_is_short_enough_to_still_be_in_lane(self):
    """The justification for aborting at all: at 60mph a change covers 3.6m in 4-5s, so one
    second is about 0.7m and the car has not left its lane."""
    assert LANE_CHANGE_ABORT_S <= 1.0
