"""Slowing for what is already in the lane being moved into.

Until the change completes, that vehicle is not a lead and nothing anticipates it -- so a move in
behind a lorry is made at the speed we were already doing, and the braking happens afterwards.
Measured at the moments lane changes began, 3 of 7 had a slower vehicle ahead in the target lane,
a median 2 m/s slower at 16-37 m: a gap that closes in 8-18 s.

The decision is a pure function so it can be tested at all -- importing radard pulls in the whole
messaging stack -- and because it is the decision, not the plumbing, that can be wrong.
"""
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.lane_change_guards import target_lane_lead

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


class Lead:
  def __init__(self, present=True, d_rel=30.0):
    self.present, self.dRel = present, d_rel


class Meta:
  def __init__(self, state, direction):
    self.laneChangeState, self.laneChangeDirection = state, direction


class State:
  def __init__(self, left, right, two):
    self.leadLeft, self.leadRight, self.leadTwo = left, right, two


def run(state, meta):
  """What radard does with the answer: take it when there is one, else leave leadTwo alone."""
  target = target_lane_lead(meta, state.leadLeft, state.leadRight, state.leadTwo)
  return target if target is not None else state.leadTwo


STARTING = LaneChangeState.laneChangeStarting
PRE = LaneChangeState.preLaneChange


class TestTakesTheTargetLane:
  @pytest.mark.parametrize("state", [STARTING, PRE])
  def test_moving_left_takes_the_left_lane(self, state):
    left = Lead(d_rel=20.0)
    out = run(State(left, Lead(d_rel=10.0), Lead(present=False)),
              Meta(state, LaneChangeDirection.left))
    assert out is left

  def test_moving_right_takes_the_right_lane(self):
    right = Lead(d_rel=20.0)
    out = run(State(Lead(d_rel=10.0), right, Lead(present=False)),
              Meta(STARTING, LaneChangeDirection.right))
    assert out is right

  def test_it_wins_when_it_is_the_nearer(self):
    left = Lead(d_rel=15.0)
    out = run(State(left, Lead(present=False), Lead(d_rel=40.0)),
              Meta(STARTING, LaneChangeDirection.left))
    assert out is left


class TestLeavesItAlone:
  def test_when_no_lane_change_is_happening(self):
    two = Lead(present=False)
    out = run(State(Lead(d_rel=10.0), Lead(d_rel=10.0), two),
              Meta(LaneChangeState.off, LaneChangeDirection.none))
    assert out is two

  def test_when_the_direction_is_not_yet_known(self):
    two = Lead(present=False)
    out = run(State(Lead(d_rel=10.0), Lead(d_rel=10.0), two),
              Meta(PRE, LaneChangeDirection.none))
    assert out is two

  def test_when_that_lane_is_empty(self):
    two = Lead(present=False)
    out = run(State(Lead(present=False), Lead(d_rel=10.0), two),
              Meta(STARTING, LaneChangeDirection.left))
    assert out is two

  def test_when_leadTwo_is_already_the_more_binding(self):
    """A cut-in nearer than the target lane's vehicle must not be displaced by it."""
    two = Lead(d_rel=12.0)
    out = run(State(Lead(d_rel=30.0), Lead(present=False), two),
              Meta(STARTING, LaneChangeDirection.left))
    assert out is two

  def test_the_other_side_is_never_taken(self):
    """Moving left must not slow the car for traffic in the right lane."""
    two = Lead(present=False)
    out = run(State(Lead(present=False), Lead(d_rel=5.0), two),
              Meta(STARTING, LaneChangeDirection.left))
    assert out is two
