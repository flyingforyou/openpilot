from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import lane_change_boost as lb

S, D = log.LaneChangeState, log.LaneChangeDirection
V = 30.0


def model(state=S.off, direction=D.none, left_y=-1.8, right_y=1.8):
  line = lambda y: SimpleNamespace(x=[0.0, 50.0], y=[y, y])
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction),
                         laneLines=[line(left_y - 3.6), line(left_y), line(right_y), line(right_y + 3.6)])


def run(b, bend_lat=0.0, direction=D.left, seconds=3.0, gain=1.0, line=None, v=V):
  """Settle in the bend, then a lane change; returns the lateral acceleration added (+ right), per frame."""
  k = bend_lat / v ** 2
  for _ in range(500):
    b.update(k, model(), v, True, gain)
  out = []
  for i in range(int(seconds / DT_CTRL)):
    kw = {} if line is None else ({'left_y': line(i * DT_CTRL)} if direction == D.left else {'right_y': line(i * DT_CTRL)})
    out.append(b.update(k, model(S.laneChangeStarting, direction, **kw), v, True, gain) * v ** 2)
  return np.array(out)


class TestLaneChangeBoost:
  def test_straight_gets_the_base_push_toward_the_change(self):
    left = run(lb.LaneChangeBoost(), direction=D.left)
    right = run(lb.LaneChangeBoost(), direction=D.right)
    assert left.min() == pytest.approx(-lb.BASE, rel=0.01) and left.max() <= 0.0
    assert right.max() == pytest.approx(lb.BASE, rel=0.01) and right.min() >= 0.0

  def test_outward_in_a_bend_gets_more_inward_does_not(self):
    # right-hand bend at 1.0 m/s^2: left is the outside
    outward = run(lb.LaneChangeBoost(), bend_lat=1.0, direction=D.left)
    inward = run(lb.LaneChangeBoost(), bend_lat=1.0, direction=D.right)
    assert -outward.min() == pytest.approx(lb.BASE + lb.OUTSIDE_GAIN * 1.0, rel=0.02)
    assert inward.max() == pytest.approx(lb.BASE, rel=0.02)
    # and mirrored: left-hand bend, right is the outside
    assert run(lb.LaneChangeBoost(), bend_lat=-1.0, direction=D.right).max() == pytest.approx(lb.BASE + lb.OUTSIDE_GAIN * 1.0, rel=0.02)

  def test_capped(self):
    assert -run(lb.LaneChangeBoost(), bend_lat=3.0, direction=D.left).min() == pytest.approx(lb.MAX_PUSH, rel=0.01)

  def test_ramps_in_then_fades_by_itself(self):
    out = -run(lb.LaneChangeBoost(), bend_lat=1.0, direction=D.left, seconds=3.0)
    t = np.arange(len(out)) * DT_CTRL
    assert out[t < 0.1].max() < 0.5 * out.max()                       # no step at the start
    assert out[(t > lb.RAMP_S) & (t < lb.PUSH_S)].min() == pytest.approx(out.max(), rel=0.01)
    assert out[t > lb.PUSH_S + 1.5].max() < 0.02                      # gone well before the model finishes
    assert np.abs(np.diff(out)).max() / DT_CTRL < 2.5                 # m/s^3

  def test_ends_when_the_line_is_nearly_under_the_car(self):
    # the left line comes in at 1.5 m/s: under 0.6 m from 0.8 s
    out = -run(lb.LaneChangeBoost(), direction=D.left, line=lambda t: -1.8 + 1.5 * t)
    t = np.arange(len(out)) * DT_CTRL
    assert out[(t > 0.5) & (t < 0.78)].min() > 0.9 * lb.BASE
    assert out[t > 0.8 + 1.0].max() < 0.01

  def test_gain_scales_and_zero_is_off(self):
    assert -run(lb.LaneChangeBoost(), direction=D.left, gain=1.5).min() == pytest.approx(1.5 * lb.BASE, rel=0.01)
    assert np.all(run(lb.LaneChangeBoost(), bend_lat=1.0, direction=D.left, gain=0.0) == 0.0)

  def test_nothing_outside_a_lane_change_or_without_lateral(self):
    b = lb.LaneChangeBoost()
    assert all(b.update(1.0 / V ** 2, model(), V, True, 1.0) == 0.0 for _ in range(300))
    assert all(b.update(0.0, model(S.preLaneChange, D.left), V, True, 1.0) == 0.0 for _ in range(100))
    assert all(b.update(0.0, model(S.laneChangeStarting, D.left), V, False, 1.0) == 0.0 for _ in range(100))

  def test_the_bend_is_not_relearned_from_the_change_itself(self):
    b = lb.LaneChangeBoost()
    run(b, bend_lat=0.0, direction=D.left)             # on a straight
    # the model's own lane-change curvature (to the left) must not have been read as a left-hand bend:
    # a second change to the right straight after would then count as outward
    for _ in range(300):
      b.update(-0.8 / V ** 2, model(S.laneChangeStarting, D.left), V, True, 1.0)
    for _ in range(20):
      b.update(0.0, model(), V, True, 1.0)
    assert lb.LaneChangeBoost.push_for(b._road_curvature, V, True, 1.0) == pytest.approx(lb.BASE, rel=0.05)

  def test_a_new_change_starts_its_own_push(self):
    b = lb.LaneChangeBoost()
    run(b, direction=D.left)
    for _ in range(100):
      b.update(0.0, model(), V, True, 1.0)
    assert run(b, direction=D.right).max() == pytest.approx(lb.BASE, rel=0.01)
