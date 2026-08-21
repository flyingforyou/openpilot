"""Holding a side while a vehicle we just passed is still level with us.

The property that matters most is not accuracy -- one distance and one closing speed cannot be
accurate -- it is that the hold always ends. Waiting on the blind spot instead was rejected
precisely because it fired on only 32-66% of overtakes, leaving a gate that never reopens.
"""
import pytest

from openpilot.selfdrive.controls.lib.overtake_block import (
  ALONGSIDE_DX,
  CLEAR_DX,
  GROUP_LEFT,
  GROUP_RIGHT,
  LOST_AFTER_S,
  MAX_BLOCK_S,
  MAX_LOST_DX,
  MIN_CLOSING,
  OvertakeBlock,
)

DT = 0.05


class Obj:
  def __init__(self, group, dx, vx_rel=-3.0, obj_id=1):
    self.group, self.dx, self.vxRel, self.objId = group, dx, vx_rel, obj_id


def pass_and_lose(ob, *, group=GROUP_LEFT, dx=8.0, vx_rel=-3.0, t0=0.0, v_ego=25.0, obj_id=1):
  """Report a vehicle, then stop -- an overtake as the camera sees it. Returns the time of loss."""
  for i in range(6):
    ob.update([Obj(group, dx, vx_rel, obj_id)], t0 + i * DT, v_ego)
  t = t0 + 6 * DT
  while t < t0 + 6 * DT + LOST_AFTER_S + 0.2:
    ob.update([], t, v_ego)
    t += DT
  return t


class TestHolds:
  def test_a_vehicle_we_passed_holds_its_side(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, group=GROUP_LEFT)
    assert ob.blocked(t)[0] is True

  def test_and_only_its_side(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, group=GROUP_LEFT)
    assert ob.blocked(t)[1] is False

  def test_the_right_side_too(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, group=GROUP_RIGHT)
    assert ob.blocked(t) == (False, True)

  def test_longer_when_we_are_barely_passing(self):
    """The whole point of using the closing speed: crawling past holds longer than sweeping past."""
    fast, slow = OvertakeBlock(), OvertakeBlock()
    t_f = pass_and_lose(fast, vx_rel=-8.0)
    t_s = pass_and_lose(slow, vx_rel=-1.0)
    end_f = next(x for x in range(200) if not fast.blocked(t_f + x * 0.1)[0])
    end_s = next(x for x in range(200) if not slow.blocked(t_s + x * 0.1)[0])
    assert end_s > end_f


class TestAlwaysEnds:
  """The failure the blind spot approach had, and the one this must not have."""

  def test_the_hold_expires(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob)
    assert ob.blocked(t + MAX_BLOCK_S + 1.0) == (False, False)

  def test_even_at_a_crawl(self):
    """A closing speed barely over the threshold would compute an enormous window."""
    ob = OvertakeBlock()
    t = pass_and_lose(ob, dx=MAX_LOST_DX - 0.1, vx_rel=-(MIN_CLOSING + 0.01))
    assert ob.blocked(t + MAX_BLOCK_S + 0.5) == (False, False)

  def test_nothing_further_is_needed_to_release_it(self):
    """No flag, no second sighting -- time alone."""
    ob = OvertakeBlock()
    t = pass_and_lose(ob)
    for i in range(int((MAX_BLOCK_S + 2.0) / DT)):
      ob.update([], t + i * DT, 25.0)
    assert ob.blocked(t + MAX_BLOCK_S + 2.0) == (False, False)


class TestDoesNotHold:
  def test_a_vehicle_that_drove_away_ahead(self):
    """Lost far out and pulling away is not a vehicle we passed."""
    ob = OvertakeBlock()
    t = pass_and_lose(ob, dx=MAX_LOST_DX + 5.0, vx_rel=-3.0)
    assert ob.blocked(t) == (False, False)

  def test_a_vehicle_pulling_ahead_of_us(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, vx_rel=+3.0)
    assert ob.blocked(t) == (False, False)

  def test_one_keeping_station_with_us(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, vx_rel=-(MIN_CLOSING / 2))
    assert ob.blocked(t) == (False, False)

  def test_a_vehicle_further_up_the_next_lane(self):
    """Something you change lanes behind, not into. Holding on this refuses ordinary moves."""
    ob = OvertakeBlock()
    for i in range(40):
      ob.update([Obj(GROUP_LEFT, ALONGSIDE_DX + 5.0)], i * DT, 25.0)
    assert ob.blocked(40 * DT) == (False, False)

  def test_a_momentary_dropout_is_not_a_loss(self):
    """Measured beyond ALONGSIDE_DX, so nothing is holding it on sight -- this is only about the
    inferred hold, which a gap in reporting must not start."""
    ob = OvertakeBlock()
    for i in range(10):
      ob.update([Obj(GROUP_LEFT, ALONGSIDE_DX + 2.0)], i * DT, 25.0)
    for i in range(10, 10 + int(LOST_AFTER_S / DT) - 2):
      ob.update([], i * DT, 25.0)
    assert ob.blocked(20 * DT) == (False, False)

  def test_standing_still(self):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, v_ego=0.0)
    assert ob.blocked(t) == (False, False)


class TestStillVisible:
  """The half that needs no inference at all. Without it the side only locked out once the
  vehicle had vanished from the camera, which on the road meant the hold arrived after the
  moment it was for."""

  def test_a_vehicle_level_with_us_holds_immediately(self):
    ob = OvertakeBlock()
    for i in range(10):
      ob.update([Obj(GROUP_LEFT, ALONGSIDE_DX - 2.0)], i * DT, 25.0)
    assert ob.blocked(10 * DT) == (True, False)

  def test_no_waiting_for_it_to_disappear(self):
    """It holds on the very first frame it is seen that close."""
    ob = OvertakeBlock()
    ob.update([Obj(GROUP_RIGHT, 5.0)], 0.0, 25.0)
    assert ob.blocked(0.0) == (False, True)

  def test_and_releases_when_it_is_gone_and_no_overtake_was_inferred(self):
    """A vehicle that pulls ahead rather than falling back leaves nothing behind it."""
    ob = OvertakeBlock()
    for i in range(10):
      ob.update([Obj(GROUP_LEFT, 5.0, vx_rel=+3.0)], i * DT, 25.0)
    t = 10 * DT
    while t < 10 * DT + LOST_AFTER_S + 0.2:
      ob.update([], t, 25.0)
      t += DT
    assert ob.blocked(t) == (False, False)

  def test_it_hands_over_to_the_inferred_hold(self):
    """Seen at close range, then lost while being passed: the hold must not lapse in between."""
    ob = OvertakeBlock()
    for i in range(10):
      ob.update([Obj(GROUP_LEFT, 6.0, vx_rel=-3.0)], i * DT, 25.0)
    assert ob.blocked(10 * DT)[0] is True
    t = 10 * DT
    while t < 10 * DT + LOST_AFTER_S + 0.2:
      ob.update([], t, 25.0)
      assert ob.blocked(t)[0] is True, f"the side came free at t={t:.2f}"
      t += DT


class TestWindow:
  @pytest.mark.parametrize("dx, vx_rel", [(8.0, -3.0), (4.0, -5.0), (12.0, -2.0)])
  def test_it_is_the_time_to_clear_our_tail(self, dx, vx_rel):
    ob = OvertakeBlock()
    t = pass_and_lose(ob, dx=dx, vx_rel=vx_rel)
    want = min((dx - CLEAR_DX) / abs(vx_rel), MAX_BLOCK_S)
    assert ob.blocked(t + want - 0.3)[0] is True
    assert ob.blocked(t + want + 0.3)[0] is False

  def test_the_longest_hold_wins(self):
    """Two vehicles passed in quick succession must not let the first one's expiry free the side."""
    ob = OvertakeBlock()
    t = pass_and_lose(ob, dx=4.0, vx_rel=-8.0)
    pass_and_lose(ob, dx=14.0, vx_rel=-2.0, t0=t, obj_id=2)
    assert ob.blocked(t + 3.0)[0] is True
