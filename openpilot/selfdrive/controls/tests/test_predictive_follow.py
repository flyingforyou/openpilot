"""ApproachCoast (carrot_t_follow.approach_accel_cap) and LeadAheadBrake (lib/lead_ahead.py)."""
import pytest

from openpilot.selfdrive.controls.lib import carrot_t_follow as ctf
from openpilot.selfdrive.controls.lib import lead_ahead as la


class T:
  def __init__(self, tid, d, v, a=0.0, off=0.0, cnt=100):
    self.identifier, self.dRel, self.vLead, self.aLeadK, self.pathOffset, self.cnt = tid, d, v, a, off, cnt


def lead(d=30.0, v=20.0, a=0.0, tid=1, radar=True):
  return {"present": True, "radar": radar, "radarTrackId": tid, "dRel": d, "vLead": v, "aLeadK": a}


class TestApproachCoast:
  def setup_method(self):
    ctf.APPROACH_COAST = (12.0, 3.0, 2.0)

  def teardown_method(self):
    ctf.APPROACH_COAST = None

  def test_off_is_untouched(self):
    ctf.APPROACH_COAST = None
    assert ctf.approach_accel_cap(1.2, True, 40.0, 0.0, 20.0, 15.0, 20.0) == 1.2

  def test_no_acceleration_into_a_slower_car_near_the_gap(self):
    # 5 m/s faster than a lead 10 m beyond the following gap: nothing to gain by speeding up
    assert ctf.approach_accel_cap(1.2, True, 30.0, 0.0, 20.0, 15.0, 20.0) == 0.0

  def test_plenty_of_room_keeps_the_acceleration(self):
    # 2 m/s faster with 150 m to spare: closing at 12.5 m/s would still be fine
    assert ctf.approach_accel_cap(1.2, True, 170.0, 0.0, 20.0, 18.0, 20.0) == 1.2

  def test_a_braking_lead_counts_as_slower_than_it_reads(self):
    steady = ctf.approach_accel_cap(1.2, True, 60.0, 0.0, 20.0, 19.0, 20.0)
    braking = ctf.approach_accel_cap(1.2, True, 60.0, -1.0, 20.0, 19.0, 20.0)
    assert braking < steady

  def test_a_lead_pulling_away_is_untouched(self):
    # every launch: the lead is faster, the gap is short
    assert ctf.approach_accel_cap(1.5, True, 6.0, 0.5, 6.0, 8.0, 12.0) == 1.5

  def test_below_min_speed_is_untouched(self):
    assert ctf.approach_accel_cap(1.5, True, 8.0, 0.0, 3.0, 0.0, 6.0) == 1.5

  def test_never_brakes_and_never_adds(self):
    for a in (-1.0, -0.2, 0.0):
      assert ctf.approach_accel_cap(a, True, 20.0, -2.0, 25.0, 10.0, 20.0) == a
    for d in range(10, 200, 10):
      out = ctf.approach_accel_cap(0.8, True, float(d), 0.0, 20.0, 17.0, 20.0)
      assert 0.0 <= out <= 0.8

  def test_fades_in_with_closing_speed(self):
    outs = [ctf.approach_accel_cap(1.0, True, 25.0, 0.0, 20.0, 20.0 - c, 20.0) for c in (0.0, 0.25, 0.5, 0.75, 1.0)]
    assert outs[0] == 1.0 and outs[-1] == 0.0
    assert all(a >= b for a, b in zip(outs, outs[1:], strict=False))


class TestApproachCoastRate:
  def setup_method(self):
    ctf.APPROACH_COAST = (12.0, 3.0, 2.0)

  def teardown_method(self):
    ctf.APPROACH_COAST = None

  def test_takes_acceleration_away_gradually_and_gives_it_back_gradually(self):
    f = ctf.ApproachCoast(0.05)
    slow = [(30.0, 0.0, 15.0, 20.0, 20.0)]
    outs = [f.update(1.2, slow) for _ in range(40)]
    assert outs[0] == pytest.approx(1.2 - ctf.APPROACH_RATE * 0.05)
    assert all(a - b <= ctf.APPROACH_RATE * 0.05 + 1e-9 for a, b in zip(outs, outs[1:], strict=False))
    assert outs[-1] == 0.0
    back = [f.update(1.2, []) for _ in range(40)]
    assert all(b - a <= ctf.APPROACH_RATE * 0.05 + 1e-9 for a, b in zip(back, back[1:], strict=False))
    assert back[-1] == pytest.approx(1.2)

  def test_never_turns_into_braking(self):
    f = ctf.ApproachCoast(0.05)
    slow = [(30.0, 0.0, 15.0, 20.0, 20.0)]
    for _ in range(20):
      f.update(1.2, slow)
    assert f.update(0.3, slow) >= 0.0
    assert f.update(-1.0, slow) == -1.0


class TestLeadAhead:
  def test_needs_nothing_when_the_car_ahead_is_not_slower(self):
    assert la.needed_lead_accel(20.0, 40.0, 20.0, 0.0) == 0.0

  def test_need_grows_as_the_room_shrinks(self):
    far = la.needed_lead_accel(20.0, 60.0, 15.0, 0.0)
    near = la.needed_lead_accel(20.0, 30.0, 15.0, 0.0)
    assert near < far < 0.0

  def test_bounded_by_speed(self):
    assert la.needed_lead_accel(10.0, 8.0, 0.0, 0.0) == pytest.approx(-1.5)
    assert la.needed_lead_accel(30.0, 30.0, 10.0, 0.0) == pytest.approx(-0.8)

  def test_finds_the_nearest_on_path_track_beyond_the_lead(self):
    tracks = {1: T(1, 30, 20), 2: T(2, 33, 20), 3: T(3, 70, 15), 4: T(4, 55, 15, off=3.5), 5: T(5, 60, 15, cnt=5), 6: T(6, 90, 10)}
    assert la.find_ahead(tracks, 1, 30.0).identifier == 3

  def test_lowers_the_leads_acceleration_after_the_hold_and_only_downwards(self):
    f = la.LeadAhead(0.05)
    tracks = {1: T(1, 30, 20), 2: T(2, 60, 12, a=-1.0)}
    outs = [f.update(tracks, lead()) for _ in range(la.HOLD_FRAMES + 1)]
    assert outs[:la.HOLD_FRAMES - 1] == [None] * (la.HOLD_FRAMES - 1)
    assert outs[-1] is not None and outs[-1] < la.ENGAGE
    # a lead already braking harder than expected keeps its own figure
    assert f.update(tracks, lead(a=-3.0)) is None

  def test_lets_go_gradually(self):
    f = la.LeadAhead(0.05)
    tracks = {1: T(1, 30, 20), 2: T(2, 60, 12, a=-1.0)}
    for _ in range(10):
      low = f.update(tracks, lead())
    gone = {1: T(1, 30, 20)}
    a1 = f.update(gone, lead())
    assert a1 is not None and low < a1 <= low + la.RELEASE_RATE * 0.05 + 1e-9
    for _ in range(40):
      a1 = f.update(gone, lead())
    assert a1 is None

  def test_a_lead_pulling_away_is_not_expected_to_brake(self):
    f = la.LeadAhead(0.05)
    tracks = {1: T(1, 12, 8), 2: T(2, 30, 3)}
    for _ in range(10):
      assert f.update(tracks, lead(d=12.0, v=8.0, a=1.0)) is None

  def test_vision_only_lead_is_left_alone(self):
    f = la.LeadAhead(0.05)
    tracks = {2: T(2, 60, 12, a=-1.0)}
    for _ in range(10):
      assert f.update(tracks, lead(radar=False, tid=-1)) is None
