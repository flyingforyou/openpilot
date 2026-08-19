import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.map_cruise import (
  MapCruiseController,
  OFFSET_ABOVE,
  OFFSET_BELOW,
  OFFSET_SPLIT,
)

MPH = CV.MPH_TO_MS
CAR_OFFSET = 10 * MPH          # UI_userSpeedOffset, +10mph on this car in every logged drive


class FakeNav:
  """Only the fields _posted_limit / _with_offset / _fleet_speed actually read."""
  def __init__(self, limit_mph, road_class=4):
    self.valid = True
    self.baseSpeedLimit = limit_mph * MPH
    self.mapSpeedLimit = limit_mph * MPH
    self.mppSpeedLimit = limit_mph * MPH
    self.fusedSpeedLimit = limit_mph * MPH
    self.speedOffset = CAR_OFFSET
    self.roadClass = road_class
    self.rampType = 0
    self.splineConfidence = 99.0
    self.gpsRoadMatch = 1
    self.fleetSplineSpeed = 0.0
    self.fleetTopQuartileSpeed = 0.0
    self.fleetMedianSpeed = 0.0


class FakeCS:
  def __init__(self, nav):
    self.navMap = nav


def make(v_max_mph=90, use_curve=False):
  c = MapCruiseController()
  c.set_config(enabled=True, offset_ratio=1.0,
               v_max=v_max_mph * MPH, use_curve=use_curve, sync_cluster=True)
  return c


def road_class_for(limit_mph):
  """The class cross-check rejects a posted limit above what the class can carry, so a fixture
  has to pair them the way the map does -- a 65 zone is a freeway, not an arterial."""
  if limit_mph > 50:
    return 1
  if limit_mph > 35:
    return 4
  if limit_mph > 30:
    return 5
  return 6


def settle(c, limit_mph, stalk_mph, v_ego_mph=40, n=40, nav=None):
  """Run enough frames for slews and dwells to finish, return the ceiling in mph."""
  cs = FakeCS(nav if nav is not None else FakeNav(limit_mph, road_class_for(limit_mph)))
  for _ in range(n):
    c.update(cs, v_ego_mph * MPH, stalk_mph * MPH)
  return c.v_ceiling / MPH


class TestPerLimitOffset:
  """Below 40mph the fleet runs about 5 over, not 10. See OFFSET_SPLIT."""

  @pytest.mark.parametrize("limit,expected", [(25, 30), (30, 35), (35, 40)])
  def test_low_limits_get_five(self, limit, expected):
    assert settle(make(), limit, 70) == pytest.approx(expected, abs=1.0)

  @pytest.mark.parametrize("limit,expected", [(40, 50), (45, 55), (65, 75)])
  def test_forty_and_up_get_ten(self, limit, expected):
    assert settle(make(), limit, 85) == pytest.approx(expected, abs=1.0)

  def test_split_is_inclusive_at_forty(self):
    assert (40 * MPH) >= OFFSET_SPLIT
    assert settle(make(), 40, 85) == pytest.approx(50, abs=1.0)

  def test_car_offset_no_longer_participates(self):
    # UI_userSpeedOffset used to cap the ladder. It is one number for every road and read +10
    # in 99.9% of logged frames, so it only ever suppressed the ladder where it matters.
    nav = FakeNav(25, road_class_for(25))
    nav.speedOffset = 3 * MPH
    assert settle(make(), 25, 70, nav=nav) == pytest.approx(30, abs=1.0)

  def test_ladder_constants_are_ordered(self):
    assert OFFSET_BELOW < OFFSET_ABOVE


class TestStalkIsATriggerNotASetpoint:
  """A press hands the road to the map; it must not pin the target to itself."""

  def test_ceiling_rises_with_the_posted_limit(self):
    # The logged failure: driver dials 45 on a 45 zone, road opens to 65, MAX stayed at 45.
    c = make()
    settle(c, 45, 45)
    assert settle(c, 65, 45) > 60.0

  def test_ceiling_falls_with_the_posted_limit(self):
    c = make()
    settle(c, 65, 70)
    assert settle(c, 25, 70) < 35.0

  def test_target_ignores_where_the_stalk_sits(self):
    # Same road, three different stalk positions, one answer: the road's.
    c = make()
    assert settle(c, 45, 40) == pytest.approx(55, abs=1.0)
    assert settle(c, 45, 55) == pytest.approx(55, abs=1.0)
    assert settle(c, 45, 80) == pytest.approx(55, abs=1.0)

  def test_a_press_does_not_survive_into_the_next_zone(self):
    c = make()
    settle(c, 45, 80)                       # driver winds it up on a 45
    assert settle(c, 25, 80) < 35.0         # 25 zone still gets 25+5


class TestCapsStillBind:
  def test_fleet_does_not_cap_an_ordinary_road(self):
    # The fleet caps ramps only. It used to cap every road on the theory that it stood in for the
    # bends point curvature misses; measured over 183k non-ramp frames its correlation with
    # lateral acceleration is +0.043, so it was capping for traffic, not geometry. A slow fleet
    # reading on a straight 45 must now leave the posted target alone.
    nav = FakeNav(45, road_class_for(45))
    nav.fleetSplineSpeed = 30 * MPH
    assert settle(make(use_curve=True), 45, 85, nav=nav) == pytest.approx(55.0, abs=1.0)

  def test_ramp_fleet_speed_still_used(self):
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 1
    nav.fleetSplineSpeed = 30 * MPH
    assert settle(make(use_curve=True), 45, 55, v_ego_mph=25, nav=nav) < 45.0

  def test_configured_max_still_binds(self):
    assert settle(make(v_max_mph=50), 65, 85) <= 51.0

  def test_class_ceiling_still_rejects_an_impossible_limit(self):
    nav = FakeNav(65, 6)                    # local road, 30mph class ceiling
    assert settle(make(), 65, 85, nav=nav) < 55.0

  def test_cap_uses_the_same_ladder(self):
    # A 25 zone may not be capped at 25+10 while its target is 25+5.
    assert settle(make(), 25, 85) <= (25 * MPH + OFFSET_BELOW) / MPH + 1.0


class TestCurveLatAccelCap:
  """The curvature limit is the only geometry cap; the fleet caps ramps only."""

  def _run(self, c, limit_mph, fleet_mph, radius_m, stalk=70, n=40):
    nav = FakeNav(limit_mph, road_class_for(limit_mph))
    nav.fleetSplineSpeed = fleet_mph * MPH
    cs = FakeCS(nav)
    curv = (1.0 / radius_m) if radius_m > 0 else 0.0
    for _ in range(n):
      c.update(cs, 40 * MPH, stalk * MPH, curv)
    return c.v_ceiling / MPH

  def make_curve(self, lat_accel=3.0):
    c = MapCruiseController()
    c.set_config(enabled=True, offset_ratio=1.0, v_max=90 * MPH,
                 use_curve=True, sync_cluster=True, curve_lat_accel=lat_accel)
    return c

  def test_hairpin_curvature_beats_fleet(self):
    # radius 31m, fleet 28.5mph -- the logged hairpin. sqrt(3.0 * 31) = 9.6m/s = 21.4mph.
    assert self._run(self.make_curve(), 45, 28.5, 31) == pytest.approx(21.4, abs=1.5)

  def test_straight_leaves_the_posted_target_alone(self):
    # No curvature to speak of, so the curvature limit is ~100mph and nothing else caps a
    # non-ramp road: the posted target stands even with the fleet reading well below it.
    assert self._run(self.make_curve(), 45, 47.0, 3000) == pytest.approx(55.0, abs=1.0)

  def test_neither_exceeds_the_map_target(self):
    # Both inputs high -> the posted target still bounds it.
    assert self._run(self.make_curve(), 45, 80.0, 3000) == pytest.approx(55, abs=1.0)

  def test_off_leaves_the_hairpin_uncapped(self):
    # curve_lat_accel = 0 turns the only geometry cap off. Nothing else looks at the corner, so
    # even the logged hairpin comes back at the posted target -- this is what the toggle costs.
    assert self._run(self.make_curve(lat_accel=0.0), 45, 28.5, 31) == pytest.approx(55.0, abs=1.0)

  def test_lower_criterion_slows_more(self):
    tight = self._run(self.make_curve(2.5), 45, 40.0, 100)
    loose = self._run(self.make_curve(3.5), 45, 40.0, 100)
    assert tight < loose

  def test_on_ramp_loop_does_get_the_curvature_cap(self):
    # The tight loop at the start of an on-ramp: 14.2% of logged on-ramp frames, where the
    # fleet reads 38.6mph against a car actually doing 27.8.
    c = self.make_curve()
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 1
    nav.fleetSplineSpeed = 38.6 * MPH
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 40 * MPH, 70 * MPH, 1.0 / 31)
    assert c.v_ceiling / MPH < 30.0

  def test_on_ramp_merge_is_not_held_back(self):
    # Straight acceleration lane: no curvature, so the cap lets go and the merge guard rules.
    c = self.make_curve()
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 1
    nav.fleetSplineSpeed = 38.6 * MPH
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 55 * MPH, 70 * MPH, 0.0)
    assert c.v_ceiling / MPH >= 55.0

  def test_off_ramp_does_get_the_curvature_cap(self):
    c = self.make_curve()
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 2
    nav.fleetSplineSpeed = 34.0 * MPH
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 40 * MPH, 70 * MPH, 1.0 / 31)
    assert c.v_ceiling / MPH < 30.0


class TestSetpointFollowsTheRoad:
  """The setpoint is the answer, not the route to it.

  There used to be a rate limit here, and at 0.5 m/s^2 up it was three to four times tighter
  than the planner's own acceleration limits -- a measured 45 -> 55 change took 11 s to reach
  the setpoint while the cluster had shown the new number within half a second. How fast the car
  closes the gap is the planner's decision now; what survives is the wait before a *higher*
  limit is believed at all.
  """
  DT = 0.05

  @staticmethod
  def _drive(c, limit_mph, frames, v_ego_mph=40, ramp=0):
    nav = FakeNav(limit_mph, road_class_for(limit_mph))
    nav.rampType = ramp
    cs = FakeCS(nav)
    out = []
    for _ in range(frames):
      out.append(c.update(cs, v_ego_mph * MPH, 70 * MPH) / MPH)
    return out

  def test_lower_limit_is_taken_at_once(self):
    c = make()
    self._drive(c, 65, 100)
    first = self._drive(c, 35, 1)[0]
    assert first == pytest.approx(40, abs=1.0), "a lower limit is not a suggestion"

  def test_higher_limit_waits_then_arrives_whole(self):
    c = make()
    self._drive(c, 35, 100)
    during = self._drive(c, 65, int(2.0 / self.DT))
    assert during[-1] == pytest.approx(40, abs=1.0), "the wait has to hold the old number"
    after = self._drive(c, 65, int(1.5 / self.DT))
    assert after[-1] == pytest.approx(75, abs=1.0), "and then hand over all of it, not a ramp"

  def test_no_intermediate_values_on_the_way_up(self):
    """The point of the change: one step, not a staircase."""
    c = make()
    self._drive(c, 35, 100)
    seen = self._drive(c, 65, int(6.0 / self.DT))
    between = [v for v in seen if 41 < v < 74]
    assert not between, f"setpoint passed through {sorted({round(v) for v in between})}"

  def test_merging_skips_the_wait(self):
    c = make()
    self._drive(c, 35, 100)
    first = self._drive(c, 65, 1, ramp=1)[0]
    assert first > 41, "an on-ramp cannot afford to wait to be believed"

  def test_ceiling_is_the_road_not_the_setpoint(self):
    """What a cluster shows tracks the road immediately, even inside the wait."""
    c = make()
    self._drive(c, 35, 100)
    self._drive(c, 65, 1)
    assert c.v_ceiling / MPH == pytest.approx(75, abs=1.0)
    assert c.v_output / MPH == pytest.approx(40, abs=1.0)


class TestBaseLimitOutvoted:
  """base leads the other sources, but a lone low reading is a bad map match, not early news.

  Measured over three drives: base agrees with the posted band 91.6% of the time, and on 0.44%
  of frames it read 15+ mph below every other source while those agreed -- a class-4 road
  posting 65 with base claiming 40 or 20. Believing it put the car at 25 there.
  """

  @staticmethod
  def _nav(base, others, road_class=4, ramp=0):
    nav = FakeNav(others, road_class)
    nav.baseSpeedLimit = base * MPH
    nav.rampType = ramp
    return nav

  def test_lone_low_base_is_outvoted(self):
    c = make(v_max_mph=90)
    got = settle(c, 65, 70, nav=self._nav(20, 65, road_class=1))
    assert got == pytest.approx(75, abs=1.0), "three sources saying 65 should beat one saying 20"

  def test_the_other_observed_case(self):
    c = make(v_max_mph=90)
    got = settle(c, 65, 70, nav=self._nav(40, 65, road_class=1))
    assert got == pytest.approx(75, abs=1.0)

  def test_ordinary_disagreement_still_prefers_base(self):
    """base leading the posted band by a few mph is the normal, useful case."""
    c = make()
    got = settle(c, 45, 70, nav=self._nav(40, 45, road_class=4))
    assert got == pytest.approx(50, abs=1.0), "a 5 mph lead is base doing its job"

  def test_base_higher_than_the_others_is_untouched(self):
    """The rule is one-directional: it only fires when base is the slow outlier."""
    c = make()
    got = settle(c, 35, 70, nav=self._nav(45, 35, road_class=4))
    assert got == pytest.approx(55, abs=1.0), "base still leads upward"

  def test_never_on_a_ramp(self):
    """Off-ramp: base dropping first is the early warning, not a bad match. 539 of the observed
    frames were here, and overriding base would hold the freeway limit down the exit."""
    c = make(v_max_mph=90)
    got = settle(c, 65, 70, nav=self._nav(20, 65, road_class=1, ramp=2))
    assert got == pytest.approx(25, abs=1.0), "base has to keep its say on a ramp"

  def test_dropped_not_replaced(self):
    """The others are not thereby right: a 65 on a class-4 road is still refused, and the car
    falls back to the class ceiling rather than to either bad number."""
    c = make()
    got = settle(c, 65, 70, nav=self._nav(20, 65, road_class=4))
    assert got > 30, f"should not still be crawling at 25, got {got:.0f}"
    assert got <= 61, f"and should not have believed the 65 either, got {got:.0f}"

  def test_needs_two_agreeing_sources(self):
    c = make()
    nav = self._nav(20, 65, road_class=1)
    nav.mppSpeedLimit = 0.0
    nav.fusedSpeedLimit = 0.0
    got = settle(c, 65, 70, nav=nav)
    assert got == pytest.approx(25, abs=1.0), "one dissenter is not a vote"


class TestClassCeilingSizing:
  """The cross-check has to reject a limit from the road behind without rejecting this one.

  Measured over 421k frames: 35 is the most common posted limit on a class-6 road (9725 frames,
  against 8378 for 30). With the ceiling at 30 it was thrown away as stale on every one of them,
  and the unmapped fallback then held whatever the target already was -- a measured minute at
  25 mph on a 35 mph road.
  """

  @pytest.mark.parametrize("limit,expected", [(25, 30), (30, 35), (35, 40)])
  def test_class_six_accepts_its_own_limits(self, limit, expected):
    c = make()
    assert settle(c, limit, 70, nav=FakeNav(limit, 6)) == pytest.approx(expected, abs=1.0)

  @pytest.mark.parametrize("limit", [45, 65])
  def test_class_six_still_refuses_a_faster_road(self, limit):
    """These are the stale-limit case the cross-check exists for. Refusing means the target is
    held under the class cap rather than the limit's own offset -- not that it drops below the
    number itself, which the offset ladder was always going to sit above."""
    c = make()
    got = settle(c, limit, 70, nav=FakeNav(limit, 6))
    believed = limit + (10 if limit >= 40 else 5)
    assert got < believed, f"a {limit} on a residential road was believed, got {got:.0f}"
    assert got <= 41, f"and should be held at the class cap, got {got:.0f}"

  def test_class_four_still_refuses_a_freeway_limit(self):
    """5.97% of frames, and the reason the whole cross-check is here."""
    c = make()
    assert settle(c, 65, 70, nav=FakeNav(65, 4)) < 65
