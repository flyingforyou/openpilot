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
  def test_curve_still_slows(self):
    nav = FakeNav(45, road_class_for(45))
    nav.fleetSplineSpeed = 30 * MPH
    assert settle(make(use_curve=True), 45, 55, nav=nav) < 45.0

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
  """min(fleet, curvature limit): each covers what the other misses."""

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

  def test_straight_fleet_beats_curvature(self):
    # No curvature to speak of: the limit is ~100mph and the fleet governs.
    assert self._run(self.make_curve(), 45, 47.0, 3000) == pytest.approx(47.0, abs=1.5)

  def test_neither_exceeds_the_map_target(self):
    # Both inputs high -> the posted target still bounds it.
    assert self._run(self.make_curve(), 45, 80.0, 3000) == pytest.approx(55, abs=1.0)

  def test_off_defaults_to_fleet_only(self):
    assert self._run(self.make_curve(lat_accel=0.0), 45, 28.5, 31) == pytest.approx(28.5, abs=1.5)

  def test_lower_criterion_slows_more(self):
    tight = self._run(self.make_curve(2.5), 45, 40.0, 100)
    loose = self._run(self.make_curve(3.5), 45, 40.0, 100)
    assert tight < loose

  def test_on_ramp_is_not_slowed_by_curvature(self):
    # Merging must not be held back; the ramp branch takes fleet outright.
    c = self.make_curve()
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 1
    nav.fleetSplineSpeed = 50 * MPH
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 45 * MPH, 70 * MPH, 1.0 / 31)
    assert c.v_ceiling / MPH > 30.0

  def test_off_ramp_does_get_the_curvature_cap(self):
    c = self.make_curve()
    nav = FakeNav(45, road_class_for(45))
    nav.rampType = 2
    nav.fleetSplineSpeed = 34.0 * MPH
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 40 * MPH, 70 * MPH, 1.0 / 31)
    assert c.v_ceiling / MPH < 30.0
