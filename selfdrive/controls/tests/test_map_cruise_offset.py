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
  c.set_config(enabled=True, offset_ratio=1.0, use_car_offset=True,
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

  def test_car_offset_is_a_ceiling_not_the_value(self):
    # A driver who set +3 in the car's own menu gets +3, not the ladder's +10.
    nav = FakeNav(65, road_class_for(65))
    nav.speedOffset = 3 * MPH
    assert settle(make(), 65, 85, nav=nav) == pytest.approx(68, abs=1.0)

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
