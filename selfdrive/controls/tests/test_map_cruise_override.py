import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.map_cruise import (
  MapCruiseController,
  OVERRIDE_MAX_DELTA,
  OVER_LIMIT_CAP,
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


def make(v_max_mph=80):
  c = MapCruiseController()
  c.set_config(enabled=True, offset_ratio=1.0, use_car_offset=True,
               v_max=v_max_mph * MPH, use_curve=False, sync_cluster=True)
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


def settle(c, limit_mph, stalk_mph, v_ego_mph=40, n=40):
  """Run enough frames for slews and dwells to finish, return the ceiling in mph."""
  cs = FakeCS(FakeNav(limit_mph, road_class_for(limit_mph)))
  for _ in range(n):
    c.update(cs, v_ego_mph * MPH, stalk_mph * MPH)
  return c.v_ceiling / MPH


class TestOverrideFollowsTheRoad:
  def test_map_target_is_posted_plus_car_offset(self):
    c = make()
    assert settle(c, 45, 55) == pytest.approx(55, abs=1.0)

  def test_ceiling_rises_with_the_posted_limit(self):
    # The logged failure: driver dials 45 on a 45 zone, road opens to 65, MAX stayed at 45
    # through the whole stretch because the override was an absolute speed.
    c = make()
    settle(c, 45, 45)
    assert settle(c, 65, 45) > 55.0

  def test_ceiling_falls_with_the_posted_limit(self):
    c = make()
    settle(c, 65, 70)
    assert settle(c, 25, 70) < 45.0

  def test_driver_delta_is_preserved_across_zones(self):
    # "5 under" has to keep meaning 5 under when the road changes, not 40 forever.
    c = make()
    settle(c, 45, 55)            # sitting on the map's own target, no delta
    low = settle(c, 45, 50)      # driver dials 5 below it
    assert low == pytest.approx(50, abs=1.5)
    moved = settle(c, 65, 50)    # same stalk value, new road
    assert moved > low + 5.0

  def test_delta_is_bounded(self):
    c = make()
    settle(c, 45, 55)
    settle(c, 45, 5)             # absurd dial-down
    assert abs(c.override_delta) <= OVERRIDE_MAX_DELTA + 1e-6

  def test_returning_to_the_map_clears_the_override(self):
    c = make()
    settle(c, 45, 55)
    settle(c, 45, 48)
    assert c.has_override
    settle(c, 45, 55)            # driver puts it back where the map wanted it
    assert not c.has_override

  def test_over_limit_cap_still_binds(self):
    # The runaway guard has to survive the rework: nothing may sit above posted + cap.
    c = make(v_max_mph=90)
    ceiling = settle(c, 25, 80)
    assert ceiling <= (25 * MPH + OVER_LIMIT_CAP) / MPH + 1.0

  def test_no_posted_limit_does_not_latch_a_delta(self):
    c = make()
    nav = FakeNav(45, road_class_for(45))
    nav.baseSpeedLimit = nav.mapSpeedLimit = nav.mppSpeedLimit = nav.fusedSpeedLimit = 0.0
    cs = FakeCS(nav)
    for _ in range(40):
      c.update(cs, 40 * MPH, 60 * MPH)
    assert abs(c.override_delta) <= OVERRIDE_MAX_DELTA + 1e-6
