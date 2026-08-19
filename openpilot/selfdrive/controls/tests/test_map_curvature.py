"""The curve cap reads two sources split by distance. These pin the split.

The model owns everything inside MAP_CURVE_NEAR and the map everything past it, out to the same
4s horizon the model is read over. Whichever is tighter wins, so the map can slow the car and
never speed it up -- that direction is the whole safety argument for letting a map source touch
a speed cap at all, and it is what most of this file checks.
"""
import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.map_cruise import (
  CURVE_LOOKAHEAD_T,
  MAP_CURVE_NEAR,
  MapCruiseController,
)

MPH = CV.MPH_TO_MS


class Nav:
  """Only the fields _map_curvature reads."""
  def __init__(self, c2=0.0, c3=0.0, rng=252.0, health=3):
    self.curvC2 = c2
    self.curvC3 = c3
    self.curvRange = rng
    self.curvHealth = health


def controller(use_map_curve=True, lat_accel=3.0):
  c = MapCruiseController()
  c.set_config(enabled=True, offset_ratio=1.0, v_max=90 * MPH, use_curve=True,
               curve_lat_accel=lat_accel, use_map_curve=use_map_curve)
  return c


def const_curvature(k):
  """A cubic with no c3 term is a constant curvature of 2*c2 everywhere."""
  return Nav(c2=k / 2.0)


# 60 mph, so the 4s horizon lands at 107m -- past MAP_CURVE_NEAR, which is the case that matters
FAST = 60 * MPH
TOWN = 25 * MPH


class TestWindow:
  def test_reads_a_constant_bend(self):
    assert controller()._map_curvature(const_curvature(0.004), FAST) == pytest.approx(0.004)

  def test_town_speed_asks_nothing_of_the_map(self):
    """At 25mph the 4s horizon is 45m, inside the model's own territory."""
    assert TOWN * CURVE_LOOKAHEAD_T < MAP_CURVE_NEAR
    assert controller()._map_curvature(const_curvature(0.02), TOWN) == 0.0

  def test_horizon_moves_with_speed(self):
    """A bend that tightens with distance is seen further into by a faster car."""
    nav = Nav(c2=0.0, c3=1e-6)     # curvature 6e-6 * x, zero at the bumper
    slow = controller()._map_curvature(nav, 40 * MPH)
    fast = controller()._map_curvature(nav, 80 * MPH)
    assert 0.0 < slow < fast

  def test_never_reads_past_the_declared_range(self):
    """Past curvRange the cubic is extrapolation, and a c3 term extrapolates hard."""
    nav = Nav(c2=0.0, c3=1e-6, rng=80.0)
    capped = controller()._map_curvature(nav, FAST)
    assert capped == pytest.approx(6e-6 * 80.0)
    assert capped < controller()._map_curvature(Nav(c2=0.0, c3=1e-6, rng=252.0), FAST)

  def test_extreme_is_found_at_either_end(self):
    """Curvature is linear in x, so a cubic that straightens with distance peaks at the near
    edge and one that tightens peaks at the far edge. Both have to be caught."""
    straightening = Nav(c2=0.005, c3=-1e-7)   # 0.010 at the bumper, easing off with distance
    near = abs(2 * 0.005 + 6 * -1e-7 * MAP_CURVE_NEAR)
    far = abs(2 * 0.005 + 6 * -1e-7 * (FAST * CURVE_LOOKAHEAD_T))
    assert near > far      # this cubic really does peak at the near edge of the window
    assert controller()._map_curvature(straightening, FAST) == pytest.approx(near)

    tightening = Nav(c2=0.001, c3=1e-7)
    far_t = abs(2 * 0.001 + 6 * 1e-7 * (FAST * CURVE_LOOKAHEAD_T))
    assert controller()._map_curvature(tightening, FAST) == pytest.approx(far_t)

  def test_sign_does_not_matter(self):
    """A left bend and a right bend of the same radius cap the same."""
    left = controller()._map_curvature(const_curvature(0.004), FAST)
    right = controller()._map_curvature(const_curvature(-0.004), FAST)
    assert left == pytest.approx(right) == pytest.approx(0.004)


class TestRefusals:
  @pytest.mark.parametrize("nav", [
    Nav(c2=0.01, health=0),      # gateway says do not use it
    Nav(c2=0.01, rng=0.0),       # nothing described ahead
  ])
  def test_unusable_message_yields_nothing(self, nav):
    assert controller()._map_curvature(nav, FAST) == 0.0

  def test_option_off_yields_nothing(self):
    assert controller(use_map_curve=False)._map_curvature(const_curvature(0.02), FAST) == 0.0

  def test_a_flat_cubic_is_not_a_bend(self):
    assert controller()._map_curvature(Nav(), FAST) == 0.0


class TestOnlySlowsDown:
  """The map is combined with max(), so every one of these is really the same property."""

  def test_map_lowers_the_cap_the_model_would_have_set(self):
    c = controller()
    nav = const_curvature(0.004)
    model_only = c._curve_speed(0.0005)
    combined = c._curve_speed(max(0.0005, c._map_curvature(nav, FAST)))
    assert combined < model_only

  def test_map_cannot_raise_a_cap_the_model_set(self):
    """The model sees a hairpin the map has flattened into a gentle bend. The hairpin wins."""
    c = controller()
    model_k = 0.03
    combined = max(model_k, c._map_curvature(const_curvature(0.001), FAST))
    assert combined == model_k

  def test_a_silent_map_leaves_the_model_alone(self):
    c = controller()
    model_k = 0.008
    assert max(model_k, c._map_curvature(Nav(health=0), FAST)) == model_k

  def test_turning_the_option_off_can_only_raise_the_cap(self):
    """Which is to say: on is the conservative setting, and that is why it is the default."""
    nav = const_curvature(0.004)
    on = controller(use_map_curve=True)._map_curvature(nav, FAST)
    off = controller(use_map_curve=False)._map_curvature(nav, FAST)
    assert on >= off


class TestSpeedItAllows:
  """Sanity on the numbers themselves, at the 3.0 m/s^2 default."""

  @pytest.mark.parametrize("radius_m, expect_mph", [
    (1000, 122),    # motorway sweeper: no constraint
    (250, 61),      # fast A-road bend
    (100, 39),
    (31, 21),       # the measured hairpin, driven through at 17.3mph
  ])
  def test_radius_to_speed(self, radius_m, expect_mph):
    c = controller()
    k = c._map_curvature(const_curvature(1.0 / radius_m), FAST)
    assert c._curve_speed(k) * CV.MS_TO_MPH == pytest.approx(expect_mph, abs=1)

  def test_a_straight_road_is_not_capped_into_traffic(self):
    """1.1% of straights get called a bend by this source; none of them may cap below 45mph.
    The measured worst case over a whole route was 58mph at the 1st percentile."""
    c = controller()
    # the tightest curvature that survived on a genuinely straight road in that measurement
    k = c._map_curvature(const_curvature(0.0007), FAST)
    assert c._curve_speed(k) * CV.MS_TO_MPH > 45


class FullNav(Nav):
  """Everything update() reads, set to a plain 55mph freeway with no bend."""
  def __init__(self, **curv):
    super().__init__(**curv)
    self.valid = True
    self.baseSpeedLimit = self.mapSpeedLimit = 55 * MPH
    self.mppSpeedLimit = self.fusedSpeedLimit = 55 * MPH
    self.speedOffset = 0.0
    self.roadClass = 1
    self.rampType = 0
    self.splineConfidence = 99.0
    self.gpsRoadMatch = 1
    self.fleetSplineSpeed = self.fleetTopQuartileSpeed = self.fleetMedianSpeed = 0.0


class CS:
  def __init__(self, nav):
    self.navMap = nav


def settle(c, nav, v_ego=FAST, model_k=0.0, n=200):
  """Run past the raise dwell so the target is the road's answer, not the ramp toward it."""
  for _ in range(n):
    c.update(CS(nav), v_ego, 70 * MPH, model_k)
  return c.v_target * CV.MS_TO_MPH


class TestWiredIn:
  """The helper is only useful if update() actually consults it."""

  def test_a_bend_only_the_map_sees_slows_the_car(self):
    straight = settle(controller(), FullNav())
    bend = settle(controller(), FullNav(c2=0.002))    # curvature 0.004, R=250m
    assert straight > 60          # 55 limit plus the over-40 offset, uncapped
    assert bend == pytest.approx(61, abs=2)
    assert bend < straight

  def test_the_option_off_restores_the_old_behaviour(self):
    nav = FullNav(c2=0.002)
    assert settle(controller(use_map_curve=False), nav) == settle(controller(), FullNav())

  def test_the_model_still_wins_where_it_is_tighter(self):
    """Map says gentle, model says hairpin -- the cap is the hairpin's, and it is not credited
    to the map."""
    c = controller()
    assert settle(c, FullNav(c2=0.0005), model_k=0.03) == pytest.approx(21, abs=2)
    assert not c.curve_from_map

  def test_the_map_is_credited_when_it_is_the_tighter_one(self):
    c = controller()
    settle(c, FullNav(c2=0.002), model_k=0.0001)
    assert c.curve_from_map

  def test_nothing_is_credited_on_a_straight(self):
    """curve_from_map is a readout, so it must not light up when no cap is being applied."""
    c = controller()
    settle(c, FullNav())
    assert not c.curve_from_map
