import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.carrot_functions import DEFAULT_COMFORT_BRAKE_2
from openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc import (
  COMFORT_BRAKE,
  desired_follow_distance,
  get_stopped_equivalence_factor,
)

MPH = 0.44704
SPEEDS = [10.0, 20.0, 30.0, 40.0, 50.0, 70.0]
STOP = 3.5
T_FOLLOW = 0.66


def steady_gap(v, b1, b2, t_follow=T_FOLLOW, stop=STOP):
  return desired_follow_distance(v, v, b1, stop, t_follow, b2)


def curvature(b1, b2):
  return 1 / (2 * b1) - 1 / (2 * b2)


class TestComfortBrakePair:
  def test_default_matches_module_constant(self):
    # carrot_functions cannot import long_mpc (long_mpc imports XState from it), so the
    # default is duplicated. If they drift, an unset ComfortBrake2 silently bends the curve.
    assert DEFAULT_COMFORT_BRAKE_2 == COMFORT_BRAKE

  def test_lead_term_still_defaults(self):
    assert get_stopped_equivalence_factor(20.0) == pytest.approx(20.0**2 / (2 * COMFORT_BRAKE))

  def test_equal_pair_cancels(self):
    # b1 == b2 is upstream's symmetric case: the v**2 terms cancel and the gap is linear,
    # so the pair's value stops mattering at steady state.
    for b in (2.0, 2.5, 3.0):
      for s in SPEEDS:
        v = s * MPH
        assert steady_gap(v, b, b) == pytest.approx(T_FOLLOW * v + STOP)

  def test_curvature_is_the_quadratic_coefficient(self):
    for b1, b2 in ((2.16, 2.5), (2.0, 2.5), (2.8, 3.0)):
      k = curvature(b1, b2)
      for s in SPEEDS:
        v = s * MPH
        assert steady_gap(v, b1, b2) == pytest.approx(k * v * v + T_FOLLOW * v + STOP)

  def test_positive_curvature_only_adds_at_speed(self):
    # The property that makes the pair usable: raising curvature must not pull the low-speed
    # end in. Tightening low speed has to come from t_follow, never from k.
    base = [steady_gap(s * MPH, 2.5, 2.5) for s in SPEEDS]
    curved = [steady_gap(s * MPH, 2.16, 2.5) for s in SPEEDS]
    deltas = [c - b for c, b in zip(curved, base, strict=True)]
    assert all(d >= 0.0 for d in deltas)
    assert deltas == sorted(deltas)          # grows monotonically with speed
    # and it stays concentrated at the top of the range -- that concentration is the whole
    # reason the pair can buy highway room without giving any of it back at low speed.
    assert deltas[0] / deltas[-1] < 0.05
    assert deltas[-1] > 5.0

  def test_inverted_pair_shrinks_gap_with_speed(self):
    # The failure mode the clamp exists to prevent -- b1 > b2 made the 74mph target 6m.
    gaps = [steady_gap(s * MPH, 3.2, 2.5) for s in SPEEDS]
    assert gaps[-1] < gaps[0]
    assert gaps[-1] < 10.0

  @pytest.mark.parametrize("b1_raw,b2", [(3.2, 2.5), (2.8, 2.5), (3.6, 3.0), (2.16, 2.5)])
  def test_clamp_keeps_curvature_non_negative(self, b1_raw, b2):
    b1 = min(b1_raw, b2)                     # the clamp in CarrotPlanner._params_update
    assert curvature(b1, b2) >= 0.0
    gaps = [steady_gap(s * MPH, b1, b2) for s in SPEEDS]
    assert gaps == sorted(gaps)              # never shrinks as speed rises

  def test_solved_config_hits_both_targets(self):
    # b1=2.16 / b2=2.50 / t_follow=0.66 / stop=3.5 was solved for 18m at 30mph and 55m at
    # 70mph. Guard both ends so a change to either half is caught.
    assert steady_gap(30 * MPH, 2.16, 2.50) == pytest.approx(18.0, abs=0.3)
    assert steady_gap(70 * MPH, 2.16, 2.50) == pytest.approx(55.0, abs=0.3)

  def test_headway_floor_across_gap_table(self):
    # Every gap position, every speed: the 2026-08-14 near-miss sat at 0.87s.
    table = [0.46, 0.51, 0.56, 0.66, 0.76, 0.86, 0.96]
    worst = min(
      steady_gap(s * MPH, 2.16, 2.50, t_follow=t) / (s * MPH)
      for t in table for s in SPEEDS
    )
    assert worst > 1.1, f"minimum headway {worst:.2f}s"

  def test_approach_demand_is_preserved(self):
    # Raising curvature while lowering t_follow must not quietly add braking demand when
    # closing on a slower lead, which is what would resurrect the hard-braking complaint.
    for ve_mph, vl_mph in ((70, 50), (70, 25), (50, 30), (30, 10)):
      ve, vl = ve_mph * MPH, vl_mph * MPH
      before = desired_follow_distance(ve, vl, 2.4, 4.5, 1.30, 2.5)
      after = desired_follow_distance(ve, vl, 2.16, STOP, T_FOLLOW, 2.5)
      assert abs(after - before) < 6.0, f"{ve_mph}/{vl_mph}: {before:.1f} -> {after:.1f}"


class TestObstacleFormulation:
  def test_obstacle_uses_the_same_lead_brake(self):
    # lead_0_obstacle is what the MPC actually constrains against; if it kept the hardcoded
    # constant while the cost took the parameter, the number would move but the car would not.
    v_lead = np.array([0.0, 10.0, 20.0, 30.0])
    x_lead = np.array([10.0, 20.0, 40.0, 60.0])
    for b2 in (2.5, 3.0, 3.5):
      obstacle = x_lead + get_stopped_equivalence_factor(v_lead, b2)
      assert np.allclose(obstacle, x_lead + v_lead**2 / (2 * b2))
