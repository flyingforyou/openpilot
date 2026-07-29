import pytest
import itertools
from openpilot.common.parameterized import parameterized_class

from cereal import log

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longitudinal_planner import (get_max_accel, A_CRUISE_MAX_VALS,
                                                                    A_CRUISE_MAX_BP)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (LongitudinalMpc, get_safe_obstacle_distance,
                                                                          get_stopped_equivalence_factor, get_T_FOLLOW,
                                                                          limit_t_follow_increase, gap_t_follow_table,
                                                                          GAP_PROFILES, MIN_T_FOLLOW, T_FOLLOW_RISE_RATE)
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)


@pytest.mark.parametrize("gap, expected", [
  (1, 0.80),
  (2, 0.96),
  (3, 1.12),
  (4, 1.28),
  (5, 1.43),
  (6, 1.59),
  (7, 1.75),
])
def test_tesla_gap_t_follow(gap, expected):
  assert get_T_FOLLOW(log.LongitudinalPersonality.standard, gap_adjust=gap) == pytest.approx(expected)


def test_invalid_gap_uses_personality():
  assert get_T_FOLLOW(log.LongitudinalPersonality.standard, gap_adjust=0) == pytest.approx(1.45)


def test_t_follow_decrease_is_immediate():
  assert limit_t_follow_increase(1.75, 1.10, 0.05) == pytest.approx(1.10)


def test_t_follow_increase_is_rate_limited():
  # rate passed explicitly so this keeps testing the limiter, not whatever the default is
  assert limit_t_follow_increase(1.10, 1.75, 0.05, 0.10) == pytest.approx(1.105)
  assert limit_t_follow_increase(1.10, 1.75, 0.05, 0.50) == pytest.approx(1.125)


def test_default_rise_rate_crosses_the_range_in_a_few_seconds():
  table = gap_t_follow_table(0)
  assert (table[7] - table[1]) / T_FOLLOW_RISE_RATE < 4.0, "gap 1 to 7 should not take 9 seconds"


def test_launch_accel_default_is_unchanged():
  assert get_max_accel(0.0) == pytest.approx(A_CRUISE_MAX_VALS[0])
  for v, expected in zip(A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS, strict=True):
    assert get_max_accel(v) == pytest.approx(expected)


def test_launch_accel_raises_only_the_low_speed_end():
  raised = 2.0
  assert get_max_accel(0.0, raised) == pytest.approx(raised)
  assert get_max_accel(10.0, raised) > get_max_accel(10.0), "sub-10m/s scales with it"
  # the highway end is somebody else's tuning
  assert get_max_accel(25.0, raised) == pytest.approx(A_CRUISE_MAX_VALS[2])
  assert get_max_accel(40.0, raised) == pytest.approx(A_CRUISE_MAX_VALS[3])


def test_launch_accel_never_exceeds_what_panda_allows():
  # tesla_legacy.h caps DAS_control at 2.0 m/s^2; asking for more just gets the command rejected
  for v in (0.0, 5.0, 10.0, 25.0, 40.0):
    assert get_max_accel(v, 5.0) <= ACCEL_MAX + 1e-9


def test_launch_accel_leaves_braking_alone():
  # the complaint was acceleration only; ACCEL_MIN is not a function of this knob
  assert get_max_accel(0.0, 2.0) > get_max_accel(0.0)
  assert ACCEL_MIN == pytest.approx(-3.5)


def test_gap_range_is_worth_turning_the_knob():
  """The old table moved 18m between the extremes at 100km/h, under 3m a step, which is why
  running the knob end to end barely changed anything."""
  table = gap_t_follow_table(0)
  v = 100 / 3.6
  assert (table[7] - table[1]) * v > 25, "1 to 7 was only worth 18m before"

  steps = [(table[g + 1] - table[g]) * v for g in range(1, 7)]
  assert min(steps) > 4.0, "every step should be a car length or more at 100km/h"
  assert max(steps) - min(steps) < 0.5, "evenly spaced, as chosen"


def test_gap_7_is_unchanged():
  # the far end was the one setting that already felt right
  assert gap_t_follow_table(0)[7] == pytest.approx(1.75)


@pytest.mark.parametrize("profile", list(GAP_PROFILES))
def test_no_profile_goes_below_the_floor(profile):
  # 'closer' and 'wider' both shift gap 1 down, and the base is already deliberately close
  assert min(gap_t_follow_table(profile).values()) >= MIN_T_FOLLOW


def test_profiles_still_move_the_table():
  assert gap_t_follow_table(2)[4] > gap_t_follow_table(0)[4], "'further' must still be further"


def test_no_gap_falls_back_to_personality():
  mpc = LongitudinalMpc()
  assert mpc.update_t_follow(log.LongitudinalPersonality.standard, 0) == pytest.approx(1.45)
  assert mpc.update_t_follow(log.LongitudinalPersonality.relaxed, 0) == pytest.approx(1.75)


def test_gap_slew_survives_solver_reset():
  # A solver reset must not re-arm the "first valid gap applies immediately" path, otherwise a
  # pending tFollow increase lands in one step and brakes the car.
  mpc = LongitudinalMpc()
  assert mpc.update_t_follow(log.LongitudinalPersonality.standard, 1) == pytest.approx(0.80)
  step = T_FOLLOW_RISE_RATE * mpc.dt
  assert mpc.update_t_follow(log.LongitudinalPersonality.standard, 7) == pytest.approx(0.80 + step)

  mpc.reset()
  assert mpc.update_t_follow(log.LongitudinalPersonality.standard, 7) == pytest.approx(0.80 + 2 * step)


def run_following_distance_simulation(v_lead, t_end=100.0, e2e=False, personality=0):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=float(v_lead),
    lead_relevancy=True,
    initial_distance_lead=100,
    speed_lead_values=[v_lead],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1,2] - output[-1,1]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      [log.LongitudinalPersonality.relaxed, # personality
                       log.LongitudinalPersonality.standard,
                       log.LongitudinalPersonality.aggressive],
                      [0,10,35])) # speed
class TestFollowingDistance:
  def test_following_distance(self):
    v_lead = float(self.speed)
    simulation_steady_state = run_following_distance_simulation(v_lead, e2e=self.e2e, personality=self.personality)
    correct_steady_state = desired_follow_distance(v_lead, v_lead, get_T_FOLLOW(self.personality))
    err_ratio = 0.2 if self.e2e else 0.1
    abs_err_margin = 0.5 if v_lead > 0.0 else 1.15
    assert simulation_steady_state == pytest.approx(correct_steady_state, abs=err_ratio * correct_steady_state + abs_err_margin)
