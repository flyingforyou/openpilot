import pytest
import itertools
from openpilot.common.parameterized import parameterized_class

from cereal import log

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (get_safe_obstacle_distance, get_stopped_equivalence_factor,
                                                                          get_T_FOLLOW, limit_t_follow_increase)
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)


@pytest.mark.parametrize("gap, expected", [
  (1, 1.10),
  (2, 1.20),
  (3, 1.30),
  (4, 1.40),
  (5, 1.50),
  (6, 1.60),
  (7, 1.75),
])
def test_tesla_gap_t_follow(gap, expected):
  assert get_T_FOLLOW(log.LongitudinalPersonality.standard, gap_adjust=gap) == pytest.approx(expected)


def test_invalid_gap_uses_personality():
  assert get_T_FOLLOW(log.LongitudinalPersonality.standard, gap_adjust=0) == pytest.approx(1.45)


def test_t_follow_decrease_is_immediate():
  assert limit_t_follow_increase(1.75, 1.10, 0.05) == pytest.approx(1.10)


def test_t_follow_increase_is_rate_limited():
  assert limit_t_follow_increase(1.10, 1.75, 0.05) == pytest.approx(1.105)


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
