"""The slow-traffic smoothness floor.

carrot ties jerk_factor to the gap position alone, so asking to follow closely also asks the MPC
to change acceleration cheaply. This floor separates the two below 40 km/h.
"""
import pytest

from openpilot.selfdrive.controls.lib.carrot_t_follow import (
  LOW_SPEED_JERK_BP,
  low_speed_jerk_factor,
)

GAP2 = 0.567      # what the gap table gives at position 2, the setting in the 09-09 logs
GAP7 = 1.0


class TestFloor:
  def test_full_smoothness_at_a_crawl(self):
    assert low_speed_jerk_factor(GAP2, 0.0, 1.0) == pytest.approx(1.0)
    assert low_speed_jerk_factor(GAP2, LOW_SPEED_JERK_BP[0], 1.0) == pytest.approx(1.0)

  def test_faded_out_by_the_upper_breakpoint(self):
    assert low_speed_jerk_factor(GAP2, LOW_SPEED_JERK_BP[1], 1.0) == pytest.approx(GAP2)
    assert low_speed_jerk_factor(GAP2, 80.0, 1.0) == pytest.approx(GAP2)

  def test_monotone_between(self):
    vals = [low_speed_jerk_factor(GAP2, k, 1.0) for k in (0, 10, 20, 30, 40, 60)]
    assert vals == sorted(vals, reverse=True)

  def test_highway_is_untouched(self):
    """The whole point of the fade: nothing above the breakpoint may move."""
    for k in (40.0, 55.0, 88.0, 120.0):
      for jf in (0.5, 0.567, 0.7, 1.0):
        assert low_speed_jerk_factor(jf, k, 1.0) == jf


class TestNeverLowers:
  def test_a_smooth_gap_setting_keeps_what_it_asked_for(self):
    for k in (0.0, 15.0, 30.0):
      assert low_speed_jerk_factor(GAP7, k, 1.0) == GAP7

  def test_floor_below_the_gap_value_does_nothing(self):
    assert low_speed_jerk_factor(0.8, 0.0, 0.6) == 0.8

  def test_never_returns_less_than_given(self):
    for jf in (0.4, 0.5, 0.567, 0.7, 1.0):
      for k in (0.0, 5.0, 20.0, 39.0, 41.0, 100.0):
        assert low_speed_jerk_factor(jf, k, 1.0) >= jf


class TestDisabled:
  def test_zero_is_off(self):
    assert low_speed_jerk_factor(GAP2, 0.0, 0.0) == GAP2

  def test_negative_is_off(self):
    assert low_speed_jerk_factor(GAP2, 0.0, -1.0) == GAP2


def test_partial_floor_is_honoured():
  """A user dialling the knob to 0.8 gets 0.8 at rest, not 1.0."""
  assert low_speed_jerk_factor(GAP2, 0.0, 0.8) == pytest.approx(0.8)
