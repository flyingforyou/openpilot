"""A faster lead is credited as no faster than us, so the desired gap cannot collapse."""
import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.carrot_t_follow import lead_speed_for_credit


def gap(v_ego, v_lead, tf=0.46, b1=2.16, b2=2.374, sd=4.5):
  return v_ego**2 / (2*b1) + tf*v_ego + sd - v_lead**2 / (2*b2)


def test_the_logged_collapse_is_prevented():
  """Ego 19 km/h, lead 29 km/h: uncapped target ~0 m, capped keeps the equal-speed gap."""
  v_e, v_l = 19.1 / 3.6, 29.0 / 3.6
  assert gap(v_e, v_l) < 1.0
  capped = gap(v_e, lead_speed_for_credit(v_l, v_e, True))
  assert capped == pytest.approx(gap(v_e, v_e))
  assert capped > 5.0


def test_slower_lead_untouched():
  for v_l in (0.0, 3.0, 8.0):
    assert lead_speed_for_credit(v_l, 10.0, True) == v_l


def test_only_ever_raises_the_target():
  for v_e in (2.0, 8.0, 20.0):
    for v_l in (0.0, 5.0, 10.0, 25.0, 35.0):
      assert gap(v_e, lead_speed_for_credit(v_l, v_e, True)) >= gap(v_e, v_l) - 1e-9


def test_trajectory_arrays():
  traj = np.array([5.0, 8.0, 12.0, 15.0])
  np.testing.assert_allclose(lead_speed_for_credit(traj, 10.0, True), [5.0, 8.0, 10.0, 10.0])


def test_disabled_passes_through():
  traj = np.array([5.0, 12.0])
  np.testing.assert_allclose(lead_speed_for_credit(traj, 10.0, False), traj)


def test_negative_ego_speed_clamped():
  assert lead_speed_for_credit(5.0, -0.1, True) == 0.0
