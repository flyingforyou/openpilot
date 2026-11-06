import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.turn_load_guard import LAT_BUDGET, MIN_ACCEL, TurnLoadGuard, required_decel

T = np.linspace(0.0, 10.0, 13)


def plan(v):
  return T, np.full_like(T, v), v * T


def model(curvature, v=10.0):
  t = np.linspace(0.0, 10.0, 33)
  return v * t, np.full_like(t, v), np.full_like(t, curvature * v)


def test_a_bend_within_budget_needs_nothing():
  v = 10.0
  k = 0.9 * LAT_BUDGET / v**2
  assert required_decel(*plan(v), *model(k), 0.0) == 0.0


def test_a_bend_over_budget_needs_deceleration():
  v = 11.0
  k = 1.4 * LAT_BUDGET / v**2
  assert required_decel(*plan(v), *model(k), 0.0) > 0.5


def test_bank_toward_the_turn_allows_more():
  v = 11.0
  k = 1.1 * LAT_BUDGET / v**2
  flat = required_decel(*plan(v), *model(k), 0.0)
  banked = required_decel(*plan(v), *model(k), 0.05)
  assert banked < flat


def test_guard_only_lowers_and_is_jerk_limited():
  g = TurnLoadGuard(DT_MDL)
  outs = [g.update(0.5, 3.0) for _ in range(40)]
  assert all(o <= 0.5 for o in outs)
  assert outs[1] - outs[0] >= -2.0 * DT_MDL * 3      # no step: filtered and rate-limited
  assert min(outs) >= MIN_ACCEL


def test_no_need_leaves_the_command_alone():
  g = TurnLoadGuard(DT_MDL)
  assert g.update(0.7, 0.0) == 0.7
  assert g.update(-1.2, 0.0) == -1.2
