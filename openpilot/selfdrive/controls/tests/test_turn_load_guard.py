import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.turn_load_guard import LAT_BUDGET, MIN_ACCEL, TurnLoadGuard, required_decel

T = np.linspace(0.0, 10.0, 13)


def plan(v, a_cmd=0.0):
  return T, np.full_like(T, v), v, a_cmd


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


def test_a_plan_running_ahead_of_the_car_is_not_believed():
  # the MPC keeps the speed the guard has already taken off; what counts is the car's speed now
  v, k = 10.0, 1.0 * LAT_BUDGET / 11.0**2
  t, _, _, _ = plan(v)
  ahead_plan = (t, np.full_like(t, 14.0), v, 0.0)
  assert required_decel(*ahead_plan, *model(k), 0.0) == 0.0


def test_planned_braking_still_counts():
  v, k = 12.0, 1.2 * LAT_BUDGET / 12.0**2
  t, _, _, _ = plan(v)
  braking = (t, np.maximum(12.0 - 1.0 * t, 5.0), v, 0.0)
  assert required_decel(*braking, *model(k), 0.0) < required_decel(*plan(v), *model(k), 0.0)


def test_correction_does_not_wind_up_past_the_floor():
  g = TurnLoadGuard(DT_MDL)
  for _ in range(100):
    g.update(0.0, 8.0)
  assert g.correction <= 0.0 - MIN_ACCEL + 1e-6
  # once the need is gone it lets go within a short ramp, not seconds of extra braking
  outs = [g.update(0.0, 0.0) for _ in range(int(1.2 / DT_MDL))]
  assert outs[-1] > -0.05
