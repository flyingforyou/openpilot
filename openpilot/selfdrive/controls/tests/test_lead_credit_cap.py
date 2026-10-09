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


# --- brake follow / buffer use (experiment knobs) ---
from openpilot.selfdrive.controls.lib import carrot_t_follow as ct  # noqa: E402


def test_brake_follow_off_by_default():
  assert ct.lead_brake_follow(0.5, True, 20.0, -2.0, 10.0, 9.0) == 0.5


def test_brake_follow_brakes_at_k_times_lead_decel(monkeypatch):
  monkeypatch.setattr(ct, "LEAD_BRAKE_FOLLOW", (0.7, -0.5, 40.0))
  assert abs(ct.lead_brake_follow(0.5, True, 20.0, -2.0, 10.0, 9.0) - (-1.4)) < 1e-9
  assert ct.lead_brake_follow(-3.0, True, 20.0, -2.0, 10.0, 9.0) == -3.0     # never weakens braking
  assert ct.lead_brake_follow(0.5, True, 60.0, -2.0, 10.0, 9.0) == 0.5       # far lead
  assert ct.lead_brake_follow(0.5, True, 20.0, -0.3, 10.0, 9.0) == 0.5       # lead not really braking
  assert ct.lead_brake_follow(0.5, True, 20.0, -2.0, 10.0, 13.0) == 0.5      # lead pulling away


def test_buffer_scales_only_while_lead_brakes(monkeypatch):
  monkeypatch.setattr(ct, "BRAKE_TF_SCALE", ((-2.5, -0.5), (0.5, 1.0)))
  assert ct.brake_t_follow_scale(1.0, True, 0.0) == 1.0
  assert abs(ct.brake_t_follow_scale(1.0, True, -2.5) - 0.5) < 1e-9
  assert ct.brake_t_follow_scale(1.0, False, -2.5) == 1.0


def test_buffer_brake_uses_the_gap():
  far = ct.buffer_brake_accel(30.0, -2.5, 13.5, 13.0, 4.5, 0.25)
  near = ct.buffer_brake_accel(10.0, -2.5, 13.5, 13.0, 4.5, 0.25)
  assert near < far < 0.0                       # less room, firmer start


def test_buffer_brake_stops_at_the_standstill_gap():
  """Braking at the returned rate, with the lead braking at a_lead to a stop, ends standstill behind."""
  d, al, v, vl, s0, tr = 16.0, -2.5, 13.5, 13.0, 4.5, 0.25
  a = ct.buffer_brake_accel(d, al, v, vl, s0, tr)
  ego_travel = v * tr + v * v / (2 * -a)
  lead_travel = vl * vl / (2 * -al)
  assert abs((d + lead_travel - ego_travel) - s0) < 1e-6


def test_buffer_mode_only_brakes_harder(monkeypatch):
  monkeypatch.setattr(ct, "LEAD_BRAKE_BUFFER", (-0.5, 4.5, 0.25, 60.0))
  assert ct.lead_brake_follow(-3.0, True, 30.0, -2.5, 13.5, 13.0) == -3.0
  assert ct.lead_brake_follow(0.4, True, 30.0, -2.5, 13.5, 13.0) < 0.0
  assert ct.lead_brake_follow(0.4, True, 30.0, -0.2, 13.5, 13.0) == 0.4


def test_brake_follow_smoothing(monkeypatch):
  monkeypatch.setattr(ct, "LEAD_BRAKE_BUFFER", (-0.5, 4.5, 0.25, 60.0))
  bf = ct.BrakeFollow(0.05)
  # no lead braking: passes the planner straight through, including fast rises
  assert bf.update(0.0, True, 20.0, 0.0, 13.0, 13.0) == 0.0
  assert bf.update(1.5, True, 20.0, 0.0, 13.0, 13.0) == 1.5
  # lead brakes hard: the extra braking deepens at no more than BF_RATE_IN
  outs = [bf.update(0.5, True, 16.0, -3.0, 13.5, 13.0) for _ in range(40)]
  steps = [b - a for a, b in zip(outs, outs[1:], strict=False)]
  assert min(outs) < -1.0
  assert min(steps) >= -ct.BF_RATE_IN * 0.05 - 1e-9
  # lead stops braking: lets go no faster than BF_RATE_OUT
  rel = [bf.update(0.5, True, 16.0, 0.0, 13.5, 13.0) for _ in range(60)]
  steps = [b - a for a, b in zip([outs[-1]] + rel, rel, strict=False)]
  assert max(steps) <= ct.BF_RATE_OUT * 0.05 + 1e-9
  assert rel[-1] == 0.5
  # never weaker than the planner
  assert bf.update(-3.0, True, 16.0, 0.0, 13.5, 13.0) == -3.0


def test_release_on_opening(monkeypatch):
  monkeypatch.setattr(ct, "RELEASE_ON_OPENING", (0.3, 4.0, 0.0))
  r = ct.ReleaseOnOpening(0.05)
  r.update(-2.5, True, 12.0, 6.0, 6.0, 8.0)
  out = r.update(-2.5, True, 12.0, 6.0, 7.0, 8.0)          # lead pulling away, gap above target
  assert abs(out - (-2.5 + 4.0 * 0.05)) < 1e-9
  assert r.update(-2.5, True, 7.0, 6.0, 7.0, 8.0) == -2.5   # gap below target: planner keeps its brake
  assert r.update(-2.5, True, 12.0, 7.0, 6.0, 8.0) == -2.5  # still closing: no release


def test_brake_onset_limit_only_with_time_to_spare():
  """BrakeOnsetJerk: a step in braking comes in at the set rate while the nearest lead is >= 3 s away,
  and goes straight through when it is closer in time (a real emergency is never rate-limited)."""
  import openpilot.selfdrive.controls.lib.carrot_t_follow as ct
  old = ct.BRAKE_ONSET_JERK
  try:
    ct.BRAKE_ONSET_JERK = 2.0
    lim = ct.BrakeOnsetLimiter(0.05)
    lim.update(0.0, True, 40.0, 0.0, 20.0, 18.0, 4.5)
    # plenty of time (40 m, closing 2 m/s -> 20 s): a -1.5 step comes in at ~0.1 per frame, never less
    # than what stopping behind the lead needs (the floor, ~-0.12 here)
    out = lim.update(-1.5, True, 40.0, 0.0, 20.0, 18.0, 4.5, ttc=20.0, d_min=40.0)
    assert -0.2 < out < -0.05
    lim2 = ct.BrakeOnsetLimiter(0.05)
    lim2.update(0.0, True, 20.0, 0.0, 20.0, 10.0, 4.5)
    # 2 s to contact: no limit
    assert lim2.update(-2.5, True, 20.0, 0.0, 20.0, 10.0, 4.5, ttc=2.0, d_min=20.0) == -2.5
    ct.BRAKE_ONSET_JERK = None
    lim3 = ct.BrakeOnsetLimiter(0.05)
    lim3.update(0.0, True, 40.0, 0.0, 20.0, 18.0, 4.5)
    assert lim3.update(-1.5, True, 40.0, 0.0, 20.0, 18.0, 4.5, ttc=20.0, d_min=40.0) == -1.5
  finally:
    ct.BRAKE_ONSET_JERK = old


def test_cutin_soft_cap_holds_acceleration_for_a_merging_car():
  """CutInSoftCap mode 1: a side-lane car closing 0.7 m on our centreline within 2.8 m, nearer than the
  lead and within headway, stops acceleration (ramped) for CUTIN_HOLD; braking is never reduced."""
  from types import SimpleNamespace as NS
  import openpilot.selfdrive.controls.lib.carrot_t_follow as ct
  cap = ct.CutInSoftCap(0.05)
  out = None
  for i in range(30):                       # 1.5 s: dy 3.5 -> 2.45, dx 25 m, slower than us
    dy = 3.5 - 0.035 * i
    out = cap.update(1.0, 1, [NS(group=2, objId=7, dx=25.0, dy=dy, vxRel=-2.0)], True, 60.0, 20.0)
  assert cap.active and out < 1.0
  for _ in range(20):
    out = cap.update(1.0, 1, [], True, 60.0, 20.0)
  assert out == 0.0                         # ramped down to no acceleration
  assert cap.update(-1.5, 1, [], True, 60.0, 20.0) == -1.5   # braking passes through
  # a merging car that is faster than us and far off does not trigger it
  cap2 = ct.CutInSoftCap(0.05)
  for i in range(30):
    cap2.update(1.0, 1, [NS(group=2, objId=8, dx=55.0, dy=3.5 - 0.035 * i, vxRel=+3.0)], False, 0.0, 15.0)
  assert not cap2.active
