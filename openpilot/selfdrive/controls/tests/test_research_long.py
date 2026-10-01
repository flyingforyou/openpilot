"""The literature path: IDM + CAH (Kesting 2010) with FollowerStopper-style flow smoothing."""
import pytest

from openpilot.selfdrive.controls.lib.research_long import (
  IDM_S0, IDM_T, ResearchLongitudinal, flow_v0, idm_cah_accel,
)

A = 1.5


def test_equilibrium_gap_holds_speed():
  v = 10.0
  s = IDM_S0 + v * IDM_T
  assert abs(idm_cah_accel(v, v, 0.0, s, 30.0, A)) < 0.1


def test_faster_lead_does_not_run_to_the_ceiling():
  """00000178 seg 6: ego 19, lead 29 km/h, 12 m back. Bare IDM still accelerates hard (the gap is
  longer than its target); it is the flow smoothing that holds it back in stop-and-go."""
  v, vl = 19.1 / 3.6, 29.0 / 3.6
  bare = idm_cah_accel(v, vl, 1.5, 12.0, 30.0, A)
  smoothed = idm_cah_accel(v, vl, 1.5, 12.0, flow_v0(6.0, 30.0), A)
  assert bare > 1.0
  assert 0.0 < smoothed < bare


def test_stopped_lead_close_brakes_hard():
  assert idm_cah_accel(10.0, 0.0, 0.0, 15.0, 30.0, A) < -2.0


def test_cah_softens_a_pulling_away_cut_in():
  """Short gap but the lead is faster and accelerating: CAH keeps it from a hard stab."""
  assert idm_cah_accel(15.0, 17.0, 0.5, 6.0, 30.0, A) > -1.5


def test_never_exceeds_accel_ceiling():
  for v in (0.0, 5.0, 15.0, 25.0):
    for s in (5.0, 20.0, 80.0):
      assert idm_cah_accel(v, v + 5, 1.0, s, 35.0, A) <= A + 1e-9


def test_standing_start_behind_a_departing_lead():
  assert idm_cah_accel(0.0, 2.0, 1.0, 8.0, 5.0, A) > 0.0


def test_flow_smoothing_fades_out_on_open_road():
  assert flow_v0(8.0, 30.0) == pytest.approx(8.0 + 1.5)
  assert flow_v0(80 / 3.6, 30.0) == pytest.approx(30.0)
  assert flow_v0(5.0, 4.0) == pytest.approx(4.0)       # never above cruise


def test_over_flow_speed_only_coasts():
  """v > v0 must not brake harder than the coast floor on its own (no lead influence)."""
  assert idm_cah_accel(12.0, 12.0, 0.0, 500.0, 10.0, A) >= -0.5 - 1e-9


def test_no_lead_hands_back():
  r = ResearchLongitudinal(0.05)
  assert r.update(10.0, False, 0.0, 0.0, 0.0, 25.0, A) is None
  assert r.update(10.0, True, 20.0, 10.0, 0.0, 0.0, A) is None     # stopping: MPC's job


# --- IIDM and the hand-over blend ------------------------------------------------------------

from openpilot.selfdrive.controls.lib.research_long import BLEND_TIME, iidm_accel  # noqa: E402


def test_iidm_equilibrium_gap_is_s_star_even_near_v0():
  """The defect plain IDM has and IIDM fixes: at s = s*, zero accel whatever v0 is."""
  v = 27.0
  s_star = IDM_S0 + v * IDM_T
  for v0 in (27.5, 28.0, 30.0, 40.0):
    assert abs(iidm_accel(v, 0.0, s_star, v0)[0]) < 1e-9, v0


def test_plain_idm_would_have_drifted_back():
  """Guard the premise: plain IDM at the same point brakes, i.e. wants a bigger gap."""
  v, v0 = 27.0, 28.0
  s_star = IDM_S0 + v * IDM_T
  plain = 1.4 * (1 - (v / v0) ** 4) - 1.4 * (s_star / s_star) ** 2
  assert plain < -0.1


def test_iidm_closes_a_large_gap_below_v0():
  assert iidm_accel(20.0, 0.0, 80.0, 30.0)[0] > 0.5


def test_blend_never_steps_the_command():
  r = ResearchLongitudinal(0.05)
  out = [r.blend(None, 1.0)]
  for _ in range(20):
    out.append(r.blend(-0.5, 1.0))
  for _ in range(20):
    out.append(r.blend(None, 1.0))
  steps = [abs(b - a) for a, b in zip(out, out[1:])]
  assert max(steps) <= 1.5 * 0.05 / BLEND_TIME + 1e-9


# --- EIDM jerk limit (Salles 2020 eq. 20) -----------------------------------------------------

from openpilot.selfdrive.controls.lib.research_long import EIDM_JMAX, IDM_A, eidm_limit_ratio  # noqa: E402


def test_eidm_ratio_falls_no_faster_than_jmax():
  dt = 0.05
  z = eidm_limit_ratio(0.5, 2.0, dt, IDM_A)
  assert z * z == pytest.approx(4.0 - dt * EIDM_JMAX / IDM_A)


def test_eidm_rising_ratio_passes_straight_through():
  """Gap closing (cut-in) must be felt at once: braking is never delayed."""
  assert eidm_limit_ratio(3.0, 1.0, 0.05, IDM_A) == 3.0


def test_the_seg18_flip_is_damped():
  """Lead track alternating 24.6 m / 36.2 m each frame at 106 km/h: the output must not flip."""
  r = ResearchLongitudinal(0.05)
  outs = []
  for i in range(20):
    near = i % 2 == 0
    outs.append(r.update(29.4, True, 24.6 if near else 36.2, (99.4 if near else 107.9) / 3.6,
                         0.0, 33.6, 1.5))
  steps = [abs(b - a) for a, b in zip(outs[2:], outs[3:])]
  assert max(steps) < 0.8, (max(steps), outs[:6])


# --- LongResearchMode 1: time-headway FollowerStopper (CIRCLES reference) ---
from openpilot.selfdrive.controls.lib.research_long import (  # noqa: E402
  fs_bands, fs_safe_velocity, follower_stopper_accel,
)

STOP = 4.5


def _s_eq(v, t_follow):
  return STOP + t_follow * v


@pytest.mark.parametrize("t_follow", [0.46, 0.88, 1.30])
@pytest.mark.parametrize("v", [5.0, 15.0, 28.0])
def test_fs_holds_speed_at_every_gap_positions_equilibrium(t_follow, v):
  s = _s_eq(v, t_follow)
  assert abs(follower_stopper_accel(v, v, s, 33.0, s, STOP)) < 1e-6


def test_fs_bands_keep_the_papers_ratios():
  v = 20.0
  s_eq = _s_eq(v, 0.88)
  dx1, dx2, dx3 = fs_bands(v, v, s_eq, STOP)
  assert dx2 == pytest.approx(s_eq)
  assert (dx1 - STOP * 4.5 / 5.25) / (dx2 - STOP) == pytest.approx(0.4 / 0.6)
  assert (dx3 - STOP * 6.0 / 5.25) / (dx2 - STOP) == pytest.approx(0.8 / 0.6)


def test_fs_closing_speed_widens_the_bands():
  s_eq = _s_eq(20.0, 0.88)
  assert all(c > o for c, o in zip(fs_bands(20.0, 15.0, s_eq, STOP), fs_bands(20.0, 20.0, s_eq, STOP), strict=True))


def test_fs_wider_gap_position_brakes_earlier():
  v, s = 20.0, 25.0
  assert follower_stopper_accel(v, v, s, 33.0, _s_eq(v, 1.30), STOP) < follower_stopper_accel(v, v, s, 33.0, _s_eq(v, 0.46), STOP)


def test_fs_stopped_lead_close_brakes_at_the_limit():
  assert follower_stopper_accel(10.0, 0.0, 15.0, 20.0, _s_eq(10.0, 0.88), STOP) == pytest.approx(-3.0)


def test_fs_failsafe_stops_short_of_the_lead():
  v = fs_safe_velocity(0.0, 20.0)
  # reaction then full braking fits in 20 m less the 2.5 m min gap
  assert v * 0.1 + v * v / (2 * 3.0) == pytest.approx(20.0 - 2.5)
  assert fs_safe_velocity(0.0, 2.0) == 0.0


def test_fs_never_exceeds_the_ceiling():
  assert follower_stopper_accel(0.0, 20.0, 80.0, 30.0, STOP, STOP, a_max=0.8) == pytest.approx(0.8)


def test_mode_1_uses_the_follower_stopper():
  r = ResearchLongitudinal(0.05)
  v = 15.0
  s_eq = _s_eq(v, 0.88)
  assert abs(r.update(v, True, s_eq, v, 0.0, 30.0, 2.0, mode=1, s_eq=s_eq, stop_distance=STOP)) < 1e-6
  assert r.update(v, True, s_eq - 5, v, 0.0, 30.0, 2.0, mode=1, s_eq=s_eq, stop_distance=STOP) < -0.5


# --- mode 0 follows the gap stalk ---
from openpilot.selfdrive.controls.lib.research_long import gap_idm_params  # noqa: E402


@pytest.mark.parametrize("t_follow", [0.46, 0.88, 1.30])
@pytest.mark.parametrize("v", [3.0, 15.0, 28.0])
def test_iidm_holds_speed_at_every_gap_positions_equilibrium(t_follow, v):
  r = ResearchLongitudinal(0.05)
  s_eq = STOP + t_follow * v
  a = r.update(v, True, s_eq, v, 0.0, 33.0, 2.0, mode=0, s_eq=s_eq, stop_distance=STOP)
  assert abs(a) < 0.05


def test_iidm_gap_params_recover_t_follow():
  assert gap_idm_params(20.0, STOP + 0.88 * 20.0, STOP) == pytest.approx((0.88, STOP))


def test_iidm_wider_gap_position_brakes_earlier():
  v, s = 20.0, 25.0
  tight = ResearchLongitudinal(0.05).update(v, True, s, v, 0.0, 33.0, 2.0, s_eq=STOP + 0.46 * v, stop_distance=STOP)
  loose = ResearchLongitudinal(0.05).update(v, True, s, v, 0.0, 33.0, 2.0, s_eq=STOP + 1.30 * v, stop_distance=STOP)
  assert loose < -0.5 < tight


# --- leadTwo (cut-in / target lane lead) is respected too ---
from openpilot.selfdrive.controls.lib.research_long import more_binding  # noqa: E402


def test_more_binding_takes_the_stricter_lead():
  assert more_binding(0.4, -1.2) == -1.2
  assert more_binding(None, -0.3) == -0.3
  assert more_binding(0.5, None) == 0.5
  assert more_binding(None, None) is None


def test_a_merging_car_closer_than_the_lead_brakes_the_output():
  v = 25.0
  s_eq = STOP + 0.60 * v
  lead_one = ResearchLongitudinal(0.05).update(v, True, s_eq + 5, v, 0.0, 30.0, 2.0, s_eq=s_eq, stop_distance=STOP)
  cut_in = ResearchLongitudinal(0.05).update(v, True, 12.0, v - 2.0, 0.0, 30.0, 2.0, s_eq=s_eq, stop_distance=STOP)
  assert lead_one > 0.0 > cut_in
  assert more_binding(lead_one, cut_in) == cut_in


# --- the CAH blend's reach below CAH is CAH_BLEND_B, not b ---
from openpilot.selfdrive.controls.lib.research_long import CAH_BLEND_B, cah_accel  # noqa: E402


def test_braking_stays_within_blend_of_cah():
  """00000178 replay, 60 km/h at 15.9 m behind a lead 5 km/h slower and braking -1.2: IIDM alone
  asks -2.6, CAH says -1.24 is enough. The result may go below CAH by at most CAH_BLEND_B."""
  v, vl, s, al = 58.7 / 3.6, 53.4 / 3.6, 15.9, -1.2
  s_eq = STOP + 0.93 * v
  out = ResearchLongitudinal(0.05).update(v, True, s, vl, al, 33.0, 2.0, s_eq=s_eq, stop_distance=STOP)
  a_cah = cah_accel(v, vl, al, s)
  assert a_cah - CAH_BLEND_B - 0.05 <= out < a_cah


def test_cah_stopped_lead_needs_stopping_within_the_gap():
  # the 0/0 case: lead stopped, zero acceleration
  assert cah_accel(10.0, 0.0, 0.0, 15.0) == pytest.approx(-10.0 ** 2 / (2 * 15.0))


def test_hard_cah_keeps_the_papers_margin():
  """Closing on a stopped car the blend must not shave the margin: the result stays at least b below
  CAH's contact-limited deceleration, as in Kesting's own eq. (2.4)."""
  out = idm_cah_accel(10.0, 0.0, 0.0, 15.0, 30.0, 1.5)
  assert out < cah_accel(10.0, 0.0, 0.0, 15.0) - 1.5


def test_blend_relaxes_only_with_time_to_spare():
  """Same IIDM-vs-CAH gap, different time to contact: relaxed at 10 s, the paper's b at 2 s."""
  v, vl, al = 20.0, 18.5, -0.5
  far = idm_cah_accel(v, vl, al, 15.0, 30.0, 1.5)          # TTC 10 s
  near = idm_cah_accel(v, vl - 4.0, al, 11.0, 30.0, 1.5)   # TTC ~2 s
  assert far >= cah_accel(v, vl, al, 15.0) - CAH_BLEND_B - 0.05
  assert near < cah_accel(v, vl - 4.0, al, 11.0) - CAH_BLEND_B - 0.3


# --- output jerk limit ---
from openpilot.selfdrive.controls.lib.research_long import JerkLimiter  # noqa: E402


# a_floor is CAH's acceleration; a positive value means collision avoidance needs no braking at all
def test_jerk_limiter_rate_limits_the_brake_side():
  j = JerkLimiter(0.05)
  j.limit(0.0, 1.0, 25.0)
  assert j.limit(-2.0, 1.0, 25.0) == pytest.approx(-2.5 * 0.05)    # 2.5 m/s^3 above 20 m/s


def test_jerk_limiter_never_brakes_less_than_cah():
  j = JerkLimiter(0.05)
  j.limit(0.0, 1.0, 25.0)
  assert j.limit(-2.0, -1.0, 25.0) == pytest.approx(-1.0)


def test_jerk_limiter_lets_braking_through_when_time_is_short():
  j = JerkLimiter(0.05)
  j.limit(0.0, 1.0, 25.0)
  assert j.limit(-2.0, 1.0, 25.0, ttc=2.0) == pytest.approx(-2.0)


def test_jerk_limiter_is_looser_at_low_speed():
  j = JerkLimiter(0.05)
  j.limit(0.0, 1.0, 2.0)
  assert j.limit(-2.0, 1.0, 2.0) == pytest.approx(-5.0 * 0.05)
