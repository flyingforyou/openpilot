"""The literature path: IDM + CAH (Kesting 2010) with FollowerStopper-style flow smoothing."""
import pytest

from openpilot.selfdrive.controls.lib.research_long import (
  IDM_S0, IDM_T, MPC_OVERRIDE_BELOW, ResearchLongitudinal, combine_with_mpc, flow_v0, idm_cah_accel,
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
  assert 0.0 < smoothed < bare - 0.3


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


def test_mpc_keeps_the_last_word_on_hard_braking():
  assert combine_with_mpc(-0.5, -2.8) == -2.8
  assert combine_with_mpc(-0.5, MPC_OVERRIDE_BELOW + 0.1) == -0.5
  assert combine_with_mpc(0.8, 1.6) == 0.8


def test_no_lead_hands_back():
  r = ResearchLongitudinal(0.05)
  assert r.update(10.0, False, 0.0, 0.0, 0.0, 25.0, A) is None
  assert r.update(10.0, True, 20.0, 10.0, 0.0, 0.0, A) is None     # stopping: MPC's job
