"""The optional lateral smoothing (LatSmoothSec) ported from CarrotPilot.

The port exists to kill the ~2 Hz wheel shake that shows up rolling to a stop, so what these
lock down is the two things that made carrot's own copy not do anything: the array index into
plan_stds, and the off-by-default contract.
"""
import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.drive_helpers import (
  LAT_SMOOTH_SECONDS_MAX,
  LAT_SMOOTH_T_IDX_1S,
  get_lat_smooth_seconds_dynamic,
  smooth_value,
)

PLAN_IDX_N = 33
PLAN_WIDTH = 15
CARROT_BASE = 0.13


def _model_output(y_std_1s: float):
  """A plan_stds shaped like the real one: (batch, IDX_N, PLAN_WIDTH)."""
  stds = np.zeros((1, PLAN_IDX_N, PLAN_WIDTH), dtype=np.float32)
  stds[0, LAT_SMOOTH_T_IDX_1S, 1] = y_std_1s   # Plan.POSITION is x,y,z at 0,1,2 -> y is 1
  return {'plan_stds': stds}


class TestLatSmoothDynamic:
  def test_off_by_default(self):
    # base 0 is the shipped default: the feature must be completely inert, not "a little bit on"
    assert get_lat_smooth_seconds_dynamic(_model_output(0.5), 0.0) == (0.0, 0.0)

  def test_confident_model_gets_base_only(self):
    tau, y_std = get_lat_smooth_seconds_dynamic(_model_output(0.10), CARROT_BASE)
    assert tau == CARROT_BASE
    assert y_std == np.float32(0.10)

  def test_unsure_model_gets_extra(self):
    # at/after the top of the ramp the extra is 2x base
    tau, _ = get_lat_smooth_seconds_dynamic(_model_output(0.30), CARROT_BASE)
    assert tau == CARROT_BASE * 3.0

  def test_extra_ramps_in_between(self):
    tau, _ = get_lat_smooth_seconds_dynamic(_model_output(0.20), CARROT_BASE)
    assert CARROT_BASE < tau < CARROT_BASE * 3.0

  def test_total_is_capped(self):
    tau, _ = get_lat_smooth_seconds_dynamic(_model_output(0.40), 0.30)
    assert tau == LAT_SMOOTH_SECONDS_MAX

  def test_index_actually_resolves(self):
    """The bug in carrot's copy: a 4-index read of a 3-D array always raised, so the extra was
    silently dead. If this port ever regresses to that, the value stops responding to y_std."""
    low, _ = get_lat_smooth_seconds_dynamic(_model_output(0.10), CARROT_BASE)
    high, _ = get_lat_smooth_seconds_dynamic(_model_output(0.30), CARROT_BASE)
    assert high > low, "extra term is dead -- plan_stds index is wrong"

  def test_malformed_output_keeps_base(self):
    # a missing/!shaped plan_stds must not silently turn the feature off
    for bad in ({}, {'plan_stds': np.zeros((1, 2))}, {'plan_stds': None}):
      tau, _ = get_lat_smooth_seconds_dynamic(bad, CARROT_BASE)
      assert tau == CARROT_BASE

  def test_non_finite_std_keeps_base(self):
    tau, _ = get_lat_smooth_seconds_dynamic(_model_output(np.nan), CARROT_BASE)
    assert tau == CARROT_BASE


class TestSmoothingActuallyAttenuates:
  def test_two_hz_shake_is_reduced(self):
    """The measured shake was ~2 Hz. Feed that through at 20 Hz (DT_MDL) and it must come out
    materially smaller, while a constant input must pass through unchanged (no authority loss)."""
    dt, f, n = 0.05, 2.0, 400
    tau = 0.30

    sig = [np.sin(2 * np.pi * f * i * dt) for i in range(n)]
    out, prev = [], 0.0
    for v in sig:
      prev = smooth_value(v, prev, tau, dt)
      out.append(prev)

    settled = out[n // 2:]
    amp_in, amp_out = 1.0, (max(settled) - min(settled)) / 2
    assert amp_out < 0.35 * amp_in, f"2 Hz only attenuated to {amp_out:.3f}"

  def test_steady_state_is_unchanged(self):
    prev = 0.0
    for _ in range(500):
      prev = smooth_value(0.01, prev, 0.30, 0.05)
    assert abs(prev - 0.01) < 1e-6, "a low-pass must not cost steady-state curvature"


# --- lane-line criterion -------------------------------------------------------------------

from openpilot.selfdrive.controls.lib.drive_helpers import LANE_STD_RANGE   # noqa: E402


def _lane_output(lane_std, path_std=0.0):
  """Model output carrying both criteria; lane_lines_stds is (batch, line, idx, feat)."""
  stds = np.zeros((1, 4, 1, 1), dtype=np.float32)
  stds[0, 1, 0, 0] = lane_std
  stds[0, 2, 0, 0] = lane_std
  out = dict(_model_output(path_std))
  out['lane_lines_stds'] = stds
  return out


class TestLaneStdCriterion:
  def test_clean_lines_change_nothing(self):
    """Above 70 km/h the measured lane std is 0.04-0.06; that band must be untouched."""
    # float32 stores 0.1 as 0.10000000149, a hair over the breakpoint, so approx here.
    for std in (0.02, 0.045, 0.062, LANE_STD_RANGE[0]):
      tau, _ = get_lat_smooth_seconds_dynamic(_lane_output(std), CARROT_BASE)
      assert tau == pytest.approx(CARROT_BASE, abs=1e-6), std

  def test_weak_lines_add_smoothing(self):
    """The 40-70 km/h median (0.177) has to land mid-ramp, not at either end."""
    tau, driver = get_lat_smooth_seconds_dynamic(_lane_output(0.177), CARROT_BASE)
    assert tau > CARROT_BASE
    assert tau < CARROT_BASE * 3.0
    assert driver == pytest.approx(0.177, abs=1e-6)

  def test_bad_lines_reach_the_ceiling_of_the_ramp(self):
    tau, _ = get_lat_smooth_seconds_dynamic(_lane_output(LANE_STD_RANGE[1]), CARROT_BASE)
    assert tau == pytest.approx(min(CARROT_BASE * 3.0, LAT_SMOOTH_SECONDS_MAX))

  def test_monotone_in_lane_std(self):
    taus = [get_lat_smooth_seconds_dynamic(_lane_output(s), CARROT_BASE)[0]
            for s in (0.0, 0.1, 0.18, 0.25, 0.35, 0.6)]
    assert taus == sorted(taus)

  def test_worst_of_the_two_lines_wins(self):
    stds = np.zeros((1, 4, 1, 1), dtype=np.float32)
    stds[0, 1, 0, 0] = 0.05
    stds[0, 2, 0, 0] = 0.30
    out = dict(_model_output(0.0)); out['lane_lines_stds'] = stds
    tau, driver = get_lat_smooth_seconds_dynamic(out, CARROT_BASE)
    assert driver == pytest.approx(0.30, abs=1e-6)
    assert tau > CARROT_BASE


class TestTwoCriteriaTogether:
  def test_whichever_asks_for_more_wins(self):
    lane_only = get_lat_smooth_seconds_dynamic(_lane_output(0.35, path_std=0.0), CARROT_BASE)[0]
    path_only = get_lat_smooth_seconds_dynamic(_lane_output(0.0, path_std=0.25), CARROT_BASE)[0]
    both = get_lat_smooth_seconds_dynamic(_lane_output(0.35, path_std=0.25), CARROT_BASE)[0]
    assert both == max(lane_only, path_only)

  def test_path_criterion_still_works_without_lane_data(self):
    """The old behaviour has to survive for an artifact that publishes no lane stds."""
    tau, driver = get_lat_smooth_seconds_dynamic(_model_output(0.25), CARROT_BASE)
    assert tau > CARROT_BASE
    assert driver == pytest.approx(0.25, abs=1e-6)

  def test_neither_engaged_reports_the_path_std(self):
    tau, driver = get_lat_smooth_seconds_dynamic(_lane_output(0.05, path_std=0.10), CARROT_BASE)
    assert tau == CARROT_BASE
    assert driver == pytest.approx(0.10, abs=1e-6)

  def test_disabled_base_ignores_both(self):
    assert get_lat_smooth_seconds_dynamic(_lane_output(0.9, path_std=0.9), 0.0) == (0.0, 0.0)

  def test_malformed_lane_data_falls_back(self):
    out = dict(_model_output(0.0)); out['lane_lines_stds'] = np.zeros((1, 1, 1, 1))
    tau, _ = get_lat_smooth_seconds_dynamic(out, CARROT_BASE)
    assert tau == CARROT_BASE

  def test_nan_lane_std_falls_back(self):
    stds = np.full((1, 4, 1, 1), np.nan, dtype=np.float32)
    out = dict(_model_output(0.0)); out['lane_lines_stds'] = stds
    tau, _ = get_lat_smooth_seconds_dynamic(out, CARROT_BASE)
    assert tau == CARROT_BASE

  def test_never_exceeds_the_ceiling(self):
    tau, _ = get_lat_smooth_seconds_dynamic(_lane_output(5.0, path_std=5.0), 0.30)
    assert tau == LAT_SMOOTH_SECONDS_MAX
