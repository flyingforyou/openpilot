"""Distance-based accel ceiling for stop-and-go (route 00000178 seg 6: +1.67 at 12-15 m, then -2.31)."""
import pytest

from openpilot.selfdrive.controls.lib.carrot_t_follow import (
  CLOSE_LEAD_DIST_BP,
  CLOSE_LEAD_SPEED_BP,
  close_lead_accel_cap as cap,
)

A_MAX = 1.73      # CruiseMaxVals at 30 km/h with the 09-11 curve -- what the surges rode


def test_the_logged_surge_is_trimmed():
  """12-15 m behind at ~20-30 km/h: the frames that commanded +1.67."""
  for d in (12.1, 12.9, 15.0):
    assert cap(A_MAX, d, 25.0, 1.0) < 1.2, d


def test_full_cap_inside_the_near_breakpoint():
  assert cap(A_MAX, 5.0, 20.0, 1.0) == pytest.approx(1.0)
  assert cap(A_MAX, CLOSE_LEAD_DIST_BP[0], 20.0, 1.0) == pytest.approx(1.0)


def test_open_road_gap_closing_untouched():
  """The 09-11 complaint was a lead pulling away at 40-67 m; nothing past 35 m may change."""
  for d in (CLOSE_LEAD_DIST_BP[1], 40.0, 67.0):
    assert cap(A_MAX, d, 30.0, 1.0) == pytest.approx(A_MAX), d


def test_highway_untouched():
  for d in (8.0, 15.0, 30.0):
    assert cap(1.30, d, CLOSE_LEAD_SPEED_BP[1], 1.0) == pytest.approx(1.30)
    assert cap(1.10, d, 100.0, 1.0) == pytest.approx(1.10)


def test_monotone_in_distance():
  vals = [cap(A_MAX, d, 25.0, 1.0) for d in (5, 10, 15, 20, 25, 30, 35, 50)]
  assert vals == sorted(vals)


def test_never_raises_the_ceiling():
  for a in (0.5, 1.0, 1.5, 2.0):
    for d in (3.0, 15.0, 40.0):
      for v in (0.0, 30.0, 50.0, 90.0):
        assert cap(a, d, v, 1.0) <= a + 1e-9


def test_cap_above_a_max_is_a_no_op():
  assert cap(0.8, 12.0, 20.0, 1.0) == 0.8


def test_off_and_no_lead():
  assert cap(A_MAX, 12.0, 20.0, 0.0) == A_MAX
  assert cap(A_MAX, None, 20.0, 1.0) == A_MAX
  assert cap(A_MAX, 0.0, 20.0, 1.0) == A_MAX


def test_fades_with_speed():
  vals = [cap(A_MAX, 12.0, v, 1.0) for v in (20, 40, 50, 60)]
  assert vals == sorted(vals)
