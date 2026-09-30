"""aLeadK from the radar's own LongAccel instead of the lagging Kalman derivative."""
import math

import pytest

from openpilot.selfdrive.controls.lib.radar_lead_hold import RADAR_LEAD_ACCEL_CLIP, radar_lead_accel


def test_relative_plus_ego_is_absolute():
  """LongAccel is relative: a lead matching our -1.0 braking reads 0 relative, -1.0 absolute."""
  assert radar_lead_accel(0.0, -1.0) == pytest.approx(-1.0)
  assert radar_lead_accel(-1.5, 0.3) == pytest.approx(-1.2)
  assert radar_lead_accel(1.0, 0.5) == pytest.approx(1.5)


def test_missing_value_falls_back():
  """None tells radard to keep the Kalman estimate."""
  assert radar_lead_accel(float('nan'), 0.0) is None
  assert radar_lead_accel(None, 0.0) is None
  assert radar_lead_accel(-1.0, float('nan')) is None


def test_clipped_to_physical_range():
  lo, hi = RADAR_LEAD_ACCEL_CLIP
  assert radar_lead_accel(-15.9, -2.0) == lo
  assert radar_lead_accel(15.9, 2.0) == hi


def test_both_directions_pass_through():
  """Accelerating leads matter as much as braking ones -- the lurch rode a stale +0.75."""
  for a in (-3.0, -0.5, 0.0, 0.5, 2.0):
    assert radar_lead_accel(a, 0.0) == pytest.approx(a)


def test_returns_float():
  v = radar_lead_accel(-0.5, 0.2)
  assert isinstance(v, float) and math.isfinite(v)
