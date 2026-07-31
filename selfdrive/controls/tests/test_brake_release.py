"""The velocity PID was giving away a third of the braking demand under hard braking.

Route 0000001b: the plan asked for -3.50 (its ACCEL_MIN floor) and the delivered command was
-2.34, because longitudinalPlan.speeds is anchored on the planner's own filtered state rather
than vEgo, so the tracking error goes positive while braking and the correction releases.
"""
import numpy as np

from openpilot.selfdrive.controls.lib.longcontrol import BRAKE_RELEASE_FADE_BP, limit_brake_release


class TestLimitBrakeRelease:
  def test_hard_braking_keeps_the_full_plan(self):
    assert limit_brake_release(-2.34, -3.50) == -3.50

  def test_extra_braking_is_never_limited(self):
    # a correction asking for MORE braking than the plan must pass straight through
    assert limit_brake_release(-4.0, -3.5) == -4.0
    assert limit_brake_release(-1.2, -0.5) == -1.2

  def test_gentle_braking_is_untouched(self):
    # this is where the precise-stop benefit lives; logs showed 0.01-0.03 error here
    assert limit_brake_release(-0.64, -0.70) == -0.64
    assert limit_brake_release(-0.10, -0.14) == -0.10

  def test_fade_is_proportional_in_the_band(self):
    lo, hi = BRAKE_RELEASE_FADE_BP          # [-2.0, -1.0]
    mid = (lo + hi) / 2                     # -1.5 -> half the release allowed
    assert limit_brake_release(mid + 1.0, mid) == mid + 0.5
    # at the bottom of the band none of it survives, at the top all of it does
    assert limit_brake_release(lo + 1.0, lo) == lo
    assert limit_brake_release(hi + 1.0, hi) == hi + 1.0

  def test_monotonic_across_the_band(self):
    # a fixed release request must never produce less braking as the plan leans harder
    out = [limit_brake_release(a + 1.0, a) for a in np.linspace(-3.5, 0.0, 40)]
    assert all(b >= a - 1e-9 for a, b in zip(out, out[1:], strict=False))

  def test_survives_nan(self):
    assert np.isnan(limit_brake_release(float('nan'), -3.0))
    assert limit_brake_release(-2.0, float('nan')) == -2.0
