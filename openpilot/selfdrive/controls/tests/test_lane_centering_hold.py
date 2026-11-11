"""The integrated lane-centre trim (LaneCenteringCurveHold) and a stand-in car to try it against."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import lane_centering as lc

X = np.linspace(0.0, 100.0, 33)


def model(center_near, center_far=None, path_far=0.0, state=log.LaneChangeState.off, prob=0.9):
  """Lane 3.6 m wide whose centre is `center_near` m to the right of the car now."""
  center_far = center_near if center_far is None else center_far
  c = center_near + (center_far - center_near) * X / 30.0
  line = lambda y: SimpleNamespace(x=X, y=y)
  return SimpleNamespace(
    laneLines=[line(c - 5.4), line(c - 1.8), line(c + 1.8), line(c + 5.4)],
    laneLineProbs=[prob] * 4, laneLineStds=[0.1] * 4,
    position=SimpleNamespace(x=X, y=path_far * X / 30.0, yStd=np.full_like(X, 0.2)),
    meta=SimpleNamespace(laneChangeState=state))


def step(c, m, curvature=0.0, v=30.0, hold=True, n=1, **kw):
  for _ in range(n):
    out = c.update(curvature, m, v, True, 0.0, 1.0, True, True, curve_hold=hold, **kw)
  return out - curvature


class TestHold:
  K = 1.5 / 30.0 ** 2                                  # a right-hand bend at 1.5 m/s^2 and 30 m/s

  def test_pulls_toward_the_centre_in_a_bend(self):
    c = lc.LaneCenteringController()
    # the car is 0.25 m toward the inside (right): the centre is 0.25 m to its left; the model's plan
    # shows no error at the lookahead, so the proportional trim there has nothing to say
    early = step(c, model(-0.25, center_far=0.0), self.K, n=200)
    later = step(c, model(-0.25, center_far=0.0), self.K, n=600)
    assert early < 0.0                                 # less right: back toward the middle
    # 2 s in: the proportional part plus 2 s of integral, through the 0.4 s smoothing
    assert early * 900 == pytest.approx(-(lc.HOLD_KP_LAT + lc.HOLD_KI * 900 * 2.0) * (0.25 - 0.08), rel=0.2)
    assert later < early                               # and the integral on top of it

  def test_off_is_the_old_output(self):
    m = model(-0.25, center_far=0.0)
    assert step(lc.LaneCenteringController(), m, self.K, hold=False, n=500) == pytest.approx(0.0, abs=1e-9)
    assert step(lc.LaneCenteringController(), m, self.K, hold=True, n=500) != 0.0

  def test_bounded_by_the_bend(self):
    k = 1.0 / 900
    out = step(lc.LaneCenteringController(), model(-0.6, center_far=0.0), k, n=6000)
    assert abs(out) == pytest.approx(lc.HOLD_BEND_SHARE * k + lc.HOLD_FLOOR_LAT / 900, rel=0.02)
    hard = step(lc.LaneCenteringController(), model(-0.6, center_far=0.0), 4.0 / 900, n=20000)
    assert abs(hard) * 900 == pytest.approx(lc.HOLD_MAX_LAT, rel=0.02)

  def test_leaves_with_the_bend(self):
    c = lc.LaneCenteringController()
    step(c, model(-0.6, center_far=0.0), self.K, n=3000)
    # straight again and still off centre the same way: only the floor is left, and no integral
    out = step(c, model(-0.6, center_far=0.0), 0.0, n=800)
    assert abs(out) * 900 <= lc.HOLD_FLOOR_LAT + 0.01
    assert abs(c._hold_i) * 900 < 0.01

  def test_inside_the_deadband_nothing_builds(self):
    assert step(lc.LaneCenteringController(), model(-0.07, center_far=0.0), self.K, n=1000) == 0.0

  def test_fades_on_a_blinker_and_drops_in_a_lane_change(self):
    c = lc.LaneCenteringController()
    m = model(-0.3, center_far=0.0)
    full = step(c, m, self.K, n=3000)
    faded = step(c, m, self.K, n=100, pause_on_signal=True, turn_signal_active=True)
    assert abs(faded) < 0.2 * abs(full)
    c2 = lc.LaneCenteringController()
    step(c2, m, self.K, n=3000)
    assert step(c2, model(-0.3, center_far=0.0, state=log.LaneChangeState.laneChangeStarting), self.K) == 0.0

  def test_fades_without_lines(self):
    c = lc.LaneCenteringController()
    full = step(c, model(-0.3, center_far=0.0), self.K, n=3000)
    assert abs(step(c, model(-0.3, center_far=0.0, prob=0.3), self.K, n=300)) < 0.05 * abs(full)


def drive(omega, hold, v=30.0, peak=1.5, zeta=1.0, tau=0.2):
  """A car whose model steers back to its own line like a spring (natural frequency `omega` rad/s)
  and prefers to sit toward the inside of a bend, as measured: 0.08 + 0.09 m per m/s^2. Returns
  (t, p): metres to the right of the lane centre. The bend is to the right: in over 2 s, 12 s, out."""
  c = lc.LaneCenteringController()
  kp, kd = omega ** 2 / v ** 2, 2 * zeta * omega / v ** 2
  p = dp = applied = 0.0
  ts, ps = [], []
  for i in range(int(26.0 / DT_CTRL)):
    t = i * DT_CTRL
    lat = peak * float(np.clip(min(t / 2.0, (16.0 - t) / 2.0), 0.0, 1.0))
    kr = lat / v ** 2
    pref = (0.08 + 0.09 * lat) if lat > 0.05 else 0.0
    k_model = kp * (pref - p) - kd * dp
    look = float(np.clip(v, 8.0, 35.0))
    m = model(-p, center_far=-p, path_far=-p * 30.0 / look)    # the plan is on the centre a lookahead out
    cmd = c.update(kr + k_model, m, v, True, 0.0, 1.0, True, True, curve_hold=hold) - kr
    applied += (cmd - applied) * DT_CTRL / tau
    dp += applied * v ** 2 * DT_CTRL
    p += dp * DT_CTRL
    ts.append(t); ps.append(p)
  return np.array(ts), np.array(ps)


class TestAgainstAStandInCar:
  """The spring is measured where lane changes end: 2.0 rad/s, critically damped (117 of them)."""

  @pytest.mark.parametrize("omega", [1.0, 1.5, 2.0, 3.0])
  def test_takes_the_car_off_the_inside_without_ringing(self, omega):
    t, off = drive(omega, False)
    t, on = drive(omega, True)
    bend = (t > 9.0) & (t < 14.0)
    assert off[bend].mean() == pytest.approx(0.215, abs=0.01)        # the measured inside line
    assert on[bend].mean() < off[bend].mean() - 0.05                 # less of it
    assert on[bend].mean() > 0.05                                    # never pushed to the outside
    assert on[t > 16.0].min() > -0.08                                # a small swing the other way on the way out
    assert abs(on[-1]) < 0.01                                        # and nothing left on the straight
    steady = on[(t > 8.0) & (t < 14.0)]
    assert np.ptp(steady) < 0.03                                     # no ringing in the bend

  def test_the_measured_spring(self):
    t, on = drive(2.0, True)
    assert on[(t > 9.0) & (t < 14.0)].mean() == pytest.approx(0.124, abs=0.01)
    assert on[t > 16.0].min() > -0.05


if __name__ == "__main__":
  for omega in (0.6, 1.0, 1.5, 2.0, 3.0):
    t, off = drive(omega, False); _, on = drive(omega, True)
    b = (t > 9) & (t < 14)
    print(f"omega {omega:.1f} rad/s: in the bend {off[b].mean():+.3f} -> {on[b].mean():+.3f} m, at 4 s into it {on[(t>3.9)&(t<4.1)].mean():+.3f}, "
          f"widest after the bend {on[t > 16].min():+.3f} (off {off[t > 16].min():+.3f}), settled {on[-1]:+.3f}")
