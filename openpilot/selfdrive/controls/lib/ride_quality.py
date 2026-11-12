"""Ride-quality vector for longitudinal following: what a passenger feels, not what a controller tracks.

Shared by tests/test_follow_quality.py (the follow chain, open loop on recorded planner output) and
tools/tesla_analysis/commit_quality.py (whole commits, closed loop over a fixed set of morning-commute
minutes). All inputs are equal-length arrays at a fixed dt over steady following.

  brake_on    brake applications a minute (command crossing -0.3 m/s^2)
  ubon        of those, unprovoked: the lead was not braking (> -0.5 over the last second), not
              slower than us by more than 1 m/s, and at least 0.9 s ahead
  accel_on    throttle applications a minute (command crossing +0.3 m/s^2)
  rev         accel <-> brake reversals a minute (+-0.15 m/s^2 hysteresis)
  cmd_jerk    RMS rate of change of the command, m/s^3
  calm_brake  % of calm time (lead not closing, not braking, >= 1 s ahead) spent braking past -0.2
  abs_cmd     mean |command|, m/s^2
"""
import numpy as np

BRAKE_ON = -0.3
ACCEL_ON = 0.3
REV_BAND = 0.15


def crossings_down(a: np.ndarray, thr: float) -> int:
  return int(np.sum((a[1:] < thr) & (a[:-1] >= thr)))


def crossings_up(a: np.ndarray, thr: float) -> int:
  return int(np.sum((a[1:] > thr) & (a[:-1] <= thr)))


def reversals(a: np.ndarray, band: float = REV_BAND) -> int:
  st, n = 0, 0
  for x in a:
    s = 1 if x > band else (-1 if x < -band else 0)
    if s and st and s != st:
      n += 1
    if s:
      st = s
  return n


def components(cmd, v_ego, d_rel, v_lead, a_lead, dt: float) -> dict:
  """Additive sums for one stretch of steady following."""
  cmd, v, d, vl, al = (np.asarray(x, dtype=float) for x in (cmd, v_ego, d_rel, v_lead, a_lead))
  look = max(int(round(1.0 / dt)), 1)
  ubon = 0
  for q in np.where((cmd[1:] < BRAKE_ON) & (cmd[:-1] >= BRAKE_ON))[0] + 1:
    b = max(q - look, 0)
    if al[b:q + 1].min() > -0.5 and vl[q] >= v[q] - 1.0 and d[q] / max(v[q], 0.1) > 0.9:
      ubon += 1
  calm = (vl >= v - 0.3) & (d / np.maximum(v, 0.1) > 1.0) & (al > -0.3)
  return dict(seconds=len(cmd) * dt, brake_on=crossings_down(cmd, BRAKE_ON), ubon=ubon, accel_on=crossings_up(cmd, ACCEL_ON),
              rev=reversals(cmd), jerk_sq=float(np.sum((np.diff(cmd) / dt) ** 2)), n_jerk=len(cmd) - 1,
              abs_cmd=float(np.abs(cmd).sum()), n=len(cmd), calm=int(calm.sum()), calm_brake=int((calm & (cmd < -0.2)).sum()))


def vector(parts: list[dict]) -> dict:
  m = sum(p['seconds'] for p in parts) / 60.0
  if m <= 0:
    raise ValueError("no following time")
  return dict(minutes=m,
              brake_on=sum(p['brake_on'] for p in parts) / m,
              ubon=sum(p['ubon'] for p in parts) / m,
              accel_on=sum(p['accel_on'] for p in parts) / m,
              rev=sum(p['rev'] for p in parts) / m,
              cmd_jerk=float(np.sqrt(sum(p['jerk_sq'] for p in parts) / max(sum(p['n_jerk'] for p in parts), 1))),
              calm_brake=100.0 * sum(p['calm_brake'] for p in parts) / max(sum(p['calm'] for p in parts), 1),
              abs_cmd=sum(p['abs_cmd'] for p in parts) / max(sum(p['n'] for p in parts), 1))
