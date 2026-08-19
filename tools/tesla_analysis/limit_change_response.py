#!/usr/bin/env python3
"""When the map limit changes, what actually paces the car -- the slew, or the stalk?

The cluster MAX is moved by pressing the stalk, and the setpoint is moved by a rate limit in
map_cruise. Both ramp, so from the seat they are easy to confuse. This separates them: it finds
every limit change while moving and reports how fast the ceiling, the cluster and the setpoint
each got to the new number, and what acceleration the car was actually given.

  ./limit_change_response.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025
KPH_TO_MS = 1 / 3.6
MIN_STEP_MPH = 5.0      # what counts as a limit change
MOVING_MPH = 20.0


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  rows = []
  st, t0 = {}, None

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w == 'carState':
        c = msg.carState
        st['max'] = float(c.cruiseState.speed) * MPH
        st['v'] = float(c.vEgo) * MPH
        st['a'] = float(c.aEgo)
      elif w == 'longitudinalPlan' and 'v' in st:
        lp = msg.longitudinalPlan
        rows.append((t - t0,
                     float(lp.cruiseCeiling) * KPH_TO_MS * MPH,   # what the road allows, instant
                     float(lp.cruiseTarget) * 0.621371,           # the slewed setpoint
                     st['max'], st['v'], st['a'], float(lp.aTarget)))

  if not rows:
    print("no plan frames")
    return
  arr = np.array(rows)
  t, ceil, tgt, mx, v, a_ego, a_tgt = arr.T

  print("rate limits in play")
  print("  setpoint slew up      0.5 m/s^2  = 1.12 mph/s   (after a 3.0 s dwell)")
  print("  setpoint slew down    1.0 m/s^2  = 2.24 mph/s")
  print("  stalk press           1 or 5 mph every 0.30 s  = 3.3 or 16.7 mph/s")
  print("  -> the stalk is 3-15x faster than the setpoint, so the cluster arrives first\n")

  # a change is where the ceiling steps and stays stepped
  events = []
  for i in range(5, len(t) - 1):
    if abs(ceil[i] - ceil[i - 5]) >= MIN_STEP_MPH and v[i] > MOVING_MPH:
      if not events or t[i] - events[-1][0] > 20.0:
        events.append((t[i], ceil[i - 5], ceil[i]))

  print(f"-- limit changes while moving: {len(events)} --")
  for et, before, after in events[:10]:
    rising = after > before
    win = (t >= et - 2) & (t <= et + 45)
    if win.sum() < 20:
      continue
    tw, cw, gw, mw, vw = t[win], ceil[win], tgt[win], mx[win], v[win]

    def reach(series, goal, up, tw=tw, et=et):
      hit = np.where(series >= goal - 1.0 if up else series <= goal + 1.0)[0]
      return tw[hit[0]] - et if len(hit) else None

    r_ceil = reach(cw, after, rising)
    r_max = reach(mw, after, rising)
    r_tgt = reach(gw, after, rising)
    aw = a_ego[win]
    print(f"\n  t+{et/60:5.1f} min   {before:.0f} -> {after:.0f} mph  ({'up' if rising else 'down'})   car was doing {vw[0]:.0f} mph")
    for label, val in (("ceiling (road)", r_ceil), ("cluster MAX (stalk)", r_max),
                       ("setpoint (slew)", r_tgt)):
      print(f"      {label:22} {'never in 45 s' if val is None else f'{val:5.1f} s'}")
    if rising:
      moved = np.abs(np.diff(gw))
      rate = np.median(moved[moved > 1e-6]) / 0.05 if (moved > 1e-6).any() else 0.0
      print(f"      setpoint moved at      {rate:5.2f} mph/s   (limit 1.12)")
      print(f"      aEgo while ramping     median {np.median(aw):+.2f}  p90 {np.percentile(aw, 90):+.2f} m/s^2")


if __name__ == '__main__':
  main(sys.argv[1:])
