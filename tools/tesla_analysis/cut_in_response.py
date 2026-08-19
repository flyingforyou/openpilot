#!/usr/bin/env python3
"""Someone merges in front. Does the planner open the gap back up, or just live with it?

A cut-in is not a lead that slowed down -- it is a new car appearing at a distance the plan never
asked for. The gap error goes sharply negative in one frame, and the only way back to the target
is to give up speed. This finds those moments and reports whether the plan actually asked for
that: what it wanted, what it had, and what acceleration it commanded while the gap was short.

A cut-in is detected as dRel collapsing faster than the lead could physically have braked, which
separates a merge from a lead simply slowing.

  ./cut_in_response.py op-logs/0000007f--ed761c79a7--*
"""
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

DROP_M = 8.0            # how much dRel must collapse to count
DROP_WINDOW_S = 1.0     # ...within this long
MIN_SPEED = 5.0
HOLD_S = 8.0            # how long to watch afterwards
CAR_LENGTH = 5.0


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  rows = []
  st = {'v': 0.0, 'a': 0.0}
  t0 = None

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w == 'carState':
        st['v'] = float(msg.carState.vEgo)
        st['a'] = float(msg.carState.aEgo)
      elif w == 'radarState':
        lead = msg.radarState.leadOne
        st['d'] = float(lead.dRel) if lead.present else np.nan
        st['vl'] = float(lead.vLead) if lead.present else np.nan
        st['prob'] = float(lead.modelProb) if lead.present else 0.0
      elif w == 'longitudinalPlan' and 'd' in st:
        lp = msg.longitudinalPlan
        rows.append((t - t0, st['v'], st['a'], st['d'], st['vl'], st['prob'],
                     float(lp.desiredDistance), float(lp.tFollow), float(lp.aTarget)))

  if not rows:
    print("no plan frames with a lead")
    return
  a = np.array(rows)
  t, v, a_ego, d, v_lead, prob, want, tf, a_tgt = a.T
  dt = np.median(np.diff(t))
  win = max(1, int(DROP_WINDOW_S / dt))
  hold = max(1, int(HOLD_S / dt))

  print(f"{len(t)} plan frames, {np.sum(np.isfinite(d))} with a tracked lead, dt {dt*1000:.0f}ms")

  # A cut-in: dRel drops by DROP_M inside the window, and the drop is not something the lead
  # could have done by braking -- a lead decelerating hard closes the gap far more slowly.
  events = []
  for i in range(win, len(t) - hold):
    if not (np.isfinite(d[i]) and np.isfinite(d[i - win])):
      continue
    if v[i] < MIN_SPEED:
      continue
    if d[i - win] - d[i] < DROP_M:
      continue
    # closing this fast means a different car, not the same one slowing
    closing = (d[i - win] - d[i]) / (t[i] - t[i - win])
    if closing < 6.0:
      continue
    if events and t[i] - events[-1] < 10.0:
      continue
    events.append(t[i])

  print(f"\n-- cut-ins found: {len(events)} --")
  if not events:
    return

  print("   'short' = the gap the plan wanted minus the gap it had, at the moment of the merge")
  head = f"\n  {'when':>8} {'mph':>5} {'dRel':>6} {'wanted':>7} {'short':>7} {'headway':>8}"
  print(f"{head} {'aTarget':>8} {'aEgo':>7}   {'recovered in':>13}")

  shorts, recov, never = [], [], 0
  for et in events:
    i = int(np.searchsorted(t, et))
    j = min(i + hold, len(t) - 1)
    short = want[i] - d[i]
    shorts.append(short)
    # when the gap first gets back to within a metre of what was asked
    seg = slice(i, j)
    ok = np.where(d[seg] >= want[seg] - 1.0)[0]
    back = t[i + ok[0]] - et if ok.size else None
    if back is None:
      never += 1
    else:
      recov.append(back)
    left = f"  {et/60:7.1f}m {v[i]*2.23694:5.0f} {d[i]:5.1f}m {want[i]:6.1f}m {short:6.1f}m"
    got = 'never in 8s' if back is None else f'{back:8.1f}s'
    print(f"{left} {d[i]/max(v[i],0.1):7.2f}s {a_tgt[i]:+7.2f} {a_ego[i]:+6.2f}   {got}")

  s = np.array(shorts)
  print(f"\n  short by      median {np.median(s):.1f} m   worst {s.max():.1f} m")
  print(f"  recovered     {len(recov)} of {len(events)}"
        + (f", median {np.median(recov):.1f}s" if recov else "")
        + f"   never inside 8s: {never}")

  # What the plan asked for while the gap was short, which is the actual question: a plan that
  # wants more distance and cannot get it looks the same from the seat as one that never asked.
  print("\n-- while short of the target gap, what did the plan command? --")
  short_mask = np.isfinite(d) & (want - d > 2.0) & (v > MIN_SPEED)
  if short_mask.any():
    at = a_tgt[short_mask]
    print(f"  frames short by >2m   {short_mask.sum()} ({100*short_mask.mean():.1f}% of all)")
    tail = f"p10 {np.percentile(at,10):+.2f}  p90 {np.percentile(at,90):+.2f} m/s^2"
    print(f"  aTarget               median {np.median(at):+.2f}  {tail}")
    print(f"  asked to slow at all  {100*np.mean(at < -0.1):.0f}% of those frames")
    print(f"  asked to *speed up*   {100*np.mean(at > 0.1):.0f}% of those frames")

  print("\n-- how close does it ever actually sit? --")
  near = np.isfinite(d) & (v > MIN_SPEED)
  dn = d[near]
  for thresh, label in ((CAR_LENGTH, 'under one car length (5m)'), (10.0, 'under 10m'), (15.0, 'under 15m')):
    frames = np.sum(dn < thresh)
    print(f"  {label:28} {frames:6} frames ({100*frames/dn.size:.2f}%)")
  print(f"  closest with a lead tracked  {dn.min():.1f} m")


if __name__ == '__main__':
  main(sys.argv[1:])
