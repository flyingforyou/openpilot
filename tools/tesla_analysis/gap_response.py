#!/usr/bin/env python3
"""Did moving the gap stalk actually move the following distance?

Three things have to line up and each can fail quietly: the stalk has to reach `gapAdjust`,
`gapAdjust` has to reach the planner's `tFollow`, and `tFollow` has to be what is limiting the
car rather than the set speed. This reports all three, then the headway actually achieved per
gap setting.

Headway is only counted on a lead that has been tracked continuously -- `dRel` teleports tens
of metres on re-association, and naive statistics measure that instead of the car.

  ./gap_response.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys
from collections import defaultdict

import numpy as np

from openpilot.tools.lib.logreader import LogReader

CONTINUITY_M = 1.5      # a jump bigger than this is a different object
MIN_HOLD_S = 1.5        # how long a track must be clean before it counts
STEADY_VREL = 1.0       # |vRel| below this is following, not closing or dropping back


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)

  gap, v_ego, t_follow, desired, plan_src = 0, 0.0, 0.0, 0.0, ''
  long_active = False
  prev = None                      # (t, dRel, vRel)
  clean_since = None
  gap_timeline, samples = [], []
  tf_by_gap = defaultdict(list)
  src_by_gap = defaultdict(lambda: defaultdict(int))
  t0 = None

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      if w == 'carState':
        c = msg.carState
        v_ego = float(c.vEgo)
        g = int(c.cruiseState.gapAdjust)
        if g != gap:
          gap_timeline.append((t - t0, gap, g))
          gap = g
      elif w == 'carControl':
        long_active = bool(msg.carControl.longActive)
      elif w == 'longitudinalPlan':
        lp = msg.longitudinalPlan
        t_follow = float(lp.tFollow)
        desired = float(lp.desiredDistance)
        plan_src = str(lp.longitudinalPlanSource)
        if long_active and gap:
          tf_by_gap[gap].append(t_follow)
          src_by_gap[gap][plan_src] += 1
      elif w == 'radarState':
        lead = msg.radarState.leadOne
        if not lead.present:
          prev, clean_since = None, None
          continue
        d, vr = float(lead.dRel), float(lead.vRel)
        if prev is not None:
          dt = t - prev[0]
          pred = prev[1] + prev[2] * dt
          if dt <= 0 or dt > 0.5 or abs(d - pred) > CONTINUITY_M:
            clean_since = None
          elif clean_since is None:
            clean_since = t
        prev = (t, d, vr)
        # Only count frames where the lead is what the planner is solving for. Cruising at set
        # speed behind a lead that happens to sit at a steady distance says nothing about the
        # gap setting, and those frames swamped the comparison when they were included.
        if (clean_since is not None and t - clean_since >= MIN_HOLD_S and long_active
            and v_ego > 8.0 and abs(vr) < STEADY_VREL and gap
            and plan_src.startswith('lead')):
          samples.append((gap, v_ego, d, d / max(v_ego, 1e-3), t_follow, desired))

  print(f"segments {len(paths)}   gap stalk changes: {len(gap_timeline)}")
  for tt, a, b in gap_timeline[:24]:
    print(f"    t+{tt/60:5.1f} min   gap {a} -> {b}")
  if len(gap_timeline) > 24:
    print(f"    ... and {len(gap_timeline)-24} more")

  print("\n-- did gapAdjust reach the planner? (tFollow commanded per gap, long engaged) --")
  for g in sorted(tf_by_gap):
    a = np.array(tf_by_gap[g])
    src = src_by_gap[g]
    lead_share = 100 * (src.get('lead0', 0) + src.get('lead1', 0)) / max(sum(src.values()), 1)
    spread = f"p10 {np.percentile(a,10):.3f}  p90 {np.percentile(a,90):.3f}"
    print(f"    gap {g}:  n={len(a):6}  tFollow median {np.median(a):.3f}  {spread}   lead-limited {lead_share:.0f}% of frames")

  if not samples:
    print("\nno clean steady-following samples")
    return

  g, v, d, hw, tf, des = (np.array(x, dtype=float) for x in zip(*samples, strict=True))
  print(f"\n-- headway actually achieved, clean tracks only (n={len(g)}) --")
  print("    gap    n     v(mph)   dRel(m)          headway(s)         commanded")
  for gg in sorted(set(g.astype(int))):
    s = g == gg
    if s.sum() < 30:
      print(f"    {gg:3}  {s.sum():5}   (too few)")
      continue
    d_col = f"{np.median(d[s]):5.1f} (p25 {np.percentile(d[s],25):4.1f} p75 {np.percentile(d[s],75):4.1f})"
    hw_col = f"{np.median(hw[s]):5.2f} (p25 {np.percentile(hw[s],25):4.2f} p75 {np.percentile(hw[s],75):4.2f})"
    cmd = f"tF {np.median(tf[s]):.2f}  want {np.median(des[s]):5.1f}m"
    print(f"    {gg:3}  {s.sum():5}   {np.median(v[s])*2.23694:5.1f}   {d_col}   {hw_col}   {cmd}")

  # speed-matched, so a gap that only ever ran on the freeway is not compared to town
  print("\n-- speed-matched headway (only speeds where 2+ gap settings have data) --")
  bins = [(15, 20), (20, 25), (25, 30)]
  for lo, hi in bins:
    sel = (v >= lo) & (v < hi)
    present = [gg for gg in sorted(set(g[sel].astype(int))) if (sel & (g == gg)).sum() >= 30]
    if len(present) < 2:
      continue
    print(f"    {lo*2.23694:.0f}-{hi*2.23694:.0f} mph:")
    for gg in present:
      s = sel & (g == gg)
      print(f"        gap {gg}  n={s.sum():5}  dRel {np.median(d[s]):5.1f} m   headway {np.median(hw[s]):.2f} s   tFollow {np.median(tf[s]):.2f}")


if __name__ == '__main__':
  main(sys.argv[1:])
