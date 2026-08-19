#!/usr/bin/env python3
"""Could this radar populate a useful leadLeft/leadRight?

Deriving them is easy -- classify tracks by lateral offset and take the nearest. Whether the
answer is worth anything depends on where the radar can actually see, so this measures the real
coverage: how far out in lateral offset tracks appear, and crucially how *close* in dRel a track
in an adjacent lane ever gets. A lane change cares about the car beside you; a forward radar may
only ever see the one well ahead.

  ./side_radar_coverage.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

# Lane is ~3.3 m here (measured median on this drive). Adjacent-lane centre sits about there,
# so anything from half a lane out to two lanes out is "not my lane".
ADJACENT_LO, ADJACENT_HI = 1.8, 5.4
BESIDE_M = 10.0     # within this much longitudinally is "alongside", the lane-change question


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  d, y, v = [], [], []
  frames = 0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      if msg.which() != 'radarTracks':
        continue
      frames += 1
      for pt in msg.radarTracks.points:
        d.append(float(pt.dRel))
        y.append(float(pt.yRel))
        v.append(float(pt.vRel))

  d, y, v = np.array(d), np.array(y), np.array(v)
  print(f"radar frames {frames}, points {len(d)}  ({len(d)/max(frames,1):.1f} per frame)")

  print("\n-- longitudinal reach --")
  print(f"  dRel  min {d.min():.1f}  p1 {np.percentile(d,1):.1f}  median {np.median(d):.1f}  max {d.max():.1f} m")
  print(f"  points behind the car (dRel < 0): {(d < 0).sum()}")
  for lim in (5, 10, 15, 20, 30):
    print(f"  closer than {lim:2} m: {(d < lim).sum():6}  ({100*(d < lim).mean():.2f}% of points)")

  print("\n-- lateral reach --")
  print(f"  yRel  min {y.min():+.1f}  p1 {np.percentile(y,1):+.1f}  median {np.median(y):+.1f}  p99 {np.percentile(y,99):+.1f}  max {y.max():+.1f} m")
  ay = np.abs(y)
  for lo, hi, label in ((0, 1.8, 'my lane'), (1.8, 5.4, 'adjacent lane'), (5.4, 99, 'two lanes out')):
    s = (ay >= lo) & (ay < hi)
    print(f"  {label:16} {s.sum():7} points ({100*s.mean():5.2f}%)"
          + (f"   dRel min {d[s].min():.1f}  p1 {np.percentile(d[s],1):.1f}  median {np.median(d[s]):.1f} m"
             if s.any() else ""))

  print("\n-- the lane-change question: anything alongside, in the next lane? --")
  adj = (ay >= ADJACENT_LO) & (ay < ADJACENT_HI)
  beside = adj & (d < BESIDE_M)
  print(f"  adjacent-lane points within {BESIDE_M:.0f} m longitudinally: {beside.sum()}  ({100*beside.mean():.3f}% of all points)")
  if beside.any():
    print(f"    dRel {d[beside].min():.1f}..{d[beside].max():.1f} m")
  for lim in (15, 20, 30, 50):
    s = adj & (d < lim)
    print(f"  adjacent-lane points closer than {lim:2} m: {s.sum():6}")

  print("\n-- lateral spread as a function of distance (is yRel even usable up close?) --")
  print(f"  {'dRel band':>14} {'points':>8} {'|yRel| p50':>11} {'|yRel| p95':>11} {'max':>7}")
  for lo, hi in ((0, 10), (10, 20), (20, 40), (40, 70), (70, 200)):
    s = (d >= lo) & (d < hi)
    if s.sum() < 10:
      print(f"  {f'{lo}-{hi} m':>14} {s.sum():>8}   (too few)")
      continue
    print(f"  {f'{lo}-{hi} m':>14} {s.sum():>8} {np.percentile(ay[s],50):>11.1f} {np.percentile(ay[s],95):>11.1f} {ay[s].max():>7.1f}")


if __name__ == '__main__':
  main(sys.argv[1:])
