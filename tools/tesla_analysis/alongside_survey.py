#!/usr/bin/env python3
"""When a car is level with us, does anything on this bus say so?

The lane-change gate reads carState.leftBlindspot / rightBlindspot, which on this car is the
ultrasonic park-assist warning OR the AP module's DAS_blindSpotRear*. Both names say rear, and a
lane change into a car that is exactly abreast is the case that matters most, so this asks the
logs directly: while the factory camera reports a vehicle in the next lane at close range, is
either blind spot flag actually set?

Also reports how near the camera's own side groups ever get, since a vehicle it stops reporting
once it draws level is no use for this either.

  ./alongside_survey.py op-logs/00000087--0fb852a22a--*
"""
import sys
from collections import Counter

import numpy as np

from opendbc.can import CANParser
from opendbc.car.tesla.das_object import parse_das_object
from openpilot.tools.lib.logreader import LogReader

DAS_OBJECT = 777
GROUP_LEFT, GROUP_RIGHT = 1, 2
ALONGSIDE_M = 12.0      # near enough that a lane change into it would be a collision
MIN_SPEED = 8.0
# DAS_object gives each group about 6.7Hz; CAN frames arrive far faster, so this is a fraction of
# a second of silence -- a vehicle that has gone, not one between updates.
STALE_FRAMES = 400


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  cp = CANParser("tesla_can", [], 2)
  cp_ch = CANParser("tesla_can", [], 0)

  v_ego = 0.0
  bs = {'left': False, 'right': False}
  das_raw = Counter()
  park_raw = Counter()
  rows = []          # (side, dx, blindspot flag, das raw, park raw)
  near = {GROUP_LEFT: [], GROUP_RIGHT: []}
  # (group, id) -> [frames since last refreshed, vehicle]. Without the ageing every object the
  # camera ever reported stays in the picture forever, which silently turned this survey into a
  # count of everything that had ever been alongside rather than what is alongside now.
  latest: dict = {}

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
        bs['left'] = bool(msg.carState.leftBlindspot)
        bs['right'] = bool(msg.carState.rightBlindspot)
      elif w == 'can':
        frames = [(x.address, bytes(x.dat), x.src) for x in msg.can]
        if any(a == DAS_OBJECT and s == 2 for a, _, s in frames):
          cp.update([(msg.logMonoTime, frames)])
          for veh in parse_das_object(cp.vl["DAS_object"]):
            latest[(veh.group, veh.obj_id)] = [0, veh]
        cp_ch.update([(msg.logMonoTime, frames)])

        for key in list(latest):
          latest[key][0] += 1
          if latest[key][0] > STALE_FRAMES:
            del latest[key]

        if v_ego < MIN_SPEED:
          continue
        aps = cp.vl["AutopilotStatus"]
        pk = cp_ch.vl["PARK_status2"]
        for key, side in ((GROUP_LEFT, 'left'), (GROUP_RIGHT, 'right')):
          d_raw = int(aps[f"DAS_blindSpotRear{'Left' if side == 'left' else 'Right'}"])
          p_raw = int(pk[f"PARK_sdiBlindSpot{'Left' if side == 'left' else 'Right'}"])
          das_raw[d_raw] += 1
          park_raw[p_raw] += 1
          for (g, _), (_, veh) in latest.items():
            if g != key:
              continue
            near[key].append(veh.dx)
            if veh.dx < ALONGSIDE_M:
              rows.append((side, veh.dx, bs[side], d_raw, p_raw))

  print(f"raw values seen above {MIN_SPEED} m/s")
  print(f"  DAS_blindSpotRear*  {dict(sorted(das_raw.items()))}   (1,2 = warning, 3 = SNA)")
  print(f"  PARK_sdiBlindSpot*  {dict(sorted(park_raw.items()))}   (1 = warning)")

  print("\nhow close the camera's own side groups ever report a vehicle")
  for key, label in ((GROUP_LEFT, 'left'), (GROUP_RIGHT, 'right')):
    a = np.array(near[key])
    if a.size:
      print(f"  {label:5} n={a.size:6}  min {a.min():5.1f} m  p1 {np.percentile(a, 1):5.1f}  "
            f"p10 {np.percentile(a, 10):5.1f}  median {np.median(a):5.1f}")

  print(f"\nwith a camera-reported vehicle inside {ALONGSIDE_M:.0f} m in that lane:")
  if not rows:
    print("  never happened in this route")
    return
  a = np.array([(d, float(f), dr, pr) for _, d, f, dr, pr in rows])
  print(f"  frames                {len(a)}")
  print(f"  blindspot flag set    {100 * a[:, 1].mean():.1f}% of them")
  print(f"  DAS raw values        {dict(sorted(Counter(a[:, 2].astype(int)).items()))}")
  print(f"  PARK raw values       {dict(sorted(Counter(a[:, 3].astype(int)).items()))}")
  for lo, hi in ((0, 3), (3, 6), (6, 9), (9, 12)):
    sel = (a[:, 0] >= lo) & (a[:, 0] < hi)
    if sel.any():
      print(f"    {lo:2}-{hi:2} m  n={sel.sum():5}  flag set {100 * a[sel, 1].mean():5.1f}%")


if __name__ == '__main__':
  main(sys.argv[1:])
