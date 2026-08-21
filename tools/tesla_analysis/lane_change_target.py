#!/usr/bin/env python3
"""What is already in the lane we are about to move into, and what would it cost to care?

Two things came back from the road that both come down to this question.

The lane change is only held once a vehicle we passed has dropped out of the camera's view --
so while the camera can still plainly see a car alongside or just ahead in the next lane, nothing
stops the move. The block should start there, not after it.

And moving in behind something slower should slow the car to fit. Today the vehicle in the target
lane is not a lead until the change has finished, so nothing anticipates it: a lane change behind
a lorry is made at the speed we were already doing.

So this measures both from the same pass: how much driving time a "something is in that lane"
block would cost at each threshold, and, at the moments a lane change actually began, what was
ahead in the target lane and how much slower it was.

  ./lane_change_target.py op-logs/00000087--0fb852a22a--*
"""
import sys

import numpy as np

from opendbc.can import CANParser
from opendbc.car.tesla.das_object import parse_das_object
from openpilot.tools.lib.logreader import LogReader

DAS_OBJECT = 777
GROUP_LEFT, GROUP_RIGHT = 1, 2
SIDES = {GROUP_LEFT: 'left', GROUP_RIGHT: 'right'}
STALE_S = 0.6
MIN_SPEED = 8.0


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  cp = CANParser("tesla_can", [], 2)

  v_ego, t0, last_state = 0.0, None, 'off'
  direction = 'none'
  seen: dict = {}
  frames = {'left': [], 'right': []}     # nearest dx in that lane, per sampled frame
  starts = []                            # (side, dx, vx_rel, v_ego) when a change began
  n_frames = 0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      rel = t - t0
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'modelV2':
        state = str(msg.modelV2.meta.laneChangeState)
        if state == 'laneChangeStarting' and last_state != 'laneChangeStarting':
          side = 'left' if direction == 'left' else 'right' if direction == 'right' else None
          if side:
            group = GROUP_LEFT if side == 'left' else GROUP_RIGHT
            ahead = [(v.dx, v.vx_rel) for (g, _), (ts, v) in seen.items()
                     if g == group and rel - ts < STALE_S and v.dx > 0]
            if ahead:
              dx, vx = min(ahead)
              starts.append((side, dx, vx, v_ego))
            else:
              starts.append((side, np.nan, np.nan, v_ego))
        last_state = state
        d = str(msg.modelV2.meta.laneChangeDirection)
        if d != 'none':
          direction = d
      elif w == 'can' and any(x.address == DAS_OBJECT and x.src == 2 for x in msg.can):
        cp.update([(msg.logMonoTime, [(x.address, bytes(x.dat), x.src) for x in msg.can])])
        for veh in parse_das_object(cp.vl["DAS_object"]):
          if veh.group in SIDES:
            seen[(veh.group, veh.obj_id)] = (rel, veh)
        for key in list(seen):
          if rel - seen[key][0] > STALE_S:
            del seen[key]

        if v_ego < MIN_SPEED:
          continue
        n_frames += 1
        for group, side in SIDES.items():
          near = [v.dx for (g, _), (_, v) in seen.items() if g == group and v.dx > 0]
          frames[side].append(min(near) if near else np.inf)

  if not n_frames:
    print("nothing to measure")
    return

  print(f"{n_frames} sampled frames above {MIN_SPEED} m/s\n")
  print("-- holding a side whenever the camera sees anything in it within X --")
  for thresh in (10.0, 15.0, 20.0, 30.0, 40.0):
    held = [100 * np.mean(np.array(frames[s]) < thresh) for s in ('left', 'right')]
    print(f"  within {thresh:4.0f} m   left {held[0]:5.1f}% of the drive   right {held[1]:5.1f}%")

  print(f"\n-- what was in the target lane when a lane change began ({len(starts)} of them) --")
  if not starts:
    return
  have = [(s, dx, vx, v) for (s, dx, vx, v) in starts if np.isfinite(dx)]
  print(f"  something ahead in that lane: {len(have)} of {len(starts)}")
  if have:
    a = np.array([(dx, vx, v) for _, dx, vx, v in have])
    print(f"  its distance     median {np.median(a[:, 0]):5.1f} m   min {a[:, 0].min():5.1f}   max {a[:, 0].max():5.1f}")
    print(f"  closing speed    median {np.median(a[:, 1]):+5.1f} m/s  (negative = we are faster)")
    slower = a[a[:, 1] < -0.5]
    print(f"  slower than us   {len(slower)} of {len(have)}")
    if len(slower):
      print(f"    distance       median {np.median(slower[:, 0]):5.1f} m   min {slower[:, 0].min():5.1f}")
      deficit = f"median {np.median(-slower[:, 1]):5.1f} m/s   max {(-slower[:, 1]).max():5.1f}"
      print(f"    speed deficit  {deficit}")
      gap_t = slower[:, 0] / np.maximum(-slower[:, 1], 0.1)
      print(f"    time to close  median {np.median(gap_t):5.1f} s   min {gap_t.min():5.1f}")


if __name__ == '__main__':
  main(sys.argv[1:])
