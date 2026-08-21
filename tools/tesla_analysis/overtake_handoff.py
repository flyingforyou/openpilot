#!/usr/bin/env python3
"""Can the gap between "the camera lost it" and "the blind spot found it" be bridged?

Nothing on this car reports a vehicle that is exactly level with us. The factory camera tracks the
next lane down to about 4-5 m and then stops; the blind spot flags are named Rear and behave like
it, set on 2% of the frames where the camera still has a vehicle inside 9 m.

But an overtake is not a mystery in between. The camera hands over a last distance and a relative
speed, and if the blind spot reliably picks the same vehicle up a predictable moment later, the
interval between them is a vehicle known to be alongside without either sensor saying so.

So this asks the two questions that decide whether that inference is sound:

  1. After the camera loses a vehicle it was closing on, does the blind spot always fire?
  2. Is the delay the one the closing speed predicts?

A prediction that holds is a lane-change gate. One that does not is a guess about where a car is,
which is worse than admitting the car cannot see it.

  ./overtake_handoff.py op-logs/00000087--0fb852a22a--*
"""
import sys
from collections import Counter

import numpy as np

from opendbc.can import CANParser
from opendbc.car.tesla.das_object import parse_das_object
from openpilot.tools.lib.logreader import LogReader

DAS_OBJECT = 777
GROUP_LEFT, GROUP_RIGHT = 1, 2
SIDES = {GROUP_LEFT: 'left', GROUP_RIGHT: 'right'}

LOST_AFTER_S = 0.6        # no refresh this long means the camera has dropped it
HANDOFF_MAX_DX = 15.0     # only vehicles lost while close: those are the ones we are passing
MIN_CLOSING = 0.5         # m/s of overtake -- we must actually be passing it
WATCH_S = 12.0            # how long after the loss to wait for the blind spot
MIN_SPEED = 8.0
MIRROR_DX = -2.0          # roughly where a vehicle sits when the blind spot ought to see it
CLEAR_DX = -6.0           # behind our own tail: past here a lane change is no longer into it


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  cp = CANParser("tesla_can", [], 2)
  cp_ch = CANParser("tesla_can", [], 0)

  v_ego, t0 = 0.0, None
  bs = {'left': False, 'right': False}
  edges = {'left': [], 'right': []}      # times the flag went false -> true
  seen: dict = {}                        # (group, id) -> (t, dx, vx_rel)
  losses = []                            # (t, side, dx, vx_rel)

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
        for side, now in (('left', bool(msg.carState.leftBlindspot)),
                          ('right', bool(msg.carState.rightBlindspot))):
          if now and not bs[side]:
            edges[side].append(t - t0)
          bs[side] = now
      elif w == 'can':
        frames = [(x.address, bytes(x.dat), x.src) for x in msg.can]
        if not any(a == DAS_OBJECT and s == 2 for a, _, s in frames):
          continue
        cp.update([(msg.logMonoTime, frames)])
        cp_ch.update([(msg.logMonoTime, frames)])
        rel = t - t0
        for veh in parse_das_object(cp.vl["DAS_object"]):
          if veh.group in SIDES:
            seen[(veh.group, veh.obj_id)] = (rel, float(veh.dx), float(veh.vx_rel))

        for key in list(seen):
          last_t, last_dx, last_vx = seen[key]
          if rel - last_t < LOST_AFTER_S:
            continue
          del seen[key]
          # Lost while close, while we were passing it, and while actually moving
          if last_dx < HANDOFF_MAX_DX and last_vx < -MIN_CLOSING and v_ego > MIN_SPEED:
            losses.append((last_t, SIDES[key[0]], last_dx, last_vx))

  if not losses:
    print("no overtakes of this shape in this route")
    return

  print(f"{len(losses)} vehicles lost by the camera inside {HANDOFF_MAX_DX:.0f} m while being passed\n")

  found, delays, predicted, misses = 0, [], [], Counter()
  for (t_lost, side, dx, vx) in losses:
    after = [e for e in edges[side] if t_lost <= e <= t_lost + WATCH_S]
    pred = (dx - MIRROR_DX) / abs(vx)
    predicted.append(pred)
    if after:
      found += 1
      delays.append(after[0] - t_lost)
    else:
      misses[side] += 1

  print("1. after the camera loses it, does the blind spot fire?")
  print(f"   {found} of {len(losses)}  ({100 * found / len(losses):.0f}%)"
        f"   never fired: {dict(misses)}")

  if delays:
    d = np.array(delays)
    print(f"\n2. how long afterwards")
    print(f"   median {np.median(d):.1f}s   p10 {np.percentile(d, 10):.1f}   "
          f"p90 {np.percentile(d, 90):.1f}   max {d.max():.1f}")

    p = np.array([pr for pr, (tl, s, _, _) in zip(predicted, losses, strict=True)
                  if any(tl <= e <= tl + WATCH_S for e in edges[s])])
    err = d - p
    print(f"\n3. does the closing speed predict it?")
    print(f"   predicted median {np.median(p):.1f}s against measured {np.median(d):.1f}s")
    print(f"   error   median {np.median(err):+.1f}s   p10 {np.percentile(err, 10):+.1f}   "
          f"p90 {np.percentile(err, 90):+.1f}")
    print(f"   within 1s of prediction on {100 * np.mean(np.abs(err) < 1.0):.0f}% of them")

  # The other way to use this: do not wait for the blind spot at all. Take the last distance and
  # the closing speed, work out how long the vehicle needs to clear our tail, and refuse a lane
  # change for exactly that long. It cannot get stuck on a flag that never comes -- but it can
  # run very long when we are barely passing, which is the number that decides it.
  print(f"\n-- blocking for as long as it takes the vehicle to clear {abs(CLEAR_DX):.0f} m behind --")
  wins = np.array([(dx - CLEAR_DX) / abs(vx) for (_, _, dx, vx) in losses])
  drive_s = max(e for side in edges.values() for e in side) if any(edges.values()) else 0.0
  print(f"  window   median {np.median(wins):5.1f}s  p90 {np.percentile(wins, 90):5.1f}  max {wins.max():5.1f}")
  for cap in (5.0, 8.0, 12.0):
    capped = np.minimum(wins, cap)
    over = 100 * np.mean(wins > cap)
    print(f"  capped at {cap:4.1f}s: median {np.median(capped):4.1f}s  total blocked "
          f"{capped.sum():6.0f}s over {drive_s / 60:.0f} min  ({100 * capped.sum() / max(drive_s, 1):4.1f}% of the drive)"
          f"   cap hit on {over:.0f}%")


if __name__ == '__main__':
  main(sys.argv[1:])
