#!/usr/bin/env python3
"""Does the gateway actually broadcast a stop line, and would it have helped?

carrot infers a stop entirely from the driving model's predicted speed: a low predicted speed
ahead is called red, a predicted speed above 5 m/s is called green. There is no notion of a stop
sign in it at all, which is why a stop can release itself the moment the model starts predicting
that we carry on -- measured, 12 of 18 stopping episodes ended without the car ever stopping, and
mostly while trafficState still read red or off.

But UI_driverAssistRoadSign has slots for exactly this: a stop sign's stop line and a traffic
light's stop line, each with a distance and a confidence. If those are populated on this car they
are a real detection rather than an inference from a speed profile.

So: are they broadcast, how far out, how confident, and were they present at the moments the car
was slowing for something?

  ./stop_line_survey.py op-logs/0000007f--ed761c79a7--*
"""
import sys
from collections import Counter

import numpy as np

from openpilot.tools.lib.logreader import LogReader

ROAD_SIGN = 568
MUX_STOP_SIGN, MUX_TRAFFIC_LIGHT = 1, 2
X_E2E_STOP, X_E2E_STOPPED = 3, 5


def sig(payload: bytes, start: int, length: int, scale: float, offset: float) -> float:
  raw = int.from_bytes(payload.ljust(8, b'\x00')[:8], 'little')
  return ((raw >> start) & ((1 << length) - 1)) * scale + offset


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  mux_seen = Counter()
  stop_sign, traffic_light = [], []      # (dist, conf)
  during_stop = {'sign': 0, 'light': 0, 'neither': 0}
  x_state = 0
  latest = {'sign': None, 'light': None}
  frames_stopping = 0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'longitudinalPlan':
        x_state = int(msg.longitudinalPlan.xState)
        if x_state in (X_E2E_STOP, X_E2E_STOPPED):
          frames_stopping += 1
          got_sign = latest['sign'] is not None and latest['sign'][1] > 0
          got_light = latest['light'] is not None and latest['light'][1] > 0
          if got_sign:
            during_stop['sign'] += 1
          elif got_light:
            during_stop['light'] += 1
          else:
            during_stop['neither'] += 1
      elif w == 'can':
        for c in msg.can:
          if c.address != ROAD_SIGN or c.src > 2:
            continue
          d = bytes(c.dat)
          mux = int(sig(d, 0, 8, 1, 0))
          mux_seen[mux] += 1
          if mux == MUX_STOP_SIGN:
            dist = sig(d, 8, 10, 0.25, -8.0)
            conf = sig(d, 18, 7, 1, 0)
            stop_sign.append((dist, conf))
            latest['sign'] = (dist, conf) if conf > 0 else None
          elif mux == MUX_TRAFFIC_LIGHT:
            dist = sig(d, 8, 10, 0.25, -8.0)
            conf = sig(d, 18, 7, 1, 0)
            traffic_light.append((dist, conf))
            latest['light'] = (dist, conf) if conf > 0 else None

  print(f"UI_roadSign selector values seen: {dict(sorted(mux_seen.items()))}")
  print("  (1 = stop sign stop line, 2 = traffic light stop line)\n")

  for label, data in (("stop sign", stop_sign), ("traffic light", traffic_light)):
    if not data:
      print(f"  {label:14} never broadcast")
      continue
    a = np.array(data)
    live = a[a[:, 1] > 0]
    pct = f"({100 * len(live) / len(a):.1f}%)"
    print(f"  {label:14} {len(a)} frames, {len(live)} with confidence > 0 {pct}")
    if len(live):
      d_tail = f"p10 {np.percentile(live[:, 0], 10):5.1f}  max {live[:, 0].max():6.1f}"
      print(f"    distance   median {np.median(live[:, 0]):6.1f} m  {d_tail}")
      c_tail = f"p10 {np.percentile(live[:, 1], 10):4.0f}   max {live[:, 1].max():4.0f}"
      print(f"    confidence median {np.median(live[:, 1]):5.0f}   {c_tail}")

  print(f"\nwhile the planner was stopping ({frames_stopping} frames):")
  total = max(sum(during_stop.values()), 1)
  for k, n in during_stop.items():
    print(f"  {k:8} {n:6}  ({100 * n / total:5.1f}%)")


if __name__ == '__main__':
  main(sys.argv[1:])
