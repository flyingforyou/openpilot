#!/usr/bin/env python3
"""Every disengagement on a route: who ended it, and whether it is worth looking at.

Usage: disengage_review.py <route>...   (e.g. 00000263--5ebbf1768e, segments under OP_LOG_ROOT)

Most disengagements are the driver doing something the car was never asked to do, and reading them
as control problems wastes the analysis. The rule that started this: on 11/10 (00000263 seg 21) the
car was following a slower car 41 m ahead at 63 km/h with a gentle -0.2 m/s^2 when the driver braked.
It looked like late braking. It was a right turn: right blinker 1 s after the brake, 64 -> 27 km/h in
5 s, the wheel to -257 deg at 21 km/h, then a spiral ramp at -270..-380 deg for 40 s. So:

- driver_turn: the wheel goes past TURN_ANGLE_DEG below TURN_MAX_KPH within TURN_WINDOW_S of the
  disengagement, whatever ended it (brake, accelerator, wheel). Not a problem.
- driver_lane_change: a blinker within LANE_CHANGE_WINDOW_S at speed, without a turn. Not a problem
  (00000263 seg 14: a brake tap at 104 km/h, then the right blinker).
- problem: a fault or an alert that was not the driver's -- always listed, even inside a turn.
- review: everything else; these are the ones to read.
"""
import glob
import os
import sys
from dataclasses import dataclass

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))

TURN_WINDOW_S = 10.0
TURN_ANGLE_DEG = 90.0
TURN_MAX_KPH = 40.0
LANE_CHANGE_WINDOW_S = 6.0
LANE_CHANGE_MIN_KPH = 40.0
# events that only describe the driver taking over, never a fault
DRIVER_EVENTS = {'gasPressedOverride', 'steerOverride', 'pedalPressed', 'steerDisengage', 'buttonCancel',
                 'pcmDisable', 'laneChange', 'preLaneChangeLeft', 'preLaneChangeRight', 'laneChangeBlocked',
                 'wrongGear', 'reverseGear', 'seatbeltNotLatched', 'doorOpen', 'manualRestart'}


@dataclass
class Sample:
  t: float          # s from the disengagement
  kph: float
  angle: float      # steering wheel deg, sign as carState
  left: bool
  right: bool


@dataclass
class Disengagement:
  route_t: float
  kph: float
  cause: str        # brake / gas / steer / cancel / unknown
  events: list      # onroadEvents names within 1 s that are not driver-only
  after: list       # Samples from 0 to TURN_WINDOW_S after


def classify(d: Disengagement) -> tuple[str, str]:
  if d.events:
    return 'problem', ', '.join(sorted(set(d.events)))
  turn = [s for s in d.after if s.t <= TURN_WINDOW_S and abs(s.angle) >= TURN_ANGLE_DEG and s.kph <= TURN_MAX_KPH]
  if turn:
    s = max(turn, key=lambda s: abs(s.angle))
    side = 'right' if s.angle < 0 else 'left'
    return 'driver_turn', f'{side} turn, wheel {s.angle:+.0f} deg at {s.kph:.0f} km/h {s.t:.1f} s later'
  blink = [s for s in d.after if s.t <= LANE_CHANGE_WINDOW_S and (s.left or s.right) and s.kph >= LANE_CHANGE_MIN_KPH]
  if blink:
    return 'driver_lane_change', f"{'left' if blink[0].left else 'right'} blinker {blink[0].t:.1f} s later"
  return 'review', ''


def read_route(route: str) -> list[Disengagement]:
  from openpilot.tools.lib.logreader import LogReader
  segs = sorted(glob.glob(f'{LOG_ROOT}/{route}--*/rlog.zst'), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  t0 = None
  cs = []          # (t, kph, angle, left, right, brake, gas, steer, cruise)
  disen = []       # t of each enabled -> disabled
  events = []      # (t, name)
  enabled = None
  for p in segs:
    for m in LogReader(p):
      w = m.which()
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      t -= t0
      if w == 'carState':
        c = m.carState
        if not cs or t - cs[-1][0] >= 0.1:
          cs.append((t, c.vEgo * 3.6, c.steeringAngleDeg, c.leftBlinker, c.rightBlinker, c.brakePressed,
                     c.gasPressed, c.steeringPressed, c.cruiseState.enabled))
      elif w == 'selfdriveState':
        e = m.selfdriveState.enabled
        if enabled and not e:
          disen.append(t)
        enabled = e
      elif w == 'onroadEvents':
        for ev in m.onroadEvents:
          if str(ev.name) not in DRIVER_EVENTS:
            events.append((t, str(ev.name)))
  out = []
  for td in disen:
    near = [x for x in cs if td - 0.5 <= x[0] <= td + 0.2]
    if not near:
      continue
    cause = ('brake' if any(x[5] for x in near) else 'gas' if any(x[6] for x in near) else
             'steer' if any(x[7] for x in near) else 'cancel' if not near[-1][8] else 'unknown')
    after = [Sample(x[0] - td, x[1], x[2], x[3], x[4]) for x in cs if td <= x[0] <= td + TURN_WINDOW_S]
    evs = [n for t, n in events if abs(t - td) <= 1.0]
    out.append(Disengagement(td, near[-1][1], cause, evs, after))
  return out


def main(routes):
  for route in routes:
    rows = read_route(route)
    counts = {}
    print(f'== {route}: {len(rows)} disengagements')
    for d in rows:
      kind, why = classify(d)
      counts[kind] = counts.get(kind, 0) + 1
      seg, sec = divmod(d.route_t, 60)
      print(f'  seg {int(seg):3d} {sec:5.1f}s  {d.kph:5.1f} km/h  {d.cause:7s} {kind:18s} {why}')
    print('  ' + '  '.join(f'{k} {v}' for k, v in sorted(counts.items())))


if __name__ == '__main__':
  main(sys.argv[1:])
