#!/usr/bin/env python3
"""The car slows for a stop and then pulls away without having stopped. What let it go?

carrot has no notion of a stop sign. trafficState is inferred entirely from the driving model's
predicted speed profile: a low predicted speed ahead is called red, and a predicted speed above
5 m/s -- or 2 m/s more than now -- is called green. So "green" does not mean a light turned; it
means the model has started predicting that we carry on.

Which is exactly what the model does once the car has slowed enough that carrying on is the
likelier future. The stop releases itself.

This finds every stopping episode and reports what ended it, and crucially whether the car had
actually come to a stop when it did.

  ./stop_release.py op-logs/0000007f--ed761c79a7--*
"""
import sys
from collections import Counter

import numpy as np

from openpilot.tools.lib.logreader import LogReader

# carrot's XState
X_LEAD, X_CRUISE, X_E2E_CRUISE, X_E2E_STOP, X_E2E_PREPARE, X_E2E_STOPPED = range(6)
STOPPING = (X_E2E_STOP, X_E2E_STOPPED)
NAMES = {X_LEAD: 'lead', X_CRUISE: 'cruise', X_E2E_CRUISE: 'e2eCruise',
         X_E2E_STOP: 'e2eStop', X_E2E_PREPARE: 'e2ePrepare', X_E2E_STOPPED: 'e2eStopped'}
TRAFFIC = {0: 'off', 1: 'red', 2: 'green'}

STOPPED_MPS = 0.3


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  rows = []
  v_ego, t0, route_t0 = 0.0, None, None
  lead_d = lead_v = None

  for p in paths:
    seg = seg_no(p)
    seg_t0 = None
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w in ('carState', 'longitudinalPlan'):
        seg_t0 = t if seg_t0 is None else min(seg_t0, t)
        route_t0 = t if route_t0 is None else min(route_t0, t)
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'radarState':
        one = msg.radarState.leadOne
        lead_d = float(one.dRel) if one.present else None
        lead_v = float(one.vLead) if one.present else None
      elif w == 'longitudinalPlan':
        lp = msg.longitudinalPlan
        # Both clocks: segment-relative to find the moment on video, route-relative to measure
        # a duration. Using the segment clock for both made an episode that crossed a boundary
        # report a negative length, which is how this bug announced itself.
        rows.append((seg, t - (seg_t0 or t), t - (route_t0 or t), int(lp.xState),
                     int(lp.trafficState), v_ego, lead_d, lead_v, float(lp.desiredDistance)))

  if not rows:
    print("no plan frames")
    return

  episodes = []
  cur = None
  for (seg, rel, abs_t, x_state, traffic, v, lead_d, lead_v, stop_d) in rows:
    if x_state in STOPPING:
      if cur is None:
        cur = {'seg': seg, 'start': rel, 'abs_start': abs_t, 'v0': v, 'vmin': v,
               'traffic': Counter(), 'states': set()}
      cur['vmin'] = min(cur['vmin'], v)
      cur['traffic'][traffic] += 1
      cur['states'].add(x_state)
    elif cur is not None:
      cur.update(end=rel, held=abs_t - cur['abs_start'], exit_state=x_state,
                 exit_traffic=traffic, exit_v=v, lead_d_rel=lead_d, lead_v_lead=lead_v,
                 stop_dist=stop_d)
      episodes.append(cur)
      cur = None

  print(f"{len(episodes)} stopping episodes\n")
  if not episodes:
    return

  head = f"  {'seg':>4} {'at':>6} {'held':>6} {'entered at':>11} {'slowest':>8} {'left at':>8}"
  print(f"{head} {'ended as':>10} {'traffic then':>13}  stopped?")
  stopped, released = 0, 0
  for e in episodes:
    did_stop = e['vmin'] < STOPPED_MPS
    stopped += did_stop
    released += not did_stop
    speeds = f"{e['v0'] * 2.23694:9.0f}mph {e['vmin'] * 2.23694:6.0f}mph {e['exit_v'] * 2.23694:6.0f}mph"
    tail = f"{NAMES.get(e['exit_state'], '?'):>10} {TRAFFIC.get(e['exit_traffic'], '?'):>13}"
    verdict = 'yes' if did_stop else 'NO'
    seek = f"seg {e['seg']} t={max(0, e['start'] - 4):.0f}"
    print(f"  {e['seg']:>4} {e['start']:6.1f} {e['held']:5.1f}s {speeds} {tail}  {verdict:>3}  {seek}")

  print(f"\n  came to a stop      {stopped}")
  print(f"  released while moving {released}")
  by_exit = Counter(NAMES.get(e['exit_state'], '?') for e in episodes)
  by_traffic = Counter(TRAFFIC.get(e['exit_traffic'], '?') for e in episodes)

  print(f"  ended as            {dict(by_exit)}")
  print(f"  trafficState then   {dict(by_traffic)}")

  print("\n-- B: the ones that ended as `lead`. What was that lead? --")
  leads = [e for e in episodes if e['exit_state'] == X_LEAD]
  if leads:
    print(f"  {'seg':>4} {'at':>6}  lead dRel  lead vLead  our v   stop dist  was it stopped?")
    for e in leads:
      d = e.get('lead_d_rel')
      vl = e.get('lead_v_lead')
      sd = e.get('stop_dist')
      stopped_car = vl is not None and vl < 1.0
      cols = f"{'  n/a' if d is None else f'{d:8.1f}m'}  {'  n/a' if vl is None else f'{vl:9.1f}'}"
      cols += f"  {e['exit_v']:5.1f}  {'  n/a' if sd is None else f'{sd:9.1f}m'}"
      print(f"  {e['seg']:>4} {e['end']:6.1f}  {cols}   {'yes' if stopped_car else 'NO -- still moving'}")
    moving_leads = [e for e in leads if (e.get('lead_v_lead') or 99) >= 1.0]
    print(f"\n  of {len(leads)} lead exits, {len(moving_leads)} handed over to a moving vehicle"
          )

  moving = [e for e in episodes if e['vmin'] >= STOPPED_MPS]
  if moving:
    a = np.array([e['vmin'] * 2.23694 for e in moving])
    slowest = f"median {np.median(a):.0f} mph, min {a.min():.0f}, max {a.max():.0f}"
    print(f"\n  of the ones released while still moving: slowest reached {slowest}")
    print(f"  trafficState at release {dict(Counter(TRAFFIC.get(e['exit_traffic'], '?') for e in moving))}")


if __name__ == '__main__':
  main(sys.argv[1:])
