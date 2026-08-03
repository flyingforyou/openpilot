#!/usr/bin/env python3
"""Grade the two longitudinal controllers against the driver, from logs you already have.

plannerd runs whether or not openpilot owns longitudinal, so every recorded drive already
contains a matched pair: what the car actually did, and what openpilot would have done instead.
The driver's brake and steering overrides are the labels. This prints them.

Two questions it answers, and the second is the one that decides whether the idea is any good:

  1. At the moments the driver took over, was openpilot already asking for something more
     conservative than what the car was doing? Those are the cases openpilot would have caught.
  2. How often is openpilot more conservative WITHOUT the driver intervening? Those are the
     false positives -- the phantom braking a gate on "openpilot is more conservative" would
     have caused. If that count is large, the gate is not worth building.

Usage:  shadow_compare.py <seg>...      (rlog.zst paths, see README)
"""
import os

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys
from collections import Counter

import capnp
import zstandard
from cereal import log as capnp_log

# How much more conservative openpilot has to be before it counts as a real disagreement.
# Below this the two are effectively agreeing and the difference is noise.
DISAGREE_MS2 = 0.5
# Only the lead-up to a takeover is evidence about that takeover.
LOOKBACK_S = 3.0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'sc-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      try:
        yield from capnp_log.Event.read_multiple(f)
      except capnp.KjException:
        pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def main(paths):
  t0 = None
  cur = {'aTarget': 0.0, 'aEgo': 0.0, 'vEgo': 0.0, 'enabled': False,
         'lead': False, 'leadRadar': False, 'dRel': None}
  prev = {'brake': False, 'steer': False, 'gas': False, 'enabled': False}

  history = []          # (dt, aTarget, aEgo, lead, leadRadar, dRel, vEgo)
  takeovers = []        # (dt, cause, worst disagreement in the lookback, context)
  disagree_frames = 0
  engaged_frames = 0
  causes = Counter()

  for path in paths:
    for evt in read_events(path):
      w = evt.which()
      t = evt.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      dt = t - t0

      if w == 'longitudinalPlan':
        cur['aTarget'] = evt.longitudinalPlan.aTarget
      elif w == 'selfdriveState':
        cur['enabled'] = bool(evt.selfdriveState.enabled)
      elif w == 'radarState':
        lead = evt.radarState.leadOne
        cur['lead'] = bool(lead.status)
        cur['leadRadar'] = bool(lead.radar)
        cur['dRel'] = round(lead.dRel, 1) if lead.status else None
      elif w == 'carState':
        cs = evt.carState
        cur['aEgo'], cur['vEgo'] = cs.aEgo, cs.vEgo

        if cur['enabled']:
          engaged_frames += 1
          history.append((dt, cur['aTarget'], cur['aEgo'], cur['lead'],
                          cur['leadRadar'], cur['dRel'], cur['vEgo']))
          if len(history) > 1000:
            history.pop(0)
          # openpilot asking for meaningfully less than the car is doing
          if cur['aTarget'] < cur['aEgo'] - DISAGREE_MS2:
            disagree_frames += 1

        now = {'brake': bool(cs.brakePressed), 'steer': bool(cs.steeringDisengage),
               'gas': bool(cs.gasPressed)}
        for k, v in now.items():
          if v and not prev[k] and prev['enabled']:
            causes[k] += 1
            window = [h for h in history if dt - h[0] <= LOOKBACK_S]
            if window:
              worst = min(window, key=lambda h: h[1] - h[2])
              takeovers.append((dt, k, worst))
        prev.update(now)
        prev['enabled'] = cur['enabled']

  print(f"=== {engaged_frames} engaged carState frames ===\n")

  print("=== driver takeovers, and what openpilot wanted in the 3s before ===")
  if not takeovers:
    print("  none")
  for dt, cause, w in takeovers:
    _, a_tgt, a_ego, lead, radar, d_rel, v_ego = w
    gap = a_tgt - a_ego
    src = ('radar' if radar else 'vision') if lead else 'no lead'
    verdict = "op wanted slower" if gap < -DISAGREE_MS2 else "op agreed"
    print(f"  +{dt:8.1f}s  {cause:5s}  v={v_ego:5.1f}  op={a_tgt:+.2f} actual={a_ego:+.2f} "
          f"({gap:+.2f})  lead={src}"
          + (f" {d_rel}m" if d_rel is not None else "") + f"   -> {verdict}")

  caught = sum(1 for _, _, w in takeovers if w[1] - w[2] < -DISAGREE_MS2)
  print(f"\n  {caught}/{len(takeovers)} takeovers had openpilot already asking for less")

  print("\n=== the cost side: how often would a 'more conservative wins' gate have fired? ===")
  pct = 100.0 * disagree_frames / engaged_frames if engaged_frames else 0.0
  print(f"  {disagree_frames} of {engaged_frames} engaged frames ({pct:.1f}%) had openpilot")
  print(f"  more than {DISAGREE_MS2} m/s^2 below actual, with no takeover to justify it.")
  print("  Read this as the phantom-intervention rate. Small is the case for building the gate;")
  print("  large means the disagreement is mostly openpilot being wrong, not the stock ACC.")


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  main(sys.argv[1:])
