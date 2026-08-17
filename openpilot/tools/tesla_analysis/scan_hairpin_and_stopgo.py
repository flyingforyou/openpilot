#!/usr/bin/env python3
"""Scan every pulled segment for two things the user flagged after driving with CarrotPilot long:

1. Steering faults (steerFaultTemporary/Permanent, steeringDisengage) during high-curvature
   driving, despite having raised the lateral force/accel limit.
2. Hard braking in stop-and-go traffic while the lead was still comfortably far away.

Read-only, no code changes. Prints candidate episodes with enough context (route, segment,
route-relative time, key signal values) to go pull the exact window for a closer look.
"""
import os
import sys
import glob
import re
from collections import deque
from multiprocessing import Pool

import capnp
import zstandard
from openpilot.cereal import log as capnp_log

LOG_ROOT = os.environ.get("OP_LOG_ROOT", os.path.expanduser("~/openpilot/op-logs-carrotlong"))
SCRATCH = '/tmp/op-analysis'

MPH = 2.23694  # vEgo is m/s


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'w-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def scan_segment(seg_dir):
  route = re.sub(r'--\d+$', '', os.path.basename(seg_dir))
  seg_n = int(re.search(r'--(\d+)$', seg_dir).group(1))
  rlog = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog):
    return route, seg_n, [], []

  steer_events = []
  stopgo_events = []

  t0 = None
  engaged = False
  steering_angle = 0.0
  v_ego = 0.0
  a_ego = 0.0
  fault_active = False
  fault_start_t = None
  fault_start_angle = 0.0
  fault_start_vego = 0.0
  fault_kind = None

  lead_status = False
  lead_drel = 0.0
  a_target = 0.0
  x_state = -1
  # Rolling low-speed history to recognise "stop and go" rather than a single approach-to-stop.
  low_speed_transitions = deque(maxlen=200)  # (t, v_ego) sampled at carState rate
  last_low = False
  brake_cooldown_until = -999.0

  for evt in read_events(rlog):
    w = evt.which()
    if t0 is None and w == 'carState':
      t0 = evt.logMonoTime
    if t0 is None:
      continue
    t = (evt.logMonoTime - t0) / 1e9

    if w == 'selfdriveState':
      engaged = evt.selfdriveState.enabled

    if w == 'carState':
      cs = evt.carState
      steering_angle = cs.steeringAngleDeg
      v_ego = cs.vEgo
      a_ego = cs.aEgo

      is_low = v_ego < 3.0  # ~6.7mph, "stopped or crawling"
      if is_low and not last_low:
        low_speed_transitions.append(t)
      last_low = is_low

      fault_now = bool(cs.steerFaultTemporary or cs.steerFaultPermanent or cs.steeringDisengage)
      if fault_now and not fault_active and engaged:
        fault_active = True
        fault_start_t = t
        fault_start_angle = steering_angle
        fault_start_vego = v_ego
        fault_kind = ('permanent' if cs.steerFaultPermanent else
                      'disengage' if cs.steeringDisengage else 'temporary')
      elif not fault_now and fault_active:
        steer_events.append({
          'seg': seg_n, 't': fault_start_t, 'dur': t - fault_start_t,
          'angle': fault_start_angle, 'vego_mph': fault_start_vego * MPH, 'kind': fault_kind,
        })
        fault_active = False

    if w == 'radarState':
      lead_status = bool(evt.radarState.leadOne.present)
      lead_drel = float(evt.radarState.leadOne.dRel)

    if w == 'longitudinalPlan':
      a_target = float(evt.longitudinalPlan.aTarget)
      x_state = int(evt.longitudinalPlan.xState)

      # Recent stop-and-go: at least 2 low-speed entries in the last 45s (i.e. we've already
      # stopped/crawled and resumed at least once before this braking event).
      recent_low_count = sum(1 for lt in low_speed_transitions if t - lt < 45.0)
      is_stopgo_context = recent_low_count >= 2

      hard_decel = min(a_target, a_ego) < -1.5
      if (engaged and is_stopgo_context and lead_status and lead_drel > 6.0 and
          hard_decel and t > brake_cooldown_until):
        stopgo_events.append({
          'seg': seg_n, 't': t, 'dRel': round(lead_drel, 1), 'vego_mph': round(v_ego * MPH, 1),
          'aTarget': round(a_target, 2), 'aEgo': round(a_ego, 2), 'xState': x_state,
          'recent_low_count': recent_low_count,
        })
        brake_cooldown_until = t + 5.0  # one report per braking event, not one per frame

  if fault_active:
    steer_events.append({
      'seg': seg_n, 't': fault_start_t, 'dur': -1.0,  # still active at segment end
      'angle': fault_start_angle, 'vego_mph': fault_start_vego * MPH, 'kind': fault_kind,
    })

  return route, seg_n, steer_events, stopgo_events


def main():
  seg_dirs = sorted(glob.glob(os.path.join(LOG_ROOT, '*--*')))
  seg_dirs = [d for d in seg_dirs if os.path.isdir(d) and '--' in os.path.basename(d)]
  print(f"scanning {len(seg_dirs)} segments in {LOG_ROOT}", file=sys.stderr)

  by_route_steer = {}
  by_route_stopgo = {}

  with Pool(8) as pool:
    for route, seg_n, steer_events, stopgo_events in pool.imap_unordered(scan_segment, seg_dirs):
      if steer_events:
        by_route_steer.setdefault(route, []).extend((seg_n, e) for e in steer_events)
      if stopgo_events:
        by_route_stopgo.setdefault(route, []).extend((seg_n, e) for e in stopgo_events)

  print("\n=== STEERING FAULTS (engaged, steerFaultTemporary/Permanent/steeringDisengage) ===")
  total_steer = 0
  for route in sorted(by_route_steer):
    events = sorted(by_route_steer[route], key=lambda x: (x[0], x[1]['t']))
    total_steer += len(events)
    print(f"\n{route}: {len(events)} episodes")
    for seg_n, e in events:
      route_t = seg_n * 60 + e['t']
      print(f"  seg {seg_n:3d}  route_t={route_t:7.1f}s  dur={e['dur']:5.2f}s  "
            f"kind={e['kind']:10s}  angle={e['angle']:7.1f}deg  v={e['vego_mph']:5.1f}mph")

  print(f"\ntotal steering-fault episodes: {total_steer}")

  print("\n\n=== STOP-AND-GO HARD BRAKING (lead present, dRel>8m, recent low-speed history) ===")
  total_stopgo = 0
  for route in sorted(by_route_stopgo):
    events = sorted(by_route_stopgo[route], key=lambda x: (x[0], x[1]['t']))
    total_stopgo += len(events)
    print(f"\n{route}: {len(events)} episodes")
    for seg_n, e in events:
      route_t = seg_n * 60 + e['t']
      print(f"  seg {seg_n:3d}  route_t={route_t:7.1f}s  dRel={e['dRel']:5.1f}m  v={e['vego_mph']:5.1f}mph  "
            f"aTarget={e['aTarget']:6.2f}  aEgo={e['aEgo']:6.2f}  xState={e['xState']}  "
            f"recent_low={e['recent_low_count']}")

  print(f"\ntotal stop-and-go hard-brake episodes: {total_stopgo}")


if __name__ == '__main__':
  main()
