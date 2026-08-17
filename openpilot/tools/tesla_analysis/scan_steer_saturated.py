#!/usr/bin/env python3
"""Find where openpilot ran out of steering in a turn, and what happened next.

The user's report: in hairpins a lateral alert appears, they add a little steering through
cooperative steering, and sometimes it ends in a disengage. steerSaturated is that alert
("Take Control / Turn Exceeds Steering Limit"), raised after CP.steerLimitTimer (0.4s) of
continuous saturation above 5 m/s while the driver is *not* pressing -- see latcontrol.py.

For Tesla, saturation is steer_limited_by_safety, computed in controlsd as

    abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > 2.5 deg

i.e. the angle the controller asked for versus the angle the carcontroller actually emitted after
apply_steer_angle_limits_vm. A gap there means the rate/accel limiter is clipping the request.
Both sides must be read from their own message -- carControl for the request, carOutput for what
was emitted. Reading the request twice makes every gap look like zero.

Because _check_saturation stops accumulating the moment steeringPressed goes true, the alert
clears as soon as the driver reacts. So the alert and the later disengage are usually separate
episodes, and this links them: for each alert it reports whether a disengage followed within
LINK_WINDOW seconds, which is the sequence the user described.
"""
import os
import sys
import glob
import re
from multiprocessing import Pool

import capnp
import zstandard
from openpilot.cereal import log as capnp_log

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/openpilot/op-logs-carrotlong'))
SCRATCH = '/tmp/op-analysis'
MPH = 2.23694  # vEgo is m/s
LINK_WINDOW = 8.0  # seconds after an alert to still call a disengage "the same event"


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

  alerts = []
  disengages = []

  t0 = None
  engaged = False
  angle = torque = v_ego = 0.0
  pressed = False
  angle_req = angle_out = 0.0
  sat_active = False
  ep = None
  dis_prev = False

  for evt in read_events(rlog):
    w = evt.which()
    if t0 is None and w == 'carState':
      t0 = evt.logMonoTime
    if t0 is None:
      continue
    t = (evt.logMonoTime - t0) / 1e9

    if w == 'selfdriveState':
      engaged = evt.selfdriveState.enabled
    elif w == 'carState':
      cs = evt.carState
      angle, torque, v_ego = cs.steeringAngleDeg, cs.steeringTorque, cs.vEgo
      pressed = bool(cs.steeringPressed)
      dis_now = bool(cs.steeringDisengage)
      if dis_now and not dis_prev and engaged:
        disengages.append({'t': t, 'angle': angle, 'torque': torque, 'v_mph': v_ego * MPH})
      dis_prev = dis_now
    elif w == 'carControl':
      angle_req = float(evt.carControl.actuators.steeringAngleDeg)
    elif w == 'carOutput':
      angle_out = float(evt.carOutput.actuatorsOutput.steeringAngleDeg)
    elif w == 'onroadEvents':
      has_sat = any('steerSaturated' in str(e.name) for e in evt.onroadEvents)
      if has_sat and not sat_active and engaged:
        sat_active = True
        ep = {'seg': seg_n, 't': t, 'angle': angle, 'v_mph': v_ego * MPH,
              'req': angle_req, 'out': angle_out, 'max_clip': abs(angle_req - angle_out),
              'max_torque': abs(torque), 'pressed_during': pressed, 'dur': 0.0}
      elif not has_sat and sat_active:
        ep['dur'] = t - ep['t']
        alerts.append(ep)
        sat_active = False
        ep = None

    if sat_active and ep is not None:
      ep['max_clip'] = max(ep['max_clip'], abs(angle_req - angle_out))
      ep['max_torque'] = max(ep['max_torque'], abs(torque))
      ep['pressed_during'] = ep['pressed_during'] or pressed

  if sat_active and ep is not None:
    ep['dur'] = -1.0
    alerts.append(ep)

  # link each alert to a disengage that followed it closely
  for a in alerts:
    a['followed_by_disengage'] = None
    for d in disengages:
      if 0 <= d['t'] - a['t'] <= LINK_WINDOW:
        a['followed_by_disengage'] = round(d['t'] - a['t'], 2)
        break

  return route, seg_n, alerts, disengages


def main():
  seg_dirs = sorted(glob.glob(os.path.join(LOG_ROOT, '*--*')))
  seg_dirs = [d for d in seg_dirs if os.path.isdir(d)]
  print(f"scanning {len(seg_dirs)} segments", file=sys.stderr)

  by_route = {}
  n_dis_total = 0
  with Pool(8) as pool:
    for route, seg_n, alerts, dis in pool.imap_unordered(scan_segment, seg_dirs):
      n_dis_total += len(dis)
      if alerts:
        by_route.setdefault(route, []).extend(alerts)

  total = linked = clipped = 0
  print('=== steerSaturated ("Turn Exceeds Steering Limit") while engaged ===')
  print('  req  = angle the controller asked for')
  print('  out  = angle the carcontroller actually emitted (after rate/accel limiting)')
  print('  clip = max |req-out| during the alert; >2.5 means the limiter was the binding thing\n')
  for route in sorted(by_route):
    eps = sorted(by_route[route], key=lambda e: (e['seg'], e['t']))
    total += len(eps)
    print(f"{route}: {len(eps)} episodes")
    for e in eps:
      clipped += int(e['max_clip'] > 2.5)
      link = ''
      if e['followed_by_disengage'] is not None:
        linked += 1
        link = f"  <-- DISENGAGE +{e['followed_by_disengage']}s"
      print(f"  seg {e['seg']:3d} t={e['seg']*60+e['t']:7.1f}s dur={e['dur']:5.2f}s v={e['v_mph']:5.1f}mph "
            f"angle={e['angle']:+7.1f} req={e['req']:+7.1f} out={e['out']:+7.1f} "
            f"clip={e['max_clip']:5.2f} maxtq={e['max_torque']:4.2f}{link}")
    print()

  print(f"total steerSaturated episodes: {total}")
  print(f"  of which the rate/accel limiter was clipping (>2.5deg): {clipped}")
  print(f"  of which a disengage followed within {LINK_WINDOW}s: {linked}")
  print(f"total steering disengages seen: {n_dis_total}")


if __name__ == '__main__':
  main()
