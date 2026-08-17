#!/usr/bin/env python3
"""Build a web visualiser that plays a drive back with both longitudinal controllers on one chart.

plannerd runs whether or not openpilot owns longitudinal, so a drive done on the factory ACC
already contains a matched pair at every instant: what the car actually did (carState.aEgo) and
what openpilot would have commanded instead (longitudinalPlan.aTarget). shadow_compare.py counts
those disagreements; this one lets you watch them next to the road video, which is the only way
to tell a real catch from a shadow of a passing car.

Both traces are drawn on one axis, normalised by the strongest command the car will accept
(|CarControllerParams.ACCEL_MIN|), so -1.0 is full braking authority and the two share a single
linear scale -- which is what makes the gap between the lines directly readable as "how much
more openpilot wanted". Raw m/s^2 stays in the JSON and in the readout; normalising is for
comparing shapes, not for reading values off.

Video comes from the device's own tuning server, which already remuxes qcamera into MP4
(selfdrive/debug/video_source.py); this only caches the result. Pass --no-video to skip it and
get the charts alone.

  tools/tesla_analysis/shadow_viz.py <route>
  OP_DEVICE=172.16.2.22 tools/tesla_analysis/shadow_viz.py 00000025--e4e41713d7
  tools/tesla_analysis/shadow_viz.py <route> --no-video --out /tmp/viz
"""

import os

# Where the copied route segments live, where to stage decompressed rlogs, and where the device
# is. Override with the environment rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
DEVICE = os.environ.get('OP_DEVICE', '172.16.2.22')
PORT = os.environ.get('OP_DEVICE_PORT', '8088')

import argparse
import glob
import json
import re
import shutil
import sys
import urllib.error
import urllib.request

import capnp
import zstandard
from openpilot.cereal import log as capnp_log

# The command range the car accepts, from CarControllerParams. Normalising by the authority the
# controller actually has is what makes 1.0 mean something -- a fixed pretty number would not.
ACCEL_MAX = 2.0
ACCEL_MIN = -3.48

LONG_MPC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'selfdrive', 'controls', 'lib',
                        'longitudinal_mpc_lib', 'long_mpc.py')


def follow_targets(gap_profile: int = 0) -> tuple[dict[int, float], float]:
  """The gap -> tFollow table and stop distance the planner actually uses.

  Read out of long_mpc.py rather than copied here: the whole point of the chart is to show what
  openpilot would have done, and a table that silently drifts from the controller's would make
  it show something else. Parsed rather than imported because importing long_mpc pulls in acados
  and casadi, which a workstation doing log analysis has no reason to have.

  gap_profile must match what the car had stored, or the target line is for a car that was never
  driven -- the knob positions mean different following times under each profile.
  """
  import ast
  tree = ast.parse(open(LONG_MPC).read())
  found: dict[str, object] = {}
  for node in tree.body:                      # module level only; later defs win, as at runtime
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
      name = getattr(node.targets[0], 'id', None)
      if name in ('TESLA_GAP_T_FOLLOW', 'STOP_DISTANCE', 'MIN_T_FOLLOW', 'GAP_PROFILES'):
        found[name] = ast.literal_eval(node.value)

  base = found['TESLA_GAP_T_FOLLOW']
  profiles = found['GAP_PROFILES']
  min_t = found['MIN_T_FOLLOW']
  _, shift, spread = profiles.get(gap_profile, profiles[0])
  mid = base[4]
  table = {g: round(max(mid + (v - mid) * spread + shift, min_t), 3) for g, v in base.items()}
  return table, found['STOP_DISTANCE']


def device_gap_profile() -> int | None:
  """What the car actually had stored, if it is reachable. Guessing here is worse than asking."""
  try:
    import subprocess
    out = subprocess.run(['ssh', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes',
                          f'comma@{DEVICE}', 'cat /data/params/d/GapProfile'],
                         capture_output=True, text=True, timeout=15)
    return int(out.stdout.strip()) if out.returncode == 0 and out.stdout.strip() else None
  except Exception:
    return None

# Chart resolution. longitudinalPlan runs at 20Hz, so this samples it without inventing detail;
# carState (100Hz) is held between samples.
HZ = 20
DT = 1.0 / HZ

# How much more braking openpilot has to ask for before it counts as a real disagreement rather
# than the two controllers tracking each other. Matches shadow_compare.py.
DISAGREE_MS2 = 0.5


def seg_no(path: str) -> int:
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path: str):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'viz-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    # Read the whole staged rlog and parse from bytes. A segment cut short by the car losing
    # power leaves a truncated final message, and the streaming reader aborts on it inside
    # libkj -- a C++ terminate that no Python except can catch, killing the whole run. Parsing
    # from bytes raises a normal KjException instead, after yielding every complete event.
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def extract_segment(path: str) -> dict:
  """Resample one segment onto a uniform timeline, holding the last value of each signal.

  t is seconds from the segment's first frame, which is also where qcamera starts, so it doubles
  as the video's currentTime.
  """
  cur = {
    'aAct': 0.0, 'aPlan': 0.0, 'vEgo': 0.0, 'vPlan': 0.0,
    'dRel': None, 'vRel': None, 'lead': False, 'leadRadar': False,
    'eng': False, 'brake': False, 'gas': False, 'gap': 0,
  }
  t0 = None
  next_t = 0.0
  rows: list[list] = []
  seen_plan = False

  # A segment's rlog opens with initData, whose logMonoTime is the route's start rather than the
  # segment's -- taking it as t0 pads every segment after the first with a minute of empty rows
  # and slides the whole chart off the video. Start the clock at the first real data frame, which
  # is also where qcamera starts.
  DATA = ('carState', 'longitudinalPlan', 'radarState', 'selfdriveState')

  for evt in read_events(path):
    w = evt.which()
    if w not in DATA:
      continue
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0

    if w == 'carState':
      cs = evt.carState
      cur['aAct'] = cs.aEgo
      cur['vEgo'] = cs.vEgo
      cur['brake'] = bool(cs.brakePressed)
      cur['gas'] = bool(cs.gasPressed)
      cur['gap'] = int(cs.cruiseState.gapAdjust)
    elif w == 'longitudinalPlan':
      lp = evt.longitudinalPlan
      cur['aPlan'] = lp.aTarget
      cur['vPlan'] = lp.speeds[0] if len(lp.speeds) else 0.0
      seen_plan = True
    elif w == 'radarState':
      lead = evt.radarState.leadOne
      cur['lead'] = bool(lead.present)
      cur['leadRadar'] = bool(lead.radar)
      cur['dRel'] = round(lead.dRel, 1) if lead.present else None
      # Closing speed separates "settled behind a lead" from "still catching up". Without it a
      # median over every lead frame counts the approach, which has nothing to do with how
      # close the controller actually sits.
      cur['vRel'] = round(lead.vRel, 2) if lead.present else None
    elif w == 'selfdriveState':
      cur['eng'] = bool(evt.selfdriveState.enabled)

    # Emit on a fixed grid so the chart's x axis is real time, not event order.
    while dt >= next_t:
      rows.append([
        round(next_t, 2),
        round(cur['aAct'], 3), round(cur['aPlan'], 3),
        round(cur['vEgo'], 2), round(cur['vPlan'], 2),
        cur['dRel'],
        (1 if cur['lead'] else 0) | (2 if cur['leadRadar'] else 0) | (4 if cur['eng'] else 0)
        | (8 if cur['brake'] else 0) | (16 if cur['gas'] else 0),
        cur['gap'],
        cur['vRel'],
      ])
      next_t += DT

  return {'rows': rows, 'hasPlan': seen_plan}


def summarise(rows: list[list]) -> dict:
  """How far apart the two controllers were, and where the worst of it was.

  Only counts frames with a lead: without one the planner is tracking set speed, where a
  disagreement means nothing.
  """
  worst = 0.0
  worst_t = None
  disagree = 0
  lead_frames = 0
  for r in rows:
    t, a_act, a_plan, _v, _vp, d_rel, flags, _gap, _v_rel = r
    if not (flags & 1):
      continue
    lead_frames += 1
    # Negative = openpilot wanted more braking than the car delivered.
    gap = a_plan - a_act
    if gap < -DISAGREE_MS2:
      disagree += 1
    if gap < worst:
      worst, worst_t = gap, t
  return {
    'worst': round(worst, 2), 'worstT': worst_t,
    'disagreeFrames': disagree, 'leadFrames': lead_frames,
    'disagreePct': round(100.0 * disagree / lead_frames, 1) if lead_frames else 0.0,
  }


def fetch_video(route: str, seg: int, dest: str) -> bool:
  if os.path.exists(dest) and os.path.getsize(dest) > 0:
    return True
  url = f'http://{DEVICE}:{PORT}/v/{route}/{seg}.mp4'
  try:
    with urllib.request.urlopen(url, timeout=180) as r, open(dest + '.part', 'wb') as f:
      shutil.copyfileobj(r, f)
    os.replace(dest + '.part', dest)
    return True
  except (urllib.error.URLError, OSError, TimeoutError) as e:
    if os.path.exists(dest + '.part'):
      os.remove(dest + '.part')
    print(f'    segment {seg}: no video ({type(e).__name__})')
    return False


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument('route', help='route name, e.g. 00000025--e4e41713d7')
  ap.add_argument('--out', default=None, help='output directory (default <scratch>/shadow-viz)')
  ap.add_argument('--no-video', action='store_true', help='charts only, do not fetch MP4s')
  ap.add_argument('--gap-profile', type=int, default=None,
                  help='gap profile the drive ran with (default: read GapProfile off the device)')
  args = ap.parse_args()

  out = args.out or os.path.join(SCRATCH, 'shadow-viz')
  paths = sorted(glob.glob(os.path.join(LOG_ROOT, f'{args.route}--*/rlog.zst')), key=seg_no)
  if not paths:
    print(f'no segments for {args.route} under {LOG_ROOT}', file=sys.stderr)
    return 1

  os.makedirs(os.path.join(out, 'data'), exist_ok=True)
  os.makedirs(os.path.join(out, 'video'), exist_ok=True)

  print(f'{args.route}: {len(paths)} segments -> {out}')
  segments = []
  for path in paths:
    n = seg_no(path)
    seg = extract_segment(path)
    stats = summarise(seg['rows'])
    with open(os.path.join(out, 'data', f'seg-{n}.json'), 'w') as f:
      json.dump({'seg': n, 'rows': seg['rows']}, f, separators=(',', ':'))

    has_video = False
    if not args.no_video:
      has_video = fetch_video(args.route, n, os.path.join(out, 'video', f'seg-{n}.mp4'))

    dur = seg['rows'][-1][0] if seg['rows'] else 0.0
    segments.append({'seg': n, 'dur': dur, 'video': has_video, 'hasPlan': seg['hasPlan'], **stats})
    print(f'  seg {n:2d}  {dur:5.1f}s  lead {stats["leadFrames"]:5d}f  '
          f'disagree {stats["disagreePct"]:5.1f}%  worst {stats["worst"]:+.2f} m/s^2'
          f'{"" if has_video else "   (no video)"}')

  profile = args.gap_profile
  if profile is None:
    profile = device_gap_profile()
    if profile is None:
      profile = 0
      print('  GapProfile을 읽지 못해 0(표준)으로 그립니다 -- 실제와 다르면 --gap-profile로 지정하세요')
    else:
      print(f'  GapProfile={profile} (기기에서 읽음)')
  t_follow, stop_distance = follow_targets(profile)
  meta = {
    'gapProfile': profile,
    'route': args.route,
    'accelMax': ACCEL_MAX, 'accelMin': ACCEL_MIN,
    'hz': HZ, 'disagreeMs2': DISAGREE_MS2,
    # What openpilot would have kept: desired gap = tFollow[knob] * v + stopDistance.
    'tFollow': {str(k): v for k, v in t_follow.items()},
    'stopDistance': stop_distance,
    'segments': segments,
  }
  with open(os.path.join(out, 'data', 'route.json'), 'w') as f:
    json.dump(meta, f, separators=(',', ':'))

  here = os.path.dirname(os.path.abspath(__file__))
  shutil.copyfile(os.path.join(here, 'shadow_viz.html'), os.path.join(out, 'index.html'))

  worst = min((s['worst'] for s in segments), default=0.0)
  print(f'\nwrote {out}/index.html   worst disagreement {worst:+.2f} m/s^2')
  print(f'serve it:  python3 -m http.server -d {out} 8730')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
