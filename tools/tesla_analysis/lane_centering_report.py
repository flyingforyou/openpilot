#!/usr/bin/env python3
"""What the lane-centring trim actually did on a recorded drive.

The trim is deterministic given modelV2 and speed, so rather than inferring it from the
difference between the model's curvature and the one that was sent, this replays the real
controller over the logged frames. That gives the exact correction alongside the thing worth
judging it by: where in the lane the car actually sat.

  ./lane_centering_report.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, '/home/compiler/openpilot')
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.tools.lib.logreader import LogReader

# Anything tighter than this is a corner rather than a straight, in 1/m.
CURVE_K = 0.002
LOOKAHEAD_LO, LOOKAHEAD_HI = 8.0, 35.0


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def lane_at(md, x_at: float):
  """Left/right lane line y and the car's own path y at a distance, or None."""
  if len(md.laneLines) < 3:
    return None
  lx = np.asarray(md.laneLines[1].x, dtype=float)
  ly = np.asarray(md.laneLines[1].y, dtype=float)
  ry = np.asarray(md.laneLines[2].y, dtype=float)
  px = np.asarray(md.position.x, dtype=float)
  py = np.asarray(md.position.y, dtype=float)
  if lx.size < 2 or px.size < 2 or not (lx[0] <= x_at <= lx[-1]):
    return None
  return (float(np.interp(x_at, lx, ly)), float(np.interp(x_at, lx, ry)),
          float(np.interp(x_at, px, py)))


def main(paths):
  paths = sorted(paths, key=seg_no)
  ctl = LaneCenteringController()

  cs = {'v': 0.0, 'blink': False}
  lat_active = False
  rows = []           # one per modelV2 frame while engaged
  gate_reasons = Counter()
  model_id = None
  frames = engaged_frames = 0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'initData' and model_id is None:
        # don't shadow the segment path being read
        boot = {e.key: bytes(e.value).decode(errors='replace') for e in msg.initData.params.entries}
        model_id = boot.get('DrivingModel') or None
        print('settings at boot   ' + '  '.join(
          f"{k}={boot.get('Lane' + k, '-')}" for k in
          ('Centering', 'CenteringE2EAuthority', 'CenterOffset', 'CenteringPauseOnSignal')))
      elif w == 'carState':
        c = msg.carState
        cs = {'v': float(c.vEgo), 'blink': bool(c.leftBlinker or c.rightBlinker)}
      elif w == 'carControl':
        lat_active = bool(msg.carControl.latActive)
      elif w == 'modelV2':
        md = msg.modelV2
        frames += 1
        base = float(md.action.desiredCurvature)
        trim = ctl.update(base, md, cs['v'], True, 0.0, 0.5, lat_active, True, True, cs['blink']) - base
        if not lat_active:
          continue
        engaged_frames += 1

        near = lane_at(md, 0.0)
        look = lane_at(md, float(np.clip(cs['v'], LOOKAHEAD_LO, LOOKAHEAD_HI)))
        if near is None or look is None:
          gate_reasons['no lane geometry'] += 1
          continue
        left0, right0, _ = near
        width = right0 - left0
        # The car sits at y=0, so the lane centre's y is how far the centre is to its right;
        # negate to get "how far right of centre the car is".
        offset = -0.5 * (left0 + right0)
        probs = np.asarray(md.laneLineProbs, dtype=float)
        stds = np.asarray(md.laneLineStds, dtype=float)
        usable = (probs[[1, 2]] >= 0.6).all() and (stds[[1, 2]] <= 0.3).all() and 2.6 <= width <= 4.8
        if not usable:
          gate_reasons['lines not trusted'] += 1
        elif cs['v'] < 5.0:
          gate_reasons['below 5 m/s'] += 1
        elif cs['blink']:
          gate_reasons['blinker'] += 1
        elif abs(trim) < 1e-9:
          gate_reasons['inside deadband / deferred to model'] += 1

        rows.append((cs['v'], base, trim, offset, width, usable, abs(float(md.action.desiredCurvature))))

  if not rows:
    print("no engaged frames with model output")
    return

  v, base, trim, offset, width, usable, k = (np.array(x, dtype=float) for x in zip(*rows, strict=True))
  usable = usable.astype(bool)
  act = np.abs(trim) > 1e-9

  print(f"route          {paths[0].rstrip('/').rsplit('--',1)[0].split('/')[-1]}  ({len(paths)} segments)")
  print(f"driving model  {model_id or 'not in log'}")
  print(f"model frames   {frames}  |  engaged {engaged_frames} ({100*engaged_frames/max(frames,1):.0f}%)"
          f"  |  {engaged_frames/20/60:.1f} min engaged")
  print(f"speed          median {np.median(v)*2.23694:.0f} mph, max {v.max()*2.23694:.0f} mph")

  print("\n-- trim --")
  print(f"  applied on          {100*act.mean():.1f}% of engaged frames")
  if act.any():
    a = np.abs(trim[act])
    print(f"  magnitude           median {np.median(a):.6f}  p90 {np.percentile(a,90):.6f}  max {a.max():.6f} 1/m")
    print(f"  as % of the cap     median {100*np.median(a)/0.0012:.0f}%  p90 {100*np.percentile(a,90)/0.0012:.0f}%")
    lat = v[act]**2 * a
    print(f"  lateral accel       median {np.median(lat):.3f}  p90 {np.percentile(lat,90):.3f} m/s^2")
    print(f"  saturated at cap    {100*np.mean(a >= 0.0012-1e-9):.1f}% of applied frames")
  print("  idle because:")
  for r, n in gate_reasons.most_common():
    print(f"    {r:36} {n:6}  ({100*n/max(engaged_frames,1):.0f}% of engaged)")

  print("\n-- where the car sat in the lane (positive = right of centre) --")
  for label, sel in (("all engaged", usable),
                     ("straight  |k| < 0.002", usable & (k < CURVE_K)),
                     ("curve     |k| >= 0.002", usable & (k >= CURVE_K)),
                     ("curve, trim applied", usable & (k >= CURVE_K) & act),
                     ("curve, trim idle", usable & (k >= CURVE_K) & ~act)):
    if sel.sum() < 20:
      print(f"  {label:24} (only {sel.sum()} frames)")
      continue
    o = offset[sel]
    print(f"  {label:24} n={sel.sum():6}  mean {o.mean():+.3f}  |mean| {np.abs(o).mean():.3f}  "
          f"p90 {np.percentile(np.abs(o),90):.3f}  max {np.abs(o).max():.3f} m")

  print("\n-- does the trim push the way the offset needs? --")
  sel = usable & act
  if sel.sum() > 20:
    agree = np.mean(np.sign(trim[sel]) == -np.sign(offset[sel]))
    print(f"  trim opposes the car's offset on {100*agree:.0f}% of applied frames")
    print(f"  (offset and trim correlation r = {np.corrcoef(offset[sel], trim[sel])[0,1]:+.2f}, "
          f"negative is correct)")

  print("\n-- lane width seen --")
  print(f"  median {np.median(width[usable]):.2f} m  p10 {np.percentile(width[usable],10):.2f}  "
        f"p90 {np.percentile(width[usable],90):.2f}")


if __name__ == '__main__':
  main(sys.argv[1:])
