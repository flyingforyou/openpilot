#!/usr/bin/env python3
"""Why the lane-centring trim came out as small as it did, and what the knobs would change.

The headline report says the correction was real but tiny. This asks what it was actually
measuring, replays the same drive at the other authority settings, and checks the trim was not
buying that smallness with any oscillation.
"""
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, '/home/compiler/openpilot')
from openpilot.selfdrive.controls.lib import lane_centering as LC
from openpilot.tools.lib.logreader import LogReader


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  ctls = {a: LC.LaneCenteringController() for a in (0.0, 0.5, 1.0)}

  v_ego, blink, lat_active = 0.0, False, False
  rows, why = [], Counter()

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'carState':
        c = msg.carState
        v_ego, blink = float(c.vEgo), bool(c.leftBlinker or c.rightBlinker)
      elif w == 'carControl':
        lat_active = bool(msg.carControl.latActive)
      elif w == 'modelV2':
        md = msg.modelV2
        base = float(md.action.desiredCurvature)
        trims = {a: c.update(base, md, v_ego, True, 0.0, a, lat_active, True, True, blink) - base
                 for a, c in ctls.items()}
        if not lat_active or len(md.laneLines) < 3:
          continue

        probs = np.asarray(md.laneLineProbs, dtype=float)
        stds = np.asarray(md.laneLineStds, dtype=float)
        lx = np.asarray(md.laneLines[1].x, dtype=float)
        ly = np.asarray(md.laneLines[1].y, dtype=float)
        ry = np.asarray(md.laneLines[2].y, dtype=float)
        px = np.asarray(md.position.x, dtype=float)
        py = np.asarray(md.position.y, dtype=float)
        if lx.size < 2 or px.size < 2:
          continue
        look = float(np.clip(v_ego, 8.0, 35.0))
        if not (lx[0] <= look <= lx[-1]):
          continue

        left0, right0 = float(ly[0]), float(ry[0])
        leftL, rightL = float(np.interp(look, lx, ly)), float(np.interp(look, lx, ry))
        width = rightL - leftL
        car_off = -0.5 * (left0 + right0)                       # car vs centre, now
        path_err = 0.5 * (leftL + rightL) - float(np.interp(look, px, py))   # what the trim sees
        path_std = float(np.interp(look, px, np.asarray(md.position.yStd, dtype=float)))

        # why the lines were rejected, when they were
        if not (probs[[1, 2]] >= 0.6).all():
          why['prob < 0.6'] += 1
        elif not (stds[[1, 2]] <= 0.3).all():
          why['std > 0.3'] += 1
        elif not 2.6 <= width <= 4.8:
          why[f'width {width:.1f} out of range'.replace(f'{width:.1f}', 'out of 2.6-4.8')] += 1

        rows.append((v_ego, abs(base), car_off, path_err, path_std, width,
                     trims[0.0], trims[0.5], trims[1.0],
                     bool((probs[[1, 2]] >= 0.6).all() and (stds[[1, 2]] <= 0.3).all()
                          and 2.6 <= width <= 4.8)))

  v, k, car_off, path_err, path_std, width, t0, t50, t100, ok = (
    np.array(x, dtype=float) for x in zip(*rows, strict=True))
  ok = ok.astype(bool)

  print("-- what the trim measures vs what you feel --")
  s = ok
  print(f"  car's own offset from centre    |mean| {np.abs(car_off[s]).mean():.3f} m  p90 {np.percentile(np.abs(car_off[s]),90):.3f}")
  print(f"  model path error at lookahead   |mean| {np.abs(path_err[s]).mean():.3f} m  p90 {np.percentile(np.abs(path_err[s]),90):.3f}   <- this is the input")
  print(f"  after the {LC.CENTER_ERROR_DEADBAND} m deadband        |mean| {np.maximum(np.abs(path_err[s])-LC.CENTER_ERROR_DEADBAND,0).mean():.3f} m")
  print(f"  correlation car offset vs path error  r = {np.corrcoef(car_off[s], path_err[s])[0,1]:+.2f}")
  print(f"  frames where the path is already centred (<{LC.CENTER_ERROR_DEADBAND} m): {100*np.mean(np.abs(path_err[s]) < LC.CENTER_ERROR_DEADBAND):.0f}%")

  print("\n-- authority, replayed over the same drive --")
  for name, t in (("0   (correct everything)", t0), ("50  (as driven)", t50), ("100 (defer to model)", t100)):
    a = np.abs(t[ok])
    on = a > 1e-9
    med = np.median(a[on]) if on.any() else 0
    p90 = np.percentile(a[on], 90) if on.any() else 0
    print(f"  {name:26} applied {100*on.mean():5.1f}%   median {med:.6f}   p90 {p90:.6f}   max {a.max():.6f}")
  give = np.abs(t0[ok]) - np.abs(t50[ok])
  print(f"  authority 50 gives up  median {np.median(give):.6f}  p90 {np.percentile(give,90):.6f} 1/m vs correcting everything")
  big = ok & (np.abs(path_err) > LC.E2E_BREAK_IN_START)
  print(f"  frames past the 0.15 m break-in (where authority even applies): {100*big.mean():.1f}% of usable")

  print("\n-- curvature bins (|model curvature|) --")
  for lo, hi, label in ((0.0, 0.002, 'straight'), (0.002, 0.005, 'gentle curve'),
                        (0.005, 0.01, 'curve'), (0.01, 9.9, 'tight curve')):
    sel = ok & (k >= lo) & (k < hi)
    if sel.sum() < 30:
      print(f"  {label:14} n={sel.sum():6}  (too few)")
      continue
    off = f"|mean| {np.abs(car_off[sel]).mean():.3f}  p90 {np.percentile(np.abs(car_off[sel]),90):.3f}"
    trim = f"median {np.median(np.abs(t50[sel])):.6f}  at auth0 {np.median(np.abs(t0[sel])):.6f}"
    print(f"  {label:14} n={sel.sum():6}  car offset {off}   trim {trim}")

  print("\n-- steadiness (is the trim fighting itself?) --")
  d = np.diff(t50[ok])
  flips = np.mean(np.sign(t50[ok][1:]) * np.sign(t50[ok][:-1]) < 0)
  print(f"  sign flips between frames  {100*flips:.2f}%  (20 Hz frames)")
  cap = 0.0012 * (1 - np.exp(-0.05 / LC.SMOOTH_TAU))
  print(f"  frame-to-frame change      p99 {np.percentile(np.abs(d),99):.7f} 1/m (cap on a step from smoothing is ~{cap:.7f})")

  print("\n-- why lines were rejected --")
  tot = len(rows)
  for r, n in why.most_common():
    print(f"  {r:28} {n:6}  ({100*n/tot:.0f}% of engaged frames)")
  print(f"  usable                       {ok.sum():6}  ({100*ok.mean():.0f}%)")


if __name__ == '__main__':
  main(sys.argv[1:])
