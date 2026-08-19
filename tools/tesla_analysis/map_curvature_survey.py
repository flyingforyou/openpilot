#!/usr/bin/env python3
"""Does the map's own road curvature tell us anything the model does not?

The curve controller reads the model's curvature, which is whatever the camera can see -- and
around a bend that is not far. UI_roadCurvature (0x2C8) carries a cubic for the road ahead with
a declared range, so in principle it knows about a corner before the camera is pointed at it.
In principle. This measures whether it actually leads, and by how much, before anything is
wired to it.

  ./map_curvature_survey.py op-logs/00000073--d6cfdd05f3--*
"""
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

ROAD_CURVATURE = 712    # 0x2C8


def sig(payload: bytes, start: int, length: int, scale: float, signed: bool) -> float:
  raw = int.from_bytes(payload.ljust(8, b'\x00')[:8], 'little')
  val = (raw >> start) & ((1 << length) - 1)
  if signed and val >= (1 << (length - 1)):
    val -= (1 << length)
  return val * scale


def decode(payload: bytes) -> dict:
  return {
    'c0': sig(payload, 0, 11, 0.02, True),
    'c1': sig(payload, 11, 10, 0.00075, True),
    'c2': sig(payload, 21, 14, 7.5e-06, True),
    'c3': sig(payload, 35, 13, 3e-08, True),
    'range': sig(payload, 48, 6, 4.0, False),
    'health': sig(payload, 54, 2, 1.0, False),
  }


def model_curvature(md) -> float:
  try:
    rate = np.asarray(md.orientationRate.z, dtype=float)
    vel = np.asarray(md.velocity.x, dtype=float)
    if rate.size and vel.size:
      return float(rate[0] / max(vel[0], 1.0))
  except (AttributeError, TypeError, ValueError):
    pass
  return 0.0


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  rows = []
  cur_map, v_ego, cur_model = None, 0.0, 0.0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'modelV2':
        cur_model = model_curvature(msg.modelV2)
      elif w == 'can':
        for c in msg.can:
          if c.address == ROAD_CURVATURE and c.src <= 2:
            cur_map = decode(bytes(c.dat))
            if v_ego > 5:
              rows.append((cur_map['c2'] * 2, cur_model, cur_map['range'], cur_map['health'], v_ego))

  if not rows:
    print("no 0x2C8 frames")
    return
  k_map, k_model, rng, health, v = (np.array(x) for x in zip(*rows, strict=True))

  print(f"frames {len(k_map)} above 5 m/s")
  print(f"  health      {dict(zip(*np.unique(health, return_counts=True), strict=True))}")
  print(f"  range (m)   median {np.median(rng):.0f}  p10 {np.percentile(rng, 10):.0f}  max {rng.max():.0f}   zero on {100*np.mean(rng == 0):.0f}% of frames")

  ok = (health > 0) & (rng > 0)
  print(f"  usable      {100*ok.mean():.0f}% of frames")
  if not ok.any():
    print("  -> nothing usable; do not wire this up")
    return

  km, kd = np.abs(k_map[ok]), np.abs(k_model[ok])
  print("\n-- how big is the map's curvature, against the model's --")
  print(f"  map    |k| median {np.median(km):.5f}  p90 {np.percentile(km, 90):.5f}  max {km.max():.5f} 1/m")
  print(f"  model  |k| median {np.median(kd):.5f}  p90 {np.percentile(kd, 90):.5f}  max {kd.max():.5f} 1/m")
  if km.std() > 0 and kd.std() > 0:
    print(f"  correlation r = {np.corrcoef(km, kd)[0, 1]:+.2f}")

  print("\n-- does the map lead the model into a bend? --")
  # a bend by the model's reckoning; see what the map said N seconds earlier
  bend = kd > 0.005
  print(f"  model calls it a bend on {100*bend.mean():.1f}% of usable frames")
  if bend.sum() > 50:
    idx = np.where(bend)[0]
    for lead_s in (1, 2, 3, 5):
      back = idx - int(lead_s * 10)      # 0x2C8 arrives near 10 Hz
      back = back[back >= 0]
      if back.size:
        print(f"    {lead_s}s before those frames, map |k| median {np.median(km[back]):.5f}   (model then {np.median(kd[back]):.5f})")

  print("\n-- speed each would allow at 3.0 m/s^2 lateral --")
  for label, arr in (('map', km), ('model', kd)):
    with np.errstate(divide='ignore'):
      allowed = np.sqrt(3.0 / np.maximum(arr, 1e-9)) * 2.23694
    print(f"  {label:5} median {np.median(allowed):5.0f} mph   p10 {np.percentile(allowed, 10):5.0f} mph")


if __name__ == '__main__':
  main(sys.argv[1:])
