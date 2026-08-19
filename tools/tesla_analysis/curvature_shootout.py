#!/usr/bin/env python3
"""Map curvature or model curvature -- which one is actually right about the road ahead?

Both claim to describe the bend before the car is in it, so the fair test is against what the
car ends up doing. For a lookahead distance d, each source's prediction made now is compared
with the curvature the car is actually driving once it has travelled those d metres.

Ground truth is controlsState.curvature, which comes from the steering angle and the vehicle
model -- what the car really did, not what anything predicted.

The model publishes a whole predicted path, so it is read at the distance being asked about
rather than at the bumper; an earlier version of this comparison used only its first sample and
made the map look better than it is.

  ./curvature_shootout.py op-logs/00000073--d6cfdd05f3--*
"""
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

ROAD_CURVATURE = 712        # 0x2C8
LOOKAHEADS = (30.0, 60.0, 100.0, 150.0)
MIN_SPEED = 8.0
BEND = 0.003                # 1/m; below this the road is straight and everyone is trivially right


def sig(payload: bytes, start: int, length: int, scale: float, signed: bool) -> float:
  raw = int.from_bytes(payload.ljust(8, b'\x00')[:8], 'little')
  val = (raw >> start) & ((1 << length) - 1)
  if signed and val >= (1 << (length - 1)):
    val -= (1 << length)
  return val * scale


def map_curve(payload: bytes):
  """Cubic y(x) = c0 + c1 x + c2 x^2 + c3 x^3. Curvature is y'' = 2 c2 + 6 c3 x."""
  return (sig(payload, 21, 14, 7.5e-06, True), sig(payload, 35, 13, 3e-08, True),
          sig(payload, 48, 6, 4.0, False), sig(payload, 54, 2, 1.0, False))


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)

  samples = []          # (t, v_ego, truth_k, model_x[], model_k[], c2, c3, rng, health)
  truth, t_truth = [], []
  cur_map = None
  v_ego = 0.0
  model = None
  t0 = None

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'controlsState':
        truth.append(float(msg.controlsState.curvature))
        t_truth.append(t - t0)
      elif w == 'can':
        for c in msg.can:
          if c.address == ROAD_CURVATURE and c.src <= 2:
            cur_map = map_curve(bytes(c.dat))
      elif w == 'modelV2':
        md = msg.modelV2
        x = np.asarray(md.position.x, dtype=float)
        rate = np.asarray(md.orientationRate.z, dtype=float)
        vel = np.asarray(md.velocity.x, dtype=float)
        if x.size < 5 or rate.size != x.size or vel.size != x.size:
          continue
        model = (x, rate / np.maximum(vel, 1.0))
      elif w == 'longitudinalPlan' and model is not None and cur_map is not None and v_ego > MIN_SPEED:
        samples.append((t - t0, v_ego, model[0], model[1], *cur_map))

  if not samples or not truth:
    print("not enough data")
    return
  truth = np.array(truth)
  t_truth = np.array(t_truth)

  print(f"samples {len(samples)}, ground-truth frames {len(truth)}")
  print("ground truth = controlsState.curvature (steering angle through the vehicle model)\n")

  head = f"  {'lookahead':>10} {'n':>6}"
  print(f"{head} {'map err':>10} {'model err':>11} {'winner':>9}   {'map err':>9} {'model err':>11} {'winner':>9}")
  print(f"  {'':>10} {'':>6} {'--- all road ---':^32}   {'--- bends only ---':^32}")

  for d in LOOKAHEADS:
    rows = []
    for (ts, v, mx, mk, c2, c3, rng, health) in samples:
      if health <= 0 or rng < d:
        continue                                    # the map declines to describe this far
      if d > mx[-1]:
        continue
      k_map = 2 * c2 + 6 * c3 * d
      k_model = float(np.interp(d, mx, mk))
      # when the car has travelled d metres -- at this speed, that is d/v seconds from now
      t_then = ts + d / max(v, 1.0)
      j = np.searchsorted(t_truth, t_then)
      if j <= 0 or j >= truth.size:
        continue
      rows.append((abs(k_map), abs(k_model), abs(float(truth[j]))))
    if len(rows) < 200:
      print(f"  {d:9.0f}m {len(rows):>6}   (too few)")
      continue
    a = np.array(rows)
    e_map = np.abs(a[:, 0] - a[:, 2])
    e_mod = np.abs(a[:, 1] - a[:, 2])
    bend = a[:, 2] > BEND
    m_all, d_all = np.median(e_map), np.median(e_mod)
    m_bend = np.median(e_map[bend]) if bend.any() else float('nan')
    d_bend = np.median(e_mod[bend]) if bend.any() else float('nan')
    w_all = 'map' if m_all < d_all else 'model'
    w_bend = 'map' if m_bend < d_bend else 'model'
    left = f"  {d:9.0f}m {len(a):>6} {m_all:10.5f} {d_all:11.5f} {w_all:>9}"
    print(f"{left}   {m_bend:9.5f} {d_bend:11.5f} {w_bend:>9}   (bends {bend.sum()})")

  print("\n-- do they at least agree on *when* a bend is coming? --")
  for d in (60.0, 100.0):
    hit_map = hit_mod = tot = 0
    for (ts, v, mx, mk, c2, c3, rng, health) in samples:
      if health <= 0 or rng < d or d > mx[-1]:
        continue
      t_then = ts + d / max(v, 1.0)
      j = np.searchsorted(t_truth, t_then)
      if j <= 0 or j >= truth.size:
        continue
      if abs(float(truth[j])) <= BEND:
        continue
      tot += 1
      hit_map += abs(2 * c2 + 6 * c3 * d) > BEND
      hit_mod += abs(float(np.interp(d, mx, mk))) > BEND
    if tot:
      print(f"  {d:.0f}m ahead, {tot} real bends: map called {100*hit_map/tot:.0f}%, model called {100*hit_mod/tot:.0f}%")


if __name__ == '__main__':
  main(sys.argv[1:])
