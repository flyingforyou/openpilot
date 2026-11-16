"""lane_aim.LaneAim as it is now, run over the frames of recorded drives: every bend exit, logged -> replayed.

  replay_aim.py <rlog dir> [AIM_xxx=value ...] [TAU=s] [GAIN=m per m]

No model runs here. The car is taken to move GAIN m (1.0) per m of change of shift, 0.3 s late with a lag of TAU (1.2 s),
on the assumption that the model keeps its own line in whatever it is shown -- so the shift column is
what the code does, and the position columns are an estimate (latsim.py is the real thing).
Signs: lat + is a bend to the right, positions + right; "out" is toward the outside line.
"""
import sys, glob, os, zstandard, numpy as np
from openpilot.cereal import log as capnp_log
from openpilot.selfdrive.controls.lib import lane_aim as la

N = 33        # points in the model's plan
COLS = dict(t=0, shift=1, left=2, right=3, lprob=4, rprob=5, v=6, blinker=7, active=8, k=9, lane_change=10, rate=11, vel=11 + N, plan_t=11 + 2 * N)


def load(segs):
  """A row per modelV2 frame (COLS), cached beside the logs."""
  cache = segs[0].rsplit('--', 1)[0] + '.replay_aim.npy'
  if os.path.isfile(cache):
    return np.load(cache)
  rows = []; st = dict(v=0.0, blk=0.0, act=0.0, k=0.0)
  for d in segs:
    for m in capnp_log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(d + '/rlog.zst', 'rb')).read()):
      w = m.which()
      if w == 'carState': st['v'] = m.carState.vEgo; st['blk'] = float(m.carState.leftBlinker or m.carState.rightBlinker)
      elif w == 'carControl': st['act'] = float(m.carControl.latActive)
      elif w == 'controlsState': st['k'] = m.controlsState.curvature
      elif w == 'modelV2':
        mv = m.modelV2
        if len(mv.laneLines) < 3 or not len(mv.laneLines[1].y) or len(mv.orientationRate.z) != N: continue
        rows.append([m.logMonoTime / 1e9, mv.meta.laneAimShift, mv.laneLines[1].y[0], mv.laneLines[2].y[0], mv.laneLineProbs[1], mv.laneLineProbs[2],
                     st['v'], st['blk'], st['act'], st['k'], float(mv.meta.laneChangeState.raw != 0)]
                    + list(mv.orientationRate.z) + list(mv.velocity.x) + list(mv.orientationRate.t))
  a = np.array(rows, float)
  np.save(cache, a)
  return a


def replay(a, tau=1.2, delay=0.3, gain=1.0):
  """(shift, car position in the lane) per frame with the code as it is."""
  c = COLS; n = len(a); aim = la.LaneAim(); nd = int(delay / aim.dt)
  pos = -(a[:, c['left']] + a[:, c['right']]) / 2; width = a[:, c['right']] - a[:, c['left']]
  lat = a[:, c['rate']:c['rate'] + N] * a[:, c['vel']:c['vel'] + N]
  shift, pos2 = np.zeros(n), np.zeros(n); d = 0.0; hist = [0.0] * (nd + 1)
  for i in range(n):
    if i and a[i, 0] - a[i - 1, 0] > 1.0:                    # segments missing in between
      aim.reset(); d = 0.0; hist = [0.0] * (nd + 1)
    shift[i] = aim.shift
    hist.append(aim.shift - a[i, c['shift']]); u = hist.pop(0)
    d += (gain * u - d) * aim.dt / tau
    pos2[i] = pos[i] - d                                     # shown further right, the model takes the car further left
    seen = -pos2[i] - aim.shift
    ahead = lat[i, a[i, c['plan_t']:c['plan_t'] + N] <= la.AIM_LOOKAHEAD]
    aim.update(seen - width[i] / 2, seen + width[i] / 2, a[i, c['lprob']], a[i, c['rprob']], a[i, c['v']], True, a[i, c['active']] > 0.5,
               a[i, c['blinker']] > 0.5, a[i, c['lane_change']] > 0.5, (float(ahead.min()), float(ahead.max())))
  return shift, pos2


def bends(a):
  """(first frame, last frame, +1 right / -1 left): 4 s or more above 1 m/s2, until it falls below 0.5."""
  t, v = a[:, 0], a[:, COLS['v']]; lat = a[:, COLS['k']] * v ** 2; i = 0; out = []
  while i < len(t):
    if not (abs(lat[i]) > 1.0 and v[i] > 12): i += 1; continue
    j = i
    while j + 1 < len(t) and abs(lat[j + 1]) > 0.5 and t[j + 1] - t[j] < 1: j += 1
    if t[j] - t[i] >= 4: out.append((i, j, float(np.sign(np.mean(lat[i:j])))))
    i = j + 1
  return out


def main():
  kw = {k: float(v) for k, v in (x.split('=') for x in sys.argv[2:])}
  tau = kw.pop('TAU', 1.2); gain = kw.pop('GAIN', 1.0)
  for name, value in kw.items():
    assert hasattr(la, name), name
    setattr(la, name, value)
  routes = {}
  for d in sorted(glob.glob(sys.argv[1] + '/*--*/')):
    routes.setdefault(d.rstrip('/').rsplit('--', 1)[0], []).append(d.rstrip('/'))
  tot = []
  for r, segs in routes.items():
    segs.sort(key=lambda p: int(p.split('--')[-1]))
    a = load(segs); c = COLS; t = a[:, 0]; pos = -(a[:, c['left']] + a[:, c['right']]) / 2
    had_aim = bool(np.abs(a[:, c['shift']]).max() > 0.0)
    shift, pos2 = replay(a, tau, gain=gain)
    for i, j, side in bends(a):
      w = (t > t[j]) & (t < t[j] + 5); ww = (t > t[i] - 2) & (t < t[j] + 6)
      clean = bool((a[ww, c['active']] > 0.5).all() and not (a[ww, c['blinker']] > 0.5).any()
                   and (np.minimum(a[ww, c['lprob']], a[ww, c['rprob']]) > 0.5).all() and w.sum() > 90)
      row = (side * pos[i:j].mean(), side * pos2[i:j].mean(), side * a[j, c['shift']], side * shift[j], (-side * pos[w]).max(), (-side * pos2[w]).max())
      print(f"{os.path.basename(r)[:8]} {t[i] - t[0]:5.0f}s {'R' if side > 0 else 'L'} {t[j] - t[i]:4.1f}s | inside in the bend {row[0]:+.2f} -> {row[1]:+.2f}"
            f" | pushing out at the exit {row[2]:+.3f} -> {row[3]:+.3f} | furthest out 0-5 s after {row[4]:+.2f} -> {row[5]:+.2f}"
            f"{'' if had_aim else '  [driven without the aim]'}{'' if clean else '  [blinker / not steering / weak lines / log ends]'}")
      if clean and had_aim: tot.append(row)
  if tot:
    x = np.array(tot)
    print(f'{len(x)} clean exits driven with the aim, logged -> replayed: inside in the bend {x[:, 0].mean():.2f} -> {x[:, 1].mean():.2f} m'
          f' | pushing out at the exit {x[:, 2].mean():.3f} -> {x[:, 3].mean():.3f} m'
          f' | furthest out after, median {np.median(x[:, 4]):.2f} -> {np.median(x[:, 5]):.2f}, worst {x[:, 4].max():.2f} -> {x[:, 5].max():.2f},'
          f' over 0.3 m {(x[:, 4] > 0.3).sum()} -> {(x[:, 5] > 0.3).sum()}')


if __name__ == '__main__':
  main()
