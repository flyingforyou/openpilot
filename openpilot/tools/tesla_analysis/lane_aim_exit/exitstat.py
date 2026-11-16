"""Every bend exit in the rlogs: aim shift at the exit and the largest lane offset in the 5 s after."""
import sys, glob, zstandard, numpy as np
from openpilot.cereal import log as capnp_log
def load(segs):
  md, cc, cs = [], [], []
  for d in segs:
    for m in capnp_log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(d + '/rlog.zst', 'rb')).read()):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == 'modelV2' and len(m.modelV2.laneLines) >= 3 and len(m.modelV2.laneLines[1].y):
        mv = m.modelV2
        md.append((t, mv.meta.laneAimShift, mv.laneLines[1].y[0], mv.laneLines[2].y[0], min(mv.laneLineProbs[1], mv.laneLineProbs[2])))
      elif w == 'carControl':
        cc.append((t, m.carControl.latActive, m.carControl.actuators.curvature))
      elif w == 'carState':
        cs.append((t, m.carState.vEgo, m.carState.leftBlinker or m.carState.rightBlinker))
  return (np.array(x, float) for x in (md, cc, cs))
routes = {}
for d in sorted(glob.glob(sys.argv[1] + '/*--*')):
  routes.setdefault(d.rsplit('--', 1)[0], []).append(d)
for r, segs in routes.items():
  segs.sort(key=lambda p: int(p.split('--')[-1]))
  md, cc, cs = load(segs)
  t = md[:, 0]; v = np.interp(t, cs[:, 0], cs[:, 1]); lat = np.interp(t, cc[:, 0], cc[:, 2]) * v ** 2
  act = np.interp(t, cc[:, 0], cc[:, 1]) > 0.5; blink = np.interp(t, cs[:, 0], cs[:, 2]) > 0.5
  shift = md[:, 1]; pos = -(md[:, 2] + md[:, 3]) / 2
  inb = (np.abs(lat) > 1.0) & (v > 12)
  i = 0
  while i < len(t):
    if not inb[i]: i += 1; continue
    j = i
    while j + 1 < len(t) and np.abs(lat[j + 1]) > 0.5: j += 1
    if t[j] - t[i] >= 4:
      side = np.sign(np.mean(lat[i:j]))                 # + left bend; inside is left (pos -)
      w = (t > t[j]) & (t < t[j] + 5)
      ww = (t > t[i] - 2) & (t < t[j] + 6)
      clean = act[ww].all() and not blink[ww].any() and (md[ww, 4] > 0.5).all()
      k = np.argmax(np.abs(pos[w])) if w.any() else None
      peak = pos[w][k] if k is not None else np.nan
      toward = 'inside-dir' if np.sign(peak) == -side else 'outside-dir'
      push = 'pushes inside' if np.sign(shift[j]) == side else 'pushes outside'   # shift<0 -> model steers right
      print(f"{r.split('/')[-1][:8]} {t[i] - md[0, 0]:5.0f}s {'L' if side > 0 else 'R'} {t[j] - t[i]:4.1f}s | in-bend pos {side * -np.mean(pos[i:j]):+.2f} inside"
            f" | shift at exit {shift[j]:+.3f} ({push}) | 0-5 s after: peak {abs(peak):.2f} m ({toward}), shift 5 s later {shift[w][-1] if w.any() else np.nan:+.3f}"
            f"{'' if clean else '  [blinker/lat off/weak lines]'}")
    i = j + 1
