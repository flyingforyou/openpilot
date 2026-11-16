"""Bends and what follows them, from qlogs: lane position (road frame, drivingModelData 2 Hz)."""
import sys, glob, zstandard, numpy as np
from openpilot.cereal import log as capnp_log
def rows(route):
  segs = sorted(glob.glob(route + '--*/qlog.zst'), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  cs, cc, ln = [], [], []
  for p in segs:
    try:
      evs = capnp_log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(p, 'rb')).read())
      for m in evs:
        t = m.logMonoTime / 1e9; w = m.which()
        if w == 'carState':
          c = m.carState; cs.append((t, c.vEgo, c.leftBlinker or c.rightBlinker, c.steeringPressed))
        elif w == 'carControl':
          cc.append((t, m.carControl.latActive, m.carControl.actuators.curvature))
        elif w == 'drivingModelData':
          l = m.drivingModelData.laneLineMeta
          ln.append((t, l.leftY, l.rightY, l.leftProb, l.rightProb, m.drivingModelData.meta.laneChangeState.raw))
    except Exception:
      pass
  return np.array(cs, float), np.array(cc, float), np.array(ln, float), segs[0]
for route in sys.argv[1:]:
  cs, cc, ln, s0 = rows(route)
  if not len(cc) or not len(ln):
    continue
  t0 = cc[0, 0]
  v = np.interp(cc[:, 0], cs[:, 0], cs[:, 1])
  lat = cc[:, 2] * v ** 2                      # + = left
  bend = (np.abs(lat) > 1.0) & (v > 12) & (cc[:, 1] > 0)
  # contiguous runs >= 4 s
  runs, i = [], 0
  while i < len(bend):
    if bend[i]:
      j = i
      while j + 1 < len(bend) and (bend[j + 1] or (j + 3 < len(bend) and bend[j + 3])):
        j += 1
      if cc[j, 0] - cc[i, 0] >= 4:
        runs.append((i, j))
      i = j + 1
    else:
      i += 1
  name = route.split('/')[-1]
  for i, j in runs:
    ta, tb = cc[i, 0], cc[j, 0]
    side = np.sign(np.mean(lat[i:j]))         # + left bend
    # car position in lane: + = car right of centre (y right positive)
    pos = -(ln[:, 1] + ln[:, 2]) / 2
    ok = (np.minimum(ln[:, 3], ln[:, 4]) > 0.5) & ((ln[:, 2] - ln[:, 1]) > 2.6) & ((ln[:, 2] - ln[:, 1]) < 4.6) & (ln[:, 5] == 0)
    blink = cs[(cs[:, 0] > ta - 2) & (cs[:, 0] < tb + 30), 2].any()
    lat_on = cc[(cc[:, 0] > ta) & (cc[:, 0] < tb + 30), 1].min() > 0
    def outward(a, b):                         # m toward the outside of the bend (left bend -> outside is right)
      m = ok & (ln[:, 0] >= a) & (ln[:, 0] < b)
      return side * np.mean(pos[m]) if m.sum() >= 3 else np.nan
    # straight after? next 30 s mean |lat|
    nxt = (cc[:, 0] > tb) & (cc[:, 0] < tb + 30)
    print(f"{name[:8]} t={ta - t0:6.0f}s {'L' if side > 0 else 'R'} {tb - ta:4.0f}s lat{np.mean(np.abs(lat[i:j])):4.1f} v{np.mean(v[i:j]):4.1f} | outward in-bend {outward(ta, tb):+.2f}  exit 0-5s {outward(tb, tb + 5):+.2f}  5-15s {outward(tb + 5, tb + 15):+.2f}  15-30s {outward(tb + 15, tb + 30):+.2f}"
          f" | after lat {np.mean(np.abs(lat[nxt])):.2f}{' BLINK' if blink else ''}{'' if lat_on else ' LAT-OFF'}")
