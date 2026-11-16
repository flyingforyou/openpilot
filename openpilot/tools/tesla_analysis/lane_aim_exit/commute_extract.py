"""qlogs of chosen routes -> q_<route>.npy (a row per drivingModelData frame, 2 Hz) in the current directory.

  commute_extract.py <route prefix> ...      qlogs under ~/op-logs/commute-20261116/ (copied from the device, qlog only)

Columns: t shift(0, not in qlog) leftY rightY leftProb rightProb 0 desiredCurvature v blinker steeringPressed latActive curvature lat lon.
Also prints the LaneCentering* params each drive started with.
"""
"""qlogs of the chosen routes -> one npy per route (a row per modelV2 frame) and the params the drive started with."""
import sys, glob, os, zstandard, numpy as np, pickle
from openpilot.cereal import log as capnp_log
root = os.path.expanduser('~/op-logs/commute-20261116')
KEYS = ('LaneCentering', 'LaneCenteringAim', 'LaneCenteringCurveHold')
res = {}
for pre in sys.argv[1:]:
  segs = sorted(glob.glob(f'{root}/{pre}--*--*'), key=lambda p: int(p.split('--')[-1]))
  md, cc, cs, ct, gp = [], [], [], [], []; params = {}
  for d in segs:
    raw = zstandard.ZstdDecompressor().stream_reader(open(d + '/qlog.zst', 'rb')).read()
    it = iter(capnp_log.Event.read_multiple_bytes(raw))
    while True:
      try: m = next(it)
      except Exception: break
      w = m.which(); t = m.logMonoTime / 1e9
      if w == 'initData' and not params:
        for kv in m.initData.params.entries:
          if kv.key in KEYS: params[kv.key] = bytes(kv.value).decode()
      elif w == 'drivingModelData':
        lm = m.drivingModelData.laneLineMeta
        md.append((t, 0.0, lm.leftY, lm.rightY, lm.leftProb, lm.rightProb, 0.0, m.drivingModelData.action.desiredCurvature))
      elif w == 'carControl': cc.append((t, m.carControl.latActive))
      elif w == 'carState': cs.append((t, m.carState.vEgo, float(m.carState.leftBlinker or m.carState.rightBlinker), float(m.carState.steeringPressed)))
      elif w == 'controlsState': ct.append((t, m.controlsState.curvature))
      elif w == 'gpsLocationExternal' and m.gpsLocationExternal.hasFix: gp.append((t, m.gpsLocationExternal.latitude, m.gpsLocationExternal.longitude))
  md, cc, cs, ct, gp = (np.array(x, float) for x in (md, cc, cs, ct, gp))
  tt = md[:, 0]; I = lambda a, j: np.interp(tt, a[:, 0], a[:, j])
  rows = np.c_[tt, md[:, 1:], I(cs, 1), I(cs, 2), I(cs, 3), I(cc, 1), I(ct, 1), I(gp, 1), I(gp, 2)]
  np.save(f'q_{pre}.npy', rows); res[pre] = params
  print(pre, rows.shape, 'median dt %.2f s' % np.median(np.diff(tt)), params)
pickle.dump(res, open('qparams.pkl', 'wb'))
