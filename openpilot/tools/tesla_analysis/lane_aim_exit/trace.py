"""20 Hz from rlogs: lateral accel, aim shift, car position in the lane (road and as the model saw it)."""
import sys, glob, zstandard, numpy as np
from openpilot.cereal import log as capnp_log
segs = sorted(sys.argv[2:], key=lambda p: int(p.rstrip('/').split('--')[-1]))
md, cc, cs = [], [], []
for d in segs:
  for m in capnp_log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(d + '/rlog.zst', 'rb')).read()):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'modelV2':
      mv = m.modelV2
      if len(mv.laneLines) >= 3 and len(mv.laneLines[1].y):
        md.append((t, mv.meta.laneAimShift, mv.laneLines[1].y[0], mv.laneLines[2].y[0], mv.laneLineProbs[1], mv.laneLineProbs[2]))
    elif w == 'carControl':
      cc.append((t, m.carControl.latActive, m.carControl.actuators.curvature))
    elif w == 'carState':
      cs.append((t, m.carState.vEgo, m.carState.leftBlinker or m.carState.rightBlinker, m.carState.steeringPressed))
md, cc, cs = (np.array(x, float) for x in (md, cc, cs))
t0 = float(sys.argv[1])
v = np.interp(md[:, 0], cs[:, 0], cs[:, 1])
lat = np.interp(md[:, 0], cc[:, 0], cc[:, 2]) * v ** 2           # + left
act = np.interp(md[:, 0], cc[:, 0], cc[:, 1])
blink = np.interp(md[:, 0], cs[:, 0], cs[:, 2])
shift = md[:, 1]
pos_road = -(md[:, 2] + md[:, 3]) / 2                              # + car right of the lane centre
pos_view = pos_road + shift                                        # where the model, shown the shift, thinks it is
T0 = md[0, 0]
print(' t(s)   lat   shift  pos_road pos_view  act blink')
for k in range(0, len(md), 20):
  if t0 <= md[k, 0] - T0:
    print(f'{md[k, 0] - T0:6.0f} {lat[k]:+5.2f} {shift[k]:+6.3f}  {pos_road[k]:+6.2f}  {pos_view[k]:+6.2f}   {int(act[k])}  {int(blink[k])}')
