"""Series for the page: rlog segments -> 10 Hz arrays, cut around bends."""
import json, os, zstandard, numpy as np
from openpilot.cereal import log as capnp_log
R = os.path.expanduser('~/op-logs/laneaim-20261115/rlog')
def load(route, segs):
  md, cc, cs, seg_t = [], [], [], {}
  for s in segs:
    first = None
    for m in capnp_log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(f'{R}/{route}--{s}/rlog.zst', 'rb')).read()):
      w = m.which(); t = m.logMonoTime / 1e9
      if first is None and w in ('carState', 'modelV2'):
        first = t; seg_t[s] = t
      if w == 'modelV2' and len(m.modelV2.laneLines) >= 3 and len(m.modelV2.laneLines[1].y):
        mv = m.modelV2
        md.append((t, mv.meta.laneAimShift, mv.laneLines[1].y[0], mv.laneLines[2].y[0], min(mv.laneLineProbs[1], mv.laneLineProbs[2])))
      elif w == 'controlsState':
        cc.append((t, m.controlsState.curvature))
      elif w == 'carControl':
        cs.append((t, 'act', m.carControl.latActive))
      elif w == 'carState':
        cs.append((t, 'v', m.carState.vEgo)); cs.append((t, 'blk', m.carState.leftBlinker or m.carState.rightBlinker))
  md, cc = np.array(md), np.array(cc)
  ser = lambda k: np.array([(t, float(x)) for t, kk, x in cs if kk == k])
  v, act, blk = ser('v'), ser('act'), ser('blk')
  t = np.arange(md[0, 0], md[-1, 0], 0.1)
  vv = np.interp(t, v[:, 0], v[:, 1])
  out = dict(t=t, v=vv, lat=np.interp(t, cc[:, 0], cc[:, 1]) * vv ** 2,
             shift=np.interp(t, md[:, 0], md[:, 1]), pos=np.interp(t, md[:, 0], -(md[:, 2] + md[:, 3]) / 2),
             prob=np.interp(t, md[:, 0], md[:, 4]), act=np.interp(t, act[:, 0], act[:, 1]) > 0.5,
             blk=np.interp(t, blk[:, 0], blk[:, 1]) > 0.5)
  out['view'] = out['pos'] + out['shift']
  return out, seg_t
def cut(d, a, b, t0):
  m = (d['t'] >= a) & (d['t'] <= b)
  r = lambda x, n=3: [round(float(y), n) for y in x[m]]
  return dict(t=r(d['t'] - t0, 1), lat=r(d['lat'], 2), shift=r(d['shift']), pos=r(d['pos']), view=r(d['view']),
              act=[int(x) for x in d['act'][m]], blk=[int(x) for x in d['blk'][m]], v=r(d['v'], 1))
def bends(d, thr=1.0, rel=0.4, mind=2.5):
  out, i, t, lat = [], 0, d['t'], d['lat']
  while i < len(t):
    if abs(lat[i]) > thr and d['v'][i] > 8:
      j = i
      while j + 1 < len(t) and abs(lat[j + 1]) > rel and np.sign(lat[j + 1]) == np.sign(lat[i]): j += 1
      if t[j] - t[i] >= mind: out.append((t[i], t[j], int(np.sign(np.mean(lat[i:j])))))
      i = j + 1
    else: i += 1
  return out
page = {'windows': [], 'exits': [], 'sbends': []}
# (A) replay windows from lateral_windows.txt that are still on the device
for route, seg, a, b, nxt in (('0000026f--8b2ed4124f', 21, 10, 35, 22), ('00000270--cf522eca99', 7, 18, 40, 8), ('00000270--cf522eca99', 10, 1, 23, 11)):
  d, st = load(route, [seg, nxt])
  s0 = st[seg]
  bs = [x for x in bends(d) if x[1] > s0 + a - 5 and x[0] < s0 + b + 15]
  page['windows'].append(dict(name=f'{route[:8]}--{seg}', close=a, bend_from=a + 5, end=b, bends=[[round(x[0] - s0, 1), round(x[1] - s0, 1), x[2]] for x in bs],
                              **cut(d, s0 + a - 8, s0 + min(b + 25, 115), s0)))
# (B) single exits and (C) S-bends from the aim drives
for route, segs in (('00000272--8786cbdd07', [4, 5, 6, 7, 8]), ('00000273--f77b82b2cf', [11, 12, 13, 14, 15, 16, 17])):
  d, st = load(route, segs)
  T0 = st[segs[0]]
  bs = bends(d)
  for x, y in zip(bs, bs[1:]):
    if x[2] != y[2] and y[0] - x[1] < 8:
      page['sbends'].append(dict(name=f'{route[:8]} {x[0] - T0 + segs[0] * 60:.0f}s', bends=[[round(x[0] - x[0], 1), round(x[1] - x[0], 1), x[2]], [round(y[0] - x[0], 1), round(y[1] - x[0], 1), y[2]]],
                                 **cut(d, x[0] - 10, y[1] + 14, x[0])))
  for x in bs:
    if x[1] - x[0] >= 4 and d['v'][(d['t'] >= x[0]) & (d['t'] <= x[1])].mean() > 12:
      m = (d['t'] > x[0] - 2) & (d['t'] < x[1] + 8)
      if d['act'][m].all() and not d['blk'][m].any() and (d['prob'][m] > 0.5).all():
        page['exits'].append(dict(name=f'{route[:8]} {x[0] - T0 + segs[0] * 60:.0f}s', bend=[0.0, round(x[1] - x[0], 1), x[2]], **cut(d, x[0] - 8, x[1] + 12, x[0])))
json.dump(page, open('page.json', 'w'))
print({k: [p['name'] for p in v] for k, v in page.items()})
for w in page['windows']:
  print(w['name'], 'window', w['close'], '->', w['end'], 'metrics from', w['bend_from'], '| bends (s in seg):', w['bends'])
for s in page['sbends']:
  print('S', s['name'], s['bends'])
