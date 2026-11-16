"""Opposite bends back to back (S / chained hairpins) in qlogs: where, and which segment."""
import sys, numpy as np
sys.argv, routes = [sys.argv[0]], sys.argv[1:]
exec(open(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'exits.py')).read().split('for route in sys.argv')[0])
import glob
for route in routes:
  cs, cc, ln, _ = rows(route)
  if not len(cc): continue
  v = np.interp(cc[:, 0], cs[:, 0], cs[:, 1]); lat = cc[:, 2] * v ** 2; t = cc[:, 0]
  # segment start times
  seg_t0 = []
  for p in sorted(glob.glob(route + '--*/qlog.zst'), key=lambda p: int(p.split('--')[-1].split('/')[0])):
    seg_t0.append(int(p.split('--')[-1].split('/')[0]))
  bends, i = [], 0
  while i < len(t):
    if abs(lat[i]) > 0.8 and v[i] > 8:
      j = i
      while j + 1 < len(t) and abs(lat[j + 1]) > 0.4 and np.sign(lat[j + 1]) == np.sign(lat[i]): j += 1
      if t[j] - t[i] >= 2.5:
        bends.append((t[i], t[j], np.sign(np.mean(lat[i:j])), np.max(np.abs(lat[i:j])), np.mean(v[i:j]), cc[i:j, 1].min()))
      i = j + 1
    else:
      i += 1
  T0 = t[0]
  for a, b in zip(bends, bends[1:]):
    if a[2] != b[2] and b[0] - a[1] < 8:
      print(f"{route.split('/')[-1][:8]} {a[0] - T0:6.0f}s {'L' if a[2] > 0 else 'R'}({a[1] - a[0]:.0f}s, {a[3]:.1f} m/s2) -> gap {b[0] - a[1]:.1f}s -> {'L' if b[2] > 0 else 'R'}({b[1] - b[0]:.0f}s, {b[3]:.1f}) v{a[4]:.0f}/{b[4]:.0f} lat_on {int(min(a[5], b[5]))}  seg~{int((a[0] - T0) // 60)}")
