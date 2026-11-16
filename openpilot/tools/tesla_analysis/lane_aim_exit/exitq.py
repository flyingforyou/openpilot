"""Same measure from qlogs (2 Hz lane position): every clean bend exit, peak offset in the 5 s after."""
import sys, numpy as np
sys.argv, routes = [sys.argv[0]], sys.argv[1:]
exec(open(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'exits.py')).read().split('for route in sys.argv')[0])
for route in routes:
  cs, cc, ln, _ = rows(route)
  if not len(cc) or not len(ln): continue
  v = np.interp(cc[:, 0], cs[:, 0], cs[:, 1]); lat = cc[:, 2] * v ** 2
  t = cc[:, 0]; pos = -(ln[:, 1] + ln[:, 2]) / 2
  ok = (np.minimum(ln[:, 3], ln[:, 4]) > 0.5) & (ln[:, 5] == 0)
  i = 0; out = []
  while i < len(t):
    if not ((abs(lat[i]) > 1.0) and v[i] > 12 and cc[i, 1] > 0): i += 1; continue
    j = i
    while j + 1 < len(t) and abs(lat[j + 1]) > 0.5: j += 1
    if t[j] - t[i] >= 4:
      side = np.sign(np.mean(lat[i:j]))
      w = ok & (ln[:, 0] > t[j]) & (ln[:, 0] < t[j] + 5)
      ww = (cs[:, 0] > t[i] - 2) & (cs[:, 0] < t[j] + 6)
      clean = not cs[ww, 2].any() and cc[(t > t[i] - 2) & (t < t[j] + 6), 1].min() > 0 and w.sum() >= 6
      if clean:
        k = np.argmax(np.abs(pos[w])); peak = pos[w][k]
        out.append((abs(peak), 'inside-dir' if np.sign(peak) == -side else 'outside-dir'))
    i = j + 1
  print("JSON", route.split("/")[-1][:8], [[round(float(o[0]), 3), o[1]] for o in out])
  if out:
    a = np.array([o[0] for o in out]); n_in = sum(o[1] == 'inside-dir' for o in out)
    print(f"{route.split('/')[-1][:8]}: {len(out):2d} clean exits, peak 0-5 s after: median {np.median(a):.2f} m, max {a.max():.2f}, >0.3 m {np.mean(a > 0.3) * 100:3.0f}%, toward the bend's inside {n_in}/{len(out)}")
