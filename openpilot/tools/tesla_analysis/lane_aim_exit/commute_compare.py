"""The same road driven on different days, bend by bend, by what the drive had on (run commute_extract.py first).

  commute_compare.py <reference route> plain=a,b hold=c aim=d,e      [MINCOV=0.4 in the environment]

Samples are put on the reference route by GPS (10 m bins), kept where steering, no blinker, v > 12 m/s, lines seen.
Per bend (|curvature| > 0.0007 for 150 m or more): the mean lean toward the inside, and the furthest the car gets toward the
outside line in the 250 m after the bend. Positions + right, taken from the road-frame lines in drivingModelData.
"""
import sys, numpy as np
import os
GROUPS = {k: v.split(',') for k, v in (x.split('=') for x in sys.argv[2:])}
for g in ('plain', 'hold', 'aim'): GROUPS.setdefault(g, [])
REF = sys.argv[1]; BIN = 10.0; MINCOV = float(os.environ.get('MINCOV', 0.4))
def xy(a): return np.c_[(a[:, 13] - 37.0) * 111e3, (a[:, 14] + 122.0) * 88e3]
D = {r: np.load(f'q_{r}.npy') for g in GROUPS.values() for r in g}
ref = D[REF]; rp = xy(ref); keep = np.r_[True, np.hypot(*np.diff(rp, axis=0).T) > 1.0]; rp = rp[keep]
cum = np.r_[0, np.cumsum(np.hypot(*np.diff(rp, axis=0).T))]
fine = np.arange(0, cum[-1], 5.0); rp = np.c_[np.interp(fine, cum, rp[:, 0]), np.interp(fine, cum, rp[:, 1])]; cum = fine; nb = int(cum[-1] // BIN) + 1
prof = {}
for r, a in D.items():
  p = xy(a); d2 = ((p[:, None, :] - rp[None, :, :]) ** 2).sum(-1); j = d2.argmin(1); near = np.sqrt(d2.min(1)) < 25
  s = cum[j]; ok = near & (a[:, 4] > .5) & (a[:, 5] > .5) & (a[:, 8] > 12) & (a[:, 9] < .5) & (a[:, 10] < .5) & (a[:, 11] > .5)
  pos = -(a[:, 2] + a[:, 3]) / 2; k = a[:, 12]
  P, K, V, N = (np.full(nb, np.nan) for _ in range(4))
  for b in range(nb):
    m = ok & (s >= b * BIN) & (s < (b + 1) * BIN)
    if m.any(): P[b], K[b], V[b], N[b] = pos[m].mean(), k[m].mean(), a[m, 8].mean(), m.sum()
  prof[r] = (P, K, V)
  cov = np.isfinite(P); print(r, 'covers %.1f km of the %.1f km reference, mean v %.0f km/h' % (cov.sum() * BIN / 1e3, cum[-1] / 1e3, np.nanmean(V) * 3.6))
Kall = np.nanmean([prof[r][1] for r in D], axis=0)
Ks = np.convolve(np.nan_to_num(Kall), np.ones(3) / 3, 'same')
hard = np.abs(Ks) > 0.0007
bends = []; i = 0
while i < nb:
  if not hard[i]: i += 1; continue
  j = i
  while j + 1 < nb and (hard[j + 1] or hard[j + 1:j + 6].any()): j += 1
  if (j - i + 1) * BIN >= 150: bends.append((i, j, np.sign(Ks[i:j + 1].mean())))
  i = j + 1
EXIT = 25
rows = []
print('\n bend  from-start  len   dir | inside lean in the bend (m) plain / hold / aim | furthest out in the 250 m after (m) plain / hold / aim')
for n, (i, j, side) in enumerate(bends):
  line = {}
  for g, rs in GROUPS.items():
    ins, out = [], []
    for r in rs:
      P = prof[r][0]
      if np.isfinite(P[i:j + 1]).mean() > MINCOV and np.isfinite(P[j + 1:j + 1 + EXIT]).mean() > MINCOV:
        ins.append(side * np.nanmean(P[i:j + 1])); out.append(np.nanmax(-side * P[j + 1:j + 1 + EXIT]))
    line[g] = (np.mean(ins) if ins else np.nan, np.mean(out) if out else np.nan, len(ins))
  rows.append(line)
  f = lambda g, k: ('%+.2f' % line[g][k]) if np.isfinite(line[g][k]) else '  - '
  print(f'{n:4d} {i*BIN/1e3:7.1f} km {(j-i+1)*BIN:4.0f} m {"R" if side>0 else "L"} |      {f("plain",0)} / {f("hold",0)} / {f("aim",0)}      |      {f("plain",1)} / {f("hold",1)} / {f("aim",1)}      (n {line["plain"][2]}/{line["hold"][2]}/{line["aim"][2]})')
def cmp(a, b, k, name):
  x = [(r[a][k], r[b][k]) for r in rows if np.isfinite(r[a][k]) and np.isfinite(r[b][k])]
  if x: x = np.array(x); print(f'  {name}: {a} {x[:,0].mean():+.3f} vs {b} {x[:,1].mean():+.3f} over {len(x)} bends, {b} worse in {(x[:,1] > x[:,0] + 0.05).sum()}, better in {(x[:,1] < x[:,0] - 0.05).sum()}')
for k, nm in ((0, 'inside lean'), (1, 'furthest out after exit')):
  for a, b in (('plain', 'hold'), ('plain', 'aim'), ('hold', 'aim')): cmp(a, b, k, nm)
