"""Lead-following regression: 붕끽 events, launches and the synthetic ACC maneuvers, for any variants.

Every longitudinal change gets judged on all three, because each one hides something the others
show: 2026-10-01's IIDM path scored smoother on replays and was unusable on the road (it simply
stopped following), and a fix that softens braking can quietly soften launches too. **Launch
acceleration is a gate, not a statistic**: a variant whose launches are weaker than the baseline's
is flagged, whatever else it improves.

  SIMCP=<CarParamsPersistent> .venv/bin/python3 openpilot/tools/tesla_analysis/follow_regression.py \\
      '[["baseline", {...}], ["candidate", {...}]]'

The first variant is the baseline. Windows come from regression_windows.txt; settings that were live
on the car at the time (map auto-speed off from 10/01 12:57 UTC) are applied per route.
"""
import json, os, subprocess, sys, tempfile, statistics as st
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
OUT = tempfile.mkdtemp(prefix='follow_reg_')
MAP_OFF_ROUTES = ('0000017b', '0000017c', '0000017d', '0000017f')
LAUNCH_A4_TOL = 0.05     # m/s^2 below the baseline's first-4 s launch accel counts as sluggish
LAUNCH_V8_TOL = 1.0      # km/h below the baseline's speed 8 s after the lead moves


def windows():
    for l in open(os.path.join(HERE, 'regression_windows.txt')):
        if l.strip() and not l.startswith('#'):
            yield l.split()[:5]


def run(job):
    (route, seg, t0, t1, kind), variants = job
    dump = f"{OUT}/{route}_{seg}_{t0}.json"
    extra = {"TeslaMapAutoSpeed": 0} if route.startswith(MAP_OFF_ROUTES) else {}
    env = dict(os.environ, SIMRADARD='1', SIMLEAD2='1', SIMLOC='1', SIMDUMP=dump, SIMROOT=f"{ROOT}/op-logs",
               SIMDELAY='0.15', SIMTAU='0.10', PYTHONPATH=f"{ROOT}:{ROOT}/opendbc_repo",
               VARIANTS=json.dumps([[n, {**extra, **o}] for n, o in variants]))
    r = subprocess.run([sys.executable, f"{HERE}/closedloop_sim.py", route, seg, t0, t1], env=env, capture_output=True, text=True)
    return kind, (json.load(open(dump)) if r.returncode == 0 else None)


def bung_metrics(a):
    t, ae, gap = a[:, 0], a[:, 9], a[:, 2]
    j = np.diff(ae) / np.diff(t)
    k = int(np.argmin(ae))                    # the hardest braking moment in the window
    s0 = k
    while s0 > 0 and ae[s0 - 1] >= ae[s0] - 1e-3 and ae[s0 - 1] < -0.05:
        s0 -= 1                               # walk back up the slope to where this brake application began
    return dict(apk=ae.max(), p5=np.percentile(ae, 5), dmin=gap.min(), jerk=float(np.sqrt(np.mean(j ** 2))),
                amin=ae.min(), onset=float(-j.min()), ramp=float(t[k] - t[s0]))


def launch_metrics(a):
    t = a[:, 0] - a[0, 0]; ve = a[:, 1] / 3.6; gap = a[:, 2]; vl = a[:, 3] / 3.6; ae = a[:, 9]
    i0 = int(np.argmax(vl > 1.5)); go = np.where((ve > 0.5) & (t >= t[i0]))[0]
    w4 = (t >= t[i0]) & (t <= t[i0] + 4); w8 = (t >= t[i0]) & (t <= t[i0] + 8)
    return dict(delay=(t[go[0]] - t[i0]) if len(go) else np.nan, a4=ae[w4].mean(),
                v8=ve[int(np.argmin(np.abs(t - (t[i0] + 8))))] * 3.6, grow=gap[w8].max() - gap[i0])


def main():
    variants = json.loads(sys.argv[1])
    names = [n for n, _ in variants]
    with Pool(12) as p:
        res = p.map(run, [(w, variants) for w in windows()])
    B = {n: [] for n in names}; L = {n: [] for n in names}
    for kind, D in res:
        if D is None:
            continue
        for n in names:
            a = np.array(D[n])
            (B if kind == 'SG' else L)[n].append(bung_metrics(a) if kind == 'SG' else launch_metrics(a))
    print(f"\n붕끽 {len(B[names[0]])}장면 (실제 차량 가속도) · 체감 기준: 0.1~0.2 m/s² 미만 차이는 못 느낌")
    print(f"  {'설정':18s} {'붕(최대)':>8s} {'끽 최대(평균)':>11s} {'가장 센 끽':>9s} {'제동 걸리는 저크(최대)':>16s} {'최대감속까지':>10s} {'최소간격':>8s}")
    for n in names:
        R = B[n]
        print(f"  {n:18s} {st.mean(r['apk'] for r in R):+8.2f} {st.mean(r['amin'] for r in R):+11.2f} {min(r['amin'] for r in R):+9.2f} "
              f"{st.mean(r['onset'] for r in R):13.1f} m/s³ {st.mean(r['ramp'] for r in R):9.2f}s {min(r['dmin'] for r in R):7.1f}m")
    print(f"\n출발 {len(L[names[0]])}장면  ※ 출발 가속은 합격 조건: 기준보다 답답하면 표시")
    print(f"  {'설정':18s} {'출발지연':>8s} {'첫4초가속':>9s} {'8초뒤속도':>10s} {'간격벌어짐':>10s}  판정")
    base = {k: st.median(r[k] for r in L[names[0]]) for k in ('delay', 'a4', 'v8', 'grow')}
    for n in names:
        m = {k: st.median(r[k] for r in L[n]) for k in ('delay', 'a4', 'v8', 'grow')}
        slow = m['a4'] < base['a4'] - LAUNCH_A4_TOL or m['v8'] < base['v8'] - LAUNCH_V8_TOL
        print(f"  {n:18s} {m['delay']:7.2f}s {m['a4']:+9.2f} {m['v8']:8.1f}km/h {m['grow']:9.1f}m  {'⚠ 출발 답답' if slow else 'OK'}")
    print("\n표준 시나리오")
    env = dict(os.environ, SCNJSON='1', SCNVARS=json.dumps([[n, {"TeslaMapAutoSpeed": 0, **o}] for n, o in variants]))
    r = subprocess.run([sys.executable, f"{HERE}/acc_scenarios.py"], env=env, capture_output=True, text=True)
    S = {}
    for line in r.stdout.splitlines():
        if line.startswith('JSON '):
            sc, n, m = json.loads(line[5:]); S.setdefault(sc, {})[n] = m
    for sc, d in S.items():
        print(f"  {sc[:30]:30s} " + "  ".join(f"{('충돌!' if d[n]['crash'] else '')}{d[n]['dmin']:5.1f}m {d[n]['amin']:+.2f}".rjust(15) for n in names))


if __name__ == '__main__':
    main()
