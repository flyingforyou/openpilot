"""openpilot's stock longitudinal maneuvers plus standard ACC scenarios, run through closedloop_sim.py.

The maneuvers are selfdrive/test/longitudinal_maneuvers' (written for the stock planner and its own
plant); here the lead is synthetic (SIMSYNTH), the background is a recorded engaged minute, the
planner is carrot's and LongControl sits between it and the fitted actuator lag (SIMLOC).

  SIMCP=<CarParamsPersistent> .venv/bin/python3 openpilot/tools/tesla_analysis/acc_scenarios.py
  optional: SCNVARS='[["name", {...}], ...]' (closedloop_sim VARIANTS), SCNJSON=1 (machine-readable rows)
"""
import json, math, os, subprocess, sys, tempfile, numpy as np
from multiprocessing import Pool
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
S = tempfile.mkdtemp(prefix='acc_scenarios_')
BG = ('00000178--d7a467d67f', 13, 5.0)    # background: engaged highway minute, scenario starts at t=5

def sin_profile(T=50, base=20, amp=2, period=10):
    bp = list(np.arange(0, T + 0.01, 0.25)); return bp, [base + amp * math.sin(2 * math.pi * t / period) for t in bp]

def stop_go(cycles=3):
    bp, v, t = [0.0], [0.0], 0.0
    for _ in range(cycles):
        for dt, vv in ((2.0, 0.0), (8 / 1.5, 8.0), (3.0, 8.0), (4.0, 0.0)):
            t += dt; bp.append(t); v.append(vv)
    return bp, v

SB, SV = sin_profile(); GB, GV = stop_go()
SC = [
  # openpilot selfdrive/test/longitudinal_maneuvers (cruise 50 m/s unless the maneuver sets it)
  ("[순정] 정지차 접근 25m/s, 120m",        20, dict(speed=25, distance=120, bp=[0, 1], v=[30, 0], cruise=50)),
  ("[순정] 정지차 접근 20m/s, 90m",         20, dict(speed=20, distance=90, bp=[0, 1], v=[20, 0], cruise=50)),
  ("[순정] 20m/s 추종 → 앞차 1m/s² 정지",  50, dict(speed=20, distance=35, bp=[0, 15, 35], v=[20, 20, 0], cruise=50)),
  ("[순정] 20m/s 추종 → 앞차 2m/s² 정지",  50, dict(speed=20, distance=35, bp=[0, 15, 25], v=[20, 20, 0], cruise=50)),
  ("[순정] 20m/s 추종 → 앞차 3m/s² 정지",  50, dict(speed=20, distance=35, bp=[0, 15, 21.66], v=[20, 20, 0], cruise=50)),
  ("[순정] 3+m/s² (감속 중 앞차 첫 인지)",  40, dict(speed=20, distance=35, bp=[2, 2.01, 8.8], v=[20, 20, 0], prob=[0, 1, 1], cruise=20)),
  ("[순정] 정지차 늦게 인지 20m/s, 120m",   30, dict(speed=20, distance=120, bp=[0, 2, 2.01], v=[0, 0, 0], prob=[0, 0, 1], cruise=20)),
  ("[순정] 느린 차 끼어듦 (leadTwo) 20→15",20, dict(speed=20, distance=50, bp=[1, 11], v=[15, 15], only_lead2=True, cruise=50)),
  ("[순정] 정지 상태에서 출발",              20, dict(speed=0, distance=6, bp=[1, 10, 15], v=[0, 0, 7], cruise=50)),
  # standard ACC situations (ISO 15622 / Euro NCAP style, OpenACC string stability)
  ("[추가] 앞차 빠짐 → 정지차 60m 드러남",  30, dict(speed=20, distance=35, bp=[0, 30], v=[20, 20], switch=dict(t=10, distance=60, v=0), cruise=25)),
  ("[추가] 앞차 빠짐 → 정지차 80m 드러남",  30, dict(speed=20, distance=35, bp=[0, 30], v=[20, 20], switch=dict(t=10, distance=80, v=0), cruise=25)),
  ("[추가] 앞차 빠짐 → 정지차 100m 드러남", 30, dict(speed=20, distance=35, bp=[0, 30], v=[20, 20], switch=dict(t=10, distance=100, v=0), cruise=25)),
  ("[추가] 근거리 끼어듦 15m, 15m/s",       25, dict(speed=20, distance=40, bp=[0, 30], v=[20, 20], switch=dict(t=10, distance=15, v=15), cruise=25)),
  ("[추가] 앞차 속도 출렁임 ±2m/s, 10초",   50, dict(speed=20, distance=20, bp=SB, v=SV, cruise=30)),
  ("[추가] 가다서다 3회 (0↔8m/s)",          45, dict(speed=0, distance=5, bp=GB, v=GV, cruise=25)),
  ("[추가] 앞차 급가속 이탈 10→25m/s",      25, dict(speed=10, distance=25, bp=[0, 2, 12], v=[10, 10, 25], cruise=30)),
]
VARS = json.loads(os.environ['SCNVARS']) if os.environ.get('SCNVARS') else [["MPC", {"LongResearchPath": 0, "TeslaMapAutoSpeed": 0}], ["IIDM", {"TeslaMapAutoSpeed": 0}]]

def run(i):
    name, dur, syn = SC[i]; dump = f"{S}/scn_{i}.json"
    env = dict(os.environ, SIMSYNTH=json.dumps(syn), SIMLOC='1', SIMDUMP=dump, SIMCP=os.environ['SIMCP'],
               SIMROOT=f"{ROOT}/op-logs", SIMDELAY='0.15', SIMTAU='0.10', VARIANTS=json.dumps(VARS),
               PYTHONPATH=f"{ROOT}:{ROOT}/opendbc_repo")
    r = subprocess.run([f"{ROOT}/.venv/bin/python3", f"{ROOT}/openpilot/tools/tesla_analysis/closedloop_sim.py",
                        BG[0], str(BG[1]), str(BG[2]), str(BG[2] + dur)], env=env, capture_output=True, text=True)
    if r.returncode: return name, None, r.stderr[-800:]
    D = json.load(open(dump)); res = {}
    for k, rows in D.items():
        a = np.array(rows); t = a[:, 0] - a[0, 0]; ve = a[:, 1] / 3.6; gap = a[:, 2]; vl = a[:, 3] / 3.6; cmd = a[:, 5]
        close = ve - vl; ttc = np.where(close > 0.5, gap / np.maximum(close, 1e-3), 99)
        m = dict(crash=bool(gap.min() < 0.4), dmin=gap.min(), dend=gap[-1], amin=cmd.min(), amax=cmd.max(),
                 jerk=float(np.sqrt(np.mean((np.diff(cmd) / 0.05) ** 2))), ttc=ttc.min())
        if 'switch' in syn: m['dmin'] = gap[t >= syn['switch']['t'] + 0.05].min()
        if '출렁임' in name: m['amp'] = np.std(ve[t > 10]) / np.std(vl[t > 10])
        if '출발' in name:
            moving = np.where((vl > 0.05) & (t > 10))[0]; go = np.where((ve > 0.3) & (t > 10))[0]
            m['start'] = (t[go[0]] - t[moving[0]]) if len(go) and len(moving) else float('nan')
        res[k] = m
    return name, res, None

if __name__ == '__main__':
    with Pool(8) as p:
        for name, res, err in p.imap(run, range(len(SC))):
            if err: print(f"{name}: ERROR {err}"); continue
            print(f"\n{name}")
            for k, m in res.items():
                if os.environ.get('SCNJSON'): print('JSON', json.dumps([name, k, {kk: (float(vv) if not isinstance(vv, bool) else vv) for kk, vv in m.items()}]))
                extra = (f"  증폭 {m['amp']:.2f}" if 'amp' in m else '') + (f"  출발지연 {m['start']:.2f}s" if 'start' in m else '')
                print(f"  {k:5s} {'충돌!!' if m['crash'] else '      '} 최소거리 {m['dmin']:5.1f}m  최종거리 {m['dend']:5.1f}m  "
                      f"최대감속 {m['amin']:+.2f}  최대가속 {m['amax']:+.2f}  저크 {m['jerk']:.2f}  최소TTC {m['ttc']:4.1f}s{extra}")
