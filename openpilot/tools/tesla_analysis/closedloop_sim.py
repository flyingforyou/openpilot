"""Closed-loop replay (run ON THE DEVICE -- the acados solver only exists there):

  cd /data/openpilot && SIMDELAY=0.15 SIMTAU=0.10 PYTHONPATH=/data/openpilot \\
    python3 openpilot/tools/tesla_analysis/closedloop_sim.py <route> <seg> <t_start> <t_end>
  optional: SIMROOT=<dir holding route--seg/rlog.zst>  VARIANTS='[["name", {"Param": value}], ...]'

  d=0.15 s / tau=0.10 s is the actuator response fitted by actuator_lag.py (RMS 0.137 m/s^2).

Closed-loop replay: does the stop-and-go lurch go away when aLeadK comes from the radar?

The lead is exogenous -- its recorded position, speed and true acceleration (radar LongAccel +
recorded a_ego). The ego car is simulated: the carrot planner's output drives a delayed first-order
actuator, and the planner is fed a synthetic radarState/carState built from the simulated state.
Mode A estimates the lead's acceleration the old way (radard's Kalman on vLead); mode B uses the
radar value. Mode A must reproduce the recorded lurch for the comparison to mean anything.
"""
import sys, math, collections
import numpy as np, zstandard
from openpilot.cereal import log
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.radard import KalmanParams
from openpilot.selfdrive.controls.lib.longitudinal_planner_carrot import CarrotLongitudinalPlanner
from openpilot.selfdrive.debug.shadow_replay import _ReplaySM

route, seg, T0, T1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
import os, json
DELAY = float(os.environ.get('SIMDELAY', 0.15)); TAU = float(os.environ.get('SIMTAU', 0.35)); DT = 0.05
ADDRS = {0x310 + 3*i for i in range(32)}
def bits(d, s, n): return (int.from_bytes(bytes(d).ljust(8, b'\0')[:8], 'little') >> s) & ((1 << n) - 1)

import os
path = f"{os.environ.get('SIMROOT', '/data/media/0/realdata')}/{route}--{seg}/rlog.zst"
evts = list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(path, 'rb'), read_across_frames=True).read()))
CP = next(e.carParams for e in evts if e.which() == 'carParams') if any(e.which()=='carParams' for e in evts) else None
if CP is None:
    import glob
    from openpilot.common.params import Params
    from opendbc.car import structs
    CP = log.Event.from_bytes(open('/data/params/d/CarParamsPersistent','rb').read()) if False else None
    from openpilot.cereal import car
    CP = car.CarParams.from_bytes(Params().get("CarParamsPersistent"))

def run(mode, cap=0.0, credit_cap=False, ovr=None):
    NEEDED = ('carControl', 'carState', 'controlsState', 'radarState', 'modelV2', 'selfdriveState', 'vehicleParameters')
    sm = _ReplaySM(); planner = None; pts = {}; t0 = None
    kp = KalmanParams(DT); kf = None
    xe = ve = ae = None; x_rec = 0.0; last_t = None
    cmd_hist = collections.deque([0.0] * int(round(DELAY / DT) + 1), maxlen=int(round(DELAY / DT) + 1))
    out = []; rec_cs = None; rec_rs = None; state = {'d': 99.0, 've': 0.0}
    import openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc as lm
    if not hasattr(lm, '_orig_sef'): lm._orig_sef = lm.get_stopped_equivalence_factor
    if credit_cap:
        # credit the lead's braking distance only up to our own speed: a lead that is faster than
        # us is not assumed to stay faster
        lm.get_stopped_equivalence_factor = lambda v_lead, cb2=2.5: lm._orig_sef(np.minimum(v_lead, max(state['ve'], 0.0)), cb2)
    else:
        lm.get_stopped_equivalence_factor = lm._orig_sef
    for e in evts:
        w = e.which(); t = e.logMonoTime / 1e9
        if t0 is None and w == 'carState': t0 = t
        if w == 'can':
            for c in e.can:
                if c.src == 1 and c.address in ADDRS:
                    if not bits(c.dat, 62, 1): pts.pop(c.address, None); continue
                    pts[c.address] = (bits(c.dat,0,12)*0.0625, bits(c.dat,12,12)*0.0625-128, bits(c.dat,40,10)*0.03125-16)
            continue
        if w == 'carState':
            if last_t is not None: x_rec += e.carState.vEgo * (t - last_t)
            last_t = t; rec_cs = e.carState
        if w == 'radarState': rec_rs = e.radarState
        if w in NEEDED: sm.data[w] = getattr(e, w); sm.logMonoTime[w] = e.logMonoTime; sm.seen[w] = sm.alive[w] = True
        if w != 'modelV2' or len(sm.data) < len(NEEDED) or t0 is None: continue
        rt = t - t0
        if rt < T0 - 3.0 or rt > T1: continue
        if rec_rs is None or rec_cs is None: continue
        rs_rec = rec_rs; ld = rs_rec.leadOne
        if not ld.present: continue
        # lead: exogenous absolute state
        x_lead = x_rec + ld.dRel; v_lead = ld.vLead
        a_true = None
        for dx, vx, ax in pts.values():
            if abs(dx - ld.dRel) < 2 and abs(vx - ld.vRel) < 1.5: a_true = ax + rec_cs.aEgo; break
        if kf is None: kf = KF1D([[v_lead], [0.0]], kp.A, kp.C, kp.K)
        kf.update(v_lead)
        if rt < T0 or xe is None:     # warm-up (or no earlier lead frame): track the recording
            xe, ve, ae = x_rec, rec_cs.vEgo, rec_cs.aEgo
        a_lead = float(kf.x[1][0]) if mode == 'A' else (a_true if a_true is not None else float(kf.x[1][0]))
        cs = rec_cs.as_builder(); cs.vEgo = cs.vEgoRaw = cs.vEgoCluster = float(max(ve, 0.0)); cs.aEgo = float(ae)
        cs.standstill = ve < 0.1; cs.gasPressed = False; cs.brakePressed = False
        rs = rs_rec.as_builder(); L = rs.leadOne
        L.dRel = float(x_lead - xe); L.vRel = float(v_lead - ve); L.vLead = L.vLeadK = float(v_lead); L.aLeadK = float(a_lead)
        rs.leadTwo.present = False
        state['d'] = float(x_lead - xe); state['ve'] = float(ve)
        sm.data['carState'] = cs; sm.data['radarState'] = rs
        sm.frame += 1; sm.updated = dict.fromkeys(sm.data, True)
        if planner is None:
            planner = CarrotLongitudinalPlanner(CP)
            if ovr:
                P = planner.carrot.params
                for fn in ('get_float', 'get_int', 'get_bool'):
                    orig_fn = getattr(P, fn)
                    def wrapped(key, *a, _o=orig_fn, _fn=fn, **k):
                        if key in ovr:
                            v = ovr[key]
                            return bool(v) if _fn == 'get_bool' else (int(v) if _fn == 'get_int' else float(v))
                        return _o(key, *a, **k)
                    setattr(P, fn, wrapped)
            if cap > 0:
                orig = planner.carrot.get_carrot_accel
                def capped(v, _o=orig):
                    a = _o(v); d = state['d']; kmh = v * 3.6
                    near = float(np.interp(d, [10., 35.], [cap, a])); w_ = float(np.interp(kmh, [40., 60.], [1., 0.]))
                    return min(a, w_ * near + (1 - w_) * a)
                planner.carrot.get_carrot_accel = capped
        planner.update(sm)
        cmd = float(planner.planner.output_a_target)
        if rt >= T0:
            cmd_hist.append(cmd); a_in = cmd_hist[0]
            ae += (a_in - ae) * DT / TAU; ve = max(ve + ae * DT, 0.0); xe += ve * DT
            out.append((rt, ve * 3.6, float(x_lead - xe), v_lead * 3.6, a_lead, a_true if a_true is not None else float('nan'), cmd,
                        rec_cs.vEgo * 3.6, ld.dRel, sm.data['carControl'].actuators.accel))
    return np.array(out)

def stats(c, lbl):
    sw = 0; st = 0
    for x in c:
        if x > 0.5 and st <= 0: st = 1; sw += 1
        elif x < -0.5 and st >= 0: st = -1; sw += 1
    return f"{lbl:>26}  가속max {c.max():+.2f}  제동min {c.min():+.2f}  ±0.5교차 {sw:>2}회  |Δ|>1.5 {np.sum(np.abs(np.diff(c))>0.075):>3}"
VARIANTS = json.loads(os.environ.get('VARIANTS', 'null'))
if VARIANTS:
    rec = None
    for name, ovr in VARIANTS:
        r = run('B', ovr=ovr)
        if rec is None:
            rec = r
            print(stats(r[:, 9], "녹화 실제 명령"))
        print(stats(r[:, 6], name) + f"   최소거리 {r[:,2].min():.1f}m  평균거리 {np.mean(r[:,2]):.1f}m")
    raise SystemExit
res = {'A': run('A'), 'B': run('B'), 'C': run('A', 1.0), 'D': run('B', 1.0), 'E': run('A', 0.8), 'F': run('B', 0.8), 'G': run('A', 0.0, True), 'H': run('B', 0.0, True)}
def stats(c, lbl):
    sw = 0; st = 0
    for x in c:
        if x > 0.5 and st <= 0: st = 1; sw += 1
        elif x < -0.5 and st >= 0: st = -1; sw += 1
    return f"{lbl:>18}  가속max {c.max():+.2f}  제동min {c.min():+.2f}  ±0.5교차 {sw:>2}회  |Δ|>1.5 구간 {np.sum(np.abs(np.diff(c))>0.075)}"
A, B = res['A'], res['B']
print(f"{route} seg{seg} t+{T0}~{T1}  (지연 {DELAY}s, 1차지연 {TAU}s)\n")
print(stats(A[:, 9], "녹화 실제 명령"))
print(stats(A[:, 6], "시뮬 A (칼만)"))
print(stats(B[:, 6], "시뮬 B (레이더)"))
for k,lbl in (("C","C 칼만+근거리1.0"),("D","D 레이더+근거리1.0"),("E","E 칼만+근거리0.8"),("F","F 레이더+근거리0.8"),("G","G 칼만+크레딧상한"),("H","H 레이더+크레딧상한")):
    print(stats(res[k][:, 6], lbl))
print("\n최소거리  녹화 %.1f  " % A[:,8].min() + "  ".join(f"{k} {res[k][:,2].min():.1f}" for k in "ABCDEFGH"))
print(f"\n{'t+':>5} {'녹화v':>5} {'녹화cmd':>7} | {'A v':>5} {'A거리':>6} {'A aLead':>7} {'A cmd':>6} | {'B v':>5} {'B거리':>6} {'B aLead':>7} {'B cmd':>6}")
for i in range(0, min(len(A), len(B)), 8):
    a, b = A[i], B[i]
    print(f"{a[0]:>5.1f} {a[7]:>5.1f} {a[9]:>+7.2f} | {a[1]:>5.1f} {a[2]:>5.1f}m {a[4]:>+7.2f} {a[6]:>+6.2f} | {b[1]:>5.1f} {b[2]:>5.1f}m {b[4]:>+7.2f} {b[6]:>+6.2f}")
