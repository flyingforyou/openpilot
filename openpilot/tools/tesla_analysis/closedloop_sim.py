"""Closed-loop replay: the ego car is simulated, everything else comes from a recorded drive.

Runs locally as well as on the device. Locally (x86, ~2 s per window instead of minutes):

  uv sync --frozen && source .venv/bin/activate && mkdir -p panda/board/obj
  scons -j16 openpilot/selfdrive/controls/lib/longitudinal_mpc_carrot msgq_repo/msgq openpilot/common
  SIMCP=<CarParamsPersistent copied off the device> SIMROOT=op-logs SIMDELAY=0.15 SIMTAU=0.10 \
    PYTHONPATH=$PWD:$PWD/opendbc_repo python3 openpilot/tools/tesla_analysis/closedloop_sim.py <route> <seg> <t0> <t1>

d=0.15 s / tau=0.10 s is the actuator response fitted by actuator_lag.py (RMS 0.137 m/s^2).

  VARIANTS='[["name", {"Param": value, ...}], ...]'   params overridden per variant; also
      "GAP": 1-7 (gap stalk), "KALMAN": 1 (lead accel from the Kalman filter instead of the radar),
      "rl.<CONST>": value (a research_long.py module constant)
      "mpc.<CONST>": value (a long_mpc.py LEAD_MODEL_* constant)
      "RADAR_DELAY": seconds (radard's vEgo alignment; default the log's CarParams.radarDelay)
      "rdd.<CONST>": value (a radard.py module constant)
      "cf.<CONST>": value (a carrot_functions.py STOP_* constant)
  SIMRADARD=1   rerun today's radard over the recorded radarTracks (lead hold, radar accel, cut-in)
                instead of trusting the radarState recorded at the time
  SIMLEAD2=1    keep leadTwo (cut-ins, the target lane's lead) instead of dropping it
  SIMTRACE=1    print frames where the command moves;  SIMDUMP=<file.json>  per-frame traces

The lead is exogenous: its recorded absolute position and speed. The planner gets a synthetic
carState/radarState built from the simulated ego, so a variant changes what happens next.
"""
import sys, math, collections, types
import numpy as np, zstandard
from openpilot.cereal import log
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.radard import KalmanParams
from openpilot.selfdrive.controls.lib.longitudinal_planner_carrot import CarrotLongitudinalPlanner
from openpilot.selfdrive.debug.shadow_replay import _ReplaySM

if __import__('os').environ.get('SIMNOOVR'):
    import openpilot.selfdrive.controls.lib.research_long as _rl; _rl.MPC_OVERRIDE_BELOW = -99.0
route, seg, T0, T1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
# SIMSYNTH: a synthetic lead in place of the recorded one -- {"speed": ego m/s, "distance": m, "bp": [s],
# "v": [lead m/s], "prob": [..], "cruise": m/s, "only_lead2": bool, "switch": {"t", "distance", "v"}}.
# The recording then only supplies the background (model output, car state); T0..T1 is the scenario.
import os, json
LOC = bool(os.environ.get('SIMLOC'))
NOLEAD = bool(os.environ.get('SIMNOLEAD'))
# SIMWARMUP: seconds before T0 the sim car just follows the recording (planner running, state building)
WARMUP = float(os.environ.get('SIMWARMUP', 3.0))
SYN = json.loads(os.environ['SIMSYNTH']) if os.environ.get('SIMSYNTH') else None
DELAY = float(os.environ.get('SIMDELAY', 0.15)); TAU = float(os.environ.get('SIMTAU', 0.35)); DT = 0.05
ADDRS = {0x310 + 3*i for i in range(32)}
def bits(d, s, n): return (int.from_bytes(bytes(d).ljust(8, b'\0')[:8], 'little') >> s) & ((1 << n) - 1)

import os
# SIMSEGS=n: play n consecutive segments as one run (times stay relative to the first segment)
evts = []
for _s in range(seg, seg + int(os.environ.get('SIMSEGS', 1))):
    path = f"{os.environ.get('SIMROOT', '/data/media/0/realdata')}/{route}--{_s}/rlog.zst"
    evts += list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(path, 'rb'), read_across_frames=True).read()))
CP = next(e.carParams for e in evts if e.which() == 'carParams') if any(e.which()=='carParams' for e in evts) else None

# The device's params as they were when this segment was recorded (initData.params). Without this the
# sim runs on this machine's unset params -- i.e. defaults -- which on 0x1a3 meant map auto-speed on
# where the car had it off, and carrot's accel profile and gap tables from defaults: the replayed car
# reached 87 km/h where the real one held 79 on the same approach. Variant overrides still win (they
# wrap the planner's/radard's params objects above this). SIMNOLOGPARAMS=1 restores the old behaviour.
LOGP = {}
if not os.environ.get('SIMNOLOGPARAMS'):
    _init = next((e.initData for e in evts if e.which() == 'initData'), None)
    if _init is not None:
        LOGP = {kv.key: bytes(kv.value) for kv in _init.params.entries}
if LOGP:
    from openpilot.common.params import Params as _P
    _get, _get_bool = _P.get, _P.get_bool
    def _logged_get(self, key, block=False, return_default=False):
        k = key.decode() if isinstance(key, bytes) else key
        if k in LOGP:
            t = self.get_type(k)
            default = self._default(self.check_key(k)) if return_default else None
            return self._cpp2python(t, LOGP[k] if LOGP[k] != b"" else default, default, k)
        return _get(self, key, block, return_default)
    def _logged_get_bool(self, key, block=False):
        k = key.decode() if isinstance(key, bytes) else key
        if k in LOGP:
            return LOGP[k] in (b"1", b"true", b"True")
        return _get_bool(self, key, block)
    _P.get, _P.get_bool = _logged_get, _logged_get_bool
if CP is None:
    import glob
    from openpilot.common.params import Params
    from opendbc.car import structs
    CP = log.Event.from_bytes(open('/data/params/d/CarParamsPersistent','rb').read()) if False else None
    from openpilot.cereal import car
    CP = car.CarParams.from_bytes(open(os.environ['SIMCP'], 'rb').read() if os.environ.get('SIMCP') else Params().get("CarParamsPersistent"))

def _sstar(planner, v, v_lead):
    """The IIDM path's dynamic target gap s* for the current frame (research_long, mode 0)."""
    from openpilot.selfdrive.controls.lib.research_long import gap_idm_params, IDM_A, IDM_B
    import openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc as lm_
    cr = planner.carrot
    seq = lm_.desired_follow_distance(v, v, cr.comfort_brake, cr.stop_distance, planner.planner.mpc.t_follow, cr.comfort_brake_2)
    T, s0 = gap_idm_params(v, float(seq), cr.stop_distance)
    return float(s0 + max(0.0, v * T + v * (v - v_lead) / (2 * math.sqrt(IDM_A * IDM_B))))

def run(mode, cap=0.0, credit_cap=False, ovr=None):
    import openpilot.selfdrive.controls.lib.research_long as _rl
    if not hasattr(_rl, '_defaults'): _rl._defaults = {k: getattr(_rl, k) for k in dir(_rl) if k.isupper()}
    for k, v in _rl._defaults.items(): setattr(_rl, k, v)
    import openpilot.selfdrive.controls.lib.carrot_t_follow as _ct
    if not hasattr(_ct, '_defaults'): _ct._defaults = {k: getattr(_ct, k) for k in dir(_ct) if k.isupper()}
    for k, v in _ct._defaults.items(): setattr(_ct, k, v)
    import openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc as _mpc
    if not hasattr(_mpc, '_defaults'): _mpc._defaults = {k: getattr(_mpc, k) for k in dir(_mpc) if k.startswith('LEAD_MODEL')}
    for k, v in _mpc._defaults.items(): setattr(_mpc, k, v)
    import openpilot.selfdrive.controls.radard as _rdd0
    import openpilot.selfdrive.controls.lib.carrot_functions as _cf0
    for k, v in getattr(_cf0, '_defaults', {}).items(): setattr(_cf0, k, v)
    for k, v in getattr(_rdd0, '_defaults', {}).items(): setattr(_rdd0, k, v)
    for k, v in (ovr or {}).items():
        if k.startswith('rl.'): setattr(_rl, k[3:], v)
        if k.startswith('ct.'): setattr(_ct, k[3:], v)
        if k.startswith('mpc.'): setattr(_mpc, k[4:], v)
        if k.startswith('cf.'):
            import openpilot.selfdrive.controls.lib.carrot_functions as _cf
            if not hasattr(_cf, '_defaults'): _cf._defaults = {kk: getattr(_cf, kk) for kk in dir(_cf) if kk.startswith('STOP_')}
            setattr(_cf, k[3:], v)
        if k.startswith('rdd.'):
            import openpilot.selfdrive.controls.radard as _rdd
            if not hasattr(_rdd, '_defaults'): _rdd._defaults = {kk: getattr(_rdd, kk) for kk in dir(_rdd) if kk.isupper()}
            setattr(_rdd, k[4:], v)
    gap = (ovr or {}).get('GAP')
    from openpilot.selfdrive.controls.lib.carrot_functions import GAP_TO_PERSONALITY_INT
    NEEDED = ('carControl', 'carState', 'controlsState', 'radarState', 'modelV2', 'selfdriveState', 'vehicleParameters')
    # SIMRADARD: rerun today's radard over the recorded radar tracks instead of trusting the radarState
    # recorded at the time -- the lead hold, radar accel and cut-in logic may have changed since.
    rd = None; rd_state = None; syn = None; loc = None
    if os.environ.get('SIMRADARD'):
        from openpilot.selfdrive.controls.radard import RadarD
        rd = RadarD((ovr or {}).get('RADAR_DELAY', CP.radarDelay))
        if ovr:   # radard reads its own Params; let variants override them too
            _rg = rd.params.get
            rd.params.get = lambda key, *a, _g=_rg, **k: ovr[key] if key in ovr else _g(key, *a, **k)
            rd.refresh_tuning()
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
        if w in NEEDED or (rd is not None and w in ('radarTracks', 'deviceMotion')):
            sm.data[w] = getattr(e, w); sm.logMonoTime[w] = e.logMonoTime; sm.seen[w] = sm.alive[w] = True
            if not hasattr(sm, 'recv_frame'): sm.recv_frame = {}
            sm.recv_frame[w] = sm.recv_frame.get(w, 0) + 1
        if rd is not None and w == 'modelV2' and 'radarTracks' in sm.data and 'selfdriveState' in sm.data and rec_cs is not None:
            keep = sm.data.get('carState'); sm.data['carState'] = rec_cs   # tracks are relative to the recorded car
            sm.updated = dict.fromkeys(sm.data, True)
            rd.update(sm, sm.data['radarTracks']); rd_state = rd.radar_state
            sm.data['carState'] = keep
        if w != 'modelV2' or len(sm.data) < len(NEEDED) or t0 is None: continue
        rt = t - t0
        if rt < T0 - WARMUP or rt > T1: continue
        if rec_rs is None or rec_cs is None: continue
        if SYN:
            # synthetic lead: the recording only supplies the background (model, states)
            ts = max(rt - T0, 0.0)
            if syn is None:
                syn = {'xl': SYN['distance'], 'vl': float(np.interp(0.0, SYN['bp'], SYN['v'])), 'switched': False, 'id': 1}
                xe, ve, ae = 0.0, SYN['speed'], 0.0
            if rt >= T0:
                sw = SYN.get('switch')
                if sw and not syn['switched'] and ts >= sw['t']:
                    syn.update(xl=xe + sw['distance'], vl=sw['v'], switched=True, id=2)
                v_new = float(sw['v']) if syn['switched'] else float(np.interp(ts, SYN['bp'], SYN['v']))
                a_syn = (v_new - syn['vl']) / DT
                syn['xl'] += (syn['vl'] + v_new) / 2 * DT; syn['vl'] = v_new
            else:
                a_syn = 0.0
            prob = float(np.interp(ts, SYN['bp'], SYN.get('prob', [1.0] * len(SYN['bp']))))
            x_lead, v_lead, a_lead, a_true = syn['xl'], syn['vl'], a_syn, a_syn
            ld = types.SimpleNamespace(present=prob > 0.5, dRel=float(x_lead - xe), radarTrackId=syn['id'])
            rs = rec_rs.as_builder()
            for slot, on in ((rs.leadOne, prob > 0.5 and not SYN.get('only_lead2')), (rs.leadTwo, prob > 0.5 and bool(SYN.get('only_lead2')))):
                slot.present = bool(on); slot.dRel = float(x_lead - xe); slot.yRel = 0.0; slot.vRel = float(v_lead - ve)
                slot.vLead = slot.vLeadK = float(v_lead); slot.aLeadK = slot.aLead = float(a_lead); slot.jLead = 0.0
                slot.radar = True; slot.radarTrackId = syn['id']; slot.modelProb = 1.0
            cs = rec_cs.as_builder(); cs.vEgo = cs.vEgoRaw = cs.vEgoCluster = float(max(ve, 0.0)); cs.aEgo = float(ae)
            cs.standstill = ve < 0.1; cs.gasPressed = False; cs.brakePressed = False
            cs.vCruise = cs.vCruiseCluster = float(SYN.get('cruise', 50.0) * 3.6); cs.cruiseState.speed = float(SYN.get('cruise', 50.0))
        rs_rec = rd_state if (rd is not None and rd_state is not None) else rec_rs
        if not SYN:
            ld = rs_rec.leadOne
        nolead = not SYN and not ld.present
        if nolead and not NOLEAD: continue
        if nolead:
            # SIMNOLEAD: keep simulating with no lead, so the model-stop (e2eStop) phase before a lead
            # exists is closed-loop too. A far phantom stands in for x_lead in the bookkeeping only;
            # radarState goes to the planner with no lead.
            if rt < T0 or xe is None:
                xe, ve, ae = x_rec, rec_cs.vEgo, rec_cs.aEgo
            x_lead, v_lead, a_lead, a_true = xe + 250.0, ve, 0.0, None
            cs = rec_cs.as_builder(); cs.vEgo = cs.vEgoRaw = cs.vEgoCluster = float(max(ve, 0.0)); cs.aEgo = float(ae)
            cs.standstill = ve < 0.1; cs.gasPressed = False; cs.brakePressed = False
        elif not SYN:
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
            if rd is not None and mode != 'A':
                a_lead = float(ld.aLeadK)   # radard's own value (radar accel when the lead is a radar track)
            cs = rec_cs.as_builder(); cs.vEgo = cs.vEgoRaw = cs.vEgoCluster = float(max(ve, 0.0)); cs.aEgo = float(ae)
            cs.standstill = ve < 0.1; cs.gasPressed = False; cs.brakePressed = False
        if gap:
            cs.cruiseState.gapAdjust = int(gap)
            sd = sm.data['selfdriveState']; sd = sd.as_builder() if hasattr(sd, 'as_builder') else sd
            sd.personality = GAP_TO_PERSONALITY_INT[int(gap) - 1]; sm.data['selfdriveState'] = sd
        if nolead:
            rs = rs_rec.as_builder() if hasattr(rs_rec, 'as_builder') else rs_rec.copy()
            rs.leadTwo.present = False
        elif not SYN:
            rs = rs_rec.as_builder() if hasattr(rs_rec, 'as_builder') else rs_rec.copy(); L = rs.leadOne
            # vLead and vLeadK are both absolute lead speeds, so neither depends on our simulated car:
            # keep radard's (or the recording's) filtered speed instead of copying the raw one over it
            L.dRel = float(x_lead - xe); L.vRel = float(v_lead - ve); L.vLead = float(v_lead); L.vLeadK = float(ld.vLeadK); L.aLeadK = float(a_lead)
            L2 = rs.leadTwo; ld2 = rs_rec.leadTwo
            if os.environ.get('SIMLEAD2') and ld2.present:
                # same treatment as leadOne: the recorded car's absolute position, our simulated one
                L2.dRel = float(x_rec + ld2.dRel - xe); L2.vRel = float(ld2.vLead - ve)
            else:
                L2.present = False
        state['d'] = float(x_lead - xe); state['ve'] = float(ve)
        sm.data['carState'] = cs; sm.data['radarState'] = rs
        model_rec = None
        if NOLEAD and not os.environ.get('SIMNOMODELSHIFT') and xe is not None and abs(x_rec - xe) > 0.05:
            # the model's path is measured from the recorded car; from ours a fixed point ahead (a stop
            # line, a stopped car) is further by however far we have fallen behind the recording
            model_rec = sm.data['modelV2']; mb = model_rec.as_builder(); dxm = float(x_rec - xe)
            mb.position.x = [float(xx) + dxm for xx in model_rec.position.x]
            sm.data['modelV2'] = mb
        sm.frame += 1; sm.updated = dict.fromkeys(sm.data, True)
        if planner is None:
            planner = CarrotLongitudinalPlanner(CP)
            for P in (planner.carrot.params, planner.planner.params):
                P.put = lambda *a, **k: None       # never write the device's real params from a sim
            if (ovr or {}).get('VTG'):
                sys.path.insert(0, '/data/tmp'); from vtg import vtg_time_gap
                cr = planner.carrot
                def vtg_dyn(t_follow, lead, desired_distance, prev_a, _cr=cr):
                    _cr.jerk_factor_apply = _cr.jerk_factor
                    if not lead.present:
                        return _cr.apply_t_follow(t_follow, 0.0)
                    seq = lambda v, t: _cr.stop_distance + t * v + v * v * (1 / (2 * _cr.comfort_brake) - 1 / (2 * _cr.comfort_brake_2))
                    tg, _ = vtg_time_gap(t_follow, state['ve'], float(lead.vLead), float(lead.dRel), seq)
                    return _cr.apply_t_follow(tg, 0.0)
                cr.dynamic_t_follow = vtg_dyn
            if ovr:
              for P in (planner.carrot.params, planner.planner.params):
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
        if model_rec is not None:
            sm.data['modelV2'] = model_rec
        cmd = float(planner.planner.output_a_target)
        if os.environ.get('SIMTRACE') and rt >= T0:
            rp = planner.planner.research
            L2s = sm.data['radarState'].leadTwo
            TRACE.append((rt, ve*3.6, float(x_lead-xe), v_lead*3.6, int(planner.carrot.xState.value), bool(ld.present),
                          round(rp.weight,2), round(rp.last_a,2), round(cmd,2), round(float(planner.carrot.v_cruise)*3.6,1),
                          float(L2s.dRel) if L2s.present else -1.0, float(L2s.vLead)*3.6 if L2s.present else -1.0,
                          int(L2s.radar), int(L2s.radarTrackId), int(ld.radarTrackId), float(L2s.yRel)))
        if rt >= T0 and LOC:
            # controlsd's LongControl between the plan and the car (stopping hold and release ramp,
            # accel limits), at its own 100 Hz -- the fitted actuator lag is from its output to aEgo
            if loc is None:
                from openpilot.selfdrive.controls.lib.longcontrol import LongControl
                from opendbc.car.interfaces import CarInterfaceBase
                loc = LongControl(CP); loc_hist = collections.deque([0.0] * (int(round(DELAY / 0.01)) + 1), maxlen=int(round(DELAY / 0.01)) + 1)
            should_stop = bool(planner.planner.output_should_stop)
            for _ in range(5):
                lcs = types.SimpleNamespace(vEgo=float(ve), aEgo=float(ae), brakePressed=False,
                                            cruiseState=types.SimpleNamespace(standstill=False))
                lim = CarInterfaceBase.get_pid_accel_limits(CP, ve, 0.0)
                a_out = loc.update(True, lcs, cmd, should_stop, lim, -0.5, float(x_lead - xe), (0.0, 0.0, 1.0))
                loc_hist.append(float(a_out)); a_in = loc_hist[0]
                ae += (a_in - ae) * 0.01 / TAU; ve = max(ve + ae * 0.01, 0.0); xe += ve * 0.01
        elif rt >= T0:
            cmd_hist.append(cmd); a_in = cmd_hist[0]
            ae += (a_in - ae) * DT / TAU; ve = max(ve + ae * DT, 0.0); xe += ve * DT
        if rt >= T0:
            out.append((rt, ve * 3.6, float(x_lead - xe), v_lead * 3.6, a_lead, a_true if a_true is not None else float('nan'), cmd,
                        rec_cs.vEgo * 3.6, ld.dRel, sm.data['carControl'].actuators.accel, float(planner.planner.mpc.t_follow),
                        float(getattr(planner.planner.mpc, 'desired_distance', float('nan'))), float(planner.planner.research.weight), _sstar(planner, ve, v_lead),
                        float(ae), float(a_in)))
    return np.array(out)

def stats(c, lbl):
    sw = 0; st = 0
    for x in c:
        if x > 0.5 and st <= 0: st = 1; sw += 1
        elif x < -0.5 and st >= 0: st = -1; sw += 1
    return f"{lbl:>26}  가속max {c.max():+.2f}  제동min {c.min():+.2f}  ±0.5교차 {sw:>2}회  |Δ|>1.5 {np.sum(np.abs(np.diff(c))>0.075):>3}"
TRACE = []
VARIANTS = json.loads(os.environ.get('VARIANTS', 'null'))
DUMP = {}
if VARIANTS:
    rec = None
    for name, ovr in VARIANTS:
        r = run('A' if ovr.get('KALMAN') else 'B', ovr=ovr)
        if rec is None:
            rec = r
            print(stats(r[:, 9], "녹화 실제 명령"))
        ve_, vl_ = r[:, 1], r[:, 3]
        amp = np.std(ve_) / max(np.std(vl_), 1e-3)                       # OpenACC-style speed amplification
        p2p = (ve_.max() - ve_.min()) / max(vl_.max() - vl_.min(), 1e-3)
        tg = r[:, 2] / np.maximum(ve_ / 3.6, 0.5)
        jerk = np.sqrt(np.mean((np.diff(r[:, 6]) / 0.05) ** 2))
        # t, v_ego km/h, gap, v_lead km/h, a_lead, plan accel, MPC desired gap, research weight, s*, actual accel, actuator command
        DUMP[name] = [[round(float(x), 3) for x in (row[0], row[1], row[2], row[3], row[4], row[6], row[11], row[12], row[13], row[14], row[15])] for row in r]
        if SYN:
            close = np.maximum(r[:, 1] - r[:, 3], 0.01) / 3.6
            ttc = np.where(r[:, 1] > r[:, 3] + 0.5, r[:, 2] / close, 99.0)
            print(f"{name:>26}  {'충돌!!' if r[:,2].min() < 0.4 else '충돌없음'}  최소TTC {ttc.min():.1f}s  최종거리 {r[-1,2]:.1f}m  최종속도 {r[-1,1]:.1f}km/h")
        print(stats(r[:, 6], name) + f"  증폭 {amp:.2f} p2p {p2p:.2f}  최소거리 {r[:,2].min():.1f}m 최소시간간격 {tg[ve_>7].min() if (ve_>7).any() else float('nan'):.2f}s  평균거리 {np.mean(r[:,2]):.1f}m  저크RMS {jerk:.2f}  목표시간간격 평균 {r[:,10].mean():.2f} 최소 {r[:,10].min():.2f} 최대 {r[:,10].max():.2f}s")
    if os.environ.get('SIMDUMP'):
        json.dump(DUMP, open(os.environ['SIMDUMP'], 'w'))
    if os.environ.get('SIMTRACE'):
        prev=None
        for row in TRACE:
            if prev is None or abs(row[8]-prev[8])>0.12 or row[6]!=prev[6] and (row[6] in (0.0,1.0) or prev[6] in (0.0,1.0)):
                print("  t+%5.1f v%5.1f d%5.1f vl%5.1f xs%d lead%d w%.2f aRes%+.2f out%+.2f vcru%.0f | L2 d%5.1f vl%5.1f radar%d id%d (L1 id%d) y%+.1f" % row)
            prev=row
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
