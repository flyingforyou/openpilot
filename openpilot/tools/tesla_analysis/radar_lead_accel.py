"""Does the Bosch radar's own LongAccel see a lead braking sooner than the Kalman aLeadK?

radard never reads aRel -- it differentiates vLead through a low-gain Kalman filter instead. The
point messages (bus 1, 0x310 + 3i) carry LongAccel at 0.03125 m/s^2. Decode the point matching
the radar lead and time it against the same clean onsets used for aLeadK.
"""
import glob, os, sys
from multiprocessing import Pool
import numpy as np, zstandard

ADDRS = {0x310 + 3*i for i in range(32)}

def bits(d, start, ln):
    raw = int.from_bytes(bytes(d).ljust(8, b'\0')[:8], 'little')
    return (raw >> start) & ((1 << ln) - 1)

def seg_files():
    seen={}
    for root in ("/home/compiler/openpilot/op-logs",
                 "/tmp/claude-1000/-home-compiler-openpilot/619fd59f-869b-4342-9985-8349606bd5fc/scratchpad/logs"):
        for p in glob.glob(f"{root}/*/rlog.zst"):
            seen.setdefault(os.path.basename(os.path.dirname(p)), p)
    return sorted(seen.items())

def one(item):
    name, path = item
    from openpilot.cereal import log
    try:
        buf = zstandard.ZstdDecompressor().stream_reader(open(path,"rb"), read_across_frames=True).read()
    except Exception: return None
    pts = {}; aego = 0.0; R = []
    try:
        for m in log.Event.read_multiple_bytes(buf):
            w = m.which()
            if w == "can":
                for c in m.can:
                    if c.src == 1 and c.address in ADDRS:
                        d = c.dat
                        if not bits(d, 62, 1): pts.pop(c.address, None); continue
                        pts[c.address] = (bits(d,0,12)*0.0625, bits(d,12,12)*0.0625-128,
                                          bits(d,24,11)*0.125-128, bits(d,40,10)*0.03125-16)
            elif w == "carState": aego = m.carState.aEgo
            elif w == "radarState":
                l = m.radarState.leadOne
                arel = np.nan
                if l.present and l.radar:
                    best = None
                    for (dx, vx, dy, ax) in pts.values():
                        e = abs(dx - l.dRel) + abs(vx - l.vRel)
                        if abs(dx - l.dRel) < 2.0 and abs(vx - l.vRel) < 1.5 and (best is None or e < best[0]):
                            best = (e, ax)
                    if best: arel = best[1]
                R.append((m.logMonoTime/1e9, l.present and l.radar, l.dRel, l.vLead, l.aLeadK, arel, aego))
    except Exception: pass
    if len(R) < 200: return None
    A = np.array(R, dtype=float); T, OK, DR, VL, AK, AR, AE = A.T
    ok = (OK > 0) & (DR < 60) & (DR > 2) & np.isfinite(AR)
    ok &= ~np.r_[False, np.abs(np.diff(DR)) > 3]
    h = 4; ta = np.full(len(VL), np.nan); ta[h:-h] = (VL[2*h:] - VL[:-2*h]) / (T[2*h:] - T[:-2*h])
    ABS = AE + AR          # radar LongAccel read as relative -> absolute lead accel
    ev = []; i = 25
    while i < len(T) - 35:
        if (ta[i] < -0.5 and ta[i-1] >= -0.5 and ok[i-20:i+30].all()
            and np.all(ta[i-20:i-2] > -0.3) and np.nanmin(ta[i:i+30]) < -1.0):
            def lat(x):
                if x[i-20] < -0.3: return None
                k = np.where(x[i-20:i+35] < -0.5)[0]
                return float(T[i-20+k[0]] - T[i]) if len(k) else None
            ev.append((lat(AK), lat(ABS), lat(AR)))
            i += 60
        else: i += 1
    st = ok & (np.abs(ta) < 0.2)
    corr = (ta[ok & np.isfinite(ta)], ABS[ok & np.isfinite(ta)], AR[ok & np.isfinite(ta)])
    return ev, int(st.sum()), int((AK[st] < -0.5).sum()), int((ABS[st] < -0.5).sum()), int(ok.sum()), corr

if __name__ == "__main__":
    files = seg_files()
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 16) as p:
        res = [r for r in p.map(one, files, chunksize=4) if r]
    ev = [e for r in res for e in r[0]]
    nst = sum(r[1] for r in res); fak = sum(r[2] for r in res); far_ = sum(r[3] for r in res)
    matched = sum(r[4] for r in res)
    ta = np.concatenate([r[5][0] for r in res]); ab = np.concatenate([r[5][1] for r in res]); ar = np.concatenate([r[5][2] for r in res])
    print(f"세그 {len(files)}, 레이더 LongAccel 이 앞차에 매칭된 프레임 {matched} ({matched/20/3600:.1f}시간)")
    print(f"실제 앞차 가속과의 상관:  aEgo+LongAccel {np.corrcoef(ta,ab)[0,1]:+.3f}   LongAccel 단독 {np.corrcoef(ta,ar)[0,1]:+.3f}")
    print(f"  (실제 대비 RMS 오차: aEgo+LongAccel {np.sqrt(np.mean((ab-ta)**2)):.3f}  LongAccel 단독 {np.sqrt(np.mean((ar-ta)**2)):.3f})\n")
    print(f"깨끗한 감속 개시 {len(ev)}건 (0 = 실제 앞차 감속 -0.5 통과)")
    for idx, lbl in ((0, "aLeadK (칼만)"), (1, "레이더 LongAccel(+aEgo)"), (2, "LongAccel 단독")):
        x = np.array([e[idx] for e in ev if e[idx] is not None])
        if len(x): print(f"  {lbl:>24}: 잡은 비율 {100*len(x)/len(ev):>3.0f}%  중앙 {np.median(x):+.2f}s  p25 {np.percentile(x,25):+.2f}  p75 {np.percentile(x,75):+.2f}")
    pr = np.array([e[1]-e[0] for e in ev if e[0] is not None and e[1] is not None])
    if len(pr): print(f"\n  둘 다 잡은 {len(pr)}건: LongAccel - aLeadK 중앙 {np.median(pr):+.2f}s, LongAccel 이 먼저 {100*np.mean(pr<0):.0f}%")
    print(f"\n앞차 일정속도 {nst}프레임 중 오경보(-0.5 미만): aLeadK {100*fak/max(nst,1):.2f}%   LongAccel(+aEgo) {100*far_/max(nst,1):.2f}%")
