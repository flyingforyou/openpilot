"""Actuator response: fit a_ego = first-order lag (tau) of the command delayed by d, over engaged driving."""
import glob, os, sys
from multiprocessing import Pool
import numpy as np, zstandard
def files():
    seen={}
    for root in ("/home/compiler/openpilot/op-logs","/tmp/claude-1000/-home-compiler-openpilot/619fd59f-869b-4342-9985-8349606bd5fc/scratchpad/logs"):
        for p in glob.glob(f"{root}/*/rlog.zst"): seen.setdefault(os.path.basename(os.path.dirname(p)),p)
    names=sorted(seen)
    return [seen[n] for n in names if n.split("--")[0] >= "00000116"]   # recent drives: current code & params
def one(path):
    from openpilot.cereal import log
    try: buf=zstandard.ZstdDecompressor().stream_reader(open(path,"rb"),read_across_frames=True).read()
    except Exception: return None
    cur=dict(a=0.,v=0.,ok=False,gas=False,br=False,cmd=0.); R=[]
    try:
        for m in log.Event.read_multiple_bytes(buf):
            w=m.which()
            if w=="carState":
                cs=m.carState; cur.update(a=cs.aEgo,v=cs.vEgo,gas=cs.gasPressed,br=cs.brakePressed)
            elif w=="carControl":
                cc=m.carControl; cur["cmd"]=float(cc.actuators.accel); cur["ok"]=bool(cc.longActive)
                R.append((cur["cmd"],cur["a"],cur["ok"] and not cur["gas"] and not cur["br"] and cur["v"]>2.0))
    except Exception: pass
    return np.array(R,dtype=float) if len(R)>500 else None
def sim(cmd,a0,d,tau,dt=0.01):
    k=int(round(d/dt)); u=np.r_[np.full(k,cmd[0]),cmd[:len(cmd)-k]] if k else cmd
    y=np.empty_like(u); acc=a0; al=dt/(tau+dt)
    for i,x in enumerate(u): acc+=al*(x-acc); y[i]=acc
    return y
if __name__=="__main__":
    with Pool(16) as p: res=[r for r in p.map(one,files(),chunksize=2) if r is not None]
    # contiguous engaged chunks of 3-15 s
    chunks=[]
    for A in res:
        ok=A[:,2]>0; i=0
        while i<len(A):
            if not ok[i]: i+=1; continue
            j=i
            while j<len(A) and ok[j] and j-i<1500: j+=1
            if j-i>300 and np.std(A[i:j,0])>0.2: chunks.append(A[i:j,:2])
            i=j
    print(f"인게이지 구간 {len(chunks)}개 ({sum(len(c) for c in chunks)/100/60:.0f}분)\n")
    best=None; grid=[]
    for d in (0.0,0.05,0.1,0.15,0.2,0.25,0.3,0.4):
        for tau in (0.1,0.2,0.3,0.4,0.5,0.6,0.8):
            e=[];n=0
            for c in chunks[::3]:
                y=sim(c[:,0],c[0,1],d,tau); e.append(np.sum((y-c[:,1])**2)); n+=len(c)
            rms=np.sqrt(sum(e)/n); grid.append((rms,d,tau))
    grid.sort()
    print("상위 피팅 (RMS 오차 m/s², 지연 d, 1차시정수 τ)")
    for rms,d,tau in grid[:6]: print(f"  RMS {rms:.3f}  d={d:.2f}s  τ={tau:.2f}s   → 등가 지연 d+τ = {d+tau:.2f}s")
    rms0=[g for g in grid if g[1]==0.15 and g[2]==0.3]
    print(f"\n(현재 시뮬 가정 d=0.15/τ≈0.35 근처: RMS {rms0[0][0]:.3f})")
