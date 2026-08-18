#!/usr/bin/env python3
"""A broad look over a recorded drive for anything that went wrong.

Not a replacement for the focused scripts next to it -- this is the sweep you run first to find
out which of them to reach for: crashes, model health, hard braking, lateral saturation, the
device's own state, and the CAN/panda side.

  ./drive_health.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys
from collections import Counter, defaultdict

import numpy as np

from openpilot.tools.lib.logreader import LogReader

HARD_BRAKE = -2.0       # m/s^2, the threshold this project has been using
MODEL_BUDGET = 0.05     # 20 Hz


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)

  crashes, errors = [], []
  model_id = None
  exec_t, drops, big = [], [], 0
  a_ego, v_ego, engaged_s = [], [], 0
  brake_events, in_event = [], None
  lat_sat = long_sat = lat_frames = 0
  events = Counter()
  temps, cpu, mem, free_gb = [], [], [], []
  panda_faults = Counter()
  can_err = defaultdict(int)
  desired_neg = desired_tot = 0
  lead_present = lead_tot = 0
  t0 = None

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      if w == 'logMessage':
        s = str(msg.logMessage)
        if model_id is None and 'modeld starting on' in s:
          model_id = s.split('modeld starting on')[-1].split('"')[0].strip()
        if 'Falling back to stock' in s or 'failed to load' in s:
          errors.append(('model fallback', s[:120]))
      elif w == 'errorLogMessage':
        s = str(msg.errorLogMessage)
        name = s.split('"daemon": "')[-1].split('"')[0] if '"daemon"' in s else '?'
        crashes.append((round(t - t0, 1), name, s[:160]))
      elif w == 'modelV2':
        md = msg.modelV2
        exec_t.append(float(md.modelExecutionTime))
        drops.append(float(md.frameDropPerc))
        if float(md.modelExecutionTime) > MODEL_BUDGET:
          big += 1
      elif w == 'carState':
        c = msg.carState
        a_ego.append(float(c.aEgo))
        v_ego.append(float(c.vEgo))
      elif w == 'carControl':
        cc = msg.carControl
        if cc.latActive:
          lat_frames += 1
      elif w == 'controlsState':
        st = msg.controlsState
        if st.lateralControlState.which() == 'curvatureState':
          if st.lateralControlState.curvatureState.saturated:
            lat_sat += 1
        engaged_s += 0.01
      elif w == 'longitudinalPlan':
        lp = msg.longitudinalPlan
        desired_tot += 1
        if float(lp.desiredDistance) < 0:
          desired_neg += 1
        a = float(lp.aTarget)
        if a <= HARD_BRAKE and in_event is None:
          in_event = [t - t0, a, float(lp.tFollow)]
        elif in_event is not None:
          in_event[1] = min(in_event[1], a)
          if a > HARD_BRAKE + 0.3:
            brake_events.append(tuple(in_event))
            in_event = None
      elif w == 'radarState':
        lead_tot += 1
        if msg.radarState.leadOne.present:
          lead_present += 1
      elif w == 'onroadEvents':
        for e in msg.onroadEvents:
          events[str(e.name)] += 1
      elif w == 'deviceState':
        d = msg.deviceState
        temps.append(max(list(d.cpuTempC) or [0]))
        cpu.append(float(np.mean(list(d.cpuUsagePercent) or [0])))
        mem.append(float(d.memoryUsagePercent))
        free_gb.append(float(d.freeSpacePercent))
      elif w == 'pandaStates':
        for ps in msg.pandaStates:
          if str(ps.faultStatus) != 'none':
            panda_faults[str(ps.faultStatus)] += 1
          for f in ps.faults:
            panda_faults[str(f)] += 1
          # these are running totals on the panda, so the last value is the count for the
          # drive -- summing the samples would just multiply by the publish rate
          for name in ('safetyRxInvalid', 'safetyTxBlocked', 'rxBufferOverflow',
                       'txBufferOverflow', 'spiErrorCount', 'heartbeatLost'):
            can_err[name] = max(can_err[name], int(getattr(ps, name)))

  eng_min = engaged_s / 60
  print(f"segments {len(paths)}   controls frames {engaged_s*100:.0f}   ~{eng_min:.1f} min of stack time")

  print("\n-- crashes and load failures --")
  if not crashes and not errors:
    print("  none")
  for tt, name, s in crashes[:8]:
    print(f"  t+{tt/60:.1f}min  {name}: {s[:110]}")
  for k, s in errors[:5]:
    print(f"  {k}: {s}")

  print("\n-- driving model --")
  print(f"  loaded            {model_id or '(no log line captured)'}")
  if exec_t:
    e = np.array(exec_t)
    print(f"  execution time    median {np.median(e)*1e3:.1f} ms  p99 {np.percentile(e,99)*1e3:.1f} ms  "
          f"max {e.max()*1e3:.1f} ms   over {MODEL_BUDGET*1e3:.0f}ms budget: {big} frames ({100*big/len(e):.2f}%)")
    d = np.array(drops)
    print(f"  frame drop        median {np.median(d):.2f}%  max {d.max():.2f}%")

  print("\n-- longitudinal --")
  if brake_events:
    per10 = len(brake_events) / max(eng_min, 1e-9) * 10
    worst = min(brake_events, key=lambda x: x[1])
    print(f"  aTarget <= {HARD_BRAKE}    {len(brake_events)} events  ({per10:.1f} per 10 min)")
    print(f"  worst             {worst[1]:.2f} m/s^2 at t+{worst[0]/60:.1f} min (tFollow {worst[2]:.2f})")
    mags = np.array([b[1] for b in brake_events])
    print(f"  event depth       median {np.median(mags):.2f}  p10 {np.percentile(mags,10):.2f} m/s^2")
  else:
    print(f"  aTarget <= {HARD_BRAKE}    none")
  if a_ego:
    a = np.array(a_ego)
    print(f"  measured aEgo     p1 {np.percentile(a,1):.2f}  p99 {np.percentile(a,99):.2f} m/s^2")
  if desired_tot:
    print(f"  desiredDistance<0 {100*desired_neg/desired_tot:.0f}% of plan frames "
          f"(known: faked v_lead when no lead is tracked)")
  if lead_tot:
    print(f"  lead present      {100*lead_present/lead_tot:.0f}% of radar frames")

  print("\n-- lateral --")
  print(f"  latActive         {lat_frames} frames")
  print(f"  saturated         {lat_sat} frames ({100*lat_sat/max(lat_frames,1):.1f}% of active)")

  print("\n-- device --")
  if temps:
    print(f"  cpu temp          median {np.median(temps):.0f}C  max {max(temps):.0f}C")
    print(f"  cpu usage         median {np.median(cpu):.0f}%  max {max(cpu):.0f}%")
    print(f"  memory            median {np.median(mem):.0f}%  max {max(mem):.0f}%")
    print(f"  free space        {np.median(free_gb):.0f}%")
  print(f"  panda faults      {dict(panda_faults) or 'none'}")
  nz = {k: v for k, v in can_err.items() if v}
  print(f"  panda counters    {nz or 'all zero'}  (running totals at end of drive)")

  print("\n-- onroad events --")
  for k, n in events.most_common(14):
    print(f"  {k:32} {n}")


if __name__ == '__main__':
  main(sys.argv[1:])
