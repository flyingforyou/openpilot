#!/usr/bin/env python3
"""One drive, every feature that still waits for the road: pass, fail, or "did not come up".

  PYTHONPATH=.:opendbc_repo .venv/bin/python openpilot/tools/tesla_analysis/drive_report.py <route>...
  routes are prefixes under OP_LOG_ROOT (00000271, or 00000271--abcdef); several are read as one drive.
  options (env): OUT=<file.json>   BASE=<baseline file>   JOBS=<n>
  drive_report.py --baseline <route>...   measures those routes and writes the baseline file

Why: by 11/13 more than ten changes had been deployed and never checked on the road (stopped cars from
the radar, the on-ramp cap, the exit slow-down, ApproachCoast, LeadAheadBrake, the relay handover, the
lane aim ...), each with its own "after the next drive look at ..." note. This reads a drive once and
answers all of them.

How a line is judged
  measure()  turns the drive into plain numbers. The same code measured the reference drives
             (00000258-00000266, the last ones before the 11/11 changes) into drive_report_baseline.json.
  checks()   compares the two. A check names the scene it needs; without enough of it the line reads
             "장면 없음" instead of a verdict from nothing.
  합격/불합격 are for things the code decides (a fault, a gap on the bus, where the car sits in a bend).
  One drive is a small sample -- a commute has one or two minutes of bends, and the reference drives
  themselves read 0.09 to 0.19 on lateral swing -- so a comparison has two limits: past the pooled
  reference by its tolerance is 주의, and only worse than every single reference drive is 불합격.
  Give it several drives at once to tighten that. Numbers traffic moves as much as the code does
  (brake applications, hard brakes; 10/26-11/11 read 40 % busier than 10/01-10/25 with identical
  following code) never go past 주의: commit_quality.py is the tool that separates code from traffic.

Sign conventions: lane y and curvature + are to the right.
"""
import glob
import json
import os
import pickle
import sys
from dataclasses import dataclass, field
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.join(ROOT, 'op-logs'))
CACHE = os.path.expanduser('~/.cache/drive_report')
BASELINE = os.environ.get('BASE', os.path.join(HERE, 'drive_report_baseline.json'))
EXTRACT_VERSION = 5

DT = 0.05   # rows are modelV2 frames
COLS = ('t seg sec v a_ego angle pressed blink_l blink_r gas brake enabled lat_active long_active k k_des dc '
        'lane_l lane_r prob_l prob_r lc aim lead d_rel v_lead a_lead_k a_lead lead_radar lead_prob lead_id y_rel '
        'a_target has_lead fcw cmd stopped_cap nav_valid ramp fleet posted branch steer_fault acc_fault '
        'exec_ms drop_pct standstill').split()
C = {name: i for i, name in enumerate(COLS)}

# onroad events that only describe the driver or normal operation
QUIET_EVENTS = {'gasPressedOverride', 'steerOverride', 'pedalPressed', 'steerDisengage', 'buttonCancel', 'buttonEnable',
                'pcmDisable', 'pcmEnable', 'laneChange', 'preLaneChangeLeft', 'preLaneChangeRight', 'laneChangeBlocked',
                'wrongGear', 'reverseGear', 'seatbeltNotLatched', 'doorOpen', 'manualRestart', 'parkBrake', 'brakeHold',
                'preEnableStandstill', 'resumeRequired', 'belowSteerSpeed', 'startup', 'startupMaster', 'startupNoControl',
                'startupNoCar', 'startupNoFw', 'cruiseDisabled', 'wrongCarMode', 'trafficSignGreen', 'trafficSignChanged',
                'trafficStopping', 'speedTooLow', 'driverDistracted1', 'promptDriverDistracted', 'preDriverDistracted',
                'belowEngageSpeed', 'stockLkas', 'personalityChanged', 'audioPrompt', 'audioRefuse', 'calibrationIncomplete'}
PARAMS = ('LaneCentering', 'LaneCenteringAim', 'LaneCenteringCurveHold', 'LaneChangeBoost', 'RadarStoppedAhead',
          'LeadAheadBrake', 'ApproachCoast', 'LeadBrakeFollow', 'LeadAccelSlope', 'TeslaMapAutoSpeed', 'TeslaMapExitDecel',
          'StoppedLeadEarly', 'ComfortBrake', 'LongitudinalPersonality', 'RadarLeadHoldCm')
STEER_ADDR = 0x488


# ---------------------------------------------------------------- reading

def seg_number(path: str) -> int:
  return int(path.rstrip('/').split('/')[-2].split('--')[-1])


def segments(route: str) -> list[str]:
  return sorted(glob.glob(f'{LOG_ROOT}/{route}*--*/rlog.zst'), key=lambda p: (p.split('/')[-2].rsplit('--', 1)[0], seg_number(p)))


def extract(path: str) -> dict:
  """One segment: rows at the model rate, and the few things that need every message."""
  name = path.split('/')[-2]
  key = os.path.join(CACHE, f'{name}-{os.path.getsize(path)}-{EXTRACT_VERSION}.pkl')
  if os.path.isfile(key):
    return pickle.load(open(key, 'rb'))
  from openpilot.tools.lib.logreader import LogReader
  s = dict.fromkeys(COLS, 0.0)
  rows, events, steer, procs, safety = [], [], [], {}, []
  meta = {}
  t_first = None
  have = set()
  try:
    for m in LogReader(path):
      w = m.which()
      t = m.logMonoTime * 1e-9
      if t_first is None:
        t_first = t
      if w == 'carState':
        c = m.carState
        s.update(v=c.vEgo, a_ego=c.aEgo, angle=c.steeringAngleDeg, pressed=c.steeringPressed, blink_l=c.leftBlinker,
                 blink_r=c.rightBlinker, gas=c.gasPressed, brake=c.brakePressed, steer_fault=c.steerFaultTemporary or c.steerFaultPermanent,
                 acc_fault=c.accFaulted, standstill=c.standstill)
        n = c.navMap
        s.update(nav_valid=n.valid, ramp=n.rampType, fleet=n.fleetSplineSpeed, posted=n.baseSpeedLimit, branch=n.nextBranchDist)
        if c.stockFcw:
          events.append((t, 'stockFcw', bool(s['enabled'])))
        have.add(w)
      elif w == 'carControl':
        c = m.carControl
        s.update(lat_active=c.latActive, long_active=c.longActive, cmd=c.actuators.accel)
      elif w == 'controlsState':
        s.update(k=m.controlsState.curvature, k_des=m.controlsState.desiredCurvature)
      elif w == 'selfdriveState':
        s['enabled'] = m.selfdriveState.enabled
      elif w == 'radarState':
        l = m.radarState.leadOne
        s.update(lead=l.present, d_rel=l.dRel, v_lead=l.vLead, a_lead_k=l.aLeadK, a_lead=l.aLead, lead_radar=l.radar,
                 lead_prob=l.modelProb, lead_id=l.radarTrackId, y_rel=l.yRel)
      elif w == 'longitudinalPlan':
        p = m.longitudinalPlan
        s.update(a_target=p.aTarget, has_lead=p.hasLead, fcw=p.fcw, stopped_cap=p.stoppedLeadCap)
      elif w == 'onroadEvents':
        events += [(t, str(e.name), bool(s['enabled'])) for e in m.onroadEvents if str(e.name) not in QUIET_EVENTS]
      elif w == 'can':
        for f in m.can:
          if f.address == STEER_ADDR and f.src in (0, 128):
            steer.append((t, f.src, f.dat[2] & 0xF))
      elif w == 'pandaStates' and len(m.pandaStates):
        safety.append((t, str(m.pandaStates[0].safetyModel)))
      elif w == 'managerState':
        for p in m.managerState.processes:
          if p.shouldBeRunning and not p.running:
            procs[p.name] = procs.get(p.name, 0) + 1
      elif w == 'initData':
        meta['commit'] = m.initData.gitCommit[:9]
        meta['params'] = {e.key: bytes(e.value).decode(errors='ignore').strip() for e in m.initData.params.entries if e.key in PARAMS}
      elif w == 'modelV2' and 'carState' in have:
        md = m.modelV2
        try:
          s.update(dc=md.action.desiredCurvature, lane_l=md.laneLines[1].y[0], lane_r=md.laneLines[2].y[0],
                   prob_l=md.laneLineProbs[1], prob_r=md.laneLineProbs[2], lc=str(md.meta.laneChangeState) != 'off',
                   aim=md.meta.laneAimShift, exec_ms=md.modelExecutionTime * 1e3, drop_pct=md.frameDropPerc)
        except Exception:
          continue
        s.update(t=t, seg=seg_number(path), sec=0.0)
        rows.append([float(s[c]) for c in COLS])
  except Exception as e:     # a truncated last segment is normal
    meta['error'] = repr(e)
  out = dict(name=name, t_first=t_first, rows=np.array(rows, dtype=float).reshape(-1, len(COLS)), events=events, steer=steer, procs=procs,
             safety=safety, meta=meta)
  os.makedirs(CACHE, exist_ok=True)
  pickle.dump(out, open(key, 'wb'))
  return out


@dataclass
class Drive:
  a: np.ndarray                      # rows, time made continuous across routes
  events: list
  starts: list                       # per route: (name, steer frames, safety model changes) of its first segment
  procs: dict
  commits: list
  params: dict
  names: list = field(default_factory=list)
  route: np.ndarray = None           # per row: index into names

  def col(self, name):
    return self.a[:, C[name]]


def load(routes: list[str]) -> Drive:
  paths = [p for r in routes for p in segments(r)]
  if not paths:
    raise SystemExit(f'no segments for {routes} under {LOG_ROOT}')
  with Pool(int(os.environ.get('JOBS', os.cpu_count() or 1))) as pool:
    segs = pool.map(extract, paths, chunksize=1)
  rows, events, starts, procs, commits, params, names, route_of = [], [], [], {}, [], {}, [], []
  shift, last_route = 0.0, None
  for path, s in zip(paths, segs):
    route = s['name'].rsplit('--', 1)[0]
    if route != last_route:          # each route has its own clock: keep them apart
      shift += 1e5
      last_route = route
      names.append(route)
      starts.append((route, s['steer'], s['safety']))
      t0 = (s['t_first'] or 0.0) - 60.0 * seg_number(path)     # the route's first message
    r = s['rows'].copy()
    if len(r):
      r[:, C['seg']], r[:, C['sec']] = np.divmod(r[:, C['t']] - t0, 60.0)
      r[:, C['t']] += shift
      rows.append(r)
      route_of.append(np.full(len(r), len(names) - 1))
    events += [(t + shift, n, seg_number(path), eng) for t, n, eng in s['events']]
    for k, v in s['procs'].items():
      procs[k] = procs.get(k, 0) + v
    if s['meta'].get('commit') and s['meta']['commit'] not in commits:
      commits.append(s['meta']['commit'])
    params.update(s['meta'].get('params', {}))
  return Drive(np.concatenate(rows), events, starts, procs, commits, params, names, np.concatenate(route_of))


# ---------------------------------------------------------------- measuring

def runs(mask: np.ndarray, t: np.ndarray, min_len: int = 1) -> list[tuple[int, int]]:
  """[start, end) index ranges where mask holds and time does not jump."""
  ok = mask.astype(bool).copy()
  brk = np.concatenate([[True], np.diff(t) > 1.0])
  out, i, n = [], 0, len(ok)
  while i < n:
    if not ok[i]:
      i += 1
      continue
    j = i + 1
    while j < n and ok[j] and not brk[j]:
      j += 1
    if j - i >= min_len:
      out.append((i, j))
    i = j
  return out


def moving_average(x, n):
  k = np.ones(n) / n
  return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode='edge'), k, 'valid')


def band(x):
  """What is left between 0.25 and 2.5 Hz: the swing a passenger feels in a bend."""
  return moving_average(x, 8) - moving_average(x, 80)


def rms(x):
  return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')


def where(d: Drive, i: int) -> str:
  route = f'{d.names[int(d.route[i])][:8]} ' if d.route is not None and len(d.names) > 1 else ''
  return f"{route}{int(d.a[i, C['seg']])}구간 {d.a[i, C['sec']]:.0f}초"


def measure(d: Drive) -> dict:
  """Numbers only, plus 'scenes': where each thing happened, for the report."""
  g = d.col
  t, v = g('t'), g('v')
  en = g('enabled') > 0
  m: dict = {'scenes': {}}
  m['minutes'] = len(t) * DT / 60
  m['engaged_min'] = float(en.sum() * DT / 60)

  # -- lateral: bends and straights with both lines seen, hands off the stalk
  # where the lane's middle is from the car (+ right); the model sees the lane moved by the aim
  centre = 0.5 * (g('lane_l') + g('lane_r')) + g('aim')
  lat_model = v ** 2 * g('dc')
  lat_car = v ** 2 * g('k')
  swing, jerk = band(lat_car), np.gradient(lat_car, DT)
  ok = (g('lat_active') > 0) & (v > 12) & (g('lc') == 0) & (g('blink_l') == 0) & (g('blink_r') == 0) & (g('prob_l') > .5) & (g('prob_r') > .5)
  bend, straight = ok & (np.abs(lat_model) > 0.8), ok & (np.abs(lat_model) < 0.3)
  m['bend_min'] = float(bend.sum() * DT / 60)
  m['bend_inside'] = float(np.mean(-centre[bend] * np.sign(lat_model[bend]))) if bend.any() else float('nan')
  m['bend_swing'], m['bend_jerk'] = rms(swing[bend]), rms(jerk[bend])
  m['straight_min'] = float(straight.sum() * DT / 60)
  m['straight_swing'], m['straight_jerk'] = rms(swing[straight]), rms(jerk[straight])
  m['straight_centre'] = float(np.mean(centre[straight])) if straight.any() else float('nan')
  act = g('lat_active') > 0
  m['aim_straight'] = float(np.mean(g('aim')[straight])) if straight.any() else float('nan')
  m['aim_bend'] = float(np.mean(g('aim')[bend] * np.sign(lat_model[bend]))) if bend.any() else float('nan')
  m['aim_used'] = float(np.mean(np.abs(g('aim')[act]) > 0.01)) if act.any() else 0.0
  m['aim_limit'] = float(np.mean(np.abs(g('aim')[act]) > 0.29)) if act.any() else 0.0
  m['lat_cmd_max'] = float(np.max(np.abs(v ** 2 * g('k_des'))[act])) if act.any() else 0.0
  m['steer_fault'] = int(len(runs((g('steer_fault') > 0) & en, t)))

  # -- lane changes
  lcs = runs((g('lc') > 0) & act & (v > 15), t, 10)
  m['lc_n'] = len(lcs)
  m['lc_seconds'] = float(np.median([(j - i) * DT for i, j in lcs])) if lcs else float('nan')
  m['lc_helped'] = float(np.mean([g('pressed')[i:j].any() for i, j in lcs])) if lcs else float('nan')

  # -- which sensor the lead is from: flips with the same lead still there
  lead = en & (g('lead') > 0)
  flips = jumps = 0
  for i, j in runs(lead, t, 20):
    f = np.where(np.diff(g('lead_radar')[i:j]) != 0)[0]
    flips += len(f)
    jumps += int(np.sum(np.abs(np.diff(g('d_rel')[i:j])[f]) > 5.0))
  m['lead_min'] = float(lead.sum() * DT / 60)
  m['flips_min'] = flips / max(m['lead_min'], 1e-6)
  m['flip_jumps_min'] = jumps / max(m['lead_min'], 1e-6)
  m['radar_share'] = float(np.mean(g('lead_radar')[lead])) if lead.any() else float('nan')

  # -- following comfort (lib/ride_quality.py on what was commanded)
  from openpilot.selfdrive.controls.lib import ride_quality as rq
  follow = en & (g('long_active') > 0) & (g('lead') > 0) & (v > 11) & (g('d_rel') > 0) & (g('d_rel') < 3 * np.maximum(v, 1))
  parts = [rq.components(g('cmd')[i:j], v[i:j], g('d_rel')[i:j], g('v_lead')[i:j], g('a_lead_k')[i:j], DT) for i, j in runs(follow, t, 200)]
  if parts:
    q = rq.vector(parts)
    m.update(follow_min=q['minutes'], brake_on=q['brake_on'], unprovoked=q['ubon'], accel_on=q['accel_on'], reversals=q['rev'],
             cmd_jerk=q['cmd_jerk'], headway=float(np.median((g('d_rel') / np.maximum(v, 1))[follow])))
    m['lead_ahead_share'] = float(np.mean((g('a_lead_k') < g('a_lead') - 0.3)[follow]))
  else:
    m.update(follow_min=0.0)
  lng = en & (g('long_active') > 0)
  a = g('a_ego')
  hard, armed = [], True                      # one count per application: below -2.0, re-armed above -1.0
  for i in np.where(lng)[0]:
    if armed and a[i] < -2.0:
      hard.append(i)
      armed = False
    elif a[i] > -1.0:
      armed = True
  m['hard_h'] = len(hard) / max(lng.sum() * DT / 3600, 1e-6)
  m['hard_worst'] = float(a[lng].min()) if lng.any() else 0.0
  m['scenes']['hard'] = [f'{where(d, i)} {v[i] * 3.6:.0f} km/h' for i in hard]
  fcw = [i for i, _ in runs(g('fcw') > 0, t)]
  m['fcw'] = len(fcw) + len({round(e[0] / 5) for e in d.events if e[1] in ('fcw', 'stockFcw', 'stockAeb')})
  m['scenes']['fcw'] = [where(d, i) for i in fcw]

  # -- accelerating into a slower lead, then braking (what ApproachCoast is for)
  cmd = g('cmd')
  ab, last = [], -1e9
  for i in np.where(lng & (g('lead') > 0) & (cmd > 0.8) & (g('v_lead') < v - 1.0) & (v > 8))[0]:
    if t[i] - last < 10:
      continue
    j = min(i + int(6 / DT), len(t))
    same = g('lead')[i:j].all() and np.abs(np.diff(g('d_rel')[i:j])).max() < 5.0      # not a car cutting in
    if t[j - 1] - t[i] < 7 and lng[i:j].all() and same and (cmd[i:j] < -1.0).any():
      ab.append(i)
      last = t[i]
  m['accel_brake_h'] = len(ab) / max(m['lead_min'] / 60, 1e-6)
  m['scenes']['accel_brake'] = [where(d, i) for i in ab]

  # -- pulling away behind a lead
  v8, a4 = [], []
  for i, j in runs(lng & (v < 0.3) & (g('lead') > 0) & (g('d_rel') < 12), t, int(2 / DT)):
    k = j + int(12 / DT)
    if k >= len(t) or t[k] - t[j] > 13:
      continue
    go = np.where(g('v_lead')[i:k] > 1.5)[0]
    if not len(go):
      continue
    s = i + go[0]
    e8 = s + int(8 / DT)
    if e8 >= len(t) or not lng[s:e8].all() or g('gas')[s:e8].any() or g('lead')[s:e8].min() == 0:
      continue
    v8.append((g('v_lead')[e8] - v[e8]) * 3.6)
    a4.append(float(np.mean(a[s:s + int(4 / DT)])))
    m['scenes'].setdefault('launch', []).append(f'{where(d, s)} 8초 뒤 {v[e8] * 3.6:.0f} km/h, 앞차보다 {v8[-1]:.0f} 느림')
  m['launch_n'] = len(v8)
  m['launch_v8'] = float(np.median(v8)) if v8 else float('nan')     # km/h slower than the lead 8 s after it moved
  m['launch_a4'] = float(np.median(a4)) if a4 else float('nan')

  # -- a stopped car ahead while we are still moving
  stopped, last = [], -1e9
  cand = lng & (g('lead') > 0) & (g('v_lead') < 1.0) & (v > 8) & (g('d_rel') > 15)
  for i, j in runs(cand, t, 4):
    if t[i] - last < 15:
      continue
    last = t[i]
    k = min(i + int(12 / DT), len(t))
    stopped.append(dict(i=i, room=g('d_rel')[i] / v[i], decel=float(a[i:k].min()), brake=bool(g('brake')[i:k].any()), kph=v[i] * 3.6))
  m['stopped_n'] = len(stopped)
  m['stopped_room'] = float(np.median([s['room'] for s in stopped])) if stopped else float('nan')
  m['stopped_decel'] = float(min(s['decel'] for s in stopped)) if stopped else float('nan')
  m['stopped_driver'] = sum(s['brake'] for s in stopped)
  m['scenes']['stopped'] = [f"{where(d, s['i'])} {s['kph']:.0f} km/h, {s['room']:.1f}초 앞에서 인식, 최대 {s['decel']:.1f}" + (' 운전자 제동' if s['brake'] else '')
                            for s in stopped]

  # -- on-ramps: held under what the traffic drives with nothing ahead
  ramps, held_all, gas_n = runs(en & (g('ramp') == 1) & (g('nav_valid') > 0), t, int(3 / DT)), [], 0
  for i, j in ramps:
    held = (v[i:j] < g('fleet')[i:j] - 4.0) & (g('has_lead')[i:j] == 0) & (cmd[i:j] < 0.3) & (g('fleet')[i:j] > 0)
    held_all.append(float(held.sum() * DT))
    gas_n += bool(g('gas')[i:j].any())
    m['scenes'].setdefault('onramp', []).append(f'{where(d, i)} {v[i] * 3.6:.0f}→{v[j - 1] * 3.6:.0f} km/h, 묶임 {held_all[-1]:.1f}초')
  m['onramp_n'], m['onramp_held'], m['onramp_gas'] = len(ramps), float(max(held_all)) if held_all else float('nan'), gas_n

  # -- exits: speed over the traffic's where the ramp begins
  over, last = [], -1e9
  for i in np.where((g('ramp')[1:] == 2) & (g('ramp')[:-1] == 0) & (np.diff(t) < 1))[0] + 1:
    back = max(i - int(5 / DT), 0)
    if t[i] - last > 30 and en[back:i + 1].all() and g('fleet')[i] > 0 and g('nav_valid')[i] > 0:
      last = t[i]
      over.append((v[i] - g('fleet')[i]) * 3.6)
      m['scenes'].setdefault('exit', []).append(f'{where(d, i)} {v[i] * 3.6:.0f} km/h, 흐름보다 {over[-1]:+.0f}')
  m['exit_n'] = len(over)
  m['exit_over'] = float(np.median(over)) if over else float('nan')

  # -- the bus when the panda takes the steering frame over
  gaps, breaks = [], 0
  for route, frames, safety in d.starts:
    if not frames:
      continue
    f = np.array(frames)
    ours = np.where(f[:, 1] == 128)[0]
    if not len(ours) or ours[0] == 0:
      continue
    k = ours[0]
    heard = f[max(k - 100, 0):k + 100]             # what the steering rack hears: the stock frame, then ours
    gaps.append(float(np.max(np.diff(heard[:, 0])) * 1e3))
    breaks += int((f[k, 2] - f[k - 1, 2]) % 16 not in (1, 2))
    m['scenes'].setdefault('relay', []).append(f'{route[:8]} 공백 {gaps[-1]:.0f} ms, 카운터 {int(f[k - 1, 2])}→{int(f[k, 2])}')
  m['relay_n'], m['relay_gap'], m['relay_breaks'] = len(gaps), float(max(gaps)) if gaps else float('nan'), breaks

  # -- anything the car or openpilot complained about
  names = {}
  for te, n, seg, eng in d.events:
    names.setdefault((n, eng), set()).add(seg)
  m['events'] = {n: len(s) for (n, eng), s in names.items() if eng}          # while engaged: these count
  m['events_idle'] = {n: len(s) for (n, eng), s in names.items() if not eng}  # start-up and parked: listed only
  m['scenes']['events'] = [f'{n} ({"켜진 중" if eng else "꺼진 중"}, {len(s)}개 구간: {", ".join(map(str, sorted(s)[:6]))})'
                           for (n, eng), s in sorted(names.items(), key=lambda x: (not x[0][1], x[0][0]))]
  m['acc_fault'] = int(len(runs(g('acc_fault') > 0, t)))
  m['procs'] = {k: n for k, n in d.procs.items() if n >= 3}
  run_ms = g('exec_ms')[en]
  m['model_ms'] = float(np.median(run_ms)) if len(run_ms) else float('nan')
  m['model_drop'] = float(np.mean(g('drop_pct')[en] > 20)) if en.any() else 0.0
  return m


# ---------------------------------------------------------------- judging

PASS, FAIL, WATCH, NONE, INFO = '합격', '불합격', '주의', '장면 없음', '참고'


@dataclass
class Check:
  group: str
  name: str
  status: str
  got: str
  ref: str = ''
  scenes: list = field(default_factory=list)


def on(params: dict, key: str) -> bool:
  """Whether the drive ran with it. A key the drive's build did not know is a feature it did not have."""
  return params.get(key, '0') not in ('', '0', 'false', 'False')


# reference drives too short on the scene to count toward the spread: (metric, the amount it needs, how much)
SPREAD = {'bend_swing': ('bend_min', 0.8), 'bend_jerk': ('bend_min', 0.8), 'straight_swing': ('straight_min', 2.0),
          'straight_jerk': ('straight_min', 2.0), 'launch_v8': ('launch_n', 4), 'launch_a4': ('launch_n', 4)}


def grade(value: float, pooled: float, spread: list, tol: float = 1.10, margin: float = 0.0, lower_is_worse: bool = False) -> str:
  """PASS within the pooled reference's tolerance, FAIL beyond every reference drive, WATCH between."""
  sign = -1.0 if lower_is_worse else 1.0
  limit = pooled * tol + margin if not lower_is_worse else pooled - margin
  if sign * value <= sign * limit:
    return PASS
  worst = max((sign * x for x in spread), default=sign * limit)
  return FAIL if sign * value > max(worst, sign * limit) else WATCH


def worst_of(a: str, b: str) -> str:
  return FAIL if FAIL in (a, b) else WATCH if WATCH in (a, b) else PASS


def checks(m: dict, b: dict, params: dict) -> list[Check]:
  out = []
  sc = m['scenes']
  sp = b.get('spread', {})

  def add(group, name, status, got, ref='', scenes=()):
    out.append(Check(group, name, status, got, ref, list(scenes)))

  # ---- 조향
  if m['bend_min'] < 1.0:
    add('조향', '커브 출렁임', NONE, f"커브 {m['bend_min']:.1f}분 (1분 필요)")
    add('조향', '커브 차선 위치', NONE, '')
  else:
    st = worst_of(grade(m['bend_swing'], b['bend_swing'], sp.get('bend_swing', [])), grade(m['bend_jerk'], b['bend_jerk'], sp.get('bend_jerk', [])))
    add('조향', '커브 출렁임', st, f"횡가속 흔들림 {m['bend_swing']:.3f}, 저크 {m['bend_jerk']:.2f} (커브 {m['bend_min']:.1f}분)",
        f"기준 {b['bend_swing']:.3f} / {b['bend_jerk']:.2f} (+10 %까지), 기준 주행 중 최악 {max(sp.get('bend_swing', [0])):.3f} / {max(sp.get('bend_jerk', [0])):.2f}")
    if on(params, 'LaneCenteringAim'):
      st = PASS if -0.05 < m['bend_inside'] < 0.15 else FAIL
      add('조향', '커브 차선 위치', st, f"안쪽으로 {m['bend_inside']:+.2f} m (이동량 평균 {m['aim_bend']:+.2f} m)",
          f"에임 없이 {b['bend_inside']:+.2f}; 목표 -0.05 ~ +0.15")
    else:
      add('조향', '커브 차선 위치', INFO, f"안쪽으로 {m['bend_inside']:+.2f} m (에임 없이 달림)", f"기준 {b['bend_inside']:+.2f}")
  if m['straight_min'] >= 2.0:
    st = worst_of(grade(m['straight_swing'], b['straight_swing'], sp.get('straight_swing', []), 1.15),
                  grade(m['straight_jerk'], b['straight_jerk'], sp.get('straight_jerk', []), 1.15))
    add('조향', '직선 안정', st, f"흔들림 {m['straight_swing']:.3f}, 저크 {m['straight_jerk']:.2f}",
        f"기준 {b['straight_swing']:.3f} / {b['straight_jerk']:.2f} (+15 %까지), 최악 {max(sp.get('straight_swing', [0])):.3f} / {max(sp.get('straight_jerk', [0])):.2f}")
    if on(params, 'LaneCenteringAim') and m['aim_used'] > 0:
      bad = abs(m['aim_straight']) > 0.06 or m['aim_limit'] > 0.05 or abs(m['straight_centre']) > abs(b['straight_centre']) + 0.06
      add('조향', '에임 이동량', FAIL if bad else PASS,
          f"직선 평균 {m['aim_straight']:+.3f} m, 한계(0.30)에 {100 * m['aim_limit']:.1f} %, 직선 차선 위치 {m['straight_centre']:+.2f} m",
          f"직선 |평균| 0.06 이하, 한계 5 % 이하, 차선 위치 기준 {b['straight_centre']:+.2f}")
    elif on(params, 'LaneCenteringAim'):
      add('조향', '에임 이동량', FAIL, 'laneAimShift가 한 번도 움직이지 않음 (기능이 돌지 않음)')
  else:
    add('조향', '직선 안정', NONE, f"직선 {m['straight_min']:.1f}분 (2분 필요)")
  if m['lc_n'] >= 3:
    slow = m['lc_seconds'] > b['lc_seconds'] + 0.5 or m['lc_helped'] > b['lc_helped'] + 0.25
    add('조향', '차선변경', WATCH if slow else PASS, f"{m['lc_n']}회, 중앙값 {m['lc_seconds']:.1f}초, 운전자가 도운 비율 {100 * m['lc_helped']:.0f} %",
        f"기준 {b['lc_seconds']:.1f}초 / {100 * b['lc_helped']:.0f} %")
  else:
    add('조향', '차선변경', NONE, f"{m['lc_n']}회 (3회 필요)")
  add('조향', '조향 폴트', FAIL if m['steer_fault'] else PASS, f"켜진 중 EPAS 폴트 {m['steer_fault']}회, 요구 횡가속 최대 {m['lat_cmd_max']:.1f}",
      f"폴트 0 (기준 주행 최대 {b['lat_cmd_max']:.1f}, 4.6부터 폴트가 났음)")

  # ---- 앞차 인식
  if m['lead_min'] >= 3.0:
    bad = m['flip_jumps_min'] > b['flip_jumps_min'] * 1.5 + 0.2
    add('앞차', '레이더·카메라 매칭', WATCH if bad else PASS,
        f"전환 {m['flips_min']:.1f}회/분, 그중 5 m 넘게 튄 것 {m['flip_jumps_min']:.2f}회/분, 레이더 비율 {100 * m['radar_share']:.0f} %",
        f"기준 {b['flips_min']:.1f} / {b['flip_jumps_min']:.2f} / {100 * b['radar_share']:.0f} % (에임이 시야를 옮겨 생길 수 있는 부작용)")
  else:
    add('앞차', '레이더·카메라 매칭', NONE, f"앞차 {m['lead_min']:.1f}분 (3분 필요)")
  if m['stopped_n']:
    bad = m['stopped_decel'] < -3.5 or m['fcw']
    add('앞차', '정지차 인식', FAIL if bad else PASS,
        f"{m['stopped_n']}건, 인식 시점 여유 중앙값 {m['stopped_room']:.1f}초, 최대 감속 {m['stopped_decel']:.1f}, 운전자 제동 {m['stopped_driver']}건",
        f"감속 -3.5보다 약하게, FCW 없음 (기준 주행 여유 {b.get('stopped_room', float('nan')):.1f}초, 최대 {b.get('stopped_decel', float('nan')):.1f})", sc['stopped'])
  else:
    add('앞차', '정지차 인식', NONE, '달리는 중에 정지차를 만난 적 없음')
  if m['follow_min'] > 0:
    add('앞차', '앞앞차 감속 반영', INFO, f"추종 시간의 {100 * m['lead_ahead_share']:.1f} %에서 앞차 가속도를 낮춰 봄", f"기능 없던 기준 {100 * b['lead_ahead_share']:.1f} %")

  # ---- 가감속
  add('가감속', '전방 충돌 경고', FAIL if m['fcw'] else PASS, f"{m['fcw']}회", '0회', sc['fcw'])
  if m['follow_min'] >= 5.0:
    bad = m['brake_on'] > b['brake_on'] + 0.6 or m['cmd_jerk'] > b['cmd_jerk'] * 1.25
    add('가감속', '추종 잔제동', WATCH if bad else PASS,
        f"제동 {m['brake_on']:.2f}회/분 (앞차 탓 아닌 것 {m['unprovoked']:.2f}), 가속 {m['accel_on']:.2f}회/분, 저크 {m['cmd_jerk']:.2f}, 차간 {m['headway']:.2f}초 ({m['follow_min']:.0f}분)",
        f"기준 {b['brake_on']:.2f} ({b['unprovoked']:.2f}) / {b['accel_on']:.2f} / {b['cmd_jerk']:.2f} / {b['headway']:.2f}초; 교통량 영향 큼")
    bad = m['accel_brake_h'] > b['accel_brake_h'] * 1.3 + 2
    add('가감속', '가속하다 제동', WATCH if bad else PASS, f"{m['accel_brake_h']:.1f}회/시간", f"기준 {b['accel_brake_h']:.1f} (ApproachCoast 전)", sc['accel_brake'])
  else:
    add('가감속', '추종 잔제동', NONE, f"고속 추종 {m['follow_min']:.1f}분 (5분 필요)")
  if m['engaged_min'] >= 10:
    bad = m['hard_h'] > b['hard_h'] * 1.5 + 2
    add('가감속', '급제동 (-2 이하)', WATCH if bad else PASS, f"{m['hard_h']:.1f}회/시간, 최대 {m['hard_worst']:.1f}", f"기준 {b['hard_h']:.1f}회/시간; 교통량 영향 큼", sc['hard'])
  if m['launch_n'] >= 5:
    st = worst_of(grade(m['launch_v8'], b['launch_v8'], sp.get('launch_v8', []), 1.0, 3.0),
                  grade(m['launch_a4'], b['launch_a4'], sp.get('launch_a4', []), margin=0.15, lower_is_worse=True))
    add('가감속', '앞차 따라 출발', st, f"{m['launch_n']}회, 8초 뒤 앞차보다 {m['launch_v8']:.1f} km/h 느림, 첫 4초 가속 {m['launch_a4']:.2f}",
        f"기준 {b['launch_v8']:.1f} km/h / {b['launch_a4']:.2f} (약해지면 탈락)", sc.get('launch', []))
  else:
    add('가감속', '앞차 따라 출발', NONE, f"{m['launch_n']}회 (5회 필요)")
  if m['onramp_n']:
    add('가감속', '진입램프 속도', FAIL if m['onramp_held'] > 3.0 else PASS,
        f"{m['onramp_n']}곳, 흐름보다 15 km/h 넘게 느린 채 묶인 시간 최대 {m['onramp_held']:.1f}초, 가속페달 {m['onramp_gas']}곳", '3초 이하 (고치기 전 6~7초)', sc.get('onramp', []))
  else:
    add('가감속', '진입램프 속도', NONE, '켠 채로 지난 진입램프 없음')
  if m['exit_n']:
    add('가감속', '진출 전 감속', FAIL if m['exit_over'] > 12 else PASS, f"{m['exit_n']}곳, 분기점에서 흐름보다 {m['exit_over']:+.0f} km/h",
        f"12 이하 (기준 {b.get('exit_over', float('nan')):+.0f})", sc.get('exit', []))
  else:
    add('가감속', '진출 전 감속', NONE, '켠 채로 지난 진출로 없음')

  # ---- 차량·시스템
  if m['relay_n']:
    add('시스템', '시동 직후 조향 프레임 공백', FAIL if m['relay_gap'] > 45 or m['relay_breaks'] else PASS,
        f"최대 {m['relay_gap']:.0f} ms, 카운터 끊김 {m['relay_breaks']}회", '45 ms 이하, 끊김 0 (고치기 전 최대 89 ms)', sc.get('relay', []))
  else:
    add('시스템', '시동 직후 조향 프레임 공백', NONE, '전환 순간이 로그에 없음')
  new = sorted(set(m['events']) - set(b['events']))        # the reference drives had these too; anything else is new
  add('시스템', '경고·폴트', FAIL if new or m['acc_fault'] else PASS,
      f"켜진 중 {len(m['events'])}종" + (f" (기준에 없던 것: {', '.join(new)})" if new else '') + f", 꺼진 중 {len(m['events_idle'])}종, 크루즈 폴트 {m['acc_fault']}회",
      f"기준 주행에도 있던 것: {', '.join(sorted(b['events'])) or '없음'}", sc['events'])
  add('시스템', '프로세스', FAIL if m['procs'] else PASS, ', '.join(f'{k} 멈춤' for k in m['procs']) or '모두 실행', '')
  add('시스템', '모델 실행 시간', FAIL if m['model_ms'] > b['model_ms'] * 1.25 or m['model_drop'] > 0.02 else PASS,
      f"중앙값 {m['model_ms']:.1f} ms, 프레임 누락 구간 {100 * m['model_drop']:.1f} %", f"기준 {b['model_ms']:.1f} ms")
  return out


def disengagements(routes: list[str]) -> Check:
  from openpilot.tools.tesla_analysis import disengage_review as dr
  dr.LOG_ROOT = LOG_ROOT
  counts, scenes = {}, []
  for r in routes:
    for d in dr.read_route(r):
      kind, why = dr.classify(d)
      counts[kind] = counts.get(kind, 0) + 1
      if kind in ('review', 'problem'):
        seg, sec = divmod(d.route_t, 60)
        scenes.append(f'{r[:8]} {int(seg)}구간 {sec:.0f}초 {d.kph:.0f} km/h {d.cause} {why}'.rstrip())
  status = FAIL if counts.get('problem') else WATCH if counts.get('review') else PASS
  got = f"회전 {counts.get('driver_turn', 0)}, 차선변경 {counts.get('driver_lane_change', 0)}, 볼 것 {counts.get('review', 0)}, 문제 {counts.get('problem', 0)}"
  return Check('시스템', '해제', status, got, '운전자 회전·차선변경은 문제 아님', scenes)


# ---------------------------------------------------------------- printing

def report(d: Drive, m: dict, cs: list[Check]) -> str:
  lines = [f"주행 {', '.join(n[:8] for n in d.names)}  커밋 {', '.join(d.commits)}  {m['minutes']:.0f}분 중 {m['engaged_min']:.0f}분 켜짐",
           '설정 ' + '  '.join(f'{k}={v}' for k, v in sorted(d.params.items()) if v != '')]
  group = None
  for c in cs:
    if c.group != group:
      group = c.group
      lines.append(f'\n[{group}]')
    lines.append(f'  {c.status:6s} {c.name}: {c.got}' + (f'   ← {c.ref}' if c.ref else ''))
    if c.status in (FAIL, WATCH) or c.name in ('정지차 인식', '진입램프 속도', '진출 전 감속', '시동 직후 조향 프레임 공백'):
      lines += [f'           · {s}' for s in c.scenes[:12]]
      if len(c.scenes) > 12:
        lines.append(f'           · ... 외 {len(c.scenes) - 12}건')
  n = {s: sum(c.status == s for c in cs) for s in (PASS, FAIL, WATCH, NONE)}
  lines.append(f"\n합격 {n[PASS]}  불합격 {n[FAIL]}  주의 {n[WATCH]}  장면 없음 {n[NONE]}")
  return '\n'.join(lines)


def main(argv):
  if argv and argv[0] == '--baseline':
    m = measure(load(argv[1:]))
    m.pop('scenes')
    m['routes'] = argv[1:]
    each = [measure(load([r])) for r in argv[1:] if segments(r)]
    m['spread'] = {k: [e[k] for e in each if e.get(need, 0) >= amount and np.isfinite(e[k])] for k, (need, amount) in SPREAD.items()}
    json.dump(m, open(BASELINE, 'w'), indent=1, sort_keys=True)
    print(f'baseline written: {BASELINE}')
    return
  if not argv:
    raise SystemExit(__doc__)
  d = load(argv)
  m = measure(d)
  b = json.load(open(BASELINE))
  cs = checks(m, b, d.params) + [disengagements(d.names)]
  print(report(d, m, cs))
  if os.environ.get('OUT'):
    json.dump(dict(measured={k: v for k, v in m.items() if k != 'scenes'},
                   checks=[c.__dict__ for c in cs]), open(os.environ['OUT'], 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
  main(sys.argv[1:])
