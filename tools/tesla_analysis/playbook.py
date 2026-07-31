#!/usr/bin/env python3
"""One-shot longitudinal diagnosis playbook: point it at recorded routes, get the report.

Runs, per route, every analysis this project reached its longitudinal conclusions with:

  1. car ID        -- which physical Model X (0x398/0x359 config broadcast) + wall-clock time
  2. gap usage     -- engaged time at each steering-wheel gap (1-7), all vs city
  3. FCW           -- "Emergency Braking" episodes, rate per gap, onset context
  4. stop distance -- resting dRel behind stopped vs decelerating leads (too-close check)
  5. headway       -- measured following headway (s) per gap
  6. accel response-- how quickly accel builds when a lead pulls away (sluggish-follow check)

Each section prints the numbers and a one-line 진단.

  export OP_LOG_ROOT=~/op-logs           # where <route>--<n>/rlog.zst live
  PYTHONPATH=<repo> tools/tesla_analysis/playbook.py                 # every route
  PYTHONPATH=<repo> tools/tesla_analysis/playbook.py 00000013 00000016
  PYTHONPATH=<repo> tools/tesla_analysis/playbook.py 00000016 --json  # machine-readable

A truncated segment can crash capnp with a C++ terminate that Python cannot catch, so each
segment is read in its own subprocess (this same file, --worker); one bad segment is skipped,
the rest of the drive still reports. See tools/tesla_analysis/README.md for the field notes.
"""
import os
import sys
import glob
import json
import math
import datetime
import statistics
import subprocess
from collections import Counter, defaultdict

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
DT = 0.05  # radarState / longitudinalPlan cadence used for time-weighting

# Known cars in this log set: first byte of GTW_carConfig (0x398) splits them. See project notes.
CARS = {'29': 'Car A', '2b': 'Car B'}

# Current in-tree max-accel ceiling (get_max_accel), for the accel-response section.
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10., 25., 40.]

# gap 1-7 target tFollow (for context in the headway section). Kept loose -- the point is the
# measured headway, not the exact table, which the running build may have retuned.
CITY_MS = 16.0  # < ~58 km/h counts as city

FIELDS = ['t', 'vEgo', 'aEgo', 'ss', 'en', 'gap', 'accel', 'aTarget', 'allowT',
          'fcw', 'dRel', 'vRel', 'vLeadK', 'aLeadK', 'st', 'radar']
IDX = {f: i for i, f in enumerate(FIELDS)}


# ---------------------------------------------------------------------------- worker (one segment)
def worker(seg):
  """Emit one meta line (#M<tab>f398<tab>f359<tab>wallTime) and one TSV row per radarState."""
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from scan_can import read_events
  f398 = f359 = ''
  wall = 0.0
  vEgo = aEgo = accel = aTarget = 0.0
  ss = en = gap = allowT = 0
  dRel = vRel = vLeadK = aLeadK = 0.0
  st = radar = 0
  out = []
  try:
    for e in read_events(seg):
      w = e.which()
      t = e.logMonoTime / 1e9
      if w == 'can':
        for m in e.can:
          if m.src == 0 and m.address == 0x398 and not f398:
            f398 = bytes(m.dat).hex()
          elif m.src == 0 and m.address == 0x359 and not f359:
            f359 = bytes(m.dat).hex()
      elif w == 'clocks' and not wall:
        wall = e.clocks.wallTimeNanos / 1e9
      elif w == 'carState':
        vEgo = e.carState.vEgo
        aEgo = e.carState.aEgo
        ss = int(e.carState.standstill)
        gap = int(e.carState.cruiseState.gapAdjust)
      elif w == 'selfdriveState':
        en = int(e.selfdriveState.enabled)
      elif w == 'carControl':
        accel = e.carControl.actuators.accel
      elif w == 'longitudinalPlan':
        aTarget = e.longitudinalPlan.aTarget
        allowT = int(e.longitudinalPlan.allowThrottle)
        fcw = int(e.longitudinalPlan.fcw)
      elif w == 'radarState':
        L = e.radarState.leadOne
        dRel, vRel, vLeadK, aLeadK = L.dRel, L.vRel, L.vLeadK, L.aLeadK
        st, radar = int(L.status), int(L.radar)
        out.append(f"{t:.3f}\t{vEgo:.2f}\t{aEgo:.2f}\t{ss}\t{en}\t{gap}\t{accel:.3f}\t{aTarget:.3f}\t"
                   f"{allowT}\t{fcw}\t{dRel:.2f}\t{vRel:.2f}\t{vLeadK:.2f}\t{aLeadK:.2f}\t{st}\t{radar}")
  except Exception:
    pass
  sys.stdout.write(f"#M\t{f398}\t{f359}\t{wall}\n")
  sys.stdout.write("\n".join(out))
  if out:
    sys.stdout.write("\n")


# ------------------------------------------------------------------------------------- collection
def collect(prefix):
  """Run the worker over every segment of a route, return (rows, meta)."""
  segs = sorted(glob.glob(f"{LOG_ROOT}/{prefix}--*/rlog.zst"),
                key=lambda p: int(p.split('--')[-1].split('/')[0]))
  rows = []
  meta = {'f398': '', 'f359': '', 'wall': 0.0}
  for seg in segs:
    try:
      r = subprocess.run([sys.executable, os.path.abspath(__file__), '--worker', seg],
                         capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
      continue
    for line in r.stdout.splitlines():
      if line.startswith('#M'):
        _, f398, f359, wall = line.split('\t')
        meta['f398'] = meta['f398'] or f398
        meta['f359'] = meta['f359'] or f359
        meta['wall'] = meta['wall'] or float(wall or 0)
        continue
      p = line.split('\t')
      if len(p) == len(FIELDS):
        rows.append((float(p[0]), float(p[1]), float(p[2]), int(p[3]), int(p[4]), int(p[5]),
                     float(p[6]), float(p[7]), int(p[8]), int(p[9]), float(p[10]), float(p[11]),
                     float(p[12]), float(p[13]), int(p[14]), int(p[15])))
  rows.sort()
  return rows, meta


def ceil_accel(v):
  return float(_interp(v, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x < xp[i]:
      f = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + f * (fp[i] - fp[i - 1])
  return fp[-1]


# ---------------------------------------------------------------------------------- the six checks
def sec_car(meta):
  f398, f359, wall = meta['f398'], meta['f359'], meta['wall']
  car = CARS.get(f398[:2], '알 수 없음') if f398 else '식별 불가(0x398 없음)'
  when = ''
  if wall:
    loc = datetime.datetime.fromtimestamp(wall - 7 * 3600, datetime.timezone.utc)  # device UTC, user Pacific(UTC-7)
    when = f"{loc:%Y-%m-%d %H:%M} (local, UTC-7 가정)"
  return {'car': car, 'f398': f398, 'f359': f359, 'time': when}


def sec_gap(rows):
  allc, city = Counter(), Counter()
  for r in rows:
    if r[IDX['en']] and r[IDX['gap']] > 0:
      allc[r[IDX['gap']]] += DT
      if r[IDX['vEgo']] < CITY_MS:
        city[r[IDX['gap']]] += DT
  return {g: (round(allc[g] / 60, 1), round(city[g] / 60, 1)) for g in sorted(allc)}


def sec_fcw(rows):
  eng = Counter()
  for r in rows:
    if r[IDX['en']] and r[IDX['gap']] > 0:
      eng[r[IDX['gap']]] += DT
  # merge fcw frames within 1.5s into one episode, tag by gap at onset
  episodes = []
  last_t = -99
  for r in rows:
    if r[IDX['fcw']]:
      if r[0] - last_t > 1.5:
        episodes.append(r)
      last_t = r[0]
  by_gap = Counter(e[IDX['gap']] for e in episodes)
  rate = {}
  for g in sorted(eng):
    mins = eng[g] / 60
    rate[g] = (by_gap.get(g, 0), round(mins, 1), round(by_gap.get(g, 0) / mins * 10, 1) if mins else 0)
  onsets = [{'gap': e[IDX['gap']], 'kph': round(e[IDX['vEgo']] * 3.6, 1), 'dRel': e[IDX['dRel']],
             'vRel': e[IDX['vRel']], 'aLeadK': e[IDX['aLeadK']]} for e in episodes]
  return {'rate': rate, 'onsets': onsets, 'n': len(episodes)}


def sec_stop(rows):
  """Resting dRel at standstill onsets, split stopped-car vs decelerating-lead."""
  n = len(rows)
  events = []
  i = 1
  while i < n:
    r, pr = rows[i], rows[i - 1]
    if r[IDX['ss']] and not pr[IDX['ss']] and r[IDX['en']] and r[IDX['st']] and r[IDX['dRel']] > 0:
      back = [rows[j] for j in range(max(0, i - 60), i) if rows[j][IDX['st']]]
      stopped = bool(back) and sum(1 for b in back if abs(b[IDX['vLeadK']]) < 1.5) / len(back) > 0.7
      fwd = [rows[j] for j in range(i, min(n, i + 60)) if rows[j][IDX['ss']] and rows[j][IDX['st']]]
      if fwd:
        rest = sorted(f[IDX['dRel']] for f in fwd)[len(fwd) // 2]
        events.append((rest, stopped))
      i += 40
    else:
      i += 1
  stopcar = [e[0] for e in events if e[1]]
  decel = [e[0] for e in events if not e[1]]
  out = {'n': len(events)}
  if stopcar:
    out['stopped'] = {'n': len(stopcar), 'median': round(statistics.median(stopcar), 2), 'min': round(min(stopcar), 2)}
  if decel:
    out['decel'] = {'n': len(decel), 'median': round(statistics.median(decel), 2), 'min': round(min(decel), 2)}
  return out


def sec_headway(rows):
  hw = defaultdict(list)
  for r in rows:
    if (r[IDX['en']] and r[IDX['gap']] > 0 and r[IDX['st']] and 5 < r[IDX['vEgo']] < CITY_MS
        and abs(r[IDX['vRel']]) < 1.5 and r[IDX['dRel']] > 0):
      hw[r[IDX['gap']]].append(r[IDX['dRel']] / r[IDX['vEgo']])
  out = {}
  for g in sorted(hw):
    v = sorted(hw[g])
    if len(v) >= 20:
      out[g] = {'median': round(statistics.median(v), 2), 'p15': round(v[int(len(v) * 0.15)], 2), 'n': len(v)}
  return out


def sec_accel(rows):
  """When a lead pulls away in the city, is accel ceiling-limited, throttle-gated, or just gentle?"""
  total = clip = blocked = 0
  cmds, cils = [], []
  for r in rows:
    if r[IDX['en']] and r[IDX['st']] and 3 < r[IDX['vEgo']] < 17 and r[IDX['vRel']] > 1.0:
      total += 1
      c = ceil_accel(r[IDX['vEgo']])
      if r[IDX['accel']] > c - 0.08:
        clip += 1
      if not r[IDX['allowT']]:
        blocked += 1
      if r[IDX['accel']] > 0.15:
        cmds.append(r[IDX['accel']])
        cils.append(c)
  if not total:
    return {'n': 0}
  return {'n': total, 'clip_pct': round(clip / total * 100), 'throttle_blocked_pct': round(blocked / total * 100),
          'median_cmd': round(statistics.median(cmds), 2) if cmds else 0.0,
          'median_ceil': round(statistics.median(cils), 2) if cils else 0.0}


# ------------------------------------------------------------------------------------- report text
def report(prefix, rows, meta):
  R = {'route': prefix, 'frames': len(rows), 'car': sec_car(meta), 'gap': sec_gap(rows),
       'fcw': sec_fcw(rows), 'stop': sec_stop(rows), 'headway': sec_headway(rows), 'accel': sec_accel(rows)}
  return R


def print_report(R):
  p = print
  p(f"\n{'='*70}\n라우트 {R['route']}  ({R['frames']} radar frames)\n{'='*70}")

  c = R['car']
  p(f"\n[1] 차량 식별")
  p(f"    {c['car']}   0x398={c['f398'] or '-'}  0x359={c['f359'] or '-'}")
  if c['time']:
    p(f"    시각: {c['time']}")

  p(f"\n[2] Gap 사용 (engaged 분, 전체 / 시내<58km/h)")
  if R['gap']:
    for g, (a, ci) in R['gap'].items():
      p(f"    gap {g}: {a:5.1f}분 / {ci:5.1f}분")
  else:
    p("    (engaged+gap 데이터 없음)")

  f = R['fcw']
  p(f"\n[3] FCW(이머전시) — 총 {f['n']} 에피소드")
  if f['rate']:
    p(f"    {'gap':>4} {'건수':>5} {'engaged분':>9} {'10분당':>7}")
    worst = None
    for g, (ep, mins, rt) in f['rate'].items():
      p(f"    {g:>4} {ep:>5} {mins:>9.1f} {rt:>7.1f}")
      if ep and (worst is None or rt > worst[1]):
        worst = (g, rt)
    rates = {g: v[2] for g, v in f['rate'].items() if v[0]}
    if worst and len(rates) > 1 and worst[1] > 1.5 * min(rates.values() or [0]) and worst[1] >= 1.0:
      p(f"    진단: gap {worst[0]}에서 FCW 발생률이 두드러짐 → 그 gap이 시내에서 너무 붙음.")
    elif f['n'] == 0:
      p(f"    진단: FCW 없음.")

  s = R['stop']
  p(f"\n[4] 정지 거리 (정지 시 최종 dRel)")
  if s.get('stopped'):
    st = s['stopped']
    p(f"    정지차 뒤: {st['n']}건, 중앙값 {st['median']}m, 최소 {st['min']}m")
    if st['median'] < 4.0:
      p(f"    진단: 정지차에 너무 붙음(중앙값 {st['median']}m). StopDistance 상향 / 속도PID(정밀정지) 검토.")
  if s.get('decel'):
    d = s['decel']
    p(f"    감속리드 뒤: {d['n']}건, 중앙값 {d['median']}m, 최소 {d['min']}m")
  if not s.get('stopped') and not s.get('decel'):
    p("    (정지 이벤트 없음)")

  h = R['headway']
  p(f"\n[5] 추종 헤드웨이 (시내 정속추종, 초)")
  if h:
    p(f"    {'gap':>4} {'중앙값':>7} {'p15(최근접)':>11} {'n':>6}")
    for g, v in h.items():
      p(f"    {g:>4} {v['median']:>7.2f} {v['p15']:>11.2f} {v['n']:>6}")
  else:
    p("    (정속추종 데이터 부족)")

  a = R['accel']
  p(f"\n[6] 가속 응답 (앞차 벌어질 때, 시내)")
  if a['n']:
    p(f"    프레임 {a['n']}  |  상한붙음 {a['clip_pct']}%  스로틀차단 {a['throttle_blocked_pct']}%")
    p(f"    accel 명령 중앙값 {a['median_cmd']}  vs  상한 중앙값 {a['median_ceil']}")
    if a['throttle_blocked_pct'] < 20 and a['clip_pct'] < 40 and a['median_cmd'] < 0.6 * max(a['median_ceil'], 0.1):
      p(f"    진단: 상한/스로틀 여유 있는데 명령이 낮음 → jerk 제한 램프로 '바로 못 따라감'. "
        f"DynamicTFollowGain(가속 램프 완화) 검토.")
  else:
    p("    (앞차 벌어짐 이벤트 없음)")


# -------------------------------------------------------------------------------------------- main
def main(argv):
  if argv and argv[0] == '--worker':
    worker(argv[1])
    return
  as_json = '--json' in argv
  prefixes = [a for a in argv if not a.startswith('--')]
  if not prefixes:
    prefixes = sorted({os.path.basename(p).rsplit('--', 1)[0]
                       for p in glob.glob(f"{LOG_ROOT}/*--*") if os.path.isdir(p)})
  reports = []
  for pfx in prefixes:
    rows, meta = collect(pfx)
    if not rows:
      if not as_json:
        print(f"\n라우트 {pfx}: 데이터 없음 (세그먼트/rlog 확인)")
      continue
    R = report(pfx, rows, meta)
    reports.append(R)
    if not as_json:
      print_report(R)
  if as_json:
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == '__main__':
  main(sys.argv[1:])
