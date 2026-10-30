"""Daily snapshots and the long-term battery trend, built from the always-on logger's records.

One JSON file per local calendar day (the car's time zone, BtObdTimezone, default America/Los_Angeles:
the device clock is UTC) under DAILY_DIR, apart from the raw CAN log, and always rebuildable from it:

  raw-YYYYMMDD-HH.jsonl.gz --(hour_summary, cached per file by mtime/size)--> hours/<name>.json
  hour summaries of one local day + that day's sessions/events/rx/power records --> YYYY-MM-DD.json
  every day in date order (pack assignment) --> index.json (what the dashboard lists and charts)

Rebuilding is idempotent: a day is a pure function of its sources, written over its one file, and a
day whose source signature has not changed is not touched. Exact duplicate raw lines are dropped.
Days before the raw log come from live.jsonl (one line per publish, no per-value timestamps): they are
labelled so and carry no cell-balance, resistance or decode-check figures.

Rules kept here, from the logger's own (vlog):
- a value counts only while fresh (vlog.max_age of its key); stale values are not carried forward,
  and nothing is carried from one day into the next;
- cell spread is judged only on a synchronised 96-brick sweep at |I| <= 5 A;
- resistance is kept per SOC/temperature bin and only compared within a bin;
- extremes (limits, temperatures, 12 V) keep the conditions they were measured in;
- `snapshot` is one moment (the last live sample of the day); `day` is the whole day's aggregate.

Status: NO_DATA (nothing recorded), PARTIAL (in progress, a recorded share of the car's awake time
under COMPLETE_SHARE, completeness not knowable, or live.jsonl only), OK.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import os
import statistics
import time
from datetime import date, datetime, timedelta
from zoneinfo import ZoneInfo

from openpilot.selfdrive.debug import vlog_api
from openpilot.system.btobd import tdiag, tesla_pt, vlog

VERSION = 2
DAILY_DIR = os.path.join(tdiag.LOG_DIR, 'daily')
DEFAULT_TZ = 'America/Los_Angeles'
# The device boots on a stale clock (2026-07-28) until NTP/GPS sets it; frames stamped before this are
# on that clock and belong to no real day.
STALE_CLOCK_BEFORE = 1785542400.0     # 2026-08-01 UTC
LINK_GAP_S = vlog.LINK_GAP_S          # a longer silence between two updates is the link being down
COMPLETE_SHARE = 0.8
COUNTER_SLACK_S = 120.0               # 0x562/0x3D2 are heard every ~10 s in rotation
REST_A = vlog.REST_A
SOC_BIN, TEMP_BIN = 10, 5             # resistance condition bins: SOC %, module temperature degC
MIN_REST_SETS = 3
PACK_ODO_DROP_MI = 50.0               # battery odometer going back this far = another pack
PACK_KWH_DROP = 20.0
TRACK_FRESH = ('packVoltage', 'packCurrent', 'socDisplay', 'socUI', 'nominalFullPackEnergy', 'moduleTempMax', 'cellMin',
               'maxDischargePower', 'maxRegenPower', 'bmsMaxChargeA', 'bmsMaxDischargeA', 'dcdcOutputVoltage', 'bmsState',
               'kwhChargeTotal', 'kwhDischargeTotal', 'batteryOdometerMi', 'vehicleSpeedMph', 'gear')
EXT_KEYS = ('packVoltage', 'packCurrent', 'socDisplay', 'socUI', 'nominalFullPackEnergy', 'cellMin', 'cellMax',
            'moduleTempMax', 'moduleTempMin', 'moduleSpread', 'maxDischargePower', 'maxRegenPower', 'bmsMaxChargeA',
            'bmsMaxDischargeA', 'bmsMaxChargeV', 'bmsMinDischargeV', 'brickMaxV', 'brickMinV', 'bmsTempMax', 'bmsTempMin',
            'dcdcOutputVoltage', 'dcdcOutputCurrent', 'rearInverter12V', 'frontInverter12V', 'dcdcInletTemp',
            'inverterTemp', 'motorTemp', 'chargeKw', 'driveKw')
CUM_KEYS = ('kwhChargeTotal', 'kwhDischargeTotal', 'batteryOdometerMi')


def zone() -> ZoneInfo:
  name = vlog_api._param_file('BtObdTimezone') or DEFAULT_TZ
  try:
    return ZoneInfo(name)
  except Exception:
    return ZoneInfo(DEFAULT_TZ)


def local_date(t: float, tz: ZoneInfo) -> str:
  return datetime.fromtimestamp(t, tz).strftime('%Y-%m-%d')


def day_bounds(d: str, tz: ZoneInfo) -> tuple[float, float]:
  y, m, dd = map(int, d.split('-'))
  a = datetime(y, m, dd, tzinfo=tz)
  return a.timestamp(), (a + timedelta(days=1)).timestamp()


# ---- one raw file -> per-local-day partial summaries ---------------------------------------------------
def _ctx(V: dict, fresh) -> dict:
  """The conditions a value was measured in, from values fresh at that moment only."""
  def g(k):
    return V.get(k) if fresh(k) else None
  tmin, tmax = g('moduleTempMin'), g('moduleTempMax')
  gear, bms = g('gear'), g('bmsState')
  mode = 'drive' if gear in ('D', 'R', 'N') else 'charge' if bms in vlog.BMS_CHARGING else 'park' if gear == 'P' or bms is not None else None
  return {'soc': g('socDisplay'), 'temp': round((tmin + tmax) / 2, 1) if tmin is not None and tmax is not None else None,
          'i': g('packCurrent'), 'mode': mode, 'mph': g('vehicleSpeedMph')}


class _Acc:
  """Mergeable aggregates of one local day within one raw file (or live.jsonl hour)."""

  def __init__(self):
    self.s: dict = {'frames': 0, 'dupes': 0, 'first': None, 'last': None, 'minutes': set(), 'keys': {}, 'ext': {},
                    'firstVals': {}, 'lastVals': {}, 'rest': [], 'restCells': None, 'pairs': [], 'vrf': {},
                    'faults': {}, 'fresh': {}, 'cand': {}, 'snapT': None, 'source': 'raw'}

  def frame(self, wall: float) -> None:
    s = self.s
    s['frames'] += 1
    s['first'] = wall if s['first'] is None else min(s['first'], wall)
    s['last'] = wall if s['last'] is None else max(s['last'], wall)
    s['minutes'].add(int(wall // 60))

  def update(self, k: str, v, wall: float, ctx: dict | None) -> None:
    s = self.s
    r = s['keys'].get(k)
    if r is None:
      r = s['keys'][k] = {'n': 0, 'first': wall, 'last': wall, 'maxGap': 0.0}
    elif 0 < wall - r['last'] <= LINK_GAP_S:
      r['maxGap'] = max(r['maxGap'], round(wall - r['last'], 1))
    r['n'] += 1
    r['last'] = wall
    s['lastVals'][k] = [v, round(wall, 2)]
    if k in CUM_KEYS and k not in s['firstVals']:
      s['firstVals'][k] = [v, round(wall, 2)]
    if k in EXT_KEYS and isinstance(v, (int, float)) and not isinstance(v, bool):
      self.ext(k, v, wall, ctx)

  def ext(self, k: str, v: float, wall: float, ctx: dict | None) -> None:
    e = self.s['ext'].get(k)
    rec = [round(v, 3), round(wall, 1), ctx]
    if e is None:
      self.s['ext'][k] = {'min': rec, 'max': rec, 'n': 1}
      return
    e['n'] += 1
    if v < e['min'][0]:
      e['min'] = rec
    if v > e['max'][0]:
      e['max'] = rec

  def done(self) -> dict:
    s = dict(self.s)
    s['minutes'] = sorted(s['minutes'])
    return s


def _read_raw(path: str):
  """(wall, mono, id, data) of one raw file; a file cut by power loss keeps what decoded before the cut;
  exact duplicate lines are counted and dropped."""
  seen = set()
  dupes = 0
  out = []
  try:
    f = gzip.open(path, 'rt')
    try:
      for ln in f:
        if ln in seen:
          dupes += 1
          continue
        seen.add(ln)
        try:
          wall, mono, fid, data, _ = json.loads(ln)
          out.append((wall, mono, int(fid, 16), bytes.fromhex(data)))
        except (ValueError, TypeError):
          continue
    except (OSError, EOFError):
      pass
    finally:
      f.close()
  except OSError:
    pass
  return out, dupes


def summarize_raw(path: str, tz: ZoneInfo) -> dict:
  """{local date: partial day summary} of one raw hour file."""
  from openpilot.system.btobd.tesla_sx import TeslaSXState
  frames, dupes = _read_raw(path)
  # raw from before the poll-order fix (2026-10-30) heard 0x102 once per 8-12 s cycle; the logger replays
  # that with a 15 s limit on fast values (vlog.REPLAY_FAST_S), and so does this, decided per file
  t102 = sorted({round(m, 1) for _, m, i, _ in frames if i == 0x102})
  gaps = [b - a for a, b in zip(t102, t102[1:], strict=False) if 0 < b - a < 60]
  fast_s = vlog.REPLAY_FAST_S if gaps and statistics.median(gaps) > vlog.FRESH_S else vlog.FRESH_S
  car = TeslaSXState()
  accs: dict[str, _Acc] = {}
  bad_sigs = [x for x in tesla_pt.LIVE_SIGS if x.bad]
  cand = {}
  for x in tesla_pt.CANDIDATES:
    cand.setdefault(x.msg, []).append(x)
  win: list = []
  last_rest_min = None
  prev_mono = None
  for wall, mono, i, d in frames:
    if wall < STALE_CLOCK_BEFORE:
      continue
    if prev_mono is not None and mono < prev_mono - 1:
      car = TeslaSXState()       # the device rebooted inside this hour
      win = []
    prev_mono = mono
    acc = accs.setdefault(local_date(wall, tz), _Acc())
    acc.frame(wall)
    before = dict(car.stamp)
    car.feed(i, d, mono)
    if i in tesla_pt.LIVE_IDS:
      tesla_pt.feed_live(car, i, d, mono)
    V = car.values
    changed = [k for k, t in car.stamp.items() if before.get(k) != t]

    def fresh(k, V=V, mono=mono, car=car):
      t = car.stamp.get(k)
      return t is not None and mono - t <= vlog.max_age(k, fast_s) and V.get(k) is not None
    ctx = _ctx(V, fresh) if changed else None
    for k in changed:
      acc.update(k, V[k], wall, ctx)
    for x in cand.get(i, ()):
      v = tesla_pt.decode(x, d)
      if v is not None:
        acc.s['cand'][x.key] = acc.s['cand'].get(x.key, 0) + 1
    if 'moduleTempMax' in changed and fresh('moduleTempMin'):
      acc.ext('moduleSpread', V['moduleTempMax'] - V['moduleTempMin'], wall, ctx)
    if 'packCurrent' in changed and V.get('packVoltage') is not None:
      kw = V['packVoltage'] * V['packCurrent'] / 1000
      if V.get('bmsState') in vlog.BMS_CHARGING:
        acc.ext('chargeKw', kw, wall, ctx)
      elif V.get('gear') in ('D', 'R', 'N'):
        acc.ext('driveKw', kw, wall, ctx)
      # resistance pairs, as vlog_api.file_stats, with the SOC/temperature they were taken at
      win.append((wall, V['packVoltage'], V['packCurrent']))
      while win and win[0][0] < wall - vlog_api.R_WINDOW_S:
        win.pop(0)
      for t0, v0, i0 in win:
        if abs(V['packCurrent'] - i0) > vlog_api.R_MIN_STEP_A:
          dv, di = V['packVoltage'] - v0, V['packCurrent'] - i0
          acc.s['pairs'].append([round(dv / di * 1000, 2), round(wall, 1), round(abs(dv), 3), round(abs(di), 1),
                                 round(wall - t0, 1), ctx['soc'], ctx['temp']])
          break
    if i == 0x102:
      # the logger takes its sample right after 0x102: how many tracked values were fresh then
      acc.s['snapT'] = wall
      for k in TRACK_FRESH:
        f = acc.s['fresh'].setdefault(k, [0, 0])
        f[0] += 1
        f[1] += fresh(k)
    if 'cellSync' in changed and V.get('cellSync') and fresh('packCurrent') and abs(V['packCurrent']) <= REST_A:
      cs = car.cell_set(mono)
      minute = int(wall // 60)
      if cs and minute != last_rest_min:
        last_rest_min = minute
        lo = min(range(96), key=lambda j: cs[j])
        hi = max(range(96), key=lambda j: cs[j])
        acc.s['rest'].append({'t': round(wall, 1), 'mv': round((cs[hi] - cs[lo]) * 1000, 1), 'low': lo + 1, 'high': hi + 1,
                              'soc': ctx['soc'] if ctx else None, 'temp': ctx['temp'] if ctx else None, 'i': V['packCurrent']})
        acc.s['restCells'] = {'t': round(wall, 1), 'cells': cs}
    for name, err in vlog_api.verify_frame(i, changed, V, car, mono):
      k = name.split(':')[0]
      r = acc.s['vrf'].setdefault(k, [0, 0])
      r[0] += 1
      r[1] += abs(err - vlog_api.VERIFY_CENTER.get(k, 0.0)) > vlog_api.VERIFY[k][3]
    for x in bad_sigs:
      if x.key in changed and V[x.key] in x.bad:
        acc.s['faults'][x.label] = acc.s['faults'].get(x.label, 0) + 1
  out = {}
  for day, a in accs.items():
    s = a.done()
    s['dupes'] = dupes if day == min(accs) else 0
    s['fastS'] = fast_s
    out[day] = s
  return out


def summarize_live(path: str, until: float, tz: ZoneInfo) -> dict:
  """{local date: partial summary} from live.jsonl lines before `until` (the first raw frame). A line
  carries the values of one publish without per-value times: every value on it counts as heard then,
  so no cell balance, resistance, decode check or freshness share is derived from these."""
  from openpilot.system.btobd.tesla_sx import bms_hv
  accs: dict[str, _Acc] = {}
  prev: dict = {}
  try:
    f = open(path, errors='replace')
  except OSError:
    return {}
  with f:
    for ln in f:
      if '"live"' not in ln:
        continue
      try:
        r = json.loads(ln)
      except ValueError:
        continue
      t = r.get('t') or 0
      if t >= until:
        break
      if t < STALE_CLOCK_BEFORE or r.get('event') != 'live':
        continue
      v = dict(r.get('values') or {})
      raw = (r.get('raw') or {}).get('102')
      if raw:
        try:
          v.update(bms_hv(bytes.fromhex(raw.replace(' ', '')), offset=None))
        except ValueError:
          pass
      acc = accs.setdefault(local_date(t, tz), _Acc())
      acc.s['source'] = 'live.jsonl'
      acc.frame(t)
      ctx = _ctx(v, lambda k: True)
      for k, x in v.items():
        if isinstance(x, (int, float, str)) and prev.get(k) != x:     # a republished value is not a new reading
          acc.update(k, x, t, ctx)
      if v.get('moduleTempMax') is not None and v.get('moduleTempMin') is not None:
        acc.ext('moduleSpread', v['moduleTempMax'] - v['moduleTempMin'], t, ctx)
      prev = v
  return {d: a.done() for d, a in accs.items()}


# ---- partial summaries -> one day ----------------------------------------------------------------------
def _merge(parts: list) -> dict:
  m: dict = {'frames': 0, 'dupes': 0, 'first': None, 'last': None, 'minutes': set(), 'keys': {}, 'ext': {}, 'firstVals': {},
             'lastVals': {}, 'rest': [], 'restCells': None, 'pairs': [], 'vrf': {}, 'faults': {}, 'fresh': {}, 'cand': {},
             'snapT': None, 'sources': set()}
  for p in sorted(parts, key=lambda p: p['first'] or 0):
    m['sources'].add(p.get('source', 'raw'))
    m['frames'] += p['frames']
    m['dupes'] += p.get('dupes', 0)
    m['first'] = p['first'] if m['first'] is None else min(m['first'], p['first'])
    m['last'] = p['last'] if m['last'] is None else max(m['last'], p['last'])
    m['minutes'].update(p['minutes'])
    for k, r in p['keys'].items():
      a = m['keys'].get(k)
      if a is None:
        m['keys'][k] = dict(r)
        continue
      gap = r['first'] - a['last']
      a['maxGap'] = max(a['maxGap'], r['maxGap'], gap if 0 < gap <= LINK_GAP_S else 0)
      a['n'] += r['n']
      a['last'] = max(a['last'], r['last'])
      a['first'] = min(a['first'], r['first'])
    for k, e in p['ext'].items():
      a = m['ext'].get(k)
      if a is None:
        m['ext'][k] = {'min': e['min'], 'max': e['max'], 'n': e['n']}
        continue
      a['n'] += e['n']
      if e['min'][0] < a['min'][0]:
        a['min'] = e['min']
      if e['max'][0] > a['max'][0]:
        a['max'] = e['max']
    for k, v in p['firstVals'].items():
      if k not in m['firstVals'] or v[1] < m['firstVals'][k][1]:
        m['firstVals'][k] = v
    for k, v in p['lastVals'].items():
      if k not in m['lastVals'] or v[1] >= m['lastVals'][k][1]:
        m['lastVals'][k] = v
    m['rest'] += p['rest']
    if p['restCells'] and (m['restCells'] is None or p['restCells']['t'] > m['restCells']['t']):
      m['restCells'] = p['restCells']
    m['pairs'] += p['pairs']
    for d_from, d_to in (('vrf', 'vrf'), ('fresh', 'fresh')):
      for k, (a, b) in p[d_from].items():
        x = m[d_to].setdefault(k, [0, 0])
        x[0] += a
        x[1] += b
    for d_from in ('faults', 'cand'):
      for k, n in p[d_from].items():
        m[d_from][k] = m[d_from].get(k, 0) + n
    if p.get('snapT') and (m['snapT'] is None or p['snapT'] > m['snapT']):
      m['snapT'] = p['snapT']
  return m


def _stat(vals: list) -> dict | None:
  if not vals:
    return None
  v = sorted(vals)
  return {'min': v[0], 'max': v[-1], 'median': round(statistics.median(v), 3), 'n': len(v)}


def _ext(m: dict, k: str) -> dict | None:
  e = m['ext'].get(k)
  if not e:
    return None
  return {'min': {'v': e['min'][0], 't': e['min'][1], 'at': e['min'][2]},
          'max': {'v': e['max'][0], 't': e['max'][1], 'at': e['max'][2]}, 'n': e['n']}


def _resistance(pairs: list) -> dict:
  """Per SOC/temperature bin: the median with its conditions and confidence. Bins are never pooled:
  a 20 C reading and a 5 C reading of the same pack differ by more than a year of ageing."""
  bins: dict = {}
  unbinned = 0
  for r, t, dv, di, dt, soc, temp in pairs:
    if soc is None or temp is None:
      unbinned += 1
      continue
    key = f'{int(soc // SOC_BIN * SOC_BIN)}|{int(temp // TEMP_BIN * TEMP_BIN)}'
    bins.setdefault(key, []).append((r, t, dv, di, dt, soc, temp))
  out = []
  for key, ps in sorted(bins.items()):
    rec = vlog_api._r_record(ps)
    soc_lo, temp_lo = map(int, key.split('|'))
    out.append({'bin': key, 'soc': [soc_lo, soc_lo + SOC_BIN], 'temp': [temp_lo, temp_lo + TEMP_BIN], **rec})
  usable = [b for b in out if b['conf'] in ('high', 'medium')]
  head = max(usable, key=lambda b: b['n']) if usable else None
  return {'bins': out, 'headline': head, 'pairs': len(pairs), 'unbinned': unbinned}


def _cell_balance(rest: list) -> dict:
  if not rest:
    return {'n': 0, 'judged': False, 'why': '휴지(|I| ≤ 5 A) 중 96셀 동시 측정 없음'}
  mv = [r['mv'] for r in rest]
  socs = [r['soc'] for r in rest if r['soc'] is not None]
  temps = [r['temp'] for r in rest if r['temp'] is not None]
  lows: dict = {}
  for r in rest:
    lows[r['low']] = lows.get(r['low'], 0) + 1
  top = sorted(lows.items(), key=lambda kv: -kv[1])[:5]
  judged = len(rest) >= MIN_REST_SETS
  return {'n': len(rest), 'medianMv': round(statistics.median(mv), 1), 'minMv': min(mv), 'maxMv': max(mv),
          'soc': [min(socs), max(socs)] if socs else None, 'temp': [min(temps), max(temps)] if temps else None,
          'judged': judged, 'why': '' if judged else f'동시 측정 {len(rest)}회 (< {MIN_REST_SETS})',
          'lowest': [{'cell': c, 'count': n} for c, n in top], 'lowestShare': round(top[0][1] / len(rest), 2)}


def _decodes(vrf: dict) -> list:
  out = []
  for k, (label, *_) in vlog_api.VERIFY.items():
    n, miss = vrf.get(k, (0, 0))
    status = 'insufficient' if n < vlog_api.VERIFY_MIN_N else 'confirmed' if miss <= vlog_api.VERIFY_MAX_MISS * n else 'failed'
    out.append({'key': k, 'label': label, 'n': n, 'miss': miss, 'status': status, 'statusText': vlog_api.VERIFY_TEXT[status]})
  return out


def _snapshot(m: dict) -> dict:
  """The car at one moment: the day's last logger sample (last 0x102), with each value's own time.
  A value older than its freshness limit at that moment is listed as missing, not copied in."""
  t = m['snapT'] or m['last']
  if t is None:
    return {'t': None, 'values': {}, 'missing': []}
  vals, missing = {}, []
  live = 'live.jsonl' in m['sources'] and 'raw' not in m['sources']
  for k, (v, at) in m['lastVals'].items():
    age = round(t - at, 1)
    if live or age <= vlog.max_age(k):
      vals[k] = {'v': v, 't': at, 'ageS': max(age, 0.0)}
    else:
      missing.append({'key': k, 'lastT': at, 'ageS': age})
  return {'t': t, 'values': vals, 'missing': sorted(missing, key=lambda x: x['key'])}


def _awake_minutes(power: list, t0: float, t1: float) -> int | None:
  rows = [r for r in power if t0 <= r['t'] < t1]
  if not rows:
    return None
  return sum(1 for r in rows if vlog.power_state(r) in ('AWAKE', 'DRIVE'))


def build_day(d: str, parts: list, tz: ZoneInfo, ctx: dict) -> dict:
  """One day's snapshot from its partial summaries and the day's sessions/events/rx/power rows."""
  t0, t1 = day_bounds(d, tz)
  out: dict = {'date': d, 'tz': str(tz.key), 'version': VERSION, 'generatedAt': round(time.time(), 1)}
  sessions = [s for s in ctx['sessions'] if t0 <= s['start'] < t1]
  events = [e for e in ctx['events'] if t0 <= e.get('t', 0) < t1]
  rx = [r for r in ctx['rx'] if t0 <= r.get('t', 0) < t1]
  today = ctx['today'] == d
  if not parts:
    out.update(status='NO_DATA', statusWhy=['이날 수신 기록 없음'], sources=[], snapshot=None, day=None,
               coverage={'recordedMin': 0, 'awakeMin': _awake_minutes(ctx['power'], t0, t1)})
    return out
  m = _merge(parts)
  ref = ctx['packNewKwh']
  recorded = len(m['minutes'])
  awake = _awake_minutes(ctx['power'], t0, t1)
  why = []
  if today:
    why.append('하루가 아직 진행 중')
  if 'live.jsonl' in m['sources']:
    why.append('live.jsonl 기록 (값별 수신 시각 없음): 셀 편차·내부저항·해독 검증 없음')
  if awake is None:
    why.append('차량 전원 기록이 없어 빠진 시간을 알 수 없음')
  elif awake and recorded < COMPLETE_SHARE * awake:
    why.append(f'차가 깨어 있던 {awake}분 중 {recorded}분만 기록')
  status = 'PARTIAL' if why else 'OK'

  nfp_e = _ext(m, 'nominalFullPackEnergy')
  nfp_day = None
  if nfp_e:
    last = m['lastVals'].get('nominalFullPackEnergy')
    nfp_day = {'min': nfp_e['min']['v'], 'max': nfp_e['max']['v'], 'last': last[0] if last else None, 'n': nfp_e['n']}
  soh = None
  if nfp_day and ref > 0:
    soh = {'value': round(nfp_day['max'] / ref * 100, 2), 'min': round(nfp_day['min'] / ref * 100, 2), 'ref': ref,
           'basis': '공칭 완충 용량 최댓값 ÷ 신품 기준'}
  cum = {}
  for k in CUM_KEYS:
    a, b = m['firstVals'].get(k), m['lastVals'].get(k)
    if a and b:
      cum[k] = {'first': a[0], 'last': b[0], 'diff': round(b[0] - a[0], 3), 'from': a[1], 'to': b[1]}
  charges = []
  for s in sessions:
    if s['type'] in vlog.CHARGE_MODES:
      charges.append({k: s.get(k) for k in ('type', 'start', 'end', 'durationS', 'coveredS', 'gapS', 'kwhIn', 'kwMax', 'avgKw',
                                            'socStart', 'socEnd', 'endCause', 'endWhy', 'tempStart', 'tempEnd', 'parts')}
                     | {'endLabel': vlog.CHARGE_END.get(s.get('endCause'), '원인 기록 없음')})
  drives = [s for s in sessions if s['type'] == 'DRIVE']
  drive_mi = sum((s.get('miles') or 0) for s in drives)
  # a counter difference only stands for the day when its first and last readings bracket every
  # session of that kind (0x562/0x3D2 rotate; on 2026-10-30 the odometer was first heard after the
  # morning drive and read 0 mi for a 15 mi day)
  def brackets(k, ss):
    c = cum.get(k)
    return bool(c) and all(c['from'] <= x['start'] + COUNTER_SLACK_S and c['to'] >= x['end'] - COUNTER_SLACK_S for x in ss)
  for k, ss in (('batteryOdometerMi', drives), ('kwhChargeTotal', sessions), ('kwhDischargeTotal', sessions)):
    if k in cum:
      cum[k]['coversSessions'] = brackets(k, ss)
  fresh = {k: {'samples': a, 'fresh': b, 'share': round(b / a, 3) if a else None} for k, (a, b) in m['fresh'].items()}
  keys = {k: {'n': r['n'], 'first': r['first'], 'last': r['last'], 'maxGapS': r['maxGap']} for k, r in m['keys'].items()}
  ids: dict = {}
  for r in rx:
    x = ids.setdefault(r['id'], {'polls': 0, 'hits': 0, 'maxMiss': 0, 'gapMax': 0.0})
    x['polls'] += r['polls']
    x['hits'] += r['hits']
    x['maxMiss'] = max(x['maxMiss'], r.get('maxMiss') or 0)
    x['gapMax'] = max(x['gapMax'], r.get('gapMax') or 0)
  for x in ids.values():
    x['rate'] = round(x['hits'] / x['polls'], 3) if x['polls'] else None
  anomalies = [{'t': e['t'], 'type': e['type'], 'label': vlog_api.EVENT_NAMES.get(e['type'], e['type']), 'value': e.get('value')}
               for e in events]
  anomalies += [{'t': None, 'type': 'FAULT', 'label': f'{k} 고장 값', 'value': n} for k, n in m['faults'].items()]
  anomalies += [{'t': c['end'], 'type': 'CHARGE_ABNORMAL', 'label': '충전 비정상 중단', 'value': ' · '.join(c['endWhy'] or [])}
                for c in charges if c.get('endCause') == 'abnormal']
  decodes = _decodes(m['vrf']) if 'raw' in m['sources'] else []
  unverified = [{'key': k, 'label': tesla_pt.SIG_BY_KEY[k].label if k in tesla_pt.SIG_BY_KEY else k, 'frames': n,
                 'why': '신호 의미 미검증'} for k, n in m['cand'].items()]
  unverified += [{'key': x['key'], 'label': x['label'], 'frames': x['n'], 'why': '해독 검증 ' + x['statusText']}
                 for x in decodes if x['status'] != 'confirmed' and x['n']]
  out.update(status=status, statusWhy=why, sources=sorted(m['sources']), snapshot=_snapshot(m),
             coverage={'firstT': m['first'], 'lastT': m['last'], 'frames': m['frames'], 'duplicateLines': m['dupes'],
                       'recordedMin': recorded, 'awakeMin': awake},
             day={'nominalFullPack': nfp_day, 'soh': soh,
                  'soc': _ext(m, 'socDisplay'), 'socUI': _ext(m, 'socUI'),
                  'cellBalance': _cell_balance(m['rest']), 'restCells': m['restCells'],
                  'moduleSpread': _ext(m, 'moduleSpread'),
                  'temps': {k: _ext(m, k) for k in ('moduleTempMax', 'moduleTempMin', 'inverterTemp', 'motorTemp', 'dcdcInletTemp')},
                  'resistance': _resistance(m['pairs']),
                  'energy': {'counters': cum, 'chargeSessionsKwh': round(sum(c['kwhIn'] or 0 for c in charges), 3),
                             'odometerMi': cum['batteryOdometerMi']['diff'] if cum.get('batteryOdometerMi', {}).get('coversSessions') else None,
                             'driveSessionsMi': round(drive_mi, 2), 'drives': len(drives)},
                  'limits': {k: _ext(m, k) for k in ('maxDischargePower', 'maxRegenPower', 'bmsMaxChargeA', 'bmsMaxDischargeA',
                                                       'bmsMaxChargeV', 'bmsMinDischargeV', 'chargeKw', 'driveKw')},
                  'lowVoltage': {k: _ext(m, k) for k in ('dcdcOutputVoltage', 'dcdcOutputCurrent', 'rearInverter12V', 'frontInverter12V')},
                  'charges': charges,
                  'reception': {'freshness': fresh, 'keys': keys, 'ids': ids},
                  'decodes': decodes, 'unverified': unverified, 'anomalies': anomalies})
  return out


# ---- the whole store -----------------------------------------------------------------------------------
def _write_json(path: str, obj) -> None:
  tmp = path + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(obj, f, ensure_ascii=False, separators=(',', ':'))
  os.replace(tmp, path)


def _read_json(path: str):
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def _hour_parts(vlog_dir: str, out_dir: str, tz: ZoneInfo) -> tuple[dict, float]:
  """{date: [partial summaries]} over every raw file (cached per file) and live.jsonl before them."""
  hdir = os.path.join(out_dir, 'hours')
  os.makedirs(hdir, exist_ok=True)
  by_day: dict = {}
  first_raw = float('inf')
  try:
    names = sorted(n for n in os.listdir(vlog_dir) if n.startswith('raw-') and n.endswith('.jsonl.gz'))
  except OSError:
    names = []
  for n in names:
    p = os.path.join(vlog_dir, n)
    try:
      st = os.stat(p)
    except OSError:
      continue
    key = [st.st_mtime, st.st_size, VERSION, str(tz.key)]
    cp = os.path.join(hdir, n[:-9] + '.json')
    hit = _read_json(cp)
    if not hit or hit.get('key') != key:
      hit = {'key': key, 'days': summarize_raw(p, tz)}
      _write_json(cp, hit)
    for d, s in hit['days'].items():
      by_day.setdefault(d, []).append(s)
      if s['first'] is not None:
        first_raw = min(first_raw, s['first'])
  live = os.path.join(tdiag.LOG_DIR, tdiag.LIVE_LOG)
  if first_raw < float('inf') or os.path.exists(live):
    cutoff = first_raw if first_raw < float('inf') else time.time()
    cp = os.path.join(hdir, 'live.json')
    key = [cutoff if first_raw < float('inf') else os.path.getmtime(live), VERSION, str(tz.key)]
    hit = _read_json(cp)
    if not hit or hit.get('key') != key:
      days = {}
      for path in (live + '.1', live):
        for d, s in summarize_live(path, cutoff, tz).items():
          days.setdefault(d, []).append(s)
      hit = {'key': key, 'days': days}
      _write_json(cp, hit)
    for d, ss in hit['days'].items():
      by_day.setdefault(d, []).extend(ss)
  return by_day, first_raw


def _sig(obj) -> str:
  return hashlib.sha1(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


def assign_pack(prev: dict | None, day: dict) -> tuple[str | None, str]:
  """pack_id: named after a BMS lifetime counter at the pack's first logged day -- the battery odometer,
  or lifetime charge energy when the odometer was not heard -- since both start from zero with a new
  pack. Either counter going backwards = a different pack. A day with neither keeps the pack before it."""
  cnt = (day.get('day') or {}).get('energy', {}).get('counters', {}) if day.get('day') else {}
  odo, dis, chg = cnt.get('batteryOdometerMi'), cnt.get('kwhDischargeTotal'), cnt.get('kwhChargeTotal')

  def name():
    return f"pack@{int(odo['first'])}mi" if odo else f"pack@{int(chg['first'])}kWh" if chg else None
  if prev is None or prev.get('packId') is None:
    return name(), '처음 기록된 팩' if name() else '팩 카운터를 못 들음'
  p_odo, p_dis = prev.get('packOdo'), prev.get('packDis')
  if odo and p_odo is not None and odo['first'] < p_odo - PACK_ODO_DROP_MI:
    return name(), '배터리 주행거리가 줄어듦: 다른 팩'
  if dis and p_dis is not None and dis['first'] < p_dis - PACK_KWH_DROP:
    return name(), '누적 방전량이 줄어듦: 다른 팩'
  return prev['packId'], '같은 팩' if (odo or dis) else '카운터 없음: 전날 팩 유지'


def ensure(vlog_dir: str | None = None, out_dir: str = DAILY_DIR, force: bool = False, now: float | None = None) -> dict:
  """Bring every day's snapshot up to date; returns {'built': [...], 'days': n}."""
  vlog_dir = vlog_dir or vlog_api.VLOG_DIR
  tz = zone()
  os.makedirs(out_dir, exist_ok=True)
  now = now or time.time()
  by_day, first_raw = _hour_parts(vlog_dir, out_dir, tz)
  own = vlog.read_jsonl(os.path.join(vlog_dir, 'sessions.jsonl'))
  first_own = min((s['start'] for s in own), default=float('inf'))
  try:
    older = vlog_api.replayed(first_own)['sessions'] if vlog_dir == vlog_api.VLOG_DIR else []
  except Exception:
    older = []
  sessions = sorted([s for s in older if s['end'] < first_own] + own, key=lambda s: s['start'])
  sessions = vlog_api.merge_charges([s for s in sessions if s.get('end', 0) >= s.get('start', 0)])
  try:
    ref = float(vlog_api._param_file('BtObdPackNewKwh') or 0)
  except ValueError:
    ref = 0.0
  ctx = {'sessions': sessions, 'events': vlog.read_jsonl(os.path.join(vlog_dir, 'events.jsonl')),
         'rx': vlog.read_jsonl(os.path.join(vlog_dir, 'rx.jsonl')),
         'power': vlog.read_jsonl(os.path.join(vlog_dir, 'power.jsonl')), 'packNewKwh': ref,
         'today': local_date(now, tz)}
  if not by_day:
    _write_json(os.path.join(out_dir, 'index.json'), {'tz': str(tz.key), 'days': [], 'packs': []})
    return {'built': [], 'days': 0}
  first = min(by_day)
  d0 = date.fromisoformat(first)
  d1 = date.fromisoformat(ctx['today'])
  built, rows, prev = [], [], None
  packs: dict = {}
  d = d0
  while d <= d1:
    ds = d.isoformat()
    t0, t1 = day_bounds(ds, tz)
    parts = by_day.get(ds, [])
    src = {'v': VERSION, 'parts': _sig(parts), 'today': ds == ctx['today'], 'ref': ref,
           'sessions': _sig([s for s in sessions if t0 <= s['start'] < t1]),
           'events': _sig([e for e in ctx['events'] if t0 <= e.get('t', 0) < t1]),
           'rx': _sig([r for r in ctx['rx'] if t0 <= r.get('t', 0) < t1]),
           'power': _sig([r['t'] for r in ctx['power'] if t0 <= r['t'] < t1]),
           'prevPack': prev.get('packId') if prev else None}
    sig = _sig(src)
    path = os.path.join(out_dir, f'{ds}.json')
    old = None if force else _read_json(path)
    if old and old.get('sig') == sig:
      snap = old
    else:
      snap = build_day(ds, parts, tz, ctx)
      pid, why = assign_pack(prev, snap)
      snap['packId'], snap['packWhy'] = pid, why
      snap['sig'] = sig
      _write_json(path, snap)
      built.append(ds)
    cnt = (snap.get('day') or {}).get('energy', {}).get('counters', {}) if snap.get('day') else {}
    if snap.get('packId'):
      packs.setdefault(snap['packId'], {'packId': snap['packId'], 'from': ds, 'to': ds})['to'] = ds
      odo, dis = cnt.get('batteryOdometerMi'), cnt.get('kwhDischargeTotal')
      prev = {'packId': snap['packId'], 'packOdo': odo['last'] if odo else (prev or {}).get('packOdo'),
              'packDis': dis['last'] if dis else (prev or {}).get('packDis')}
    rows.append(index_row(snap))
    d += timedelta(days=1)
  _write_json(os.path.join(out_dir, 'index.json'), {'tz': str(tz.key), 'generatedAt': round(time.time(), 1), 'days': rows,
                                                    'packs': list(packs.values())})
  return {'built': built, 'days': len(rows), 'firstRaw': first_raw}


def index_row(s: dict) -> dict:
  """The few figures a day contributes to the list and the trend charts, each with its confidence."""
  day = s.get('day') or {}
  cb = day.get('cellBalance') or {}
  r = (day.get('resistance') or {}).get('headline')
  nfp = day.get('nominalFullPack')
  tmax = (day.get('temps') or {}).get('moduleTempMax')
  en = day.get('energy') or {}
  miles = en.get('odometerMi')
  if miles is None and en.get('drives'):
    miles = en['driveSessionsMi']
  warn = [a['label'] for a in day.get('anomalies', [])]
  return {'date': s['date'], 'status': s['status'], 'statusWhy': s.get('statusWhy', []), 'packId': s.get('packId'),
          'sources': s.get('sources', []), 'recordedMin': (s.get('coverage') or {}).get('recordedMin'),
          'nominalFullPack': nfp['max'] if nfp else None, 'soh': (day.get('soh') or {}).get('value'),
          'cellDeltaMv': cb.get('medianMv') if cb.get('judged') else None,
          'cellDeltaLow': cb.get('medianMv') if cb.get('n') and not cb.get('judged') else None,
          'rMohm': r['mOhm'] if r else None, 'rBin': r['bin'] if r else None, 'rConf': r['conf'] if r else None,
          'rBins': {b['bin']: [b['mOhm'], b['conf'], b['n']] for b in (day.get('resistance') or {}).get('bins', [])},
          'tempMax': tmax['max']['v'] if tmax else None, 'tempMin': ((day.get('temps') or {}).get('moduleTempMin') or {}).get('min', {}).get('v'),
          'miles': miles, 'milesSource': 'odometer' if en.get('odometerMi') is not None else 'sessions' if miles else None,
          'chargedKwh': en.get('chargeSessionsKwh') if day.get('charges') else None,
          'charges': len(day.get('charges') or []), 'warnings': warn,
          'unverified': len(day.get('unverified') or [])}


# ---- trend ---------------------------------------------------------------------------------------------
TREND_METRICS = ('nominalFullPack', 'soh', 'cellDeltaMv', 'rMohm', 'tempMax', 'miles', 'chargedKwh')
WHOLE_DAY_METRICS = ('miles', 'chargedKwh')   # a partial day's figure covers only what was recorded


def trend(rows: list, pack: str | None = None) -> dict:
  """Per metric over one pack's days: last value, change from the day before (only when the day before
  has a value: no carrying across gaps), 7 and 30 day medians, and the slope per 30 days by least
  squares over at least 5 days spanning a week. Resistance uses the pack's most measured SOC/temp bin
  only, so every point compared was taken under the same conditions."""
  rows = [r for r in rows if pack is None or r.get('packId') == pack]
  out: dict = {}
  if not rows:
    return out
  bins: dict = {}
  for r in rows:
    for b, (_, conf, n) in (r.get('rBins') or {}).items():
      if conf in ('high', 'medium'):
        bins[b] = bins.get(b, 0) + n
  ref_bin = max(bins, key=bins.get) if bins else None
  last_day = date.fromisoformat(rows[-1]['date'])
  for k in TREND_METRICS:
    pts = []
    for r in rows:
      if k == 'rMohm':
        v = (r.get('rBins') or {}).get(ref_bin)
        v = v[0] if v and v[1] in ('high', 'medium') else None
      elif k in WHOLE_DAY_METRICS and r.get('status') != 'OK':
        v = None
      else:
        v = r.get(k)
      if v is not None:
        pts.append((date.fromisoformat(r['date']), v))
    m = {'n': len(pts), 'last': pts[-1][1] if pts else None, 'lastDate': pts[-1][0].isoformat() if pts else None}
    if k in WHOLE_DAY_METRICS:
      m['note'] = '완전히 기록된 날만'
    if len(pts) >= 2 and (pts[-1][0] - pts[-2][0]).days == 1:
      m['dayChange'] = round(pts[-1][1] - pts[-2][1], 3)
    else:
      m['dayChange'] = None
    for w in (7, 30):
      win = [v for dd, v in pts if (last_day - dd).days < w]
      m[f'median{w}'] = round(statistics.median(win), 3) if win else None
    if len(pts) >= 5 and (pts[-1][0] - pts[0][0]).days >= 7:
      xs = [(dd - pts[0][0]).days for dd, _ in pts]
      ys = [v for _, v in pts]
      mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
      sxx = sum((x - mx) ** 2 for x in xs)
      m['slope30'] = round(sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True)) / sxx * 30, 4) if sxx else None
    else:
      m['slope30'] = None
      m['slopeWhy'] = '5일 이상, 7일 넘는 기간이 필요'
    out[k] = m
  out['rMohm']['bin'] = ref_bin
  return out


def index(out_dir: str = DAILY_DIR) -> dict:
  return _read_json(os.path.join(out_dir, 'index.json')) or {'days': [], 'packs': []}


def day(d: str, out_dir: str = DAILY_DIR) -> dict | None:
  if not (len(d) == 10 and d[4] == '-' and d[7] == '-' and d.replace('-', '').isdigit()):
    return None
  return _read_json(os.path.join(out_dir, f'{d}.json'))


if __name__ == '__main__':
  import sys
  # python -m openpilot.selfdrive.debug.daily [rebuild]   (run as the page's user, not root)
  r = ensure(force=len(sys.argv) > 1 and sys.argv[1] == 'rebuild')
  print(json.dumps({**r, 'built': len(r['built'])}))


class Runner:
  """Keeps the store current from the page server: once shortly after start, then every EVERY_S, and
  on request. One build at a time."""
  EVERY_S = 600.0
  START_DELAY_S = 30.0

  def __init__(self):
    import threading
    self.lock = threading.Lock()
    self.building = False
    self.last: dict | None = None
    self.error = ''
    self._threading = threading

  def run_once(self, force: bool = False) -> dict:
    with self.lock:
      self.building = True
      try:
        self.last = ensure(force=force)
        self.error = ''
      except Exception as e:     # a bad file must not stop the page server's thread
        self.error = f'{type(e).__name__}: {e}'
      finally:
        self.building = False
    return {'built': len((self.last or {}).get('built', [])), 'error': self.error}

  def start(self) -> None:
    def loop():
      time.sleep(self.START_DELAY_S)
      while True:
        self.run_once()
        time.sleep(self.EVERY_S)
    self._threading.Thread(target=loop, daemon=True, name='daily').start()
