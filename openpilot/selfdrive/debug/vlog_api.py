"""What the /vlog page shows: btobdd's always-on vehicle log, plus a re-analysis of the logs that
existed before it (live.jsonl through the same session logic, swaglog STATUS_PACKETs for power).

Read-only over the files; nothing here talks to the daemon except reading its tmpfs state.
"""
from __future__ import annotations

import collections
import csv
import gzip
import io
import json
import os
import threading
import time

from openpilot.system.btobd import tdiag, vlog

VLOG_DIR = os.path.join(tdiag.LOG_DIR, 'vlog')
SWAGLOG_DIR = '/data/log'
OBD_STATE_PATH = '/dev/shm/btobd.json'
POWER_DAYS = 14
REALDATA = '/data/media/0/realdata'
# Not under VLOG_DIR: btobdd runs as root and owns that directory, this server runs as comma. With the
# cache there (6c4adb819) every write failed, so every page poll re-read 14 days of qlogs (~6 s of CPU
# per route) and drives were never merged.
ROUTES_CACHE = os.path.join(tdiag.LOG_DIR, 'routes-cache.json')
ROUTES_EVERY_S = 120.0
_cache: dict = {}
_routes_lock = threading.Lock()
_routes_busy = False
_routes_next = 0.0


def _mtime(path: str) -> float:
  try:
    return os.path.getmtime(path)
  except OSError:
    return 0.0


def _swag_stamp() -> tuple:
  try:
    names = os.listdir(SWAGLOG_DIR)
  except OSError:
    return (0, '')
  return (len(names), max(names) if names else '')


# The two re-analyses below only matter for the time before the always-on logger's own records begin,
# so once that start is known their result never changes: it is kept on disk keyed by it. Before,
# each server start re-read live.jsonl (16 MB, 2.2 s on the device) and 27 MB of swaglog (0.7 s), and
# again whenever either grew -- live.jsonl grows the whole time the dongle is linked.
DISK_CACHE = os.path.join(tdiag.LOG_DIR, 'vlog-history-cache.json')


def _disk_cached(name: str, key, fn):
  hit = _cache.get(name)
  if hit and hit[0] == key:
    return hit[1]
  try:
    with open(DISK_CACHE) as f:
      disk = json.load(f)
  except (OSError, ValueError):
    disk = {}
  if disk.get(name, {}).get('key') == key:
    val = disk[name]['val']
  else:
    val = fn()
    disk[name] = {'key': key, 'val': val}
    try:
      with open(DISK_CACHE + '.tmp', 'w') as f:
        json.dump(disk, f, ensure_ascii=False)
      os.replace(DISK_CACHE + '.tmp', DISK_CACHE)
    except OSError:
      pass
  _cache[name] = (key, val)
  return val


def replayed(until: float = float('inf')) -> dict:
  """live.jsonl (+ its .1 rotation) through vlog's session logic, for drives/charges logged before
  the always-on logger existed (sessions ending before `until`, its first own session)."""
  paths = [os.path.join(tdiag.LOG_DIR, n) for n in (tdiag.LIVE_LOG + '.1', tdiag.LIVE_LOG)]

  def run():
    out = {'sessions': [], 'rows': 0}
    for p in paths:
      r = vlog.replay_live(p, until)
      out['sessions'] += [s for s in r['sessions'] if s['end'] < until]
      out['rows'] += r['rows']
    for s in out['sessions']:
      s['source'] = 'live.jsonl 재분석'
    return out
  # with no logger sessions yet the cutoff is open and the file still matters as it grows
  key = until if until != float('inf') else [_mtime(p) for p in paths]
  return _disk_cached('replay', key, run)


def power_rows() -> list:
  since = time.time() - POWER_DAYS * 86400
  own = [r for r in vlog.read_jsonl(os.path.join(VLOG_DIR, 'power.jsonl')) if r.get('t', 0) >= since]
  first_own = own[0]['t'] if own else float('inf')
  # the logger's own 1-minute record wins where it exists; swaglog fills in before it
  key = first_own if own else list(_swag_stamp())
  swag = _disk_cached('swag', key, lambda: [r for r in vlog.power_from_swaglog(SWAGLOG_DIR) if r['t'] < first_own])
  return [r for r in swag if since <= r['t'] < first_own] + own


# ---- drives from openpilot's own route logs ---------------------------------------------------------
# A dongle session covers only the minutes the dongle was linked (before the always-on logger: only
# while a page was open), so its duration and distance are a slice of the drive. openpilot records
# every drive whole; its qlogs give the real start, end and distance (carState.vEgo integrated).
def _route_summary(paths: list) -> dict | None:
  """Start, end and distance of one route. Times come from one wall-minus-monotonic offset for the
  whole route, taken from the LAST segment's clocks (or GPS) message: the device boots with a stale
  clock (2026-07-28) and loggerd's initData keeps it for the whole route, so 0000021a, a 10/28
  drive, read as July. monotonic time runs straight through a route, so one late offset fits all."""
  import zstandard
  from openpilot.cereal import log as capnp_log
  cs: list = []
  off = gps_off = init_off = None
  for p in paths:
    try:
      with open(p, 'rb') as f:
        data = zstandard.ZstdDecompressor().stream_reader(f).read()
      events = capnp_log.Event.read_multiple_bytes(data)
      for m in events:
        w = m.which()
        if w == 'carState':
          cs.append((m.logMonoTime / 1e9, m.carState.vEgo))
        elif w == 'clocks' and m.clocks.wallTimeNanos > 0:
          off = m.clocks.wallTimeNanos / 1e9 - m.logMonoTime / 1e9
        elif w == 'gpsLocationExternal' and m.gpsLocationExternal.hasFix and m.gpsLocationExternal.unixTimestampMillis > 0:
          gps_off = m.gpsLocationExternal.unixTimestampMillis / 1000 - m.logMonoTime / 1e9
        elif w == 'initData' and init_off is None:
          init_off = m.initData.wallTimeNanos / 1e9 - m.logMonoTime / 1e9
    except Exception:
      continue   # a segment cut by power loss ends mid-message
  # clocks is system time; before NTP/GPS it is the stale boot time, so prefer GPS when they disagree
  if off is None or (gps_off is not None and abs(off - gps_off) > 120):
    off = gps_off if gps_off is not None else (off if off is not None else init_off)
  if not cs or off is None:
    return None
  dist = moving = 0.0
  for (t0, v0), (t1, _) in zip(cs, cs[1:], strict=False):
    dt = t1 - t0
    if 0 < dt < 5:
      dist += v0 * dt
      if v0 > 0.5:
        moving += dt
  return {'start': round(cs[0][0] + off, 1), 'end': round(cs[-1][0] + off, 1), 'miles': round(dist / 1609.344, 2),
          'movingS': round(moving), 'clock': 'gps' if off == gps_off else 'system'}


def _route_dirs(days: float) -> dict:
  since = time.time() - days * 86400
  routes: dict = {}
  try:
    entries = os.listdir(REALDATA)
  except OSError:
    return routes
  for e in entries:
    q = os.path.join(REALDATA, e, 'qlog.zst')
    if '--' not in e or not os.path.isfile(q):
      continue
    mt = os.path.getmtime(q)
    if mt < since:
      continue
    name, seg = e.rsplit('--', 1)
    if seg.isdigit():
      routes.setdefault(name, []).append((int(seg), q, mt))
  return routes


def _load_routes() -> dict:
  try:
    with open(ROUTES_CACHE) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def _refresh_routes(days: float) -> None:
  global _routes_busy
  path = ROUTES_CACHE
  cache = _load_routes()
  try:
    for name, segs in _route_dirs(days).items():
      segs.sort()
      key = f'{len(segs)}:{int(max(s[2] for s in segs))}'
      if cache.get(name, {}).get('key') == key:
        continue
      # a route still being written is summarised again once it stops growing
      summ = _route_summary([s[1] for s in segs])
      cache[name] = {'key': key, 'segments': len(segs), **(summ or {'start': None})}
      with open(path + '.tmp', 'w') as f:
        json.dump(cache, f)
      os.replace(path + '.tmp', path)
  except Exception as e:
    print(f'vlog_api: route summaries failed: {type(e).__name__}: {e}', flush=True)
  finally:
    with _routes_lock:
      _routes_busy = False


def drives(days: float = POWER_DAYS) -> list:
  """openpilot drives on this device, from a cache filled in the background."""
  global _routes_busy, _routes_next
  with _routes_lock:
    if not _routes_busy and time.monotonic() >= _routes_next:
      _routes_busy, _routes_next = True, time.monotonic() + ROUTES_EVERY_S
      threading.Thread(target=_refresh_routes, args=(days,), daemon=True).start()
  cache = _load_routes()
  out = [{'route': n, **{k: v for k, v in r.items() if k != 'key'}} for n, r in cache.items() if r.get('start')]
  return sorted(out, key=lambda r: r['start'])


def merge_drives(sessions: list, routes: list) -> list:
  """One row per drive: the route's real start/end/distance, with the dongle sessions that fall in
  it as parts. Energy is summed over the parts and labelled as covering only that share of the
  drive; Wh/mi is only given when the parts cover most of it. Drives no dongle heard are left out."""
  rows, used = [], set()
  for r in routes:
    span = max(1.0, r['end'] - r['start'])
    parts = [s for s in sessions if s['type'] == 'DRIVE' and s['start'] < r['end'] + 60 and s['end'] > r['start'] - 60]
    if not parts:
      continue
    used.update(id(s) for s in parts)
    covered = sum(min(s['end'], r['end']) - max(s['start'], r['start']) for s in parts)
    row = {'type': 'DRIVE', 'start': r['start'], 'end': r['end'], 'durationS': round(span, 1), 'miles': r['miles'],
           'milesSource': 'openpilot', 'movingS': r['movingS'], 'route': r['route'], 'coverage': round(max(0.0, covered) / span, 3),
           'source': 'openpilot + 동글', 'parts': len(parts), 'flags': []}
    for k in ('kwhIn', 'kwhOut', 'gapS', 'samples'):
      row[k] = round(sum(s.get(k) or 0 for s in parts), 3)
    kws = [s for s in parts if s.get('kwMin') is not None]
    row['kwMin'] = min((s['kwMin'] for s in kws), default=None)
    row['kwMax'] = max((s['kwMax'] for s in kws), default=None)
    row['avgKw'] = round(sum((s.get('avgKw') or 0) * s['durationS'] for s in parts) / max(1, sum(s['durationS'] for s in parts)), 2)
    row['socStart'], row['socEnd'] = parts[0].get('socStart'), parts[-1].get('socEnd')
    row['missingRate'] = None
    part_mi = sum(s.get('miSpeed') or 0 for s in parts)
    row['partMiles'] = round(part_mi, 2)
    net = row['kwhOut'] - row['kwhIn']
    if part_mi > 0.3:
      wh = round(net * 1000 / part_mi, 1)
      row['whPerMi' if row['coverage'] >= 0.8 else 'whPerMiPart'] = wh
    row['regenKwh'] = row['kwhIn']
    for k in ('tempMin', 'limitMinDischarge', 'limitMinRegen'):
      vals = [s[k] for s in parts if s.get(k) is not None]
      row[k] = min(vals) if vals else None
    vals = [s['tempMax'] for s in parts if s.get('tempMax') is not None]
    row['tempMax'] = max(vals) if vals else None
    row['events'] = [e for s in parts for e in s.get('events') or []]
    row['partList'] = [{'start': s['start'], 'end': s['end'], 'source': s.get('source')} for s in parts]
    if row['coverage'] < 0.8:
      row['flags'].append('partial')
    rows.append(row)
  rest = [s for s in sessions if id(s) not in used]
  return rows + rest


def merge_charges(sessions: list, join_s: float = vlog.CHARGE_JOIN_S) -> list:
  """Charge sessions less than join_s apart are one plug-in split by a link drop or a restart (and,
  before 2026-10-30, by the AC/unknown flip-flop): sum them into one row. Input sorted by start."""
  out: list = []
  for s in sessions:
    p = out[-1] if out else None
    if not (p and s['type'] in vlog.CHARGE_MODES and p['type'] in vlog.CHARGE_MODES and 0 <= s['start'] - p['end'] <= join_s):
      out.append(s)
      continue
    m = dict(p)
    m['parts'] = (p.get('parts') or 1) + 1
    if m['type'] == 'CHARGE_UNKNOWN':
      m['type'] = s['type']
    m['end'] = s['end']
    span = m['end'] - m['start']
    m['durationS'] = round(span, 1)
    for k in ('kwhIn', 'kwhOut', 'samples', 'coveredS'):
      m[k] = round((p.get(k) or 0) + (s.get(k) or 0), 3)
    m['gapS'] = round(max(0.0, span - m['coveredS']), 1)
    m['missingRate'] = round(m['gapS'] / span, 3) if span > 0 else None
    for k, fn in (('kwMax', max), ('kwMin', min), ('tempMax', max), ('tempMin', min), ('cellDeltaSyncMax', max)):
      vals = [x[k] for x in (p, s) if x.get(k) is not None]
      m[k] = fn(vals) if vals else None
    n = (p.get('kwN') or 0) + (s.get('kwN') or 0)
    if n:
      m['avgKw'] = round(((p.get('avgKw') or 0) * (p.get('kwN') or 0) + (s.get('avgKw') or 0) * (s.get('kwN') or 0)) / n, 2)
    m['kwN'] = n
    if m.get('socStart') is None:
      m['socStart'] = s.get('socStart')
    if s.get('socEnd') is not None:
      m['socEnd'] = s['socEnd']
    m['curve'] = {**(p.get('curve') or {}), **(s.get('curve') or {})}
    m['socPctS'] = {k: round((p.get('socPctS') or {}).get(k, 0) + (s.get('socPctS') or {}).get(k, 0), 1)
                    for k in {*(p.get('socPctS') or {}), *(s.get('socPctS') or {})}}
    if m.get('tempStart') is None:
      m['tempStart'] = s.get('tempStart')
    if s.get('tempEnd') is not None:
      m['tempEnd'] = s['tempEnd']
    # how the plug-in ended is how its last part ended
    for k in ('endCause', 'endWhy', 'endT', 'endAfterS'):
      m.pop(k, None)
      if k in s:
        m[k] = s[k]
    m['events'] = (p.get('events') or []) + (s.get('events') or [])
    m['flags'] = sorted(set((p.get('flags') or []) + (s.get('flags') or [])) - {'interrupted'}) + ['merged']
    out[-1] = m
  return out


ANALYSIS_LO, ANALYSIS_HI = 10, 80


def charge_analysis(s: dict) -> dict | None:
  """10->80% for one charge: covered seconds of every whole percent this charge passed through,
  and the rest of 10-79 filled in from this charge's own median seconds per percent. The first and
  last percent seen are partial and left out. AC charges at a near-constant power, so the fill is a
  fair estimate there; DC power falls with SOC, so a DC estimate is only given with 80% of the range
  measured."""
  pct = {int(k): v for k, v in (s.get('socPctS') or {}).items()}
  if len(pct) < 3:
    return None
  lo, hi = min(pct), max(pct)
  whole = {k: v for k, v in pct.items() if lo < k < hi and v > 0}
  if not whole:
    return None
  span = range(ANALYSIS_LO, ANALYSIS_HI)
  meas = {k: whole[k] for k in span if k in whole}
  per = sorted(whole.values())
  med = per[len(per) // 2]
  missing = [k for k in span if k not in whole]
  out = {'socLo': lo, 'socHi': hi, 'measuredPct': len(meas), 'measuredS': round(sum(meas.values())),
         'secPerPct': round(med, 1), 'missingPct': len(missing)}
  if s.get('kwhIn') and s.get('socEnd') is not None and s.get('socStart') is not None and s['socEnd'] - s['socStart'] >= 3:
    out['kwhPerPct'] = round(s['kwhIn'] / (s['socEnd'] - s['socStart']), 3)
  share = len(meas) / len(span)
  if s['type'] == 'DC_CHARGE' and share < 0.8:
    out.update(totalS=None, basis='none', note='급속은 SOC에 따라 전력이 크게 바뀌어 측정 구간이 80% 미만이면 추정하지 않음')
    return out
  out['totalS'] = round(sum(meas.values()) + len(missing) * med)
  out['basis'] = 'measured' if not missing else 'estimated'
  out['confidence'] = 'high' if share >= 0.9 else 'medium' if share >= 0.3 else 'low'
  if missing:
    out['note'] = f'측정 {len(meas)}% + 미측정 {len(missing)}%는 이 충전의 %당 중앙 {med / 60:.1f}분으로 추정'
  return out


def charge_check(segs: list, sessions: list) -> dict:
  """Was the device up while the car charged? Charging is taken from logged charge sessions, and
  from AWAKE stretches (DC-DC up, not driving) of 30 min or more, which is what a charge looks like
  on the 12 V side when no dongle was listening."""
  awake = [s for s in segs if s['state'] == 'AWAKE' and s['end'] - s['start'] >= 1800]
  charges = [s for s in sessions if s['type'] in vlog.CHARGE_MODES]
  off = [s for s in segs if s['state'] == 'DEVICE_OFF']

  def covered(a, b):
    return not any(o['start'] < b and o['end'] > a for o in off)
  return {'awakeLong': [{'start': s['start'], 'end': s['end'], 'vMax': s['vMax'], 'deviceUp': covered(s['start'], s['end'])}
                        for s in awake],
          'chargeSessions': [{'start': s['start'], 'end': s['end'], 'deviceUp': True} for s in charges],
          'deviceOff': [{'start': s['start'], 'end': s['end']} for s in off]}


def _status() -> dict:
  try:
    with open(OBD_STATE_PATH) as f:
      st = json.load(f)
  except (OSError, ValueError):
    return {'stale': True}
  v = st.get('values') or {}
  return {'stale': time.time() - st.get('updated', 0) > 10, 'link': st.get('link'), 'bt': st.get('bt'),
          'mode': st.get('mode'), 'modeSince': st.get('modeSince'), 'wanted': st.get('wanted'),
          'power': st.get('power') or {}, 'quality': st.get('quality') or {}, 'error': st.get('error', ''),
          'lastFrame': max((r.get('t', 0) for r in (st.get('raw') or {}).values()), default=None),
          'values': {k: v.get(k) for k in ('packVoltage', 'packCurrent', 'packPower', 'socUI', 'socDisplay', 'gear',
                                           'vehicleSpeedMph', 'cellDelta', 'cellSync', 'cellSpanS', 'moduleTempMin',
                                           'moduleTempMax', 'bmsState', 'nominalFullPackEnergy')},
          'age': {k: (st.get('age') or {}).get(k) for k in ('packVoltage', 'socUI', 'cellDelta', 'bmsState')}}


def now() -> dict:
  return {'now': time.time(), 'status': _status()}


def overview() -> dict:
  own = vlog.read_jsonl(os.path.join(VLOG_DIR, 'sessions.jsonl'))
  for s in own:
    s['source'] = '상시 로거' + (' (raw 재구성)' if s.get('rebuilt') else '')
  first_own = min((s['start'] for s in own), default=float('inf'))
  rep = replayed(first_own)
  sessions = [s for s in rep['sessions'] if s['end'] < first_own] + own
  # a session that straddled the boot-time clock being set runs backwards; it has no usable times
  sessions = [s for s in sessions if s.get('end', 0) >= s.get('start', 0)]
  sessions.sort(key=lambda s: s['start'])
  sessions = merge_charges(sessions)
  for x in sessions:
    if x.get('type') in vlog.CHARGE_MODES:
      x['analysis'] = charge_analysis(x)
  routes = drives()
  sessions = merge_drives(sessions, routes)
  sessions.sort(key=lambda s: s['start'], reverse=True)
  events = vlog.read_jsonl(os.path.join(VLOG_DIR, 'events.jsonl'), 300)
  events = [{k: e.get(k) for k in ('t', 'type', 'mode', 'value', 'threshold')} for e in events][::-1]
  health = vlog.read_jsonl(os.path.join(VLOG_DIR, 'health.jsonl'))
  rows = power_rows()
  segs = vlog.power_segments(rows)
  return {'now': time.time(), 'status': _status(), 'sessions': sessions[:300], 'events': events, 'health': health,
          'power': segs, 'powerFrom': rows[0]['t'] if rows else None, 'replayRows': rep['rows'],
          'chargeCheck': charge_check(segs, sessions)}


def series(start: float, end: float, limit: int = 1500) -> list:
  """Samples between start and end: the logger's series files, else the old live.jsonl lines."""
  out = []
  day = start - start % 86400
  while day <= end:
    name = 'series-' + time.strftime('%Y%m%d', time.gmtime(day)) + '.jsonl'
    out += [r for r in vlog.read_jsonl(os.path.join(VLOG_DIR, name)) if start <= r.get('t', 0) <= end]
    day += 86400
  if not out:
    from openpilot.system.btobd.tesla_sx import bms_hv
    for p in (tdiag.LIVE_LOG + '.1', tdiag.LIVE_LOG):
      for r in vlog.read_jsonl(os.path.join(tdiag.LOG_DIR, p)):
        if r.get('event') != 'live' or not start <= r.get('t', 0) <= end:
          continue
        v = dict(r.get('values') or {})
        raw = (r.get('raw') or {}).get('102')
        if raw:
          try:
            v.update(bms_hv(bytes.fromhex(raw.replace(' ', '')), offset=None))
          except ValueError:
            pass
        out.append({'t': r['t'], **{k: v[k] for k in vlog.SERIES_KEYS if k in v and k != 'cellDelta'}})
  out.sort(key=lambda r: r['t'])
  step = max(1, len(out) // limit)
  return out[::step]


def event_detail(t: float) -> dict | None:
  for e in vlog.read_jsonl(os.path.join(VLOG_DIR, 'events.jsonl')):
    if abs(e.get('t', 0) - t) < 0.01:
      return e
  return None


def _csv(rows: list, cols: list) -> bytes:
  buf = io.StringIO()
  w = csv.writer(buf)
  w.writerow(cols)
  for r in rows:
    w.writerow(['' if r.get(c) is None else r.get(c) for c in cols])
  return buf.getvalue().encode()


SESSION_COLS = ['source', 'type', 'route', 'start', 'end', 'durationS', 'movingS', 'coverage', 'parts', 'socStart', 'socEnd',
                'kwhIn', 'kwhOut', 'avgKw', 'kwMax', 'kwMin', 'miles', 'milesSource', 'partMiles', 'whPerMi', 'whPerMiPart', 'regenKwh', 'min10to80', 'min20to80', 'tempMin', 'tempMax',
                'cellDeltaSyncMax', 'limitMinDischarge', 'limitMinRegen', 'samples', 'gapS', 'missingRate', 'staleRate', 'flags']


def export(kind: str, arg: str = '') -> tuple[bytes, str] | None:
  """CSV for sessions/health/series/power. Times are unix seconds (UTC); units are in the headers'
  page legend: V, A (+ into pack), kW (+ into pack), kWh, %, mph, degC."""
  if kind == 'sessions':
    rows = overview()['sessions']
    for r in rows:
      r['flags'] = ';'.join(r.get('flags') or [])
    return _csv(rows, SESSION_COLS), 'vlog-sessions.csv'
  if kind == 'health':
    rows = vlog.read_jsonl(os.path.join(VLOG_DIR, 'health.jsonl'))
    cols = ['t', 'soc', 'packVoltage', 'packCurrent', 'cellsSumV', 'sumCheck', 'cellMin', 'cellMax', 'deltaMv', 'lowIndex',
            'nominalFullPackEnergy', 'nominalEnergyRemaining', 'energyBuffer', 'kwhChargeTotal', 'kwhDischargeTotal',
            'batteryOdometerMi', 'moduleTempMin', 'moduleTempMax'] + [f'cell{i + 1}' for i in range(96)]
    for r in rows:
      for i, c in enumerate(r.get('cells') or []):
        r[f'cell{i + 1}'] = c
    return _csv(rows, cols), 'vlog-health.csv'
  if kind == 'series':
    try:
      start, end = (float(x) for x in arg.split('-', 1))
    except ValueError:
      return None
    rows = series(start, end, limit=10 ** 7)
    return _csv(rows, ['t', 'mode'] + list(vlog.SERIES_KEYS)), f'vlog-series-{int(start)}.csv'
  if kind == 'power':
    rows = power_rows()
    for r in rows:
      r['state'] = vlog.power_state(r)
    return _csv(rows, ['t', 'state', 'v', 'harness', 'ign', 'started', 'src']), 'vlog-power.csv'
  if kind == 'raw':
    # one hour of raw frames, as stored (gzip of JSON lines: wall, mono, id, data, window)
    p = os.path.join(VLOG_DIR, f'raw-{arg}.jsonl.gz')
    if not arg.replace('-', '').isdigit() or not os.path.exists(p):
      return None
    with gzip.open(p, 'rt') as f:
      rows = [json.loads(ln) for ln in f]
    return _csv([dict(zip(['wall', 'mono', 'id', 'data', 'window'], r, strict=False)) for r in rows],
                ['wall', 'mono', 'id', 'data', 'window']), f'vlog-raw-{arg}.csv'
  return None


def raw_files() -> list:
  try:
    return sorted((n[4:-9] for n in os.listdir(VLOG_DIR) if n.startswith('raw-') and n.endswith('.jsonl.gz')), reverse=True)
  except OSError:
    return []


SUMMARY_HOURS = 6


def recent_summary(hours: float = SUMMARY_HOURS) -> dict | None:
  """The /diag charge/thermal/ident card from the always-on logger's raw frames: the last `hours` of
  logging, ending at the last frame heard (the car may have slept since). Replaces the one-off
  capture it used to read, which went stale as soon as it was taken."""
  from openpilot.system.btobd import tesla_pt
  names = sorted(raw_files())          # newest first, then sorted oldest first
  if not names:
    return None
  paths = [os.path.join(VLOG_DIR, f'raw-{n}.jsonl.gz') for n in names[-int(hours) - 2:]]
  key = [(p, _mtime(p)) for p in paths]
  hit = _cache.get('summary')
  if hit and hit[0] == key:
    return hit[1]
  frames = []
  for p in paths:
    try:
      with gzip.open(p, 'rt') as f:
        lines = f.readlines()
    except (OSError, EOFError):
      continue
    for ln in lines:
      try:
        wall, _, fid, data, _ = json.loads(ln)
        frames.append((wall, int(fid, 16), bytes.fromhex(data)))
      except (ValueError, TypeError):
        continue
  if not frames:
    return None
  frames.sort(key=lambda f: f[0])
  end = frames[-1][0]
  frames = [f for f in frames if f[0] >= end - hours * 3600]
  out = tesla_pt.summarize(frames, {'saved': end, 'seconds': round(end - frames[0][0]), 'clockRef': '수신 시각 기준'})
  out.update(name='vlog', source='vlog', start=frames[0][0], end=end)
  _cache['summary'] = (key, out)
  return out


# ---- /diag without a live link ---------------------------------------------------------------------
STATE_WINDOW_S = 1800.0   # 0x562 (odometer) comes every ~10 s and is read in rotation
PARAMS_DIR = '/data/params/d'


def _raw_frames(paths: list) -> list:
  """(wall, mono, id, data) of raw files, in file order."""
  out = []
  for p in paths:
    try:
      with gzip.open(p, 'rt') as f:
        lines = f.readlines()
    except (OSError, EOFError):
      continue
    for ln in lines:
      try:
        wall, mono, fid, data, _ = json.loads(ln)
        out.append((wall, mono, int(fid, 16), bytes.fromhex(data)))
      except (ValueError, TypeError):
        continue
  return out


def last_state() -> dict | None:
  """What /diag shows when the dongle is not linked: the car as it was in the last STATE_WINDOW_S of
  logging, decoded again from raw. Ages are counted back from the last frame, not from now."""
  from openpilot.system.btobd import tesla_pt
  from openpilot.system.btobd.tesla_sx import TeslaSXState
  names = sorted(raw_files())
  if not names:
    return None
  paths = [os.path.join(VLOG_DIR, f'raw-{n}.jsonl.gz') for n in names[-2:]]
  key = [(p, _mtime(p)) for p in paths]
  hit = _cache.get('state')
  if hit and hit[0] == key:
    return hit[1]
  frames = _raw_frames(paths)
  if not frames:
    return None
  end_wall, end_mono = frames[-1][0], frames[-1][1]
  car = TeslaSXState()
  for wall, mono, i, d in frames:
    if wall < end_wall - STATE_WINDOW_S or mono > end_mono:
      continue
    car.feed(i, d, mono)
    if i in tesla_pt.LIVE_IDS:
      tesla_pt.feed_live(car, i, d, mono)
  snap = car.snapshot(end_mono)
  out = {'values': snap['values'], 'age': snap['age'], 'cellsV': snap['cellsV'], 'cellsT': snap['cellsT'], 'asOf': end_wall}
  _cache['state'] = (key, out)
  return out


# ---- reception (rx.json / rx.jsonl, written by btobdd) ----------------------------------------------
RX_HISTORY_S = 24 * 3600


def rx_state() -> dict:
  """btobdd's last per-ID / per-signal reception summary with labels, plus the last 24 h of hourly
  per-ID counts. 'liveS' is how long ago btobdd wrote it: more than ~30 s and it is the state the
  logger was in when the link last stopped, not now."""
  from openpilot.system.btobd import tesla_pt
  try:
    with open(os.path.join(VLOG_DIR, 'rx.json')) as f:
      cur = json.load(f)
  except (OSError, ValueError):
    cur = {'t': None, 'ids': [], 'signals': []}
  labels = {k: lb for _, k, lb, *_ in CHECKS}
  labels.update({k: s.label for k, s in tesla_pt.SIG_BY_KEY.items()})
  for x in cur.get('signals', []):
    x['label'] = labels.get(x['key'], x['key'])
  since = time.time() - RX_HISTORY_S
  hist: dict = {}
  for r in vlog.read_jsonl(os.path.join(VLOG_DIR, 'rx.jsonl')):
    if r.get('t', 0) < since:
      continue
    h = hist.setdefault(r['id'], {'polls': 0, 'hits': 0, 'gapMax': 0.0, 'maxMiss': 0, 'hours': 0})
    h['polls'] += r['polls']
    h['hits'] += r['hits']
    h['gapMax'] = max(h['gapMax'], r.get('gapMax') or 0)
    h['maxMiss'] = max(h['maxMiss'], r.get('maxMiss') or 0)
    h['hours'] += 1
  for x in cur.get('ids', []):
    h = hist.get(x['id'])
    if h and h['polls']:
      x['day'] = {**h, 'missRate': round(1 - h['hits'] / h['polls'], 3)}
  count: dict = {}
  for x in cur.get('signals', []):
    count[x['status']] = count.get(x['status'], 0) + 1
  return {**cur, 'liveS': round(time.time() - cur['t'], 1) if cur.get('t') else None, 'count': count}


# ---- checks over the log -----------------------------------------------------------------------------
# Every frame of the raw log is decoded again and each signal's min/max kept (with when), per raw hour
# file, cached on disk by the file's mtime: a closed hour is decoded once. Derived values are taken
# only in the condition they mean something in (charging power while the BMS says charging, the
# discharge limit only above 20% and 10 C, cell spread only from a synchronised sweep, ...).
CHECK_DAYS = 14
STATS_CACHE = os.path.join(tdiag.LOG_DIR, 'vlog-stats-cache.json')
STATS_VERSION = 3           # bump when file_stats derives something new: cached hours are decoded again
BMS_FAULT = (7, 8)
MIN_N = 3                   # fewer samples than this and a check is "표본 부족", never 정상
# Pack resistance: dV/dI between two 0x102 readings at most R_WINDOW_S apart whose current differs by
# R_MIN_STEP_A. A single pair is noisy (58-87 mOhm on 2026-10-30), so one median per hour file is kept,
# with what it was measured from and how far to trust it (_r_record). Only medium/high hours feed the
# check row; the others are listed with their reason.
R_WINDOW_S = 10.0
R_MIN_STEP_A = 60.0
R_MIN_PAIRS = 15
R_HIGH_PAIRS = 40
# Decoder cross-checks, repeated on every logged hour (tesla_pt.BMS_LIMITS): 0x332 brick max/min
# against a synchronised 0x6F2 sweep at rest, its temperatures against 0x6F2's module temps, 0x202's
# currents x pack voltage against 0x232's power limits while driving. A comparison outside its
# tolerance counts as a miss; a decode is confirmed with VERIFY_MIN_N comparisons and at most
# VERIFY_MAX_MISS of them missed.
VERIFY_MIN_N = 5
VERIFY_MAX_MISS = 0.1
VERIFY = {
  '332v': ('0x332 셀 최고/최저 전압', '0x6F2 동시 측정 96셀의 최고/최저 (휴지, |I| < 5 A)', 'mV', 10.0),
  '332t': ('0x332 모듈 최고/최저 온도', '0x6F2 모듈 온도 (30초 이내)', '°C', 1.5),
  '202c': ('0x202 충전·회생 허용 전류', '× 팩 전압 ÷ 0x232 회생 한계 (주행 중)', '비', 0.15),
  '202d': ('0x202 방전 허용 전류', '× 팩 전압 ÷ 0x232 방전 한계 (주행 중)', '비', 0.15),
}
VERIFY_CENTER = {'202c': 0.95, '202d': 0.95}
DCDC_EFF_MIN_W = 500        # input power is in 16 W steps: below this the ratio is +-3% noise


def _param_file(key: str) -> str:
  try:
    with open(os.path.join(PARAMS_DIR, key)) as f:
      return f.read().strip()
  except OSError:
    return ''


def file_stats(path: str) -> dict:
  """{key: [min, max, n, t_min, t_max]} for one raw file; 'bad:<sig>' keys count fault enum values."""
  from openpilot.system.btobd import tesla_pt
  from openpilot.system.btobd.tesla_sx import TeslaSXState
  car = TeslaSXState()
  st: dict = {}
  bad_sigs = [x for x in tesla_pt.LIVE_SIGS if x.bad]
  cand = {}
  for x in tesla_pt.CANDIDATES:
    cand.setdefault(x.msg, []).append(x)

  def put(k, v, t):
    if v is None or isinstance(v, (str, bool)):
      return
    r = st.get(k)
    if r is None:
      st[k] = [v, v, 1, t, t]
      return
    r[2] += 1
    if v < r[0]:
      r[0], r[3] = v, t
    if v > r[1]:
      r[1], r[4] = v, t

  def vrf(name, err, t):
    # one comparison of a decode against its reference: |error| kept as min/max, misses counted
    k = name.split(':')[0]
    put('vrf:' + name, abs(err), t)
    if abs(err - VERIFY_CENTER.get(k, 0.0)) > VERIFY[k][3]:
      r = st.setdefault('vrfmiss:' + name, [0, 0, 0, t, t])
      r[2] += 1
      r[4] = t

  win: collections.deque = collections.deque()
  rs: list = []
  for wall, mono, i, d in _raw_frames([path]):
    before = dict(car.stamp)
    car.feed(i, d, mono)
    if i in tesla_pt.LIVE_IDS:
      tesla_pt.feed_live(car, i, d, mono)
    for x in cand.get(i, ()):
      put(x.key, tesla_pt.decode(x, d), wall)
    V = car.values
    changed = [k for k, t in car.stamp.items() if before.get(k) != t]
    for k in changed:
      put(k, V[k], wall)
    if 'packCurrent' in changed and V.get('packVoltage') is not None:
      kw = V['packVoltage'] * V['packCurrent'] / 1000
      if V.get('bmsState') in (3, 4):
        put('chargeKw', kw, wall)
      elif V.get('gear') in ('D', 'R', 'N'):
        put('driveKw', kw, wall)
      win.append((wall, V['packVoltage'], V['packCurrent']))
      while win[0][0] < wall - R_WINDOW_S:
        win.popleft()
      for t0, v0, i0 in win:
        if abs(V['packCurrent'] - i0) > R_MIN_STEP_A:
          dv, di = V['packVoltage'] - v0, V['packCurrent'] - i0
          tm = (V['moduleTempMin'] + V['moduleTempMax']) / 2 if V.get('moduleTempMin') is not None and V.get('moduleTempMax') is not None else None
          rs.append((dv / di * 1000, wall, abs(dv), abs(di), wall - t0, V.get('socDisplay'), tm))
          break
    if 'dcdcInputPower' in changed and V['dcdcInputPower'] >= DCDC_EFF_MIN_W and V.get('dcdcOutputPower') is not None:
      put('dcdcEfficiency', V['dcdcOutputPower'] / V['dcdcInputPower'] * 100, wall)
    if 'pilotLow' in changed and V.get('pilot') == 2:
      put('pilotLowCharge', V['pilotLow'], wall)
    if 'rangeIdealMi' in changed and (V.get('socUI') or 0) > 30:
      put('rangeIdealFull', V['rangeIdealMi'] / V['socUI'] * 100, wall)
    if 'moduleTempMax' in changed and V.get('moduleTempMin') is not None:
      put('moduleSpread', V['moduleTempMax'] - V['moduleTempMin'], wall)
    if 'cellDelta' in changed and V.get('cellSync') and V.get('bmsState') in (3, 4):
      put('chargeDeltaMv', V['cellDelta'] * 1000, wall)
    if 'maxDischargePower' in changed and (V.get('socDisplay') or 0) > 20 and (V.get('moduleTempMin') or 0) > 10:
      put('dischargeLimitOk', V['maxDischargePower'], wall)
    for k in ('acVolts1', 'acVolts2', 'acVolts3'):
      if k in changed and V[k] > 80:
        put('acVoltsCharge', V[k], wall)
    if 'pilotHigh' in changed and V.get('pilot') == 2:
      put('pilotHighCharge', V['pilotHigh'], wall)
    if i == 0x332 and 'brickMaxV' in changed:
      cs = car.cell_set(mono)
      at = [a for a in car.cells_v_at if a is not None]
      if cs and mono - max(at) <= 10 and abs(V.get('packCurrent') if V.get('packCurrent') is not None else 99) < 5 \
         and V.get('bmsState') != 1:
        vrf('332v:max', (V['brickMaxV'] - max(cs)) * 1000, wall)
        if 'brickMinV' in changed:
          vrf('332v:min', (V['brickMinV'] - min(cs)) * 1000, wall)
    if i == 0x332 and 'bmsTempMax' in changed and car.stamp.get('moduleTempMax') is not None \
       and mono - car.stamp['moduleTempMax'] <= 30:
      vrf('332t:max', V['bmsTempMax'] - V['moduleTempMax'], wall)
      if 'bmsTempMin' in changed:
        vrf('332t:min', V['bmsTempMin'] - V['moduleTempMin'], wall)
    if i == 0x202 and V.get('bmsState') == 1 and V.get('packVoltage') and car.stamp.get('maxRegenPower') is not None \
       and mono - car.stamp['maxRegenPower'] <= 3:
      if 'bmsMaxChargeA' in changed and V['maxRegenPower'] > 10:
        vrf('202c', V['bmsMaxChargeA'] * V['packVoltage'] / 1000 / V['maxRegenPower'], wall)
      if 'bmsMaxDischargeA' in changed and V['maxDischargePower'] > 20:
        vrf('202d', V['bmsMaxDischargeA'] * V['packVoltage'] / 1000 / V['maxDischargePower'], wall)
    for x in bad_sigs:
      if x.key in changed and V[x.key] in x.bad:
        r = st.setdefault('bad:' + x.key, [V[x.key], V[x.key], 0, wall, wall])
        r[2] += 1
        r[4] = wall
  rec = _r_record(rs)
  if rec:
    st['_r'] = rec
    if rec['conf'] in ('high', 'medium'):
      st['packResistance'] = [rec['mOhm'], rec['mOhm'], 1, rec['t'], rec['t']]
  return st


def _r_record(rs: list) -> dict | None:
  """One hour's resistance: median of its pairs with the ΔV/ΔI/Δt/SOC/temperature behind it."""
  if not rs:
    return None
  import statistics
  rs = sorted(rs)
  def med(k):
    vals = [x[k] for x in rs if x[k] is not None]
    return statistics.median(vals) if vals else None
  r = rs[len(rs) // 2][0]
  q1, q3 = rs[len(rs) // 4][0], rs[3 * len(rs) // 4][0]
  spread = (q3 - q1) / r if r > 0 else None
  temp = med(6)
  why = []
  if len(rs) < R_MIN_PAIRS:
    why.append(f'쌍 {len(rs)}개 (< {R_MIN_PAIRS})')
  if spread is None or spread > 0.3:
    why.append('흩어짐 큼' if spread is not None else '음수 저항')
  if temp is None:
    why.append('온도 모름')
  if why:
    conf = 'low'
  elif len(rs) >= R_HIGH_PAIRS and spread <= 0.15 and med(3) >= 80:
    conf = 'high'
  else:
    conf = 'medium'
  def rnd(v, n):
    return None if v is None else round(v, n)
  return {'t': rs[len(rs) // 2][1], 'n': len(rs), 'mOhm': round(r, 1), 'iqr': [round(q1, 1), round(q3, 1)],
          'spread': rnd(spread, 3), 'dV': rnd(med(2), 2), 'dI': rnd(med(3), 1), 'dt': rnd(med(4), 1),
          'soc': rnd(med(5), 1), 'temp': rnd(temp, 1), 'conf': conf, 'why': why}


def _stats(days: float) -> dict:
  try:
    with open(STATS_CACHE) as f:
      disk = json.load(f)
  except (OSError, ValueError):
    disk = {}
  since = time.strftime('%Y%m%d-%H', time.gmtime(time.time() - days * 86400))
  names = [n for n in sorted(raw_files()) if n >= since]
  agg: dict = {}
  rrec: list = []
  dirty = False
  for n in names:
    p = os.path.join(VLOG_DIR, f'raw-{n}.jsonl.gz')
    mt = _mtime(p)
    hit = disk.get(n)
    if not hit or hit['mtime'] != mt or hit.get('v') != STATS_VERSION:
      hit = disk[n] = {'mtime': mt, 'v': STATS_VERSION, 'stats': file_stats(p)}
      dirty = True
    if '_r' in hit['stats']:
      rrec.append(hit['stats']['_r'])
    for k, (lo, hi, cnt, tlo, thi) in ((k, v) for k, v in hit['stats'].items() if not k.startswith('_')):
      a = agg.get(k)
      if a is None:
        agg[k] = [lo, hi, cnt, tlo, thi]
        continue
      a[2] += cnt
      if lo < a[0]:
        a[0], a[3] = lo, tlo
      if hi > a[1]:
        a[1], a[4] = hi, thi
  if dirty:
    disk = {n: v for n, v in disk.items() if n >= since}
    try:
      with open(STATS_CACHE + '.tmp', 'w') as f:
        json.dump(disk, f)
      os.replace(STATS_CACHE + '.tmp', STATS_CACHE)
    except OSError:
      pass
  return {'agg': agg, 'files': len(names), 'from': names[0] if names else None, 'resistance': rrec}


# (group, key, label, unit, digits, warn (lo, hi), bad (lo, hi), reference shown). No warn/bad = 참고:
# shown with its range, not judged. Limits are general Li-ion/Tesla figures or this car's own normal.
CHECKS = [
  ('배터리', 'packVoltage', '팩 전압', 'V', 1, (300, 403), (288, 410), '96셀 × 3.1~4.2 V'),
  ('배터리', 'packCurrent', '팩 전류', 'A', 1, None, None, '+ 충전·회생 / − 방전'),
  ('배터리', 'driveKw', '주행 중 팩 출력', 'kW', 1, None, None, '+ 회생 / − 구동'),
  ('배터리', 'socDisplay', 'SOC (계기판)', '%', 1, (5, None), None, '5% 미만 주의'),
  ('배터리', 'cellMin', '셀 최저 전압', 'V', 3, (3.2, None), (3.0, None), '3.2 V 이상 (부하 중 포함)'),
  ('배터리', 'cellMax', '셀 최고 전압', 'V', 3, (None, 4.17), (None, 4.2), '4.17 V 이하'),
  ('배터리', 'restDeltaMv', '셀 편차 (휴지·동시 측정)', 'mV', 0, (None, 20), (None, 40), '20 mV 미만'),
  ('배터리', 'chargeDeltaMv', '셀 편차 (충전 중·동시 측정)', 'mV', 0, (None, 25), (None, 40), '25 mV 미만'),
  ('배터리', 'moduleTempMax', '모듈 최고 온도', '°C', 1, (None, 45), (None, 55), '45 °C 이하'),
  ('배터리', 'moduleTempMin', '모듈 최저 온도', '°C', 1, (0, None), None, '0 °C 미만은 충전·회생 제한'),
  ('배터리', 'moduleSpread', '모듈 온도 편차', '°C', 1, (None, 5), (None, 10), '5 °C 미만'),
  ('배터리', 'nominalFullPackEnergy', '공칭 완충 용량', 'kWh', 1, None, None, 'BMS 추정, 몇 % 오르내림'),
  ('배터리', 'degradation', '열화 (신품 기준 대비)', '%', 1, (None, 10), (None, 20), '10% 미만 좋음 · 20% 이상 나쁨'),
  ('배터리', 'dischargeLimitOk', 'BMS 방전 한계 (SOC>20%·>10 °C)', 'kW', 0, (150, None), (80, None), '150 kW 이상'),
  ('배터리', 'maxRegenPower', 'BMS 회생 한계', 'kW', 0, None, None, '고SOC·저온에서 낮아짐'),
  ('배터리', 'packResistance', '팩 내부저항 (시간별 중앙값)', 'mΩ', 0, None, None, '10초 내 60 A 이상 변화의 ΔV/ΔI · 온도 따라 변함, 추세 비교용'),
  ('배터리', 'socUI', 'SOC (BMS 실제)', '%', 1, None, None, '계기판보다 높음 (바닥 버퍼 포함)'),
  ('배터리', 'energyBuffer', '바닥 버퍼 (벽돌 방지)', 'kWh', 1, None, None, ''),
  ('배터리', 'rangeIdealFull', '완충 환산 이상 주행거리', 'mi', 0, None, None, '이상 주행거리 ÷ SOC(BMS) · SOC 30% 초과에서만'),
  ('배터리', 'brickMaxV', '셀 최고 전압 (BMS 0x332)', 'V', 3, (None, 4.17), (None, 4.2), '4.17 V 이하 · 해독 검증 통과 시만 판정'),
  ('배터리', 'brickMinV', '셀 최저 전압 (BMS 0x332)', 'V', 3, (3.2, None), (3.0, None), '3.2 V 이상 · 해독 검증 통과 시만 판정'),
  ('배터리', 'bmsTempMax', '모듈 최고 온도 (BMS 0x332)', '°C', 1, (None, 45), (None, 55), '45 °C 이하 · 해독 검증 통과 시만 판정'),
  ('배터리', 'bmsTempMin', '모듈 최저 온도 (BMS 0x332)', '°C', 1, (0, None), None, '0 °C 미만은 충전·회생 제한'),
  ('배터리', 'bmsMaxChargeA', 'BMS 충전·회생 허용 전류 (0x202)', 'A', 0, None, None, '주차 0 · 충전·주행 중 BMS가 받아들이는 최대'),
  ('배터리', 'bmsMaxDischargeA', 'BMS 방전 허용 전류 (0x202)', 'A', 0, None, None, ''),
  ('배터리', 'bmsMaxChargeV', 'BMS 충전 상한 전압 (0x202)', 'V', 2, None, None, '96 × 4.2 V = 403.2 V 부근'),
  ('배터리', 'bmsMinDischargeV', 'BMS 방전 하한 전압 (0x202)', 'V', 2, None, None, ''),
  ('12V', 'dcdcOutputVoltage', 'DC-DC 출력 전압', 'V', 2, (13.0, 15.0), (12.5, 15.5), '13.0~15.0 V'),
  ('12V', 'dcdcOutputCurrent', 'DC-DC 전류', 'A', 0, (None, 150), None, '150 A 이하'),
  ('12V', 'rearInverter12V', '12V (뒤 인버터 측)', 'V', 1, (12.5, None), (12.0, None), '12.5 V 이상'),
  ('12V', 'frontInverter12V', '12V (앞 인버터 측)', 'V', 1, (12.5, None), (12.0, None), '12.5 V 이상'),
  ('12V', 'dcdcEfficiency', 'DC-DC 효율 (입력 500 W 이상)', '%', 0, (80, None), (70, None), '80% 이상'),
  ('12V', 'dcdcInletTemp', 'DC-DC 냉각수 입구 온도', '°C', 0, None, None, ''),
  ('구동', 'inverterTemp', '인버터 온도', '°C', 0, (None, 75), (None, 90), '75 °C 이하 (대략)'),
  ('구동', 'motorTemp', '모터 온도', '°C', 0, (None, 120), (None, 150), '120 °C 이하 (대략)'),
  ('구동', 'rearMechPower', '뒤 모터 출력', 'kW', 0, None, None, ''),
  ('구동', 'frontMechPower', '앞 모터 출력', 'kW', 0, None, None, ''),
  ('구동', 'rearStatorCurrent', '뒤 고정자 전류', 'A', 0, None, None, ''),
  ('구동', 'frontStatorCurrent', '앞 고정자 전류', 'A', 0, None, None, ''),
  ('구동', 'rearDrivePowerMax', '뒤 구동 허용 최대 출력', 'kW', 0, None, None, '인버터가 그 순간 허용하는 값'),
  ('구동', 'frontDrivePowerMax', '앞 구동 허용 최대 출력', 'kW', 0, None, None, '인버터가 그 순간 허용하는 값'),
  ('구동', 'rearRegenPowerMax', '뒤 회생 허용 최대', 'kW', 0, None, None, ''),
  ('구동', 'vehicleSpeedMph', '속도', 'mph', 0, None, None, ''),
  ('충전', 'chargeKw', '충전 전력 (팩 기준)', 'kW', 1, None, None, ''),
  ('충전', 'acVoltsCharge', 'AC 입력 전압 (충전 중)', 'V', 0, (190, 264), (180, 270), '190~264 V'),
  ('충전', 'pilotHighCharge', '파일럿 전압 (충전 중)', 'V', 2, (5.0, 7.0), None, 'J1772 충전 중 6 V'),
  ('충전', 'pilotLowCharge', '파일럿 음전압 (충전 중)', 'V', 2, (None, -11.0), (None, -10.0), 'J1772 −12 V · 높으면 충전기/배선 의심'),
  ('열관리', 'refrigPressure', '냉매 압력', 'bar', 1, None, None, ''),
  ('열관리', 'suctionPressure', '냉매 흡입 압력', 'bar', 1, None, None, ''),
  ('열관리', 'dischargeTemp', '냉매 토출 온도', '°C', 0, None, None, ''),
  ('열관리', 'heaterTemp', '히터 온도', '°C', 1, None, None, ''),
  ('열관리', 'pump1', '배터리 펌프', '%', 0, None, None, ''),
  ('열관리', 'pump2', '펌프 2', '%', 0, None, None, ''),
  ('열관리', 'ptPump', '구동계 펌프', '%', 0, None, None, ''),
  ('열관리', 'suctionTemp', '냉매 흡입 온도', '°C', 1, None, None, ''),
  ('열관리', 'radBypass', '라디에이터 바이패스 밸브', '%', 0, None, None, ''),
  ('열관리', 'compCooling', '컴프레서 냉각 요구', '', 0, None, None, ''),
  ('누적', 'batteryOdometerMi', '배터리 주행거리', 'mi', 0, None, None, '이 팩 기준 (차 계기판 아님)'),
  ('누적', 'kwhDischargeTotal', '누적 방전', 'kWh', 0, None, None, ''),
  ('누적', 'kwhChargeTotal', '누적 충전 (회생 포함)', 'kWh', 0, None, None, ''),
  ('누적', 'cycles', '등가 완충 사이클', '회', 0, None, None, '누적 방전 ÷ 신품 용량'),
  ('후보 (미검증)', 'tpms1', '타이어 공기압? 1', 'psi', 1, (32, 45), None, '32~45 psi'),
  ('후보 (미검증)', 'tpms2', '타이어 공기압? 2', 'psi', 1, (32, 45), None, '32~45 psi'),
  ('후보 (미검증)', 'tpms3', '타이어 공기압? 3', 'psi', 1, (32, 45), None, '32~45 psi'),
  ('후보 (미검증)', 'tpms4', '타이어 공기압? 4', 'psi', 1, (32, 45), None, '32~45 psi'),
]
# A check whose decode is only trusted while its cross-check holds (VERIFY): judged when confirmed,
# otherwise "미확인" with the reason.
CHECK_NEEDS = {'brickMaxV': '332v', 'brickMinV': '332v', 'bmsTempMax': '332t', 'bmsTempMin': '332t',
               'bmsMaxChargeA': '202c', 'bmsMaxChargeV': '202c', 'bmsMinDischargeV': '202c', 'bmsMaxDischargeA': '202d'}
UNVERIFIED_GROUPS = ('후보 (미검증)',)
EVENT_NAMES = {'CHARGE_STOPPED': '충전 멈춤', 'CHARGE_POWER_DROP': '충전 전력 급감', 'LIMIT_DROP_DISCHARGE': 'BMS 방전 한계 급감',
               'LIMIT_DROP_REGEN': 'BMS 회생 한계 급감', 'LOW_12V': '12V 낮음', 'MODULE_HOT': '모듈 고온', 'CELL_DELTA_HIGH': '셀 편차 큼'}


def _out(v, rng) -> bool:
  return rng is not None and ((rng[0] is not None and v < rng[0]) or (rng[1] is not None and v > rng[1]))


def checks(days: float = CHECK_DAYS) -> dict:
  """Every check with its logged min/max, the range it is held to and a verdict, plus the fault
  states and logger events seen. Nothing is left out for being normal."""
  from openpilot.system.btobd import tesla_pt
  S = _stats(days)
  agg = S['agg']
  since = time.time() - days * 86400
  health = [h for h in vlog.read_jsonl(os.path.join(VLOG_DIR, 'health.jsonl')) if h.get('t', 0) >= since]
  for h in health:
    if h.get('deltaMv') is not None:
      a = agg.setdefault('restDeltaMv', [h['deltaMv'], h['deltaMv'], 0, h['t'], h['t']])
      a[2] += 1
      if h['deltaMv'] < a[0]:
        a[0], a[3] = h['deltaMv'], h['t']
      if h['deltaMv'] > a[1]:
        a[1], a[4] = h['deltaMv'], h['t']
  try:
    ref = float(_param_file('BtObdPackNewKwh') or 0)
  except ValueError:
    ref = 0.0
  if ref > 0 and 'nominalFullPackEnergy' in agg:
    lo, hi, n, tlo, thi = agg['nominalFullPackEnergy']
    agg['degradation'] = [(1 - hi / ref) * 100, (1 - lo / ref) * 100, n, thi, tlo]
  if ref > 0 and 'kwhDischargeTotal' in agg:
    lo, hi, n, tlo, thi = agg['kwhDischargeTotal']
    agg['cycles'] = [lo / ref, hi / ref, n, tlo, thi]

  decodes = decode_checks(agg)
  dstat = {d['key']: d for d in decodes}
  rows = []
  for group, key, label, unit, dg, warn, bad, refs in CHECKS:
    a = agg.get(key)
    row = {'group': group, 'key': key, 'label': label, 'unit': unit, 'digits': dg, 'ref': refs}
    if a is None:
      rows.append({**row, 'level': 'na'})
      continue
    lo, hi, n, tlo, thi = a
    level = 'info' if warn is None and bad is None else 'ok'
    worst = None
    for lvl, rng in (('warn', warn), ('bad', bad)):
      for v, t in ((lo, tlo), (hi, thi)):
        if _out(v, rng):
          level, worst = lvl, t
    why = ''
    # rule: nothing is called normal on data that was not confirmed. An out-of-range value still shows
    # as unconfirmed rather than ok; only ok/info are downgraded.
    need = dstat.get(CHECK_NEEDS.get(key, ''))
    if group in UNVERIFIED_GROUPS:
      level, why = 'unverified', '신호 의미 미검증: 판정하지 않음'
    elif need and need['status'] != 'confirmed':
      level, why = 'unverified', f"해독 검증 {need['statusText']}: 판정하지 않음"
    elif n < MIN_N and level == 'ok':           # a reference row (참고) is not a verdict, nothing to withhold
      level, why = 'few', f'표본 {n}개: 판정하지 않음'
    rows.append({**row, 'min': round(lo, dg), 'max': round(hi, dg), 'n': n, 'tMin': tlo, 'tMax': thi, 'level': level,
                 'tWorst': worst, 'why': why})
  if ref > 0:
    next(r for r in rows if r['key'] == 'degradation')['ref'] += f' (신품 {ref:.1f} kWh)'
  else:
    r = next(r for r in rows if r['key'] == 'degradation')
    r.update(level='na', why='신품 용량(BtObdPackNewKwh) 미설정')

  faults = []
  for x in tesla_pt.LIVE_SIGS:
    a = agg.get('bad:' + x.key)
    if a:
      faults.append({'label': x.label, 'value': x.enum.get(a[0], a[0]) if x.enum else a[0], 'n': a[2], 't': a[4], 'level': 'bad'})
  sessions = [s for s in vlog.read_jsonl(os.path.join(VLOG_DIR, 'sessions.jsonl'))
              if s.get('type') in vlog.CHARGE_MODES and s.get('end', 0) >= since]
  events = []
  for e in vlog.read_jsonl(os.path.join(VLOG_DIR, 'events.jsonl')):
    if e.get('t', 0) < since:
      continue
    level, note = 'warn', ''
    if e['type'] == 'CHARGE_STOPPED':
      s = next((s for s in sessions if s['start'] - 60 <= e['t'] <= s['end'] + 300 and s.get('endCause')), None)
      cause = s.get('endCause') if s else None
      level = END_LEVEL.get(cause, 'warn')
      note = (vlog.CHARGE_END.get(cause, cause) + ': ' + ' · '.join(s.get('endWhy') or [])) if s else '이 충전의 종료 원인 기록 없음 (원인 불명)'
    events.append({'type': e['type'], 'label': EVENT_NAMES.get(e['type'], e['type']), 'value': e.get('value'),
                   'threshold': e.get('threshold'), 't': e['t'], 'level': level, 'note': note})
  ends = [{'t': s.get('endT') or s['end'], 'type': s['type'], 'cause': s.get('endCause'),
           'label': vlog.CHARGE_END.get(s.get('endCause'), '원인 기록 없음'), 'why': s.get('endWhy') or [],
           'level': END_LEVEL.get(s.get('endCause'), 'warn')} for s in sessions]
  count = {}
  for r in rows + faults + events:
    count[r['level']] = count.get(r['level'], 0) + 1
  return {'rows': rows, 'faults': faults, 'events': events, 'count': count, 'days': days, 'files': S['files'],
          'health': len(health), 'from': S['from'], 'decodes': decodes, 'resistance': S.get('resistance', []),
          'chargeEnds': ends}


END_LEVEL = {'complete': 'info', 'user': 'info', 'unplugged': 'info', 'abnormal': 'bad', 'unknown': 'warn'}
VERIFY_TEXT = {'confirmed': '확인됨', 'failed': '불일치', 'insufficient': '비교 표본 부족'}


def decode_checks(agg: dict) -> list:
  """Each VERIFY comparison over the window: how many, how far off, how many outside tolerance."""
  out = []
  for k, (label, against, unit, tol) in VERIFY.items():
    parts = [v for n, v in agg.items() if n.startswith('vrf:' + k)]
    n = sum(p[2] for p in parts)
    miss = sum(v[2] for name, v in agg.items() if name.startswith('vrfmiss:' + k))
    status = 'insufficient' if n < VERIFY_MIN_N else 'confirmed' if miss <= VERIFY_MAX_MISS * n else 'failed'
    out.append({'key': k, 'label': label, 'against': against, 'unit': unit, 'tol': tol, 'center': VERIFY_CENTER.get(k, 0.0),
                'n': n, 'miss': miss, 'min': round(min(p[0] for p in parts), 3) if parts else None,
                'max': round(max(p[1] for p in parts), 3) if parts else None, 'status': status, 'statusText': VERIFY_TEXT[status]})
  return out
