"""What the /vlog page shows: btobdd's always-on vehicle log, plus a re-analysis of the logs that
existed before it (live.jsonl through the same session logic, swaglog STATUS_PACKETs for power).

Read-only over the files; nothing here talks to the daemon except reading its tmpfs state.
"""
from __future__ import annotations

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
    m['events'] = (p.get('events') or []) + (s.get('events') or [])
    m['flags'] = sorted(set((p.get('flags') or []) + (s.get('flags') or [])) - {'interrupted'}) + ['merged']
    out[-1] = m
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


# ---- anomalies from the log -------------------------------------------------------------------------
# Thresholds are the /diag health criteria; the rest are where this car's own logs say "normal" ends
# (DC-DC 13.5-14.7 V awake, module spread <= 2 C, discharge limit >= 330 kW on 2026-10-29/30).
ANOMALY_DAYS = 14
LIMITS = {'cellDeltaMv': (20.0, 40.0), 'weakCellMv': 30.0, 'degradation': (0.10, 0.20), 'moduleHotC': (45.0, 55.0),
          'moduleSpreadC': (5.0, 10.0), 'low12V': 13.0, 'dischargeLimitKw': 150.0, 'tpmsLowPsi': 32.0, 'tpmsSpreadPsi': 4.0}
BMS_FAULT = (7, 8)


def _param_file(key: str) -> str:
  try:
    with open(os.path.join(PARAMS_DIR, key)) as f:
      return f.read().strip()
  except OSError:
    return ''


def _series_days(days: float) -> list:
  out, now = [], time.time()
  for k in range(int(days) + 1):
    name = 'series-' + time.strftime('%Y%m%d', time.gmtime(now - k * 86400)) + '.jsonl'
    out += vlog.read_jsonl(os.path.join(VLOG_DIR, name))
  return sorted((r for r in out if r.get('t', 0) >= now - days * 86400), key=lambda r: r['t'])


def anomalies(days: float = ANOMALY_DAYS) -> dict:
  """Only what is out of line in the logged data, each with the number behind it and when."""
  L = LIMITS
  items: list = []

  def add(level, title, detail, t=None):
    items.append({'level': level, 'title': title, 'detail': detail, 't': t})
  since = time.time() - days * 86400

  health = [h for h in vlog.read_jsonl(os.path.join(VLOG_DIR, 'health.jsonl')) if h.get('t', 0) >= since]
  if health:
    last = health[-1]
    worst = max(health, key=lambda h: h.get('deltaMv') or 0)
    if (worst.get('deltaMv') or 0) >= L['cellDeltaMv'][0]:
      add('bad' if worst['deltaMv'] >= L['cellDeltaMv'][1] else 'warn', '셀 편차 큼 (휴지·동시 측정)',
          f"{worst['deltaMv']:.0f} mV (기준 {L['cellDeltaMv'][0]:.0f} mV 미만)", worst['t'])
    cells = last.get('cells') or []
    if len(cells) == 96:
      avg = sum(cells) / 96
      weak = [(i + 1, (avg - v) * 1000) for i, v in enumerate(cells) if (avg - v) * 1000 >= L['weakCellMv']]
      if weak:
        add('bad', '약한 셀', ', '.join(f'#{i} −{mv:.0f} mV' for i, mv in weak[:6]) + ' (평균 대비)', last['t'])
    lows = [h.get('lowIndex') for h in health[-5:]]
    if len(lows) >= 4:
      top = max(set(lows), key=lows.count)
      if lows.count(top) >= 3:
        add('warn', '같은 셀이 반복 최저', f'#{top}: 최근 스냅샷 {len(lows)}개 중 {lows.count(top)}번', last['t'])
    bad_sum = [h for h in health if h.get('sumCheck') and abs(h['sumCheck'] - 1) > 0.02]
    if bad_sum:
      add('warn', '셀 합 ≠ 팩 전압', f"합/팩 {bad_sum[-1]['sumCheck']:.3f} — 셀 해독 확인 필요", bad_sum[-1]['t'])

  rows = _series_days(days)
  nfp = next((h['nominalFullPackEnergy'] for h in reversed(health) if h.get('nominalFullPackEnergy')), None)
  st = last_state()
  if nfp is None and st:
    nfp = st['values'].get('nominalFullPackEnergy')
  try:
    ref = float(_param_file('BtObdPackNewKwh') or 0)
  except ValueError:
    ref = 0.0
  if nfp and ref > 0:
    deg = 1 - nfp / ref
    if deg >= L['degradation'][0]:
      add('bad' if deg >= L['degradation'][1] else 'warn', '배터리 열화',
          f"{deg * 100:.1f}% · 공칭 완충 {nfp:.1f} / 신품 {ref:.1f} kWh (기준 10% 미만, 20% 이상 나쁨)", None)

  hot = [r for r in rows if (r.get('moduleTempMax') or 0) > L['moduleHotC'][0]]
  if hot:
    m = max(hot, key=lambda r: r['moduleTempMax'])
    add('bad' if m['moduleTempMax'] > L['moduleHotC'][1] else 'warn', '배터리 모듈 고온',
        f"최고 {m['moduleTempMax']:.1f} °C · {len(hot)}회 (기준 {L['moduleHotC'][0]:.0f} °C)", m['t'])
  spread = [r for r in rows if r.get('moduleTempMax') is not None and r.get('moduleTempMin') is not None
            and r['moduleTempMax'] - r['moduleTempMin'] > L['moduleSpreadC'][0]]
  if spread:
    m = max(spread, key=lambda r: r['moduleTempMax'] - r['moduleTempMin'])
    d = m['moduleTempMax'] - m['moduleTempMin']
    add('bad' if d > L['moduleSpreadC'][1] else 'warn', '모듈 온도 편차 큼', f'{d:.1f} °C · {len(spread)}회 (기준 5 °C)', m['t'])
  low = [r for r in rows if r.get('dcdcOutputVoltage') is not None and r['dcdcOutputVoltage'] < L['low12V']]
  if low:
    m = min(low, key=lambda r: r['dcdcOutputVoltage'])
    add('warn', '12V 낮음 (DC-DC 출력)', f"최저 {m['dcdcOutputVoltage']:.2f} V · {len(low)}회 (평소 13.5~14.7 V)", m['t'])
  fault = [r for r in rows if r.get('bmsState') in BMS_FAULT]
  if fault:
    add('bad', 'BMS 오류 상태', f'{len(fault)}회 기록', fault[-1]['t'])
  lim = [r for r in rows if r.get('maxDischargePower') is not None and r['maxDischargePower'] < L['dischargeLimitKw']
         and (r.get('socDisplay') or 100) > 20 and (r.get('moduleTempMin') or 20) > 10]
  if lim:
    m = min(lim, key=lambda r: r['maxDischargePower'])
    add('warn', 'BMS 방전 한계 낮음', f"{m['maxDischargePower']:.0f} kW (SOC 20% 초과·10 °C 초과인데 {L['dischargeLimitKw']:.0f} kW 미만)", m['t'])

  for e in vlog.read_jsonl(os.path.join(VLOG_DIR, 'events.jsonl')):
    if e.get('t', 0) < since:
      continue
    ctx = e.get('context') or {}
    # the charge ending because AC went away (unplugged, schedule, charge limit) is not a fault
    if e['type'] == 'CHARGE_STOPPED' and ((ctx.get('acVolts1') or 0) < 50 or ctx.get('pilot') == 0):
      continue
    add('warn', {'CHARGE_STOPPED': 'AC가 있는데 충전 멈춤', 'CHARGE_POWER_DROP': '충전 전력 급감',
                 'LIMIT_DROP_DISCHARGE': 'BMS 방전 한계 급감', 'LIMIT_DROP_REGEN': 'BMS 회생 한계 급감',
                 'LOW_12V': '12V 낮음', 'MODULE_HOT': '모듈 고온', 'CELL_DELTA_HIGH': '셀 편차 큼'}.get(e['type'], e['type']),
        f"값 {e.get('value')} · 기준 {e.get('threshold')}", e['t'])

  sm = recent_summary()
  if sm:
    for g in sm['groups']:
      for r in g['rows']:
        if g['key'] in ('charge', 'thermal', 'ident') and r['status'] in ('bad', 'warn'):
          add(r['status'], r['label'], f"{r['value']} {r.get('unit') or ''} {r.get('detail') or ''}".strip(), sm['end'])
    tp = [float(r['value']) for g in sm['groups'] if g['key'] == 'candidate' for r in g['rows']
          if r['key'].startswith('tpms') and r['status'] != 'na']
    if len(tp) == 4 and (min(tp) < L['tpmsLowPsi'] or max(tp) - min(tp) > L['tpmsSpreadPsi']):
      add('warn', '타이어 공기압 차이 (미검증 신호 0x3F8)', ' / '.join(f'{p:.1f}' for p in tp) + ' psi', sm['end'])

  order = {'bad': 0, 'warn': 1}
  items.sort(key=lambda x: (order.get(x['level'], 2), -(x['t'] or 0)))
  return {'items': items, 'days': days, 'samples': len(rows), 'health': len(health),
          'from': rows[0]['t'] if rows else None, 'to': rows[-1]['t'] if rows else None}
