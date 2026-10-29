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
import time

from openpilot.system.btobd import tdiag, vlog

VLOG_DIR = os.path.join(tdiag.LOG_DIR, 'vlog')
SWAGLOG_DIR = '/data/log'
OBD_STATE_PATH = '/dev/shm/btobd.json'
POWER_DAYS = 14
_cache: dict = {}


def _cached(key: str, stamp, fn):
  hit = _cache.get(key)
  if hit and hit[0] == stamp:
    return hit[1]
  val = fn()
  _cache[key] = (stamp, val)
  return val


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


def replayed() -> dict:
  """live.jsonl (+ its .1 rotation) through vlog's session logic, for drives/charges logged before
  the always-on logger existed. Sessions the logger itself wrote later take precedence."""
  paths = [os.path.join(tdiag.LOG_DIR, n) for n in (tdiag.LIVE_LOG + '.1', tdiag.LIVE_LOG)]

  def run():
    out = {'sessions': [], 'events': [], 'rows': 0}
    for p in paths:
      r = vlog.replay_live(p)
      out['sessions'] += r['sessions']
      out['events'] += r['events']
      out['rows'] += r['rows']
    for s in out['sessions']:
      s['source'] = 'live.jsonl 재분석'
    return out
  return _cached('replay', tuple(_mtime(p) for p in paths), run)


def power_rows() -> list:
  since = time.time() - POWER_DAYS * 86400
  swag = _cached('swag', _swag_stamp(), lambda: vlog.power_from_swaglog(SWAGLOG_DIR))
  own = [r for r in vlog.read_jsonl(os.path.join(VLOG_DIR, 'power.jsonl')) if r.get('t', 0) >= since]
  first_own = own[0]['t'] if own else float('inf')
  # the logger's own 1-minute record wins where it exists; swaglog fills in before it
  return [r for r in swag if since <= r['t'] < first_own] + own


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


def overview() -> dict:
  own = vlog.read_jsonl(os.path.join(VLOG_DIR, 'sessions.jsonl'))
  for s in own:
    s['source'] = '상시 로거'
  rep = replayed()
  first_own = min((s['start'] for s in own), default=float('inf'))
  sessions = [s for s in rep['sessions'] if s['end'] < first_own] + own
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


SESSION_COLS = ['source', 'type', 'start', 'end', 'durationS', 'socStart', 'socEnd', 'kwhIn', 'kwhOut', 'avgKw', 'kwMax',
                'kwMin', 'miles', 'milesSource', 'whPerMi', 'regenKwh', 'min10to80', 'min20to80', 'tempMin', 'tempMax',
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
