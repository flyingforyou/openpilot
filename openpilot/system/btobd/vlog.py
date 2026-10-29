"""Always-on vehicle log: what the car did (drive / charge / parked), how the battery behaved, and
the raw frames behind it.

btobdd feeds this once per poll cycle with the decoded values and their ages. Nothing here talks to
the dongle or the car, so the whole thing replays from saved logs (tests, and the page's re-analysis
of live.jsonl from before this existed).

Files, all under LOG_DIR/vlog:
  sessions.jsonl     one line per finished DRIVE / AC_CHARGE / DC_CHARGE / CHARGE_UNKNOWN / PARKED
  events.jsonl       one line per event, with the 60 s of samples before it and 20 s after
  health.jsonl       resting snapshots: all 96 bricks read together at |I| <= 5 A
  series-YYYYMMDD.jsonl   downsampled values (5 s moving, 60 s parked), fresh values only
  raw-YYYYMMDD-HH.jsonl.gz  every frame heard, with monotonic and wall time and the poll window
  open.json          the session in progress, so a restart closes it instead of losing it

Rules the spec asked for and this keeps: a value older than FRESH_S is not a sample (never copied
forward as new); energy is integrated only between fresh samples less than MAX_GAP_S apart and the
rest is counted as a gap; the 96 bricks are compared with each other only when they were read
within CELL_SYNC_S of each other; anything not verified stays out of the derived numbers.
Sign: packCurrent and packPower are positive INTO the pack (charging, regen).
"""
from __future__ import annotations

import gzip
import json
import os
import time
from collections import deque
from datetime import datetime, timezone, UTC

FRESH_S = 5.0          # a decoded value older than this is not a current sample
MAX_GAP_S = 15.0       # energy is not integrated across a gap longer than this
LINK_GAP_S = 600.0     # a session with no sample for this long is closed where it stopped
REST_A = 5.0           # |packCurrent| at or under this counts as rest
REST_HOLD_S = 60.0     # ... for this long before a resting snapshot
HEALTH_EVERY_S = 6 * 3600
SERIES_DT = {'DRIVE': 5.0, 'AC_CHARGE': 5.0, 'DC_CHARGE': 5.0, 'CHARGE_UNKNOWN': 5.0, 'PARKED': 60.0}
HOLD_S = {'DRIVE': 3.0, 'CHARGE': 20.0, 'PARKED': 30.0}   # a new mode must persist this long
RING_S = 60.0
POST_S = 20.0
STORE_MAX = 400_000_000   # series + raw files beyond this are deleted oldest first
CHARGE_MODES = ('AC_CHARGE', 'DC_CHARGE', 'CHARGE_UNKNOWN')
BMS_CHARGING = (3, 4)     # tesla_pt.BMS_STATE: CHARGER, FASTCHARGE

THRESHOLDS = {
  'chargeStopKw': 0.3,        # charging power under this for chargeStopS after being over 1 kW
  'chargeStopS': 10.0,
  'chargeDropFrac': 0.4,      # charging power fell by this fraction vs. the last 30 s peak
  'limitDropFrac': 0.3,       # BMS discharge/regen limit fell by this fraction vs. the last 60 s peak
  'low12V': 12.0,             # DC-DC output under this (V)
  'moduleHotC': 45.0,
  'cellDeltaMv': 40.0,        # synchronised, resting brick spread
  'confirmN': 3,              # consecutive samples before an event fires
}

SERIES_KEYS = ('packVoltage', 'packCurrent', 'packPower', 'socUI', 'socDisplay', 'nominalEnergyRemaining',
               'vehicleSpeedMph', 'gear', 'rearMechPower', 'frontMechPower', 'rearTorqueMeasured', 'frontTorqueMeasured',
               'rearMotorRpm', 'maxDischargePower', 'maxRegenPower', 'moduleTempMin', 'moduleTempMax', 'cellMin',
               'cellMax', 'cellDelta', 'cellSync', 'dcdcOutputVoltage', 'inverterTemp', 'motorTemp', 'bmsState',
               'pilot', 'acVolts1', 'acVolts2', 'acVolts3', 'heaterState', 'pump1', 'refrigPressure')


def _day(t: float) -> str:
  return datetime.fromtimestamp(t, UTC).strftime('%Y%m%d')


def fresh(values: dict, age: dict, max_age: float = FRESH_S) -> dict:
  """Only values that were set within max_age seconds. A value with no age is not trusted."""
  return {k: v for k, v in values.items() if age.get(k) is not None and age[k] <= max_age}


class ModeMachine:
  """DRIVE / AC_CHARGE / DC_CHARGE / CHARGE_UNKNOWN / PARKED from signals that agree, with hold.

  Charging: BMS state CHARGER/FASTCHARGE, or (gear not D/R and >3 A into the pack). DC when the BMS
  says FASTCHARGE or the fast-charge contactor is closed or the pilot says fast charge; AC when the
  BMS says CHARGER with a line-charge pilot; otherwise CHARGE_UNKNOWN. Driving: gear D/R/N, or
  openpilot onroad, or moving >3 mph -- speed alone only when the gear is not being heard.
  """

  def __init__(self):
    self.mode: str | None = None
    self.since = 0.0
    self._cand: str | None = None
    self._cand_since = 0.0

  @staticmethod
  def candidate(v: dict, onroad: bool) -> str | None:
    gear, spd, cur = v.get('gear'), v.get('vehicleSpeedMph'), v.get('packCurrent')
    bms, fc, pilot = v.get('bmsState'), v.get('fcContactor'), v.get('pilot')
    moving_gear = gear in ('D', 'R', 'N')
    charging = bms in BMS_CHARGING or (not moving_gear and cur is not None and cur > 3.0)
    if charging and not moving_gear:
      if bms == 4 or fc == 1 or pilot == 3:
        return 'DC_CHARGE'
      if bms == 3 and pilot == 2:
        return 'AC_CHARGE'
      return 'CHARGE_UNKNOWN'
    if moving_gear or onroad or (gear is None and spd is not None and spd > 3.0):
      return 'DRIVE'
    if gear == 'P' or cur is not None:
      return 'PARKED'
    return None   # nothing fresh to judge by: keep whatever we had

  def update(self, t: float, v: dict, onroad: bool = False) -> bool:
    """Returns True when the mode changed."""
    c = self.candidate(v, onroad)
    if c is None or c == self.mode:
      self._cand = None
      return False
    if c != self._cand:
      self._cand, self._cand_since = c, t
    # between charge kinds (AC->DC once the contactor reports) switch without waiting
    hold = 0.0 if self.mode in CHARGE_MODES and c in CHARGE_MODES else HOLD_S['CHARGE' if c in CHARGE_MODES else c]
    if t - self._cand_since < hold:
      return False
    self.mode, self.since, self._cand = c, self._cand_since, None
    return True


class Session:
  def __init__(self, kind: str, t: float, v: dict):
    self.d = {'type': kind, 'start': t, 'end': t, 'socStart': v.get('socUI'), 'socEnd': v.get('socUI'),
              'odoStart': v.get('batteryOdometerMi'), 'odoEnd': v.get('batteryOdometerMi'),
              'kwhIn': 0.0, 'kwhOut': 0.0, 'kwMax': None, 'kwMin': None, 'kwSum': 0.0, 'kwN': 0,
              'samples': 0, 'staleSamples': 0, 'gapS': 0.0, 'coveredS': 0.0, 'miSpeed': 0.0,
              'tempMin': None, 'tempMax': None, 'cellDeltaSyncMax': None, 'events': [], 'flags': [],
              'socFirstAt': {}, 'socBinS': {}, 'curve': {}, 'limitMinDischarge': None, 'limitMinRegen': None}
    self._last_t: float | None = None
    self._last_kw: float | None = None
    self._last_soc: float | None = None

  def add(self, t: float, v: dict, stale: int) -> None:
    d = self.d
    d['end'] = t
    d['samples'] += 1
    d['staleSamples'] += stale
    if v.get('socUI') is not None:
      d['socStart'] = d['socStart'] if d['socStart'] is not None else v['socUI']
      d['socEnd'] = v['socUI']
      s = int(v['socUI'])
      d['socFirstAt'].setdefault(str(s), t)
    if v.get('batteryOdometerMi') is not None:
      d['odoStart'] = d['odoStart'] if d['odoStart'] is not None else v['batteryOdometerMi']
      d['odoEnd'] = v['batteryOdometerMi']
    kw = None
    if v.get('packVoltage') is not None and v.get('packCurrent') is not None:
      kw = v['packVoltage'] * v['packCurrent'] / 1000.0
      d['kwMax'] = kw if d['kwMax'] is None else max(d['kwMax'], kw)
      d['kwMin'] = kw if d['kwMin'] is None else min(d['kwMin'], kw)
      d['kwSum'] += kw
      d['kwN'] += 1
    if self._last_t is not None:
      dt = t - self._last_t
      if kw is not None and self._last_kw is not None and 0 < dt <= MAX_GAP_S:
        e = (kw + self._last_kw) / 2 * dt / 3600.0          # trapezoid, kWh
        if e >= 0:
          d['kwhIn'] += e
        else:
          d['kwhOut'] -= e
        d['coveredS'] += dt
        if self._last_soc is not None:
          b = str(int(self._last_soc // 10 * 10))
          d['socBinS'][b] = d['socBinS'].get(b, 0.0) + dt
      elif dt > 0:
        d['gapS'] += dt
      spd = v.get('vehicleSpeedMph')
      if spd is not None and 0 < dt <= MAX_GAP_S:
        d['miSpeed'] += spd * dt / 3600.0
    if kw is not None:
      self._last_t, self._last_kw = t, kw
    elif self._last_t is None:
      self._last_t = t
    if v.get('socUI') is not None:
      self._last_soc = v['socUI']
      if kw is not None and d['type'] in CHARGE_MODES:
        c = d['curve'].setdefault(str(int(v['socUI'])), [0.0, 0])
        c[0] += kw
        c[1] += 1
    for k, lo in (('moduleTempMin', 'tempMin'), ('moduleTempMax', 'tempMax')):
      if v.get(k) is not None:
        d[lo] = v[k] if d[lo] is None else (min if lo == 'tempMin' else max)(d[lo], v[k])
    if v.get('cellSync') and v.get('cellDelta') is not None:
      mv = v['cellDelta'] * 1000
      d['cellDeltaSyncMax'] = mv if d['cellDeltaSyncMax'] is None else max(d['cellDeltaSyncMax'], mv)
    for k, lo in (('maxDischargePower', 'limitMinDischarge'), ('maxRegenPower', 'limitMinRegen')):
      if v.get(k) is not None:
        d[lo] = v[k] if d[lo] is None else min(d[lo], v[k])

  def finish(self, reason: str = '') -> dict:
    d = dict(self.d)
    span = d['end'] - d['start']
    d['durationS'] = round(span, 1)
    d['avgKw'] = round(d['kwSum'] / d['kwN'], 2) if d['kwN'] else None
    for k in ('kwhIn', 'kwhOut', 'miSpeed', 'gapS', 'coveredS'):
      d[k] = round(d[k], 3)
    d['missingRate'] = round(d['gapS'] / span, 3) if span > 0 else None
    d['staleRate'] = round(d['staleSamples'] / max(1, d['samples'] * len(SERIES_KEYS)), 3)
    miles = None
    odo = d['odoEnd'] - d['odoStart'] if d['odoStart'] is not None and d['odoEnd'] is not None else None
    # 0x562 comes every ~10 s; a start value carried over from before the session (seen replaying
    # live.jsonl: 22 mi "driven" in 1.7 min) disagrees with the speed integral, which then wins
    if odo is not None and odo > 0 and (d['miSpeed'] < 0.05 or odo <= d['miSpeed'] * 1.5 + 0.3):
      miles, d['milesSource'] = odo, 'odometer'
    elif odo is not None and odo > 0:
      d['flags'].append('odometer-inconsistent')
    if miles is None and d['miSpeed'] > 0.05:
      miles, d['milesSource'] = d['miSpeed'], 'speed'
    d['miles'] = round(miles, 2) if miles is not None else None
    if d['type'] == 'DRIVE' and miles:
      d['whPerMi'] = round((d['kwhOut'] - d['kwhIn']) * 1000 / miles, 1)
      d['regenKwh'] = d['kwhIn']
    if d['type'] in CHARGE_MODES:
      first = {int(k): v for k, v in d['socFirstAt'].items()}
      for a, b in ((10, 80), (20, 80)):
        # only when the session itself was seen at or below `a` and reached `b`
        if d['socStart'] is not None and d['socStart'] <= a and a in first and b in first:
          d[f'min{a}to{b}'] = round((first[b] - first[a]) / 60, 1)
      d['curve'] = {k: round(s / n, 2) for k, (s, n) in sorted(d['curve'].items(), key=lambda kv: int(kv[0]))}
    else:
      d.pop('curve')
    d.pop('socFirstAt')
    d.pop('kwSum')
    if reason:
      d['flags'].append(reason)
    if d['missingRate'] and d['missingRate'] > 0.2:
      d['flags'].append('gappy')
    return d


class Events:
  """Threshold events confirmed over consecutive samples, each saved with the samples around it."""

  def __init__(self, thresholds: dict | None = None):
    self.th = {**THRESHOLDS, **(thresholds or {})}
    self.ring: deque = deque()
    self.pending: list = []
    self.count: dict = {}
    self.peak_kw: deque = deque()        # (t, kw) while charging
    self.peak_lim: deque = deque()       # (t, discharge, regen)
    self.was_charging_at: float | None = None
    self.low_since: float | None = None

  def _hit(self, name: str, cond: bool) -> bool:
    n = self.count.get(name, 0) + 1 if cond else 0
    self.count[name] = n
    return n == self.th['confirmN']

  def update(self, t: float, mode: str | None, v: dict) -> list:
    """Returns events that just finished collecting their after-samples."""
    sample = {'t': round(t, 2), **{k: v[k] for k in SERIES_KEYS if k in v}}
    self.ring.append(sample)
    while self.ring and t - self.ring[0]['t'] > RING_S:
      self.ring.popleft()
    fired = []
    kw = v['packVoltage'] * v['packCurrent'] / 1000 if 'packVoltage' in v and 'packCurrent' in v else None
    th = self.th
    if mode in CHARGE_MODES and kw is not None:
      if kw > 1.0:
        self.was_charging_at, self.low_since = t, None
      elif self.was_charging_at is not None and kw < th['chargeStopKw']:
        self.low_since = self.low_since or t
        if t - self.low_since >= th['chargeStopS']:
          fired.append(('CHARGE_STOPPED', kw, th['chargeStopKw']))
          self.was_charging_at = self.low_since = None
      self.peak_kw.append((t, kw))
      while self.peak_kw and t - self.peak_kw[0][0] > 30:
        self.peak_kw.popleft()
      peak = max(k for _, k in self.peak_kw)
      if self._hit('CHARGE_POWER_DROP', peak > 1.0 and kw > 0.3 and kw < peak * (1 - th['chargeDropFrac'])):
        fired.append(('CHARGE_POWER_DROP', round(kw, 2), round(peak, 2)))
    if 'maxDischargePower' in v and 'maxRegenPower' in v:
      self.peak_lim.append((t, v['maxDischargePower'], v['maxRegenPower']))
      while self.peak_lim and t - self.peak_lim[0][0] > 60:
        self.peak_lim.popleft()
      pd = max(x[1] for x in self.peak_lim)
      pr = max(x[2] for x in self.peak_lim)
      if self._hit('LIMIT_DROP_DISCHARGE', pd > 20 and v['maxDischargePower'] < pd * (1 - th['limitDropFrac'])):
        fired.append(('LIMIT_DROP_DISCHARGE', v['maxDischargePower'], pd))
      if self._hit('LIMIT_DROP_REGEN', pr > 10 and v['maxRegenPower'] < pr * (1 - th['limitDropFrac'])):
        fired.append(('LIMIT_DROP_REGEN', v['maxRegenPower'], pr))
    if self._hit('LOW_12V', v.get('dcdcOutputVoltage') is not None and v['dcdcOutputVoltage'] < th['low12V']):
      fired.append(('LOW_12V', v['dcdcOutputVoltage'], th['low12V']))
    if self._hit('MODULE_HOT', v.get('moduleTempMax') is not None and v['moduleTempMax'] > th['moduleHotC']):
      fired.append(('MODULE_HOT', v['moduleTempMax'], th['moduleHotC']))
    rest = v.get('packCurrent') is not None and abs(v['packCurrent']) <= REST_A
    if self._hit('CELL_DELTA_HIGH', rest and v.get('cellSync') and v.get('cellDelta') is not None
                 and v['cellDelta'] * 1000 > th['cellDeltaMv']):
      fired.append(('CELL_DELTA_HIGH', round(v['cellDelta'] * 1000, 1), th['cellDeltaMv']))
    for name, value, ref in fired:
      self.pending.append({'t': round(t, 2), 'type': name, 'mode': mode, 'value': value, 'threshold': ref,
                           'context': sample, 'before': list(self.ring), 'after': []})
    done = []
    for e in self.pending:
      if t > e['t']:
        e['after'].append(sample)
      if t - e['t'] >= POST_S:
        done.append(e)
    self.pending = [e for e in self.pending if e not in done]
    return done


class VehicleLog:
  """The whole logger: mode, session, events, health snapshots, series and raw frames on disk."""

  def __init__(self, directory: str, thresholds: dict | None = None, write: bool = True):
    self.dir = directory
    self.write = write
    self.modes = ModeMachine()
    self.events = Events(thresholds)
    self.session: Session | None = None
    self.last_series = 0.0
    self.rest_since: float | None = None
    self.last_health = 0.0
    self.last_health_soc: float | None = None
    self.raw_buf: list = []
    self.raw_flush = 0.0
    self.out: dict = {'sessions': [], 'events': [], 'health': []}   # what was written, for tests/replay
    if write:
      os.makedirs(directory, exist_ok=True)
      self._recover()

  # ---- files ------------------------------------------------------------------------------------
  def _append(self, name: str, obj: dict) -> None:
    kind = name.split('.', maxsplit=1)[0]
    if kind in self.out:
      self.out[kind].append(obj)
    if not self.write:
      return
    try:
      with open(os.path.join(self.dir, name), 'a') as f:
        f.write(json.dumps(obj, ensure_ascii=False, separators=(',', ':')) + '\n')
    except OSError:
      pass

  def _save_open(self) -> None:
    if not self.write:
      return
    try:
      tmp = os.path.join(self.dir, 'open.json.tmp')
      with open(tmp, 'w') as f:
        json.dump(self.session.d if self.session else None, f)
      os.replace(tmp, os.path.join(self.dir, 'open.json'))
    except OSError:
      pass

  def _recover(self) -> None:
    """A session left open by a crash or power loss is closed at its last sample, flagged."""
    p = os.path.join(self.dir, 'open.json')
    try:
      with open(p) as f:
        d = json.load(f)
    except (OSError, ValueError):
      return
    if d:
      s = Session(d['type'], d['start'], {})
      s.d.update(d)
      self._append('sessions.jsonl', s.finish('interrupted'))
    try:
      os.remove(p)
    except OSError:
      pass

  def prune(self) -> None:
    """Delete the oldest series/raw files once they pass STORE_MAX together."""
    try:
      files = [(os.path.getmtime(os.path.join(self.dir, n)), os.path.getsize(os.path.join(self.dir, n)), n)
               for n in os.listdir(self.dir) if n.startswith(('series-', 'raw-'))]
    except OSError:
      return
    total = sum(f[1] for f in files)
    for _, size, n in sorted(files):
      if total <= STORE_MAX:
        break
      try:
        os.remove(os.path.join(self.dir, n))
        total -= size
      except OSError:
        pass

  # ---- raw frames -------------------------------------------------------------------------------
  def frame(self, wall: float, mono: float, msg_id: int, data: bytes, window: str) -> None:
    self.raw_buf.append([round(wall, 3), round(mono, 3), f'{msg_id:03X}', data.hex().upper(), window])
    if wall - self.raw_flush > 10 or len(self.raw_buf) > 2000:
      self.flush_raw(wall)

  def flush_raw(self, wall: float | None = None) -> None:
    if not self.raw_buf:
      return
    buf, self.raw_buf = self.raw_buf, []
    self.raw_flush = wall or time.time()
    if not self.write:
      return
    name = 'raw-' + datetime.fromtimestamp(buf[0][0], UTC).strftime('%Y%m%d-%H') + '.jsonl.gz'
    try:
      # appending gzip members keeps each flush self-contained; gzip readers read them as one stream
      with gzip.open(os.path.join(self.dir, name), 'at') as f:
        f.writelines(json.dumps(r, separators=(',', ':')) + '\n' for r in buf)
    except OSError:
      pass

  # ---- per poll cycle ---------------------------------------------------------------------------
  def update(self, t: float, values: dict, age: dict, onroad: bool = False, cells: list | None = None,
             temps: list | None = None, quality: dict | None = None) -> None:
    """t is wall time. values/age come from TeslaSXState.snapshot(); cells is TeslaSXState.cell_set()
    (None unless the 96 bricks were read together)."""
    v = fresh(values, age)
    stale = sum(1 for k in SERIES_KEYS if k in values and k not in v)
    if self.session and t - self.session.d['end'] > LINK_GAP_S:
      # the link was down long enough that the car may have done anything meanwhile
      self.close(self.session.d['end'], 'link-gap')
    if self.modes.update(t, v, onroad):
      if self.session:
        self._append('sessions.jsonl', self.session.finish())
      self.session = Session(self.modes.mode, self.modes.since, v)
      self._save_open()
    if self.session:
      self.session.add(t, v, stale)
    for e in self.events.update(t, self.modes.mode, v):
      if self.session:
        self.session.d['events'].append({'t': e['t'], 'type': e['type']})
      self._append('events.jsonl', e)
    self._health(t, v, cells, temps)
    mode = self.modes.mode or 'PARKED'
    if t - self.last_series >= SERIES_DT.get(mode, 60.0):
      self.last_series = t
      row = {'t': round(t, 2), 'mode': self.modes.mode, **{k: v[k] for k in SERIES_KEYS if k in v}}
      if quality:
        row['q'] = quality
      if self.write:
        try:
          with open(os.path.join(self.dir, f'series-{_day(t)}.jsonl'), 'a') as f:
            f.write(json.dumps(row, separators=(',', ':')) + '\n')
        except OSError:
          pass
      if self.session and self.session.d['samples'] % 12 == 1:
        self._save_open()

  def _health(self, t: float, v: dict, cells: list | None, temps: list | None) -> None:
    cur = v.get('packCurrent')
    if cur is None or abs(cur) > REST_A or self.modes.mode not in ('PARKED', None):
      self.rest_since = None
      return
    self.rest_since = self.rest_since or t
    soc = v.get('socUI')
    due = t - self.last_health >= HEALTH_EVERY_S or (
      soc is not None and self.last_health_soc is not None and abs(soc - self.last_health_soc) >= 5)
    if t - self.rest_since < REST_HOLD_S or not due or cells is None:
      return
    vsum = sum(cells)
    low = min(range(96), key=lambda i: cells[i])
    snap = {'t': round(t, 2), 'soc': soc, 'socDisplay': v.get('socDisplay'), 'packVoltage': v.get('packVoltage'),
            'packCurrent': cur, 'cellsSumV': round(vsum, 2), 'cells': cells, 'temps': temps,
            'cellMin': min(cells), 'cellMax': max(cells), 'deltaMv': round((max(cells) - min(cells)) * 1000, 1),
            'lowIndex': low + 1, 'nominalFullPackEnergy': v.get('nominalFullPackEnergy'),
            'nominalEnergyRemaining': v.get('nominalEnergyRemaining'), 'energyBuffer': v.get('energyBuffer'),
            'kwhChargeTotal': v.get('kwhChargeTotal'), 'kwhDischargeTotal': v.get('kwhDischargeTotal'),
            'batteryOdometerMi': v.get('batteryOdometerMi'), 'moduleTempMin': v.get('moduleTempMin'),
            'moduleTempMax': v.get('moduleTempMax')}
    pv = v.get('packVoltage')
    # 96 bricks in series: their sum should be the pack voltage. A mismatch means a decode or mux fault.
    snap['sumCheck'] = None if pv is None else round(vsum / pv, 4)
    if snap['sumCheck'] is not None and abs(snap['sumCheck'] - 1) > 0.02:
      snap['flags'] = ['cell-sum-mismatch']
    self._append('health.jsonl', snap)
    self.last_health, self.last_health_soc = t, soc

  def close(self, t: float, reason: str = 'stopped') -> None:
    if self.session:
      self.session.d['end'] = max(self.session.d['end'], t if t else 0)
      self._append('sessions.jsonl', self.session.finish(reason))
      self.session = None
      self.modes = ModeMachine()
    self._save_open()
    self.flush_raw(t)


# ---- re-analysis of what was logged before this existed --------------------------------------------
def replay_live(path: str) -> dict:
  """Run live.jsonl (btobdd's ~1 line per publish) through the same logger in memory.

  Old lines predate the 0x102 offset fix, so pack voltage/current are decoded again from the raw
  0x102 frame each line carries. Lines have no per-value ages: a value counts as fresh on a line
  only if its raw frame or decoder output changed recently -- approximated by trusting every value
  on the line (they were at most one poll cycle old), except cells, which are never synchronised.
  """
  from openpilot.system.btobd.tesla_sx import bms_hv
  vl = VehicleLog('', write=False)
  last_t = None
  rows = 0
  try:
    f = open(path, errors='replace')
  except OSError:
    return {'sessions': [], 'events': [], 'health': [], 'rows': 0}
  with f:
    for line in f:
      try:
        r = json.loads(line)
      except ValueError:
        continue
      if r.get('event') != 'live':
        continue
      t = r['t']
      if last_t is not None and t - last_t > 300:
        vl.close(last_t, 'log-gap')
      last_t = t
      v = dict(r.get('values') or {})
      raw = (r.get('raw') or {}).get('102')
      if raw:
        try:
          v.update(bms_hv(bytes.fromhex(raw.replace(' ', '')), offset=None))
        except ValueError:
          pass
      v['cellSync'] = False
      vl.update(t, v, dict.fromkeys(v, 0.0))
      rows += 1
  if last_t is not None:
    vl.close(last_t, 'end-of-log')
  return {**vl.out, 'rows': rows}


def read_jsonl(path: str, n: int | None = None) -> list:
  out = []
  try:
    with open(path, errors='replace') as f:
      lines = f.readlines()
  except OSError:
    return out
  for ln in lines[-n:] if n else lines:
    try:
      out.append(json.loads(ln))
    except ValueError:
      pass
  return out


# ---- power history ---------------------------------------------------------------------------------
# The car's 12 V as the panda reports it, which says what the car was doing even when no dongle was
# linked: >= AWAKE_V the DC-DC runs (driving, charging, or awake), ~12.4-13 V asleep with the harness
# still powered (this car keeps it powered), ~5 V or none = no harness power (device out of the car
# or on a bench supply). Before power.jsonl existed, pandad's STATUS_PACKET in swaglog (~every 10 min)
# is the only record, so both are read.
AWAKE_V = 13.4
DEVICE_GAP_S = 25 * 60     # no record for this long = the device itself was off


def power_from_swaglog(log_dir: str) -> list:
  rows = []
  try:
    names = sorted(os.listdir(log_dir))
  except OSError:
    return rows
  for n in names:
    try:
      f = open(os.path.join(log_dir, n), errors='replace')
    except OSError:
      continue
    with f:
      for line in f:
        if 'STATUS_PACKET' not in line:
          continue
        try:
          j = json.loads(line)
          m = j['msg']
          p = (m.get('pandaStates$a') or [{}])[0]
          ds = (m.get('deviceState') or {}).get('deviceState', {})
        except (ValueError, KeyError, TypeError, AttributeError):
          continue
        t = j.get('created') or 0
        if t < 1_700_000_000:          # before the clock was set at boot
          continue
        v = p.get('voltage')
        rows.append({'t': round(t, 1), 'v': round(v / 1000, 2) if v else None, 'harness': p.get('harnessStatus'),
                     'ign': bool(p.get('ignitionLine') or p.get('ignitionCan')), 'started': bool(ds.get('started$b')),
                     'src': 'swaglog'})
  rows.sort(key=lambda r: r['t'])
  return rows


def power_state(r: dict) -> str:
  if r.get('started') or r.get('onroad'):
    return 'DRIVE'
  v = r.get('v')
  if v is None or v < 6.0:
    return 'NO_HARNESS'
  if v >= AWAKE_V or r.get('ign'):
    return 'AWAKE'
  return 'ASLEEP'


def power_segments(rows: list) -> list:
  """Consecutive records of the same state, split where the device left no record (it was off)."""
  segs: list = []
  prev_t = None
  for r in sorted(rows, key=lambda r: r['t']):
    st = power_state(r)
    gap = prev_t is not None and r['t'] - prev_t > DEVICE_GAP_S
    if gap:
      segs.append({'state': 'DEVICE_OFF', 'start': prev_t, 'end': r['t'], 'n': 0, 'vMin': None, 'vMax': None})
    s = segs[-1] if segs else None
    if s and s['state'] == st and not gap:
      s['end'] = r['t']
      s['n'] += 1
      if r.get('v') is not None:
        s['vMin'] = r['v'] if s['vMin'] is None else min(s['vMin'], r['v'])
        s['vMax'] = r['v'] if s['vMax'] is None else max(s['vMax'], r['v'])
    else:
      segs.append({'state': st, 'start': r['t'], 'end': r['t'], 'n': 1, 'vMin': r.get('v'), 'vMax': r.get('v')})
    prev_t = r['t']
  return segs
