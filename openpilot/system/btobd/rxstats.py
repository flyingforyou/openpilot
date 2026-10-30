"""Reception bookkeeping for every CAN ID the logger listens to and every signal decoded from it.

The dongle hears one ID at a time (ATCRA), so "not received" has a precise meaning here: the ID was
listened to for its dwell and no frame came. Per ID this keeps how often it was listened to and heard,
when it was last heard, the interval between hearings and the run of consecutive misses; per signal
which ID it comes from. btobdd writes summary() to rx.json every few seconds (what the page shows
while linked, and the last known state when not) and appends one line per ID per hour to rx.jsonl.

Statuses, the same words the page uses:
  ID      ok (heard on the last poll or within MISS_N polls) / missing (MISS_N or more polls in a row
          with nothing) / never (listened to, never heard)
  signal  fresh (age within vlog.max_age for that key) / stale (older) / missing (its ID is missing)
          / never (no value yet)
"""
from __future__ import annotations

import json
import os
import statistics
from collections import deque

MISS_N = 3          # consecutive empty polls before an ID counts as missing
GAPS_KEEP = 60


class RxStats:
  def __init__(self):
    self.ids: dict[int, dict] = {}
    self.keys: dict[str, int] = {}       # signal -> the ID it was last decoded from
    self.hour: dict[int, dict] = {}      # per-ID counters since the top of the hour
    self.hour_start: float | None = None

  def poll(self, msg_id: int, nframes: int, wall: float, mono: float) -> None:
    """One listen to msg_id that returned nframes frames."""
    r = self.ids.get(msg_id)
    if r is None:
      r = self.ids[msg_id] = {'polls': 0, 'hits': 0, 'frames': 0, 'lastRx': None, 'lastRxMono': None,
                              'lastPoll': None, 'miss': 0, 'maxMiss': 0, 'gaps': deque(maxlen=GAPS_KEEP)}
    h = self.hour.setdefault(msg_id, {'polls': 0, 'hits': 0, 'frames': 0, 'gapMax': 0.0, 'maxMiss': 0})
    r['polls'] += 1
    h['polls'] += 1
    r['lastPoll'] = wall
    if nframes:
      if r['lastRxMono'] is not None:
        gap = mono - r['lastRxMono']
        r['gaps'].append(gap)
        h['gapMax'] = max(h['gapMax'], round(gap, 1))
      r['hits'] += 1
      h['hits'] += 1
      r['frames'] += nframes
      h['frames'] += nframes
      r['lastRx'], r['lastRxMono'], r['miss'] = wall, mono, 0
    else:
      r['miss'] += 1
      r['maxMiss'] = max(r['maxMiss'], r['miss'])
      h['maxMiss'] = max(h['maxMiss'], r['miss'])

  def learn(self, msg_id: int, keys) -> None:
    for k in keys:
      self.keys[k] = msg_id

  @staticmethod
  def id_status(r: dict) -> str:
    if not r['hits']:
      return 'never'
    return 'missing' if r['miss'] >= MISS_N else 'ok'

  def summary(self, wall: float, mono: float, age: dict, max_age) -> dict:
    """The JSON rx.json holds. age is TeslaSXState.snapshot()['age']; max_age(key) the freshness
    limit vlog applies to that key."""
    ids = []
    for i, r in sorted(self.ids.items()):
      gaps = list(r['gaps'])
      ids.append({'id': f'{i:03X}', 'polls': r['polls'], 'hits': r['hits'], 'frames': r['frames'],
                  'missRate': round(1 - r['hits'] / r['polls'], 3) if r['polls'] else None,
                  'lastRx': r['lastRx'], 'sinceS': round(mono - r['lastRxMono'], 1) if r['lastRxMono'] is not None else None,
                  'gapMed': round(statistics.median(gaps), 1) if gaps else None, 'gapMax': round(max(gaps), 1) if gaps else None,
                  'miss': r['miss'], 'maxMiss': r['maxMiss'], 'status': self.id_status(r)})
    by_id = {x['id']: x for x in ids}
    sigs = []
    for k, i in sorted(self.keys.items()):
      a = age.get(k)
      lim = max_age(k)
      ir = by_id.get(f'{i:03X}')
      if a is None:
        st = 'never'
      elif ir and ir['status'] == 'missing':
        st = 'missing'
      else:
        st = 'fresh' if a <= lim else 'stale'
      sigs.append({'key': k, 'id': f'{i:03X}', 'ageS': a, 'maxAgeS': lim, 'status': st,
                   'gapMed': ir['gapMed'] if ir else None, 'gapMax': ir['gapMax'] if ir else None})
    return {'t': wall, 'ids': ids, 'signals': sigs, 'missN': MISS_N}

  def roll_hour(self, wall: float) -> list:
    """Per-ID lines for the hour that just ended (empty when the hour has not turned)."""
    hour = wall - wall % 3600
    if self.hour_start is None:
      self.hour_start = hour
    if hour == self.hour_start:
      return []
    out = [{'t': self.hour_start, 'id': f'{i:03X}', **h} for i, h in sorted(self.hour.items())]
    self.hour, self.hour_start = {}, hour
    return out


def write(directory: str, summary: dict, hour_lines: list) -> None:
  try:
    tmp = os.path.join(directory, 'rx.json.tmp')
    with open(tmp, 'w') as f:
      json.dump(summary, f, separators=(',', ':'))
    os.chmod(tmp, 0o644)
    os.replace(tmp, os.path.join(directory, 'rx.json'))
    if hour_lines:
      with open(os.path.join(directory, 'rx.jsonl'), 'a') as f:
        f.writelines(json.dumps(x, separators=(',', ':')) + '\n' for x in hour_lines)
  except OSError:
    pass
