"""Daily snapshots: one per local day, rebuildable from raw without duplicates, no value carried across
days, resistance compared only within its SOC/temperature bin, and gaps kept apart from warnings."""
import gzip
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.debug import daily, vlog_api
from openpilot.system.btobd import tdiag

RAW = Path(__file__).parent / 'data' / 'raw_charge_20261030.jsonl.gz'   # 12 min of AC charging, 10/30 07:4x PDT
DAY = '2026-10-30'


@pytest.fixture
def store(tmp_path, monkeypatch):
  v = tmp_path / 'vlog'
  v.mkdir()
  (v / 'raw-20261030-14.jsonl.gz').write_bytes(RAW.read_bytes())
  params = tmp_path / 'params'
  params.mkdir()
  (params / 'BtObdPackNewKwh').write_text('85.8')
  monkeypatch.setattr(vlog_api, 'PARAMS_DIR', str(params))
  monkeypatch.setattr(tdiag, 'LOG_DIR', str(tmp_path))        # no live.jsonl here
  return v, tmp_path / 'daily'


def build(store, now=1793400000.0, **kw):
  v, out = store
  return daily.ensure(str(v), str(out), now=now, **kw)


def test_day_from_raw(store):
  r = build(store)
  assert r['built'] == [DAY]
  s = daily.day(DAY, str(store[1]))
  assert s['status'] == 'PARTIAL' and any('진행 중' in w for w in s['statusWhy'])      # 'now' is still 10/30 there
  assert s['tz'] == 'America/Los_Angeles' and s['sources'] == ['raw'] and s['packId'].startswith('pack@')
  d = s['day']
  assert d['nominalFullPack']['max'] == pytest.approx(73.0, abs=0.2)
  assert d['soh']['value'] == pytest.approx(73.0 / 85.8 * 100, abs=0.3) and d['soh']['ref'] == 85.8
  # limits keep the conditions they were measured in
  lim = d['limits']['maxDischargePower']
  assert lim['min']['at']['mode'] in ('drive', 'charge') and lim['min']['at']['temp'] is not None   # the capture opens arriving
  assert set(lim['min']['at']) == {'soc', 'temp', 'i', 'mode', 'mph'}
  assert 5.0 < d['limits']['chargeKw']['max']['v'] < 6.5
  # one moment vs the whole day: the snapshot holds fresh values only, each with its own time
  snap = s['snapshot']
  assert snap['t'] is not None and all(x['ageS'] <= 90 for x in snap['values'].values())
  assert {'packVoltage', 'packCurrent'} <= set(snap['values'])
  assert d['reception']['freshness']['packVoltage']['samples'] > 0


def test_rebuild_is_idempotent_and_skips_unchanged_days(store):
  build(store)
  first = (store[1] / f'{DAY}.json').read_text()
  assert build(store)['built'] == []                          # nothing changed: nothing rewritten
  assert build(store, force=True)['built'] == [DAY]
  again = json.loads((store[1] / f'{DAY}.json').read_text())
  old = json.loads(first)
  for x in (again, old):
    x.pop('generatedAt')
  assert again == old
  assert len(list(store[1].glob('2026-*.json'))) == 1        # one file per day, never appended


def test_duplicate_raw_lines_are_dropped(store):
  build(store)
  base = daily.day(DAY, str(store[1]))
  v = store[0]
  with gzip.open(RAW, 'rt') as f:
    lines = f.readlines()
  with gzip.open(v / 'raw-20261030-14.jsonl.gz', 'wt') as f:   # the same frames written twice
    f.writelines(lines + lines[:500])
  build(store)
  dup = daily.day(DAY, str(store[1]))
  assert dup['coverage']['duplicateLines'] == base['coverage']['duplicateLines'] + 500
  assert dup['coverage']['frames'] == base['coverage']['frames']
  assert dup['day']['limits'] == base['day']['limits']


def test_missing_days_are_no_data_and_nothing_carries_over(store):
  v, out = store
  # the same 12 minutes moved two days later: 10/31 has nothing
  with gzip.open(RAW, 'rt') as f:
    rows = [json.loads(x) for x in f]
  with gzip.open(v / 'raw-20261101-14.jsonl.gz', 'wt') as f:
    for r in rows:
      r[0] += 2 * 86400
      f.write(json.dumps(r) + '\n')
  build(store, now=1793400000.0 + 3 * 86400)
  idx = daily.index(str(out))
  assert [(r['date'], r['status']) for r in idx['days']] == [('2026-10-30', 'PARTIAL'), ('2026-10-31', 'NO_DATA'),
                                                              ('2026-11-01', 'PARTIAL'), ('2026-11-02', 'NO_DATA')]
  gap = daily.day('2026-10-31', str(out))
  assert gap['snapshot'] is None and gap['day'] is None
  t = daily.trend(idx['days'])
  assert t['nominalFullPack']['n'] == 2 and t['nominalFullPack']['dayChange'] is None   # 10/30 -> 11/01 is not "the day before"


def test_partial_vs_complete_from_awake_time():
  tz = daily.zone()
  t0, _ = daily.day_bounds(DAY, tz)
  part = {'source': 'raw', 'frames': 10, 'dupes': 0, 'first': t0 + 60, 'last': t0 + 600, 'minutes': list(range(int(t0 // 60) + 1, int(t0 // 60) + 11)),
          'keys': {}, 'ext': {}, 'firstVals': {}, 'lastVals': {}, 'rest': [], 'restCells': None, 'pairs': [], 'vrf': {},
          'faults': {}, 'fresh': {}, 'cand': {}, 'snapT': t0 + 600}
  ctx = {'sessions': [], 'events': [], 'rx': [], 'packNewKwh': 0.0, 'today': '2099-01-01',
         'power': [{'t': t0 + 60 * k, 'v': 13.9} for k in range(10)]}
  assert daily.build_day(DAY, [part], tz, ctx)['status'] == 'OK'
  ctx['power'] = [{'t': t0 + 60 * k, 'v': 13.9} for k in range(60)]        # awake an hour, 10 min recorded
  s = daily.build_day(DAY, [part], tz, ctx)
  assert s['status'] == 'PARTIAL' and '60분 중 10분' in s['statusWhy'][0]
  assert daily.build_day(DAY, [], tz, ctx)['status'] == 'NO_DATA'


def test_resistance_is_kept_per_condition_bin():
  warm = [[75 + k % 3, 1000.0 + k, 7.5, 100.0, 5.0, 62.0, 24.0] for k in range(45)]
  cold = [[110 + k % 3, 2000.0 + k, 11.0, 100.0, 5.0, 62.0, 6.0] for k in range(20)]
  r = daily._resistance(warm + cold + [[90, 3000.0, 9.0, 100.0, 5.0, None, 20.0]])
  bins = {b['bin']: b for b in r['bins']}
  assert set(bins) == {'60|20', '60|5'} and r['unbinned'] == 1
  assert bins['60|20']['mOhm'] == pytest.approx(76, abs=1) and bins['60|5']['mOhm'] == pytest.approx(111, abs=1)
  assert r['headline']['bin'] == '60|20'                         # the most measured usable bin, not a pooled mean


def test_trend_compares_resistance_within_one_bin_only():
  rows = []
  for k in range(10):
    bins = {'60|20': [75.0 + k * 0.1, 'high', 50]}
    if k == 9:
      bins = {'60|5': [120.0, 'high', 60]}                       # a cold day: other conditions
    rows.append({'date': f'2026-11-{k + 1:02d}', 'packId': 'p', 'nominalFullPack': 73.0 - k * 0.01, 'rBins': bins})
  t = daily.trend(rows, 'p')
  assert t['rMohm']['bin'] == '60|20' and t['rMohm']['last'] == pytest.approx(75.8)   # the 120 is not in this series
  assert t['nominalFullPack']['dayChange'] == pytest.approx(-0.01) and t['nominalFullPack']['median7'] == pytest.approx(72.94)
  assert t['nominalFullPack']['slope30'] == pytest.approx(-0.3, abs=0.01)


def test_pack_id_changes_when_the_bms_counters_restart():
  def snap(odo, dis):
    return {'day': {'energy': {'counters': {'batteryOdometerMi': {'first': odo, 'last': odo + 10},
                                            'kwhDischargeTotal': {'first': dis, 'last': dis + 3}}}}}
  pid, _ = daily.assign_pack(None, snap(139197.6, 76264))
  assert pid == 'pack@139197mi'
  prev = {'packId': pid, 'packOdo': 139207.6, 'packDis': 76267}
  assert daily.assign_pack(prev, snap(139210.0, 76270))[0] == pid
  new, why = daily.assign_pack(prev, snap(12.0, 5))
  assert new == 'pack@12mi' and '다른 팩' in why
  # no odometer heard on the first day: named after lifetime charge energy instead
  first = {'day': {'energy': {'counters': {'kwhChargeTotal': {'first': 71921.6, 'last': 71927.4}}}}}
  assert daily.assign_pack(None, first)[0] == 'pack@71921kWh'
  assert daily.assign_pack({'packId': 'pack@71921kWh', 'packOdo': None, 'packDis': None}, {'day': None})[0] == 'pack@71921kWh'


def test_cell_balance_needs_enough_synchronised_rest_sweeps():
  rest = [{'t': k, 'mv': 5.0 + k, 'low': 90 if k < 2 else 54, 'high': 1, 'soc': 60.0, 'temp': 24.0, 'i': 0.3} for k in range(2)]
  assert daily._cell_balance(rest)['judged'] is False
  cb = daily._cell_balance(rest + [{**rest[0], 't': 9, 'low': 90}])
  assert cb['judged'] and cb['medianMv'] == 5.0 and cb['lowest'][0] == {'cell': 90, 'count': 3}
  assert daily._cell_balance([])['why'].startswith('휴지')


def test_days_before_raw_come_from_live_jsonl_marked_as_such(tmp_path):
  p = tmp_path / 'live.jsonl'
  t0 = 1793200000.0                                               # 2026-10-28 PDT
  lines = [{'t': t0 + k, 'event': 'live', 'values': {'nominalFullPackEnergy': 73.0, 'socDisplay': 60.0 - (k // 30) * 0.1,
                                                      'moduleTempMin': 20.0, 'moduleTempMax': 22.0}} for k in range(90)]
  p.write_text(''.join(json.dumps(x) + '\n' for x in lines))
  tz = daily.zone()
  days = daily.summarize_live(str(p), t0 + 60, tz)                # stops at the first raw frame
  (d, s), = days.items()
  assert d == '2026-10-28' and s['source'] == 'live.jsonl' and s['frames'] == 60
  assert s['keys']['socDisplay']['n'] == 2                        # a republished value is not a new reading
  ctx = {'sessions': [], 'events': [], 'rx': [], 'packNewKwh': 0.0, 'today': '2099-01-01', 'power': []}
  snap = daily.build_day(d, [s], tz, ctx)
  assert snap['status'] == 'PARTIAL' and any('live.jsonl' in w for w in snap['statusWhy'])
  assert snap['day']['cellBalance']['n'] == 0 and snap['day']['resistance']['pairs'] == 0 and snap['day']['decodes'] == []


def test_partial_days_do_not_feed_the_distance_trend():
  rows = [{'date': '2026-11-01', 'status': 'OK', 'miles': 30.0}, {'date': '2026-11-02', 'status': 'PARTIAL', 'miles': 2.0}]
  t = daily.trend(rows)
  assert t['miles']['last'] == 30.0 and t['miles']['n'] == 1 and t['miles']['note']
