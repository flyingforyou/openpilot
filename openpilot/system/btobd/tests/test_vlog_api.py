"""Drive rows on /vlog: openpilot's route gives the drive's real time and distance, the dongle sessions
inside it only the share they covered (2026-10-28: three page-open fragments of a 30 min drive had
been shown as three 1-4 min drives of 1-2 mi)."""
import pytest
from pathlib import Path

from openpilot.selfdrive.debug import vlog_api


def frag(start, end, mi, out, inn=0.0):
  return {'type': 'DRIVE', 'start': start, 'end': end, 'durationS': end - start, 'miSpeed': mi, 'kwhOut': out,
          'kwhIn': inn, 'kwMin': -80.0, 'kwMax': 30.0, 'avgKw': -10.0, 'samples': 10, 'gapS': 0.0,
          'socStart': 66.0, 'socEnd': 65.0, 'source': 'live.jsonl 재분석'}


def test_fragments_become_one_partial_drive():
  route = {'route': 'r1', 'start': 1000.0, 'end': 2836.0, 'miles': 13.52, 'movingS': 1340}
  parts = [frag(1100, 1360, 1.17, 0.65, 0.19), frag(1700, 1900, 1.88, 0.90, 0.17), frag(2700, 2747, 0.02, 0.03, 0.01)]
  charge = {'type': 'AC_CHARGE', 'start': 5000.0, 'end': 6000.0, 'durationS': 1000.0}
  rows = vlog_api.merge_drives(parts + [charge], [route])
  assert len(rows) == 2 and rows[1] is charge
  d = rows[0]
  assert d['durationS'] == 1836 and d['miles'] == 13.52 and d['milesSource'] == 'openpilot' and d['parts'] == 3
  assert d['coverage'] == pytest.approx(507 / 1836, abs=1e-3)
  assert 'whPerMi' not in d and d['whPerMiPart'] == pytest.approx((1.58 - 0.37) * 1000 / 3.07, rel=1e-3)
  assert 'partial' in d['flags']


def test_drive_without_dongle_is_not_listed():
  assert vlog_api.merge_drives([], [{'route': 'r2', 'start': 0.0, 'end': 600.0, 'miles': 5.0, 'movingS': 500}]) == []


def test_fully_covered_drive_gets_wh_per_mile():
  route = {'route': 'r3', 'start': 0.0, 'end': 600.0, 'miles': 5.0, 'movingS': 500}
  d = vlog_api.merge_drives([frag(0, 600, 5.0, 1.6, 0.1)], [route])[0]
  assert d['coverage'] == 1.0 and d['whPerMi'] == pytest.approx(300.0)


def chg(start, end, kind='AC_CHARGE', kwh=1.0):
  return {'type': kind, 'start': start, 'end': end, 'durationS': end - start, 'kwhIn': kwh, 'kwhOut': 0.0,
          'coveredS': end - start, 'samples': 10, 'kwN': 10, 'avgKw': 6.0, 'kwMax': 6.2, 'kwMin': 5.8,
          'socStart': 50.0, 'socEnd': 51.0, 'flags': ['gappy']}


def test_charge_fragments_are_one_plug_in():
  rows = vlog_api.merge_charges([chg(0, 100, 'CHARGE_UNKNOWN'), chg(110, 500), chg(520, 900, 'CHARGE_UNKNOWN'),
                                 chg(5000, 5100)])
  assert len(rows) == 2
  m = rows[0]
  assert m['type'] == 'AC_CHARGE' and m['start'] == 0 and m['end'] == 900 and m['parts'] == 3
  assert m['kwhIn'] == 3.0 and m['gapS'] == pytest.approx(30) and 'merged' in m['flags']


def test_charge_is_not_merged_across_a_drive():
  drive = {'type': 'DRIVE', 'start': 110, 'end': 200}
  assert len(vlog_api.merge_charges([chg(0, 100), drive, chg(210, 300)])) == 3


def test_history_is_computed_once_per_cutoff(tmp_path, monkeypatch):
  monkeypatch.setattr(vlog_api, 'DISK_CACHE', str(tmp_path / 'c.json'))
  monkeypatch.setattr(vlog_api, '_cache', {})
  calls = []
  assert vlog_api._disk_cached('x', 100.0, lambda: calls.append(1) or [1, 2]) == [1, 2]
  vlog_api._cache.clear()                                   # a server restart: memory gone, disk kept
  assert vlog_api._disk_cached('x', 100.0, lambda: calls.append(1) or [9]) == [1, 2]
  assert vlog_api._disk_cached('x', 200.0, lambda: calls.append(1) or [3]) == [3]
  assert len(calls) == 2


def test_diag_summary_comes_from_the_logger_raw(tmp_path, monkeypatch):
  import os
  raw = os.path.join(os.path.dirname(__file__), 'data', 'raw_charge_20261030.jsonl.gz')
  (tmp_path / 'raw-20261030-14.jsonl.gz').write_bytes(Path(raw).read_bytes())
  monkeypatch.setattr(vlog_api, 'VLOG_DIR', str(tmp_path))
  monkeypatch.setattr(vlog_api, '_cache', {})
  out = vlog_api.recent_summary()
  assert out['source'] == 'vlog' and out['frames'] > 2000 and out['end'] - out['start'] <= 6 * 3600
  charge = next(g for g in out['groups'] if g['key'] == 'charge')
  rows = {r['key']: r for r in charge['rows']}
  assert rows['bmsState']['value'] == '완속 충전' and rows['pilot']['status'] == 'ok'
  assert 5.0 < float(rows['packPower']['value']) < 6.5


def _dir_with(tmp_path, monkeypatch, health=(), series=(), events=(), pack=''):
  import json
  import time
  d = tmp_path / 'vlog'
  d.mkdir()
  (d / 'health.jsonl').write_text(''.join(json.dumps(h) + '\n' for h in health))
  (d / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
  (d / ('series-' + time.strftime('%Y%m%d', time.gmtime()) + '.jsonl')).write_text(''.join(json.dumps(r) + '\n' for r in series))
  p = tmp_path / 'params'
  p.mkdir()
  if pack:
    (p / 'BtObdPackNewKwh').write_text(pack)
  monkeypatch.setattr(vlog_api, 'VLOG_DIR', str(d))
  monkeypatch.setattr(vlog_api, 'PARAMS_DIR', str(p))
  monkeypatch.setattr(vlog_api, '_cache', {})


RAW_FIXTURE = Path(__file__).parent / 'data' / 'raw_charge_20261030.jsonl.gz'


def test_every_check_has_min_max_and_a_verdict(tmp_path, monkeypatch):
  import time
  now = time.time()
  _dir_with(tmp_path, monkeypatch, pack='85.8',
            health=[{'t': now - 5400, 'deltaMv': 3.5}, {'t': now - 3600, 'deltaMv': 3.0}, {'t': now - 1800, 'deltaMv': 4.2}],
            events=[{'t': now - 100, 'type': 'CHARGE_STOPPED', 'context': {'acVolts1': 1.0, 'pilot': 0}, 'value': -5.2, 'threshold': 0.3},
                    {'t': now - 50, 'type': 'CHARGE_STOPPED', 'context': {'acVolts1': 230.0, 'pilot': 2}, 'value': 0.1, 'threshold': 0.3}])
  # the fixture's hour, renamed into the window
  (Path(vlog_api.VLOG_DIR) / ('raw-' + time.strftime('%Y%m%d-%H', time.gmtime(now)) + '.jsonl.gz')).write_bytes(RAW_FIXTURE.read_bytes())
  monkeypatch.setattr(vlog_api, 'STATS_CACHE', str(tmp_path / 'stats.json'))
  c = vlog_api.checks()
  rows = {r['key']: r for r in c['rows']}
  assert set(rows) == {x[1] for x in vlog_api.CHECKS}            # normal rows are listed too
  assert rows['packVoltage']['level'] == 'ok' and 360 < rows['packVoltage']['min'] < rows['packVoltage']['max'] < 385
  assert rows['restDeltaMv']['min'] == 3.0 and rows['restDeltaMv']['max'] == 4 and rows['restDeltaMv']['level'] == 'ok'
  assert 5.0 < rows['chargeKw']['max'] < 6.5 and rows['chargeKw']['level'] == 'info'
  assert rows['acVoltsCharge']['min'] >= 200 and rows['acVoltsCharge']['level'] == 'ok'
  assert rows['degradation']['level'] == 'warn' and rows['degradation']['max'] == pytest.approx(14.9, abs=0.1)
  assert rows['motorTemp']['level'] == 'na'                       # parked and charging: no drive frames
  assert -12.2 < rows['pilotLowCharge']['min'] <= rows['pilotLowCharge']['max'] < -11.0 and rows['pilotLowCharge']['level'] == 'ok'
  assert 200 < rows['rangeIdealFull']['min'] < 230 and rows['cycles']['max'] == pytest.approx(rows['kwhDischargeTotal']['max'] / 85.8, abs=1)
  # no charge session recorded a cause for these stops: neither is called normal
  assert [e['level'] for e in c['events']] == ['warn', 'warn'] and c['faults'] == []
  assert vlog_api.checks()['rows'] == c['rows']                   # second call from the stats cache


def test_out_of_range_values_are_flagged(tmp_path, monkeypatch):
  _dir_with(tmp_path, monkeypatch)
  agg = {'cellMin': [2.95, 3.9, 10, 1.0, 2.0], 'dcdcOutputVoltage': [12.8, 14.6, 10, 3.0, 4.0], 'moduleSpread': [0.5, 6.0, 10, 5.0, 6.0],
         'bad:bmsState': [7, 7, 2, 7.0, 8.0]}
  monkeypatch.setattr(vlog_api, '_stats', lambda days: {'agg': agg, 'files': 1, 'from': None})
  c = vlog_api.checks()
  rows = {r['key']: r for r in c['rows']}
  assert rows['cellMin']['level'] == 'bad' and rows['cellMin']['tWorst'] == 1.0
  assert rows['dcdcOutputVoltage']['level'] == 'warn' and rows['moduleSpread']['level'] == 'warn'
  assert c['faults'][0]['label'] == 'BMS 상태' and c['faults'][0]['n'] == 2
  assert c['count']['bad'] == 2


def test_pack_resistance_is_dv_over_di(tmp_path, monkeypatch):
  # 0x102: voltage 0.01 V, current 0.1 A signed 15-bit, a 75 mOhm pack stepping between 0 and -200 A
  def frame(v, a):
    raw = round(a * 10) & 0x7FFF
    return bytes([round(v * 100) & 0xFF, round(v * 100) >> 8, raw & 0xFF, raw >> 8, 0, 0, 0xFC, 0x1F])
  frames = []
  for k in range(60):
    a = -200.0 if k % 4 < 2 else 0.0
    frames.append((1000.0 + k, 1000.0 + k, 0x102, frame(380.0 + a * 0.075, a)))
  monkeypatch.setattr(vlog_api, '_raw_frames', lambda paths: iter(frames))
  st = vlog_api.file_stats('x')
  rec = st['_r']
  assert rec['mOhm'] == pytest.approx(75, abs=1) and rec['dI'] == pytest.approx(200) and rec['dV'] == pytest.approx(15, abs=0.1)
  assert rec['n'] >= vlog_api.R_MIN_PAIRS and 0 < rec['dt'] <= vlog_api.R_WINDOW_S
  # no module temperature was heard: recorded, but not trusted for the check row
  assert rec['conf'] == 'low' and '온도 모름' in rec['why'] and 'packResistance' not in st


def test_resistance_confidence():
  pairs = [(75 + (k % 5) - 2, 1000.0 + k, 15.0, 200.0, 2.0, 60.0, 22.0) for k in range(50)]
  assert vlog_api._r_record(pairs)['conf'] == 'high'
  assert vlog_api._r_record(pairs[:20])['conf'] == 'medium'
  noisy = [(r * (1 + (k % 2)), *rest) for k, (r, *rest) in enumerate(pairs)]
  assert vlog_api._r_record(noisy)['conf'] == 'low'


def test_unconfirmed_decodes_are_not_called_normal(tmp_path, monkeypatch):
  _dir_with(tmp_path, monkeypatch)
  agg = {'brickMaxV': [3.8, 3.9, 50, 1.0, 2.0], 'bmsMaxChargeA': [0.0, 250.0, 50, 1.0, 2.0], 'tpms1': [36.0, 39.0, 9, 1.0, 2.0],
         'cellMax': [3.9, 3.95, 2, 1.0, 2.0],
         'vrf:202c': [0.9, 0.99, 40, 1.0, 2.0],                                           # 40 ratios, none off
         'vrf:332v:max': [0.0, 120.0, 10, 1.0, 2.0], 'vrfmiss:332v:max': [0, 0, 4, 1.0, 2.0]}   # 4 of 10 off
  monkeypatch.setattr(vlog_api, '_stats', lambda days: {'agg': agg, 'files': 1, 'from': None})
  c = vlog_api.checks()
  rows = {r['key']: r for r in c['rows']}
  dec = {d['key']: d['status'] for d in c['decodes']}
  assert dec == {'332v': 'failed', '332t': 'insufficient', '202c': 'confirmed', '202d': 'insufficient'}
  assert rows['brickMaxV']['level'] == 'unverified' and '불일치' in rows['brickMaxV']['why']
  assert rows['bmsMaxChargeA']['level'] == 'info'
  assert rows['tpms1']['level'] == 'unverified' and rows['cellMax']['level'] == 'few'
  assert rows['degradation']['level'] == 'na'                                             # no new-pack figure set


def test_charge_stop_event_takes_the_sessions_cause(tmp_path, monkeypatch):
  import json
  import time
  now = time.time()
  _dir_with(tmp_path, monkeypatch, events=[{'t': now - 500, 'type': 'CHARGE_STOPPED', 'value': 0.1, 'threshold': 0.3},
                                           {'t': now - 100, 'type': 'CHARGE_STOPPED', 'value': 0.1, 'threshold': 0.3}])
  with open(Path(vlog_api.VLOG_DIR) / 'sessions.jsonl', 'w') as f:
    f.write(json.dumps({'type': 'AC_CHARGE', 'start': now - 4000, 'end': now - 480, 'endCause': 'unplugged',
                        'endWhy': ['커넥터 근접 = 분리됨'], 'endT': now - 510}) + '\n')
  monkeypatch.setattr(vlog_api, '_stats', lambda days: {'agg': {}, 'files': 0, 'from': None})
  c = vlog_api.checks()
  assert [(e['level'], e['note'].split(':')[0]) for e in c['events']] == [('info', '플러그 분리'), ('warn', '이 충전의 종료 원인 기록 없음 (원인 불명)')]
  assert c['chargeEnds'][0]['label'] == '플러그 분리'


def test_last_state_decodes_the_last_minutes(tmp_path, monkeypatch):
  import os
  raw = os.path.join(os.path.dirname(__file__), 'data', 'raw_charge_20261030.jsonl.gz')
  (tmp_path / 'raw-20261030-14.jsonl.gz').write_bytes(Path(raw).read_bytes())
  monkeypatch.setattr(vlog_api, 'VLOG_DIR', str(tmp_path))
  monkeypatch.setattr(vlog_api, '_cache', {})
  s = vlog_api.last_state()
  assert 360 < s['values']['packVoltage'] < 380 and s['values']['bmsState'] == 3
  assert s['age']['packVoltage'] < 15 and sum(c is not None for c in s['cellsV']) == 96
