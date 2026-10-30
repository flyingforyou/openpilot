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


def test_a_healthy_log_lists_only_degradation(tmp_path, monkeypatch):
  import time
  now = time.time()
  cells = [3.95 + (i % 4) * 0.001 for i in range(96)]
  _dir_with(tmp_path, monkeypatch, pack='85.8',
            health=[{'t': now - 3600, 'deltaMv': 3.0, 'cells': cells, 'lowIndex': 54, 'sumCheck': 1.0, 'nominalFullPackEnergy': 73.0}],
            series=[{'t': now - 60, 'moduleTempMin': 27.0, 'moduleTempMax': 28.2, 'dcdcOutputVoltage': 14.6, 'bmsState': 3,
                     'maxDischargePower': 340.0, 'socDisplay': 60.0}],
            # the 2026-10-30 unplug: AC gone, pilot off -- a normal end, not an anomaly
            events=[{'t': now - 100, 'type': 'CHARGE_STOPPED', 'context': {'acVolts1': 1.0, 'pilot': 0}, 'value': -5.2, 'threshold': 0.3}])
  items = vlog_api.anomalies()['items']
  assert [i['title'] for i in items] == ['배터리 열화'] and items[0]['level'] == 'warn' and '14.9%' in items[0]['detail']


def test_anomalies_are_found_and_ranked(tmp_path, monkeypatch):
  import time
  now = time.time()
  cells = [3.95] * 96
  cells[40] = 3.90
  _dir_with(tmp_path, monkeypatch,
            health=[{'t': now - 3600 * k, 'deltaMv': 50.0, 'cells': cells, 'lowIndex': 41, 'sumCheck': 1.0} for k in (5, 4, 3, 2, 1)],
            series=[{'t': now - 60, 'moduleTempMin': 30.0, 'moduleTempMax': 47.0, 'dcdcOutputVoltage': 12.6, 'bmsState': 7,
                     'maxDischargePower': 90.0, 'socDisplay': 60.0}],
            events=[{'t': now - 100, 'type': 'CHARGE_STOPPED', 'context': {'acVolts1': 230.0, 'pilot': 2}, 'value': 0.1, 'threshold': 0.3}])
  items = vlog_api.anomalies()['items']
  titles = {i['title'] for i in items}
  assert {'셀 편차 큼 (휴지·동시 측정)', '약한 셀', '같은 셀이 반복 최저', '배터리 모듈 고온', '모듈 온도 편차 큼', '12V 낮음 (DC-DC 출력)',
          'BMS 오류 상태', 'BMS 방전 한계 낮음', 'AC가 있는데 충전 멈춤'} <= titles
  levels = [i['level'] for i in items]
  assert levels == sorted(levels, key=lambda x: {'bad': 0, 'warn': 1}[x])
  assert next(i for i in items if i['title'] == '약한 셀')['detail'].startswith('#41')


def test_last_state_decodes_the_last_minutes(tmp_path, monkeypatch):
  import os
  raw = os.path.join(os.path.dirname(__file__), 'data', 'raw_charge_20261030.jsonl.gz')
  (tmp_path / 'raw-20261030-14.jsonl.gz').write_bytes(Path(raw).read_bytes())
  monkeypatch.setattr(vlog_api, 'VLOG_DIR', str(tmp_path))
  monkeypatch.setattr(vlog_api, '_cache', {})
  s = vlog_api.last_state()
  assert 360 < s['values']['packVoltage'] < 380 and s['values']['bmsState'] == 3
  assert s['age']['packVoltage'] < 15 and sum(c is not None for c in s['cellsV']) == 96
