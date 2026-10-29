"""Drive rows on /vlog: openpilot's route gives the drive's real time and distance, the dongle sessions
inside it only the share they covered (2026-10-28: three page-open fragments of a 30 min drive had
been shown as three 1-4 min drives of 1-2 mi)."""
import pytest

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


def test_drive_without_dongle_still_listed():
  rows = vlog_api.merge_drives([], [{'route': 'r2', 'start': 0.0, 'end': 600.0, 'miles': 5.0, 'movingS': 500}])
  assert rows[0]['source'] == 'openpilot (동글 기록 없음)' and rows[0]['coverage'] == 0 and 'kwhOut' not in rows[0]


def test_fully_covered_drive_gets_wh_per_mile():
  route = {'route': 'r3', 'start': 0.0, 'end': 600.0, 'miles': 5.0, 'movingS': 500}
  d = vlog_api.merge_drives([frag(0, 600, 5.0, 1.6, 0.1)], [route])[0]
  assert d['coverage'] == 1.0 and d['whPerMi'] == pytest.approx(300.0)
