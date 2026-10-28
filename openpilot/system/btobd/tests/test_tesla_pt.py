"""Charging/thermal/identity summary against frames from the 2026-10-27 parked, AC-charging capture.

The expected values are what the car was doing then (J1772 state C, 206 V single phase, heater off),
so a failure means a field moved. The VIN here is made up; the real capture is not in the repo.
"""
from datetime import datetime, timezone

import pytest

from openpilot.system.btobd import tesla_pt as p


def b(s):
  return bytes(int(x, 16) for x in s.split())


REAL = {
  0x212: 'DC 08 32 54 04', 0x77D: '12 00 11 1A 21', 0x207: '00 CE DA 1C 68 C7 61 DB',
  0x51F: '6A 08 E8 F3 B5 5A 02', 0x21C: '41 5A 40 09 8D 2A 40 24', 0x42C: '00 C4 8C 5A FE 00 00 00',
  0x25C: '00 02 00 09 00 F0 21 82', 0x20E: '01 56 22 10 C6 01 0A 08', 0x26A: 'FC 01 00',
  0x32A: '1C 1C 1C 64 7F 00 00 1E', 0x20A: '00 00 00 00 90 1F 03 03', 0x33A: '06 82 7A 29 69 00 00 00',
  0x3F8: 'CB 02 E1 02 B3 02 C8 02', 0x102: 'EA 8F 9B 00 A4 00 FC 1F',
}


def sig(key):
  return next(s for s in p.CHARGE + p.THERMAL + p.CANDIDATES if s.key == key)


@pytest.mark.parametrize('key,want', [
  ('bmsState', 3), ('pilot', 2), ('proximity', 3), ('phase', 0), ('hvil1', 1), ('fcContactor', 0), ('portType', 0),
  ('heaterState', 0),
])
def test_enum_fields(key, want):
  s = sig(key)
  assert p.decode(s, b(REAL[s.msg])) == want


@pytest.mark.parametrize('key,want', [
  ('pilotHigh', 6.02), ('pilotLow', -11.85), ('acVolts1', 206), ('heaterTemp', 23.5), ('pump1', 28),
  ('refrigPressure', 3.875), ('dischargeTemp', 24.75), ('suctionTemp', 21.0),
  ('tpms1', 44.6875),
])
def test_numeric_fields(key, want):
  s = sig(key)
  assert p.decode(s, b(REAL[s.msg])) == pytest.approx(want)


def test_mux_and_sna_and_short_frames():
  pilot = sig('pilotHigh')
  assert p.decode(pilot, b('69 08 E8 F3 B5 5A 02')) is None      # mux 1 = proximity, not pilot
  assert p.decode(sig('pump1'), b('7F 1C 1C 64 7F 00 00 1E')) is None  # 127 = SNA
  assert p.decode(sig('acVolts1'), b('00')) is None


def test_summary_rows_and_warnings():
  frames = [(float(k), i, b(d)) for k, (i, d) in enumerate(REAL.items())]
  frames.append((20.0, 0x212, b('DC 08 72 54 04')))                # BMS_FAULT appears once
  out = p.summarize(frames, {'saved': 100.0})
  charge = {r['key']: r for r in out['groups'][0]['rows']}
  assert charge['bmsState']['value'] == '폴트' and charge['bmsState']['status'] == 'bad'
  assert '완속 충전 → 폴트' in charge['bmsState']['detail']
  assert charge['pilot']['value'] == '완속 충전'
  assert float(charge['packPower']['value']) == pytest.approx(5.68, abs=0.05)
  assert float(charge['acCurrent']['value']) == pytest.approx(28, abs=1)
  assert charge['acVolts2']['status'] == 'na'                         # not in these frames


def test_vin_is_masked_and_decoded():
  vin = 'TESXC0000GF123456'
  frames = [(0.0, 0x508, bytes([k]) + vin[k * 7:k * 7 + 7].ljust(7, '\0').encode()) for k in range(3)]
  out = p.summarize(frames, {})
  row = next(r for r in out['groups'][2]['rows'] if r['key'] == 'vin')
  assert row['value'] == 'TESXC0000GF••••••'
  assert '123456' not in str(out)
  assert row['detail'] == 'Model X · 2016년식 · 프리몬트'


def test_vin_needs_all_three_parts():
  out = p.summarize([(0.0, 0x508, b'\x00TESTX00')], {})
  assert next(r for r in out['groups'][2]['rows'] if r['key'] == 'vin')['status'] == 'na'


def test_car_clock_skew():
  # car clock 1793116253 at t=110 of a capture saved at 1793116257.25 that ended at t=112
  frames = [(110.0, 0x528, (1793116253).to_bytes(4, 'big')), (112.0, 0x212, b(REAL[0x212]))]
  row = next(r for r in p.summarize(frames, {'saved': 1793116257.25})['groups'][2]['rows'] if r['key'] == 'clock')
  assert '-2 초' in row['detail'] and row['status'] == 'ok'
  assert row['ts'] == 1793116253
  assert row['value'] == datetime.fromtimestamp(1793116253, timezone.utc).strftime('%m/%d %H:%M:%S')


def test_solar_and_heading():
  # 2026-10-27 15:50 UTC: azimuth 119, car-relative -38, sun up, elevation 14 -> heading 157 (SSE)
  out = p.summarize([(0.0, 0x246, b('77 00 DA 03 0E'))], {})
  rows = {r['key']: r for r in next(g for g in out['groups'] if g['key'] == 'where')['rows']}
  assert (rows['sunAzimuth']['value'], rows['sunCarRef']['value'], rows['sunElevation']['value']) == ('119', '-38', '14')
  assert rows['heading']['value'] == '157' and rows['heading']['detail'].startswith('남남동')
  # a chassis-bus drive frame: azimuth 104, car-relative 61, GPS bearing 43
  out = p.summarize([(0.0, 0x246, b('68 00 3D 00 FE'))], {})
  rows = {r['key']: r for r in next(g for g in out['groups'] if g['key'] == 'where')['rows']}
  assert rows['heading']['value'] == '43' and rows['sunElevation']['value'] == '-2'
