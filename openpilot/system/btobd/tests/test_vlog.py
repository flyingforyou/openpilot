"""The always-on logger, replayed without a car: modes, energy, freshness, cell synchronisation,
events, restart recovery, raw frames, and a real 2026-10-27 charging capture end to end."""
import gzip
import json
import os

import pytest

from openpilot.system.btobd import tesla_pt, vlog
from openpilot.system.btobd.tesla_sx import TeslaSXState

DATA = os.path.join(os.path.dirname(__file__), 'data', 'charge_20261027.jsonl')


def run(vl, rows, step=1.0, t0=1_800_000_000.0, onroad=False):
  """rows: list of value dicts, one per `step` seconds, all fresh."""
  t = t0
  for v in rows:
    vl.update(t, v, dict.fromkeys(v, 0.0), onroad=onroad)
    t += step
  return t


def test_mode_needs_hold_and_agreeing_signals():
  m = vlog.ModeMachine()
  assert not m.update(0, {'gear': 'D', 'vehicleSpeedMph': 20})
  assert m.update(3, {'gear': 'D', 'vehicleSpeedMph': 20}) and m.mode == 'DRIVE' and m.since == 0
  # a 10 s blip of P (e.g. a dropped 0x116 decode) does not end the drive
  for t in range(4, 14):
    m.update(t, {'gear': 'P', 'packCurrent': -1})
  assert m.mode == 'DRIVE'
  for t in range(14, 50):
    m.update(t, {'gear': 'P', 'packCurrent': -1})
  assert m.mode == 'PARKED'


@pytest.mark.parametrize('v,want', [
  ({'gear': 'P', 'bmsState': 3, 'pilot': 2, 'packCurrent': 15}, 'AC_CHARGE'),
  ({'gear': 'P', 'bmsState': 4, 'packCurrent': 200}, 'DC_CHARGE'),
  ({'gear': 'P', 'bmsState': 3, 'fcContactor': 1, 'packCurrent': 200}, 'DC_CHARGE'),
  ({'gear': 'P', 'packCurrent': 15}, 'CHARGE_UNKNOWN'),          # current alone: charging, kind unknown
  ({'gear': 'D', 'packCurrent': 60}, 'DRIVE'),                    # regen in D is not charging
  ({'packCurrent': -2}, 'PARKED'),
  ({}, None),
])
def test_mode_candidates(v, want):
  assert vlog.ModeMachine.candidate(v, False) == want


def test_charge_energy_is_integrated_and_signed():
  vl = vlog.VehicleLog('', write=False)
  rows = [{'gear': 'P', 'bmsState': 3, 'pilot': 2, 'packVoltage': 400.0, 'packCurrent': 25.0, 'socUI': 50 + i / 360}
          for i in range(3600)]
  t = run(vl, rows)
  vl.close(t)
  s = vl.out['sessions'][-1]
  assert s['type'] == 'AC_CHARGE'
  # 10 kW for the hour minus the 20 s hold before the session opened
  assert s['kwhIn'] == pytest.approx(10 * (3600 - 21) / 3600, rel=1e-3)
  assert s['kwhOut'] == 0 and s['avgKw'] == pytest.approx(10.0)
  assert s['curve']['55'] == pytest.approx(10.0)


def test_gap_is_not_integrated():
  vl = vlog.VehicleLog('', write=False)
  base = {'gear': 'P', 'packVoltage': 400.0, 'packCurrent': 25.0}
  t = run(vl, [base] * 60)
  t = run(vl, [base] * 60, t0=t + 120)        # 2 min with nothing heard
  vl.close(t)
  s = vl.out['sessions'][-1]
  assert s['gapS'] == pytest.approx(121, abs=1)
  # integrated from the switch at +20 s to +59 s, then +180 s to +239 s: 39 + 59 s at 10 kW
  assert s['kwhIn'] == pytest.approx(10 * 98 / 3600, rel=1e-3)


def test_stale_values_are_not_samples():
  vl = vlog.VehicleLog('', write=False)
  t = 1_800_000_000.0
  for i in range(100):
    vl.update(t + i, {'gear': 'P', 'packVoltage': 400.0, 'packCurrent': 25.0}, {'gear': 0, 'packVoltage': 30.0, 'packCurrent': 30.0})
  vl.close(t + 100)
  # gear fresh -> PARKED; the 30 s old voltage/current must not be integrated or start a charge
  assert all(s['type'] == 'PARKED' for s in vl.out['sessions'])
  assert vl.out['sessions'][-1]['kwhIn'] == 0


def test_drive_wh_per_mile_and_regen():
  vl = vlog.VehicleLog('', write=False)
  rows = [{'gear': 'D', 'vehicleSpeedMph': 36.0, 'packVoltage': 360.0, 'packCurrent': -50.0 if i % 60 < 50 else 50.0}
          for i in range(600)]
  t = run(vl, rows)
  vl.close(t)
  s = vl.out['sessions'][-1]
  assert s['type'] == 'DRIVE' and s['milesSource'] == 'speed'
  assert s['miles'] == pytest.approx(36 * 596 / 3600, rel=0.02)
  assert s['regenKwh'] > 0 and s['kwhOut'] > s['kwhIn']
  assert s['whPerMi'] == pytest.approx((s['kwhOut'] - s['kwhIn']) * 1000 / s['miles'], rel=1e-3)


def test_carried_over_odometer_is_rejected():
  vl = vlog.VehicleLog('', write=False)
  rows = [{'gear': 'D', 'vehicleSpeedMph': 30.0, 'packVoltage': 360.0, 'packCurrent': -30.0,
           'batteryOdometerMi': 100.0 if i < 50 else 122.0} for i in range(100)]
  t = run(vl, rows)
  vl.close(t)
  s = vl.out['sessions'][-1]
  assert s['milesSource'] == 'speed' and 'odometer-inconsistent' in s['flags']


def test_cells_from_different_times_are_not_one_measurement():
  st = TeslaSXState()
  frame = bytes([0, 0x4C, 0x0D, 0x67, 0xC3, 0x4C, 0x0D, 0x67])     # any valid brick frame body
  for idx in range(24):
    st.feed(0x6F2, bytes([idx]) + frame[1:], 0.0 if idx < 12 else 20.0)
  assert st.cell_set(20.0) is None and st.values['cellSync'] is False and st.values['cellSpanS'] == 20.0
  for idx in range(24):
    st.feed(0x6F2, bytes([idx]) + frame[1:], 30.0 + idx * 0.1)
  assert st.cell_set(33.0) is not None and st.values['cellSync'] is True


def test_health_snapshot_only_at_rest_with_synced_cells():
  vl = vlog.VehicleLog('', write=False)
  cells = [3.80 + (i % 7) * 0.001 for i in range(96)]
  rest = {'gear': 'P', 'packVoltage': sum(cells), 'packCurrent': -1.0, 'socUI': 60.0}
  t = 1_800_000_000.0
  for i in range(200):
    vl.update(t + i, rest, dict.fromkeys(rest, 0.0), cells=None)
  assert not vl.out['health']                      # never synced
  for i in range(200, 300):
    vl.update(t + i, rest, dict.fromkeys(rest, 0.0), cells=cells)
  assert len(vl.out['health']) == 1
  h = vl.out['health'][0]
  assert h['deltaMv'] == pytest.approx(6.0) and h['sumCheck'] == pytest.approx(1.0) and h['lowIndex'] == 1
  loaded = dict(rest, packCurrent=-80.0)
  for i in range(300, 400):
    vl.update(t + i, loaded, dict.fromkeys(loaded, 0.0), cells=cells)
  assert len(vl.out['health']) == 1                # not at rest


def test_events_confirm_and_keep_context():
  vl = vlog.VehicleLog('', write=False)
  charging = {'gear': 'P', 'bmsState': 3, 'pilot': 2, 'packVoltage': 380.0, 'packCurrent': 15.0, 'dcdcOutputVoltage': 14.5}
  t = run(vl, [charging] * 60)
  t = run(vl, [dict(charging, packCurrent=0.2, dcdcOutputVoltage=11.5)] * 40, t0=t)
  types = [e['type'] for e in vl.out['events']]
  assert 'CHARGE_STOPPED' in types and 'LOW_12V' in types
  e = next(e for e in vl.out['events'] if e['type'] == 'CHARGE_STOPPED')
  assert e['before'] and len(e['after']) >= 19
  # one 11.5 V sample alone is not an event
  vl2 = vlog.VehicleLog('', write=False)
  run(vl2, [charging] * 30 + [dict(charging, dcdcOutputVoltage=11.5)] + [charging] * 40)
  assert not vl2.out['events']


def test_restart_closes_the_open_session(tmp_path):
  vl = vlog.VehicleLog(str(tmp_path))
  run(vl, [{'gear': 'D', 'vehicleSpeedMph': 30, 'packVoltage': 360.0, 'packCurrent': -40.0}] * 30)
  assert (tmp_path / 'open.json').exists()
  vlog.VehicleLog(str(tmp_path))                   # the daemon came back after a crash
  s = [json.loads(ln) for ln in open(tmp_path / 'sessions.jsonl')]
  assert s[-1]['type'] == 'DRIVE' and 'interrupted' in s[-1]['flags']


def test_series_and_raw_on_disk(tmp_path):
  vl = vlog.VehicleLog(str(tmp_path))
  t0 = 1_800_000_000.0
  for i in range(20):
    vl.frame(t0 + i, 100.0 + i, 0x102, bytes.fromhex('EA8F9B00A400FC1F'), f'{i}:102')
  vl.flush_raw()
  run(vl, [{'gear': 'P', 'packVoltage': 368.0, 'packCurrent': -1.0, 'stale': 1}] * 70, t0=t0)
  raw = [n for n in os.listdir(tmp_path) if n.startswith('raw-')]
  rows = [json.loads(ln) for ln in gzip.open(tmp_path / raw[0], 'rt')]
  assert rows[0] == [t0, 100.0, '102', 'EA8F9B00A400FC1F', '0:102'] and len(rows) == 20
  series = [json.loads(ln) for n in os.listdir(tmp_path) if n.startswith('series-') for ln in open(tmp_path / n)]
  assert 2 <= len(series) <= 3 and 'stale' not in series[0]      # only known keys, PARKED every 60 s


def test_replay_redecodes_old_current(tmp_path):
  # a live.jsonl line from before the 0x102 offset fix: -984.6 A stored, raw frame says +15.4 A
  p = tmp_path / 'live.jsonl'
  with open(p, 'w') as f:
    for i in range(120):
      f.write(json.dumps({'t': 1_793_114_905.0 + i * 3.7, 'event': 'live',
                          'values': {'packVoltage': 366.5, 'packCurrent': -984.6, 'socUI': 52.0},
                          'raw': {'102': 'EA 8F 9A 00 A0 00 FC 1F'}}) + '\n')
  r = vlog.replay_live(str(p))
  s = r['sessions'][-1]
  assert s['type'] == 'CHARGE_UNKNOWN' and s['kwhOut'] == 0 and s['kwhIn'] > 0


def test_power_segments():
  rows = [{'t': 0, 'v': 12.5}, {'t': 600, 'v': 12.6}, {'t': 1200, 'v': 14.5}, {'t': 1800, 'v': 14.6},
          {'t': 9000, 'v': 14.5}, {'t': 9600, 'v': 5.0}, {'t': 10200, 'v': 13.0, 'started': True}]
  segs = vlog.power_segments(rows)
  assert [s['state'] for s in segs] == ['ASLEEP', 'AWAKE', 'DEVICE_OFF', 'AWAKE', 'NO_HARNESS', 'DRIVE']


def test_real_charging_capture_end_to_end():
  """2026-10-27, parked on a 206 V single-phase charger: every decoder and the mode logic together."""
  st = TeslaSXState()
  vl = vlog.VehicleLog('', write=False)
  frames = []
  with open(DATA) as f:
    for ln in f:
      d = json.loads(ln)
      if 'id' in d:
        frames.append((d['t'], int(d['id'], 16), bytes.fromhex(d['data'])))
  t0 = 1_793_116_137.0
  last = 0.0
  for t, i, d in frames:
    st.feed(i, d, t)
    if i in tesla_pt.LIVE_IDS:
      tesla_pt.feed_live(st, i, d, t)
    if t - last >= 1.0:
      snap = st.snapshot(t)
      vl.update(t0 + t, snap['values'], snap['age'], cells=st.cell_set(t))
      last = t
  v = st.values
  assert v['bmsState'] == 3 and v['pilot'] == 2 and v['proximity'] == 3 and v['hvil1'] == 1
  assert v['packCurrent'] == pytest.approx(15.5, abs=0.5) and 360 < v['packVoltage'] < 375
  assert vl.modes.mode == 'AC_CHARGE'
  vl.close(t0 + frames[-1][0])
  s = vl.out['sessions'][-1]
  assert s['type'] == 'AC_CHARGE' and s['kwhOut'] == 0
  assert s['avgKw'] == pytest.approx(5.7, abs=0.3)
