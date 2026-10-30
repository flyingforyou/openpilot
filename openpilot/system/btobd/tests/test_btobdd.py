"""Poll order: pack power is read last in every cycle, so the logger's sample is fresh."""
from openpilot.system.btobd import btobdd, tesla_pt
from openpilot.system.btobd.tesla_sx import WATCH_IDS


def test_core_ids_come_last_and_power_is_final():
  for mode in ('DRIVE', 'AC_CHARGE', 'CHARGE_UNKNOWN', 'PARKED', None):
    for cycle in range(12):
      ids = btobdd.cycle_ids(mode, cycle, WATCH_IDS, tesla_pt.LIVE_IDS)
      core = btobdd.CORE_IDS['CHARGE' if mode and 'CHARGE' in mode else mode]
      assert ids[-len(core):] == core and ids[-1] == 0x102
      assert len(ids) == len(set(ids))


def test_rotation_reaches_everything():
  seen: set = set()
  for cycle in range(40):
    seen.update(btobdd.cycle_ids('AC_CHARGE', cycle, WATCH_IDS, tesla_pt.LIVE_IDS))
  assert set(WATCH_IDS) | set(tesla_pt.LIVE_IDS) <= seen


def test_charge_cycle_is_short():
  # 2026-10-30: 37 windows a cycle while charging; now the core four plus a handful
  assert max(len(btobdd.cycle_ids('AC_CHARGE', c, WATCH_IDS, tesla_pt.LIVE_IDS)) for c in range(20)) <= 10


def test_bms_limits_are_polled_while_driving():
  # their cross-check against 0x232 needs driving frames
  seen: set = set()
  for cycle in range(40):
    seen.update(btobdd.cycle_ids('DRIVE', cycle, WATCH_IDS, tesla_pt.LIVE_IDS))
  assert set(tesla_pt.BMS_LIMIT_IDS) <= seen and not set(tesla_pt.BMS_LIMIT_IDS) & set(btobdd.DISCOVERY)


def test_rx_stats_miss_interval_and_signal_freshness():
  from openpilot.system.btobd import rxstats
  from openpilot.system.btobd.vlog import max_age
  rx = rxstats.RxStats()
  for k in range(5):
    rx.poll(0x102, 3, 1000.0 + 4 * k, 10.0 + 4 * k)
    rx.poll(0x212, 1 if k < 2 else 0, 1000.0 + 4 * k, 10.0 + 4 * k)
  rx.poll(0x3F8, 0, 1020.0, 30.0)
  rx.learn(0x102, ['packVoltage', 'packCurrent'])
  rx.learn(0x212, ['bmsState'])
  out = rx.summary(1020.0, 30.0, {'packVoltage': 0.2, 'packCurrent': 7.0, 'bmsState': 12.0}, max_age)
  ids = {x['id']: x for x in out['ids']}
  assert ids['102']['status'] == 'ok' and ids['102']['gapMed'] == 4.0 and ids['102']['missRate'] == 0
  assert ids['212']['status'] == 'missing' and ids['212']['miss'] == 3 and ids['212']['missRate'] == 0.6
  assert ids['3F8']['status'] == 'never'
  sig = {x['key']: x['status'] for x in out['signals']}
  assert sig == {'packVoltage': 'fresh', 'packCurrent': 'stale', 'bmsState': 'missing'}
  assert rx.roll_hour(1000.0) == [] and len(rx.roll_hour(1000.0 + 3600)) == 3


def test_rx_stats_on_real_capture_timing():
  # every poll window of the 2026-10-30 raw fixture as one listen; raw only keeps windows that heard
  # something, so this checks intervals and freshness on real timing, not misses
  import gzip
  import json
  from pathlib import Path
  from openpilot.system.btobd import rxstats
  from openpilot.system.btobd.tesla_sx import TeslaSXState
  from openpilot.system.btobd.vlog import max_age
  rx, car, prev, n = rxstats.RxStats(), TeslaSXState(), None, 0
  with gzip.open(Path(__file__).parent / 'data' / 'raw_charge_20261030.jsonl.gz', 'rt') as f:
    rows = [json.loads(x) for x in f]
  for wall, mono, fid, data, window in rows:
    i = int(fid, 16)
    if window != prev:
      if prev:
        rx.poll(int(prev.split(':')[1], 16), n, last[0], last[1])
      prev, n = window, 0
    n += window.endswith(fid)
    car.feed(i, bytes.fromhex(data), mono)
    if i in tesla_pt.LIVE_IDS:
      tesla_pt.feed_live(car, i, bytes.fromhex(data), mono)
    rx.learn(i, [k for k, t in car.stamp.items() if t == mono])
    last = (wall, mono)
    if i == 0x102:
      at102 = (wall, mono)
  out = rx.summary(last[0], last[1], car.snapshot(last[1])['age'], max_age)
  ids = {x['id']: x for x in out['ids']}
  # this capture predates the poll-order fix: 0x102 came once per 8-12 s cycle, so pack power was
  # stale (> 5 s) between hearings -- the reason the 3 h charge had no power that day
  assert ids['102']['status'] == 'ok' and 5 < ids['102']['gapMed'] < 15
  sig = {x['key']: x for x in out['signals']}
  assert sig['packVoltage']['id'] == '102' and sig['packVoltage']['status'] == 'stale'
  assert sig['bmsState']['id'] == '212' and sig['cellMin']['id'] == '6F2'
  just = {x['key']: x['status'] for x in rx.summary(*at102, car.snapshot(at102[1])['age'], max_age)['signals']}
  assert just['packVoltage'] == 'fresh'
