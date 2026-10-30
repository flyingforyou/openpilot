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
