#!/usr/bin/env python3
"""What the longitudinal settings were on each recorded drive, and where they changed.

Every route's segment 0 carries an `initData.params` snapshot taken at boot, so the settings a
drive actually ran are in the log rather than a matter of memory. This prints them in
chronological order and marks every value that moved from the drive before it.

  ./param_history.py op-logs op-logs-carrotlong
"""
import sys
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader

WATCH = ('CarrotLongEnabled', 'TeslaStockLong', 'LongitudinalPersonality', 'GapProfile',
         'TFollowGap1', 'TFollowGap2', 'TFollowGap3', 'TFollowGap4', 'TFollowGap5',
         'TFollowGap6', 'TFollowGap7', 'TFollowDecelBoost', 'TFollowRiseRatePct',
         'DynamicTFollow', 'EnableSpeedTF', 'ComfortBrake', 'ComfortBrake2',
         'StopDistanceCarrot', 'StopDistanceCm', 'StoppingAccel', 'LongActuatorDelay',
         'LongTuningKpV', 'LongTuningKiV', 'LongTuningKf', 'DrivingModel', 'LaneCentering')


def snapshot(seg0: Path):
  """(wall time, params) from a route's first segment, or None."""
  params, wall = None, None
  for msg in LogReader(str(seg0 / 'rlog.zst')):
    w = msg.which()
    if w == 'initData' and params is None:
      params = {e.key: bytes(e.value).decode(errors='replace') for e in msg.initData.params.entries}
    elif w == 'clocks' and wall is None:
      wall = msg.clocks.wallTimeNanos
    if params is not None and wall is not None:
      break
  return None if params is None else (wall, params)


def main(roots):
  routes = {}
  for root in roots:
    for seg0 in sorted(Path(root).glob('*--0')):
      routes[seg0.name[:-3]] = seg0

  rows = []
  for name, seg0 in routes.items():
    try:
      snap = snapshot(seg0)
    except Exception as e:
      print(f"  {name}: unreadable ({e})")
      continue
    if snap is None:
      continue
    wall, params = snap
    rows.append((wall or 0, name, params))
  rows.sort()

  import datetime
  prev = None
  for wall, name, params in rows:
    # device clock is UTC; the user drives US Pacific
    when = (datetime.datetime.fromtimestamp(wall / 1e9, datetime.UTC) -
            datetime.timedelta(hours=7)).strftime('%m-%d %H:%M') if wall else '     ?    '
    cur = {k: params.get(k, '-') for k in WATCH}
    if prev is None:
      print(f"\n=== {name}  {when} local  (first snapshot) ===")
      for k in WATCH:
        print(f"    {k:26} {cur[k]}")
    else:
      diff = {k: (prev[k], cur[k]) for k in WATCH if prev[k] != cur[k]}
      mark = '  <-- CHANGED' if diff else ''
      print(f"\n=== {name}  {when} local ==={mark}")
      if not diff:
        print("    (identical to the drive before)")
      for k, (a, b) in diff.items():
        print(f"    {k:26} {a}  ->  {b}")
    prev = cur


if __name__ == '__main__':
  main(sys.argv[1:] or ['op-logs'])
