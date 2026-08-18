#!/usr/bin/env python3
"""Are the ultrasonics awake while driving, and is anything on the bus carrying their readings?

The Model X has twelve ultrasonic sensors, but park assist is not obviously a moving-car system.
This answers it from recorded CAN rather than from assumption: it decodes PARK_status2 -- the one
park message the DBC knows -- against speed, and separately surveys every id on the bus for one
that behaves like a proximity feed (present at speed, and actually changing).

  ./uss_survey.py op-logs/0000006e--2ad4d92ec7--*
"""
import sys
from collections import defaultdict

import numpy as np

from openpilot.tools.lib.logreader import LogReader

PARK_STATUS2 = 782      # 0x30E
SPEED_BINS = [(0, 1), (1, 5), (5, 10), (10, 20), (20, 100)]   # mph


def bits(payload: bytes, start: int, length: int) -> int:
  """Little-endian (Intel) signal extraction, the @1 layout in the DBC."""
  raw = int.from_bytes(payload.ljust(8, b'\x00')[:8], 'little')
  return (raw >> start) & ((1 << length) - 1)


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)

  v_mph = 0.0
  park_by_bin = defaultdict(lambda: defaultdict(list))
  seen = defaultdict(lambda: {'n': 0, 'bus': set(), 'vals': set(), 'len': 0,
                              'moving': 0, 'moving_vals': set()})

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'carState':
        v_mph = float(msg.carState.vEgo) * 2.23694
      elif w == 'can':
        moving = v_mph > 20
        for c in msg.can:
          if c.src > 2:            # 0/1/2 are the real buses; higher are our own echoes
            continue
          d = seen[(c.address, c.src)]
          d['n'] += 1
          d['bus'].add(c.src)
          d['len'] = len(c.dat)
          if len(d['vals']) < 400:
            d['vals'].add(bytes(c.dat))
          if moving:
            d['moving'] += 1
            if len(d['moving_vals']) < 400:
              d['moving_vals'].add(bytes(c.dat))

          if c.address == PARK_STATUS2:
            payload = bytes(c.dat)
            for lo, hi in SPEED_BINS:
              if lo <= v_mph < hi:
                b = park_by_bin[(lo, hi)]
                b['active'].append(bits(payload, 24, 1))
                b['noise'].append(bits(payload, 25, 2))
                b['bs_right'].append(bits(payload, 27, 2))
                b['bs_left'].append(bits(payload, 29, 2))
                break

  print("-- PARK_status2 (0x30E), the only park message the DBC decodes --")
  print(f"  {'speed (mph)':>12} {'frames':>8}  {'sdiActive':>22} {'sdiNoise':>16} "
        f"{'blindSpot L / R':>22}")
  for lo, hi in SPEED_BINS:
    b = park_by_bin.get((lo, hi))
    if not b:
      continue
    a = np.array(b['active'])
    n = np.array(b['noise'])
    left, right = np.array(b['bs_left']), np.array(b['bs_right'])
    print(f"  {f'{lo}-{hi}':>12} {len(a):>8}  {f'{100*a.mean():.0f}% set':>22} "
          f"{f'vals {sorted(set(n.tolist()))}':>16} "
          f"{f'{sorted(set(left.tolist()))} / {sorted(set(right.tolist()))}':>22}")

  print("\n-- every id on the bus: which ones are alive and changing above 20 mph --")
  rows = []
  for (addr, bus), d in seen.items():
    if not d['moving']:
      continue
    rows.append((len(d['moving_vals']), addr, bus, d['n'], d['moving'], d['len']))
  rows.sort(reverse=True)
  print(f"  {'addr':>8} {'bus':>4} {'len':>4} {'frames':>8} {'>20mph':>8} {'distinct payloads >20mph':>26}")
  for nv, addr, bus, n, mv, ln in rows[:28]:
    print(f"  {hex(addr):>8} {bus:>4} {ln:>4} {n:>8} {mv:>8} {nv:>26}")

  static_ids = [hex(a) for nv, a, b, n, mv, ln in rows if nv <= 1]
  print(f"\n  ids that never change while moving ({len(static_ids)}): {" ".join(static_ids[:24])}"
        f"{' ...' if len(static_ids) > 24 else ''}")


if __name__ == '__main__':
  main(sys.argv[1:])
