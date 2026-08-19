#!/usr/bin/env python3
"""Hunt the bus for messages the DBC does not decode, and rank them as proximity-sensor candidates.

The car has ultrasonics at the front, the rear and both corners, but the only park message the
DBC knows carries a two-bit warning per side. If the individual ranges are published at all they
are in a message nobody has named yet, so this lists every unknown id and scores it on the one
behaviour that would give it away: bytes that move while creeping and go quiet at speed.

  ./undecoded_can.py op-logs/0000006e--2ad4d92ec7--*
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader

DBC_DIR = Path('/home/compiler/openpilot/opendbc_repo/opendbc/dbc')
DBCS = ('tesla_can.dbc', 'tesla_powertrain.dbc', 'tesla_radar_bosch_generated.dbc')
SLOW_MS, FAST_MS = 4.0, 20.0     # creeping vs clearly driving


def known_ids() -> set[int]:
  ids = set()
  for name in DBCS:
    path = DBC_DIR / name
    if not path.is_file():
      continue
    for line in path.read_text(errors='replace').splitlines():
      m = re.match(r'BO_ (\d+) ', line)
      if m:
        ids.add(int(m.group(1)))
  return ids


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def main(paths):
  paths = sorted(paths, key=seg_no)
  known = known_ids()
  print(f"DBC knows {len(known)} message ids")

  slow = defaultdict(list)
  fast = defaultdict(list)
  seen = defaultdict(lambda: {'n': 0, 'len': 0})
  v = 0.0

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      if w == 'carState':
        v = float(msg.carState.vEgo)
      elif w == 'can':
        for c in msg.can:
          if c.src > 2:
            continue
          key = (c.address, c.src)
          seen[key]['n'] += 1
          seen[key]['len'] = len(c.dat)
          if v < SLOW_MS and len(slow[key]) < 4000:
            slow[key].append(bytes(c.dat))
          elif v > FAST_MS and len(fast[key]) < 4000:
            fast[key].append(bytes(c.dat))

  unknown = sorted(k for k in seen if k[0] not in known)
  print(f"ids on the bus: {len(seen)}   not in the DBC: {len(unknown)}")

  def byte_activity(samples):
    """How many of the bytes actually move, and how much."""
    if len(samples) < 20:
      return None
    n = min(len(s) for s in samples)
    arr = np.array([list(s[:n]) for s in samples], dtype=np.uint8)
    moving = int((arr.std(axis=0) > 1.0).sum())
    return moving, float(arr.std(axis=0).mean())

  rows = []
  for key in unknown:
    a_slow = byte_activity(slow.get(key, []))
    a_fast = byte_activity(fast.get(key, []))
    # Both bands need samples: a message that simply stops transmitting at speed would
    # otherwise score as "goes quiet", which is a different thing entirely.
    if a_slow is None or a_fast is None:
      continue
    ms, mf = (a_slow[0] if a_slow else 0), (a_fast[0] if a_fast else 0)
    ss, sf = (a_slow[1] if a_slow else 0.0), (a_fast[1] if a_fast else 0.0)
    # A parking sensor is busy while creeping and idle at speed.
    score = ss - sf
    rows.append((score, key, seen[key], ms, mf, ss, sf))

  rows.sort(reverse=True)
  print("\n-- unknown ids, most 'busy slow, quiet fast' first --")
  print(f"  {'addr':>7} {'bus':>4} {'len':>4} {'frames':>8} {'bytes mv slow':>14} {'fast':>6} {'std slow':>9} {'fast':>7} {'delta':>7}")
  for score, (addr, bus), d, ms, mf, ss, sf in rows[:20]:
    print(f"  {hex(addr):>7} {bus:>4} {d['len']:>4} {d['n']:>8} {ms:>14} {mf:>6} {ss:>9.1f} {sf:>7.1f} {score:>+7.1f}")

  if not rows:
    print("  (none had enough samples in both speed bands)")

  print("\n-- for reference, ids the DBC does know, same measure --")
  ref = []
  for key in sorted(k for k in seen if k[0] in known):
    a_slow, a_fast = byte_activity(slow.get(key, [])), byte_activity(fast.get(key, []))
    if a_slow is None or a_fast is None:
      continue
    ref.append((a_slow[1] - a_fast[1], key, a_slow[1], a_fast[1]))
  ref.sort(reverse=True)
  for score, (addr, bus), ss, sf in ref[:5]:
    print(f"  {hex(addr):>7} bus{bus}  std slow {ss:6.1f}  fast {sf:6.1f}  delta {score:+6.1f}")


if __name__ == '__main__':
  main(sys.argv[1:])
