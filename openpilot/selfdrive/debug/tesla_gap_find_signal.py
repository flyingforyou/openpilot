#!/usr/bin/env python3
"""Find which CAN signal actually tracks the Tesla gap knob, from a recorded route.

STW_ACTN_RQ's DTR_Dist_Rq sat at raw=100 for an entire 21 minute drive even though the driver
used the gap knob and watched the instrument cluster change, so that signal is not (or not
only) what the knob drives. openpilot already records every CAN frame to rlog, so no live
capture is needed: drive/power on, work the knob, then point this at the route.

  PYTHONPATH=. python3 selfdrive/debug/tesla_gap_find_signal.py <route_dir_or_rlog> [--markers]

Every byte on the bus is tracked over time; anything that changes as often as an analog sensor
is dropped, and what remains -- switch-like bytes with a handful of discrete values -- is
printed with the timestamps of each change, so it can be lined up against when the knob moved.

--markers additionally prints turn signal stalk transitions (STW_ACTN_RQ TurnIndLvr_Stat).
Flicking the blinker before and after working the knob brackets it with signals we already
know how to decode, which is the easiest way to timestamp the knob movements from the car.
"""
import os
import sys
from collections import defaultdict

from tools.lib.logreader import LogReader

MAX_CHANGES = 40         # a knob has a handful of positions, not hundreds
MAX_CHANGES_PER_SEC = 1.0  # drop analog-ish signals (speed, angle, counters, checksums)

STW_ACTN_RQ = 69
TURN_STALK_BYTE = 2       # TurnIndLvr_Stat is bits 16-17 -> byte 2, low 2 bits


def rlog_paths(target: str) -> list[str]:
  if os.path.isfile(target):
    return [target]

  paths = []
  for entry in sorted(os.listdir(target), key=lambda p: (len(p), p)):
    rlog = os.path.join(target, entry, 'rlog.zst')
    if os.path.isfile(rlog):
      paths.append(rlog)
  return paths


def main(target: str, show_markers: bool):
  paths = rlog_paths(target)
  if not paths:
    print(f"no rlog found under {target}")
    return
  print(f"scanning {len(paths)} segment(s)\n")

  history: dict[tuple[int, int], list[tuple[float, int]]] = defaultdict(list)
  markers: list[tuple[float, int]] = []
  t0 = None

  for path in paths:
    for msg in LogReader(path):
      if msg.which() != 'can':
        continue
      if t0 is None:
        t0 = msg.logMonoTime
      t = (msg.logMonoTime - t0) / 1e9

      for c in msg.can:
        if c.src != 0:  # one bus is enough; both carry the same frames here
          continue
        for i, b in enumerate(c.dat):
          key = (c.address, i)
          h = history[key]
          if not h or h[-1][1] != b:
            h.append((t, b))

        if show_markers and c.address == STW_ACTN_RQ and len(c.dat) > TURN_STALK_BYTE:
          stalk = c.dat[TURN_STALK_BYTE] & 0x3
          if not markers or markers[-1][1] != stalk:
            markers.append((t, stalk))

  if show_markers:
    print("=== turn signal stalk (0=idle 1=left 2=right) ===")
    for t, v in markers:
      print(f"  {t:8.2f}s  -> {v}")
    print()

  print("=== candidate knob signals ===")
  found = False
  for (addr, byte_idx), h in sorted(history.items()):
    n_changes = len(h) - 1
    if not (2 <= n_changes <= MAX_CHANGES):
      continue
    span = max(h[-1][0] - h[0][0], 0.01)
    if n_changes / span > MAX_CHANGES_PER_SEC:
      continue

    found = True
    values = sorted({v for _, v in h})
    print(f"\naddr={addr} (0x{addr:03x}) byte[{byte_idx}]  {n_changes} changes, values={values}")
    for t, v in h:
      print(f"    {t:8.2f}s  -> {v}")

  if not found:
    print("nothing survived filtering -- loosen MAX_CHANGES / MAX_CHANGES_PER_SEC")


if __name__ == "__main__":
  args = [a for a in sys.argv[1:] if not a.startswith('--')]
  if len(args) != 1:
    print(f"usage: {sys.argv[0]} <route_dir_or_rlog> [--markers]")
    sys.exit(1)
  main(args[0], '--markers' in sys.argv)
