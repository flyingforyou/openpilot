#!/usr/bin/env python3
"""Find the real Tesla gap-knob signal by diffing raw CAN activity over time.

We assumed STW_ACTN_RQ's DTR_Dist_Rq (msg 69, byte 1) tracks the left-side gap knob, but a full
drive log showed it pinned at raw=100 for 21 minutes straight while the driver reported changing
gap and watching the instrument cluster update. So either DTR_Dist_Rq isn't the right signal, or
it only pulses briefly instead of holding the current value.

Run this stationary (ignition/accessory on, car not moving):

  cd /data/openpilot && PYTHONPATH=. python3 selfdrive/debug/tesla_gap_can_diff.py

Turn the left gap knob through positions 1 -> 2 -> ... -> 7 (or however many stops it has),
pausing ~2-3s at each one, then let it finish. It buffers every byte on the bus, drops anything
that changes too often (analog sensors like steering angle) or never changes at all, and prints
the surviving candidates -- switch-like bytes with a small number of distinct values -- along
with the timestamped sequence of when each one changed. Match that timeline against when you
actually turned the knob to find the real signal.
"""
import time
from collections import defaultdict

import openpilot.cereal.messaging as messaging

DURATION_S = 90
MAX_CHANGES_PER_SEC = 2.0  # drop bytes that change more often than this (analog/noisy sensors)


def main():
  sm = messaging.SubMaster(['can'])
  history: dict[tuple[int, int], list[tuple[float, int]]] = defaultdict(list)
  t0 = time.monotonic()

  print(f"Recording all CAN traffic for {DURATION_S}s. Turn the gap knob through its positions now,")
  print("pausing a couple seconds at each one.\n")

  while time.monotonic() - t0 < DURATION_S:
    sm.update(100)
    t = time.monotonic() - t0

    for msg in sm['can']:
      for i, b in enumerate(msg.dat):
        key = (msg.address, i)
        h = history[key]
        if not h or h[-1][1] != b:
          h.append((t, b))

  print(f"\nDone. Scanned {len(history)} (address, byte) pairs.\n")
  print("Candidates (changed 2-15 times total, and no faster than "
        f"{MAX_CHANGES_PER_SEC}/s on average):\n")

  found = False
  for (addr, byte_idx), h in sorted(history.items()):
    n_changes = len(h) - 1
    if not (1 <= n_changes <= 15):
      continue
    span = max(h[-1][0] - h[0][0], 0.01)
    rate = n_changes / span
    if rate > MAX_CHANGES_PER_SEC:
      continue
    found = True
    print(f"addr={addr} (0x{addr:03x})  byte[{byte_idx}]:")
    for t, v in h:
      print(f"    {t:6.2f}s  -> {v}")
    print()

  if not found:
    print("No candidates survived filtering. Either nothing on the bus changed while you turned")
    print("the knob (check the harness/panda is actually connected), or it changes too fast/slow")
    print("for these thresholds -- rerun with a longer DURATION_S or looser MAX_CHANGES_PER_SEC.")


if __name__ == "__main__":
  main()
