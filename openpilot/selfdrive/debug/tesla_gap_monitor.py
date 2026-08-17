#!/usr/bin/env python3
"""Print the Tesla legacy gap signal and the decoded gap it produces.

Run with the car on (engine/accessory) and no need to drive:

  cd /data/openpilot && PYTHONPATH=. python3 selfdrive/debug/tesla_gap_monitor.py

Cycle the steering wheel gap setting through 1..7 and confirm that:
  1. raw changes when you change the setting, and
  2. raw HOLDS the new value instead of falling back to 0 or 255 when you stop adjusting.

If raw only briefly shows a value while adjusting and reads 0 otherwise, the signal is a
momentary request and the latching logic in carstate.py has to be reworked before use.
"""
import time

import cereal.messaging as messaging
from opendbc.car.tesla.carstate import decode_tesla_gap


def main():
  sm = messaging.SubMaster(['can', 'carState'])
  last_line = None

  while True:
    sm.update(100)

    raw = None
    for msg in sm['can']:
      # STW_ACTN_RQ, DTR_Dist_Rq is byte 1 (bits 8-15), little endian, scale 1 offset 0
      if msg.address == 69 and len(msg.dat) >= 2:
        raw = msg.dat[1]

    if raw is None:
      continue

    gap_adjust = sm['carState'].cruiseState.gapAdjust
    line = f"raw={raw:<5} decoded={decode_tesla_gap(raw)}  ->  carState.gapAdjust={gap_adjust}"
    if line != last_line:
      print(f"{time.monotonic():10.2f}  {line}")
      last_line = line


if __name__ == "__main__":
  main()
