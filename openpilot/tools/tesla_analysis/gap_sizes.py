#!/usr/bin/env python3
"""How sparse the stock module's requests are, and what the gear did around the maneuver.

The hold has to be long enough to bridge the gaps between the frames that actually carry an APC
state or a steering command, but every millisecond of it is also a millisecond openpilot stays
silent after the maneuver is over. So measure the gaps rather than guessing.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import glob
import re

import capnp
import zstandard
from openpilot.cereal import log as capnp_log

ROUTE = '0000000c--a28806246c'
LO, HI = 60.0, 90.0


def seg_no(path):
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'gs-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    # Read the whole staged rlog and parse from bytes. A segment cut short by the car losing
    # power leaves a truncated final message, and the streaming reader aborts on it inside
    # libkj -- a C++ terminate that no Python except can catch, killing the whole run. Parsing
    # from bytes raises a normal KjException instead, after yielding every complete event.
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


asks = []      # times the stock module asked for the bus
gears = []     # gear transitions
t0 = None
gear = None
for path in sorted(glob.glob(os.path.join(LOG_ROOT, f'{ROUTE}--*/rlog.zst')), key=seg_no):
  for evt in read_events(path):
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if dt < LO or dt > HI:
      continue
    w = evt.which()
    if w == 'carState':
      g = str(evt.carState.gearShifter)
      if g != gear:
        gears.append((dt, gear, g))
        gear = g
    elif w == 'can':
      for f in evt.can:
        d = bytes(f.dat)
        if f.src == 2 and f.address == 0x2B9 and len(d) >= 2:
          if 5 <= ((d[1] >> 4) & 0x0F) <= 11:
            asks.append((dt, 'APC'))
        elif f.src == 2 and f.address == 0x488 and len(d) >= 3:
          if (d[2] >> 6) in (1, 2):
            asks.append((dt, 'STEER'))

asks.sort()
print(f'stock module asked for the bus on {len(asks)} frames, '
      f'from +{asks[0][0]:.2f}s to +{asks[-1][0]:.2f}s')
gaps = [(asks[i + 1][0] - asks[i][0], asks[i][0]) for i in range(len(asks) - 1)]
gaps.sort(reverse=True)
print(f'\nlargest gaps between requests:')
for g, at in gaps[:8]:
  print(f'   {g * 1000:7.0f} ms   (after +{at:.2f}s)')
print(f'\n=> a hold shorter than {gaps[0][0] * 1000:.0f} ms would have split the session')

print('\ngear transitions in the window:')
for dt, a, b in gears:
  print(f'   +{dt:7.2f}s  {a} -> {b}')
