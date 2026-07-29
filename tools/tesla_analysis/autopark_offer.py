#!/usr/bin/env python3
"""Windows where the car offered autopark, and what openpilot was doing then.

DAS_autoparkReady going high is the screen showing the option. If openpilot was engaged during
those windows, the forwarding gate was shut by design -- it only opens while controls are not
allowed -- so the stock module's first commands never reached the car.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import glob
import re
import sys

import capnp
import zstandard
from cereal import log as capnp_log

ACC_STATE = {0: 'ACC_CANCEL_GENERIC', 3: 'ACC_HOLD', 4: 'ACC_ON', 5: 'APC_BACKWARD',
             6: 'APC_FORWARD', 7: 'APC_COMPLETE', 8: 'APC_ABORT', 9: 'APC_PAUSE',
             10: 'APC_UNPARK_COMPLETE', 11: 'APC_SELFPARK_START',
             13: 'ACC_CANCEL_GENERIC_SILENT', 15: 'FAULT_SNA'}


def seg_no(path):
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'ao-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      try:
        yield from capnp_log.Event.read_multiple(f)
      except capnp.KjException:
        pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


for route in sys.argv[1:]:
  print(f'\n===== {route} =====')
  t0 = None
  enabled = False
  gear = acc = None
  cur = None
  episodes = []
  for path in sorted(glob.glob(f'{LOG_ROOT}/{route}--*/rlog.zst'), key=seg_no):
    for e in read_events(path):
      t = e.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      w = e.which()
      if w == 'selfdriveState':
        enabled = bool(e.selfdriveState.enabled)
      elif w == 'carState':
        gear = str(e.carState.gearShifter)
      elif w == 'can':
        for f in e.can:
          d = bytes(f.dat)
          if f.src == 2 and f.address == 0x2B9 and len(d) >= 2:
            acc = ACC_STATE.get((d[1] >> 4) & 0x0F)
          elif f.src == 2 and f.address == 0x399 and len(d) >= 4:
            ready = d[3] & 1
            if ready:
              if cur is None:
                cur = {'start': t, 'last': t, 'eng': 0, 'n': 0, 'gears': set(), 'accs': set()}
              cur['last'] = t
              cur['eng'] += enabled
              cur['n'] += 1
              cur['gears'].add(gear)
              cur['accs'].add(acc)
            elif cur is not None:
              episodes.append(cur)
              cur = None
  if cur is not None:
    episodes.append(cur)

  if not episodes:
    print('  autoparkReady never went high')
    continue
  print(f'  {len(episodes)} window(s) where the car offered autopark:')
  for e in episodes:
    print(f'    +{e["start"] - t0:8.1f}s  dur={e["last"] - e["start"]:5.2f}s  '
          f'openpilot engaged {100 * e["eng"] / max(e["n"], 1):3.0f}% of it  '
          f'gear={sorted(x for x in e["gears"] if x)}  stock_accState={sorted(x for x in e["accs"] if x)}')
