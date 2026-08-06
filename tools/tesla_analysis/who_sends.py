#!/usr/bin/env python3
"""Who is putting DAS_control and DAS_steeringControl on the car bus during the autopark window.

src 0/2 are received frames, src 128+ are the panda's own transmissions (bus + 128). If openpilot
is still transmitting 0x2B9 onto bus 0 while disengaged, the stock autopark module has no channel
to take the car over with, no matter what the forwarding gate does.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import glob
import re
import sys
from collections import Counter

import capnp
import zstandard
from cereal import log as capnp_log

LO, HI = float(sys.argv[1]), float(sys.argv[2])
ROUTE = sys.argv[3]

ACC_STATE = {0: 'ACC_CANCEL_GENERIC', 3: 'ACC_HOLD', 4: 'ACC_ON', 5: 'APC_BACKWARD',
             6: 'APC_FORWARD', 7: 'APC_COMPLETE', 8: 'APC_ABORT', 9: 'APC_PAUSE',
             10: 'APC_UNPARK_COMPLETE', 11: 'APC_SELFPARK_START',
             13: 'ACC_CANCEL_GENERIC_SILENT', 15: 'FAULT_SNA'}
STEER_TYPE = {0: 'NONE', 1: 'ANGLE_CONTROL', 2: 'LKA', 3: 'EMERG'}


def seg_no(path):
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'ws-{os.getpid()}.rlog')
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


counts: Counter = Counter()
t0 = None
for path in sorted(glob.glob(os.path.join(LOG_ROOT, f'{ROUTE}--*/rlog.zst')), key=seg_no):
  for e in read_events(path):
    if e.which() != 'can':
      continue
    t = e.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if dt < LO or dt > HI:
      continue
    for f in e.can:
      d = bytes(f.dat)
      if f.address == 0x2B9 and len(d) >= 2:
        counts[(f.src, '0x2B9 DAS_control', ACC_STATE.get((d[1] >> 4) & 0x0F))] += 1
      elif f.address == 0x488 and len(d) >= 3:
        counts[(f.src, '0x488 DAS_steeringControl', STEER_TYPE.get(d[2] >> 6))] += 1

print(f'{ROUTE}  window +{LO:.0f}s..+{HI:.0f}s')
print(f'\n{"src":>5s}  {"의미":<28s} {"메시지":<26s} {"값":<26s} n')
for (src, msg, val), n in sorted(counts.items()):
  if src == 0:
    role = 'bus0 수신 (차량측)'
  elif src == 2:
    role = 'bus2 수신 (순정 AP)'
  elif src == 128:
    role = 'bus0 송신 (openpilot/panda)'
  elif src == 130:
    role = 'bus2 송신'
  else:
    role = f'src {src}'
  print(f'{src:5d}  {role:<28s} {msg:<26s} {str(val):<26s} {n}')
