#!/usr/bin/env python3
"""Find recorded moments where the driver actually took the wheel.

Bit positions match what panda reads out of EPAS_sysStatus in tesla_legacy.h, so this agrees
with the safety code rather than re-deriving the layout.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys
from collections import Counter

import capnp
import zstandard
from cereal import log as capnp_log

EAC_STATUS = {0: 'EAC_INHIBITED', 1: 'EAC_AVAILABLE', 2: 'EAC_ACTIVE', 3: 'EAC_FAULT'}


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'ho-{os.getpid()}.rlog')
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


for path in sys.argv[1:]:
  hol_hist, eac_hist, err_hist = Counter(), Counter(), Counter()
  hol3_with_eac = Counter()
  for evt in read_events(path):
    if evt.which() != 'can':
      continue
    for f in evt.can:
      if f.src != 0 or f.address != 0x370:
        continue
      d = bytes(f.dat)
      hol = d[4] >> 6
      eac = d[6] >> 5
      err = d[2] >> 4
      hol_hist[hol] += 1
      eac_hist[eac] += 1
      if hol >= 3:
        hol3_with_eac[(EAC_STATUS.get(eac, eac), err)] += 1
      if eac == 0:
        err_hist[err] += 1
  seg = os.path.basename(os.path.dirname(path))
  n3 = sum(v for k, v in hol_hist.items() if k >= 3)
  print(f'{seg}: handsOnLevel={dict(sorted(hol_hist.items()))} '
        f'>=3:{n3}  eacStatus={ {EAC_STATUS.get(k, k): v for k, v in sorted(eac_hist.items())} }')
  if err_hist:
    print(f'    EAC_INHIBITED errorCodes: {dict(sorted(err_hist.items()))}')
  if hol3_with_eac:
    print(f'    while handsOnLevel>=3: {dict(hol3_with_eac)}')
