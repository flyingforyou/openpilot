#!/usr/bin/env python3
"""Is something other than us (src==128) also putting 0x239/0x399 onto bus0 (src==0) while
openpilot is engaged -- a two-masters-on-one-arbitration-id conflict?"""

import os
import sys
from collections import Counter

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'w-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


route = sys.argv[1]
n = int(sys.argv[2])

enabled = False
rows = []  # (t, addr, src, enabled, hex)
t0 = None
for i in range(n):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if not os.path.exists(p):
    continue
  for evt in read_events(p):
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      t = evt.logMonoTime
      if t0 is None:
        t0 = t
      for c in evt.can:
        if c.address in (0x239, 0x399) and c.src in (0, 128):
          rows.append(((t - t0) / 1e9, c.address, c.src, enabled, bytes(c.dat).hex()))

by_addr_src_enabled = Counter()
for t, addr, src, en, h in rows:
  by_addr_src_enabled[(hex(addr), src, en)] += 1

print(f'route={route}  total 0x239/0x399 bus0 frames (src 0 or 128): {len(rows)}')
for k in sorted(by_addr_src_enabled):
  print(f'  addr={k[0]} src={k[1]:3d} enabled={k[2]!s:5s}  n={by_addr_src_enabled[k]}')

print('\nsrc=0 frames while enabled=True (the smoking gun if any exist):')
shown = 0
for t, addr, src, en, h in rows:
  if src == 0 and en and shown < 20:
    print(f'  t={t:8.3f}s addr={hex(addr)} dat={h}')
    shown += 1
if shown == 0:
  print('  (none)')
