#!/usr/bin/env python3
"""Is our own transmitted DAS_lanesCounter actually incrementing, or stuck on a stale clone?"""

import os
import sys

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC as DBCLoader
from opendbc.can.parser import get_raw_value

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
dbc = DBCLoader(os.path.join(REPO_ROOT, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc'))
lanes_msg = dbc.addr_to_msg[0x239]
counter_sig = lanes_msg.sigs['DAS_lanesCounter']


def phys(sig, raw):
  return raw * sig.factor + sig.offset


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

t0 = None
enabled = False
seq = []  # (t, src, counter, raw_bytes_hex)
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
        if c.address == 0x239 and c.src != 2 and enabled:
          cnt = int(phys(counter_sig, get_raw_value(bytes(c.dat), counter_sig)))
          seq.append(((t - t0) / 1e9, c.src, cnt, bytes(c.dat).hex()))

print(f'route={route}  our own 0x239 sends while enabled: {len(seq)}')
print('first 40:')
prev_cnt = None
stuck_run = 0
max_stuck_run = 0
repeats = 0
for t, src, cnt, hexd in seq[:40]:
  print(f'  t={t:8.3f}s src={src:4d} counter={cnt:2d}  dat={hexd}')

for t, src, cnt, hexd in seq:
  if prev_cnt is not None and cnt == prev_cnt:
    stuck_run += 1
    repeats += 1
  else:
    max_stuck_run = max(max_stuck_run, stuck_run)
    stuck_run = 0
  prev_cnt = cnt
max_stuck_run = max(max_stuck_run, stuck_run)
print(f'\nconsecutive-repeat counter values: {repeats}/{len(seq)-1}  longest stuck run: {max_stuck_run}')

# byte-identical consecutive frame check (full duplicate, not just counter)
dup = sum(1 for i in range(1, len(seq)) if seq[i][3] == seq[i-1][3])
print(f'byte-identical consecutive frames: {dup}/{len(seq)-1}')
