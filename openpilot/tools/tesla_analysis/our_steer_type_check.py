#!/usr/bin/env python3
import os
import sys
from collections import Counter

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC as DBCLoader
from opendbc.can.parser import get_raw_value

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
dbc = DBCLoader(os.path.join(REPO_ROOT, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc'))
steer_msg = None
for m in dbc.addr_to_msg.values():
  if m.name == 'DAS_steeringControl':
    steer_msg = m
type_sig = steer_msg.sigs['DAS_steeringControlType']


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
enabled = False
counts = Counter()
for i in range(n):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if not os.path.exists(p):
    continue
  for evt in read_events(p):
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      for c in evt.can:
        if c.address == steer_msg.address and c.src == 128:
          v = int(phys(type_sig, get_raw_value(bytes(c.dat), type_sig)))
          counts[(enabled, v)] += 1

print(f'route={route}  our own DAS_steeringControlType (src=128) by enabled state:')
for k in sorted(counts):
  print(f'  enabled={k[0]!s:5s} type={k[1]}  n={counts[k]}')
