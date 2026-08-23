#!/usr/bin/env python3
import os
import sys

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

n_enabled = 0
n_total = 0
for i in range(n):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if not os.path.exists(p):
    continue
  for evt in read_events(p):
    if evt.which() == 'selfdriveState':
      n_total += 1
      if evt.selfdriveState.enabled:
        n_enabled += 1

print(f'{route}: selfdriveState samples={n_total}  enabled={n_enabled}  ({100*n_enabled/n_total if n_total else 0:.1f}%)')
