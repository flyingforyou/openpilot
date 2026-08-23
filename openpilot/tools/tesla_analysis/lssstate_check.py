#!/usr/bin/env python3
"""Does the genuine (unspoofed) DAS_status2.DAS_lssState still reach the cluster with a value that
contradicts our synthetic AutopilotStatus=3 / DAS_lanes while openpilot is engaged?"""

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
status2_msg = dbc.addr_to_msg[0x389]
lss_sig = status2_msg.sigs['DAS_lssState']
robstate_sig = status2_msg.sigs['DAS_robState']


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


def run(route, n, gate_enabled=None):
  enabled = False
  counts_lss = Counter()
  counts_rob = Counter()
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
          if c.src != 2 or c.address != 0x389:
            continue
          if gate_enabled is not None and enabled != gate_enabled:
            continue
          lss = int(phys(lss_sig, get_raw_value(bytes(c.dat), lss_sig)))
          rob = int(phys(robstate_sig, get_raw_value(bytes(c.dat), robstate_sig)))
          counts_lss[lss] += 1
          counts_rob[rob] += 1
  print(f'route={route} gate_enabled={gate_enabled}')
  print(f'  DAS_lssState dist: {dict(counts_lss)}')
  print(f'  DAS_robState dist: {dict(counts_rob)}')


print('=== 9e: genuine bus2 DAS_status2 WHILE OPENPILOT IS ENGAGED ===')
run('0000009e--7f4078b620', 6, gate_enabled=True)
print('\n=== 9e: genuine bus2 DAS_status2 while NOT engaged (baseline) ===')
run('0000009e--7f4078b620', 6, gate_enabled=False)
print('\n=== 9f: genuine bus2 DAS_status2 during dashcam run (all) ===')
run('0000009f--b644363276', 3, gate_enabled=None)
