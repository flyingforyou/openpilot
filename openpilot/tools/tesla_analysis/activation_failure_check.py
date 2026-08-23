#!/usr/bin/env python3
"""What does DAS_activationFailureStatus / DAS_robState say right when openpilot engages and the
genuine bus2 autopilotStatus immediately drops from Available(2) to Unavailable(1)?"""

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
status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']
status2_msg = dbc.addr_to_msg[0x389]
fail_sig = status2_msg.sigs['DAS_activationFailureStatus']
rob_sig = status2_msg.sigs['DAS_robState']
csa_sig = dbc.addr_to_msg[0x399].sigs['DAS_csaState']
lss_sig = status2_msg.sigs['DAS_lssState']
interact_sig = status2_msg.sigs['DAS_driverInteractionLevel']


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


def scan(route, n):
  events = []
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      events.extend(read_events(p))

  t0 = min(e.logMonoTime for e in events if e.which() in ('can', 'selfdriveState'))
  enabled = False
  prev_enabled = None
  cur_status2 = {}
  print(f'\n=== {route} ===')
  for evt in events:
    t = (evt.logMonoTime - t0) / 1e9
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
      if prev_enabled is not None and enabled != prev_enabled:
        print(f't={t:8.2f}s  ENGAGED {prev_enabled} -> {enabled}   last DAS_status2: {cur_status2}')
      prev_enabled = enabled
    elif w == 'can':
      for c in evt.can:
        if c.src != 2:
          continue
        if c.address == 0x389:
          cur_status2['fail'] = int(phys(fail_sig, get_raw_value(bytes(c.dat), fail_sig)))
          cur_status2['rob'] = int(phys(rob_sig, get_raw_value(bytes(c.dat), rob_sig)))
          cur_status2['lss'] = int(phys(lss_sig, get_raw_value(bytes(c.dat), lss_sig)))
          cur_status2['interact'] = int(phys(interact_sig, get_raw_value(bytes(c.dat), interact_sig)))
        elif c.address == 0x399:
          cur_status2['status'] = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
          cur_status2['csa'] = int(phys(csa_sig, get_raw_value(bytes(c.dat), csa_sig)))


scan('0000009b--5681d134f7', 6)
scan('0000009c--7bfb4cf4af', 6)
