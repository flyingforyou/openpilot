#!/usr/bin/env python3
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
csa_sig = dbc.addr_to_msg[0x399].sigs['DAS_csaState']
status2_msg = dbc.addr_to_msg[0x389]
fail_sig = status2_msg.sigs['DAS_activationFailureStatus']
rob_sig = status2_msg.sigs['DAS_robState']
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


route = '0000009c--7bfb4cf4af'
n = 6
events = []
for i in range(n):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if os.path.exists(p):
    events.extend(read_events(p))

t0 = min(e.logMonoTime for e in events if e.which() in ('can', 'selfdriveState'))
LO, HI = 215.0, 230.0

enabled = None
for evt in events:
  t = (evt.logMonoTime - t0) / 1e9
  if t < LO or t > HI:
    continue
  w = evt.which()
  if w == 'selfdriveState':
    new_en = evt.selfdriveState.enabled
    if new_en != enabled:
      print(f't={t:7.3f}s  ENGAGED -> {new_en}')
    enabled = new_en
  elif w == 'can':
    for c in evt.can:
      if c.src != 2:
        continue
      if c.address == 0x399:
        st = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
        csa = int(phys(csa_sig, get_raw_value(bytes(c.dat), csa_sig)))
        print(f't={t:7.3f}s  AutopilotStatus: status={st} csa={csa}')
      elif c.address == 0x389:
        fail = int(phys(fail_sig, get_raw_value(bytes(c.dat), fail_sig)))
        rob = int(phys(rob_sig, get_raw_value(bytes(c.dat), rob_sig)))
        lss = int(phys(lss_sig, get_raw_value(bytes(c.dat), lss_sig)))
        interact = int(phys(interact_sig, get_raw_value(bytes(c.dat), interact_sig)))
        print(f't={t:7.3f}s  DAS_status2: fail={fail} rob={rob} lss={lss} interact={interact}')
