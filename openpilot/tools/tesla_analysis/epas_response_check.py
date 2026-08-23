#!/usr/bin/env python3
"""How does the physical EPAS module (a real bus0-native ECU, unaffected by panda's bus2->bus0
blocking) respond to OUR real steering commands vs. how it responds to genuine factory Autopilot?
Compares within the same drive (9e: enabled vs not) and against genuine AP-active (9d/9f)."""

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
epas_msg = dbc.addr_to_msg[0x370]
tune_sig = epas_msg.sigs['EPAS_currentTuneMode']
eac_sig = epas_msg.sigs['EPAS_eacStatus']
status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']


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


def check_ours(route, n):
  enabled = False
  src_counts = Counter()
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
          if c.address != 0x370:
            continue
          src_counts[c.src] += 1
          tune = int(phys(tune_sig, get_raw_value(bytes(c.dat), tune_sig)))
          eac = int(phys(eac_sig, get_raw_value(bytes(c.dat), eac_sig)))
          counts[(enabled, tune, eac)] += 1
  print(f'route={route}  EPAS_sysStatus src distribution: {dict(src_counts)}')
  print('  (enabled, tuneMode, eacStatus) -> count:')
  for k in sorted(counts):
    print(f'   {k}  n={counts[k]}')


def check_genuine(route, n):
  cur_status = None
  counts = Counter()
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if not os.path.exists(p):
      continue
    for evt in read_events(p):
      if evt.which() != 'can':
        continue
      for c in evt.can:
        if c.src == 2 and c.address == 0x399:
          cur_status = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
        elif c.address == 0x370 and cur_status in (2, 3):
          tune = int(phys(tune_sig, get_raw_value(bytes(c.dat), tune_sig)))
          eac = int(phys(eac_sig, get_raw_value(bytes(c.dat), eac_sig)))
          counts[(cur_status, tune, eac, c.src)] += 1
  print(f'route={route}  genuine EPAS by real autopilotStatus:')
  for k in sorted(counts):
    print(f'   status={k[0]} tune={k[1]} eac={k[2]} src={k[3]}  n={counts[k]}')


print('=== 9e: OUR drive, EPAS response to us ===')
check_ours('0000009e--7f4078b620', 6)
print('\n=== 9f: genuine dashcam drive, EPAS response to real AP ===')
check_genuine('0000009f--b644363276', 3)
