#!/usr/bin/env python3
"""Is DAS_steeringControlType a clean TACC/AP discriminator even inside same-road toggle windows?"""

import os
import sys
from collections import defaultdict, Counter

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
steer_msg = dbc.addr_to_msg[0x220] if 0x220 in dbc.addr_to_msg else None
# find DAS_steeringControl message by name instead of assuming address
for m in dbc.addr_to_msg.values():
  if m.name == 'DAS_steeringControl':
    steer_msg = m
steer_sig = steer_msg.sigs['DAS_steeringControlType']
steer_addr = steer_msg.address


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
n_segs = int(sys.argv[2])

events = []
for i in range(n_segs):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if os.path.exists(p):
    for evt in read_events(p):
      events.append(evt)

t0 = None
cur_status = None
transitions = []
for evt in events:
  if evt.which() != 'can':
    continue
  t = evt.logMonoTime
  if t0 is None:
    t0 = t
  for c in evt.can:
    if c.src == 2 and c.address == 0x399:
      st = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
      if cur_status is not None and st != cur_status and cur_status in (2, 3) and st in (2, 3):
        transitions.append((t - t0) / 1e9)
      cur_status = st

windows = [(t - 6.0, t + 6.0) for t in transitions]


def in_window(t):
  return any(lo <= t <= hi for lo, hi in windows)


cur_status = None
counts = defaultdict(Counter)
for evt in events:
  if evt.which() != 'can':
    continue
  t = (evt.logMonoTime - t0) / 1e9
  for c in evt.can:
    if c.src != 2:
      continue
    if c.address == 0x399:
      cur_status = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
      continue
    if c.address == steer_addr and cur_status in (2, 3):
      v = int(phys(steer_sig, get_raw_value(bytes(c.dat), steer_sig)))
      key = cur_status if not in_window(t) else (cur_status, 'windowed')
      counts[key][v] += 1

print(f'route={route}  addr=0x{steer_addr:x}  n_windows={len(windows)}')
for k in [2, 3, (2, 'windowed'), (3, 'windowed')]:
  print(f'  status={k}  DAS_steeringControlType distribution: {dict(counts[k])}')
