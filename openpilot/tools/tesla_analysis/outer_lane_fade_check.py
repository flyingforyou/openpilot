#!/usr/bin/env python3
"""How often does our own AutopilotStatus flip 2<->3 while engaged (the both_lanes coupling from
2bac9829e), and what does DAS_leftLaneExists/rightLaneExists look like at each state? Answers
whether decoupling the fade from AutopilotStatus (hold status=3, let exists/usage carry the fade)
is workable."""

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
status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']
lanes_msg = dbc.addr_to_msg[0x239]


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
  enabled = False
  our_status_prev = None
  transitions = 0
  status_dur = Counter()  # frames spent at each our-own status value while enabled
  exists_by_status = Counter()  # (status, left, right) -> n
  t0 = None
  last_t = None
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
          if not enabled:
            continue
          if c.address == 0x399 and c.src == 128:
            st = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
            if our_status_prev is not None and st != our_status_prev:
              transitions += 1
            our_status_prev = st
            status_dur[st] += 1
          elif c.address == 0x239 and c.src == 128 and our_status_prev is not None:
            le = int(phys(lanes_msg.sigs['DAS_leftLaneExists'], get_raw_value(bytes(c.dat), lanes_msg.sigs['DAS_leftLaneExists'])))
            re_ = int(phys(lanes_msg.sigs['DAS_rightLaneExists'], get_raw_value(bytes(c.dat), lanes_msg.sigs['DAS_rightLaneExists'])))
            exists_by_status[(our_status_prev, le, re_)] += 1

  print(f'route={route}  our own AutopilotStatus transitions while enabled: {transitions}')
  print(f'  time (frames) at each status: {dict(status_dur)}')
  print('  (status, leftExists, rightExists) -> n:')
  for k in sorted(exists_by_status):
    print(f'    {k}  n={exists_by_status[k]}')


scan('000000b9--400f9a5624', 11)
scan('000000b7--e8030ef344', 2)
