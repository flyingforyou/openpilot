#!/usr/bin/env python3
"""TACC vs Autopilot vs US, side by side, for every signal previously found to differ. Uses 9f
(dashcam route, has genuine TACC(2) and AP(3) segments) as the stock baseline, and 9e (our route)
filtered to enabled=True as 'ours'."""

import os
import sys
from collections import Counter, defaultdict

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

SIGNALS = [
  (0x239, 'DAS_leftLaneExists'), (0x239, 'DAS_rightLaneExists'),
  (0x239, 'DAS_leftLineUsage'), (0x239, 'DAS_rightLineUsage'),
  (0x239, 'DAS_virtualLaneWidth'), (0x239, 'DAS_virtualLaneViewRange'),
  (0x488, 'DAS_steeringControlType'),
  (0x370, 'EPAS_currentTuneMode'), (0x370, 'EPAS_eacStatus'),
  (0x101, 'GTW_epasTuneRequest'),
  (0x389, 'DAS_lssState'),
]


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


def load(route, n):
  events = []
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      events.extend(read_events(p))
  return events


def stock_tacc_ap(route, n):
  """genuine bus2, bucketed by real status 2(TACC) / 3(AP)"""
  events = load(route, n)
  cur_status = None
  data = defaultdict(lambda: defaultdict(list))
  for evt in events:
    if evt.which() != 'can':
      continue
    for c in evt.can:
      if c.src != 2:
        continue
      if c.address == 0x399:
        cur_status = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
        continue
      if cur_status not in (2, 3):
        continue
      msg = dbc.addr_to_msg.get(c.address)
      if msg is None:
        continue
      for addr, sname in SIGNALS:
        if addr != c.address or sname not in msg.sigs:
          continue
        sig = msg.sigs[sname]
        v = phys(sig, get_raw_value(bytes(c.dat), sig))
        data[cur_status][sname].append(v)
  return data


def ours(route, n):
  """our own real signals (src != 2) while engaged: 0x488 and EPAS/GTW are real bus0 ECUs (src=0),
  0x239 is our own spoof (src=128)."""
  events = load(route, n)
  enabled = False
  data = defaultdict(list)
  for evt in events:
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      if not enabled:
        continue
      for c in evt.can:
        if c.src == 2:
          continue  # skip genuine bus2 traffic, we want what reaches the cluster as OUR situation
        msg = dbc.addr_to_msg.get(c.address)
        if msg is None:
          continue
        for addr, sname in SIGNALS:
          if addr != c.address or sname not in msg.sigs:
            continue
          sig = msg.sigs[sname]
          v = phys(sig, get_raw_value(bytes(c.dat), sig))
          data[sname].append(v)
  return data


print('Loading stock (9f dashcam) TACC/AP baseline...')
stock = stock_tacc_ap('0000009f--b644363276', 3)
print('Loading our drive (9e engaged)...')
ourdata = ours('0000009e--7f4078b620', 6)

print(f'\n{"signal":28s} {"TACC(stock)":>16s} {"AP(stock)":>16s} {"OURS":>16s}  closer-to')
for addr, sname in SIGNALS:
  t = stock[2].get(sname, [])
  a = stock[3].get(sname, [])
  o = ourdata.get(sname, [])
  t_avg = sum(t) / len(t) if t else float('nan')
  a_avg = sum(a) / len(a) if a else float('nan')
  o_avg = sum(o) / len(o) if o else float('nan')
  closer = '?'
  if t and a and o:
    closer = 'TACC' if abs(o_avg - t_avg) < abs(o_avg - a_avg) else 'AP'
  print(f'{sname:28s} {t_avg:16.4f} {a_avg:16.4f} {o_avg:16.4f}  {closer}   '
        f'(n: T={len(t)} A={len(a)} O={len(o)})')
