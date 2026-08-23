#!/usr/bin/env python3
"""Does the real AP computer stop sending some bus2 address entirely when its own autopilotStatus
is Unavailable (i.e. while we're driving) that it normally sends during genuine Active_nominal --
a message the cluster might need that isn't DAS_lanes/AutopilotStatus and that we never spoof?"""

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


def scan_bus2_by_status(route, n, wanted_statuses, label):
  """Rate (Hz) of every bus2 address, gated by real autopilotStatus in wanted_statuses."""
  events = []
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      events.extend(read_events(p))

  cur_status = None
  addr_counts = Counter()
  t_by_status = Counter()
  t0 = None
  prev_t = {}
  for evt in events:
    if evt.which() != 'can':
      continue
    t = evt.logMonoTime
    if t0 is None:
      t0 = t
    for c in evt.can:
      if c.src != 2:
        continue
      if c.address == 0x399:
        cur_status = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
      if cur_status in wanted_statuses:
        addr_counts[c.address] += 1
        t_by_status[cur_status] += 1

  print(f'\n=== {label} ({route}) === total qualifying frames: {sum(addr_counts.values())}')
  return addr_counts


def scan_ours(route, n):
  """Every bus0 address WE (src=128) transmit while engaged."""
  events = []
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      events.extend(read_events(p))
  enabled = False
  addr_counts = Counter()
  for evt in events:
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      if not enabled:
        continue
      for c in evt.can:
        if c.src == 128:
          addr_counts[c.address] += 1
  return addr_counts


addr_names = {addr: msg.name for addr, msg in dbc.addr_to_msg.items()}

genuine_active = scan_bus2_by_status('0000009f--b644363276', 3, {3}, 'genuine bus2 during status=3 (Active) -- dashcam')
genuine_active_9d = scan_bus2_by_status('0000009d--94840ba29c', 20, {3}, 'genuine bus2 during status=3 (Active) -- 9d')
genuine_unavail_ours = scan_bus2_by_status('0000009e--7f4078b620', 6, {1}, 'genuine bus2 during OUR engaged drive (status=1, Unavailable)')
ours_tx = scan_ours('0000009e--7f4078b620', 6)

all_addrs = set(genuine_active) | set(genuine_active_9d) | set(genuine_unavail_ours) | set(ours_tx)
print(f'\n{"addr":>6s} {"name":30s} {"genAP(9f)":>10s} {"genAP(9d)":>10s} {"genUnavail(9e)":>15s} {"our TX(9e)":>10s}')
for addr in sorted(all_addrs):
  name = addr_names.get(addr, '???')
  ga = genuine_active.get(addr, 0)
  ga9d = genuine_active_9d.get(addr, 0)
  gu = genuine_unavail_ours.get(addr, 0)
  ot = ours_tx.get(addr, 0)
  # flag: present during genuine Active but essentially absent during our engaged drive (both
  # from the real AP computer AND from us)
  flag = ''
  if (ga > 20 or ga9d > 50) and gu == 0 and ot == 0:
    flag = '  <-- MISSING during our drive entirely'
  print(f'0x{addr:04x} {name:30s} {ga:10d} {ga9d:10d} {gu:15d} {ot:10d}{flag}')
