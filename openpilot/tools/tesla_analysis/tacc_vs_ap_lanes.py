#!/usr/bin/env python3
"""TACC vs Autopilot: what actually differs on bus 2, isolated to same-road toggle windows."""

import os
import sys
from collections import defaultdict

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC as DBCLoader
from opendbc.can.parser import get_raw_value

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DBC_PATH = os.path.join(REPO_ROOT, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc')
dbc = DBCLoader(DBC_PATH)
status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']
lanes_msg = dbc.addr_to_msg[0x239]

LANE_FIELDS = ['DAS_leftLaneExists', 'DAS_rightLaneExists', 'DAS_virtualLaneWidth',
               'DAS_virtualLaneViewRange', 'DAS_virtualLaneC0', 'DAS_virtualLaneC1',
               'DAS_virtualLaneC2', 'DAS_virtualLaneC3', 'DAS_leftLineUsage',
               'DAS_rightLineUsage']


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


def segments(route, n):
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      yield p


def main():
  route = sys.argv[1]
  n_segs = int(sys.argv[2])

  cur_status = None
  t0 = None
  status_transitions = []  # (t, from, to)
  lane_by_status = defaultdict(lambda: defaultdict(list))
  # tight-window signal capture around every 2<->3 transition (+/- 5s)
  tacc_ap_windows = []

  events = []
  for path in segments(route, n_segs):
    for evt in read_events(path):
      events.append(evt)

  # pass 1: find transitions
  for evt in events:
    if evt.which() != 'can':
      continue
    t = evt.logMonoTime
    if t0 is None:
      t0 = t
    for c in evt.can:
      if c.src == 2 and c.address == 0x399:
        st = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
        if cur_status is not None and st != cur_status:
          status_transitions.append(((t - t0) / 1e9, cur_status, st))
        cur_status = st

  print(f'route={route}  total status transitions={len(status_transitions)}')
  to3 = [x for x in status_transitions if x[2] == 3 or x[1] == 3]
  print(f'transitions touching status=3 (Active_nominal): {len(to3)}')
  for t, frm, to in to3:
    print(f'  t={t:8.2f}s  {frm} -> {to}')

  # pass 2: bucket DAS_lanes fields by status, and by TACC(2)/AP(3) restricted to
  # windows within +/-6s of a 2<->3 transition (same-road, controls for road type)
  windows = [(t - 6.0, t + 6.0) for t, frm, to in status_transitions if frm in (2, 3) and to in (2, 3)]

  def in_window(t):
    return any(lo <= t <= hi for lo, hi in windows)

  cur_status = None
  speed_by_phase = defaultdict(list)
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
      if cur_status not in (2, 3):
        continue
      if c.address == 0x239:
        for fname in LANE_FIELDS:
          sig = lanes_msg.sigs[fname]
          v = phys(sig, get_raw_value(bytes(c.dat), sig))
          lane_by_status[cur_status][fname].append(v)
          if in_window(t):
            lane_by_status[(cur_status, 'windowed')][fname].append(v)
      elif c.address == 0x171:  # DI_torque2, has DI_vehicleSpeed
        dmsg = dbc.addr_to_msg[0x171]
        if 'DI_vehicleSpeed' in dmsg.sigs:
          sig = dmsg.sigs['DI_vehicleSpeed']
          v = phys(sig, get_raw_value(bytes(c.dat), sig))
          speed_by_phase[cur_status].append(v)
          if in_window(t):
            speed_by_phase[(cur_status, 'windowed')].append(v)

  print(f'\nnum same-road toggle windows (+/-6s around a 2<->3 transition): {len(windows)}')
  print('\n=== speed sanity check (confound control) ===')
  for k in [2, 3, (2, 'windowed'), (3, 'windowed')]:
    vals = speed_by_phase.get(k, [])
    if vals:
      print(f'  status={k}  n={len(vals):5d}  avg_speed_mps={sum(vals)/len(vals):.2f}')

  print('\n=== DAS_lanes fields: ALL data vs WINDOWED (same-road only) ===')
  for fname in LANE_FIELDS:
    row = []
    for k in [2, 3, (2, 'windowed'), (3, 'windowed')]:
      vals = lane_by_status.get(k, {}).get(fname, [])
      avg = sum(vals) / len(vals) if vals else float('nan')
      row.append(f'{k}:n={len(vals)},avg={avg:.4f}')
    print(f'  {fname:28s} ' + '  '.join(row))


if __name__ == '__main__':
  main()
