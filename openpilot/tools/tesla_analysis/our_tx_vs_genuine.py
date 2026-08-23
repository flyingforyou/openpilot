#!/usr/bin/env python3
"""Compare what we actually transmit for DAS_lanes (bus0, our own TX) against genuine factory
DAS_lanes (bus2, src==2) from a same-road dashcam-mode drive, to check whether our override is
firing at all and whether the value ranges are comparable."""

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
lanes_msg = dbc.addr_to_msg[0x239]
LANE_FIELDS = ['DAS_leftLaneExists', 'DAS_rightLaneExists', 'DAS_virtualLaneWidth',
               'DAS_virtualLaneViewRange', 'DAS_virtualLaneC0', 'DAS_virtualLaneC1',
               'DAS_virtualLaneC2', 'DAS_virtualLaneC3', 'DAS_leftLineUsage', 'DAS_rightLineUsage']


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


def load_route(route, n):
  events = []
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if os.path.exists(p):
      events.extend(read_events(p))
  return events


def analyze_ours(route, n):
  """route where openpilot is engaged: what do WE actually put on bus0 addr 0x239?"""
  events = load_route(route, n)
  enabled = False
  data = defaultdict(list)
  src_counter = Counter()
  n_frames_while_enabled = 0
  for evt in events:
    w = evt.which()
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      for c in evt.can:
        if c.address != 0x239:
          continue
        src_counter[c.src] += 1
        if not enabled or c.src == 2:
          continue
        n_frames_while_enabled += 1
        for fname in LANE_FIELDS:
          sig = lanes_msg.sigs[fname]
          v = phys(sig, get_raw_value(bytes(c.dat), sig))
          data[fname].append(v)
  print(f'route={route} (openpilot-engaged run)')
  print(f'  0x239 frames by src: {dict(src_counter)}')
  print(f'  0x239 frames while enabled (non bus2): {n_frames_while_enabled}')
  for fname in LANE_FIELDS:
    vals = data[fname]
    if not vals:
      print(f'  {fname:28s} NO DATA')
      continue
    avg = sum(vals) / len(vals)
    print(f'  {fname:28s} n={len(vals):5d}  avg={avg:8.4f}  min={min(vals):8.4f}  max={max(vals):8.4f}')
  return data


def analyze_genuine(route, n):
  """dashcam-mode route: genuine bus2 DAS_lanes, bucketed by real autopilotStatus"""
  events = load_route(route, n)
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
      if c.address == 0x239 and cur_status is not None:
        for fname in LANE_FIELDS:
          sig = lanes_msg.sigs[fname]
          v = phys(sig, get_raw_value(bytes(c.dat), sig))
          data[cur_status][fname].append(v)
  print(f'\nroute={route} (dashcam/genuine run)')
  for st in sorted(data):
    n_ = len(data[st]['DAS_leftLaneExists'])
    print(f'  -- genuine status={st}  n={n_} --')
    for fname in LANE_FIELDS:
      vals = data[st][fname]
      if not vals:
        continue
      avg = sum(vals) / len(vals)
      print(f'    {fname:28s} avg={avg:8.4f}  min={min(vals):8.4f}  max={max(vals):8.4f}')
  return data


ours = analyze_ours(sys.argv[1], int(sys.argv[2]))
genuine = analyze_genuine(sys.argv[3], int(sys.argv[4]))
