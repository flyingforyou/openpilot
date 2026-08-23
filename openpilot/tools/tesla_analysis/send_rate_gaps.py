#!/usr/bin/env python3
"""Frame-to-frame gap distribution for 0x239/0x399: are we sending at a comparable, steady rate to
genuine AP, or do we have long gaps that could trip a cluster watchdog/timeout even though content
matches?"""

import os
import sys

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log


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


def gaps_for(route, n, addr, src_filter, status_gate=None):
  """status_gate: None = no gate; 'enabled' = only while selfdriveState.enabled;
  int = only while genuine bus2 autopilotStatus == that value (for the dashcam/genuine route)"""
  enabled = False
  cur_status = None
  from opendbc.can.dbc import DBC as DBCLoader
  from opendbc.can.parser import get_raw_value
  repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
  dbc = DBCLoader(os.path.join(repo_root, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc'))
  status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']

  ts = []
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
          if c.src == 2 and c.address == 0x399:
            cur_status = int(get_raw_value(bytes(c.dat), status_sig) * status_sig.factor + status_sig.offset)
          if c.address != addr or c.src not in src_filter:
            continue
          if status_gate == 'enabled' and not enabled:
            continue
          if isinstance(status_gate, int) and cur_status != status_gate:
            continue
          ts.append(evt.logMonoTime / 1e9)

  ts.sort()
  gaps = [b - a for a, b in zip(ts, ts[1:])]
  return ts, gaps


def report(label, route, n, addr, src_filter, status_gate):
  ts, gaps = gaps_for(route, n, addr, src_filter, status_gate)
  if not gaps:
    print(f'{label}: no data')
    return
  gaps.sort()
  n_ = len(gaps)
  avg = sum(gaps) / n_
  p50 = gaps[n_ // 2]
  p95 = gaps[int(n_ * 0.95)]
  p99 = gaps[int(n_ * 0.99)]
  mx = gaps[-1]
  n_big = sum(1 for g in gaps if g > 0.3)
  print(f'{label}: n_frames={len(ts)}  n_gaps={n_}  avg={avg*1000:.1f}ms  p50={p50*1000:.1f}ms  '
        f'p95={p95*1000:.1f}ms  p99={p99*1000:.1f}ms  max={mx*1000:.1f}ms  gaps>300ms={n_big}')


print('--- 0x239 (DAS_lanes) ---')
report('ours (9e, engaged, our TX src=128)', '0000009e--7f4078b620', 6, 0x239, {128}, 'enabled')
report('genuine (9f dashcam, status==3)', '0000009f--b644363276', 3, 0x239, {2}, 3)
report('genuine (9d, status==3)', '0000009d--94840ba29c', 20, 0x239, {2}, 3)

print('\n--- 0x399 (AutopilotStatus) ---')
report('ours (9e, engaged, our TX src=128)', '0000009e--7f4078b620', 6, 0x399, {128}, 'enabled')
report('genuine (9f dashcam, status==3)', '0000009f--b644363276', 3, 0x399, {2}, 3)
report('genuine (9d, status==3)', '0000009d--94840ba29c', 20, 0x399, {2}, 3)
