#!/usr/bin/env python3
"""Full bus2 signal diff between TACC(2) and Active_nominal(3), restricted to same-road windows
around real state transitions -- controls for the road-type confound that speed/torque/RPM showed
in the unwindowed version."""

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


route = sys.argv[1]
n_segs = int(sys.argv[2])
window_s = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0

events = []
for i in range(n_segs):
  p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
  if os.path.exists(p):
    events.extend(read_events(p))

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

windows = [(t - window_s, t + window_s) for t in transitions]
print(f'route={route}  same-road toggle windows: {len(windows)}')


def in_window(t):
  return any(lo <= t <= hi for lo, hi in windows)


cur_status = None
skip_words = ('Counter', 'Checksum', 'checksum', 'counter')
data = defaultdict(lambda: defaultdict(list))
for evt in events:
  if evt.which() != 'can':
    continue
  t = (evt.logMonoTime - t0) / 1e9
  if not in_window(t):
    continue
  for c in evt.can:
    if c.src != 2:
      continue
    if c.address == 0x399:
      cur_status = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
      continue
    if cur_status not in (2, 3):
      continue
    dmsg = dbc.addr_to_msg.get(c.address)
    if dmsg is None:
      continue
    phase = 'TACC' if cur_status == 2 else 'AP'
    for sname, sig in dmsg.sigs.items():
      if any(w in sname for w in skip_words):
        continue
      try:
        raw = get_raw_value(bytes(c.dat), sig)
      except Exception:
        continue
      v = phys(sig, raw)
      data[(dmsg.name, sname)][phase].append(v)

rows = []
for key, phases in data.items():
  tacc = phases.get('TACC', [])
  ap = phases.get('AP', [])
  if len(tacc) < 10 or len(ap) < 10:
    continue
  avg_t = sum(tacc) / len(tacc)
  avg_a = sum(ap) / len(ap)
  denom = abs(avg_t) + abs(avg_a) + 1e-6
  score = abs(avg_a - avg_t) / denom
  rows.append((score, key, avg_t, avg_a, len(tacc), len(ap), min(tacc), max(tacc), min(ap), max(ap)))

rows.sort(reverse=True)
print(f'{"signal":45s} {"avg_TACC":>10s} {"avg_AP":>10s} {"score":>7s} {"n_T":>6s} {"n_A":>6s}   TACC[min,max]        AP[min,max]')
for score, key, avg_t, avg_a, nt, na, tmin, tmax, amin, amax in rows[:70]:
  name = f'{key[0]}.{key[1]}'
  print(f'{name:45s} {avg_t:10.4f} {avg_a:10.4f} {score:7.3f} {nt:6d} {na:6d}   [{tmin:.3f},{tmax:.3f}]  [{amin:.3f},{amax:.3f}]')
