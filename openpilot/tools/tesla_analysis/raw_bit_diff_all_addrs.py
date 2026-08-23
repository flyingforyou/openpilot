#!/usr/bin/env python3
"""Bit-level TACC-vs-AP diff across EVERY bus2 address, named or not -- previous scans only walked
DBC-named signals, so an undocumented arbitration ID that happens to gate cluster rendering would
never have been examined. Same-road windows (+/-Ns around real 2<->3 transitions) control for the
road-type confound."""

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
addr_names = {addr: msg.name for addr, msg in dbc.addr_to_msg.items()}


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


route = sys.argv[1] if len(sys.argv) > 1 else '0000009d--94840ba29c'
n_segs = int(sys.argv[2]) if len(sys.argv) > 2 else 20
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
print(f'route={route}  same-road windows: {len(windows)}')


def in_window(t):
  return any(lo <= t <= hi for lo, hi in windows)


# bit_counts[(addr, bit_index)][phase] = [count_0, count_1]
bit_counts = defaultdict(lambda: {'TACC': [0, 0], 'AP': [0, 0]})
# byte_values[(addr, byte_index)][phase] = list of raw 0-255 values seen -- catches a multi-bit
# field with a real magnitude difference that no single bit shows cleanly on its own
byte_values = defaultdict(lambda: {'TACC': [], 'AP': []})
cur_status = None
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
    phase = 'TACC' if cur_status == 2 else 'AP'
    dat = bytes(c.dat)
    v = int.from_bytes(dat, byteorder='little')
    nbits = len(dat) * 8
    for b in range(nbits):
      bit = (v >> b) & 1
      bit_counts[(c.address, b)][phase][bit] += 1
    for bi, byte_val in enumerate(dat):
      byte_values[(c.address, bi)][phase].append(byte_val)

# score each (addr,bit): purity of TACC-mostly-0/AP-mostly-1 or reverse, with enough samples
rows = []
for (addr, bit), phases in bit_counts.items():
  t0_, t1_ = phases['TACC']
  a0_, a1_ = phases['AP']
  nt = t0_ + t1_
  na = a0_ + a1_
  if nt < 20 or na < 20:
    continue
  t_frac1 = t1_ / nt
  a_frac1 = a1_ / na
  diff = abs(a_frac1 - t_frac1)
  rows.append((diff, addr, bit, t_frac1, a_frac1, nt, na))

rows.sort(reverse=True)
print(f'\n=== bit-level (binary-flip) diffs ===')
print(f'{"addr":>8s} {"name":24s} {"bit":>4s} {"TACC frac(1)":>13s} {"AP frac(1)":>11s} {"diff":>6s} {"n_T":>6s} {"n_A":>6s}')
for diff, addr, bit, tf, af, nt, na in rows[:40]:
  name = addr_names.get(addr, '???')
  print(f'0x{addr:04x} {name:24s} {bit:4d} {tf:13.3f} {af:11.3f} {diff:6.3f} {nt:6d} {na:6d}')

byte_rows = []
for (addr, bi), phases in byte_values.items():
  t = phases['TACC']
  a = phases['AP']
  if len(t) < 20 or len(a) < 20:
    continue
  t_avg = sum(t) / len(t)
  a_avg = sum(a) / len(a)
  denom = abs(t_avg) + abs(a_avg) + 1e-6
  score = abs(a_avg - t_avg) / denom
  byte_rows.append((score, addr, bi, t_avg, a_avg, min(t), max(t), min(a), max(a), len(t), len(a)))

byte_rows.sort(reverse=True)
print(f'\n=== byte-level (raw value 0-255) magnitude diffs -- catches multi-bit fields ===')
print(f'{"addr":>8s} {"name":24s} {"byte":>4s} {"TACC avg":>9s} {"AP avg":>9s} {"score":>6s}   TACC[min,max]   AP[min,max]   n_T/n_A')
for score, addr, bi, ta, aa, tmin, tmax, amin, amax, nt, na in byte_rows[:50]:
  name = addr_names.get(addr, '???')
  print(f'0x{addr:04x} {name:24s} {bi:4d} {ta:9.2f} {aa:9.2f} {score:6.3f}   [{tmin:3.0f},{tmax:3.0f}]      [{amin:3.0f},{amax:3.0f}]      {nt}/{na}')
