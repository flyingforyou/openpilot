#!/usr/bin/env python3
"""DAS_lanes (0x239) named signals only cover 58 of 64 bits. Do the 6 unnamed bits carry real
content in genuine frames that our repack silently zeroes -- the same failure mode already found
and fixed for DAS_object's spare bits?"""

import os
import sys
from collections import Counter

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC as DBCLoader

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
dbc = DBCLoader(os.path.join(REPO_ROOT, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc'))
lanes_msg = dbc.addr_to_msg[0x239]

used_bits = set()
for sname, sig in lanes_msg.sigs.items():
  # little-endian signals: start_bit is the LSB position within the little-endian bit numbering
  # used by this DBC loader; bits occupy [start_bit, start_bit+size)
  for b in range(sig.start_bit, sig.start_bit + sig.size):
    used_bits.add(b)

all_bits = set(range(64))
spare_bits = sorted(all_bits - used_bits)
print(f'named signal bits: {len(used_bits)}/64')
print(f'spare (unnamed) bits: {spare_bits}')
for sname, sig in sorted(lanes_msg.sigs.items(), key=lambda kv: kv[1].start_bit):
  print(f'  {sname:28s} bits [{sig.start_bit},{sig.start_bit + sig.size})')


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


def spare_value(dat, bits):
  v = int.from_bytes(dat, byteorder='little')
  out = 0
  for i, b in enumerate(bits):
    out |= ((v >> b) & 1) << i
  return out


def scan(route, n, src_wanted, label):
  counts = Counter()
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if not os.path.exists(p):
      continue
    for evt in read_events(p):
      if evt.which() != 'can':
        continue
      for c in evt.can:
        if c.address == 0x239 and c.src == src_wanted:
          sv = spare_value(bytes(c.dat), spare_bits)
          counts[sv] += 1
  total = sum(counts.values())
  print(f'\n{label}  (n={total})')
  for v, n_ in sorted(counts.items(), key=lambda kv: -kv[1])[:10]:
    print(f'   spare_bits_value={v:2d} (0b{v:06b})  n={n_}  ({100*n_/total:.1f}%)')


print()
scan('0000009f--b644363276', 3, 2, 'genuine bus2 (dashcam route, real AP computer)')
scan('0000009e--7f4078b620', 6, 128, 'OUR TX (src=128, our repacked frame)')
scan('0000009e--7f4078b620', 6, 2, 'genuine bus2 during OUR drive (real AP computer, unspoofed)')
