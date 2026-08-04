#!/usr/bin/env python3
"""Histogram every (bus, address) in a recorded segment, and dump sample payloads
for a watchlist of addresses."""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys
from collections import defaultdict

import capnp
import zstandard
from cereal import log as capnp_log

WATCH = {0x102, 0x232, 0x302, 0x382}


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'scan-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    # Read the whole staged rlog and parse from bytes. A segment cut short by the car losing
    # power leaves a truncated final message, and the streaming reader aborts on it inside
    # libkj -- a C++ terminate that no Python except can catch, killing the whole run. Parsing
    # from bytes raises a normal KjException instead, after yielding every complete event.
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def main(paths):
  counts = defaultdict(int)
  samples = defaultdict(list)
  fingerprint = None

  for path in paths:
    for evt in read_events(path):
      w = evt.which()
      if w == 'carParams':
        fingerprint = evt.carParams.carFingerprint
      elif w == 'can':
        for f in evt.can:
          key = (f.src, f.address)
          counts[key] += 1
          if f.address in WATCH and len(samples[key]) < 5:
            samples[key].append(bytes(f.dat).hex())

  print(f'fingerprint: {fingerprint}')
  print(f'total distinct (bus,addr): {len(counts)}')
  print()
  print('=== WATCHLIST (0x102 0x232 0x302 0x382) ===')
  hit = False
  for (bus, addr), n in sorted(counts.items()):
    if addr in WATCH:
      hit = True
      print(f'  bus {bus}  0x{addr:03X} ({addr})  n={n}')
      for s in samples[(bus, addr)]:
        print(f'      {s}')
  if not hit:
    print('  (none present)')
  print()
  print('=== ALL (bus, addr) seen ===')
  for (bus, addr), n in sorted(counts.items()):
    print(f'  bus {bus}  0x{addr:03X} ({addr:5d})  n={n}')


if __name__ == '__main__':
  main(sys.argv[1:])
