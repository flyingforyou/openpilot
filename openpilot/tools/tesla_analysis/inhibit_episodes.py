#!/usr/bin/env python3
"""How long EPAS stayed inhibited, and what openpilot was doing while it was."""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys

import capnp
import zstandard
from cereal import log as capnp_log

EAC_STATUS = {0: 'EAC_INHIBITED', 1: 'EAC_AVAILABLE', 2: 'EAC_ACTIVE', 3: 'EAC_FAULT'}
EAC_ERR = {0: 'IDLE', 1: 'MIN_SPEED', 2: 'MAX_SPEED', 3: 'HANDS_ON', 4: 'TMP_FAULT',
           5: 'MAX_STEER_DELTA', 6: 'HIGH_ANGLE_REQ', 7: 'HIGH_ANGLE_RATE_REQ',
           8: 'HIGH_ANGLE_SAFETY', 9: 'HIGH_ANGLE_RATE_SAFETY', 10: 'HIGH_MMOT_SAFETY',
           11: 'HIGH_TORSION_SAFETY', 12: 'LOW_ASSIST', 13: 'PINION_VEL_DIFF',
           14: 'EPB_INHIBIT', 15: 'SNA'}


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'ie-{os.getpid()}.rlog')
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


episodes = []
cur = None
t0 = None
enabled = False
hol = 0
op_type = None
for path in sys.argv[1:]:
  for evt in read_events(path):
    w = evt.which()
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    if w == 'selfdriveState':
      enabled = evt.selfdriveState.enabled
    elif w == 'can':
      for f in evt.can:
        d = bytes(f.dat)
        if f.src == 0 and f.address == 0x488 and len(d) >= 3:
          op_type = d[2] >> 6
        elif f.src == 0 and f.address == 0x370 and len(d) >= 7:
          eac, err = d[6] >> 5, d[2] >> 4
          hol = d[4] >> 6
          if eac == 0:   # EAC_INHIBITED
            if cur is None:
              cur = {'start': t, 'last': t, 'errs': set(), 'hol_max': 0,
                     'enabled': 0, 'n': 0, 'op_angle': 0}
            cur['last'] = t
            cur['errs'].add(EAC_ERR.get(err, err))
            cur['hol_max'] = max(cur['hol_max'], hol)
            cur['enabled'] += bool(enabled)
            cur['op_angle'] += (op_type == 1)
            cur['n'] += 1
          elif cur is not None:
            episodes.append(cur)
            cur = None
if cur is not None:
  episodes.append(cur)

episodes.sort(key=lambda e: -(e['last'] - e['start']))
total = sum(e['last'] - e['start'] for e in episodes)
print(f'EAC_INHIBITED 구간: {len(episodes)}개, 총 {total:.1f}초')
print(f'\n{"길이":>8s}  {"시작":>9s}  {"handsOn최대":>10s}  {"OP engaged":>10s}  {"OP각도명령":>10s}  사유')
for e in episodes[:25]:
  dur = e['last'] - e['start']
  print(f'  {dur:6.2f}s  +{e["start"] - t0:8.1f}s  {e["hol_max"]:10d}  '
        f'{100 * e["enabled"] / max(e["n"], 1):9.0f}%  {100 * e["op_angle"] / max(e["n"], 1):9.0f}%  '
        f'{",".join(sorted(e["errs"]))}')
