#!/usr/bin/env python3
"""What torque the driver actually applied, and where the EPS drew its lines.

CarrotPilot's cooperative steering is built around keeping driver torque under
STEER_OVERRIDE_MAX_TORQUE = 2.5Nm, "max torque before EPS disengages". That number comes from
HW3's EPAS3S. HW1 is a different EPS generation, so measure our own before reusing it.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import glob
import re
import sys
from collections import Counter

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value


DBC_NAME = 'tesla_can'
EAC_STATUS = {0: 'EAC_INHIBITED', 1: 'EAC_AVAILABLE', 2: 'EAC_ACTIVE', 3: 'EAC_FAULT'}
EAC_ERR = {0: 'IDLE', 3: 'HANDS_ON', 6: 'HIGH_ANGLE_REQ', 9: 'HIGH_ANGLE_RATE_SAFETY'}

msg = DBC(DBC_NAME).addr_to_msg[0x370]
SIGS = {n: msg.sigs[n] for n in ('EPAS_torsionBarTorque', 'EPAS_handsOnLevel',
                                 'EPAS_eacStatus', 'EPAS_eacErrorCode')}


def decode(dat: bytes) -> dict:
  out = {}
  for name, s in SIGS.items():
    raw = get_raw_value(dat, s)
    out[name] = raw * s.factor + s.offset
  return out


def seg_no(path):
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'tq-{os.getpid()}.rlog')
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


for route in sys.argv[1:]:
  torques = []
  by_hol: dict[int, list[float]] = {}
  at_inhibit = []          # torque on the frame EPAS went to EAC_INHIBITED
  at_hol3 = []             # torque on the frame handsOnLevel first hit 3
  prev_eac = prev_hol = None
  eac_hist = Counter()

  for path in sorted(glob.glob(os.path.join(LOG_ROOT, f'{route}--*/rlog.zst')), key=seg_no):
    for evt in read_events(path):
      if evt.which() != 'can':
        continue
      for f in evt.can:
        if f.src != 0 or f.address != 0x370 or len(f.dat) < 8:
          continue
        d = decode(bytes(f.dat))
        # carstate negates it; magnitude is what matters here
        tq = abs(d['EPAS_torsionBarTorque'])
        hol = int(d['EPAS_handsOnLevel'])
        eac = int(d['EPAS_eacStatus'])
        torques.append(tq)
        by_hol.setdefault(hol, []).append(tq)
        eac_hist[EAC_STATUS.get(eac, eac)] += 1
        if eac == 0 and prev_eac not in (0, None):
          at_inhibit.append((tq, EAC_ERR.get(int(d['EPAS_eacErrorCode']), int(d['EPAS_eacErrorCode']))))
        if hol >= 3 and (prev_hol is None or prev_hol < 3):
          at_hol3.append(tq)
        prev_eac, prev_hol = eac, hol

  if not torques:
    print(f'\n===== {route}: no EPAS frames =====')
    continue

  torques.sort()
  def pct(p):
    return torques[min(int(len(torques) * p / 100), len(torques) - 1)]

  print(f'\n===== {route} =====  ({len(torques)} EPAS frames)')
  print(f'  |torque|  p50={pct(50):.2f}  p90={pct(90):.2f}  p99={pct(99):.2f}  '
        f'p99.9={pct(99.9):.2f}  max={torques[-1]:.2f} Nm')
  print(f'  eacStatus: {dict(eac_hist)}')
  print(f'  handsOnLevel별 |torque| 평균 / 최대:')
  for hol in sorted(by_hol):
    v = by_hol[hol]
    print(f'     handsOn={hol}  n={len(v):6d}  평균 {sum(v) / len(v):.2f}  최대 {max(v):.2f} Nm')
  if at_hol3:
    print(f'  handsOnLevel이 3에 처음 닿은 순간의 |torque| ({len(at_hol3)}회): '
          f'{", ".join(f"{t:.2f}" for t in at_hol3[:12])}')
    print(f'     → 최소 {min(at_hol3):.2f}  중간 {sorted(at_hol3)[len(at_hol3) // 2]:.2f}  최대 {max(at_hol3):.2f} Nm')
  if at_inhibit:
    print(f'  EAC_INHIBITED 진입 순간의 |torque| ({len(at_inhibit)}회):')
    for t, why in at_inhibit[:12]:
      print(f'     {t:.2f} Nm  ({why})')
