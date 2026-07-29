#!/usr/bin/env python3
"""Everything that happened in a time window, in order."""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys

import capnp
import zstandard
from cereal import log as capnp_log

LO, HI = float(sys.argv[1]), float(sys.argv[2])
EAC_STATUS = {0: 'EAC_INHIBITED', 1: 'EAC_AVAILABLE', 2: 'EAC_ACTIVE', 3: 'EAC_FAULT'}
EAC_ERR = {0: 'IDLE', 3: 'HANDS_ON', 4: 'TMP_FAULT', 6: 'HIGH_ANGLE_REQ', 9: 'HIGH_ANGLE_RATE_SAFETY'}
STEER_TYPE = {0: 'NONE', 1: 'ANGLE_CONTROL', 2: 'LKA', 3: 'EMERG'}


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'w-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      try:
        yield from capnp_log.Event.read_multiple(f)
      except capnp.KjException:
        pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


t0 = None
state = {}
out = []
prev_events = set()
for path in sys.argv[3:]:
  for evt in read_events(path):
    w = evt.which()
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if dt < LO or dt > HI:
      continue

    def note(k, v):
      if state.get(k) != v:
        state[k] = v
        out.append((dt, k, v))

    if w == 'onroadEvents':
      names = {str(e.name) for e in evt.onroadEvents}
      for n in names - prev_events:
        out.append((dt, 'EVENT+', n))
      for n in prev_events - names:
        out.append((dt, 'EVENT-', n))
      prev_events = names
    elif w == 'selfdriveState':
      note('sds.state', str(evt.selfdriveState.state))
      note('sds.enabled', bool(evt.selfdriveState.enabled))
    elif w == 'carControl':
      note('cc.enabled', bool(evt.carControl.enabled))
      note('cc.latActive', bool(evt.carControl.latActive))
      note('cc.longActive', bool(evt.carControl.longActive))
      note('cc.cancel', bool(evt.carControl.cruiseControl.cancel))
    elif w == 'carState':
      cs = evt.carState
      note('cs.steeringPressed', bool(cs.steeringPressed))
      note('cs.steeringDisengage', bool(cs.steeringDisengage))
      note('cs.steerFaultTemporary', bool(cs.steerFaultTemporary))
      note('cs.steerFaultPermanent', bool(cs.steerFaultPermanent))
      note('cs.accFaulted', bool(cs.accFaulted))
      note('cs.cruiseState.enabled', bool(cs.cruiseState.enabled))
      note('cs.cruiseState.available', bool(cs.cruiseState.available))
      note('cs.gearShifter', str(cs.gearShifter))
    elif w == 'can':
      for f in evt.can:
        d = bytes(f.dat)
        if f.src == 0 and f.address == 0x370 and len(d) >= 7:
          note('epas.eac', EAC_STATUS.get(d[6] >> 5))
          note('epas.err', EAC_ERR.get(d[2] >> 4, d[2] >> 4))
          note('epas.handsOn', d[4] >> 6)
        elif f.src == 0 and f.address == 0x488 and len(d) >= 3:
          note('op.steer(bus0)', STEER_TYPE.get(d[2] >> 6))
        elif f.src == 2 and f.address == 0x488 and len(d) >= 3:
          note('stock.steer(bus2)', STEER_TYPE.get(d[2] >> 6))
        elif f.src == 0 and f.address == 0x368 and len(d) >= 2:
          note('DI_cruiseState', (d[1] >> 4) & 0x07)

for dt, k, v in out:
  print(f'  +{dt:8.2f}s  {k:24s} {v}')
