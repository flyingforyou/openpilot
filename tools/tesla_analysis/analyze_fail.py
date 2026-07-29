#!/usr/bin/env python3
"""What the car and openpilot were doing when steering locked out and autopark failed.

Reads the flags out of the log's own CarParams first: a feature that was never actually enabled
explains a failure very differently from one that was.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import sys
from collections import Counter

import capnp
import zstandard
from cereal import log as capnp_log


EAC_STATUS = {0: 'EAC_INHIBITED', 1: 'EAC_AVAILABLE', 2: 'EAC_ACTIVE', 3: 'EAC_FAULT'}
EAC_ERR = {0: 'IDLE', 1: 'MIN_SPEED', 2: 'MAX_SPEED', 3: 'HANDS_ON', 4: 'TMP_FAULT',
           5: 'MAX_STEER_DELTA', 6: 'HIGH_ANGLE_REQ', 7: 'HIGH_ANGLE_RATE_REQ',
           8: 'HIGH_ANGLE_SAFETY', 9: 'HIGH_ANGLE_RATE_SAFETY', 10: 'HIGH_MMOT_SAFETY',
           11: 'HIGH_TORSION_SAFETY', 12: 'LOW_ASSIST', 13: 'PINION_VEL_DIFF',
           14: 'EPB_INHIBIT', 15: 'SNA'}
STEER_TYPE = {0: 'NONE', 1: 'ANGLE_CONTROL', 2: 'LANE_KEEP_ASSIST', 3: 'EMERG_LKA'}
ACC_STATE = {0: 'ACC_CANCEL_GENERIC', 3: 'ACC_HOLD', 4: 'ACC_ON', 5: 'APC_BACKWARD',
             6: 'APC_FORWARD', 7: 'APC_COMPLETE', 8: 'APC_ABORT', 9: 'APC_PAUSE',
             10: 'APC_UNPARK_COMPLETE', 11: 'APC_SELFPARK_START',
             13: 'ACC_CANCEL_GENERIC_SILENT', 15: 'FAULT_SNA'}


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'af-{os.getpid()}.rlog')
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


def analyze(paths):
  cp_seen = False
  transitions = []
  state = {}
  eac_hist, err_hist, hol_hist = Counter(), Counter(), Counter()
  event_counts = Counter()
  t0 = None

  for path in paths:
    seg = os.path.basename(os.path.dirname(path))
    for evt in read_events(path):
      w = evt.which()
      t = evt.logMonoTime / 1e9
      if t0 is None:
        t0 = t

      if w == 'carParams' and not cp_seen:
        cp = evt.carParams
        cp_seen = True
        print(f'=== CarParams ({seg}) ===')
        print(f'  fingerprint : {cp.carFingerprint}')
        print(f'  flags       : {cp.flags}   COOP_STEER(8)={"ON" if cp.flags & 8 else "off"}')
        for sc in cp.safetyConfigs:
          print(f'  safety      : {sc.safetyModel} param={sc.safetyParam} '
                f'HW1(8)={"y" if sc.safetyParam & 8 else "n"} '
                f'AUTOPARK(64)={"ON" if sc.safetyParam & 64 else "off"}')
        print()

      elif w == 'onroadEvents':
        for e in evt.onroadEvents:
          event_counts[str(e.name)] += 1

      elif w == 'carState':
        cs = evt.carState
        cur = {
          'pressed': bool(cs.steeringPressed), 'diseng': bool(cs.steeringDisengage),
          'ftmp': bool(cs.steerFaultTemporary), 'fperm': bool(cs.steerFaultPermanent),
        }
        for k, v in cur.items():
          if state.get(k) != v:
            state[k] = v
            transitions.append((t - t0, seg, f'carState.{k}', v))

      elif w == 'carControl':
        cc = evt.carControl
        cur = {'cc.enabled': bool(cc.enabled), 'cc.latActive': bool(cc.latActive),
               'cc.longActive': bool(cc.longActive), 'cc.cancel': bool(cc.cruiseControl.cancel)}
        for k, v in cur.items():
          if state.get(k) != v:
            state[k] = v
            transitions.append((t - t0, seg, k, v))

      elif w == 'selfdriveState':
        ss = evt.selfdriveState
        cur = {'sds.enabled': bool(ss.enabled), 'sds.active': bool(ss.active),
               'sds.state': str(ss.state)}
        for k, v in cur.items():
          if state.get(k) != v:
            state[k] = v
            transitions.append((t - t0, seg, k, v))

      elif w == 'can':
        for f in evt.can:
          d = bytes(f.dat)
          if f.src == 0 and f.address == 0x370 and len(d) >= 7:
            hol, eac, err = d[4] >> 6, d[6] >> 5, d[2] >> 4
            hol_hist[hol] += 1
            eac_hist[EAC_STATUS.get(eac, eac)] += 1
            err_hist[EAC_ERR.get(err, err)] += 1
            for k, v in (('epas.eac', EAC_STATUS.get(eac, eac)),
                         ('epas.err', EAC_ERR.get(err, err)),
                         ('epas.handsOn', hol)):
              if state.get(k) != v:
                state[k] = v
                transitions.append((t - t0, seg, k, v))
          elif f.src == 0 and f.address == 0x488 and len(d) >= 3:
            ty = STEER_TYPE.get(d[2] >> 6)
            if state.get('op.steerType') != ty:
              state['op.steerType'] = ty
              transitions.append((t - t0, seg, 'op.steerType(bus0)', ty))
          elif f.src == 2 and f.address == 0x2B9 and len(d) >= 2:
            st = ACC_STATE.get((d[1] >> 4) & 0x0F, (d[1] >> 4) & 0x0F)
            if state.get('stock.accState') != st:
              state['stock.accState'] = st
              transitions.append((t - t0, seg, 'stock.accState(bus2)', st))
          elif f.src == 2 and f.address == 0x488 and len(d) >= 3:
            ty = STEER_TYPE.get(d[2] >> 6)
            if state.get('stock.steerType') != ty:
              state['stock.steerType'] = ty
              transitions.append((t - t0, seg, 'stock.steerType(bus2)', ty))

  print('=== EPAS histograms ===')
  print(f'  handsOnLevel : {dict(sorted(hol_hist.items()))}')
  print(f'  eacStatus    : {dict(eac_hist)}')
  print(f'  eacErrorCode : {dict(err_hist)}')
  print(f'\n=== onroadEvents ===')
  for n, c in event_counts.most_common(25):
    print(f'  {c:7d}  {n}')
  print(f'\n=== transitions ({len(transitions)}) ===')
  for dt, seg, k, v in transitions:
    print(f'  +{dt:8.2f}s  {seg[-3:]}  {k:24s} = {v}')


if __name__ == '__main__':
  analyze(sys.argv[1:])
