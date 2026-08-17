#!/usr/bin/env python3
"""Replay the drive that faulted through the coop steering controller.

The question the unit tests cannot answer: on the real override where openpilot let go and the
DI faulted 0.12s after it grabbed back, how far would coop steering have moved the command
toward the driver, and would the driver have needed to push past 2.5Nm at all?
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import glob
import re
import sys

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

from opendbc.car.tesla.coop_steering import (CoopSteeringCarController, STEER_OVERRIDE_MAX_TORQUE,
                                            STEER_OVERRIDE_MIN_TORQUE)
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.vehicle_model import VehicleModel


msg = DBC('tesla_can').addr_to_msg[0x370]
SIGS = {n: msg.sigs[n] for n in ('EPAS_torsionBarTorque', 'EPAS_handsOnLevel', 'EPAS_internalSAS')}


class _Out:
  steeringTorque = 0.0
  steeringAngleDeg = 0.0
  steeringRateDeg = 0.0
  vEgo = 0.0
  vEgoRaw = 0.0


class _CS:
  out = _Out()


def seg_no(path):
  m = re.search(r'--(\d+)/rlog\.zst$', path)
  return int(m.group(1)) if m else 0


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'rc-{os.getpid()}.rlog')
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


route, lo, hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
coop = CoopSteeringCarController()
vm = VehicleModel(CarInterface.get_non_essential_params('TESLA_MODEL_S_HW3'))
cs = _CS()

t0 = None
v_ego = 0.0
rows = []
for path in sorted(glob.glob(os.path.join(LOG_ROOT, f'{route}--*/rlog.zst')), key=seg_no):
  for evt in read_events(path):
    w = evt.which()
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if w == 'carState':
      v_ego = evt.carState.vEgo
    if w != 'can' or dt < lo or dt > hi:
      continue
    for f in evt.can:
      if f.src != 0 or f.address != 0x370 or len(f.dat) < 8:
        continue
      d = bytes(f.dat)
      vals = {n: get_raw_value(d, s) * s.factor + s.offset for n, s in SIGS.items()}
      angle = -vals['EPAS_internalSAS']
      torque = -vals['EPAS_torsionBarTorque']
      hol = int(vals['EPAS_handsOnLevel'])

      cs.out.steeringTorque = torque
      cs.out.steeringAngleDeg = angle
      cs.out.vEgo = cs.out.vEgoRaw = v_ego
      # openpilot's target is what it was holding: approximate with the measured angle at entry
      out = coop.update(angle, hol < 3, cs, vm).steeringAngleDeg
      rows.append((dt, torque, hol, angle, out, coop.override_angle_accu))

print(f'{route} +{lo:.0f}..{hi:.0f}s   {len(rows)} EPAS frames')
print(f'\n{"t":>9s} {"driver Nm":>10s} {"handsOn":>8s} {"wheel deg":>10s} {"coop cmd":>9s} {"offset":>8s}')
for dt, tq, hol, angle, out, accu in rows:
  mark = ''
  if abs(tq) > STEER_OVERRIDE_MAX_TORQUE:
    mark = '  <-- past 2.5Nm'
  if hol >= 3:
    mark += '  handsOn=3'
  print(f'{dt:9.2f} {tq:10.2f} {hol:8d} {angle:10.1f} {out:9.1f} {accu:8.2f}{mark}')

past = [r for r in rows if abs(r[1]) > STEER_OVERRIDE_MAX_TORQUE]
inside = [r for r in rows if STEER_OVERRIDE_MIN_TORQUE < abs(r[1]) <= STEER_OVERRIDE_MAX_TORQUE]
print(f'\nframes with driver torque inside the coop band (0.5-2.5Nm): {len(inside)}')
print(f'frames past 2.5Nm (coop saturated, EPS could still inhibit): {len(past)}')
