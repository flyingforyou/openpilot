#!/usr/bin/env python3
"""Put the cars back on the instrument cluster after Tesla's 2026.26.1 regression.

Since that update, AP1 cars stop drawing most traffic on the cluster. Other owners report the
same and the logs pin it exactly: the gate is DAS_objVehType in DAS_object (0x309). TRUCK and
MOTORCYCLE still render; CAR does not, and CAR is 78% of everything the car sees. Nothing else is
wrong -- the camera classifies correctly, the message reaches the cluster's bus at 33 Hz, and a
car sitting dead ahead at 16 m goes undrawn while a motorcycle at 45 m in the same slot a minute
later is drawn.

So this reads the factory's own object list and puts it back on the bus with one field changed.
Everything else -- distance, lateral offset, closing speed, object id, the group rotation, the
timing -- is the factory's, untouched, because the factory has it right.

  mirror   the default. Rewrite CAR to TRUCK and re-send. Cars appear again, wearing a truck
           icon, which is the price of working around someone else's bug.

  sweep    inject one synthetic vehicle at a fixed spot and step the type through all eight
           values. Use this first if you want to know whether some other value draws something
           nicer than a truck -- this car has never emitted BICYCLE, PEDESTRIAN, IPSO or the
           undefined 7, so what the cluster does with them is unknown.

Running it
----------
Stationary, in Park, systems awake, and openpilot not running or it will fight for the panda:

    tmux kill-session -t comma
    python selfdrive/debug/tesla_cluster_probe.py            # mirror
    python selfdrive/debug/tesla_cluster_probe.py --sweep    # explore the type values

Both modes leave the factory still transmitting its own copy, so the two interleave and the
display may flicker. Blocking the factory's copy needs 0x309 added to the panda TX list and a
forwarding rule; that is the tidy version, and this is the one that needs no firmware change.

Safety
------
allOutput is entered with the passthrough flag set. That flag is not optional: without it the
safety layer stops forwarding between the buses and the car is left without the factory messages
it needs to drive. DAS_object commands nothing -- the cluster draws from it -- but stop this
before driving anyway.
"""
import argparse
import time

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from panda import Panda

DAS_OBJECT = 0x309
DAS_BUS = 2        # where the factory module talks
CLUSTER_BUS = 0    # where the cluster listens

UNKNOWN, TRUCK, CAR, MOTORCYCLE, BICYCLE, PEDESTRIAN, IPSO = range(7)
TYPE_NAMES = {UNKNOWN: 'UNKNOWN', TRUCK: 'TRUCK', CAR: 'CAR', MOTORCYCLE: 'MOTORCYCLE',
              BICYCLE: 'BICYCLE', PEDESTRIAN: 'PEDESTRIAN', IPSO: 'IPSO', 7: '(undefined)'}
GROUP_NAMES = {0: 'LEAD', 1: 'LEFT', 2: 'RIGHT', 3: 'CUTIN', 4: 'ROAD_SIGN', 5: 'HEADINGS'}
VEHICLE_GROUPS = (0, 1, 2, 3)
TWO_VEHICLE_GROUPS = (0, 1, 2)

# An unused slot saturates rather than zeroing: distance pinned to the top of its 8-bit range.
# Relabelling a saturated slot would invent a vehicle, so the distance is always checked first.
NO_OBJECT_RAW_DX = 254

# The passthrough flag keeps bus 0 <-> bus 2 forwarding alive. Without it the car goes deaf.
ALLOUTPUT_PARAM_PASSTHROUGH = 1


def relabel(data: bytes, frm: int, to: int, groups) -> tuple[bytes, int]:
  """Rewrite the vehicle type in one DAS_object frame, touching nothing else.

  Done on the raw bytes rather than through the packer on purpose: bits 6 and 37 are not claimed
  by any signal in the DBC, and a parse-and-repack would silently zero them.
  """
  d = bytearray(data)
  group = d[0] & 0x07
  if group not in groups:
    return bytes(d), 0

  changed = 0

  # first vehicle: type at bits 3-5, distance at bits 8-15
  if ((d[0] >> 3) & 0x07) == frm and d[1] < NO_OBJECT_RAW_DX:
    d[0] = (d[0] & ~0x38) | (to << 3)
    changed += 1

  # second vehicle: type at bits 34-36, distance at bits 39-46. Only the lead, left and right
  # groups carry one -- the cutin group reuses those bits for something else entirely.
  if group in TWO_VEHICLE_GROUPS:
    dx2 = ((d[4] >> 7) | (d[5] << 1)) & 0xFF
    if ((d[4] >> 2) & 0x07) == frm and dx2 < NO_OBJECT_RAW_DX:
      d[4] = (d[4] & ~0x1C) | (to << 2)
      changed += 1

  return bytes(d), changed


def run_mirror(panda, args):
  print(f"  mirroring DAS_object from bus {DAS_BUS} to bus {CLUSTER_BUS}, "
        f"{TYPE_NAMES[args.frm]} -> {TYPE_NAMES[args.to]}")
  print("  the factory keeps sending its own copy, so expect the display to flicker\n")

  seen = relabelled = 0
  last_report = time.monotonic()
  while True:
    for addr, dat, src in panda.can_recv():
      if addr != DAS_OBJECT or src != DAS_BUS:
        continue
      seen += 1
      out, changed = relabel(bytes(dat), args.frm, args.to, args.groups)
      if changed:
        relabelled += changed
      # Re-send whether or not it changed: a group we leave alone still has to arrive, or the
      # cluster sees our stream skip it. Repeat to outweigh the factory's own copy.
      for _ in range(args.repeat):
        panda.can_send(DAS_OBJECT, out, CLUSTER_BUS)

    now = time.monotonic()
    if now - last_report >= 2.0:
      print(f"    {seen:6d} frames seen, {relabelled:6d} vehicles relabelled")
      last_report = now
    time.sleep(0.002)


def run_sweep(panda, args):
  packer = CANPacker('tesla_can')
  print(f"  injecting one vehicle in the {GROUP_NAMES[args.group]} group at "
        f"{args.dx:.0f} m, {args.dy:+.1f} m across, and stepping the type\n")

  for veh_type in [int(t) for t in args.types.split(',')]:
    print(f"    type {veh_type}: {TYPE_NAMES.get(veh_type, '?')}  -- holding {args.hold:.0f}s")
    _, data, _ = packer.make_can_msg('DAS_object', CLUSTER_BUS, {
      'DAS_objectId': args.group,
      'DAS_objVehType': veh_type,
      'DAS_objVehRelevantForControl': 1 if args.group == 0 else 0,
      'DAS_objVehDx': args.dx,
      'DAS_objVehDy': args.dy,
      'DAS_objVehVxRel': 0.0,
      'DAS_objVehId': 42,
      # leave the second slot saturated so only one vehicle appears
      'DAS_objVeh2Dx': 127.5, 'DAS_objVeh2Dy': -22.05, 'DAS_objVeh2VxRel': 30.0,
      'DAS_objVeh2Id': 63, 'DAS_objVeh2Type': 0, 'DAS_objVeh2RelevantForControl': 0,
    })
    end = time.monotonic() + args.hold
    while time.monotonic() < end:
      panda.can_send(DAS_OBJECT, data, CLUSTER_BUS)
      time.sleep(1.0 / args.rate)
    time.sleep(1.5)   # a gap, so a display that latches is distinguishable from one that tracks


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('--sweep', action='store_true', help='inject synthetic objects instead of mirroring')
  p.add_argument('--from', dest='frm', type=int, default=CAR, help='type to replace (default CAR)')
  p.add_argument('--to', type=int, default=TRUCK, help='type to replace it with (default TRUCK)')
  p.add_argument('--lead-only', action='store_true',
                 help='relabel only the lead group, leaving adjacent lanes as they are')
  p.add_argument('--repeat', type=int, default=2, help='copies per received frame')
  p.add_argument('--group', type=int, default=2, help='sweep: 0 lead, 1 left, 2 right')
  p.add_argument('--dx', type=float, default=20.0, help='sweep: distance ahead, m')
  p.add_argument('--dy', type=float, default=3.5, help='sweep: lateral offset, m')
  p.add_argument('--hold', type=float, default=6.0, help='sweep: seconds per type')
  p.add_argument('--rate', type=float, default=50.0, help='sweep: Hz')
  p.add_argument('--types', type=str, default='0,1,2,3,4,5,6,7', help='sweep: types to try')
  args = p.parse_args()
  args.groups = (0,) if args.lead_only else VEHICLE_GROUPS

  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.allOutput, ALLOUTPUT_PARAM_PASSTHROUGH)
  print("\n  panda in allOutput with passthrough (forwarding still on)\n")
  try:
    (run_sweep if args.sweep else run_mirror)(panda, args)
  except KeyboardInterrupt:
    print("\n  stopped")
  finally:
    panda.set_safety_mode(CarParams.SafetyModel.noOutput)
    print("  panda returned to noOutput")


if __name__ == '__main__':
  main()
