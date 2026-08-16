"""Reading DAS_object (0x309), the factory's own list of vehicles around the car.

This is what the instrument cluster draws. Everything else this port looks at -- DAS_ACC_report
in DAS_status2 -- is a single-target ACC summary and says nothing about adjacent lanes, so a
cluster that shows a truck alongside but nothing ahead is invisible to it.

The message is multiplexed on DAS_objectId, and opendbc's DBC reader drops the multiplex token
(see SGM_RE in opendbc/can/dbc.py: it matches the `m0` form but keeps only the name and bit
position). Signals therefore decode from every frame regardless of which group it carries. That
is workable here only because the four vehicle groups -- lead, left, right, cutin -- share one
bit layout: the DBC declares that layout once, unmultiplexed, and the caller switches on
DAS_objectId to learn which group the values belong to. Doing it any other way would need parser
changes; doing it this way needs the caller to respect two rules, both enforced below:

  - ROAD_SIGN and VEHICLE_HEADINGS reuse the same bits for entirely different fields, so the
    vehicle signals are meaningless there and must not be read.
  - CUTIN_VEHICLE carries only one vehicle; the second-vehicle bits are not a vehicle.
"""
from collections import namedtuple

# DAS_objectId
LEAD_VEHICLES = 0
LEFT_VEHICLES = 1
RIGHT_VEHICLES = 2
CUTIN_VEHICLE = 3
ROAD_SIGN = 4
VEHICLE_HEADINGS = 5

VEHICLE_GROUPS = (LEAD_VEHICLES, LEFT_VEHICLES, RIGHT_VEHICLES, CUTIN_VEHICLE)
# Only the first three carry a second vehicle; the cutin group's upper bits are something else.
TWO_VEHICLE_GROUPS = (LEAD_VEHICLES, LEFT_VEHICLES, RIGHT_VEHICLES)

# An unused slot is not zeroed, it is saturated: distance pinned to the top of its 8-bit range.
# Testing distance is enough -- the other fields saturate with it.
NO_OBJECT_DX = 127.0

DasVehicle = namedtuple('DasVehicle', ['group', 'index', 'obj_type', 'relevant_for_control',
                                       'dx', 'dy', 'vx_rel', 'obj_id'])

# DAS_objVehType
UNKNOWN, TRUCK, CAR, MOTORCYCLE, BICYCLE, PEDESTRIAN, IPSO = range(7)


def substitute_type(vl, frm: int = CAR, to: int = TRUCK) -> dict | None:
  """A copy of one DAS_object frame's values with the vehicle type swapped, or None to leave it be.

  Since Tesla's 2026.26.1 update, AP1 clusters draw TRUCK and MOTORCYCLE but not CAR -- which is
  78% of what the car sees, so almost nothing appears. Everything upstream is correct, so the
  smallest honest repair is to hand the cluster the same objects under a type it still renders.

  Only the type moves. Distance, lateral offset, closing speed and object id are the factory's and
  are copied through, because the factory has them right.

  Returns None when there is nothing to change, so a caller can skip transmitting rather than add
  a redundant frame to the bus.

  Rebuilding the frame from decoded values is lossless for these groups: bits 6 and 37 belong to
  no signal, and across 89k frames bit 6 was never set and bit 37 only ever in VEHICLE_HEADINGS,
  which is not a vehicle group and is never rebuilt here.
  """
  group = int(vl['DAS_objectId'])
  if group not in VEHICLE_GROUPS:
    return None

  values = dict(vl)
  changed = False

  # An empty slot is saturated, not zeroed. Relabelling one would put a vehicle on the display
  # that the car never reported.
  if int(vl['DAS_objVehType']) == frm and vl['DAS_objVehDx'] < NO_OBJECT_DX:
    values['DAS_objVehType'] = to
    changed = True

  if group in TWO_VEHICLE_GROUPS:
    if int(vl['DAS_objVeh2Type']) == frm and vl['DAS_objVeh2Dx'] < NO_OBJECT_DX:
      values['DAS_objVeh2Type'] = to
      changed = True

  return values if changed else None


def parse_das_object(vl) -> list[DasVehicle]:
  """One DAS_object frame, already decoded by CANParser, -> the vehicles it reports.

  Returns an empty list for the non-vehicle groups and for slots reporting nothing, so a caller
  can concatenate frames without having to know which group each one was.
  """
  group = int(vl['DAS_objectId'])
  if group not in VEHICLE_GROUPS:
    return []

  out = []
  if vl['DAS_objVehDx'] < NO_OBJECT_DX:
    out.append(DasVehicle(
        group=group,
        index=0,
        obj_type=int(vl['DAS_objVehType']),
        relevant_for_control=bool(vl['DAS_objVehRelevantForControl']),
        dx=float(vl['DAS_objVehDx']),
        dy=float(vl['DAS_objVehDy']),
        vx_rel=float(vl['DAS_objVehVxRel']),
        obj_id=int(vl['DAS_objVehId']),
    ))

  if group in TWO_VEHICLE_GROUPS and vl['DAS_objVeh2Dx'] < NO_OBJECT_DX:
    out.append(DasVehicle(
        group=group,
        index=1,
        obj_type=int(vl['DAS_objVeh2Type']),
        relevant_for_control=bool(vl['DAS_objVeh2RelevantForControl']),
        dx=float(vl['DAS_objVeh2Dx']),
        dy=float(vl['DAS_objVeh2Dy']),
        vx_rel=float(vl['DAS_objVeh2VxRel']),
        obj_id=int(vl['DAS_objVeh2Id']),
    ))

  return out
