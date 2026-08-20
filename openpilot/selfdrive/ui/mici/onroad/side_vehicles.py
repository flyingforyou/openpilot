"""What is in the lane either side, ready to draw.

Only the choosing lives here; the drawing is in HudRenderer. Splitting them is what makes this
testable at all -- importing the renderer pulls in the whole UI stack -- but it earns its keep
anyway, because every mistake worth catching is in the choosing: reading the cut-in group as a
third lane, showing the far vehicle instead of the near one, or dropping the vehicle type.

The type is the reason any of this comes off the factory camera rather than the radar. This car's
radar reports 90% of its tracks as unknown, and of the rest called a clearly-a-car track fourWheel
0% of the time and a motorcycle 32%. The camera types them properly, so a motorcycle can be drawn
narrow and a lorry wide and the difference read without a legend.
"""
from typing import NamedTuple

# DAS_objectId
GROUP_LEFT, GROUP_RIGHT, GROUP_CUTIN = 1, 2, 3

# Half-widths by DAS_objVehType, metres. Same table the cut-in detector measures encroachment
# with, for the same reason: what matters about a vehicle's type is how wide it is.
HALF_WIDTH = {0: 0.90, 1: 1.25, 2: 0.90, 3: 0.40, 4: 0.30, 5: 0.30}
DEFAULT_HALF_WIDTH = HALF_WIDTH[0]

# Beyond this a marker says nothing useful at the scale this is drawn at.
MAX_DREL = 60.0

# The strip has room for two a side. A third is far enough back to be of no interest, and
# crowding the near one is the opposite of the point.
MAX_PER_SIDE = 2


class SideVehicle(NamedTuple):
  group: int          # GROUP_LEFT or GROUP_RIGHT
  d_rel: float        # metres ahead
  half_width: float   # metres, from the camera's type
  is_cutin: bool      # the camera has called this one a cut-in


def side_vehicles(das_objects) -> list[SideVehicle]:
  """Adjacent-lane vehicles from carState.dasObjects, nearest first, at most MAX_PER_SIDE a side.

  The cut-in group is not a place -- it is the same vehicle the left or right group already
  reports, named again. So it marks one rather than adding one.
  """
  if not das_objects:
    return []

  cutin_ids = {int(o.objId) for o in das_objects if int(o.group) == GROUP_CUTIN}

  found: list[SideVehicle] = []
  for obj in das_objects:
    group = int(obj.group)
    if group not in (GROUP_LEFT, GROUP_RIGHT):
      continue
    d_rel = float(obj.dx)
    if not 0.0 < d_rel < MAX_DREL:
      continue
    found.append(SideVehicle(group, d_rel,
                             HALF_WIDTH.get(int(obj.objType), DEFAULT_HALF_WIDTH),
                             int(obj.objId) in cutin_ids))

  found.sort(key=lambda v: v.d_rel)
  kept: list[SideVehicle] = []
  seen = {GROUP_LEFT: 0, GROUP_RIGHT: 0}
  for veh in found:
    if seen[veh.group] >= MAX_PER_SIDE:
      continue
    seen[veh.group] += 1
    kept.append(veh)
  return kept
