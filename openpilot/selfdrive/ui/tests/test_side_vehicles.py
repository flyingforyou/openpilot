"""What the side-lane markers are built from, without a screen.

The drawing itself needs a GL context, but the decision of what to draw does not, and that is
where the mistakes would be: picking up the cut-in group as though it were a lane, showing the
far one instead of the near one, or losing the vehicle type that is the whole reason the factory
camera is being read at all.
"""
import pytest

from openpilot.selfdrive.ui.mici.onroad.side_vehicles import (
  DEFAULT_HALF_WIDTH,
  GROUP_CUTIN,
  GROUP_LEFT,
  GROUP_RIGHT,
  HALF_WIDTH,
  MAX_DREL,
  MAX_PER_SIDE,
  side_vehicles,
)

MOTORCYCLE, CAR, TRUCK = 3, 2, 1


class Obj:
  def __init__(self, group, dx, obj_type=CAR, obj_id=1):
    self.group, self.dx, self.objType, self.objId = group, dx, obj_type, obj_id


collect = side_vehicles


class TestSelection:
  def test_both_lanes_are_picked_up(self):
    got = collect([Obj(GROUP_LEFT, 20.0), Obj(GROUP_RIGHT, 30.0)])
    assert [v.group for v in got] == [GROUP_LEFT, GROUP_RIGHT]

  def test_the_lead_and_heading_groups_are_not_lanes(self):
    assert collect([Obj(0, 20.0), Obj(5, 20.0)]) == []

  def test_the_cutin_group_is_not_drawn_as_a_lane_of_its_own(self):
    """It is the same vehicle the left or right group already reports, not a third place."""
    got = collect([Obj(GROUP_CUTIN, 20.0, obj_id=7)])
    assert got == []

  def test_but_it_marks_the_vehicle_that_is_in_a_lane(self):
    got = collect([Obj(GROUP_RIGHT, 20.0, obj_id=7), Obj(GROUP_CUTIN, 20.0, obj_id=7)])
    assert len(got) == 1 and got[0].is_cutin is True

  def test_a_different_object_is_not_marked(self):
    got = collect([Obj(GROUP_RIGHT, 20.0, obj_id=7), Obj(GROUP_CUTIN, 40.0, obj_id=9)])
    assert got[0].is_cutin is False

  def test_nearest_first(self):
    got = collect([Obj(GROUP_LEFT, 40.0), Obj(GROUP_LEFT, 10.0), Obj(GROUP_LEFT, 25.0)])
    assert [round(v.d_rel) for v in got] == [10, 25]   # MAX_PER_SIDE keeps the near two

  @pytest.mark.parametrize("dx", [0.0, -5.0, MAX_DREL + 1])
  def test_out_of_range_is_dropped(self, dx):
    assert collect([Obj(GROUP_LEFT, dx)]) == []


  def test_only_the_near_two_a_side_survive(self):
    """A third is far enough back to be of no interest, and crowding the near one is the
    opposite of the point."""
    objs = [Obj(GROUP_LEFT, d) for d in (10.0, 20.0, 30.0, 40.0)]
    objs += [Obj(GROUP_RIGHT, d) for d in (15.0, 25.0, 35.0)]
    got = collect(objs)
    assert sum(v.group == GROUP_LEFT for v in got) == MAX_PER_SIDE
    assert sum(v.group == GROUP_RIGHT for v in got) == MAX_PER_SIDE
    assert [round(v.d_rel) for v in got] == [10, 15, 20, 25]


class TestWidth:
  """The marker's width is the vehicle's own, which is the only reason type is worth reading."""

  def test_a_motorcycle_is_narrower_than_a_car_is_narrower_than_a_lorry(self):
    widths = [collect([Obj(GROUP_LEFT, 20.0, k)])[0].half_width for k in (MOTORCYCLE, CAR, TRUCK)]
    assert widths[0] < widths[1] < widths[2]

  def test_an_untyped_object_is_drawn_as_a_car(self):
    assert collect([Obj(GROUP_LEFT, 20.0, 0)])[0].half_width == HALF_WIDTH[CAR]

  def test_a_type_the_camera_has_never_sent_does_not_crash_it(self):
    assert collect([Obj(GROUP_LEFT, 20.0, 99)])[0].half_width == DEFAULT_HALF_WIDTH
