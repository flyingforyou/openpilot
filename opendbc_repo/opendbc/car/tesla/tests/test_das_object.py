"""The DAS_object layout, pinned against frames the car actually sent.

The bit positions come from a Model 3 DBC and this is an AP1 Model X, so they are not assumed:
every frame below was captured off bus 2 of the car, and the expected values were confirmed
against openpilot's own radar at the same instant -- decoded lead distance agreed with radarState
dRel to a median of 0.1 m over 81 samples, 99% within 5 m.

What these guard is a silent decode failure. Every field here shares a byte with another, so a
single wrong offset still produces plausible-looking distances; only fixed expectations catch it.
"""
from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.das_object import (CUTIN_VEHICLE, LEAD_VEHICLES, LEFT_VEHICLES,
                                          RIGHT_VEHICLES, VEHICLE_HEADINGS,
                                          parse_das_object)

DBC = 'tesla_can'
ADDR = 777


def decode(frame_hex: str):
  parser = CANParser(DBC, [('DAS_object', 0)], 0)
  parser.update([0, [(ADDR, bytes.fromhex(frame_hex), 0)]])
  return parser.vl['DAS_object']


class TestDasObject:
  def test_lead_vehicle(self):
    """A lead being followed: 35 m ahead, barely off centre, closing slowly, flagged for control."""
    vl = decode('9046083480ff0700')
    assert int(vl['DAS_objectId']) == LEAD_VEHICLES

    vehicles = parse_das_object(vl)
    assert len(vehicles) == 1
    lead = vehicles[0]
    assert lead.dx == 35.0
    # dy is an offset-and-scale away from its raw value, so it does not land exactly on 0.35
    assert abs(lead.dy - 0.35) < 1e-6
    assert lead.vx_rel == 2.0
    assert lead.obj_id == 6
    assert lead.obj_type == 2
    assert lead.relevant_for_control

  def test_right_lane_vehicle(self):
    """A car one lane right, 17 m up and closing hard -- and not flagged for control, which is
    the distinction that makes adjacent traffic separate from the lead."""
    vl = decode('1222d55c81ff0700')
    assert int(vl['DAS_objectId']) == RIGHT_VEHICLES

    vehicles = parse_das_object(vl)
    assert len(vehicles) == 1
    right = vehicles[0]
    assert right.dx == 17.0
    assert abs(right.dy - 4.90) < 1e-6
    assert right.vx_rel == -10.0
    assert right.obj_id == 43
    assert not right.relevant_for_control

  def test_empty_slot_reports_nothing(self):
    """An unused slot saturates rather than zeroing: 127.5 m, -22.05 m, +30 m/s, id 127. Read
    literally that is a real object receding fast at the edge of the lane, so it has to be
    filtered, not passed on."""
    for frame, group in (('00ff0ff883ff0700', LEAD_VEHICLES),
                         ('01ff0ff883ff0700', LEFT_VEHICLES),
                         ('02ff0ff883ff0700', RIGHT_VEHICLES)):
      vl = decode(frame)
      assert int(vl['DAS_objectId']) == group
      assert vl['DAS_objVehDx'] == 127.5
      assert parse_das_object(vl) == []

  def test_cutin_group_has_no_second_vehicle(self):
    """The cutin group reuses the second-vehicle bits for something else. This frame decodes them
    as an object 0 m away -- the car's own bumper -- if the group is not respected."""
    vl = decode('03ff0ff80300fc01')
    assert int(vl['DAS_objectId']) == CUTIN_VEHICLE
    assert vl['DAS_objVeh2Dx'] == 0.0        # would look like an object right on top of us
    assert parse_das_object(vl) == []

  def test_non_vehicle_groups_are_skipped(self):
    """Headings and road signs reuse the same bits for unrelated fields."""
    for frame in ('05ffffffffffffff', '057e7effffffffff'):
      vl = decode(frame)
      assert int(vl['DAS_objectId']) == VEHICLE_HEADINGS
      assert parse_das_object(vl) == []






  def test_pack_round_trip(self):
    """Sending this message is the point of decoding it, so the packer has to agree with the
    parser on every field -- including the quantisation, which is coarse enough to bite: relative
    speed lands on 4 m/s steps and distance on 0.5 m."""
    packer = CANPacker(DBC)
    values = {
        'DAS_objectId': LEFT_VEHICLES,
        'DAS_objVehType': 2,
        'DAS_objVehRelevantForControl': 0,
        'DAS_objVehDx': 42.5,
        'DAS_objVehDy': -3.5,
        'DAS_objVehVxRel': -6.0,
        'DAS_objVehId': 17,
        'DAS_objVeh2Dx': 61.0,
        'DAS_objVeh2Dy': -3.85,
        'DAS_objVeh2VxRel': 2.0,
        'DAS_objVeh2Id': 5,
        'DAS_objVeh2Type': 2,
        'DAS_objVeh2RelevantForControl': 0,
    }
    addr, data, _ = packer.make_can_msg('DAS_object', 0, values)
    assert addr == ADDR

    vehicles = parse_das_object(decode(data.hex()))
    assert len(vehicles) == 2
    near, far = vehicles
    assert near.group == LEFT_VEHICLES and near.index == 0
    assert near.dx == 42.5
    assert abs(near.dy - (-3.5)) < 0.35
    assert near.vx_rel == -6.0
    assert near.obj_id == 17
    assert far.index == 1
    assert far.dx == 61.0
    assert abs(far.dy - (-3.85)) < 0.35
