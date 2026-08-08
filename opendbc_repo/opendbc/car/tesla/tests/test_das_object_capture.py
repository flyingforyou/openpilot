"""Reading the factory's object list back out of a parser.

The first attempt at this shipped and did nothing at all: the Tesla parsers are built with no
message list and pick messages up on demand, but only through `vl`, which is the one wired for
lazy registration. `vl_all` is an ordinary dict that registration fills in, so asking it for a
message nobody has touched returns nothing -- forever, and without an error to notice. The object
list stayed empty and no frames were ever re-sent.

These pin the access pattern the car port depends on, since nothing else would catch it failing
quietly: registration has to happen, every group in a rotation has to survive one update, and the
values that come back out have to be good enough to pack straight into a frame.
"""
from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.das_object import CAR, LEAD_VEHICLES, RIGHT_VEHICLES, TRUCK, substitute_type

DBC = 'tesla_can'
ADDR = 777
BUS = 2

# Real frames, one rotation as the factory sends it: lead, left, right, cutin, headings.
ROTATION = [
  '9046083480ff0700',    # LEAD  car at 35 m
  '01ff0ff883ff0700',    # LEFT  empty
  '1222d55c81ff0700',    # RIGHT car at 17 m
  '03ff0ff80300fc01',    # CUTIN empty
  '05ffffffffffffff',    # HEADINGS
]


def feed(parser, frames):
  parser.update([0, [(ADDR, bytes.fromhex(f), BUS) for f in frames]])


class TestDasObjectCapture:
  def test_vl_all_is_empty_until_the_message_is_registered(self):
    """The exact failure that shipped. vl_all never registers anything by itself."""
    parser = CANParser(DBC, [], BUS)
    feed(parser, ROTATION)
    assert parser.vl_all.get('DAS_object') is None

  def test_touching_vl_registers_it(self):
    parser = CANParser(DBC, [], BUS)
    parser.vl['DAS_object']
    feed(parser, ROTATION)
    assert parser.vl_all.get('DAS_object') is not None

  def test_a_whole_rotation_survives_one_update(self):
    """Groups arrive one per frame, so reading only the latest would see one group in five. Every
    group sent between updates has to come back."""
    parser = CANParser(DBC, [], BUS)
    parser.vl['DAS_object']
    feed(parser, ROTATION)

    frames = parser.vl_all['DAS_object']
    names = list(frames)
    columns = [frames[n] for n in names]
    assert len({len(c) for c in columns}) == 1, "columns must be one entry per frame"

    groups = [int(dict(zip(names, row))['DAS_objectId']) for row in zip(*columns)]
    assert groups == [0, 1, 2, 3, 5]

  def test_captured_values_pack_back_into_the_same_frame(self):
    """What comes out of the parser is fed straight to the packer, so the round trip has to be
    exact -- otherwise the cluster would be told the car is somewhere it is not."""
    parser = CANParser(DBC, [], BUS)
    parser.vl['DAS_object']
    feed(parser, ['9046083480ff0700'])

    frames = parser.vl_all['DAS_object']
    names = list(frames)
    values = dict(zip(names, next(zip(*[frames[n] for n in names]))))

    packer = CANPacker(DBC)
    _, data, _ = packer.make_can_msg('DAS_object', 0, values)
    assert data.hex() == '9046083480ff0700'

  def test_substitution_on_captured_values(self):
    """End to end: capture the lead frame, relabel it, and confirm only the type moved."""
    parser = CANParser(DBC, [], BUS)
    parser.vl['DAS_object']
    feed(parser, ROTATION)

    frames = parser.vl_all['DAS_object']
    names = list(frames)
    captured = {}
    for row in zip(*[frames[n] for n in names]):
      values = dict(zip(names, row))
      captured[int(values['DAS_objectId'])] = values

    lead = substitute_type(captured[LEAD_VEHICLES], CAR, TRUCK)
    assert lead is not None and lead['DAS_objVehType'] == TRUCK
    assert lead['DAS_objVehDx'] == 35.0

    right = substitute_type(captured[RIGHT_VEHICLES], CAR, TRUCK)
    assert right is not None and right['DAS_objVehDx'] == 17.0

    # the empty groups have nothing to relabel and must not be re-sent
    assert substitute_type(captured[1], CAR, TRUCK) is None
    assert substitute_type(captured[3], CAR, TRUCK) is None
