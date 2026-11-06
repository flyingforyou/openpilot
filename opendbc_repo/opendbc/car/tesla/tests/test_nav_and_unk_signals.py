"""Map signals added to navMap and the self-fitted body/chassis IDs in tesla_can.dbc.

Every frame here is a real one from route 0000024b (2026-11-05). The UNK_ messages are not in any
public Tesla DBC; their layouts were fitted against carState over whole drives (0x128 speed p99
0.8 km/h, 0x21E steering r=0.9998, the bits 98-99.6% agreement), and these tests pin the layout so
a typo in the DBC cannot pass for a decode.
"""
import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CANBUS, CAR

MAP_EXIT = (0x3C8, '082e4837ae00d976')   # controlled access, right off-ramp in 140 m
MAP_SNA = (0x3C8, '302648f31f003620')    # no branch ahead (raw 31)
CSA = (0x2E8, '93f5000032f5015a')        # C2 -0.002669 1/m, range 100 m
SPEED = (0x128, '00001d0e')              # carState vEgo 54.14 km/h
STEER = (0x21E, '0100000000ce68c5')      # carState steering -298.8 deg, held
LAMP_L = (0x30A, '00d9916300540024')
LAMP_R = (0x30A, '00d9996a00141225')
DOOR_OPEN = (0x30A, '00d9965c00140028')
BRAKE = (0x248, '06008b97')              # carState brakePressed


def feed(parser, t, frames):
  parser.update([t, [(addr, bytes.fromhex(dat), CANBUS.party) for addr, dat in frames]])


def nav_after(frames):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_X_HW1, gen_empty_fingerprint(), [], False, False, False)
  cs = CarState(CP)
  parsers = CarState.get_can_parsers(CP)
  # The map messages register lazily on first read, so read once before the frames arrive (on the
  # car the next frame lands a moment later). t=0 would read as "never arrived" to the decoder.
  cs.nav_map.update(structs.CarState.new_message(), parsers[Bus.party], parsers[Bus.ap_party])
  feed(parsers[Bus.party], 10_000_000, frames)
  ret = structs.CarState.new_message()
  cs.nav_map.update(ret, parsers[Bus.party], parsers[Bus.ap_party])
  return ret.navMap


class TestNavMapAdditions:
  def test_exit_countdown(self):
    nav = nav_after([MAP_EXIT, CSA])
    assert nav.controlledAccess and nav.nextBranchRightOffRamp
    assert nav.nextBranchDist == 140

  def test_no_branch_is_minus_one_not_310(self):
    assert nav_after([MAP_SNA]).nextBranchDist == -1

  def test_csa_curvature(self):
    nav = nav_after([MAP_EXIT, CSA])
    assert nav.csaCurvC2 == pytest.approx(-0.002669)
    assert nav.csaCurvC3 == 0
    assert nav.csaCurvRange == 100

  def test_csa_absent_reads_zero(self):
    nav = nav_after([MAP_EXIT])
    assert nav.csaCurvC2 == 0 and nav.csaCurvRange == 0

  def test_csa_never_arriving_keeps_the_bus_valid(self):
    """CSA is registered with ignore_alive: twenty seconds without it must not cost a CAN error."""
    CP = CarInterface.get_params(CAR.TESLA_MODEL_X_HW1, gen_empty_fingerprint(), [], False, False, False)
    cs = CarState(CP)
    parsers = CarState.get_can_parsers(CP)
    cp = parsers[Bus.party]
    for i in range(1, 2000):
      feed(cp, i * 10_000_000, [MAP_EXIT])
      cs.nav_map.update(structs.CarState.new_message(), cp, parsers[Bus.ap_party])
    # Only CSA's own state: the other map messages are not fed here, and on the car they arrive.
    assert cp.message_states[cp.dbc.name_to_msg['UI_csaRoadCurvature'].address].valid(cp._last_update_nanos, cp.bus_timeout)


class TestUnknownIdFits:
  def parse(self, frames):
    cp = CANParser('tesla_can', [('UNK_speed128', float('nan')), ('UNK_steer21E', float('nan')),
                                 ('UNK_body30A', float('nan')), ('UNK_brake248', float('nan'))], CANBUS.party)
    feed(cp, 0, frames)
    return cp.vl

  def test_speed(self):
    assert self.parse([SPEED])['UNK_speed128']['UNK128_vehicleSpeed'] == pytest.approx(54.14, abs=0.1)

  def test_steering(self):
    assert self.parse([STEER])['UNK_steer21E']['UNK21E_steeringAngle'] == pytest.approx(-298.8, abs=3)

  def test_lamps_and_doors(self):
    assert self.parse([LAMP_L])['UNK_body30A']['UNK30A_leftTurnLamp'] == 1
    assert self.parse([LAMP_L])['UNK_body30A']['UNK30A_rightTurnLamp'] == 0
    assert self.parse([LAMP_R])['UNK_body30A']['UNK30A_rightTurnLamp'] == 1
    assert self.parse([LAMP_R])['UNK_body30A']['UNK30A_doorsClosed'] == 1
    assert self.parse([DOOR_OPEN])['UNK_body30A']['UNK30A_doorsClosed'] == 0

  def test_brake_pedal(self):
    assert self.parse([BRAKE])['UNK_brake248']['UNK248_brakePedal'] == 1
