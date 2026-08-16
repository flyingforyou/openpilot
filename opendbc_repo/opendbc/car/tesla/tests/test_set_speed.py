"""DAS_setSpeed is a control signal on this car, not a display one.

Zero is how the car is told to slow down -- the DI works to the target it is given, and the
accel command only bounds how hard it may do so. A commit that read it as "a plain kph display
signal, not read by panda safety" replaced the zero with the real cruise target and put a 30kph
floor under it, and deceleration stopped working altogether. These pin the ordering so the
display value can never again be allowed to override the decel command.
"""
import pytest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import DBC, CAR, CANBUS
from opendbc.can import CANPacker, CANDefine

SET_SPEED_SNA = 4095  # per the dbc: 'no value' is 4095, which is why 0 is free to mean zero


def _legacy():
  dbc = DBC[CAR.TESLA_MODEL_X_HW1]
  packers = {CANBUS.party: CANPacker(dbc['party']), CANBUS.powertrain: CANPacker(dbc['pt'])}
  return TeslaCANRaven(packers), CANDefine(dbc['pt'])


def _sent_set_speed(msg, define) -> float:
  """Decode DAS_setSpeed back out of the packed frame: 0|12@1+ (0.1, 0), kph."""
  d = bytes(msg[1])
  raw = ((d[1] & 0x0F) << 8) | d[0]
  return raw * 0.1


@pytest.mark.parametrize("hud_set_speed", [0.0, 5.0, 20.0, 33.0])
def test_decelerating_always_commands_zero(hud_set_speed):
  """Whatever the cluster would like to show, slowing down wins."""
  can, define = _legacy()
  msg = can.create_longitudinal_command(4, -1.5, 0, 25.0, True, False, hud_set_speed)
  assert _sent_set_speed(msg, define) == 0.0


def test_cruising_carries_the_real_target():
  can, define = _legacy()
  msg = can.create_longitudinal_command(4, 0.4, 0, 25.0, True, False, 22.0)
  assert _sent_set_speed(msg, define) == pytest.approx(22.0 * CV.MS_TO_KPH, abs=0.2)


def test_cruising_without_a_target_falls_back_to_max():
  can, define = _legacy()
  msg = can.create_longitudinal_command(4, 0.4, 0, 25.0, True, False, 0.0)
  assert _sent_set_speed(msg, define) > 100.0, "no target means do not cap the car"


def test_no_floor_under_the_target():
  """A 30kph floor is what made low-speed following and stopping impossible."""
  can, define = _legacy()
  msg = can.create_longitudinal_command(4, 0.1, 0, 5.0, True, False, 5.0)  # ~18 kph
  assert _sent_set_speed(msg, define) == pytest.approx(5.0 * CV.MS_TO_KPH, abs=0.2)


def test_inactive_reports_current_speed():
  can, define = _legacy()
  msg = can.create_longitudinal_command(4, 0.0, 0, 20.0, False, False, 33.0)
  assert _sent_set_speed(msg, define) == pytest.approx(20.0 * CV.MS_TO_KPH, abs=0.2)


def test_modern_port_orders_it_the_same_way():
  """Both ports share the contract; only legacy was exercised on a car."""
  packer = CANPacker(DBC[CAR.TESLA_MODEL_Y]['party'])
  can = TeslaCAN(None, packer)
  msg = can.create_longitudinal_command(4, -1.5, 0, 25.0, True, False, 30.0)
  d = bytes(msg[1])
  assert (((d[1] & 0x0F) << 8) | d[0]) * 0.1 == 0.0
