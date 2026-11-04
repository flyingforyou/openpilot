"""Wheel speeds, brake torque and ultrasonics off the legacy chassis bus.

These are decoded with the Model 3 layouts, which match this car byte for byte (every frame of
route 0244 passes the Tesla checksum). Nothing in control reads them, so the one thing that must
never happen is for one of them going quiet to cost a CAN error -- that would take an engagement
away over a readout. The frames below are real ones from that route.
"""
from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.carstate import CHASSIS_SENSOR_MSGS, CHASSIS_SENSOR_STALE_FRAMES, CarState
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CANBUS, CAR

WHEEL = (0x175, '59e6cb50992523b1')       # 65.0 / 65.2 / 64.8 / 64.4 km/h
BRAKE = (0x185, '344003211002dc0c')       # 104 / 104 / 66 / 66 Nm
USS_FRONT = (0x20E, 'f4e9d3a76f2e191d')   # 500 500 500 500 230 201
USS_REAR = (0x22E, 'd8ffffffff9fbe61')    # 472 511 511 511 511 500
OTHER = (0x155, 'e27397e94616a77b')       # ESP_B, to keep the bus itself alive


def feed(parser, t, frames):
  parser.update([t, [(addr, bytes.fromhex(dat), CANBUS.party) for addr, dat in frames]])


def legacy():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_X_HW1, gen_empty_fingerprint(), [], False, False, False)
  return CarState(CP), CarState.get_can_parsers(CP)[Bus.chassis]


class TestChassisSensors:
  def test_messages_that_never_arrive_do_not_invalidate_the_bus(self):
    """Twenty seconds of a bus that carries none of them: all four must still count as valid."""
    _, cp = legacy()
    for i in range(2000):
      feed(cp, i * 10_000_000, [OTHER])
    for name in CHASSIS_SENSOR_MSGS:
      assert cp.message_states[cp.dbc.name_to_msg[name].address].valid(cp._last_update_nanos, cp.bus_timeout)

  def test_real_frames_decode(self):
    cp = CANParser('tesla_can', [(n, float('nan')) for n in CHASSIS_SENSOR_MSGS], CANBUS.party)
    feed(cp, 0, [WHEEL, BRAKE, USS_FRONT, USS_REAR])
    assert [round(cp.vl['ESP_wheelSpeeds'][f'ESP_wheelSpeed{c}'], 1) for c in ('FrL', 'FrR', 'ReL', 'ReR')] == \
      [65.0, 65.2, 64.8, 64.4]
    assert [cp.vl['ESP_brakeTorque'][f'ESP_brakeTorque{c}'] for c in ('FrL', 'FrR', 'ReL', 'ReR')] == [104, 104, 66, 66]
    assert [cp.vl['PARK_sdiFront'][f'PARK_sdiSensor{i}RawDistData'] for i in range(1, 7)] == [500] * 4 + [230, 201]
    assert [cp.vl['PARK_sdiRear'][f'PARK_sdiSensor{i}RawDistData'] for i in range(7, 13)] == [472] + [511] * 4 + [500]

  def test_published_while_arriving(self):
    cs, cp = legacy()
    feed(cp, 0, [WHEEL, BRAKE, USS_FRONT, USS_REAR])
    ret = structs.CarState.new_message()
    cs.update_chassis_sensors(ret, cp)
    assert abs(ret.wheelSpeeds.fl - 65.0 / 3.6) < 0.01
    assert ret.chassisSensors.brakeTorqueValid and ret.chassisSensors.brakeTorqueRr == 66
    assert ret.chassisSensors.ussValid and list(ret.chassisSensors.ussFront)[4:] == [230, 201]

  def test_a_stopped_message_goes_invalid_instead_of_freezing(self):
    """The parser keeps a message's last value forever, so a frozen reading would otherwise look
    exactly like a live one."""
    cs, cp = legacy()
    feed(cp, 0, [WHEEL, BRAKE, USS_FRONT, USS_REAR])
    cs.update_chassis_sensors(structs.CarState.new_message(), cp)
    for i in range(1, CHASSIS_SENSOR_STALE_FRAMES + 1):
      feed(cp, i * 10_000_000, [OTHER])
      ret = structs.CarState.new_message()
      cs.update_chassis_sensors(ret, cp)
    assert not ret.chassisSensors.brakeTorqueValid
    assert not ret.chassisSensors.ussValid
    assert ret.wheelSpeeds.fl == 0
