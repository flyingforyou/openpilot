#!/usr/bin/env python3
import random
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams, TeslaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_SPEED_DELTA, MAX_WRONG_COUNTERS, away_round, round_speed

MSG_DAS_steeringControl = 0x488
MSG_DAS_Control_HW1 = 0x2b9
MSG_DI_torque1 = 0x108  # HW1 uses different message ID


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TestTeslaHW1Safety(common.CarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  # HW1 configuration - based on tesla_legacy.h
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_DAS_Control_HW1)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_DAS_Control_HW1]}
  TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_DAS_Control_HW1, 0]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3
  LONGITUDINAL = True

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_epas = 0
  cnt_angle_cmd = 0

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    from opendbc.car.tesla.interface import CarInterface
    self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

    # HW1 uses tesla_can DBC for most messages
    self.packer = CANPackerSafety("tesla_can")

    self.define = CANDefine("tesla_can")
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.steer_control_types = {d: v for v, d in
                                self.define.dv["DAS_steeringControl"]["DAS_steeringControlType"].items()}

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1))
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    # HW1 uses EPAS_sysStatus message with EPAS_* signals (not EPAS3S_*)
    values = {
      "EPAS_internalSAS": angle,
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code
    }
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values)

  def _user_brake_msg(self, brake):
    # HW1 uses BrakeMessage with driverBrakeStatus signal
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _speed_msg(self, speed):
    # HW1 uses ESP_B message with ESP_vehicleSpeed signal
    values = {"ESP_vehicleSpeed": speed * 3.6}  # Convert m/s to km/h
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _user_gas_msg(self, gas):
    # HW1 uses DI_torque1 message (0x108) with DI_pedalPos signal
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    # HW1 uses DAS_control message (0x2b9)
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # Legacy models don't have checksums for most messages
    # Test basic message reception
    for msg_type in ("angle", "long", "speed"):
      for i in range(5):
        if msg_type == "angle":
          msg = self._angle_cmd_msg(0, True, bus=2)
        elif msg_type == "long":
          msg = self._long_control_msg(0, bus=2)
        elif msg_type == "speed":
          msg = self._speed_msg(0)
        else:
          continue

        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(msg))
        self.assertTrue(self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    # HW1 uses ESP_B message for speed
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_steering_wheel_disengage(self):
    # Tesla disengages when the user forcibly overrides the locked-in angle steering control
    # Either when the hands on level is high, or if there is a high angle rate fault
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level, eac_status=eac_status,
                                                        eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_steering_control_type(self):
    # Test valid steering control types
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(4):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    # TODO: make these generic passthrough tests
    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=True, bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_no_aeb(self):
    # Test that AEB events are blocked
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    # HW1 AEB passthrough logic
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> block forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> allow forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def _stock_steering_msg(self):
    """The stock module commanding angle on the camera bus, as it does through autopark."""
    return self._angle_cmd_msg(0, state=self.steer_control_types['ANGLE_CONTROL'], bus=2)

  def test_stock_autopark_blocked_unless_opted_in(self):
    # Default HW1 config has no autopark flag, so the stock module stays gated exactly as before
    self._rx(self._stock_steering_msg())
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_SELFPARK_START'], bus=2))

    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control_HW1))

  def test_prevent_reverse(self):
    # Test reverse prevention logic - use the same test as modern Tesla
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.01 * 3.6) * 0.01 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = -1 if sign == -1 else 0

        # at limit (safety tolerance adds 1)
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 2) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.01 * 3.6) * 0.01 / 3.6)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = 1 if sign == -1 else 0

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestTeslaHW1StockAutoparkSafety(TestTeslaHW1Safety):
  """HW1 with the stock autopark opt-in. Inherits the whole suite so enabling it has to leave
  every other safety property intact."""

  def setUp(self):
    super().setUp()
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy,
                                 int(TeslaSafetyFlags.FLAG_HW1 | TeslaSafetyFlags.STOCK_AUTOPARK))
    self.safety.init_tests()

  def _assert_ap1_forwarded(self, forwarded: bool):
    expected = 0 if forwarded else -1
    self.assertEqual(expected, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))
    self.assertEqual(expected, self.safety.safety_fwd_hook(2, MSG_DAS_Control_HW1))

  def test_stock_autopark_blocked_unless_opted_in(self):
    # Overrides the base case: this class is the opted-in one, so the same traffic opens the gate
    self._rx(self._stock_steering_msg())
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_SELFPARK_START'], bus=2))
    self._assert_ap1_forwarded(True)

  def test_stock_lkas_passthrough(self):
    # Same intent as the base test, but it uses state=True -- which is ANGLE_CONTROL, the very
    # command autopark steers with -- to stand for "idle". Once opted in that is no longer idle,
    # so the quiet case has to be a genuine NONE.
    # Stays disengaged throughout: the stock LKAS latch only arms on a rising edge while
    # controls are not allowed.
    op_msg = self._angle_cmd_msg(0, state=self.steer_control_types['NONE'])
    idle_cam = self._angle_cmd_msg(0, state=self.steer_control_types['NONE'], bus=2)
    lkas_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    self.assertEqual(1, self._rx(idle_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, idle_cam.addr))
    self.assertTrue(self._tx(op_msg))

    self.assertEqual(1, self._rx(lkas_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_cam.addr))
    self.assertFalse(self._tx(op_msg))

  def test_idle_stock_system_is_still_gated(self):
    self._rx(self._angle_cmd_msg(0, state=self.steer_control_types['NONE'], bus=2))
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['ACC_CANCEL_GENERIC'], bus=2))
    self._assert_ap1_forwarded(False)

  def test_steering_before_apc_state_opens_the_gate(self):
    # Why the APC state alone is not enough: on a recorded attempt the stock module steered for
    # 2.0s while DAS_accState still read a non-APC value. Gating on APC would drop all of it.
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['ACC_CANCEL_GENERIC'], bus=2))
    self._rx(self._stock_steering_msg())
    self._assert_ap1_forwarded(True)

  def test_apc_states_open_the_gate(self):
    for state in ('APC_BACKWARD', 'APC_FORWARD', 'APC_COMPLETE', 'APC_ABORT', 'APC_PAUSE',
                  'APC_UNPARK_COMPLETE', 'APC_SELFPARK_START'):
      with self.subTest(state=state):
        self._rx(self._angle_cmd_msg(0, state=self.steer_control_types['NONE'], bus=2))
        self._rx(self._long_control_msg(0, acc_state=self.acc_states[state], bus=2))
        self._assert_ap1_forwarded(True)

  def test_non_apc_acc_states_keep_the_gate_shut(self):
    for state in ('ACC_CANCEL_GENERIC', 'ACC_HOLD', 'ACC_ON', 'ACC_CANCEL_GENERIC_SILENT', 'FAULT_SNA'):
      with self.subTest(state=state):
        self._rx(self._angle_cmd_msg(0, state=self.steer_control_types['NONE'], bus=2))
        self._rx(self._long_control_msg(0, acc_state=self.acc_states[state], bus=2))
        self._assert_ap1_forwarded(False)

  def _quiet_stock_msg(self):
    """A frame from the stock module that is not asking for the bus."""
    return self._long_control_msg(0, acc_state=self.acc_states['ACC_CANCEL_GENERIC'], bus=2)

  def test_gate_stays_open_across_gaps_in_the_stock_stream(self):
    """The whole point of the latch. A per-frame gate passed 2 of 6 recorded APC_BACKWARD frames
    and let openpilot's own DAS_control fill the gaps, which aborted the maneuver."""
    self.safety.set_timer(0)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_BACKWARD'], bus=2))
    self._assert_ap1_forwarded(True)

    # the module keeps sending on the id, but not every frame carries an APC state
    for us in (20_000, 100_000, 500_000, 900_000):
      self.safety.set_timer(us)
      self._rx(self._quiet_stock_msg())
      self._assert_ap1_forwarded(True)

  def test_gate_closes_once_the_stock_module_goes_quiet(self):
    self.safety.set_timer(0)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_BACKWARD'], bus=2))
    self._assert_ap1_forwarded(True)

    self.safety.set_timer(1_000_001)
    self._rx(self._quiet_stock_msg())
    self._assert_ap1_forwarded(False)

  def test_a_refresh_extends_the_session(self):
    self.safety.set_timer(0)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_FORWARD'], bus=2))
    self.safety.set_timer(900_000)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_FORWARD'], bus=2))

    # without the refresh this would already be past the timeout
    self.safety.set_timer(1_500_000)
    self._rx(self._quiet_stock_msg())
    self._assert_ap1_forwarded(True)

  def test_engaging_closes_the_session_even_mid_maneuver(self):
    self.safety.set_timer(0)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_BACKWARD'], bus=2))
    self._assert_ap1_forwarded(True)

    self.safety.set_controls_allowed(True)
    self._rx(self._long_control_msg(0, acc_state=self.acc_states['APC_BACKWARD'], bus=2))
    self._assert_ap1_forwarded(False)

    # and it does not silently resume when the driver disengages again
    self.safety.set_controls_allowed(False)
    self._rx(self._quiet_stock_msg())
    self._assert_ap1_forwarded(False)

  def test_openpilot_engaged_takes_the_bus_back(self):
    self._rx(self._stock_steering_msg())
    self._assert_ap1_forwarded(True)

    # engaging must close the gate again on the next stock message
    self.safety.set_controls_allowed(True)
    self._rx(self._stock_steering_msg())
    self._assert_ap1_forwarded(False)

  def test_openpilot_cannot_command_during_stock_autopark(self):
    self._rx(self._stock_steering_msg())
    self.safety.set_controls_allowed(True)

    # controls_allowed alone doesn't clear the flag until a stock message re-evaluates it, and
    # for as long as it is set openpilot must not add a second command to the bus
    self.assertFalse(self._tx(self._angle_cmd_msg(0, state=self.steer_control_types['ANGLE_CONTROL'])))
    self.assertFalse(self._tx(self._long_control_msg(10, accel_limits=(0, 0))))


if __name__ == "__main__":
  unittest.main()
