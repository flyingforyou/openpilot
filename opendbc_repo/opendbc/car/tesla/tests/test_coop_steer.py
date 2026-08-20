import pytest

from opendbc.car.tesla.carstate import legacy_steer_state
from opendbc.car.tesla.coop_steering import (CoopSteeringCarController, STEER_OVERRIDE_MIN_TORQUE,
                                             STEER_OVERRIDE_MAX_TORQUE)
from opendbc.car.tesla.values import TeslaFlags, TeslaLegacyParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.vehicle_model import VehicleModel

HIGH_RATE = "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY"


def _state(hands_on_level=0, eac_status="EAC_ACTIVE", eac_error_code=None, torque_pressed=False):
  return legacy_steer_state(hands_on_level, eac_status, eac_error_code, torque_pressed)


# ---- the disengage fallback is deliberately unchanged ----

def test_hard_takeover_still_disengages():
  """Cooperative steering prevents reaching this, it does not remove it."""
  assert _state(hands_on_level=3).disengage is True


def test_high_angle_rate_still_disengages():
  steer = _state(eac_status="EAC_INHIBITED", eac_error_code=HIGH_RATE)
  assert steer.disengage is True
  assert steer.high_angle_rate_safety is True


def test_quiet_driving_is_quiet():
  steer = _state()
  assert (steer.pressed, steer.disengage, steer.fault_temporary, steer.fault_permanent) == \
         (False, False, False, False)


def test_real_eac_fault_is_permanent():
  assert _state(eac_status="EAC_FAULT").fault_permanent is True


def test_other_inhibit_reasons_are_temporary_faults():
  steer = _state(eac_status="EAC_INHIBITED", eac_error_code="EAC_ERROR_MAX_SPEED")
  assert steer.fault_temporary is True
  assert steer.high_angle_rate_safety is False


def test_coop_steer_flag_does_not_collide():
  # TeslaLegacyParams shares CP.flags with TeslaFlags, so the bit has to be free in both
  used = {f.value for f in TeslaFlags if f is not TeslaFlags.COOP_STEER}
  used |= {f.value for f in TeslaLegacyParams}
  assert TeslaFlags.COOP_STEER.value not in used


# ---- cooperative steering: follow the driver instead of letting go ----

class _CoopOut:
  def __init__(self, torque=0.0, angle=0.0, v=20.0):
    self.steeringTorque = torque
    self.steeringAngleDeg = angle
    self.steeringRateDeg = 0.0
    self.vEgo = v
    self.vEgoRaw = v


class _CoopCS:
  def __init__(self, **kw):
    self.out = _CoopOut(**kw)


def _vm():
  return VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))


def _settle(coop, vm, torque, angle=0.0, v=20.0, frames=200):
  """Run to steady state; the offset ramps rather than stepping."""
  out = angle
  for _ in range(frames):
    out = coop.update(angle, True, _CoopCS(torque=torque, angle=angle, v=v), vm).steeringAngleDeg
  return out


def test_torque_below_the_deadzone_does_nothing():
  coop, vm = CoopSteeringCarController(), _vm()
  out = _settle(coop, vm, torque=STEER_OVERRIDE_MIN_TORQUE * 0.8)
  assert abs(out) < 0.1, "hands resting on the wheel must not steer the car"


def test_driver_torque_moves_the_command_its_way():
  coop, vm = CoopSteeringCarController(), _vm()
  right = _settle(coop, vm, torque=STEER_OVERRIDE_MAX_TORQUE)
  coop2 = CoopSteeringCarController()
  left = _settle(coop2, vm, torque=-STEER_OVERRIDE_MAX_TORQUE)

  assert right > 0.5, "pushing right should command right"
  assert left < -0.5, "pushing left should command left"
  assert right == pytest.approx(-left, rel=0.2)


def test_more_torque_moves_it_further():
  vm = _vm()
  light = _settle(CoopSteeringCarController(), vm, torque=1.0)
  heavy = _settle(CoopSteeringCarController(), vm, torque=2.5)
  assert heavy > light > 0


def test_releasing_the_wheel_returns_the_offset_to_centre():
  coop, vm = CoopSteeringCarController(), _vm()
  held = _settle(coop, vm, torque=STEER_OVERRIDE_MAX_TORQUE)
  assert held > 0.5

  released = _settle(coop, vm, torque=0.0)
  assert abs(released) < abs(held) / 2, "should unwind once the driver lets go"


def test_inactive_lateral_commands_the_measured_angle():
  # nothing to fight over, and nothing to step from when it resumes
  coop, vm = CoopSteeringCarController(), _vm()
  out = coop.update(0.0, False, _CoopCS(torque=2.0, angle=17.0), vm).steeringAngleDeg
  assert out == pytest.approx(17.0, abs=1e-6)


def test_resume_starts_from_the_wheel_not_the_old_target():
  # 10 deg at 20 m/s, comfortably inside the lateral accel limit -- pick an angle that limit
  # would clip and the clip, not the resume ramp, is what the test measures
  coop, vm = CoopSteeringCarController(), _vm()
  coop.update(0.0, False, _CoopCS(angle=10.0), vm)          # disengaged, wheel at 10 deg

  first = coop.update(0.0, True, _CoopCS(angle=10.0), vm).steeringAngleDeg
  assert abs(first - 10.0) < 1.0, "must not step from 10 deg to the 0 deg target in one frame"

  # and it walks there over many frames rather than jumping
  for _ in range(50):
    out = coop.update(0.0, True, _CoopCS(angle=10.0), vm).steeringAngleDeg
  assert out < first, "target should still be converging"


def test_tuning_changes_how_far_the_same_push_goes():
  vm = _vm()
  light = CoopSteeringCarController()
  light.set_tuning(STEER_OVERRIDE_MAX_TORQUE, 2.0)
  heavy = CoopSteeringCarController()
  heavy.set_tuning(STEER_OVERRIDE_MAX_TORQUE, 1.0)

  assert _settle(light, vm, torque=1.5) > _settle(heavy, vm, torque=1.5), \
    "more lateral accel per Nm should feel lighter"


def test_tuning_cannot_collapse_the_torque_range():
  # a max at or under the deadzone would divide by ~zero in the gain calculation
  coop = CoopSteeringCarController()
  coop.set_tuning(STEER_OVERRIDE_MIN_TORQUE, 1.5)
  assert coop.torque_range > 0
  coop.set_tuning(-5.0, -5.0)
  assert coop.torque_range > 0 and coop.max_lat_accel > 0


def test_defaults_are_the_measured_values():
  coop = CoopSteeringCarController()
  assert coop.max_torque == pytest.approx(STEER_OVERRIDE_MAX_TORQUE)
  assert coop.torque_range == pytest.approx(STEER_OVERRIDE_MAX_TORQUE - STEER_OVERRIDE_MIN_TORQUE)


# ---- stock autopark: openpilot has to go silent, not just let panda block it ----

class _FakeOut:
  def __init__(self):
    from opendbc.car import structs as _s
    self.steeringAngleDeg = 12.5
    self.vEgo = 0.0
    self.vEgoRaw = 0.0
    self.gasPressed = False
    self.gearShifter = _s.CarState.GearShifter.drive


class _FakeCS:
  def __init__(self, autopark_frames=0, autopark_offered=False, gear=None):
    from opendbc.car import structs as _s
    self.hands_on_level = 0
    self.high_angle_rate_safety = False
    self.stock_autopark_frames = autopark_frames
    self.stock_autopark_offered = autopark_offered or autopark_frames > 0
    self.out = _FakeOut()
    self.out.gearShifter = _s.CarState.GearShifter.drive if gear is None else gear
    self.das_control = {"DAS_controlCounter": 0}


class _FakeCruise:
  def __init__(self):
    self.cancel = False


class _FakeActuators:
  steeringAngleDeg = 0.0
  accel = 0.0

  def as_builder(self):
    return self


class _FakeHud:
  setSpeed = 0.0


class _FakeCC:
  def __init__(self, enabled=False, cancel=False):
    self.enabled = enabled
    self._cancel = cancel
    self.latActive = False
    self.longActive = False
    self.actuators = _FakeActuators()
    self.cruiseControl = _FakeCruise()
    self.cruiseControl.cancel = cancel
    self.hudControl = _FakeHud()


def _carcontroller():
  from opendbc.car import Bus
  from opendbc.car.tesla.carcontroller import CarController
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import DBC
  CP = CarInterface.get_non_essential_params("TESLA_MODEL_X_HW1")
  return CarController(DBC[CP.carFingerprint], CP), CP


def _addrs_over(frames, cc, CC, CS):
  seen = set()
  for _ in range(frames):
    _, sends = cc.update(CC, CS, 0)
    seen |= {m[0] for m in sends}
  return seen


def test_openpilot_normally_owns_both_command_ids():
  cc, _ = _carcontroller()
  seen = _addrs_over(8, cc, _FakeCC(enabled=False), _FakeCS())
  assert 0x488 in seen, "DAS_steeringControl is sent even disengaged"
  assert 0x2B9 in seen, "DAS_control is sent even disengaged -- this is what collided"


def test_openpilot_goes_silent_during_a_stock_autopark_session():
  cc, _ = _carcontroller()
  seen = _addrs_over(8, cc, _FakeCC(enabled=False), _FakeCS(autopark_frames=120))
  assert seen == set(), "two counter sequences on one id is what aborted the maneuver"


def test_engaged_openpilot_keeps_the_bus():
  # panda closes the gate the moment controls are allowed; openpilot must not go quiet then
  cc, _ = _carcontroller()
  seen = _addrs_over(8, cc, _FakeCC(enabled=True), _FakeCS(autopark_frames=120))
  assert 0x488 in seen and 0x2B9 in seen


def test_angle_resumes_from_the_wheel_the_stock_module_left():
  cc, _ = _carcontroller()
  cc.apply_angle_last = -300.0
  CS = _FakeCS(autopark_frames=120)
  cc.update(_FakeCC(enabled=False), CS, 0)
  assert cc.apply_angle_last == CS.out.steeringAngleDeg, "no step when openpilot takes back over"


# ---- openpilot must not cancel cruise it could not have been driving ----

def _long_states(cc, CC, CS, frames=8):
  """The DAS_control acc_state byte openpilot actually put on the bus."""
  out = []
  for _ in range(frames):
    _, sends = cc.update(CC, CS, 0)
    for addr, dat, _bus in sends:
      if addr == 0x2B9:
        out.append((dat[1] >> 4) & 0x0F)
  return out


def test_cancel_is_sent_normally():
  cc, _ = _carcontroller()
  states = _long_states(cc, _FakeCC(enabled=False, cancel=True), _FakeCS())
  assert 13 in states, "ACC_CANCEL_GENERIC_SILENT when there is something to cancel"


def test_cancel_is_held_back_while_autopark_is_offered():
  # the cancel ended the recorded maneuver 0.45s after the stock module got the steering
  cc, _ = _carcontroller()
  states = _long_states(cc, _FakeCC(enabled=False, cancel=True), _FakeCS(autopark_offered=True))
  assert 13 not in states and 4 in states, "keep feeding the channel, just never cancel"


def test_cancel_is_held_back_out_of_drive_while_disengaged():
  from opendbc.car import structs as _s
  cc, _ = _carcontroller()
  states = _long_states(cc, _FakeCC(enabled=False, cancel=True),
                        _FakeCS(gear=_s.CarState.GearShifter.reverse))
  assert 13 not in states


def test_engaged_openpilot_can_still_cancel_out_of_drive():
  from opendbc.car import structs as _s
  cc, _ = _carcontroller()
  states = _long_states(cc, _FakeCC(enabled=True, cancel=True),
                        _FakeCS(gear=_s.CarState.GearShifter.reverse))
  assert 13 in states, "only the disengaged case is the one with nothing to take over from"


def test_channel_is_still_fed_while_autopark_is_merely_offered():
  # full silence for the whole offer window would starve a 25Hz channel; it was 22s in the log
  cc, _ = _carcontroller()
  states = _long_states(cc, _FakeCC(enabled=False), _FakeCS(autopark_offered=True))
  assert len(states) > 0 and set(states) == {4}
