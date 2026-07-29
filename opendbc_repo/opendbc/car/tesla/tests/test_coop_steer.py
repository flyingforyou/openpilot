import pytest

from opendbc.car.tesla.carstate import legacy_steer_state
from opendbc.car.tesla.values import TeslaFlags, TeslaLegacyParams

HIGH_RATE = "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY"


def _state(coop, hands_on_level=0, eac_status="EAC_ACTIVE", eac_error_code=None, torque_pressed=False):
  return legacy_steer_state(hands_on_level, eac_status, eac_error_code, torque_pressed, coop)


def _lat_active(steer, lat_active_from_controls=True, hands_on_level=0.0):
  """The car controller's gate, mirrored so the tests cover the whole path.

  controlsd already drops CC.latActive on either steer fault, so the two are folded in here.
  """
  cc_lat_active = lat_active_from_controls and not steer.fault_temporary and not steer.fault_permanent
  return cc_lat_active and hands_on_level < 3 and not steer.high_angle_rate_safety


# ---- default behaviour is untouched ----

def test_default_takeover_still_disengages():
  steer = _state(coop=False, hands_on_level=3)
  assert steer.disengage is True, "steerDisengage -> ET.USER_DISABLE, the old behaviour"
  assert steer.pressed is False


def test_default_high_angle_rate_disengages_and_faults():
  steer = _state(coop=False, eac_status="EAC_INHIBITED", eac_error_code=HIGH_RATE)
  assert steer.disengage is True
  assert steer.fault_temporary is True


# ---- cooperative steering ----

def test_normal_driving_is_quiet():
  steer = _state(coop=True)
  assert (steer.pressed, steer.disengage, steer.fault_temporary, steer.fault_permanent) == \
         (False, False, False, False)
  assert _lat_active(steer) is True


def test_hands_on_takeover_pauses_lateral_only():
  steer = _state(coop=True, hands_on_level=3)

  assert steer.pressed is True, "steerOverride -> ET.OVERRIDE_LATERAL, keeps CC.enabled"
  assert steer.disengage is False, "must not become ET.USER_DISABLE"
  assert steer.fault_temporary is False
  assert _lat_active(steer, hands_on_level=3) is False, "openpilot must let go of the wheel"


def test_high_angle_rate_pauses_lateral_without_soft_disable():
  steer = _state(coop=True, eac_status="EAC_INHIBITED", eac_error_code=HIGH_RATE)

  assert steer.high_angle_rate_safety is True
  assert steer.pressed is True
  assert steer.disengage is False
  assert steer.fault_temporary is False, \
    "a temporary fault is ET.SOFT_DISABLE, and State.overriding has no other exit"
  assert _lat_active(steer) is False


def test_releasing_the_wheel_resumes_lateral():
  held = _state(coop=True, hands_on_level=3)
  released = _state(coop=True, hands_on_level=0)

  assert _lat_active(held, hands_on_level=3) is False
  assert released.pressed is False
  assert _lat_active(released) is True


def test_real_eac_fault_is_still_permanent():
  steer = _state(coop=True, eac_status="EAC_FAULT")
  assert steer.fault_permanent is True
  assert _lat_active(steer) is False


def test_other_inhibit_reasons_remain_temporary_faults():
  # Only the high-angle-rate reason is a driver takeover; anything else is a genuine fault
  steer = _state(coop=True, eac_status="EAC_INHIBITED", eac_error_code="EAC_ERROR_MAX_SPEED")
  assert steer.high_angle_rate_safety is False
  assert steer.fault_temporary is True
  assert steer.pressed is False
  assert _lat_active(steer) is False


@pytest.mark.parametrize("coop", [False, True])
def test_torque_override_is_reported_either_way(coop):
  steer = _state(coop=coop, torque_pressed=True)
  assert steer.pressed is True
  assert steer.disengage is False


def test_coop_steer_flag_does_not_collide():
  # TeslaLegacyParams shares CP.flags with TeslaFlags, so the bit has to be free in both
  used = {f.value for f in TeslaFlags if f is not TeslaFlags.COOP_STEER}
  used |= {f.value for f in TeslaLegacyParams}
  assert TeslaFlags.COOP_STEER.value not in used


# ---- stock autopark: openpilot has to go silent, not just let panda block it ----

class _FakeOut:
  steeringAngleDeg = 12.5
  vEgo = 0.0
  vEgoRaw = 0.0
  gasPressed = False


class _FakeCS:
  def __init__(self, autopark_frames=0):
    self.hands_on_level = 0
    self.high_angle_rate_safety = False
    self.stock_autopark_frames = autopark_frames
    self.out = _FakeOut()
    self.das_control = {"DAS_controlCounter": 0}


class _FakeCruise:
  cancel = False


class _FakeActuators:
  steeringAngleDeg = 0.0
  accel = 0.0

  def as_builder(self):
    return self


class _FakeCC:
  def __init__(self, enabled=False):
    self.enabled = enabled
    self.latActive = False
    self.longActive = False
    self.actuators = _FakeActuators()
    self.cruiseControl = _FakeCruise()


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
  cc.update(_FakeCC(enabled=False), _FakeCS(autopark_frames=120), 0)
  assert cc.apply_angle_last == _FakeOut.steeringAngleDeg, "no step when openpilot takes back over"
