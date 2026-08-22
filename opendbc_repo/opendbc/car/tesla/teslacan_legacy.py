from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


def j1850_crc(data: bytes) -> int:
  """SAE J1850: poly 0x1D, init 0xFF, final xor 0xFF. What STW_ACTN_RQ carries in its last byte."""
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers
    self.CCP = CarControllerParams
    self.jerk_upper = self.CCP.JERK_LIMIT_MAX
    self.jerk_lower = self.CCP.JERK_LIMIT_MIN

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_ic_lanes(self, base_values, overrides=None):
    """Clone factory AP1 0x239 and replace only the path/lane fields openpilot owns."""
    values = dict(base_values)
    if overrides:
      values.update(overrides)
    return self.packers[CANBUS.party].make_can_msg("DAS_lanes", CANBUS.party, values)

  def create_ic_leads(self, lead1, lead2):
    """Create Unity-style display-only group-0 0x309 from openpilot radar leads."""
    # An unused slot must be SATURATED, not zeroed: the factory pins an empty slot's distance to
    # the top of its 8-bit range (127.5 m), and that is how the cluster knows the slot holds no
    # object. Leaving it at 0 draws a phantom car sitting on the bumper. See das_object.py.
    no_object_dx = 127.5
    values = {
      "DAS_objectId": 0,
      "DAS_objVehType": 0,
      "DAS_objVehRelevantForControl": 0,
      "DAS_objVehDx": no_object_dx,
      "DAS_objVehVxRel": 0,
      "DAS_objVehDy": 0,
      "DAS_objVehId": 0,
      "DAS_objVeh2Type": 0,
      "DAS_objVeh2RelevantForControl": 0,
      "DAS_objVeh2Dx": no_object_dx,
      "DAS_objVeh2VxRel": 0,
      "DAS_objVeh2Dy": 0,
      "DAS_objVeh2Id": 0,
    }
    if lead1 is not None:
      values.update({"DAS_objVehType": 2, "DAS_objVehDx": lead1[0],
                     "DAS_objVehVxRel": lead1[1], "DAS_objVehDy": lead1[2], "DAS_objVehId": 1})
    if lead2 is not None:
      values.update({"DAS_objVeh2Type": 2, "DAS_objVeh2Dx": lead2[0],
                     "DAS_objVeh2VxRel": lead2[1], "DAS_objVeh2Dy": lead2[2], "DAS_objVeh2Id": 2})
    return self.packers[CANBUS.party].make_can_msg("DAS_object", CANBUS.party, values)

  def create_ic_status(self, base_values, active):
    """Clone factory AP1 0x399, preserve its counter/status bits, and change the IC AP mode.

    A logged route with the stock IC genuinely showing both lanes had autopilotStatus=3
    (ACTIVE_1) throughout -- never 5 (NAV). NAV was tried, on the theory that it was the
    "correct" active state and 3 just the conservative fallback, and reverted: it did not
    produce the NoA route-line display, and 3 is what real frames use for this rendering anyway.

    autopilotStatus is the only field patched. The same route showed DAS_autoLaneChangeState,
    DAS_autopilotHandsOnState and DAS_laneDepartureWarning changing continuously and
    meaningfully in the factory's own frames -- real lane-change availability, hands-on state,
    solid-line blocks -- none of it something openpilot has an equivalent source for. An earlier
    version of this zeroed those fields to build a frame from scratch the way Unity's
    create_das_status does; the logged route is what shows that was the wrong call here; cloning
    lets the factory's own live perception state keep describing itself, which openpilot did not
    take away from it by taking over actuation. When not active the factory autopilotStatus is
    left untouched too: this is the passthrough re-send that keeps the cluster alive while the
    feature is off (panda has blocked the factory copy).
    """
    values = dict(base_values)
    if active:
      values["autopilotStatus"] = 3  # ACTIVE_1
    values["DAS_statusChecksum"] = 0
    data = self.packers[CANBUS.party].make_can_msg("AutopilotStatus", CANBUS.party, values)[1]
    values["DAS_statusChecksum"] = self.checksum(0x399, data[:7])
    return self.packers[CANBUS.party].make_can_msg("AutopilotStatus", CANBUS.party, values)

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_stalk_command(self, values, lever):
    """The SCCM's own frame with the cruise lever overridden, counter bumped and CRC redone.

    Copied rather than built: STW_ACTN_RQ carries turn signals, wipers, follow distance and the
    gear stalk alongside the cruise lever, and the SCCM keeps sending its own copy regardless.
    Sending anything but its current state with one field moved would be inventing input.

    CRC_STW_ACTN_RQ is SAE J1850 over the first seven bytes -- reproduced exactly on 40/40 logged
    frames -- and MC_STW_ACTN_RQ is a 4-bit rolling counter the SCCM advances per frame.
    """
    values = dict(values)
    values["SpdCtrlLvr_Stat"] = lever
    values["MC_STW_ACTN_RQ"] = (int(values.get("MC_STW_ACTN_RQ", 0)) + 1) % 16
    values["CRC_STW_ACTN_RQ"] = 0
    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", CANBUS.party, values)[1]
    values["CRC_STW_ACTN_RQ"] = j1850_crc(data[:7])
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, gas_pressed, hud_set_speed=0.0):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # Zero is how this car is told to slow down, not a placeholder for a display value: the DI
      # works to the target it is given, and the accel command only bounds how hard. Sending the
      # real cruise target while decelerating leaves the car with no reason to brake, which is
      # what stopped deceleration working entirely. So the decel case is decided first and is
      # not negotiable; hud_set_speed only ever fills in the cruising case, where the target is
      # genuinely what the car should be driving to.
      if accel < 0:
        set_speed = 0
      elif hud_set_speed > 0:
        set_speed = hud_set_speed * CV.MS_TO_KPH
      else:
        set_speed = V_CRUISE_MAX

    if gas_pressed:
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      self.jerk_lower = max(self.jerk_lower - self.CCP.JERK_RAMP_RATE, self.CCP.JERK_LIMIT_MIN)
      self.jerk_upper = min(self.jerk_upper + self.CCP.JERK_RAMP_RATE, self.CCP.JERK_LIMIT_MAX)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": self.jerk_lower,
      "DAS_jerkMax": self.jerk_upper,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)
