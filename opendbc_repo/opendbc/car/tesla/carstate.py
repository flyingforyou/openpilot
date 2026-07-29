import copy
from typing import NamedTuple

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.teslacan import get_steer_ctrl_type
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, TeslaFlags, TeslaLegacyParams, CAR, LEGACY_CARS

ButtonType = structs.CarState.ButtonEvent.Type

TESLA_DTR_RAW_TO_GAP = {
  0: 1,
  33: 2,
  66: 3,
  100: 4,
  133: 5,
  166: 6,
  200: 7,
}


# Panda hands the bus back 1.0s after the stock module stops asking; openpilot has to stay quiet
# at least that long, so hold a little past it. carState runs at 100Hz.
STOCK_AUTOPARK_HOLD_FRAMES = 120


def decode_tesla_gap(raw_gap: int) -> int:
  """Convert Tesla legacy DTR_Dist_Rq raw value to a gap level.

  Returns 1 through 7 for a valid Tesla gap, or 0 for SNA/an unknown value.
  """
  return TESLA_DTR_RAW_TO_GAP.get(int(raw_gap), 0)


class LegacySteerState(NamedTuple):
  pressed: bool
  disengage: bool
  fault_temporary: bool
  fault_permanent: bool
  high_angle_rate_safety: bool


def legacy_steer_state(hands_on_level: float, eac_status: str | None,
                       eac_error_code: str | None, torque_pressed: bool) -> LegacySteerState:
  """How a driver's hands on the wheel are reported to the rest of openpilot.

  Upstream behaviour, deliberately unchanged: a hard takeover is a disengage. steeringDisengage
  becomes EventName.steerDisengage, an ET.USER_DISABLE, and openpilot drops out.

  Cooperative steering does not touch this. It works by keeping the driver from ever having to
  push hard enough to get here -- see coop_steering.py -- so this stays as the fallback for a
  driver who overpowers it anyway, which is the path that has always worked on this car.
  """
  high_angle_rate_safety = (eac_status == "EAC_INHIBITED" and
                            eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
  driver_override = hands_on_level >= 3

  return LegacySteerState(torque_pressed, driver_override or high_angle_rate_safety,
                          eac_status == "EAC_INHIBITED", eac_status == "EAC_FAULT",
                          high_angle_rate_safety)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False
    self.fsd14_error_logged = False
    self.suspected_fsd14 = False

    self.hands_on_level = 0
    self.high_angle_rate_safety = False
    self.stock_autopark_frames = 0
    self.das_control = None
    self.cruise_gap = 0

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cp_party.vl["ESP_B"]["ESP_vehicleStandstillSts"] == 1
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # FSD switched from using ANGLE_CONTROL to LANE_KEEP_ASSIST to likely keep the old steering override disengage logic.
    # LKAS switched from LANE_KEEP_ASSIST to ANGLE_CONTROL to likely allow overriding LKAS events smoothly
    lkas_ctrl_type = get_steer_ctrl_type(self.CP.flags, 2)
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == lkas_ctrl_type  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # TODO: find for TESLA_MODEL_X and HW2.5 vehicles
    if not (self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

      # Because we don't have FSD 14 detection outside of a set of FW, we should check if this FW is accidentally missing from FSD_14_FW
      # 1. If in Autosteer or FSD, already caught by invalidLkasSetting
      # 2. If in TACC and DAS ever sends ANGLE_CONTROL (1), we can infer it's trying to do LKAS on FSD 14+
      angle_control = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 1  # ANGLE_CONTROL
      if not ret.invalidLkasSetting and angle_control and not self.CP.flags & TeslaFlags.FSD_14:
        self.suspected_fsd14 = True

      if self.suspected_fsd14:
        ret.invalidLkasSetting = True
        if not self.fsd14_error_logged:
          carlog.error("FSD 14 detected, but FW not in FSD_14_FW set")
          self.fsd14_error_logged = True

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  def update_legacy(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    torque_pressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)

    steer = legacy_steer_state(self.hands_on_level, eac_status, eac_error_code, torque_pressed)
    ret.steeringPressed = steer.pressed
    ret.steeringDisengage = steer.disengage
    ret.steerFaultTemporary = steer.fault_temporary
    ret.steerFaultPermanent = steer.fault_permanent
    # read by the car controller to drop the angle command while the driver is turning
    self.high_angle_rate_safety = steer.high_angle_rate_safety

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    if self.CP.carFingerprint == CAR.TESLA_MODEL_X_HW1:
      ret.leftBlinker = cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 1
      ret.rightBlinker = cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 2

      # Steering wheel Gap 1-7 setting. Raw 0 is a valid gap (ACC_DIST_1), so the signal must not
      # be read before the message has actually been received: CANParser zero-inits every signal,
      # which would otherwise latch gap 1 (the shortest follow distance) on startup. Until a gap
      # is received we publish 0 so the planner keeps using the personality based tFollow.
      # 255/SNA and other unknown raw values hold the last valid gap.
      if cp_chassis.ts_nanos["STW_ACTN_RQ"]["DTR_Dist_Rq"] != 0:
        decoded_gap = decode_tesla_gap(cp_chassis.vl["STW_ACTN_RQ"]["DTR_Dist_Rq"])
        if decoded_gap != 0:
          self.cruise_gap = decoded_gap
      ret.cruiseState.gapAdjust = self.cruise_gap
    else:
      ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
      ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
      ret.cruiseState.gapAdjust = 0

    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # Blindspot combines two independent legacy signals:
    #  - PARK_status2 (ultrasonic Park Assist, chassis bus): what the instrument cluster's
    #    blind spot icon actually reflects, active mainly at low/parking speed.
    #  - AutopilotStatus (vision/radar, autopilot party bus): the DAS auto-lane-change
    #    blind spot assessment, active mainly at road speed. Values 1 and 2 are warning
    #    levels; 3 is SNA and must not block a lane change.
    # Raven (Model S HW3) uses a different party DBC that doesn't have AutopilotStatus, but
    # its chassis bus DBC still has PARK_status2, so that half still applies unconditionally.
    park_status = cp_chassis.vl["PARK_status2"]
    park_left_blindspot = int(park_status["PARK_sdiBlindSpotLeft"]) == 1
    park_right_blindspot = int(park_status["PARK_sdiBlindSpotRight"]) == 1

    das_left_blindspot = False
    das_right_blindspot = False
    if self.CP.carFingerprint != CAR.TESLA_MODEL_S_HW3:
      autopilot_status = cp_ap_party.vl["AutopilotStatus"]
      das_left_blindspot = int(autopilot_status["DAS_blindSpotRearLeft"]) in (1, 2)
      das_right_blindspot = int(autopilot_status["DAS_blindSpotRearRight"]) in (1, 2)

    ret.leftBlindspot = park_left_blindspot or das_left_blindspot
    ret.rightBlindspot = park_right_blindspot or das_right_blindspot

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    stock_steer_type = int(cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"])
    ret.stockLkas = stock_steer_type == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    # Stock autopark needs DAS_control and DAS_steeringControl to itself for the whole maneuver.
    # Held for a while after the module goes quiet, to match the panda's forwarding latch --
    # openpilot must stop transmitting for at least as long as panda opens the gate, or the two
    # command streams collide on the same arbitration id and the maneuver aborts.
    stock_acc_state = int(cp_ap_pt.vl["DAS_control"]["DAS_accState"])
    if 5 <= stock_acc_state <= 11 or ret.stockLkas or stock_steer_type == 1:  # APC range / ANGLE_CONTROL
      self.stock_autopark_frames = STOCK_AUTOPARK_HOLD_FRAMES
    else:
      self.stock_autopark_frames = max(self.stock_autopark_frames - 1, 0)

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in LEGACY_CARS:
      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.powertrain),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.autopilot_powertrain),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], [], CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
