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


# Panda hands the bus back 200ms after the stock module stops asking; openpilot has to stay quiet
# at least that long, so hold a little past it. carState runs at 100Hz.
STOCK_AUTOPARK_HOLD_FRAMES = 30

# UI_mapSpeedLimit is banded, not a number: raw n means "the limit is at most MAP_SPEED_BAND[n]".
# 0 is "no value", 30 is unrestricted and 31 is SNA, so only 1..29 carry a speed.
MAP_SPEED_BAND = (0, 5, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90,
                  95, 100, 105, 110, 115, 120, 130, 140, 150, 160)

# UI_roadSign multiplexes UI_driverAssistRoadSign. Only these two slots carry speed context; the
# others are stop-line and traffic-light distances. CANParser has no multiplex support at all --
# it decodes every signal in the message on every frame -- so reading a slot's signals without
# first checking the selector returns whatever the other slots' bits happened to be.
ROAD_SIGN_MUX_MAP_SPEED = 3
ROAD_SIGN_MUX_FLEET_SPEED = 4

# Message rates on the party bus: 0x238 at 10Hz cycling four slots (so ~2.5Hz per slot), 0x3C8 at
# 2Hz, 0x2F8 at 1Hz. carState runs at 100Hz, so this is three seconds without a map message.
NAV_MAP_STALE_FRAMES = 300


def decode_tesla_gap(raw_gap: int) -> int:
  """Convert Tesla legacy DTR_Dist_Rq raw value to a gap level.

  Returns 1 through 7 for a valid Tesla gap, or 0 for SNA/an unknown value.
  """
  return TESLA_DTR_RAW_TO_GAP.get(int(raw_gap), 0)


def snap_base_speed_limit(v_ms: float, units_kph: bool) -> float:
  """UI_baseMapSpeedLimitMPS -> m/s on the grid a posted limit can actually sit on.

  This is the only limit that arrives as a raw m/s number -- 8 bits at 0.25 m/s -- and the gateway
  truncates rather than rounds, so every value lands up to 0.25 m/s (0.56 mph) *below* the real
  limit. Measured: a 35 comes over as 34.673, a 40 as 39.706, a 45 as 44.739, a 65 as 64.871. The
  other three sources are quantised in the display unit and are exact.

  Left alone that fraction is not cosmetic. It drops a 40 mph road under OFFSET_SPLIT so the
  offset ladder pays +5 instead of +10 -- a 5 mph loss on the most common road of the drive -- and
  the cluster stalk's own 1 mph deadband then settles either side of it and stays, which is why
  the same road showed 44 on one pass and 45 on the next.

  Posted limits are multiples of five in whichever unit the car displays, which is also the
  quantum of the other three sources, so snapping to that grid is what puts base back on the
  number the sign actually carries. The largest correction possible is half a step, far short of
  the 5-unit spacing, so this cannot turn one limit into its neighbour.
  """
  if v_ms <= 0.0:
    return 0.0
  to_unit = CV.MS_TO_KPH if units_kph else CV.MS_TO_MPH
  from_unit = CV.KPH_TO_MS if units_kph else CV.MPH_TO_MS
  return round(v_ms * to_unit / 5.0) * 5.0 * from_unit


def decode_banded_speed_limit(raw: float, units_kph: bool) -> float:
  """UI_mapSpeedLimit / DAS_fusedSpeedLimit style band -> m/s, or 0.0 for no usable value."""
  idx = int(raw)
  if not 1 <= idx < len(MAP_SPEED_BAND):
    return 0.0
  return MAP_SPEED_BAND[idx] * (CV.KPH_TO_MS if units_kph else CV.MPH_TO_MS)


class NavMapDecoder:
  """Latches the gateway's map view out of three party-bus messages.

  Everything here is last-value-wins by design. The three messages run at different rates and
  0x238 is multiplexed, so at any instant most of these fields were last written some frames ago;
  that is the shape of the data, not a defect. What matters is that a slot is only latched from a
  frame whose selector actually says that slot, and that a value of zero is passed through rather
  than being papered over -- "the map has no limit here" is the single most useful thing this
  message says on a ramp, and it says it with a zero.
  """

  def __init__(self):
    self.base_speed_limit = 0.0
    self.fleet_top_quartile = 0.0
    self.fleet_spline_speed = 0.0
    self.fleet_spline_accel = 0.0
    self.fleet_median = 0.0
    self.spline_confidence = 0
    self.ramp_type = 0
    self.last_map_nanos = 0
    self.stale_frames = NAV_MAP_STALE_FRAMES

  def update(self, ret: structs.CarState, cp_party, cp_ap_party) -> None:
    road_sign = cp_party.vl["UI_driverAssistRoadSign"]
    map_data = cp_party.vl["UI_driverAssistMapData"]
    gps = cp_party.vl["UI_gpsVehicleSpeed"]

    # One selector read, then only the slot it names. UI_splineLocConfidence sits outside the
    # multiplexed region, so it is valid on every frame regardless of the selector.
    mux = int(road_sign["UI_roadSign"])
    self.spline_confidence = int(road_sign["UI_splineLocConfidence"])
    if mux == ROAD_SIGN_MUX_MAP_SPEED:
      self.base_speed_limit = float(road_sign["UI_baseMapSpeedLimitMPS"])
      self.fleet_top_quartile = float(road_sign["UI_topQrtlFleetSpeedMPS"])
    elif mux == ROAD_SIGN_MUX_FLEET_SPEED:
      self.fleet_spline_speed = float(road_sign["UI_meanFleetSplineSpeedMPS"])
      self.fleet_spline_accel = float(road_sign["UI_meanFleetSplineAccelMPS2"])
      self.fleet_median = float(road_sign["UI_medianFleetSpeedMPS"])
      self.ramp_type = int(road_sign["UI_rampType"])

    offset_kph = bool(gps["UI_userSpeedOffsetUnits"])
    mpp_kph = bool(gps["UI_mapSpeedLimitUnits"])
    mpp_raw = float(gps["UI_mppSpeedLimit"])

    # Staleness is counted rather than read off a clock: ts_nanos is the only timestamp the
    # parser exposes publicly, and what matters is whether the gateway is still talking, not how
    # far behind any single field is.
    map_nanos = cp_party.ts_nanos["UI_driverAssistMapData"]["UI_mapSpeedLimit"]
    if map_nanos != self.last_map_nanos:
      self.last_map_nanos = map_nanos
      self.stale_frames = 0
    else:
      self.stale_frames = min(self.stale_frames + 1, NAV_MAP_STALE_FRAMES)

    nav = ret.navMap
    nav.valid = map_nanos != 0 and self.stale_frames < NAV_MAP_STALE_FRAMES
    nav.baseSpeedLimit = snap_base_speed_limit(self.base_speed_limit, mpp_kph)
    nav.mapSpeedLimit = decode_banded_speed_limit(map_data["UI_mapSpeedLimit"],
                                                  bool(map_data["UI_mapSpeedUnits"]))
    # mppSpeedLimit is a plain number in the declared unit, not a band. 0 is no value and the
    # top of its range (155) is the message's "no limit applies here".
    nav.mppSpeedLimit = (mpp_raw * (CV.KPH_TO_MS if mpp_kph else CV.MPH_TO_MS)
                         if 0 < mpp_raw < 155 else 0.0)
    fused_raw = float(cp_ap_party.vl["AutopilotStatus"]["DAS_fusedSpeedLimit"])
    nav.fusedSpeedLimit = (fused_raw * (CV.KPH_TO_MS if mpp_kph else CV.MPH_TO_MS)
                           if 0 < fused_raw < 155 else 0.0)

    nav.fleetSplineSpeed = self.fleet_spline_speed
    nav.fleetSplineAccel = self.fleet_spline_accel
    nav.fleetMedianSpeed = self.fleet_median
    nav.fleetTopQuartileSpeed = self.fleet_top_quartile

    nav.roadClass = int(map_data["UI_roadClass"])
    nav.rampType = self.ramp_type
    nav.splineConfidence = self.spline_confidence
    nav.gpsRoadMatch = bool(map_data["UI_gpsRoadMatch"])
    nav.navRouteActive = bool(map_data["UI_navRouteActive"])
    nav.speedOffset = float(gps["UI_userSpeedOffset"]) * (CV.KPH_TO_MS if offset_kph else CV.MPH_TO_MS)


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
    self.fsd14_error_logged = False
    self.suspected_fsd14 = False

    self.hands_on_level = 0
    self.high_angle_rate_safety = False
    self.stock_autopark_frames = 0
    self.stock_autopark_offered = False
    self.das_control = None
    # The factory's latest DAS_object frame per group, kept only when the cluster workaround is on.
    self.das_objects: dict[int, dict[str, float]] = {}
    # The stalk's own last frame. Emulating a press means re-sending the SCCM's frame with one
    # field changed, so everything else on it -- turn signals, wipers, follow distance, the gear
    # stalk's own state -- goes back out as the SCCM had it rather than as something we invented.
    self.stw_actn: dict[str, float] | None = None
    self.cruise_gap = 0
    # Raven's party DBC carries none of the map messages, so it gets no decoder at all rather
    # than one that would fault the first time it looked a message up.
    self.nav_map = None if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else NavMapDecoder()

  def update_autopark_state(self, autopark_now: bool):
    # Takes the decoded "the park module has the car" boolean rather than the signal it came
    # from, so both car generations can share this. Model 3/Y read DI_autoparkState off DI_state;
    # the legacy DI_state has no such field, so HW1 derives it from AutopilotStatus.
    #
    # This used to be an edge latch: arm only on the rising edge of autopark_now, and only if
    # cruise was not already enabled the frame before. In practice cruise is very often already
    # on before the car ever offers a spot -- approaching with ACC engaged is the ordinary case,
    # not the exception -- so the arm condition never fired and cruiseState.enabled leaked
    # through for the whole encounter. Once that happens on this platform it is not a one-frame
    # blip: panda's own fwd_hook state machine drops tesla_legacy_autopark_ts_valid the instant
    # controls_allowed goes true, which closes the DAS_steeringControl forwarding gate outright
    # (tesla_legacy_stock_autopark requires !controls_allowed) -- the stock module keeps steering
    # on bus 2 but the EPS on bus 0 stops hearing it, and ~250-300ms later that surfaces as
    # EAC_INHIBITED/TMP_FAULT and an APC_ABORT. No latch, no race: mask for as long as the car
    # says autopark has the wheel, full stop.
    self.autopark = autopark_now

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
    self.update_autopark_state(autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED"))

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

    # Stock autopark, read before cruise because it gates it. Model 3/Y get DI_autoparkState
    # straight off DI_state; the legacy DI_state carries no autopark field at all, so the
    # closest equivalent is the park module's own offer on AutopilotStatus. Raven (Model S HW3)
    # uses a party DBC without that message, so it never reports the maneuver.
    autopark_offered = False
    if self.CP.carFingerprint != CAR.TESLA_MODEL_S_HW3:
      autopilot_status = cp_ap_party.vl["AutopilotStatus"]
      autopark_offered = (int(autopilot_status["DAS_autoparkReady"]) == 1 or
                          int(autopilot_status["DAS_autoparkWaitingForBrake"]) == 1)

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_offered)

    # Match panda safety cruise engaged logic. Autopark drives the car through the ACC channel,
    # so the car reports cruise enabled the moment the maneuver starts. Taking that at face value
    # made openpilot ask to cancel the cruise the park module was using and try to engage itself
    # in reverse; both came off this one signal.
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
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
      das_left_blindspot = int(autopilot_status["DAS_blindSpotRearLeft"]) in (1, 2)
      das_right_blindspot = int(autopilot_status["DAS_blindSpotRearRight"]) in (1, 2)

    ret.leftBlindspot = park_left_blindspot or das_left_blindspot
    ret.rightBlindspot = park_right_blindspot or das_right_blindspot

    # Navigation / map view of the road ahead. Decoded for every legacy car that has the
    # messages: it is pure decode off what the gateway broadcasts anyway, and whether anything
    # acts on it is a planner decision, not a car one.
    if self.nav_map is not None:
      self.nav_map.update(ret, cp_party, cp_ap_party)

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

    # What the factory ACC itself is asking for, straight off its own bus -- logged every cycle
    # regardless of who owns DAS_control, so a run with openpilot driving still carries a shadow
    # of what the stock system would have done, for comparing after the fact.
    ret.stockAccelMin = float(self.das_control["DAS_accelMin"])
    ret.stockAccelMax = float(self.das_control["DAS_accelMax"])

    # Stock autopark needs DAS_control and DAS_steeringControl to itself for the whole maneuver.
    # Held for a while after the module goes quiet, to match the panda's forwarding latch --
    # openpilot must stop transmitting for at least as long as panda opens the gate, or the two
    # command streams collide on the same arbitration id and the maneuver aborts.
    stock_acc_state = int(cp_ap_pt.vl["DAS_control"]["DAS_accState"])
    if 5 <= stock_acc_state <= 11 or ret.stockLkas or stock_steer_type == 1:  # APC range / ANGLE_CONTROL
      self.stock_autopark_frames = STOCK_AUTOPARK_HOLD_FRAMES
    else:
      self.stock_autopark_frames = max(self.stock_autopark_frames - 1, 0)

    # Deliberately wider than the silence window above, and used only to hold back the cancel.
    # Autopark drives the car through the ACC channel, so the car reports cruise enabled the
    # moment it starts and controlsd asks to cancel it -- that cancel ended the recorded
    # maneuver 0.45s after the stock module had finally taken the steering. Going fully silent
    # for the whole time autopark is merely on offer would be worse: it was 22s in that
    # recording, and DAS_control is a channel the car expects fed at 25Hz.
    self.stock_autopark_offered = autopark_offered or self.stock_autopark_frames > 0

    self.update_das_objects(cp_ap_party)

    # Same lazy-registration rule as DAS_object: touch vl once so the parser picks the message up.
    if self.CP.flags & TeslaFlags.SYNC_CLUSTER_SPEED:
      stw = cp_party.vl["STW_ACTN_RQ"]
      self.stw_actn = dict(stw) if len(stw) else None

    return ret

  def update_das_objects(self, cp_ap_party) -> None:
    """Collect the factory object frames that arrived since the last update, one per group.

    Deliberately not a running snapshot. Holding the last frame of each group and re-sending it
    every cycle would put our copy on the bus at the control rate -- eight times what the factory
    sends -- carrying values that have not changed since the group last came round. Emptying it
    each update means exactly one relabelled frame goes out per factory frame, at the factory's
    own cadence, which is also what keeps the extra bus load proportionate.

    DAS_object rotates through its groups -- lead, left, right, cutin, headings -- one per frame at
    about 6.7 Hz each, so a single read only ever sees whichever came last. vl_all carries every
    frame received since the previous update, which is what makes it possible to hold all of them
    at once. The signals arrive as parallel lists, one entry per frame, so they zip back into
    frames in order.

    Only kept for the sake of re-sending them with the vehicle type substituted; nothing here
    feeds control.
    """
    self.das_objects.clear()

    if not (self.CP.flags & TeslaFlags.CARS_AS_TRUCKS):
      return

    # These parsers are built with no message list and pick messages up on demand, but only
    # through vl -- that is the one wired for lazy registration. vl_all is a plain dict that
    # _add_message fills in, so reading it for a message nobody has touched returns nothing,
    # forever and silently. Touching vl once registers it; from the next frame on, vl_all has it.
    cp_ap_party.vl["DAS_object"]

    frames = cp_ap_party.vl_all.get("DAS_object")
    if not frames:
      return

    names = list(frames)
    columns = [frames[n] for n in names]
    # One entry per frame in every column. Guard rather than zip strictly: a mismatch here would
    # be a parser bug, and raising in CarState would take the car down over a display feature.
    if len({len(c) for c in columns}) != 1:
      return

    for row in zip(*columns):
      values = dict(zip(names, row))
      self.das_objects[int(values["DAS_objectId"])] = values

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
