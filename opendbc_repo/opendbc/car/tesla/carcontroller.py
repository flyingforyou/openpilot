import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR, StalkLever, TeslaFlags
from opendbc.car.vehicle_model import VehicleModel


# The carcontroller runs at 100Hz. Hold a press long enough for the SCCM's own frames not to be
# the only thing the DI sees in that window, then wait out its response before judging the error
# again -- measured from the logs, where the setpoint moved 0.19-0.23s after each release.
#
# The press stays short on purpose: a cruise stalk held down normally auto-repeats, and a held
# DOWN would walk the setpoint away on its own. The wait is the only part worth tuning, and 400ms
# was too close to the edge -- one step per 0.46s tracks 10.9 mph/s against a target that was
# measured moving at up to 10.85 mph/s over a real drive, which is no margin at all. 250ms tracks
# 16.1 mph/s and still leaves the DI's slowest observed response (0.23s) inside the window.
STALK_PRESS_FRAMES = 5   # 50ms
STALK_WAIT_FRAMES = 25   # 250ms


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.ic_model = None
    self.ic_radar = None
    self.ic_enabled = False
    self.ic_last_lanes_nanos = 0
    self.ic_last_status_nanos = 0
    # Follow the driver's hands rather than letting go of the wheel. Off unless opted in.
    self.coop_steering = bool(CP.flags & TeslaFlags.COOP_STEER) and CP.carFingerprint in LEGACY_CARS
    self.coop_steer = CoopSteeringCarController()
    # Cluster-speed sync. The DI applies a step on the release edge, so a press is held for a few
    # frames and then let go, and nothing else is sent until the DI's reported setpoint has had
    # time to move. Without that wait one target error would fire a burst of presses and overshoot.
    self.stalk_lever = StalkLever.IDLE
    self.stalk_press_frames = 0
    self.stalk_wait_frames = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1,):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      self.tesla_can = TeslaCANRaven(self.packers)
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def _ic_lane_overrides(self):
    """Convert modelV2 path to the AP1 IC cubic used by Tesla Unity."""
    model = self.ic_model
    if model is None or len(model.position.x) < 6 or len(model.position.y) < 6:
      return {}

    x = np.asarray(model.position.x, dtype=float)
    y = np.asarray(model.position.y, dtype=float)
    n = min(len(x), len(y))
    x, y = x[:n], y[:n]
    valid = np.isfinite(x) & np.isfinite(y) & (x >= 0.0) & (x <= 100.0)
    if np.count_nonzero(valid) < 6:
      return {}

    try:
      coefs = np.polyfit(x[valid], y[valid], 3)
    except (ValueError, np.linalg.LinAlgError):
      return {}

    # Unity's IC_LANE_SCALE was 0.5: C2 gets 2^2 and C3 gets 2^3. C1 was
    # deliberately suppressed to keep the cluster path stable/centered.
    probs = list(model.laneLineProbs)
    return {
      "DAS_leftLaneExists": int(len(probs) > 1 and probs[1] > 0.45),
      "DAS_rightLaneExists": int(len(probs) > 2 and probs[2] > 0.45),
      "DAS_virtualLaneWidth": 4.0,
      "DAS_virtualLaneViewRange": 50.0,
      "DAS_virtualLaneC0": float(np.clip(coefs[3], -3.5, 3.5)),
      "DAS_virtualLaneC1": 0.0,
      "DAS_virtualLaneC2": float(np.clip(coefs[1] * 4.0, -0.0025, 0.0025)),
      "DAS_virtualLaneC3": float(np.clip(coefs[0] * 8.0, -3.0e-5, 3.0e-5)),
      "DAS_leftLineUsage": 2 if len(probs) > 1 and probs[1] > 0.45 else 0,
      "DAS_rightLineUsage": 2 if len(probs) > 2 and probs[2] > 0.45 else 0,
      "DAS_leftFork": 1 if len(probs) > 0 and probs[0] > 0.25 else 0,
      "DAS_rightFork": 1 if len(probs) > 3 and probs[3] > 0.25 else 0,
    }

  @staticmethod
  def _ic_lead(lead, path_c0):
    if lead is None or not lead.present:
      return None
    return (
      float(np.clip(lead.dRel, 0.0, 126.0)),
      float(np.clip(int(lead.vRel), -30, 26)),
      float(np.clip(path_c0 - lead.yRel, -22.05, 22.4)),
    )

  def update_ic(self, CC, CS):
    if not (self.CP.flags & TeslaFlags.IC_INTEGRATION) or not CC.enabled:
      return []
    if self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
      return []

    # Panda always blocks the factory 0x239/0x399 while engaged, so openpilot must always re-send
    # them to keep the cluster alive -- even when the feature is off, where it re-sends the factory
    # content untouched (passthrough). ic_enabled decides only whether the path/status/leads carry
    # openpilot's data, and it is a live switch so it can be flipped mid-drive from /live.
    on = self.ic_enabled
    sends = []
    new_lane_frame = CS.das_lanes is not None and CS.das_lanes_nanos != self.ic_last_lanes_nanos
    send_leads = on and self.frame % 10 == 0 and self.ic_radar is not None
    lane_overrides = self._ic_lane_overrides() if (on and (new_lane_frame or send_leads)) else {}

    if new_lane_frame:
      sends.append(self.tesla_can.create_ic_lanes(CS.das_lanes, lane_overrides))
      self.ic_last_lanes_nanos = CS.das_lanes_nanos

    if CS.autopilot_status is not None and CS.autopilot_status_nanos != self.ic_last_status_nanos:
      sends.append(self.tesla_can.create_ic_status(CS.autopilot_status, on))
      self.ic_last_status_nanos = CS.autopilot_status_nanos

    if send_leads:
      path_c0 = float(lane_overrides.get("DAS_virtualLaneC0", 0.0))
      sends.append(self.tesla_can.create_ic_leads(self._ic_lead(self.ic_radar.leadOne, path_c0),
                                                  self._ic_lead(self.ic_radar.leadTwo, path_c0)))
    return sends

  def update_cluster_speed(self, CC, CS):
    """Nudge the DI's own setpoint toward the speed openpilot is targeting, via the stalk.

    One step per press, largest step that does not overshoot, and a wait afterwards long enough
    for the DI to act and report back -- it took about 0.2s in the logs. Steps are +/-1 and +/-5,
    and both numbers live on the cluster's own whole-unit grid, so the loop finishes on an exact
    match rather than settling for close.
    """
    # The ceiling for this road, not the moment's target. MAX answers "how fast may this car go
    # here" -- a ramp, a school zone, a hairpin -- and the driver reads it as the bound cruise
    # will not cross. Chasing the slewed target instead would have the number follow the car's
    # own speed up and down every second, which says nothing about the road and was the surface
    # the eco offset walked the setpoint up.
    #
    # Zero means the map has no opinion here: leave the stalk alone rather than drive it to a
    # number nothing decided.
    # Compare on the grid the cluster actually shows. Both numbers are whole units there and the
    # stalk only moves in whole steps, but the ceiling reaches here as a Float32 that has been
    # through m/s -> km/h -> m/s: a 40 mph limit arrives as 40.0000025. Against float thresholds
    # that lands on the wrong side of both of them -- `error <= -5.0` misses by 2.5e-6 so the
    # 5-step never fires, and then `abs(error) < 1.0` is satisfied one short of the target, so
    # MAX parks one unit high and stays there. Measured on a 35 mph road: 55 crawled down to 41
    # a unit at a time and sat there for 16 s. Rounding first makes the error an exact integer
    # and both questions unambiguous.
    to_display = CV.MS_TO_KPH if CS.speed_units == "KPH" else CV.MS_TO_MPH
    target = round(CC.hudControl.cruiseCeiling * to_display)
    current = round(CS.out.cruiseState.speed * to_display)
    if target <= 0 or current <= 0:
      return []

    if self.stalk_press_frames > 0:
      self.stalk_press_frames -= 1
      return [self.tesla_can.create_stalk_command(CS.stw_actn, self.stalk_lever)]

    if self.stalk_wait_frames > 0:
      self.stalk_wait_frames -= 1
      # Release. The DI reads the press on this edge, so it has to be sent, not just skipped.
      if self.stalk_wait_frames == STALK_WAIT_FRAMES - 1:
        return [self.tesla_can.create_stalk_command(CS.stw_actn, StalkLever.IDLE)]
      return []

    error = target - current
    if error == 0:
      return []

    if error >= 5:
      self.stalk_lever = StalkLever.UP_5
    elif error > 0:
      self.stalk_lever = StalkLever.UP_1
    elif error <= -5:
      self.stalk_lever = StalkLever.DOWN_5
    else:
      self.stalk_lever = StalkLever.DOWN_1

    self.stalk_press_frames = STALK_PRESS_FRAMES
    self.stalk_wait_frames = STALK_WAIT_FRAMES
    return [self.tesla_can.create_stalk_command(CS.stw_actn, self.stalk_lever)]

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    # Cooperative steering shifts the commanded angle toward the driver instead of relying on
    # this gate, so a takeover should never get as far as handsOnLevel 3. This stays as the
    # backstop for one that does.
    lat_active = CC.latActive and CS.hands_on_level < 3 and not CS.high_angle_rate_safety

    # Stock autopark drives the car through the same two message ids openpilot uses. Panda opens
    # the forwarding gate for the maneuver, but that is not enough on its own: openpilot sends
    # DAS_control at 25Hz and DAS_steeringControl at 50Hz whether or not it is engaged, so the
    # car would receive both modules' frames interleaved on one arbitration id, with two
    # independent counters. That is what aborted the recorded attempt. Go silent instead.
    yield_to_stock_autopark = CS.stock_autopark_frames > 0 and not CC.enabled
    if yield_to_stock_autopark:
      self.apply_angle_last = CS.out.steeringAngleDeg  # resume from where the stock module left it
      self.frame += 1
      return actuators.as_builder(), []

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      angle_cmd = self.apply_angle_last
      if self.coop_steering:
        # shifts the target by the driver's torque instead of dropping lateral, and resumes from
        # the measured angle with a ramped rate limit
        angle_cmd = self.coop_steer.update(self.apply_angle_last, lat_active, CS, self.VM).steeringAngleDeg

      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, angle_cmd, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(angle_cmd, lat_active))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))

    # Never cancel cruise openpilot could not have been driving anyway. The stock park module
    # borrows the ACC channel, so the car reports cruise enabled as soon as autopark starts and
    # controlsd asks to cancel it; sending that is what ended the recorded maneuver. Out of D
    # while disengaged is the same case more generally -- there is nothing to take over from.
    cancel = CC.cruiseControl.cancel and not (
      CS.stock_autopark_offered or
      (not CC.enabled and CS.out.gearShifter != structs.CarState.GearShifter.drive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, self.CP.minAccel, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.out.gasPressed,
                                                                     CC.hudControl.setSpeed))

    elif self.CP.carFingerprint not in LEGACY_CARS:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, CS.out.gasPressed))
    # Legacy cars with stock ACC: the factory module owns DAS_control and panda forwards its
    # frames straight through, so openpilot must stay off the id entirely -- not even a cancel.
    # Interleaving two counters on it is what the car reads as a fault, taking TACC and Autopilot
    # down with it. Cancelling is the driver's job here, via the stalk.

    # Tesla Unity-style IC integration: replace AP-side 0x239/0x399 with copies that keep the
    # factory rolling counters but carry openpilot's path/AP-active visualization. 0x309 stays
    # additive, matching the existing cluster-object workaround and preserving factory side cars.
    if self.CP.flags & TeslaFlags.IC_INTEGRATION:
      can_sends += self.update_ic(CC, CS)

    # Put the cars back on the cluster. Tesla's 2026.26.1 update left AP1 clusters drawing TRUCK
    # and MOTORCYCLE but not CAR, and CAR is most of the traffic, so the display went nearly
    # empty. Nothing upstream is wrong -- the factory classifies and transmits correctly -- so the
    # repair is to hand the cluster its own objects back under a type it still renders.
    #
    # Additive rather than a replacement: the factory keeps sending its copy, and only the groups
    # holding a car are re-sent. That keeps the road signs and heading groups, which cannot be
    # Cluster MAX speed. DAS_setSpeed does not reach that display -- the DI owns it, publishes it
    # as DI_state.DI_digitalSpeed, and only the cruise stalk moves it. So openpilot presses the
    # stalk the way a driver would, one step at a time, until the number matches what it is
    # actually driving to. The flag is read here rather than cached in
    # __init__, because card.py folds the params into CP.flags only after building the interface.
    if (bool(self.CP.flags & TeslaFlags.SYNC_CLUSTER_SPEED) and self.CP.carFingerprint in LEGACY_CARS
        and CC.enabled and CC.longActive and CS.stw_actn is not None):
      can_sends += self.update_cluster_speed(CC, CS)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
