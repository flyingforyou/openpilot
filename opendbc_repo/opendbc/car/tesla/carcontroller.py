import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.car.tesla.das_object import LEAD_VEHICLES
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR, StalkLever, TeslaFlags
from opendbc.car.vehicle_model import VehicleModel

# Same distance-only match radard.match_das_objects uses (DAS_MATCH_MAX_DREL there): the factory
# camera and this car's radar agree on distance to well under a metre and disagree on lateral
# position (see that function's docstring in radard.py), so dx alone is what says whether the
# factory's own group-0 stream -- the one Unity's IC also reads -- already has an object for a
# radar lead openpilot is about to inject.
IC_LEAD_MATCH_MAX_DREL = 2.0

# How much of the distance past TeslaICLeadMaxM survives on the cluster. 0.25 keeps a far lead
# visibly approaching (80 m stays 80, 100 -> 85, 120 -> 90) instead of parking it on the threshold.
IC_LEAD_FAR_COMPRESS = 0.25


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
    # No ic_model: the lane-change state the cluster needs comes off CarControl's blinkers instead
    # of modelV2, so nothing here has to be plumbed by card. See update_ic.
    # Floor for the braking jerk handed to the DI, m/s^3; 0 keeps the full JERK_LIMIT_MIN.
    # Set by card from TeslaBrakeJerk so it can be changed without a rebuild.
    self.brake_jerk_base = 0.0
    # Ceiling on that authority, m/s^3; 0 = the full JERK_LIMIT_MIN. Set by card from
    # TeslaBrakeJerkMax. This one does limit real hard braking -- see _brake_jerk_limit.
    self.brake_jerk_ceiling = 0.0
    # Metres to pull a distant lead in to on the cluster only; 0 leaves it truthful.
    # Set by card from TeslaICLeadMaxM. See _ic_lead.
    self.ic_lead_display_max = 0.0
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

  @staticmethod
  def _ic_lead(lead, path_c0, display_max=0.0):
    """openpilot's lead as the cluster's group-0 object.

    display_max pulls a distant lead in so the cluster will draw it. Nothing about the field or
    the factory forces this: DAS_objVehDx carries 127.5 m and the factory itself routinely
    publishes objects out to 124.5 m with RelevantForControl set, so the cull is a rule inside the
    IC that the bus does not expose. Clamping and watching is the only way to find where it is.

    This deliberately reports a car as nearer than it is, which is a lie to the driver -- a
    conservative one (it can only ever show a lead closer, never further) on a display-only
    message, and off unless asked for. Distances below the threshold are untouched.
    """
    if lead is None or not lead.present:
      return None
    d_rel = float(np.clip(lead.dRel, 0.0, 126.0))
    if display_max > 0.0 and d_rel > display_max:
      # Compressed, not clamped. A hard min() pinned every far lead to exactly the threshold --
      # measured over a drive at 80 m, all 67 injected objects came out at 80.0 m -- so the car sat
      # frozen there until it came inside the threshold and only then started moving. Squeezing the
      # tail instead keeps it approaching, which is the part that reads as real.
      d_rel = min(display_max + (d_rel - display_max) * IC_LEAD_FAR_COMPRESS, 126.0)
    return (
      d_rel,
      float(np.clip(int(lead.vRel), -30, 26)),
      float(np.clip(path_c0 - lead.yRel, -22.05, 22.4)),
    )

  @staticmethod
  def _factory_already_shows(das_vehicles, dRel):
    """Is the factory's own group-0 (LEAD_VEHICLES) object stream already carrying a car near
    this radar lead? DAS_object is never blocked -- see tesla_legacy.h's TX message comment on
    why not, it stays additive because openpilot also reads it for cut-in control -- so whatever
    openpilot injects sits on the bus alongside whatever the factory is already sending. Without
    this, the same physical car shows up as the factory's own object and openpilot's injected one
    at once, i.e. exactly the two-car/flicker symptom this exists to prevent.
    """
    return any(group == LEAD_VEHICLES and abs(veh.dx - dRel) < IC_LEAD_MATCH_MAX_DREL
              for (group, _obj_id), (_age, veh) in das_vehicles.items())

  def update_ic(self, CC, CS):
    if not (self.CP.flags & TeslaFlags.IC_INTEGRATION) or not CC.enabled:
      return []
    if self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
      return []

    # Panda always blocks the factory 0x239/0x399 while engaged, so openpilot must always re-send
    # them to keep the cluster alive. DAS_lanes goes out as a pure clone of the factory's own last
    # frame -- untouched, not fitted from our model -- on the theory that the factory's own lane
    # data is fine on its own and AutopilotStatus is the only thing gating whether the cluster
    # draws it: a same-road TACC-vs-Active_nominal comparison found DAS_lanes content barely
    # differs between the two, while AutopilotStatus is the one clean, always-binary signal.
    # ic_enabled decides only whether status/leads carry openpilot's data, and it is a live switch
    # so it can be flipped mid-drive from /live.
    on = self.ic_enabled
    sends = []
    new_lane_frame = CS.das_lanes is not None and CS.das_lanes_nanos != self.ic_last_lanes_nanos
    send_leads = on and self.frame % 10 == 0 and self.ic_radar is not None

    if new_lane_frame:
      sends.append(self.tesla_can.create_ic_lanes(CS.das_lanes, {}))
      self.ic_last_lanes_nanos = CS.das_lanes_nanos

    if CS.autopilot_status is not None and CS.autopilot_status_nanos != self.ic_last_status_nanos:
      # autopilotStatus stays pinned at Active while on, full stop -- flipping it to AVAILABLE(2)
      # whenever one lane line dropped out (mirroring stock's own coupling) made our own status
      # flicker 2/3 continuously, since DAS_lanes here is a pure clone of whatever the factory's
      # perception momentarily reports. The outer-line fade belongs to DAS_leftLaneExists/
      # DAS_rightLaneExists themselves (see create_ic_lanes, already a pure clone of that), not to
      # AutopilotStatus. Drive the dashed crossed-line animation from openpilot's own lane change.
      # Dashed crossed line during a lane change. A stock capture (0000009f, seg 1 t+56.5s) shows
      # the factory doing exactly this: DAS_autoLaneChangeState 7 -> 9 (ALC_IN_PROGRESS_L) held for
      # the ~6 s of the manoeuvre with the left blinker on, autopilotStatus staying 3 and
      # handsOnState staying 2 throughout -- so only this one field moves.
      #
      # Driven off CarControl's blinkers rather than modelV2: controlsd sets them only while a lane
      # change is running and puts the direction in them (see its "Enable blinkers while lane
      # changing"), CarControl is already subscribed by card, and a fresh CarControl each cycle
      # means they cannot latch. Reading modelV2 instead would mean deserialising the model message
      # inside the 100Hz control loop for one enum -- which is why that plumbing was dropped.
      #
      # Known difference from stock: openpilot's blinkers go true at preLaneChange, so the line
      # goes dashed when the driver signals rather than when the car starts moving over.
      lane_change = -1 if CC.leftBlinker else 1 if CC.rightBlinker else 0
      sends.append(self.tesla_can.create_ic_status(CS.autopilot_status, on, CS.hands_on_level, lane_change))
      self.ic_last_status_nanos = CS.autopilot_status_nanos

    if send_leads:
      path_c0 = float((CS.das_lanes or {}).get("DAS_virtualLaneC0", 0.0))
      # Only the primary lead is drawn. leadTwo is a second in-path car at a different distance,
      # and injecting it too drew the lead as two cars through a lead handoff -- the closer one
      # white, the farther one grey -- which read as the lead doubling as it switched.
      lead1 = self.ic_radar.leadOne
      if lead1.present and self._factory_already_shows(CS.das_vehicles, lead1.dRel):
        lead1 = None
      # Say nothing rather than saying "no object". DAS_object is shared -- the factory keeps
      # publishing its own objects on this id at ~31Hz and is never blocked -- so an empty frame
      # from us is not neutral, it actively contradicts a car the factory is drawing. Over a drive
      # 96.9% of our frames (2090/2157) were empty ones sent straight into that stream. We only
      # have something to add when we hold a lead the factory is not already showing.
      injected = self._ic_lead(lead1, path_c0, self.ic_lead_display_max)
      if injected is not None:
        sends.append(self.tesla_can.create_ic_leads(injected, None))
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
    # Experiment: let a double-stroke reach genuine Active_nominal on its own -- our own torque
    # never appears until it does, so nothing of ours is present for the AP1 computer's activation
    # check to react to. See WAIT_FOR_STOCK_AP in values.py.
    if self.CP.flags & TeslaFlags.WAIT_FOR_STOCK_AP:
      lat_active = lat_active and CS.genuine_ap_active

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
                                                                     CC.hudControl.setSpeed, self.brake_jerk_base,
                                                                     self.brake_jerk_ceiling))

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
