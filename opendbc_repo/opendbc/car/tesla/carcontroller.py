import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR, TeslaFlags
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    # Follow the driver's hands rather than letting go of the wheel. Off unless opted in.
    self.coop_steering = bool(CP.flags & TeslaFlags.COOP_STEER) and CP.carFingerprint in LEGACY_CARS
    self.coop_steer = CoopSteeringCarController()
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
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.out.gasPressed))

    elif self.CP.carFingerprint not in LEGACY_CARS:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, CS.out.gasPressed))
    # Legacy cars with stock ACC: the factory module owns DAS_control and panda forwards its
    # frames straight through, so openpilot must stay off the id entirely -- not even a cancel.
    # Interleaving two counters on it is what the car reads as a fault, taking TACC and Autopilot
    # down with it. Cancelling is the driver's job here, via the stalk.

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
