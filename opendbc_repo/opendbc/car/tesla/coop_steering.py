"""Cooperative steering: follow the driver's hands instead of letting go of the wheel.

Ported from CarrotPilot's Tesla port, which built this for HW3's EPAS3S. The idea is the part
worth keeping: rather than dropping lateral when the driver pushes and grabbing the wheel back
afterwards, openpilot shifts its own angle target by the driver's torque so the two steer
together. The driver never has to fight, so the torque never builds to the point where the EPS
inhibits -- which is what broke the pause-and-resume approach. A recorded attempt there had
openpilot re-assert control 0.28s after the override eased, and the car's DI faulted 0.12s later
and stayed faulted until a power cycle.

Prevention, not a new failure path: if the driver does overpower this and handsOnLevel still
reaches 3, steeringDisengage fires exactly as before and openpilot drops out the well-tested way.

Thresholds re-measured on this car rather than inherited, since HW1 is a different EPS
generation (8615 + 35467 EPAS frames from two recorded drives):
  - hands off the wheel sits at 0.2-0.3Nm, so the 0.5Nm deadzone is clear of the noise floor
  - handsOnLevel reaches 3 at a median of 3.2Nm, so holding under 2.5Nm keeps real margin
  - but it also tripped as low as 0.42Nm on a sustained hold: handsOnLevel is time-integrated,
    not a pure threshold, so this reduces how hard and how long the driver pushes rather than
    guaranteeing the trip never happens
"""
import math
from collections import namedtuple
from dataclasses import replace

import numpy as np

from opendbc.car import DT_CTRL, rate_limit, structs
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

DT_LAT_CTRL = DT_CTRL * CarControllerParams.STEER_STEP


class CoopSteeringCarControllerParams(CarControllerParams):
  ANGLE_LIMITS = replace(CarControllerParams.ANGLE_LIMITS, MAX_ANGLE_RATE=5)


# Tesla reports a filtered steering angle, so lead it with the rate to estimate the real one
STEERING_DEG_PHASE_LEAD_COEFF = 8.0

# angle override
STEER_OVERRIDE_MIN_TORQUE = 0.5   # Nm - measured hands-off noise floor is 0.2-0.3Nm
STEER_OVERRIDE_MAX_TORQUE = 2.5   # Nm - handsOnLevel hits 3 at a median of 3.2Nm on this car
STEER_OVERRIDE_MAX_LAT_ACCEL = 1.5              # m/s^2 - like Tesla's comfort steering mode
STEER_OVERRIDE_LAT_ACCEL_GAIN_LIMIT = 10        # deg/Nm - stability at low speed

# angle ramping
STEER_OVERRIDE_MAX_LAT_JERK = 2.0               # m/s^3 - how fast the offset builds
STEER_OVERRIDE_MAX_LAT_JERK_CENTERING = CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK
STEER_OVERRIDE_LAT_JERK_GAIN_LIMIT = 100        # deg/s/Nm
STEER_OVERRIDE_TORQUE_RANGE = STEER_OVERRIDE_MAX_TORQUE - STEER_OVERRIDE_MIN_TORQUE

# on resume, ramp the allowed angle rate up from zero instead of stepping to the limit
STEER_RESUME_RATE_LIMIT_RAMP_RATE = 500  # deg/s^2

CoopSteeringData = namedtuple("CoopSteeringData", ["steeringAngleDeg", "lat_active"])


def get_steer_from_lat_accel(lat_accel: float, v_ego: float, VM: VehicleModel) -> float:
  curvature = lat_accel / (max(1, v_ego) ** 2)
  return math.degrees(VM.get_steer_from_curvature(curvature, v_ego, 0))


def apply_bounds(signal: float, limit: float) -> float:
  return float(np.clip(signal, -limit, limit))


def apply_deadzone(signal: float, deadzone: float) -> float:
  return signal - apply_bounds(signal, deadzone)


def calc_override_angle_limited(torque: float, v_ego: float, VM: VehicleModel, lat_accel: float) -> float:
  torque_to_angle = get_steer_from_lat_accel(lat_accel, v_ego, VM) / STEER_OVERRIDE_TORQUE_RANGE
  return torque * min(torque_to_angle, STEER_OVERRIDE_LAT_ACCEL_GAIN_LIMIT)


def calc_override_angle_delta_limited(torque: float, v_ego: float, VM: VehicleModel, lat_jerk: float) -> float:
  # prevents windup in the car controller's rate limiter
  lat_jerk = min(lat_jerk, CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK)

  torque_to_angle = get_steer_from_lat_accel(lat_jerk, v_ego, VM) / STEER_OVERRIDE_TORQUE_RANGE
  gain_limit = min(STEER_OVERRIDE_LAT_JERK_GAIN_LIMIT,
                   CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / DT_CTRL / STEER_OVERRIDE_TORQUE_RANGE)
  override_angle_rate = torque * min(torque_to_angle, gain_limit)

  return apply_bounds(override_angle_rate * DT_LAT_CTRL,
                      CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE)


class SteerRateLimiter:
  def __init__(self):
    self._last = 0.0

  def reset(self, angle: float) -> None:
    self._last = angle

  def update(self, angle: float, angle_delta_lim: float) -> float:
    angle_lim = rate_limit(angle, self._last, -angle_delta_lim, angle_delta_lim)
    self._last = angle_lim
    return angle_lim


class CoopSteeringCarController:
  def __init__(self):
    self.coop_apply_angle_last = 0.0
    self.coop_apply_angle_last_sat = 0.0
    self.override_angle_accu = 0.0
    self.resume_rate_limiter_delta = SteerRateLimiter()
    self.resume_rate_limiter = SteerRateLimiter()

  def apply_override_angle_direct(self, lat_active: bool, driver_torque: float, v_ego: float,
                                  VM: VehicleModel) -> float:
    """Torque maps straight to an angle offset. Dominates once there is speed to steer with."""
    if not lat_active:
      return 0.0
    torque = apply_deadzone(driver_torque, STEER_OVERRIDE_MIN_TORQUE)
    return calc_override_angle_limited(torque, v_ego, VM, STEER_OVERRIDE_MAX_LAT_ACCEL)

  def apply_override_angle_relative(self, lat_active: bool, driver_torque: float, v_ego: float,
                                    VM: VehicleModel, unwind_weight: float = 1.0) -> float:
    """Torque integrates into a held offset. What makes low speed usable, where a direct
    torque-to-angle map would need impossible gains."""
    if not lat_active:
      self.override_angle_accu = 0.0
      return 0.0

    # unwind the accumulator if the previous command saturated, so it can't wind up
    unwind = (self.coop_apply_angle_last - self.coop_apply_angle_last_sat) * unwind_weight
    if self.override_angle_accu * unwind > 0:
      unwind = apply_bounds(unwind, abs(self.override_angle_accu))
      self.override_angle_accu -= unwind

    # bias the torque so releasing the wheel feels like the steering centering itself
    if self.override_angle_accu > 0 and abs(v_ego) > 0.1:
      torque_biased = driver_torque - STEER_OVERRIDE_MIN_TORQUE
    elif self.override_angle_accu < 0 and abs(v_ego) > 0.1:
      torque_biased = driver_torque + STEER_OVERRIDE_MIN_TORQUE
    else:
      torque_biased = apply_deadzone(driver_torque, STEER_OVERRIDE_MIN_TORQUE)

    # come back to centre faster than the offset built up
    building = (torque_biased * self.override_angle_accu) > 0
    delta = calc_override_angle_delta_limited(
      torque_biased, v_ego, VM,
      STEER_OVERRIDE_MAX_LAT_JERK if building else STEER_OVERRIDE_MAX_LAT_JERK_CENTERING)

    new_accu = self.override_angle_accu + delta
    # snap to zero rather than crawl past it once the driver is back inside the deadzone
    if (new_accu * self.override_angle_accu) < 0 and abs(driver_torque) < STEER_OVERRIDE_MIN_TORQUE:
      new_accu = 0.0

    self.override_angle_accu = new_accu
    return self.override_angle_accu

  def apply_override_angle_combined(self, lat_active: bool, driver_torque: float, v_ego: float,
                                    VM: VehicleModel) -> float:
    if not lat_active:
      self.override_angle_accu = 0.0
      return 0.0

    # how much of the wanted lateral accel the direct map can actually reach at this speed
    direct_capability = (calc_override_angle_limited(STEER_OVERRIDE_TORQUE_RANGE, v_ego, VM,
                                                    STEER_OVERRIDE_MAX_LAT_ACCEL) /
                         get_steer_from_lat_accel(STEER_OVERRIDE_MAX_LAT_ACCEL, v_ego, VM))
    relative_weight = 1.0 - direct_capability

    direct = self.apply_override_angle_direct(lat_active, driver_torque, v_ego, VM)
    relative = self.apply_override_angle_relative(lat_active, driver_torque, v_ego, VM,
                                                  unwind_weight=relative_weight)
    return direct * direct_capability + relative * relative_weight

  def resume_steer_desired_rate_limit(self, lat_active: bool, apply_angle: float,
                                      steering_angle: float) -> float:
    """Start from the wheel where it actually is, and ramp the rate limit up from zero."""
    if not lat_active:
      self.resume_rate_limiter_delta.reset(0.0)
      self.resume_rate_limiter.reset(steering_angle)
      return steering_angle

    angle_rate_delta_lim = self.resume_rate_limiter_delta.update(
      CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE,
      STEER_RESUME_RATE_LIMIT_RAMP_RATE * DT_LAT_CTRL ** 2)
    return self.resume_rate_limiter.update(apply_angle, angle_rate_delta_lim)

  def update(self, apply_angle: float, lat_active: bool, CS: structs.CarState,
             VM: VehicleModel) -> CoopSteeringData:
    steering_angle_lead = CS.out.steeringAngleDeg + CS.out.steeringRateDeg / STEERING_DEG_PHASE_LEAD_COEFF

    apply_angle = self.resume_steer_desired_rate_limit(lat_active, apply_angle, steering_angle_lead)
    apply_angle += self.apply_override_angle_combined(lat_active, CS.out.steeringTorque,
                                                      CS.out.vEgo, VM)

    self.coop_apply_angle_last = apply_angle
    self.coop_apply_angle_last_sat = apply_steer_angle_limits_vm(
      apply_angle, self.coop_apply_angle_last_sat, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
      lat_active, CoopSteeringCarControllerParams, VM)

    return CoopSteeringData(self.coop_apply_angle_last_sat, lat_active)
