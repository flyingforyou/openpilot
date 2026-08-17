"""
Curve-aware longitudinal control (feedforward profile + backward pass).

Builds a full speed profile over the distance ahead from the model's predicted curvature and
back-propagates a comfortable deceleration, so the car slows *early*, holds a steady speed through
the curve, and powers out, keeping lateral acceleration near a chosen budget.

Vision only: the model horizon (~100-150 m at road speed) is well beyond the lookahead a curve
needs, so there is no map dependency. Nothing here is car specific -- the budget is a comfort
number, not a vehicle limit -- so it runs on every car and is always enabled. It can only ever
lower the cruise target, never raise it.

Pairs with LateralLoadGovernor (lateral_load_governor.py), the reactive backstop that keeps the
*measured* lateral load inside the steering's real limit when this feedforward under-reads a curve.
"""
from enum import IntEnum

import numpy as np

import cereal.messaging as messaging
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.curve_speed import MIN_V, V_UNSET

T_IDXS = np.array(ModelConstants.T_IDXS)  # 33-pt model horizon, 0..10 s (quadratic spacing)

# Lateral-accel budget the profile plans to. This is the "feel" knob -- how hard the car loads the
# steering in curves -- and is deliberately kept a margin below MAX_LATERAL_ACCEL_NO_ROLL (3.0),
# the curvature clip openpilot itself enforces, so the governor has room to back it off further.
A_LAT_MAX = 2.2         # m/s^2
A_DECEL = 1.8           # m/s^2, comfortable decel for the backward pass (braking starts early)
A_ACCEL = 1.2           # m/s^2, comfortable accel-out cap (seed value only; the MPC does the tracking)
V_TARGET_FLOOR = 2.0    # m/s, never command a crawl below this
KAPPA_FLOOR = 1e-4      # 1/m, ignore curvature below this (treat as straight)


class CurveState(IntEnum):
  disabled = 0   # longitudinal not engaged, or driver overriding
  cruise = 1     # no constraining curve ahead
  slowing = 2    # decelerating ahead of a curve (feedforward backward pass)
  curve = 3      # holding the curve speed through the turn


class CurveSpeedController:
  def __init__(self):
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.state = CurveState.disabled

    self.v_ego = 0.
    self.a_ego = 0.
    self.v_cruise_setpoint = V_UNSET
    self.v_target = V_UNSET
    self.a_lat_max = A_LAT_MAX

    self.output_v_target = V_UNSET
    self.output_a_target = 0.

  def _plan(self, sm: messaging.SubMaster) -> None:
    """Build kappa(s) ahead from the model, cap speed by a_lat_max, back-propagate a_decel, emit v_target."""
    vel = np.asarray(sm['modelV2'].velocity.x, dtype=np.float64)
    yaw = np.abs(np.asarray(sm['modelV2'].orientationRate.z, dtype=np.float64))
    if len(vel) != len(T_IDXS) or len(yaw) != len(T_IDXS):
      self.v_target = V_UNSET
      return

    # distance to each horizon point: trapezoidal integral of predicted speed over T_IDXS
    s = np.concatenate([[0.0], np.cumsum(0.5 * (vel[:-1] + vel[1:]) * np.diff(T_IDXS))])
    kappa = yaw / np.maximum(vel, 0.1)
    kappa = np.maximum(kappa, KAPPA_FLOOR)

    v_allow = np.minimum(np.sqrt(self.a_lat_max / kappa), self.v_cruise_setpoint)

    # backward pass (far -> near): only let speed drop as fast as a_decel allows -> brake early, smooth entry
    for i in range(len(v_allow) - 2, -1, -1):
      ds = s[i + 1] - s[i]
      if ds <= 0:
        continue
      v_allow[i] = min(v_allow[i], float(np.sqrt(v_allow[i + 1] ** 2 + 2.0 * A_DECEL * ds)))

    self.v_target = max(float(v_allow[0]), V_TARGET_FLOOR)

  def _update_state(self) -> tuple[bool, bool]:
    enabled = self.long_enabled and not self.long_override
    constraining = enabled and self.v_ego > MIN_V and self.v_target < self.v_cruise_setpoint - 0.5
    if not self.long_enabled:
      self.state = CurveState.disabled
    elif self.long_override:
      self.state = CurveState.disabled  # manual override; let the driver have it
    elif not constraining:
      self.state = CurveState.cruise
    elif self.v_target < self.v_ego - 0.5:
      self.state = CurveState.slowing
    else:
      self.state = CurveState.curve
    active = self.state in (CurveState.slowing, CurveState.curve)
    return enabled, active

  def get_v_target_from_control(self) -> float:
    return self.v_target if self.is_active else V_UNSET

  def get_a_target_from_control(self) -> float:
    if self.is_active:
      return float(np.clip((self.v_target - self.v_ego) / 2.0, -A_DECEL, A_ACCEL))
    return self.a_ego

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise if v_cruise > 0 else V_UNSET

    self.a_lat_max = A_LAT_MAX
    if self.long_enabled:
      self._plan(sm)
    else:
      self.v_target = V_UNSET

    self.is_enabled, self.is_active = self._update_state()
    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()
