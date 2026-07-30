"""
Reactive lateral-load governor -- the closed-loop backstop under the curve-speed feedforward.

The feedforward (curve_speed_controller.py) plans from *predicted* curvature; it is blind to what
the steering is actually doing. This governor watches the real lateral state
(controlsState.lateralControlState, carState.yawRate) and:

  1. caps speed to keep the *measured* lateral load near a fraction of the steering's limit, which
     catches feedforward under-reads and hot curve entries;
  2. provides a throttle-fade scale so the planner will not add throttle while the steering is at
     or near its limit -- the "don't accelerate into a saturated steer" interlock -- hard-zeroed
     while the car is actually running wide.

Reactive control cannot undo a brief peak mid-curve (a_lat is proportional to v^2), so the
feedforward stays the primary edge-holder and this is the backstop.

Car independent by construction: the ceiling is MAX_LATERAL_ACCEL_NO_ROLL, the curvature clip
openpilot already enforces for every car, and the running-wide signal comes from the active lateral
controller's own saturation flag rather than any per-brand tuning.
"""
import numpy as np

import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL
from openpilot.selfdrive.controls.lib.curve_speed import MIN_V, V_UNSET

A_LAT_CEILING = MAX_LATERAL_ACCEL_NO_ROLL  # m/s^2, the lateral accel openpilot clips desired curvature to
SETPOINT = 0.85         # regulate measured load to this fraction of the ceiling; riding right at the
                        # limit overshoots it given reactive lag, so back off earlier
TAPER_START = 0.70      # fade feedforward throttle from full (here) to zero at the ceiling
LOAD_LP = 0.4           # EMA on the load signal (curvature/torque noise rejection)
UNDERSTEER_TH = 0.05    # m/s^2 of desired-minus-actual lateral accel that counts as "running wide"
V_TARGET_FLOOR = 2.0    # m/s


class LateralLoadGovernor:
  def __init__(self):
    self.ceiling = A_LAT_CEILING
    self.enabled = True    # feature toggle; set False to disable the reactive backstop entirely

    self.long_enabled = False
    self.long_override = False
    self.is_active = False
    self.load = 0.
    self.saturated = False
    self.running_wide = False
    self.output_v_target = V_UNSET

  def _measure(self, sm: messaging.SubMaster, v_ego: float) -> tuple[float, bool]:
    """Measured lateral load from the active lateral controller (+ yaw-rate fallback), plus the
    'running wide' flag for the interlock."""
    lcs = sm['controlsState'].lateralControlState
    sub = getattr(lcs, lcs.which())

    saturated = bool(getattr(sub, 'saturated', False))
    actual = abs(float(getattr(sub, 'actualLateralAccel', 0.0)))
    desired = abs(float(getattr(sub, 'desiredLateralAccel', 0.0)))
    a_lat = max(actual, abs(v_ego * sm['carState'].yawRate))  # what the car actually feels

    # 'running wide' = the steering cannot hold the line. Torque control reports it directly as
    # desired > actual lateral accel; the other controllers publish no lateral-accel pair, so their
    # own saturation flag (angle can't track / curvature clipped) IS the running-wide signal.
    if hasattr(sub, 'actualLateralAccel'):
      self.running_wide = saturated and (desired - actual) > UNDERSTEER_TH
    else:
      self.running_wide = saturated

    return a_lat, saturated

  def set_enabled(self, enabled: bool) -> None:
    self.enabled = enabled

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override

    if not self.enabled or not long_enabled or long_override:
      self.is_active = False
      self.output_v_target = V_UNSET
      self.load = 0.
      self.saturated = False
      self.running_wide = False
      return

    a_lat, self.saturated = self._measure(sm, v_ego)
    load = a_lat / max(self.ceiling, 0.1)
    self.load += LOAD_LP * (load - self.load)  # EMA

    if self.load > SETPOINT and v_ego > MIN_V:
      # speed that brings the measured load back to the setpoint (a_lat ~ v^2 -> v_cap = v*sqrt(setpoint/load))
      v_cap = v_ego * float(np.sqrt(SETPOINT / max(self.load, 1e-3)))
      self.output_v_target = max(v_cap, V_TARGET_FLOOR)
      self.is_active = True
    else:
      self.output_v_target = V_UNSET
      self.is_active = False

  def throttle_scale(self) -> float:
    """[0, 1] multiplier the planner applies to any *positive* a_target: fade throttle as load nears
    the limit, hard-zero while actually running wide."""
    if not self.enabled or not self.long_enabled or self.long_override:
      return 1.0
    if self.running_wide:
      return 0.0
    return float(np.clip((1.0 - self.load) / max(1.0 - TAPER_START, 1e-3), 0.0, 1.0))
