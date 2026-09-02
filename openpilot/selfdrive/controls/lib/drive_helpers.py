import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MIN_STABLE_DELAY = 0.3

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
# Raised from the 3.0 EU comfort figure to match the two limits downstream of it -- the Tesla
# carcontroller's own MAX_LATERAL_ACCEL (values.py) and panda's steer_angle_cmd_checks_vm
# (safety/lateral.h) are both already 5.0. At 3.0 this clamp was the only one that ever bound:
# across 844 segments of real driving, all 76 "Turn Exceeds Steering Limit" alerts came from
# curvature_limited here, and none from the carcontroller (max |requested-emitted| was 0.18 deg).
# The car undershot the curve, the driver added steering to make up the difference, and the
# resulting torque tripped handsOnLevel 3 -- which on this platform disengages cruise as well.
# Peak demand in those 76 episodes: median 4.29, p90 4.84, max 5.32 m/s^2; 5.0 covers 73 of them.
# Above 5.0 panda starts clipping instead, which is a worse failure than this one.
MAX_LATERAL_ACCEL_NO_ROLL = 5.0  # m/s^2


def should_stop(v_ego: float, a_target: float) -> bool:
  return bool(v_ego < 0.3 and a_target < 0.1)

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val


# Optional lateral smoothing (modeld's LatSmoothSec), ported from CarrotPilot. Upstream already
# runs this same low-pass on the longitudinal action (LONG_SMOOTH_SECONDS = 0.3) and leaves the
# lateral one at 0.0; carrot turns the lateral one on and adds an extra term for when the model is
# unsure where the path is. See modeld.py for why the low-speed wheel shake needs it.
#
# Carrot reads the uncertainty as plan_stds[0, 10, Plan.POSITION, 1] -- four indices into a 3-D
# (batch, IDX_N, PLAN_WIDTH) array, which raises IndexError, gets swallowed by its own try/except,
# and leaves the extra silently always zero. Indexed correctly here: Plan.POSITION is x,y,z at
# 0,1,2, so the lateral std is feature 1.
LAT_SMOOTH_SECONDS_MAX = 0.60          # ceiling on the total, as carrot has
LAT_SMOOTH_Y_STD_RANGE = (0.15, 0.25)  # m of 1s lateral std over which the extra ramps in
LAT_SMOOTH_T_IDX_1S = 10               # ModelConstants.T_IDXS[10] = 0.977 s


def get_lat_smooth_seconds_dynamic(model_output, base: float) -> tuple[float, float]:
  """Base lateral smoothing, plus more of it while the model is unsure where the path is.

  Returns (tau, y_std_1s). base <= 0 disables the feature entirely, which is the default.
  """
  if base <= 0.0:
    return 0.0, 0.0
  try:
    y_std_1s = float(model_output['plan_stds'][0, LAT_SMOOTH_T_IDX_1S, 1])
  except (KeyError, IndexError, TypeError):
    # Fall back to the plain base rather than silently disabling what the driver asked for.
    return base, 0.0
  if not np.isfinite(y_std_1s):
    return base, 0.0
  extra = float(np.interp(y_std_1s, LAT_SMOOTH_Y_STD_RANGE, [0.0, base * 2.0]))
  return float(np.clip(base + extra, 0.0, LAT_SMOOTH_SECONDS_MAX)), y_std_1s

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
  else:
    a_target = 0.0
  return a_target

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
