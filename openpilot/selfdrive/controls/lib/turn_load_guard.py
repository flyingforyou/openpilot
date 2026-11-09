"""Keep tight turns under the lateral load that latches an EPAS fault.

On 11/02 (0237 seg 18) a 40 km/h intersection turn reached 5.09 m/s^2 of lateral acceleration and
the EPAS latched EAC_ERROR_TMP_FAULT until the car was restarted (see the 4.6-4.8 m/s^2 threshold
measured in 10/21). Closed loop, the hardest turns since 10/31 still peak at 4.4-4.5. The curve cap
in map_cruise sets a cruise target from the model's curvature; this is the backstop for the turns
where the plan still carries too much speed into the bend.

The idea is upstream's CurveSpeed (PR #39242, vendored as curve_speed.py and logged in shadow): find
the extra deceleration needed so the plan meets each point's allowed speed in time, filter it, rate-
limit it, take it off the command. Two differences, both because this runs on our own planner's
output rather than the model's e2e accel:
- The speeds checked are our MPC's plan, not the model's: the model's plan already slows for the
  bend, ours may not, and subtracting a correction sized to the model's plan from a command that is
  already braking toward map_cruise's curve target would brake twice.
- Our plan and the model's travel at different speeds, so the model's curvature is looked up by
  distance along the path, not by time.
And the budget is LAT_BUDGET rather than upstream's 2.8, so ordinary curves are left to map_cruise.

The plan is re-anchored at the car's actual speed and capped by holding the command it is about to
get. The MPC never sees this correction, so its plan keeps the speed the car would have had without
it: on 11/09 (025c seg 19, a 37 m-radius left turn entered at 60 km/h with no lead) the plan 1 s out
stayed ~10 km/h above the car the whole way in, the need never fell, the correction wound up to
6.75 and held the command at -2.0 after the car was already at the model's allowed 38 km/h -- and
the driver pressed the accelerator. The correction is also held to what the output can still apply.
Closed loop over the 29 engaged tight turns since 10/31 (guard off / this / before): 0244 seg 16 peaks
at 5.26 / 4.05 / 4.03 m/s^2; 025c's turn bottoms out at 35.2 / 34.1 / 30.4 km/h; 0237 seg 4 at
60.0 / 54.3 / 48.0; the rest move by under 1 km/h.

Closed loop over the 15 tight turns since 10/31 (window from 8 s before the exit): the one that
peaked at 4.99 m/s^2 (0244 seg 16) came down to 3.98 at 3.5 (4.21 at 3.8, 3.63 at 3.2); the median
peak 2.77 -> 2.65; acceleration out of the turn unchanged. The 0237 turn that latched the fault no
longer re-accelerates into its second bend with today's planner (3.61 either way) -- the 5.4 that
appears later in that window is the wheel unwinding after the fault, not a command. Over the 50
highway lane-change windows it never moved the command, and follow_regression is unchanged.
"""
import numpy as np

from opendbc.car.lateral import AVERAGE_ROAD_ROLL
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter

LAT_BUDGET = 3.5         # m/s^2
MIN_T = 0.5              # s; inside this the bend is already the turn-accel cap's
MAX_T = 6.0
MIN_ACCEL = -2.0         # m/s^2, the most this may ask for
MAX_JERK = 2.0           # m/s^3, on the correction
COAST_CORRECTION = 0.3   # m/s^2 of correction at which acceleration is fully withheld


def required_decel(t_plan, v_plan, v_ego, a_cmd, model_x, model_v, model_yaw_rate, roll):
  """Extra deceleration (>= 0) beyond a_cmd needed to keep every point within LAT_BUDGET.

  The speeds checked are the plan's changes from now applied to v_ego, never above holding a_cmd."""
  t_plan, v_plan = (np.asarray(a, dtype=float) for a in (t_plan, v_plan))
  model_x, model_v, model_yaw_rate = (np.asarray(a, dtype=float) for a in (model_x, model_v, model_yaw_rate))
  if len(model_x) < 2 or len(t_plan) < 2 or not (len(model_x) == len(model_v) == len(model_yaw_rate)):
    return 0.0
  v_plan = np.maximum(np.minimum(v_ego + v_plan - v_plan[0], v_ego + a_cmd * t_plan), 0.0)
  x_plan = np.r_[0.0, np.cumsum((v_plan[1:] + v_plan[:-1]) / 2.0 * np.diff(t_plan))]
  ahead = (t_plan > MIN_T) & (t_plan < MAX_T)
  if not ahead.any():
    return 0.0
  curv_model = np.abs(model_yaw_rate) / np.maximum(model_v, 1.0)
  # distance along the path our plan has reached -> the model's curvature there
  dist = x_plan[ahead] - x_plan[0]
  curvature = np.interp(dist, model_x - model_x[0], curv_model)
  side = np.sign(np.interp(dist, model_x - model_x[0], model_yaw_rate))
  bank = np.minimum(roll * side, AVERAGE_ROAD_ROLL)
  v_max = np.sqrt(np.maximum(LAT_BUDGET + ACCELERATION_DUE_TO_GRAVITY * bank, 0.1) / np.maximum(curvature, 1e-4))
  return float(max(np.max((v_plan[ahead] - v_max) / t_plan[ahead], initial=0.0), 0.0))


class TurnLoadGuard:
  def __init__(self, dt):
    self.dt = dt
    self.gap = FirstOrderFilter(0.0, 0.2, dt)
    self.correction = 0.0

  def reset(self):
    self.gap.x = 0.0
    self.correction = 0.0

  def update(self, a_target, need):
    max_step = MAX_JERK * self.dt
    self.correction = float(np.clip(self.gap.update(need), self.correction - max_step, self.correction + max_step))
    # anything past what the output can still apply would only have to unwind later
    self.correction = min(self.correction, max(a_target - MIN_ACCEL, 0.0))
    self.gap.x = min(self.gap.x, self.correction + max_step)
    if self.correction <= 0.0:
      return a_target
    coast = max(a_target, 0.0) * np.interp(self.correction, [0.0, COAST_CORRECTION], [1.0, 0.0]) + min(a_target, 0.0)
    return min(a_target, coast, max(a_target - self.correction, MIN_ACCEL))
