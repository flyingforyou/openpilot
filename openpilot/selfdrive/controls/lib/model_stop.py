"""The model-stopping inference, as a pure function.

Lifted out of CarrotPlanner.check_model_stopping so it can be exercised without a running
Params -- carrot_functions pulls in libparams_c.so, which only exists on the device. The caller
keeps the state (the velocity filter, xState) and the counting; this is just the predicate.
"""
import numpy as np

# Above this the inference is off entirely. Upstream carrot uses 82, which sits just under the
# 55 mph (88.5 km/h) this car cruises the city at, so the one detector that can see a stop coming
# *before* a lead exists was disabled for exactly the driving that needs it -- see the note in
# check_model_stopping for the route-000000f7 measurement and the ten-route false-positive check.
MODEL_STOP_MAX_KPH = 89.0

# Under this the car is essentially stopped and only the short-range form applies.
MODEL_STOP_CRAWL_KPH = 1.0

CRAWL_MAX_X = 20.0
CRAWL_MAX_V = 10.0

# How far out a path end is still allowed to mean "stop", against the speed the path starts at.
PATH_END_BP = [60.0, 80.0]
PATH_END_V = [120.0, 150.0]

# The path has to end clear of the lead by this much, so a lead being tracked is not itself read
# as the stop. With no lead the caller passes a large d_rel and the term falls away.
LEAD_MARGIN = 3.0

# Path end must be roughly straight ahead; a path curving off this far is a bend, not a stop.
MAX_Y_OFFSET = 5.0


def model_stop_sign(v_ego_kph: float, model_x: float, model_v: float, v_path_0: float,
                    y_last: float, d_rel: float, decel_suppress: bool) -> bool:
  """True when the model's own path says a stop is coming.

  model_x/model_v are the end of the predicted path; v_path_0 is its start speed. d_rel is the
  lead distance, or a large number when there is no lead. decel_suppress carries the caller's
  "already braking under e2e cruise" condition, where this misfires often enough to be worth
  ignoring.
  """
  if v_ego_kph < MODEL_STOP_CRAWL_KPH:
    return model_x < CRAWL_MAX_X and model_v < CRAWL_MAX_V

  if v_ego_kph >= MODEL_STOP_MAX_KPH:
    return False

  stop_sign = (model_x < d_rel - LEAD_MARGIN and
               model_x < np.interp(v_path_0 * 3.6, PATH_END_BP, PATH_END_V) and
               ((model_v < 3.0) or (model_v < v_path_0 * 0.7)) and
               abs(y_last) < MAX_Y_OFFSET)

  return bool(stop_sign and not decel_suppress)
