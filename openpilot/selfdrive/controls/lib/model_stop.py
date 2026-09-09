"""The model-stopping inference, as a pure function.

Lifted out of CarrotPlanner.check_model_stopping so it can be exercised without a running
Params -- carrot_functions pulls in libparams_c.so, which only exists on the device. The caller
keeps the state (the velocity filter, xState) and the counting; this is just the predicate.

Provenance of the constants below, since it bears on how much they should be trusted:

Every threshold here arrived fully formed and none has a tuning history. In this tree the function
came in with the planner port (b31052a59, 2026-08-16) and was not edited once before the speed gate
was raised. Against upstream carrot (ajouatom/openpilot, carrot-wip) the code is identical
character for character, call site included -- the `d_rel = 1000` fallback that lets this fire with
no lead is carrot's design, not an artefact of the port. Carrot's own history does not go back any
further: both roots (ecf4d58c "c4-v1" and bd2ed666 "Carrot2-v9", both 2023-09-27) are parentless
squashed imports of ~3900 files, and no commit in that repo ever modifies any of these terms. The
file moved from longitudinal_mpc_lib/long_mpc.py to carrot/carrot_functions.py and the numbers rode
along unchanged.

So these are 2023 constants, chosen for Korean roads and a radar-equipped Hyundai, never revisited.
That is exactly why the 82 km/h gate did not fit here: in Korea it cleanly separates city from
motorway, but this car cruises the city at 55 mph (88.5) and the gate switched the whole inference
off for precisely the driving that needs it. The 120/150 m interp cap has the same provenance and
has NOT been measured on this car -- do not read its survival as evidence that it is right.

One piece of real history does survive, in the caller: the commented-out block above
`self.stopSignCount = ... if stopSign else 0` shows an abandoned attempt to require
`model_x > get_safe_obstacle_distance(...)`, i.e. to only count a stop that is farther away than
comfortable braking needs. What shipped instead counts every frame, and the trigger below it fires
on a single frame (`stopSignCount * DT_MDL > 0.0`).

Worth knowing when changing any of this: stock openpilot has no equivalent heuristic at all. Its
planner takes min() over {mpc, cruise, e2e} where the e2e term is modelV2.action.desiredAcceleration
-- a graded value the model publishes every frame -- so a stop emerges from the plan instead of from
thresholds. That field is populated on this car (100% of frames) and is a markedly cleaner signal
than the path end this file thresholds on; see longitudinal_planner_carrot.py, where it is computed
and then discarded unless mpc.mode is 'blended'. (modelV2.action.shouldStop is not the counterpart:
should_stop() is v_ego < 0.3 and a_target < 0.1, a standstill latch, and it is False throughout
these approaches.)
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
