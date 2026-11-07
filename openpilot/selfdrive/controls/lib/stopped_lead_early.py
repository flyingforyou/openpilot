"""Start easing off for a stopped car the model sees before the radar has it.

Both FCWs after the 11/04 update (0245, 024b) were the same thing at 72 km/h: a stationary car
that only became a lead at ~55 m, 2.7 s out, so the first command was already -4.5 and the peak
-5.8. The model had it well before that -- at 85-105 m, with probability 0.1-0.3 and a speed of
3-4 m/s -- and the radar does not report a stationary object until it can be matched to one.

Replayed over every engaged frame since 10/31 (747 segments): with the rule below at P >= 0.2 it
fired ahead of 13 of the 17 approaches to a stopped lead, a median 2.7 s before the radar lead
appeared, both FCWs included. Of the episodes not followed by a stopped lead within 10 s nearly all
still ended in a real slowdown (to 0-26 km/h); one, at 104 km/h, did not. So the cap is kept
small and short: it only ever lowers the command, it is jerk-limited on the way in, it lets go
0.6 s after the model stops saying it, and it hands over to the MPC the moment the radar has the
car.

Tuning, closed-loop on the 0245 FCW (peak braking -4.84 with this off) against the same rule
replayed open-loop over ~10 h of engaged driving for episodes with no real slowdown after them:
  onset 1.0 m/s^3, cap -1.0/-1.5:  -4.81,  1 false episode (104 km/h, -0.7 for 1.05 s)
  onset 2.0 (this):                -4.70,  1 false episode (-1.0 for 1.2 s)
  + cap -1.5/-2.5, hold 1.0 s:      -4.61,  2 (worst -1.5 for 1.75 s at 104 km/h)
  + path offset 3.5 m:              -4.42,  2 (same)
The model only grows confident late, so the gain is bounded; the stronger rows buy a little more of
it with a highway phantom brake nobody should have to feel.

The lead's lateral position is taken against the model's own path at that distance, not against
straight ahead: both FCW cars sat 8-9 m to the side of our heading at 100 m, in our lane, on a bend.

Off-ramps get looser limits. A queue at the end of a ramp is met at 45-55 km/h, under MIN_V_EGO, and
around the bend neither sensor has it early: over the 10 ramp approaches to a slower lead since
10/31 the radar reported the stopped cars only ~1 s before they became the lead, at 37-48 m, and the
model held P 0.1-0.3 until the same moment. In 0235 the car even accelerated at +1.0 toward the
queue for those seconds. Closed loop, peak braking and closest gap on the four late ones:
  0235  -3.48 / 2.1 m -> -2.14 / 4.5 m      022e  -3.41 / 3.1 m -> -2.85 / 3.5 m
  024d  -3.37 / 3.3 m -> -3.06 / 3.6 m      024b  -3.35 / 4.9 m -> -3.27 / 5.1 m
and no change on the other six. Lowering only the speed floor did nothing; nearly all of this comes
from RAMP_MIN_PROB. Replayed over all 7.9 min of engaged ramp driving: no false episode, where a
ratio of 0.8 starts to produce them. That is a small sample, so it stays on ramps, where a slow
car ahead is expected, and the highway limits above are unchanged.
"""
import numpy as np

MIN_V_EGO = 15.0          # m/s; below this the radar sees the queue in time
MIN_PROB = 0.2
SLOW_LEAD_V = 5.0         # m/s, or SLOW_LEAD_RATIO of our speed, whichever is higher
SLOW_LEAD_RATIO = 0.45
X_RANGE = (40.0, 120.0)   # m; inside 40 the radar has it, past 120 the model is guessing
MAX_PATH_OFFSET = 2.5     # m from the model's own path at the lead's distance
RADAR_MATCH_M = 15.0      # a radar lead this close to the model's distance is the same car
STOP_BUFFER = 25.0        # m short of the car the early share is planned to
EARLY_SHARE = 0.5         # of the deceleration a full stop there would need
CAP_LOW_PROB = -1.0       # m/s^2, prob < 0.5
CAP_HIGH_PROB = -1.5      # m/s^2, prob >= 0.5
ONSET_JERK = 2.0          # m/s^3
RELEASE_RATE = 2.0        # m/s^3
HOLD_S = 0.6
RAMP_MIN_V_EGO = 10.0     # m/s
RAMP_MIN_PROB = 0.1
RAMP_SLOW_LEAD_RATIO = 0.6
RAMP_MIN_X = 30.0         # m


def fires(v_ego, prob, x, v_lead, y_lead, path_x, path_y, radar_present, radar_d, on_ramp=False):
  min_v, min_prob = (RAMP_MIN_V_EGO, RAMP_MIN_PROB) if on_ramp else (MIN_V_EGO, MIN_PROB)
  ratio, min_x = (RAMP_SLOW_LEAD_RATIO, RAMP_MIN_X) if on_ramp else (SLOW_LEAD_RATIO, X_RANGE[0])
  if v_ego < min_v or prob < min_prob:
    return False
  if v_lead >= max(SLOW_LEAD_V, ratio * v_ego) or not (min_x <= x <= X_RANGE[1]):
    return False
  path_y_at = float(np.interp(x, path_x, path_y)) if len(path_x) else 0.0
  if abs(y_lead - path_y_at) >= MAX_PATH_OFFSET:
    return False
  return not (radar_present and abs(radar_d - x) < RADAR_MATCH_M)


class StoppedLeadEarly:
  def __init__(self, dt):
    self.dt = dt
    self.cap = None       # m/s^2; None = not limiting
    self.target = 0.0
    self.hold = 0.0
    self.active = False

  def reset(self):
    self.cap = None
    self.hold = 0.0
    self.active = False

  def update(self, v_ego, model, radar_lead, on_ramp=False):
    """Returns the accel ceiling to apply this frame, or None."""
    fired = False
    if len(model.leadsV3):
      lead = model.leadsV3[0]
      if len(lead.x) and len(lead.v) and len(lead.y):
        x, v_lead, y_lead = float(lead.x[0]), max(float(lead.v[0]), 0.0), float(lead.y[0])
        fired = fires(v_ego, float(lead.prob), x, v_lead, y_lead, model.position.x, model.position.y,
                      bool(radar_lead.present), float(radar_lead.dRel), on_ramp)
        if fired:
          need = (v_ego - v_lead) ** 2 / (2.0 * max(x - STOP_BUFFER, 10.0))
          floor = CAP_HIGH_PROB if lead.prob >= 0.5 else CAP_LOW_PROB
          self.target = max(-EARLY_SHARE * need, floor)
          self.hold = HOLD_S

    self.active = self.hold > 0.0
    if self.active:
      self.hold -= self.dt
      start = self.cap if self.cap is not None else 0.0
      self.cap = max(self.target, start - ONSET_JERK * self.dt) if self.target < start else self.target
    elif self.cap is not None:
      self.cap += RELEASE_RATE * self.dt
      if self.cap >= 0.0:
        self.cap = None
    return self.cap
