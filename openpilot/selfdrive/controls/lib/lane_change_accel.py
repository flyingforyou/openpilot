"""Let the car pick up speed while it moves out from behind a car, not only once it has.

During a lane change the car in the lane being left stays leadOne until the car is ~2.5 m across:
over the 18 overtakes since 10/31 with nothing ahead in the new lane, it was still the lead for the
first ~3 s of a ~5 s change, and the command sat at +0.02-0.05 m/s^2 the whole time -- holding its
speed -- before it climbed to +0.9 as the lead finally let go. A driver who has checked the lane
starts accelerating as the wheel goes over.

Closed loop over the 50 overtaking lane changes since 10/31 (lead under 70 m before, room above the
target): speed gained 3 s after the start +0.2 -> +1.2 km/h median, 6 s after +2.9 -> +4.4, 13
of 50 picking up >= 1 km/h earlier, with no case of harder braking or a shorter headway. Starting
at 0.3 s instead added two such cases -- the 1.1 s change into a slower lane in 0235 seg 13 braked
-2.68 -- for no more speed, so the ramp starts at 0.8 s.

So while a lane change is under way into a lane the side-lead check calls clear, the command is
allowed up toward what the cruise target alone would ask for, ramping in over RAMP_S. Only toward
it -- it never lowers a command -- and only while the lead being left is one we are not closing
on: at least MIN_HEADWAY_S ahead, not braking, and still at least KEEP_HEADWAY_S ahead after
LOOKAHEAD_S of the extra acceleration if the change were abandoned and it stayed in our lane.
"""
import numpy as np

START_S = 0.8          # into the lane change before anything changes
RAMP_S = 1.5           # to full weight
TARGET_TAU_S = 2.0     # the cruise target is closed at (v_cruise - v) / TARGET_TAU_S
MIN_HEADWAY_S = 1.0
MIN_GAP_M = 15.0
MAX_LEAD_DECEL = -0.5  # m/s^2; a lead braking harder than this keeps the change honest
MAX_CLOSING = 1.0      # m/s; a lead this much slower than us is not being left, it is being caught
LOOKAHEAD_S = 3.0
KEEP_HEADWAY_S = 0.8
# A slower car already in the lane being entered means braking behind it shortly; building speed
# first only makes that harder. Closed loop, 0235 seg 13: the change was 1.1 s, a car 23 km/h slower
# sat 59 m up the new lane, and the extra +0.4 carried on through the planner into -2.68 braking
# where the plain run braked at -1.47.
TARGET_LANE_SLOWER_M = 100.0


def free_accel(v_ego, v_cruise, a_max):
  return float(np.clip((v_cruise - v_ego) / TARGET_TAU_S, 0.0, a_max))


def target_lane_ok(v_ego, side_lead):
  return not (side_lead is not None and side_lead.present and float(side_lead.dRel) < TARGET_LANE_SLOWER_M
              and float(side_lead.vLead) < v_ego)


def lead_ok(v_ego, lead, a):
  if not lead.present:
    return True
  d, v_lead = float(lead.dRel), float(lead.vLead)
  if d < max(MIN_HEADWAY_S * v_ego, MIN_GAP_M) or float(lead.aLeadK) < MAX_LEAD_DECEL or v_lead < v_ego - MAX_CLOSING:
    return False
  # if the change were called off and it stayed in front of us
  t = LOOKAHEAD_S
  v_end = v_ego + a * t
  d_end = d + v_lead * t - (v_ego * t + 0.5 * a * t * t)
  return d_end >= KEEP_HEADWAY_S * v_end


class LaneChangeAccel:
  def __init__(self, dt):
    self.dt = dt
    self.t = 0.0
    self.weight = 0.0

  def reset(self):
    self.t = 0.0
    self.weight = 0.0

  def update(self, a_target, changing, side_clear, v_ego, v_cruise, a_max, lead, side_lead=None):
    """Returns the command to use: a_target, or something between it and the free-road accel."""
    if not changing:
      self.reset()
      return a_target
    self.t += self.dt
    a_free = free_accel(v_ego, v_cruise, a_max)
    if not side_clear or a_free <= a_target or not lead_ok(v_ego, lead, a_free) or not target_lane_ok(v_ego, side_lead):
      self.weight = 0.0
      return a_target
    # ramps in from START_S, and never jumps back up after a condition drops it to zero
    ramp = float(np.clip((self.t - START_S) / RAMP_S, 0.0, 1.0))
    self.weight = min(ramp, self.weight + self.dt / RAMP_S)
    return a_target + self.weight * (a_free - a_target)
