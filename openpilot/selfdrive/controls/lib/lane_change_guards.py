"""What has to be true before a lane change starts on the blinker alone.

The driver's nudge is its own authority -- someone with a hand on the wheel has looked. Starting
without one means the car is asserting the move is safe using only what it can see, so this
collects the conditions for that, separately from the state machine that acts on them.

Thresholds are measured on this car rather than inherited: the road-edge and lane-line numbers
come from `tools/tesla_analysis/side_radar_coverage.py` and the road-edge survey over the
2026-08-18 drive, and the side-lead model is CarrotPilot's, whose constants are already tuned.

What none of this can do is see behind. The radar is forward-only -- 1.6M points on that drive,
not one with dRel < 0 -- and the ultrasonics reach about five metres. A car closing from behind
in the target lane is invisible to every check here.
"""
import numpy as np

from openpilot.cereal import log

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# A lane is ~3.27 m on this car's roads (measured median). Evidence that another one exists on a
# side: the model draws the line beyond that side's lane line, or the road edge is far enough
# out to hold one. Either alone is enough -- an exit lane often has no outer paint, and a
# concrete barrier can sit close while a lane is still there.
OUTER_LINE_PROB = 0.3
EDGE_ROOM_M = 4.0

# CarrotPilot's side-lead model (selfdrive/controls/lib/desire_lib/side_state.py).
SIDE_LEAD_CLOSE_DREL = 5.0
SIDE_LEAD_MIN_GAP_BASE = 6.0
SIDE_LEAD_MIN_GAP_TIME = 0.30
SIDE_LEAD_MIN_GAP_MAX = 12.0
SIDE_LEAD_PROJECT_SEC = 1.5
SIDE_LEAD_TTC_MIN_CLOSING = 0.5
SIDE_LEAD_TTC_NEAR_MARGIN = 6.0

# The vehicle a lane change moves in behind must be in the lane being entered, not the one past it.
# get_side_leads accepts anything 1.8-5.4 m to a side, which is wider than a lane: measured over
# four lane changes, the one genuine target-lane lead sat at |yRel| 1.5 m while three others ran
# 3.9-5.3 m and never converged toward our path as the change progressed -- the signature of a car
# one lane further out. One of those braked the car from 106 to 79 km/h (commanded -3.50 m/s^2) for
# a car it was never going to end up behind, while its real lead sat untroubled at 46 m.
#
# A lane is ~3.27 m here, so a target-lane vehicle starts a change at about that and shrinks toward
# zero; the next lane over starts at ~6.5 m. This keeps the former and drops the latter.
TARGET_LANE_MAX_YREL = 3.5


def lane_exists(outer_line_prob: float, edge_distance: float) -> bool:
  """Is there room for a lane on this side? Unknown counts as yes -- this gate exists to catch
  the shoulder, not to require proof before every move."""
  if not np.isfinite([outer_line_prob, edge_distance]).all():
    return True
  return outer_line_prob > OUTER_LINE_PROB or edge_distance > EDGE_ROOM_M


def side_lead_unsafe(lead, v_ego: float) -> bool:
  """Is the nearest thing in that lane too close, or closing too fast, to move across?

  Ported from CarrotPilot. The three questions are: is it simply near, is the time to contact
  short while closing, and would the gap still be there in a second and a half.
  """
  if lead is None or not getattr(lead, 'present', False):
    return False

  d_rel = float(getattr(lead, 'dRel', 255.0))
  if not 0.1 < d_rel < 160.0:
    return False

  v_ego = max(0.0, float(v_ego))
  v_lead = float(getattr(lead, 'vLead', v_ego))
  v_rel = float(getattr(lead, 'vRel', v_lead - v_ego))

  if d_rel < SIDE_LEAD_CLOSE_DREL:
    return True

  min_gap = float(np.clip(v_ego * SIDE_LEAD_MIN_GAP_TIME, SIDE_LEAD_MIN_GAP_BASE, SIDE_LEAD_MIN_GAP_MAX))

  closing_speed = max(0.0, -v_rel)
  if closing_speed > SIDE_LEAD_TTC_MIN_CLOSING:
    ttc = d_rel / closing_speed
    ttc_limit = float(np.interp(v_ego, [0.0, 15.0, 30.0], [2.0, 3.0, 3.5]))
    near_enough = d_rel < max(min_gap + SIDE_LEAD_TTC_NEAR_MARGIN, v_ego * 1.2)
    if near_enough and ttc < ttc_limit:
      return True
    if d_rel + v_rel * SIDE_LEAD_PROJECT_SEC < min_gap:
      return True

  # Something sitting alongside at the same speed still has to be given room.
  return d_rel < min_gap and v_rel < 1.0


def target_lane_lead(meta, lead_left, lead_right, lead_two):
  """The vehicle a lane change is moving in behind, or None to leave the plan alone.

  Until the change completes the target lane's vehicle is not a lead, so nothing anticipates it: a
  move in behind a lorry is made at the speed we were already doing and braked for afterwards.
  Measured at the moments lane changes began, 3 of 7 had a slower vehicle ahead in that lane, a
  median 2 m/s slower at 16-37 m -- a gap that closes in 8-18 s.

  Only while a change is being made or asked for. Outside that the adjacent lane is none of the
  planner's business, and treating it as one would slow the car for traffic it is not going behind.
  Returns None unless it is also the more binding of the two, so a nearer cut-in is not displaced.
  """
  if meta.laneChangeState == LaneChangeState.off:
    return None
  if meta.laneChangeDirection == LaneChangeDirection.left:
    target = lead_left
  elif meta.laneChangeDirection == LaneChangeDirection.right:
    target = lead_right
  else:
    return None

  if target is None or not target.present:
    return None
  if abs(target.yRel) > TARGET_LANE_MAX_YREL:
    return None
  if lead_two is not None and lead_two.present and lead_two.dRel <= target.dRel:
    return None
  return target


# The ultrasonics that look sideways at speed: the rear side pair (8 right, 11 left; ussRear[1] and
# [4]), the rear corners (7 right, 12 left; ussRear[0] and [5]) and the front corners (6 right,
# 1 left; ussFront[5] and [0]). The centre ones only ever see anything below ~10 km/h. Anything
# inside USS_CLOSE_CM on the side being moved into is beside the car, whatever the blindspot flag
# has decided yet.
#
# Replayed over 140 lane change starts since 10/31 (start, and the 1 s abort window after it):
#   rear side + rear corner at 1.5 m: blocks none; reads "occupied" with no blindspot flag on 0.23%
#     of engaged >54 km/h frames
#   + front corner at 1.5 m, instant: blocks 1 -- a single-frame 1.26 m spike on a corner that
#     otherwise read 3.5-4.9 m (the front corners' road noise floor at speed)
#   + front corner at 1.5 m held USS_FRONT_HOLD_S: blocks none, 0.35% with no flag
#   everything at 2.0 m: blocks 4-8 completed changes, ~2% with no flag
# So the rear ones count at once and a front corner has to stay close for a moment.
USS_CLOSE_CM = 150
USS_FRONT_HOLD_S = 0.4
USS_MIN_CM = 20  # 0 is "blocked", 1 is a near-field code, neither is a distance


def _uss(carstate):
  cs = getattr(carstate, 'chassisSensors', None)
  if cs is None or not cs.ussValid or len(cs.ussRear) < 6 or len(cs.ussFront) < 6:
    return None
  return cs


def _close(reading) -> bool:
  return USS_MIN_CM < int(reading) < USS_CLOSE_CM


def uss_rear_close(carstate, left: bool) -> bool:
  cs = _uss(carstate)
  if cs is None:
    return False
  side, corner = (cs.ussRear[4], cs.ussRear[5]) if left else (cs.ussRear[1], cs.ussRear[0])
  return _close(side) or _close(corner)


def uss_front_corner_close(carstate, left: bool) -> bool:
  cs = _uss(carstate)
  return cs is not None and _close(cs.ussFront[0] if left else cs.ussFront[5])
