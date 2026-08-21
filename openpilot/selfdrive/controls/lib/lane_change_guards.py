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
  if lead_two is not None and lead_two.present and lead_two.dRel <= target.dRel:
    return None
  return target
