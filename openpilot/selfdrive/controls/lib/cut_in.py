"""See a car merging in before it is in front, from the forward radar alone.

The planner learns about a cut-in only when the followed gap collapses, because until then
nothing tells it. Measured over the 2026-08-19 drive that is far too late: with the gap 25 m
short of target at 46 mph and 0.74 s of headway, the plan asked for -0.18 m/s^2. It recovers, but
mostly because the other car pulls away rather than because this one backed off.

CarrotPilot answers this by watching each radar track move sideways towards the lane instead of
waiting for it to arrive, but its detector is built on corner radars. This car has forward radar
only. That turns out not to be the limit: the merging car is visible for a median 13.4 s before
it becomes the lead, and in every measured case it is first seen outside the lane. The limit was
in how the question is asked.

Asking "will this be inside my lane within 1.5 s" only fires once the car is nearly there --
replayed, that gave a median 0.3 s of warning, which is no warning. Asking instead for *sustained
progress towards* the lane gives a median 2.1 s (p10 1.1, max 7.1).

Rate alone still cannot tell a merge from a car correcting its own lane position, so a second,
cumulative gate asks how much ground the track has actually taken -- see MIN_PROGRESS.

The constants below come from sweeping them over three drives rather than from taste. At these
values it calls 0.24-0.92 times a minute and 89%/80%/63% of those calls are followed by the
followed gap really collapsing -- the low figure is the freeway drive, where the misses are
adjacent-lane cars on a multi-lane road that never actually merge. In false calls per minute
that is 0.04 / 0.05 / 0.28.

The response deliberately lives elsewhere: this module only decides which track is merging. What
to do about it is radard's business, and what it does is hand the track to the planner as a
second obstacle -- so a false call costs a little distance to a real car that is really there,
which is the cheapest way for this to be wrong.
"""
from collections import deque

# Range over which a merge is worth acting on. Nearer than this and the response cannot beat the
# geometry; further and the car has time to sort itself out. CarrotPilot uses 5..50 for the same
# job on a front radar (its VW MEB path); this starts a metre later.
MIN_DREL = 6.0
MAX_DREL = 50.0

# Moving traffic only. Roadside furniture drifts across dPath on a curve exactly like a merge.
MIN_VLEAD = 4.0

# |dPath| closing is measured across a window rather than from the track's own yvRel frame to
# frame: the instantaneous lateral rate of a car easing over one lane sits inside the radar's
# noise, and that is precisely the merge worth catching early.
RATE_WINDOW_S = 1.0
MIN_SAMPLES = 3

# ...and rate alone cannot tell a merge from a wobble. A car correcting its own lane position
# moves inward at a perfectly respectable 0.3 m/s for half a second and then goes back. What it
# never does is keep the ground it took, so the second gate is cumulative: how far in has this
# track come from its own recent widest point, which a wobble resets and a merge only grows.
#
# There is room to measure it. The lateral estimate is quiet -- frame to frame it moves a p90 of
# 0.026 m out to 35 m and 0.045 m beyond, so half a metre is more than ten times the noise.
#
# Measured over three drives this is free: precision goes 73%->89% and 54%->63%, false calls
# 0.13->0.04 and 0.44->0.28 per minute, and the warning time does not move (1.7s and 1.3->1.4s).
# 0.8 m was also tried and starts dropping real merges without buying more precision.
PROGRESS_WINDOW_S = 3.0
MIN_PROGRESS = 0.5

# ...and progress is only meaningful if the frame it is measured in held still. dPath is an offset
# from the model's lane centre, so while the car is turning hard that centre swings and a parked
# car appears to march across the lane. Integrating that over three seconds produces exactly the
# signature this looks for.
#
# It is not hypothetical: replaying the 2026-08-19 drive, the very first call came 1.3 s after
# engaging out of a hand-driven U-turn, on a track 42.5 m away that was not going anywhere.
#
# 0.20 rad/s separates the two cleanly. That U-turn ran a median 0.397 and peaked at 0.677, while
# ordinary driving on the freeway route reaches 0.043 at p99.9 and never crosses 0.20 at all. The
# wait afterwards is the full progress window, because that is how much settled history the
# cumulative gate consumes.
MAX_YAW_RATE = 0.20

# The same reasoning applies to our own lane change, and it is not a corner case: replayed over
# 00000087 one of the eight calls was the car changing lanes to the right, with the blinker on and
# laneChangeState running preLaneChange -> laneChangeStarting. Nobody merged. We moved, the lane
# centre moved with us, and a car sitting still in the next lane read as sweeping into ours.
#
# A merge and a lane change look identical from a track's dPath alone, because dPath cannot tell
# which of the two cars did the moving. What separates them is knowing our own intent, which the
# model already publishes.

# The three gates that make a merge a merge. Every one is tighter than CarrotPilot's equivalent,
# because this fires a brake: a miss costs comfort, a false call costs trust.
MIN_CLOSING = 0.25        # m/s towards the lane centre, sustained over the window
MAX_LANE_ENTRY_S = 3.0    # ...and on course to reach our lane edge within this
# One lane over, never two. This is the single most valuable gate: widening it to 2.5 half-widths
# takes the freeway drive from 24 calls at 54% to 34 at 47%, all of the extra ones spurious.
OUTER_LANES = 2.0

# Half a second of agreement before it counts, and a second of silence before it stops counting.
# The hold matters more than it looks: a merging car crossing the lane line is exactly when the
# radar is most likely to drop a frame.
CONFIRM_FRAMES = 10
RELEASE_FRAMES = 20


class CutInDetector:
  """Which currently-tracked object, if any, is merging in front of us.

  One instance per radard, updated with the whole track list each frame. Holds only the per-track
  history it needs, and forgets a track the moment it stops being reported.
  """

  def __init__(self) -> None:
    self._trail: dict[int, deque] = {}     # track id -> [(t, |dPath|)] over RATE_WINDOW_S
    self._confirm: dict[int, int] = {}     # track id -> consecutive qualifying frames
    self._release: dict[int, int] = {}     # track id -> frames since it last qualified
    self._unsettled_until: float = -1e9    # no call before this: the lane frame is still moving
    self.track_id: int = -1                # the merging track, or -1

  def _motion(self, tid: int, t: float, abs_d_path: float) -> tuple[float, float] | None:
    """(closing m/s over the last second, ground taken over the last three). None until both
    are measurable -- one trail serves both, since the rate is just its recent end."""
    trail = self._trail.setdefault(tid, deque())
    trail.append((t, abs_d_path))
    while trail and t - trail[0][0] > PROGRESS_WINDOW_S:
      trail.popleft()

    recent = [(ts, dp) for ts, dp in trail if t - ts <= RATE_WINDOW_S]
    span = t - recent[0][0] if recent else 0.0
    if len(recent) < MIN_SAMPLES or span < RATE_WINDOW_S * 0.5:
      return None
    closing = (recent[0][1] - abs_d_path) / span
    progress = max(dp for _, dp in trail) - abs_d_path
    return closing, progress

  def _qualifies(self, track, t: float, lead_d_rel: float) -> bool:
    if not track.measured or not (MIN_DREL < track.dRel < MAX_DREL) or track.vLead < MIN_VLEAD:
      return False
    # Nothing to gain from a car merging in behind what we already follow.
    if lead_d_rel > 0.0 and track.dRel > lead_d_rel - 1.0:
      return False

    half = max(0.1, track.lane_half_width)
    abs_dp = abs(track.dPath)
    # Already in the lane is not a merge -- that is a lead, and the lead pipeline has it.
    if not (half < abs_dp < OUTER_LANES * half):
      return False

    motion = self._motion(track.identifier, t, abs_dp)
    if motion is None:
      return False
    closing, progress = motion
    # Coming in, having actually come in, and due to arrive soon. All three, or it is a wobble.
    if closing <= MIN_CLOSING or progress <= MIN_PROGRESS:
      return False
    return (abs_dp - half) / closing < MAX_LANE_ENTRY_S

  def update(self, tracks: dict, t: float, v_ego: float, lead_d_rel: float = 0.0,
             yaw_rate: float = 0.0, lane_changing: bool = False) -> int:
    """Returns the merging track's id, or -1. `lead_d_rel` is 0.0 when nothing is followed."""
    if v_ego < MIN_VLEAD:
      self.reset()
      return -1

    # Moving the frame invalidates every track's history, not just the next frame's reading, so
    # the trails go too -- keeping them would let our own motion count as progress once the wait
    # expires. Both causes are the same bug: dPath cannot say which car moved.
    if abs(yaw_rate) > MAX_YAW_RATE or lane_changing:
      self._unsettled_until = t + PROGRESS_WINDOW_S
      self._trail.clear()
      self._confirm.clear()
      self._release.clear()
      self.track_id = -1
      return -1
    if t < self._unsettled_until:
      self.track_id = -1
      return -1

    live = set(tracks)
    for tid in list(self._trail):
      if tid not in live:
        self._trail.pop(tid, None)
        self._confirm.pop(tid, None)
        self._release.pop(tid, None)

    best, best_d = -1, 1e9
    for tid, track in tracks.items():
      if self._qualifies(track, t, lead_d_rel):
        self._confirm[tid] = self._confirm.get(tid, 0) + 1
        self._release[tid] = 0
      else:
        self._confirm[tid] = 0
        # Hold a confirmed track through a dropout rather than restarting its count.
        if self._release.get(tid, RELEASE_FRAMES) < RELEASE_FRAMES:
          self._release[tid] += 1
      held = tid == self.track_id and self._release.get(tid, RELEASE_FRAMES) < RELEASE_FRAMES
      if (self._confirm[tid] >= CONFIRM_FRAMES or held) and track.dRel < best_d:
        best, best_d = tid, track.dRel

    self.track_id = best
    return best

  def reset(self) -> None:
    self._trail.clear()
    self._confirm.clear()
    self._release.clear()
    self._unsettled_until = -1e9
    self.track_id = -1
