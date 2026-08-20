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

The constants below come from sweeping them over three drives rather than from taste. At these
values it calls 0.24-0.96 times a minute and 73%/80%/54% of those calls are followed by the
followed gap really collapsing -- the low figure is the freeway drive, where the misses are
adjacent-lane cars on a multi-lane road that never actually merge.

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
    self.track_id: int = -1                # the merging track, or -1

  def _closing(self, tid: int, t: float, abs_d_path: float) -> float | None:
    """Rate at which this track is closing on the lane centre, m/s. None until measurable."""
    trail = self._trail.setdefault(tid, deque())
    trail.append((t, abs_d_path))
    while trail and t - trail[0][0] > RATE_WINDOW_S:
      trail.popleft()
    span = t - trail[0][0]
    if len(trail) < MIN_SAMPLES or span < RATE_WINDOW_S * 0.5:
      return None
    return (trail[0][1] - abs_d_path) / span

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

    closing = self._closing(track.identifier, t, abs_dp)
    if closing is None or closing <= MIN_CLOSING:
      return False
    return (abs_dp - half) / closing < MAX_LANE_ENTRY_S

  def update(self, tracks: dict, t: float, v_ego: float, lead_d_rel: float = 0.0) -> int:
    """Returns the merging track's id, or -1. `lead_d_rel` is 0.0 when nothing is followed."""
    if v_ego < MIN_VLEAD:
      self.reset()
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
    self.track_id = -1
