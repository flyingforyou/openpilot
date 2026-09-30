from __future__ import annotations

import math

from typing import TYPE_CHECKING, Any

from openpilot.common.realtime import DT_MDL

if TYPE_CHECKING:
  from openpilot.selfdrive.controls.radard import Track

# Radar lead hold. get_lead gates on the vision prob before any radar matching, so a dip in model
# confidence throws away a radar track that is still measuring the car in front.
# Measured on this car's logs (365 segments, 148 engaged min): with the lead inside 30m, the
# radar->vision fallback moves the reported distance +4.9m median / +9.8m p90, and 59% of those
# fallbacks over-read by more than 2m (median: a real 18.4m reported as 24.8m). The car then
# believes it has room it does not have. Holding the track we were already following bridges the
# dip instead of accepting that jump.
#
# Originally scoped to close range, where over-reading is the dangerous direction. The 09-01 and
# 09-02 routes showed the same dropout happening far out and far more often: the lead is lost at a
# median 44 m and 97 m respectively, and 90% of radar<->vision flips left the vision model tracking
# the same car while the reported distance jumped 5.7-11.9 m. That is the radar association
# breaking, not the object changing, so it is exactly what this bridges -- the distance ceiling is
# a param (RadarLeadHoldCm), not anything structural.
RADAR_LEAD_HOLD_DEFAULT_MS = 1000

# What makes holding safe at range. Close in, a still-measured track that vision dropped is almost
# certainly the same car. Far out it has room to be a car in the next lane that the path has bent
# away from, so refuse to hold one that has drifted off our path. Limits follow carrot's sticky
# track, which solves the same problem with the same numbers.
RADAR_LEAD_HOLD_MAX_DPATH = 0.8      # m off the predicted path, within FAR_DREL
RADAR_LEAD_HOLD_MAX_DPATH_FAR = 1.2  # m, beyond it -- the path estimate itself is looser out there
RADAR_LEAD_HOLD_FAR_DREL = 60.0      # m


class RadarLeadHold:
  """Keep following a radar track through a vision dropout.

  Strictly persistence: the held track is one the vision model already confirmed as the lead, so
  this can never promote radar clutter (overhead signs, guardrails) into a lead on its own. It
  only refuses to *discard* a track, and only while the radar is still measuring it, it has not
  jumped, it is on our path, it is inside the configured distance, and the hold budget
  has not run out.
  """

  def __init__(self):
    self.hold_dist = 0.0      # m; 0 disables the feature
    self.max_frames = 0
    self.track_id = -1
    self.frames = 0
    self.used = False         # set by get_lead when it took the hold path this frame

  def configure(self, hold_dist: float, hold_ms: int) -> None:
    self.hold_dist = max(0.0, hold_dist)
    self.max_frames = max(1, int((hold_ms / 1000.0) / DT_MDL))

  def candidate(self, tracks: dict[int, Track]) -> Track | None:
    """The track we may keep publishing this frame, or None."""
    if self.hold_dist <= 0.0 or self.track_id < 0 or self.frames >= self.max_frames:
      return None
    track = tracks.get(self.track_id)
    # selected_count is the continuity flag: Track.update zeroes it the moment the track stops
    # being measured or jumps, so a nonzero count means this is still the same object vision
    # confirmed, and _update_match_counters zeroes it as soon as vision picks someone else.
    if track is None or not track.measured or track.selected_count <= 0:
      return None
    if not 0.0 < track.dRel < self.hold_dist:
      return None
    # Drifted out of our path: at range that is the failure mode holding would turn into following
    # the wrong car, and selected_count alone will not catch it -- the track is still measured and
    # still the one vision last confirmed, it has just stopped being in front of us.
    max_dpath = RADAR_LEAD_HOLD_MAX_DPATH if track.dRel < RADAR_LEAD_HOLD_FAR_DREL else RADAR_LEAD_HOLD_MAX_DPATH_FAR
    if abs(track.dPath) > max_dpath:
      return None
    return track

  def observe(self, lead: dict[str, Any]) -> None:
    """Bookkeeping against the lead that was actually published."""
    if lead.get('present') and lead.get('radar'):
      if self.used and lead.get('radarTrackId', -1) == self.track_id:
        self.frames += 1      # still bridging the same dropout, burn budget
      else:
        # vision agrees again (or picked a different track): restore the full budget
        self.track_id = int(lead.get('radarTrackId', -1))
        self.frames = 0
    else:
      self.track_id = -1
      self.frames = 0
    self.used = False


# The lead's acceleration straight from the radar, instead of differentiating its speed.
#
# radard has always estimated aLeadK by running the lead's speed through a two-state Kalman filter
# with low gains (K 0.20 / 0.29), tuned to keep radar speed noise from turning into phantom braking.
# The price is lag. The Bosch unit on this car reports each track's relative acceleration itself
# (LongAccel, 0.03125 m/s^2 steps), and radard never read it.
#
# Measured over 1078 segments against the lead's true acceleration (a centred difference of its
# speed, so no lag of its own), the time to register a change past 0.5 m/s^2:
#
#                       braking onset           accelerating onset      |a|>0.5 while steady
#   aLeadK (Kalman)     +0.74 s, catches 72%    +0.70 s, catches 78%    2.74%
#   a_ego + LongAccel   +0.30 s, catches 87%    +0.29 s, catches 91%    3.31%
#
# Where both saw it, the radar value was first 99% of the time. Low-pass filtering it does not
# lower the false-alarm rate at all (3.2-3.5% for tau 0.05-0.25 s) -- that disagreement is not
# high-frequency noise -- it only adds lag, so the value is used raw. Both directions matter for
# the stop-and-go lurch: on route 00000178 seg 6 the lead had finished accelerating (true +0.15)
# while aLeadK still read +0.75, and the car was still commanding +1.68 because of it.
#
# CarrotPilot has the same branch (a_lead = aRel + a_ego) but hard-disabled with `if True:`; its
# fleet is mostly Hyundai radars, where aRel is not usable. That does not transfer to this radar.
RADAR_LEAD_ACCEL_CLIP = (-10.0, 5.0)


def radar_lead_accel(a_rel: float, a_ego: float) -> float | None:
  """Absolute lead acceleration from the radar's own relative value, or None if it gave none."""
  if a_rel is None or not math.isfinite(a_rel) or not math.isfinite(a_ego):
    return None
  lo, hi = RADAR_LEAD_ACCEL_CLIP
  return min(max(a_rel + a_ego, lo), hi)
