#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# Stopped-lead matching. The vision model can read a stopped car as moving (18m/s seen in
# logs against a radar track holding 0m/s), which fails vel_sane and drops us to a vision-only
# lead with a badly wrong closing speed. Rather than loosen vel_sane -- which guards against
# braking for stationary clutter -- require sustained agreement in distance and lateral offset
# before trusting the radar track.
# The lateral gate for this case is y_sane(wide) in match_vision_to_track now; the speed test
# it used to pair with is gone, because pass B is reached precisely when speed disagrees.
STOPPED_LEAD_COUNT_UP = 2           # evidence gained per frame while the pattern holds
STOPPED_LEAD_COUNT_MAX = int(1.0 / DT_MDL)   # commit after ~0.5s of evidence (+2 per frame)
STICKY_SELECTED_COUNT_MAX = int(2.0 / DT_MDL)

# Yaw compensation for the forward projection, from CarrotPilot. Turning the car makes a
# stationary object appear to slide sideways at yaw_rate * dRel; these bound how much of that
# apparent motion is subtracted back out, so a bad yaw estimate cannot invent lateral speed.
YAW_COMP_GAIN = 0.6
YAW_COMP_MAX_DREL = 50.0
YAW_COMP_MAX_YAW_RATE = 0.35
YAW_COMP_MAX_YVREL_CORRECTION = 1.5
YAW_COMP_MAX_VREL_CORRECTION = 0.6

# How far ahead a track is projected when deciding whether it is heading into our lane.
RADAR_LAT_PROJECTION_S = 0.6

# A track that jumps this much between frames isn't the same object; drop its evidence.
TRACK_JUMP_D = 5.0   # m
TRACK_JUMP_Y = 2.0   # m
TRACK_JUMP_V = 7.0   # m/s

# Close-range radar lead hold. get_lead gates on the vision prob before any radar matching, so a
# dip in model confidence throws away a radar track that is still measuring the car in front.
# Measured on this car's logs (365 segments, 148 engaged min): with the lead inside 30m, the
# radar->vision fallback moves the reported distance +4.9m median / +9.8m p90, and 59% of those
# fallbacks over-read by more than 2m (median: a real 18.4m reported as 24.8m). The car then
# believes it has room it does not have. Holding the track we were already following bridges the
# dip instead of accepting that jump.
RADAR_LEAD_HOLD_DEFAULT_MS = 1000


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    # evidence that this track is a stopped lead the vision model is misreading, and
    # how long it has been the chosen match (see match_vision_to_track)
    self.is_stopped_car_count = 0
    self.selected_count = 0

    # Lane-relative position, from CarrotPilot. yRel is measured along a straight line out of
    # the bumper, so on a curve a roadside object reads as very nearly ahead while the lane it
    # would have to be in has already bent away. dPath asks the question in the frame that
    # matters: how far is this from the middle of my lane, at its distance.
    self.dPath = 0.0
    self.in_lane_prob = 0.0
    self.lane_half_width = 1.8
    self.dRel_future = 0.0
    self.yRel_future = 0.0
    self.yvRel = 0.0
    self.aLead = 0.0
    self.jLead = 0.0
    self.score = 0.0
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def vlead_for_matching(self, dv_max: float = 4.0, alpha: float = 0.35) -> float:
    """Speed used only for scoring a match, never published.

    A radar track's speed can spike for one frame when the return is weak. Scoring on the raw
    value lets that spike decide which object is the lead; clamping the step and smoothing keeps
    a momentary glitch from reassigning the match, without touching what the planner is told.
    """
    v = float(self.vLead)
    if self.cnt < 2:
      return v
    if not self._vLead_filt_init:
      self._vLead_last = self._vLead_filt = v
      self._vLead_filt_init = True
      return v
    v_last = self._vLead_last
    self._vLead_last = v
    v_clamped = float(np.clip(v, v_last - dv_max, v_last + dv_max))
    self._vLead_filt = alpha * v_clamped + (1.0 - alpha) * self._vLead_filt
    return float(self._vLead_filt)

  def yaw_compensated_velocities(self, yaw_rate: float) -> tuple[float, float]:
    # A curved ego path creates apparent lateral velocity in the ego frame (yaw_rate * dRel).
    # Remove it before projecting the track forward, so an adjacent-lane object on a curve is
    # not read as moving into our lane. Projection-local on purpose: the published vRel stays
    # the raw radar value, because that is what the planner is entitled to expect.
    yaw_rate = float(np.clip(yaw_rate, -YAW_COMP_MAX_YAW_RATE, YAW_COMP_MAX_YAW_RATE))
    d_rel_for_comp = float(np.clip(self.dRel, 0.0, YAW_COMP_MAX_DREL))
    yv_rel_corr = float(np.clip(-yaw_rate * d_rel_for_comp * YAW_COMP_GAIN,
                                -YAW_COMP_MAX_YVREL_CORRECTION, YAW_COMP_MAX_YVREL_CORRECTION))
    v_rel_corr = float(np.clip(yaw_rate * self.yRel * YAW_COMP_GAIN,
                               -YAW_COMP_MAX_VREL_CORRECTION, YAW_COMP_MAX_VREL_CORRECTION))
    return self.vRel + v_rel_corr, self.yvRel + yv_rel_corr

  def d_path(self, md) -> None:
    """Offset from the model's own lane centre, at this track's distance."""
    if len(md.laneLines) < 3:
      return
    lane_xs, left_ys, right_ys = md.laneLines[1].x, md.laneLines[1].y, md.laneLines[2].y
    if not len(lane_xs):
      return

    def interp_at(d_rel, y_rel):
      left_y = np.interp(d_rel, lane_xs, left_ys)
      right_y = np.interp(d_rel, lane_xs, right_ys)
      center_y = (left_y + right_y) / 2.0
      half_width = max(0.1, abs(right_y - left_y) / 2.0)
      dist_from_center = y_rel + center_y
      in_lane = max(0.0, 1.0 - (abs(dist_from_center) / half_width))
      return float(dist_from_center), float(in_lane), float(half_width)

    self.dPath, self.in_lane_prob, self.lane_half_width = interp_at(self.dRel, self.yRel)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float,
             j_lead: float = 0.0, yv_rel: float = 0.0, a_lead: float = 0.0,
             reaction_factor: float = 1.0):
    prev = None if self.cnt == 0 else (self.dRel, self.yRel, self.vLead, self.measured)

    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate
    self.jLead = j_lead        # filtered in the radar interface, see opendbc MyTrack
    self.aLead = a_lead
    self.yvRel = yv_rel

    # Only accumulate evidence across frames where this is plausibly the same object still
    # being seen. An unmeasured or discontinuous track starts over.
    if prev is not None:
      prev_d, prev_y, prev_v, prev_measured = prev
      discontinuous = prev_measured and (abs(self.dRel - prev_d) > TRACK_JUMP_D or
                                         abs(self.yRel - prev_y) > TRACK_JUMP_Y or
                                         abs(self.vLead - prev_v) > TRACK_JUMP_V)
    else:
      discontinuous = False

    if not self.measured or discontinuous:
      self.is_stopped_car_count = 0
      self.selected_count = 0

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # How long the lead's current acceleration is assumed to persist. CarrotPilot scales both
    # the threshold and the time constant by one factor, and judges "steady" on the measured
    # acceleration and jerk rather than the Kalman estimate -- the filtered value lags exactly
    # when the lead changes what it is doing, which is when this decision matters.
    a_threshold = 0.5 * reaction_factor
    steady = abs(self.aLead) < a_threshold if self.measured else abs(self.aLeadK) < a_threshold
    if steady and abs(self.jLead) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU * reaction_factor
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "aLead": float(self.aLead),
      "jLead": float(self.jLead),
      "dPath": float(self.dPath),
      "score": float(self.score),
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


class RadarLeadHold:
  """Keep following a close radar track through a vision dropout.

  Strictly persistence: the held track is one the vision model already confirmed as the lead, so
  this can never promote radar clutter (overhead signs, guardrails) into a lead on its own. It
  only refuses to *discard* a track, and only while the radar is still measuring it, it has not
  jumped, it is inside the configured distance, and the hold budget has not run out.
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
    return track

  def observe(self, lead: dict[str, Any]) -> None:
    """Bookkeeping against the lead that was actually published."""
    if lead.get('status') and lead.get('radar'):
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


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def is_vision_radar_lateral_match_sane(radar_y_rel: float, vision_y_rel: float, d_path: float) -> bool:
  """Either the two sensors agree about where the object is, or it is squarely in our lane."""
  return abs(radar_y_rel - vision_y_rel) < 2.0 or abs(d_path) < 2.4


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          update_counters: bool = True, stopped_lead_enabled: bool = True,
                          stopped_lead_count_max: int = STOPPED_LEAD_COUNT_MAX):
  """Which radar track, if any, is the object the vision model called the lead.

  Ported from CarrotPilot. The gate that matters is lateral: the previous version accepted on
  distance and speed alone, and on a curve a stationary roadside return sits at very nearly the
  vision lead's distance while being nowhere near it, so it could win the match and hand the
  planner a lead closing at ego speed. Three passes, loosening in a controlled way rather than
  falling through to whatever scored highest:

    A  normal    -- both sensors agree on distance, speed and lateral position
    B  stopped   -- a stopped car vision misreads as moving; speed may disagree, position may not
    C  cut-in    -- wider lateral window, for an object genuinely moving into the lane
  """
  if not tracks:
    return None

  offset_vision_dist = float(lead.x[0] - RADAR_TO_CAMERA)
  lead_prob = float(lead.prob)
  vision_y = float(lead.y[0])

  # distance windows: a lower bound as well as an upper one. Without the lower bound a return
  # well short of the vision lead still counts as "close enough" whenever the lead is far away.
  max_dist = max(offset_vision_dist * 1.25, 5.0)
  min_dist = max(offset_vision_dist * 0.80, 1.0)
  max_dist_wide = max(offset_vision_dist * 1.45, 5.0)
  min_dist_wide = 1.5

  # Speed tolerance scales with the lead's own speed and how sure vision is, instead of a flat
  # 10 m/s that a stationary object at urban speed slips under.
  vel_tol = float(max(lead.v[0] * np.interp(lead_prob, [0.8, 0.98], [0.3, 0.5]), 5.0))
  vel_guard = max(vel_tol * 3.0, 20.0)

  def dist_sane(t: Track, wide: bool = False) -> bool:
    return (min_dist_wide < t.dRel < max_dist_wide) if wide else (min_dist < t.dRel < max_dist)

  def y_sane(t: Track, wide: bool = False) -> bool:
    return abs(t.yRel + vision_y) < (4.0 if wide else 2.0)

  def vel_sane(t: Track) -> bool:
    dv = abs(float(t.vLead) - float(lead.v[0]))
    if dv < vel_tol:
      return True
    # Moving objects get more latitude -- vision reads their speed poorly -- but bounded, and
    # never when the track is clearly outside our lane.
    if float(t.vLead) <= 3.0 or dv > vel_guard:
      return False
    return t.in_lane_prob >= 0.25

  def score_pair(t: Track) -> tuple[float, float]:
    pd = laplacian_pdf(t.dRel, offset_vision_dist, lead.xStd[0])
    py = laplacian_pdf(t.yRel, -vision_y, lead.yStd[0])
    py_wide = laplacian_pdf(t.yRel, -vision_y, lead.yStd[0] * 2.0)
    pv = laplacian_pdf(t.vlead_for_matching(), lead.v[0], lead.vStd[0])
    return pd * py * pv, pd * py_wide * pv

  first = second = extra = None
  first_score = second_score = extra_score = -1e18
  for t in tracks.values():
    s1, s2 = score_pair(t)
    t.score = s1
    if not is_vision_radar_lateral_match_sane(t.yRel, -vision_y, t.dPath):
      continue
    if s1 > first_score:
      second, second_score = first, first_score
      first, first_score = t, s1
    elif s1 > second_score:
      second, second_score = t, s1
    if s2 > extra_score:
      extra, extra_score = t, s2

  if first is None or first_score < 1e-4:
    return None

  best = None

  # A) normal match
  if dist_sane(first) and vel_sane(first):
    # A nearer track that vision also likes is usually the real lead; the far one is often a
    # return from beyond it that happened to score well.
    if (second is not None and vel_sane(second) and second.in_lane_prob > 0.3
        and second.cnt > 5 and offset_vision_dist * 0.5 < second.dRel < first.dRel):
      best = second
    elif y_sane(first):
      if lead_prob > 0.5:
        best = first
      elif lead_prob > 0.4 and first.selected_count > 0:
        best = first
    elif lead_prob > 0.6 and abs(first.dPath) < 2.4:
      best = first

  # B) stopped car the vision model reads as moving: position agrees, speed does not. Committing
  # immediately would mean braking for any stationary clutter that lines up, so require the
  # pattern to hold -- or that we were already following this track.
  if best is None and stopped_lead_enabled and dist_sane(first) and y_sane(first, wide=True):
    if (second is not None and second_score > 1e-5
        and dist_sane(second) and y_sane(second) and vel_sane(second)):
      best = second
    elif first.selected_count > 0:
      best = first
    elif first.measured:
      if update_counters:
        first.is_stopped_car_count += STOPPED_LEAD_COUNT_UP
      if first.is_stopped_car_count > stopped_lead_count_max:
        best = first

  # C) cut-in: wider lateral window, only when vision is confident and the object is not far off
  if best is None and offset_vision_dist < 90.0 and lead_prob > 0.65:
    if (extra is not None and extra_score > first_score
        and dist_sane(extra, wide=True) and vel_sane(extra) and y_sane(extra, wide=True)):
      best = extra
    elif dist_sane(first, wide=True) and vel_sane(first) and y_sane(first, wide=True):
      best = first
    elif (second is not None and second_score > 1e-4
          and dist_sane(second, wide=True) and vel_sane(second) and y_sane(second, wide=True)):
      best = second

  # Nothing matched: leave the counters alone. Zeroing selected_count here would defeat the
  # stickiness above, since these are exactly the frames it exists to bridge. Evidence still
  # decays when the track stops being measured or jumps (see Track.update).
  if best is not None:
    _update_match_counters(tracks, best, update_counters)
  return best


def _update_match_counters(tracks: dict[int, Track], selected: Track, enabled: bool) -> None:
  if not enabled:
    return
  for t in tracks.values():
    if t is selected:
      t.selected_count = min(t.selected_count + 1, STICKY_SELECTED_COUNT_MAX)
    else:
      t.selected_count = 0
      t.is_stopped_car_count = max(0, t.is_stopped_car_count - 1)


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
    # Vision has no jerk estimate and no radar track to place in a lane; leave them at zero
    # rather than carrying over a previous frame's radar numbers under a vision-only lead.
    "aLead": float(lead_msg.a[0]),
    "jLead": 0.0,
    "dPath": 0.0,
    "score": 0.0,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, low_speed_override: bool = True,
             update_counters: bool = True, stopped_lead_enabled: bool = True,
             stopped_lead_count_max: int = STOPPED_LEAD_COUNT_MAX,
             hold: RadarLeadHold | None = None) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, update_counters,
                                  stopped_lead_enabled, stopped_lead_count_max)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  # Vision either lost the lead outright or fell back to its own distance estimate, which at close
  # range reads systematically long. Prefer the radar track we were already following.
  if hold is not None and not (lead_dict['status'] and lead_dict.get('radar')):
    held = hold.candidate(tracks)
    if held is not None:
      lead_dict = held.get_RadarState(lead_msg.prob)
      hold.used = True

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  # Re-read tuning params about twice a second. Params.get() hits disk, so this must not run
  # every frame in a realtime process.
  PARAM_REFRESH_FRAMES = int(0.5 / DT_MDL)

  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.params = Params()
    self.lead_hold = RadarLeadHold()
    # The yaw estimate is noisy frame to frame and it scales a correction applied out to 50m,
    # so it is smoothed before anything is projected with it.
    self.yaw_rate_filter = FirstOrderFilter(0.0, 0.20, DT_MDL)
    self.radar_reaction_factor = 1.0
    self.frame = 0
    self.refresh_tuning()

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def refresh_tuning(self) -> None:
    """Pick up live tuning changes so options can be A/B'd between runs without a rebuild."""
    # get_bool() ignores the key's declared default and reports False until something writes
    # the param, which would silently disable this. get(return_default=True) honors it.
    self.stopped_lead_enabled = bool(self.params.get("StoppedLeadMatchEnabled", return_default=True))
    hold_ms = self.params.get("StoppedLeadHoldMs", return_default=True) or 500
    # +STOPPED_LEAD_COUNT_UP of evidence per frame, so the threshold is half the frame count
    self.stopped_lead_count_max = max(1, int((hold_ms / 1000.0) / DT_MDL))

    # Close-range radar lead hold. Distance is in cm on the param so the tuning page can offer
    # whole-metre steps without a float param; 0 disables.
    # 100% is stock behaviour. Lower reacts to the lead's acceleration sooner and holds it
    # longer; higher assumes it fades faster and responds more gently.
    self.radar_reaction_factor = (self.params.get("RadarReactionFactor", return_default=True) or 100) / 100.0

    hold_cm = self.params.get("RadarLeadHoldCm", return_default=True) or 0
    lead_hold_ms = self.params.get("RadarLeadHoldMs", return_default=True) or RADAR_LEAD_HOLD_DEFAULT_MS
    self.lead_hold.configure(hold_cm / 100.0, lead_hold_ms)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    # Only while disengaged, so a change made mid-drive lands at the next engage instead of
    # altering lead selection under the car that is already following one.
    self.frame += 1
    if not sm['selfdriveState'].enabled and self.frame % self.PARAM_REFRESH_FRAMES == 0:
      self.refresh_tuning()

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: pt for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # Turning makes a stationary object appear to slide sideways, so the forward projection
    # needs to know how fast we are turning. livePose is the calibrated estimate; the model's
    # own orientation rate stands in when it is not trustworthy yet.
    yaw_rate = 0.0
    live_pose = sm['livePose'] if 'livePose' in sm.data else None
    if live_pose is not None and live_pose.angularVelocityDevice.valid and live_pose.inputsOK and live_pose.sensorsOK:
      yaw_rate = float(live_pose.angularVelocityDevice.z)
    elif len(sm['modelV2'].orientationRate.z):
      yaw_rate = float(sm['modelV2'].orientationRate.z[0])
    yaw_rate = float(self.yaw_rate_filter.update(yaw_rate))

    # *** compute the tracks ***
    for ids in ar_pts:
      pt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = pt.vRel + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      track = self.tracks[ids]
      track.update(pt.dRel, pt.yRel, pt.vRel, v_lead, pt.measured, pt.jLead, pt.yvRel, pt.aLead,
                   self.radar_reaction_factor)

      # Lane-relative position, and where the track is heading once the turn is accounted for.
      # Both feed match_vision_to_track, so they have to be current before the match runs.
      if self.ready:
        v_rel_future, yv_rel_future = track.yaw_compensated_velocities(yaw_rate)
        track.dRel_future = track.dRel + v_rel_future * RADAR_LAT_PROJECTION_S
        track.yRel_future = track.yRel + yv_rel_future * RADAR_LAT_PROJECTION_S
        track.d_path(sm['modelV2'])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                          low_speed_override=True,
                          stopped_lead_enabled=self.stopped_lead_enabled,
                          stopped_lead_count_max=self.stopped_lead_count_max,
                          hold=self.lead_hold)
      # The hold only applies to the lead we follow; leadTwo stays purely vision-gated.
      self.lead_hold.observe(lead_one)
      self.radar_state.leadOne = lead_one
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, low_speed_override=False,
                                                 update_counters=False,
                                                 stopped_lead_enabled=self.stopped_lead_enabled,
                                                 stopped_lead_count_max=self.stopped_lead_count_max)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks', 'selfdriveState', 'livePose'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
