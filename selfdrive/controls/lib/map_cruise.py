"""Automatic cruise setpoint from the car's own navigation map.

The Tesla gateway broadcasts three views of the road ahead, and they do not agree about when the
road changes. Measured over a 26-minute drive (27 segments, route 00000030), leaving a freeway:

    t+0.0s   UI_rampType        0 -> 2      (off-ramp)
    t+0.0s   UI_baseMapSpeedLimitMPS 29 -> 0 (m/s; "the map has no limit for this piece of road")
    t+0.8s   UI_roadClass       1 -> 4      (freeway -> arterial)
    t+11.0s  UI_controlledAccess 1 -> 0
    t+15.0s  UI_mapSpeedLimit   65 -> 40 mph

Seventy-five seconds separate the first signal from the last, and for all seventy-five of them
UI_mapSpeedLimit still said 65 mph -- through the ramp, through a full stop, and onto a 40 mph
surface street. Anything that reads the posted limit alone and sets cruise from it would have
held a freeway setpoint down an exit ramp. That single observation is what this module is shaped
around: the posted limit is the *slowest* signal the car has, so it is used for confirmation, and
the fast signals decide when to stop believing it.

Three ideas do the work:

  * baseSpeedLimit before mapSpeedLimit. Across all six limit changes in that drive it led the
    posted band by 0 to 14.6 seconds and never once lagged, and it is a real number in m/s rather
    than a "less than or equal to" band.

  * roadClass cross-checks the limit. A 65 mph limit reported while roadClass says arterial is a
    limit belonging to a road we already left. Over the whole drive that contradiction held for
    96 seconds, and all 96 were the two ramps -- no false positives anywhere else.

  * fleetSplineSpeed carries the ramp. It is indexed by position along the road spline rather
    than by road segment, so it sweeps continuously where a posted limit can only step. Down the
    off-ramp it read 51 -> 45 -> 41 -> 40 -> 36 -> 29 mph, leading the driver's own speed by four
    to eight seconds the whole way; up an on-ramp it swept 21 -> 73 mph, staying 8-15 mph ahead
    of the car. It is what the fleet actually does here, which on a ramp is the only description
    of the road that exists.

What comes out replaces the cruise target outright, up as well as down, bounded by a configured
ceiling. It has to be the ceiling and not the stalk: this car is pcmCruise, so v_cruise *is* the
stalk value, and capping the map with it would mean the setpoint could never exceed whatever was
dialled in on the last street -- reaching a freeway with 30 mph on the stalk would hold 30. Asking
the driver to pre-set 80 mph before pulling out of a residential street is not a feature, it is
the same manual work with extra steps.

So the bound is TeslaMapAutoSpeedMax, set once, and the map moves freely underneath it. The stalk
is the trigger rather than a setpoint: engaging cruise hands the road to the map, and from there
the map owns the number. The curve controller sits downstream and lowers the result further,
which is what keeps a map limit from being carried into a corner it does not fit.
"""
from enum import IntEnum

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL


class MapCruiseState(IntEnum):
  off = 0        # disabled, or no usable map data at all
  mapped = 1     # a posted limit we believe, cross-checked against road class
  ramp = 2       # on a ramp: no posted limit applies, following what the fleet does here
  unmapped = 3   # no limit and no ramp: coasting on the last value under a road-class ceiling
  curve = 4      # the fleet drives this stretch slower than the limit allows -- capped to it


# Fastest limit each road class can plausibly post, m/s. Sized from the observed pairings
# (class 1 <-> 65 mph, class 4 <-> 40/45, class 5 <-> 25/30, class 6 <-> 5/25/30) plus a margin,
# so an honest limit is never rejected and a limit left over from the previous road is. Classes
# absent here -- including 0, "unknown" -- impose no ceiling.
CLASS_CEILING = {
  1: 75 * CV.MPH_TO_MS,  # freeway / controlled access
  4: 50 * CV.MPH_TO_MS,  # arterial
  5: 35 * CV.MPH_TO_MS,  # collector
  6: 30 * CV.MPH_TO_MS,  # local / residential
}

RAMP_ON = 1
RAMP_OFF = 2

# Setpoint slew, m/s^2. Asymmetric on purpose: dropping the target is a safety action and should
# land inside a few seconds, raising it is a comfort action and should feel like a decision rather
# than a twitch. 1.0 takes 65 -> 40 mph in 11s, close to the 0.8 the driver used down that ramp.
# 0.5 takes 40 -> 65 mph in 22s, well inside what the MPC will actually deliver.
SLEW_DOWN = 1.0
SLEW_UP = 0.5
# Merging is the exception. An on-ramp's fleet profile climbs faster than any comfort rate for a
# setpoint -- 21 to 73 mph over the observed ramp -- and a setpoint that lags it is a setpoint
# below the car's own speed, which reads as a brake request in the middle of a merge.
SLEW_UP_MERGE = 1.5

# A higher target has to hold this long before the setpoint starts climbing. Covers the case
# where the map hands over to the next road a beat before the car is on it -- entering that
# freeway, the posted limit went 25 -> 65 mph while the car was still at 18 mph on the on-ramp.
RAISE_DWELL = 3.0

# How long a posted limit may go missing before the state actually changes. All four sources
# blink to zero for a frame or two while the map re-localises -- seg 0 of the reference drive did
# it twice in forty seconds -- and reacting to each blink makes the setpoint hunt.
LOSS_DEBOUNCE = 1.0

# Confidence floor for trusting a posted limit. UI_splineLocConfidence sat at 99 for essentially
# the whole drive and dipped only on genuine re-localisations, so this rejects little, but a
# limit read against the wrong road is exactly the failure worth refusing.
MIN_CONFIDENCE = 60

# Slowest the map is ever allowed to ask for. Below this it is the planner's stopping logic, the
# lead car or the curve controller doing the work -- not a speed limit.
MIN_TARGET = 20 * CV.KPH_TO_MS

# Hard ceiling over the posted limit, applied last and to everything -- the map's own target, a
# ramp's fleet speed, and the driver's stalk alike. It exists because the pieces above it can be
# argued into a number the road cannot justify: with cluster sync on, openpilot's own stalk
# presses were read as the driver disagreeing, that pinned the target to the stalk, and the two
# walked each other from 25mph up to the configured ceiling of 80. Nothing in the chain was
# anchored to the road. This is that anchor, and it is deliberately dumb.
#
# Only applied where there is a posted limit to measure against. On a ramp or an unmapped road
# there is nothing to add 10mph to, and clamping to a stale number there is worse than not
# clamping at all.
# How far over the posted limit to target, by the limit itself. A flat +10 is most of the limit
# again on a 25 zone, and the fleet does not drive it: over the whole log set, fleetSplineSpeed
# runs +4.1 over a 25 (359k samples) and +4.7 over a 35 (669k), against +7.7 over a 65 (1.6M).
# Two steps rather than a fitted curve -- the fleet signal carries traffic and intersections as
# well as pace, so it reads -7.1 on a congested 30 and +0.4 on a 55, and is not clean enough to
# justify anything finer.
# How far ahead the model's curvature is read for the curve cap, seconds. The fleet signal
# leads a curve by a median 4.0s (158 of 173 segments), so matching it keeps the two inputs
# talking about the same corner. Shorter and the cap arrives after the car is already in it;
# longer and a bend well down the road holds the straight before it down.
CURVE_LOOKAHEAD_T = 4.0

OFFSET_SPLIT = 40 * CV.MPH_TO_MS
OFFSET_BELOW = 5 * CV.MPH_TO_MS
OFFSET_ABOVE = 10 * CV.MPH_TO_MS

# How far above the car's own speed the setpoint may always sit, m/s, regardless of slew in
# either direction. Both limits need it and they need to agree on it. Raising: a setpoint the
# slew has left below the car's actual speed is a brake request, which is what a rate limit on
# the way up buys if it is applied in absolute terms -- pulling away from a light onto a 40 mph
# road, 0.5 m/s^2 on the setpoint is slower than the launch. Lowering: a setpoint loitering far
# above a car that has already slowed keeps a stale target alive for no reason. Four and a half
# mph of headroom is not an acceleration request in either direction, and the MPC's own accel
# limits still decide what the car actually does with it.
TRACK_MARGIN = 2.0
# Rate the setpoint closes on that band, m/s^2. Faster than either ordinary slew, because inside
# the band the setpoint is not asking the car for anything -- but not instant: a setpoint that
# teleports five mph releases a decel request in a single frame, and the MPC should be smoothing
# a ramp, not a step.
SLEW_CATCHUP = 3.0


class MapCruiseController:
  def __init__(self):
    self.enabled = False
    self.offset_ratio = 1.0
    self.use_curve = True
    self.sync_cluster = False
    self.curve_lat_accel = 0.0
    self.v_max = 129 * CV.KPH_TO_MS

    self.state = MapCruiseState.off
    self.source = 'off'
    self.v_target = 0.0     # what the map says, before slew
    self.v_ceiling = 0.0    # v_target after every cap: the number a cluster should show as MAX
    self.v_output = 0.0     # what is handed to the planner, after slew
    self.raise_timer = 0.0
    self.loss_timer = 0.0
    self.last_posted = 0.0


  def set_config(self, enabled: bool, offset_ratio: float,
                 v_max: float, use_curve: bool = True, sync_cluster: bool = False,
                 curve_lat_accel: float = 0.0) -> None:
    if not enabled and self.enabled:
      self.reset()
    self.enabled = enabled
    self.offset_ratio = offset_ratio
    self.use_curve = use_curve
    self.sync_cluster = sync_cluster
    self.curve_lat_accel = curve_lat_accel
    self.v_max = v_max

  def reset(self) -> None:
    self.v_ceiling = 0.0
    self.state = MapCruiseState.off
    self.source = 'off'
    self.v_target = 0.0
    self.v_output = 0.0
    self.raise_timer = 0.0
    self.loss_timer = 0.0
    self.last_posted = 0.0

  def _posted_limit(self, nav) -> tuple[float, str]:
    """Best posted limit available, and where it came from. 0.0 if none is trustworthy.

    Ordered by how quickly each source admits the road has changed, which is the opposite of how
    precise each one looks. baseSpeedLimit is a plain m/s number and moves first; mapSpeedLimit is
    a band and moves last; mpp and fused sit in between and are only reached when the map itself
    has nothing, which in practice means a road the map does not know.
    """
    if nav.splineConfidence < MIN_CONFIDENCE or not nav.gpsRoadMatch:
      return 0.0, 'none'
    for value, name in ((nav.baseSpeedLimit, 'base'), (nav.mapSpeedLimit, 'map'),
                        (nav.mppSpeedLimit, 'mpp'), (nav.fusedSpeedLimit, 'fused')):
      if value > 0.0:
        return float(value), name
    return 0.0, 'none'

  def _limit_offset(self, limit: float) -> float:
    """How far over this limit to sit. See OFFSET_SPLIT.

    The car's own UI_userSpeedOffset used to feed this. It is one number for every road, which
    is the thing the ladder exists to fix, and on this car it read +10 in 99.9% of logged frames
    -- so honouring it only ever meant "ignore the ladder below 40mph", which is where the ladder
    matters most. The ladder is the whole answer now.
    """
    return OFFSET_ABOVE if limit >= OFFSET_SPLIT else OFFSET_BELOW

  def _with_offset(self, limit: float) -> float:
    """Posted limit -> what to actually target."""
    return limit * self.offset_ratio + self._limit_offset(limit)

  def _fleet_speed(self, nav) -> float:
    """What the fleet drives at this point on the road, 0.0 if nothing is reported.

    The spline value is preferred wherever it exists: it is sampled at the car's position along
    the road rather than averaged over the whole segment, so it is the only one of these that
    changes shape partway down a ramp. The quartile is the fallback for roads the spline has no
    coverage for, and the median gets a margin because half the fleet is above it by definition.
    """
    if nav.fleetSplineSpeed > 0.0:
      return float(nav.fleetSplineSpeed)
    if nav.fleetTopQuartileSpeed > 0.0:
      return float(nav.fleetTopQuartileSpeed)
    if nav.fleetMedianSpeed > 0.0:
      return float(nav.fleetMedianSpeed) * 1.15
    return 0.0

  def _curve_speed(self, curvature: float) -> float:
    """Fastest this corner may be taken, m/s, or 0.0 when the criterion is off.

    A corner of radius R taken at v asks v**2/R of the tyres sideways. Bounding that at
    curve_lat_accel gives sqrt(bound / curvature) as the speed it may be taken at. The bound
    sits near this car's measured p99 lateral acceleration (median 0.40, p99 3.43, p99.9 4.54,
    steering's own limit 5.0), so it binds on hairpins and essentially nowhere else.

    This exists because the fleet speed cannot see a hairpin: the spline is a segment average,
    so the tightest point is flattened. Measured at radius 31m the fleet said 28.5mph, this says
    21.4, and the car was actually driven through at 17.3.
    """
    if self.curve_lat_accel <= 0.0 or curvature <= 1e-5:
      return 0.0
    return float(np.sqrt(self.curve_lat_accel / curvature))

  def update(self, CS, v_ego: float, v_cruise_driver: float, curvature: float = 0.0) -> float:
    """Returns the cruise target in m/s, or 0.0 when the map has nothing to say.

    This is the target, not a cap: the caller uses it in place of the stalk value, so it raises
    the setpoint as well as lowering it. Everything that bounds it is in here -- the configured
    ceiling, the road-class cross-check, the dwell before any raise, and the per-limit offset.
    """
    nav = CS.navMap
    if not self.enabled or not nav.valid:
      self.reset()
      self.v_ceiling = 0.0
      return 0.0

    posted, source = self._posted_limit(nav)
    ceiling = CLASS_CEILING.get(int(nav.roadClass), 0.0)
    ramp = int(nav.rampType)

    # The cross-check. A posted limit above what this class of road can carry is a limit for the
    # road behind us, and holding it is precisely the failure this module exists to avoid.
    if posted > 0.0 and ceiling > 0.0 and posted > ceiling:
      posted, source = 0.0, 'stale'

    # Debounce only a limit that vanishes, never one that appears: coming back is information,
    # going away for two frames usually is not. A limit rejected as stale skips the debounce --
    # that one is a decision, not a dropout.
    if posted > 0.0:
      self.loss_timer = 0.0
      self.last_posted = posted
    elif source != 'stale' and self.last_posted > 0.0 and self.loss_timer < LOSS_DEBOUNCE:
      self.loss_timer += DT_MDL
      posted, source = self.last_posted, 'blink'
    else:
      self.last_posted = 0.0

    fleet = self._fleet_speed(nav)

    if ramp in (RAMP_ON, RAMP_OFF) and fleet > 0.0:
      # A ramp has no posted limit of its own and the two roads it joins disagree, so neither is
      # usable. The fleet is, and on an off-ramp it is already decelerating before the car is.
      # No class ceiling here: road class is ambiguous for the whole length of a ramp (it read
      # "arterial" down the entire merge onto a 65 mph freeway), so a ceiling drawn from it would
      # cap the setpoint below merge speed at exactly the wrong moment.
      self.state = MapCruiseState.ramp
      self.source = 'ramp'
      target = fleet
      if ramp == RAMP_ON:
        # Never ask for less than the car is already doing while joining traffic.
        target = max(target, v_ego)
    elif posted > 0.0:
      self.state = MapCruiseState.mapped
      self.source = source
      target = self._with_offset(posted)
    else:
      # No limit, no ramp: an unmapped road, a parking lot, or the map still catching up. Keep
      # what we had but never above what this class of road can be, so a freeway setpoint cannot
      # survive onto a surface street even if the ramp was never flagged. The ceiling gets the
      # same offset treatment a posted limit would, or it would fight the driver's own offset
      # every time the map blinked.
      self.state = MapCruiseState.unmapped
      self.source = source if source == 'stale' else 'hold'
      target = self.v_output if self.v_output > 0.0 else v_cruise_driver
      if ceiling > 0.0:
        target = min(target, self._with_offset(ceiling))

    # Curve deceleration, from what the fleet actually drives here.
    #
    # A ramp already takes its target from the fleet outright, above; this is the same signal
    # applied to every other road, and only ever downward. It has to be a cap rather than a
    # setpoint because fleetSplineSpeed answers "how fast do people go here", which is bent by
    # traffic and intersections as well as by geometry -- measured against the model's own
    # curvature demand across 173 segments it correlates 0.66, good enough to slow for but not
    # to steer by, and on straights it frequently reads *above* the posted limit.
    #
    # It leads the curve rather than reporting it: cross-correlating the two over those segments
    # puts the best fit at a median 4.0s ahead (p25 3.0, p75 4.5), leading in 158 of 173. The
    # lead is in time rather than distance -- 3.5s/63m at 40mph against 4.0s/100m at 56mph --
    # so it does not shrink at speed, and 4s covers even the largest observed deficit (11.1mph)
    # at a comfortable 1.25 m/s^2. Slewing it is still SLEW_DOWN's job, not this line's.
    # Two inputs, whichever is lower. They miss different things: the fleet carries traffic,
    # lights and everything else the map cannot see but flattens the tightest point of a bend;
    # the curvature limit is the bend itself but says nothing about why else a road is slow. On
    # a straight the curvature limit reads around 100mph and the fleet governs; in a hairpin it
    # is the fleet that is too high and the curvature governs.
    #
    # Skipped on an on-ramp only. Merging must not be slowed, but an off-ramp is exactly where
    # the fleet's segment average is worst -- 19 logged exits needed a median 24.3mph against a
    # fleet that bottomed out at 33.6.
    if self.use_curve and ramp != RAMP_ON:
      caps = [c for c in (fleet, self._curve_speed(curvature)) if c > 0.0]
      if caps:
        cap = min(caps)
        if cap < target:
          target = cap
          self.state = MapCruiseState.curve
          self.source = 'curve'

    # The configured ceiling is the only thing standing between the map and the throttle, so it
    # is applied to the target itself rather than anywhere further down, where a later rule could
    # step over it.
    self.v_target = float(np.clip(target, MIN_TARGET, self.v_max))

    # The stalk is the trigger, not a setpoint. Engaging cruise is how the driver hands this
    # road to the map; from there the map owns the number, and a later press is just how the
    # cluster's own MAX gets moved. Treating a press as a standing override is what pinned MAX
    # to the last number dialled and stopped it following the road at all.

    # Last word. Everything above this line can be walked somewhere the road does not support --
    # a stale hold, a ramp's fleet speed carried past the merge -- and the posted limit is the
    # only number here that comes from the road rather than from the loop. Same ladder as the
    # target's own offset, so the cap cannot be looser than what the map would have asked for.
    if posted > 0.0:
      cap = posted + self._limit_offset(posted)
      if self.v_target > cap:
        self.v_target = float(max(MIN_TARGET, cap))
        self.source = 'capped'

    # Everything above has had its say; what is left is the ceiling for this stretch of road.
    # Deliberately taken before the slew: the slew is how the car gets there, not how fast the
    # road allows. A cluster showing this is answering "how fast may I go here", which only
    # changes when the road does -- not every frame as the car catches up.
    self.v_ceiling = self.v_target

    # First frame with anything to say: start from the driver's set speed rather than from zero,
    # so engaging on a road we already know does not slew up from a standstill target.
    if self.v_output <= 0.0:
      self.v_output = max(min(v_cruise_driver, self.v_target), v_ego)

    v_prev = self.v_output
    if self.v_target < self.v_output:
      self.raise_timer = 0.0
      self.v_output = max(self.v_target, self.v_output - SLEW_DOWN * DT_MDL)
      # Never leave the setpoint loitering above a speed the car is nowhere near; without this
      # the slew keeps a stale high target alive for seconds after the car has already slowed.
      band = max(self.v_target, v_ego + TRACK_MARGIN)
      self.v_output = min(self.v_output, max(band, v_prev - SLEW_CATCHUP * DT_MDL))
    elif self.v_target > self.v_output:
      # Raising takes a settled target, and on an off-ramp it never comes: the limit of the road
      # being left is not a limit that applies. Merging is the opposite case and gets no dwell at
      # all, because the whole point of an on-ramp is to reach the speed of the road being joined.
      merging = ramp == RAMP_ON
      if ramp == RAMP_OFF:
        self.raise_timer = 0.0
      else:
        self.raise_timer = RAISE_DWELL if merging else self.raise_timer + DT_MDL
      if self.raise_timer >= RAISE_DWELL:
        slew = SLEW_UP_MERGE if merging else SLEW_UP
        self.v_output = min(self.v_target, self.v_output + slew * DT_MDL)
      # The slew governs the headroom above the car, never the car itself. Whatever it has
      # reached, the setpoint may always close on TRACK_MARGIN over the current speed -- the
      # dwell still applies, since that band moves only as fast as the car does.
      band = min(self.v_target, v_ego + TRACK_MARGIN)
      self.v_output = max(self.v_output, min(band, v_prev + SLEW_CATCHUP * DT_MDL))
    else:
      self.raise_timer = 0.0

    return float(np.clip(self.v_output, MIN_TARGET, self.v_max))
