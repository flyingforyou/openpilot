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
  curve = 4      # the model's curvature says this corner needs less than the limit allows


# Fastest limit each road class can plausibly post, m/s. Sized from the observed pairings
# (class 1 <-> 65 mph, class 4 <-> 40/45, class 5 <-> 25/30, class 6 <-> 5/25/30) plus a margin,
# so an honest limit is never rejected and a limit left over from the previous road is. Classes
# absent here -- including 0, "unknown" -- impose no ceiling.
CLASS_CEILING = {
  1: 75 * CV.MPH_TO_MS,  # freeway / controlled access
  4: 50 * CV.MPH_TO_MS,  # arterial
  5: 35 * CV.MPH_TO_MS,  # collector
  # 35, not 30. The original sizing had class 6 posting 5/25/30, so 30 looked like the top of
  # the range; over 421k frames since, **35 is the most common limit on a class-6 road** -- 9725
  # frames against 8378 for 30. Rejecting it threw away the commonest residential limit as stale,
  # and the unmapped fallback then held the target wherever it already was: a measured minute at
  # 25 mph on a 35 mph road. Exactly 35 rather than 35-plus-a-margin, because this number is also
  # the cap the fallback holds under, and _with_offset crosses from +5 to +10 at 40 -- a ceiling
  # of 40 would quietly let an unmapped residential street target 50.
  6: 35 * CV.MPH_TO_MS,  # local / residential
}

RAMP_ON = 1
RAMP_OFF = 2

# A higher target has to hold this long before the setpoint takes it. Covers the case
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

# Slack on the class cross-check. Posted limits arrive as Float32 and the ceilings are computed
# in float64, so a 35 mph limit reaches us as 15.646400451660156 against a ceiling of
# 15.646400000000002 -- greater by 4.5e-7, which was enough to throw away every 35 on a
# residential road. Limits live on a whole-mph grid, so a mile an hour of slack is free.
CLASS_TOLERANCE = 1 * CV.MPH_TO_MS

# How far below the other sources baseSpeedLimit has to sit before it is treated as a bad match
# rather than as early news. The observed failures were 25 and 45 mph below; ordinary source
# disagreement is under 10.
OUTVOTED_MARGIN = 15 * CV.MPH_TO_MS
# ...and how closely those other sources have to agree with each other for their vote to count.
SOURCE_AGREEMENT = 5 * CV.MPH_TO_MS

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
# How far ahead the model's curvature is read for the curve cap, seconds. Sized when the fleet
# signal was the other half of this cap -- it leads a corner by a median 4.0s over 173 segments,
# and matching it kept the two inputs talking about the same bend. The fleet no longer caps here,
# but the number stands on its own: shorter and the cap arrives after the car is already in the
# corner, longer and a bend well down the road holds the straight before it down.
CURVE_LOOKAHEAD_T = 4.0

# Where the map's own road-ahead cubic takes over from the model, metres.
#
# Both were scored the same way over route 00000073: what each predicted for a point d metres
# ahead, against the curvature the car was actually steering once it had driven those d metres
# (controlsState.curvature, which comes from the steering angle, so it is neither's own output).
# Under 60m the model is the more accurate of the two and should be -- it is pointed at the road.
# At 100m it has effectively stopped seeing bends: of 835 real ones it called 15%, against the
# map's 72%, and on the ones it did call its error was 78% higher. 60m is where they cross.
#
# The ordering holds on the other two routes measured (0000006e, 0000007d): the map wins at 100m
# on all three, and the model's recall there collapses everywhere -- 15%, 6%, 0%.
#
# The map earns the cap rather than just winning the comparison: at 100m its false-alarm rate on
# genuinely straight road is 1.1% at worst across the three, and the speed a cap driven off it
# would allow there is 58mph even at the 1st percentile -- below 45 on 0.07% of straights. It
# cannot make the car crawl, which is the only way a map source touching a speed cap goes wrong.
#
# What it is not, yet, is load-bearing. Replayed over all three routes at the default 3.0 m/s^2
# it binds below the speed actually driven on ~0% of frames; at 2.5 it binds on 0.2-0.3%, costing
# a median 3.5-5.4mph at around 75mph. None of those drives contains a highway bend tight enough
# to need it. This is a protection that did not have to fire, not one measured saving time.
MAP_CURVE_NEAR = 60.0

OFFSET_SPLIT = 40 * CV.MPH_TO_MS
OFFSET_BELOW = 5 * CV.MPH_TO_MS
OFFSET_ABOVE = 10 * CV.MPH_TO_MS


class MapCruiseController:
  def __init__(self):
    self.enabled = False
    self.offset_ratio = 1.0
    self.use_curve = True
    self.use_map_curve = True
    self.sync_cluster = False
    self.curve_lat_accel = 0.0
    self.v_max = 129 * CV.KPH_TO_MS

    self.state = MapCruiseState.off
    self.source = 'off'
    self.v_target = 0.0     # what the map says, before the caps below it
    self.v_ceiling = 0.0    # v_target after every cap: the number a cluster should show as MAX
    self.v_output = 0.0     # what is handed to the planner
    self.raise_timer = 0.0
    self.loss_timer = 0.0
    self.last_posted = 0.0
    self.curve_from_map = False   # which of the two saw the bend that is capping


  def set_config(self, enabled: bool, offset_ratio: float,
                 v_max: float, use_curve: bool = True, sync_cluster: bool = False,
                 curve_lat_accel: float = 0.0, use_map_curve: bool = True) -> None:
    if not enabled and self.enabled:
      self.reset()
    self.enabled = enabled
    self.offset_ratio = offset_ratio
    self.use_curve = use_curve
    self.sync_cluster = sync_cluster
    self.curve_lat_accel = curve_lat_accel
    self.use_map_curve = use_map_curve
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
    self.curve_from_map = False

  def _posted_limit(self, nav) -> tuple[float, str]:
    """Best posted limit available, and where it came from. 0.0 if none is trustworthy.

    Ordered by how quickly each source admits the road has changed, which is the opposite of how
    precise each one looks. baseSpeedLimit is a plain m/s number and moves first; mapSpeedLimit is
    a band and moves last; mpp and fused sit in between and are only reached when the map itself
    has nothing, which in practice means a road the map does not know.
    """
    if nav.splineConfidence < MIN_CONFIDENCE or not nav.gpsRoadMatch:
      return 0.0, 'none'

    # Being first is not the same as being right. Over three drives base agrees with the posted
    # band 91.6% of the time, but on 0.44% of frames it reads 15+ mph below every other source
    # while those agree with each other -- a class-4 road posting 65 with base claiming 40 or 20.
    # Believing it there put the car at 25 mph with the traffic around it doing far more.
    #
    # Dropped rather than replaced: the other sources are not thereby right either. A 65 on a
    # class-4 road is still refused by the cross-check below, and what the car falls back to is
    # the held target under that class's own ceiling -- which is the honest answer when no source
    # is trustworthy, and is not 25.
    #
    # Never on a ramp. Base dropping ahead of the others is exactly the early warning this module
    # was built around, and 539 of the observed frames were on off-ramps: overriding base there
    # would hold a freeway limit down an exit, which is the failure the whole file exists to
    # avoid.
    base = float(nav.baseSpeedLimit)
    others = [float(v) for v in (nav.mapSpeedLimit, nav.mppSpeedLimit, nav.fusedSpeedLimit) if v > 0.0]
    if (base > 0.0 and not nav.rampType and len(others) >= 2
        and min(others) - base > OUTVOTED_MARGIN
        and max(others) - min(others) <= SOURCE_AGREEMENT):
      base = 0.0

    for value, name in ((base, 'base'), (nav.mapSpeedLimit, 'map'),
                        (nav.mppSpeedLimit, 'mpp'), (nav.fusedSpeedLimit, 'fused')):
      if value > 0.0:
        return float(value), name
    return 0.0, 'none'

  def _limit_offset(self, limit: float) -> float:
    """How far over this limit to sit. See OFFSET_SPLIT.

    **The driver's offset configured in the car is deliberately not consulted here.** It arrives
    as `navMap.speedOffset` (UI_userSpeedOffset) and this module ignores it on purpose -- nothing
    reads that field, and nothing should start. It is one number for every road, which is the
    thing this ladder exists to fix: on this car it reads +10 in 99.9% of logged frames, so
    honouring it would only ever mean "ignore the ladder below 40mph", which is exactly where the
    ladder matters most. Differentiating the offset by posted limit is the intended behaviour, so
    a target that sits +5 rather than +10 under 40mph is correct and not a bug to chase.
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

  def _map_curvature(self, nav, v_ego: float) -> float:
    """Tightest curvature the map's cubic puts past MAP_CURVE_NEAR, 1/m. 0.0 if it cannot say.

    The window starts where the model stops being the better source and ends at the same 4s
    horizon the model is read over, so both are describing the same stretch of road and the far
    edge stays a function of speed. Below about 33mph that horizon is inside MAP_CURVE_NEAR and
    the map contributes nothing, which is the intended answer rather than a gap: at town speed
    the model can see the whole distance that matters.

    The cubic's curvature, 2 c2 + 6 c3 x, is linear in x, so its extreme over the window is at
    one end or the other and two evaluations find it exactly.
    """
    if not self.use_map_curve or nav.curvHealth <= 0 or nav.curvRange <= 0.0:
      return 0.0
    # Never read past what the message says it describes; beyond that the cubic is extrapolation.
    far = min(v_ego * CURVE_LOOKAHEAD_T, float(nav.curvRange))
    if far <= MAP_CURVE_NEAR:
      return 0.0
    c2, c3 = float(nav.curvC2), float(nav.curvC3)
    return max(abs(2.0 * c2 + 6.0 * c3 * MAP_CURVE_NEAR), abs(2.0 * c2 + 6.0 * c3 * far))

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
    if posted > 0.0 and ceiling > 0.0 and posted > ceiling + CLASS_TOLERANCE:
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
    # Curve deceleration, from the model's own curvature. This is the only geometry cap: it is
    # read at a point rather than averaged over a segment, so it lets go the moment the road
    # straightens. Across 10.3k on-ramp frames its median is 113mph and it binds 5.1% of the
    # time -- but on the 14.2% of those frames that are genuinely tight it says 27.3mph against
    # a fleet reading 38.6 and a car actually driven through at 27.8. That is the loop at the
    # start of an on-ramp, and leaving it to the driver's brake pedal is not a merge protection,
    # just a gap. It runs everywhere, ramps included.
    #
    # The fleet used to cap here too, on every non-ramp road, on the theory that it stands in for
    # the bends the point curvature misses. It does not: measured over 183k engaged non-ramp
    # frames from four drives, the correlation between lateral acceleration and
    # (fleetSplineSpeed - posted limit) is **+0.043** -- no relationship, and the wrong sign for
    # a curve signal. The fleet reads 3.4-6.2mph *above* the posted limit in every lateral-accel
    # band including the tightest, and the share of frames where it falls below the limit goes
    # *down* as the road tightens (22% under 0.25 m/s^2, 0% over 2.0). What it actually tracks is
    # traffic, lights and intersections, which the lead-following MPC already handles from the
    # radar. Applied as a cap it bound 93-97% of straight-road frames and cost a median 3.3mph on
    # free-flowing freeway, 11.7mph on a congested arterial -- there dragging the target below the
    # posted limit outright. So the fleet stays what it is good for: the ramp branch's own target,
    # where no posted limit applies and the segment average is the only signal there is.
    #
    # The map's own cubic joins the model here rather than replacing it, split by distance: the
    # model owns everything inside MAP_CURVE_NEAR, the map everything past it, and the cap takes
    # whichever of the two is tighter. Tighter is the safe direction -- curve speed falls as
    # curvature rises, so max() over the sources is min() over the speeds they would allow, and
    # a source that has nothing to say returns 0 and cannot raise the cap.
    map_curvature = self._map_curvature(nav, v_ego)
    self.curve_from_map = map_curvature > curvature
    curvature = max(curvature, map_curvature)

    v_curve = self._curve_speed(curvature) if self.use_curve else 0.0
    if 0.0 < v_curve < target:
      target = v_curve
      self.state = MapCruiseState.curve
      self.source = 'curve'
    else:
      self.curve_from_map = False

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

    # The setpoint is the answer, not the route to it. How fast the car closes on a new limit is
    # the planner's decision -- it has acceleration and jerk limits tuned for this car, and a
    # rate limit here only ever competes with them. It used to lose that competition badly:
    # SLEW_UP was 0.5 m/s^2 against CruiseMaxVals of 1.3-2.0, so a 45->55 change measured 11 s
    # to arrive while the cluster had shown the new number within half a second.
    #
    # What is still worth holding is the decision itself. A limit that rises has to stay risen
    # before it is believed, or the setpoint chases every flicker in the map; a limit that drops
    # is taken at once, because that one is not a suggestion. An off-ramp never gets the raise:
    # the limit of the road being left does not apply to the ramp. Merging is the opposite and
    # skips the wait, since reaching the speed of the road being joined is the whole point.
    if self.v_target < self.v_output:
      self.raise_timer = 0.0
      self.v_output = self.v_target
    elif self.v_target > self.v_output:
      if ramp == RAMP_OFF:
        self.raise_timer = 0.0
      else:
        self.raise_timer = RAISE_DWELL if ramp == RAMP_ON else self.raise_timer + DT_MDL
      if self.raise_timer >= RAISE_DWELL:
        self.v_output = self.v_target
    else:
      self.raise_timer = 0.0

    return float(np.clip(self.v_output, MIN_TARGET, self.v_max))
