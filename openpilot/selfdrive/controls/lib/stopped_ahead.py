"""A stopped car ahead that the radar is measuring and the model has only started to see.

radard hands the planner a radar track only once the model's lead agrees with it in distance and
speed, and only with the model's probability over 0.5. For a car standing in the road that agreement
comes late: the model first reports it with a low probability, nearer than it is, and moving -- on
11/09 (025c seg 19) a car stopped 106 m ahead on a straight was read for 3 s as 47-70 m away doing
36-58 km/h while the radar held it at 0 the whole time, the car accelerated from 49 to 62 km/h at it,
and the driver braked at -5.2 m/s^2 0.45 s after it finally became the lead at 59 m.

The radar alone cannot be trusted with this: over the same drives a stationary return on the model's
path with nothing else asked of it turns up 552 times, 402 of them with no slowdown after (bridges,
gantries, signs -- 281 on the motorway). What separates
the car from those is the model saying, however unsurely, that something slow is about there:

  - the track is measured, has been for MIN_CNT frames, and is not moving (|v| < MAX_V)
  - it is within MAX_PATH_OFFSET of the model's own path at its distance
  - the model's lead has probability >= MIN_PROB, a distance within DIST_TOL of the track's, and a
    speed under VISION_SLOW of ours
  - it is further than MIN_GAP_S of travel: nearer than that the normal match is what should act,
    and a wrong pick would be a hard brake rather than an early one (0235 seg 13: a stationary
    return 1.7 m beside the path at 30 m, on a ramp bend, with the real queue 13 m behind it)

A car still rolling to a stop is the same case with a clue the standing one lacks: the radar has
watched it move, which no bridge or sign ever does. On 11/13 (0270 seg 14) a car slowing from 29 to
8 km/h round a bend sat on the model's path from 145 m, with the model at 0.02-0.25, reading it slow
and about there; it became the lead at 82 m and the car braked at -3.9. So a track seen at MOVED_V or
more, now under SLOW_FRAC of our speed, needs only MIN_PROB_MOVED from the model and may be as far
as RANGE_MOVED. Over routes 0201-0270 (31.5 h engaged) that is 23 episodes, none followed by nothing
(12 became the lead by the ordinary match up to 2.55 s later -- that one from 123 m -- and in the
other 11 the car or driver slowed within 6 s). Without the model's distance and speed agreeing it
is 44 with 5 followed by nothing, so those stay.

Three such frames latch the track; it is then the lead until the evidence has been gone as long as it
was there (at most COUNT_MAX frames), by which time the ordinary match has it.

Replayed over every engaged frame of routes 0201-0263 (17.4 h above 4 m/s): 31 episodes. In 25 the
track went on to become the lead by the ordinary match, a median 0.4 s later and up to 2.15 s; in the
other 6 the car or the driver braked within 6 s (024e seg 9: the driver, at -3.8, 0.1-1.5 s later);
none was followed by nothing. Both FCWs of 11/04-11/05 are among the 25. Each threshold moved both
ways: MIN_PROB 0.1 brings in 3 episodes with no slowdown after them and VISION_SLOW 0.8 one; the
others change the count and nothing else.

Closed loop (closedloop_sim.py, SIMRADARD, SIMENGAGED where the driver took over), peak braking with
this off / on:
  025c seg 19  -4.00 / -2.71      0235 seg 13 (ramp)  -3.47 / -2.80      024b seg 14 (FCW)  -5.96 / -4.99
  0245 seg 17 (FCW)  -5.67 / -5.08      022e seg 14 (ramp)  -4.16 / -3.73      024b seg 13 (ramp)  -3.83 / -3.49
  024e seg 7  -3.78 / -3.61      0237 seg 5  -3.03 / -2.91      024d seg 13 (ramp)  -4.09 / -4.06
The last one neither sensor has before 54 m at 54 km/h; no lead rule reaches it. follow_regression
(the 12 lurch events, 12 launches, 16 scenarios) is identical with it on.
"""
MIN_PROB = 0.2            # the model's lead probability
DIST_TOL = 0.4            # of the track's distance, between the model's lead and the track
VISION_SLOW = 0.7         # of our speed: the model has to read its lead as slower than this
MAX_V = 2.0               # m/s, the track's own speed
MAX_PATH_OFFSET = 2.0     # m from the model's path at the track's distance
MIN_CNT = 4               # frames the track has existed
MIN_GAP_S = 2.5           # s of travel to the track
RANGE = (15.0, 120.0)     # m
MIN_V_EGO = 4.0           # m/s; below it radard's low-speed override already stops for what is ahead
MOVED_V = 3.0             # m/s the radar has seen the track do: a vehicle, not a structure
SLOW_FRAC = 0.5           # of our speed: a track that has moved counts while slower than this
MIN_PROB_MOVED = 0.05     # the model's lead probability for a track that has moved
RANGE_MOVED = 150.0       # m, the far limit for one
COMMIT = 3                # frames of evidence to latch
COUNT_MAX = 10            # frames of evidence kept, so a latched track outlives a 0.5 s gap


def corroborated(v_ego, prob, vision_dist, vision_v, track) -> bool:
  """One frame of evidence that this track is a stopped car in our path."""
  if v_ego < MIN_V_EGO or vision_v >= VISION_SLOW * v_ego:
    return False
  if not track.measured or track.cnt < MIN_CNT:
    return False
  if abs(track.vLead) < MAX_V:
    min_prob, far = MIN_PROB, RANGE[1]
  elif getattr(track, 'vLeadMax', 0.0) >= MOVED_V and track.vLead < SLOW_FRAC * v_ego:
    min_prob, far = MIN_PROB_MOVED, RANGE_MOVED       # slowing to a stop: the radar saw it move
  else:
    return False
  if prob < min_prob or not (max(RANGE[0], MIN_GAP_S * v_ego) < track.dRel < far):
    return False
  return abs(track.pathOffset) < MAX_PATH_OFFSET and abs(vision_dist - track.dRel) < DIST_TOL * track.dRel


def update(v_ego, prob, vision_dist, vision_v, tracks, count=True):
  """Count the evidence on every track; returns the nearest latched one, or None."""
  best = None
  for t in tracks.values():
    if count:
      if corroborated(v_ego, prob, vision_dist, vision_v, t):
        t.stopped_ahead_count = min(t.stopped_ahead_count + 1, COUNT_MAX)
      else:
        t.stopped_ahead_count = max(t.stopped_ahead_count - 1, 0)
      if t.stopped_ahead_count >= COMMIT:
        t.stopped_ahead = True
      elif t.stopped_ahead_count == 0:
        t.stopped_ahead = False
    if t.stopped_ahead and t.measured and (best is None or t.dRel < best.dRel):
      best = t
  return best
