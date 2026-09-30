import numpy as np

T_FOLLOW_RISE_RATE = 0.30  # seconds of time gap per second


def ramp_t_follow(target: float, current: float, dt: float) -> float:
  """Apply increases progressively while keeping gap reductions immediate.

  Ramping only the increases stops a sudden jump in the demanded gap (which would command a hard
  brake) while still letting the gap close instantly when it should. The rate used to be raised
  during decel-boost; that boost was removed, so a single steady rate is all that's needed.
  """
  if target <= current:
    return float(target)
  return float(min(target, current + T_FOLLOW_RISE_RATE * dt))


# jerk_factor multiplies both a_change_cost and J_EGO_COST in the MPC, so a small value makes
# changing acceleration cheap. carrot derives it from the gap position alone -- 0.5 at gap 1 up to
# 1.0 at gap 7 -- which welds "follow closely" to "change acceleration abruptly". In slow traffic
# that is the whole complaint: on 000000f8 seg 14-23 (9.5 engaged minutes, 95% under 40 km/h, gap
# position 2 so factor 0.567) the command crossed +/-0.5 m/s^2 every 9.4 s, hitting the +2.0
# ceiling and -3.16 at crawl speed.
LOW_SPEED_JERK_BP = [10.0, 40.0]


def low_speed_jerk_factor(jerk_factor: float, v_ego_kph: float, floor_at_rest: float) -> float:
  """Raise the MPC's smoothness cost in slow traffic, without touching the follow distance.

  The floor fades linearly to nothing by LOW_SPEED_JERK_BP[1], so highway behaviour is untouched,
  and it only ever raises the factor -- a gap position that already asks for a smooth ride keeps
  what it asked for. floor_at_rest <= 0 disables it.
  """
  if floor_at_rest <= 0.0:
    return jerk_factor
  floor = float(np.interp(v_ego_kph, LOW_SPEED_JERK_BP, [floor_at_rest, 0.0]))
  return max(jerk_factor, floor)


# Acceleration ceiling by distance to the lead, for stop-and-go.
#
# When the lead is faster than us the desired gap collapses -- the formula credits the lead with
# v_lead**2 / (2*b2) of room -- so a car 12 m back can see a +12 m gap error and floor the
# throttle. In stop-and-go the lead then brakes again inside a couple of seconds and we arrive
# carrying the speed: on route 00000178 seg 6 that ran as a 7 s cycle of +1.67 at 12-15 m followed
# by -2.31, over and over. Speed does not separate this from the open-road case the 40 km/h accel
# knot was raised for (lead pulling away at 40-67 m), but distance does: replayed with a 1.0 cap,
# 9.9% of today's lead-following accel frames are trimmed against 1.1% on the open-road route, and
# nothing beyond 35 m on either. Positive side only -- braking is never limited by this.
CLOSE_LEAD_DIST_BP = [10.0, 35.0]     # m: full cap at or inside the first, none past the second
CLOSE_LEAD_SPEED_BP = [40.0, 60.0]    # km/h: the cap fades out over this band


def close_lead_accel_cap(a_max: float, d_rel: float | None, v_ego_kph: float, cap: float) -> float:
  """a_max, lowered while a lead is close in slow traffic. cap <= 0 or no lead leaves it alone."""
  if cap <= 0.0 or d_rel is None or d_rel <= 0.0 or cap >= a_max:
    return a_max
  near = float(np.interp(d_rel, CLOSE_LEAD_DIST_BP, [cap, a_max]))
  weight = float(np.interp(v_ego_kph, CLOSE_LEAD_SPEED_BP, [1.0, 0.0]))
  return min(a_max, weight * near + (1.0 - weight) * a_max)
