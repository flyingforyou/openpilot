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
