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
