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


# The desired gap credits the lead with its own braking distance, v_lead**2 / (2*b2), on the idea
# that a lead which brakes also takes room to stop. At equal speeds it cancels our own braking term
# and all is well. When the lead is FASTER than us the credit outgrows it and the target collapses:
# on route 00000178 seg 6, ego 19 km/h behind a lead at 29 km/h, the formula gave 13.4 m of need and
# 13.7 m of credit -- a target of about zero, so a car 12 m back saw a +12 m error and ran to the
# accel ceiling. In stop-and-go the lead stops again within seconds and the speed has to come off
# at -2.3 to -2.9.
#
# Capping the credited lead speed at our own speed means a faster lead is only ever assumed to be
# as fast as we are. The target can then only get larger, never smaller, than before. Closed-loop
# replay of the carrot MPC (simulated ego, recorded lead) on that route:
#
#                        seg6 accel / brake     seg7 accel / brake     large cmd steps (seg7)
#   uncapped, Kalman     +1.69 / -2.68          +1.58 / -2.52          51
#   capped,   radar      +1.05 / -1.42          +1.27 / -1.58          11
#
# and on the 09-11 open-road case -- lead pulling away at 30+ m -- the result is unchanged, since
# the accel ceiling binds long before the target does at that range.
# Experiments (sim only until chosen):
# LEAD_CREDIT_MARGIN -- credit a faster lead up to ego speed + this (m/s) instead of ego speed, so the
#   target still shrinks a little behind a pulling-away lead (some catch-up) but cannot collapse.
# LEAD_BRAKE_ACCEL_CAP -- (aLead at which the cap is 0, aLead at which it lifts): stop accelerating
#   once the radar sees the lead braking, instead of driving on into it. None = off.
LEAD_CREDIT_MARGIN = 0.0
# LEAD_CREDIT_WHEN_ACCEL -- credit the full lead speed while the lead is still accelerating away
#   (aLead above this, m/s^2), cap otherwise. None = off.
LEAD_CREDIT_WHEN_ACCEL = None
LEAD_BRAKE_ACCEL_CAP = None
# BRAKE_TF_SCALE -- (aLead points, t_follow multipliers): while the lead brakes, use part of the
#   time-gap buffer instead of braking hard to hold the full gap. Cuts apply at once; the gap comes
#   back through ramp_t_follow's slow rise. None = off.
BRAKE_TF_SCALE = None
# BRAKE_CB_SCALE -- (aLead points, comfort_brake multipliers): while the lead brakes, assume we may
#   stop a little harder than the comfortable 2.16, which shrinks our own stopping-distance term.
BRAKE_CB_SCALE = None
# LEAD_BRAKE_FOLLOW -- (k, aLead threshold, max distance): once the radar sees the lead braking, brake
#   at least k x its deceleration straight away instead of first waiting for the gap to justify it.
#   Starting early and gently is what lets the time-gap buffer absorb the rest. None = off.
LEAD_BRAKE_FOLLOW = None
LEAD_BRAKE_CAP_DIST = 40.0


def lead_speed_for_credit(v_lead, v_ego: float, enabled: bool, a_lead: float = 0.0):
  """The lead speed to credit in the stopped-equivalence term: capped at ego speed when enabled."""
  if not enabled or (LEAD_CREDIT_WHEN_ACCEL is not None and a_lead > LEAD_CREDIT_WHEN_ACCEL):
    return v_lead
  return np.minimum(v_lead, max(float(v_ego), 0.0) + LEAD_CREDIT_MARGIN)


def brake_t_follow_scale(t_follow: float, lead_present: bool, a_lead: float) -> float:
  """Shrink the time gap while the lead is braking (see BRAKE_TF_SCALE)."""
  if BRAKE_TF_SCALE is None or not lead_present:
    return t_follow
  bp, k = BRAKE_TF_SCALE
  return t_follow * float(np.interp(a_lead, bp, k))


def brake_comfort_scale(comfort_brake: float, lead_present: bool, a_lead: float) -> float:
  if BRAKE_CB_SCALE is None or not lead_present:
    return comfort_brake
  bp, k = BRAKE_CB_SCALE
  return comfort_brake * float(np.interp(a_lead, bp, k))


# LEAD_BRAKE_BUFFER -- (aLead threshold, standstill gap m, reaction s, max distance m): the buffer-sized
#   version of LEAD_BRAKE_FOLLOW. Instead of a fixed share of the lead's deceleration, brake at once at
#   the gentlest constant rate that still stops `standstill gap` behind the lead if it keeps braking at
#   its current rate all the way to a stop, after `reaction` seconds. A long gap gives a gentle start, a
#   short one a firm start, and the whole buffer is used without ever planning to end up closer than the
#   standstill gap. None = off.
LEAD_BRAKE_BUFFER = None


def buffer_brake_accel(d_rel: float, a_lead: float, v_ego: float, v_lead: float,
                       standstill: float, reaction: float) -> float:
  """Gentlest constant deceleration that stops `standstill` behind a lead braking to a stop at a_lead."""
  lead_stop = v_lead * v_lead / (2.0 * max(-a_lead, 0.1))
  room = d_rel + lead_stop - standstill - v_ego * reaction
  if room <= 0.1:
    return -10.0
  return -(v_ego * v_ego) / (2.0 * room)


def lead_brake_follow(a_target: float, lead_present: bool, d_rel: float, a_lead: float, v_ego: float, v_lead: float) -> float:
  """Start braking as soon as the radar sees the lead braking (LEAD_BRAKE_FOLLOW / LEAD_BRAKE_BUFFER)."""
  if not lead_present or v_ego < 1.0 or v_lead > v_ego + 2.0:
    return a_target
  if LEAD_BRAKE_BUFFER is not None:
    thr, standstill, reaction, dmax = LEAD_BRAKE_BUFFER
    if a_lead >= thr or d_rel > dmax:
      return a_target
    return min(a_target, buffer_brake_accel(d_rel, a_lead, v_ego, v_lead, standstill, reaction))
  if LEAD_BRAKE_FOLLOW is None:
    return a_target
  k, thr, dmax = LEAD_BRAKE_FOLLOW
  if a_lead >= thr or d_rel > dmax:
    return a_target
  return min(a_target, k * a_lead)


def lead_brake_accel_cap(a_target: float, lead_present: bool, d_rel: float, a_lead: float) -> float:
  """No positive acceleration toward a lead that is braking (see LEAD_BRAKE_ACCEL_CAP)."""
  if LEAD_BRAKE_ACCEL_CAP is None or not lead_present or d_rel > LEAD_BRAKE_CAP_DIST or a_target <= 0.0:
    return a_target
  cap = float(np.interp(a_lead, LEAD_BRAKE_ACCEL_CAP, [0.0, a_target]))
  return min(a_target, cap)
