"""An alternative lead-following path built from the car-following literature, off by default.

The carrot MPC stays in charge of everything else; while a lead is being followed in plain
cruise/lead states, this path's acceleration is used instead (cross-faded on hand-over). An earlier
version let the MPC take over below -2.0 m/s^2; that is not in the papers and the hard switch made
braking and jerk worse in replay (00000178 seg 7: -1.95/0.74 without it vs -2.32/1.06 with), so it
was removed.

  (3) IDM + CAH -- Kesting, Treiber & Helbing (2010), "Enhanced intelligent driver model to access
      the impact of driving strategies on traffic capacity", the IDM variant its authors propose as
      the basis of a real ACC. IDM gives the gap law with its approach-rate term floored at zero;
      the constant-acceleration heuristic (CAH) stops IDM over-braking when the gap is short but the
      lead is not actually closing on us fast (a cut-in that is pulling away, say).
  (1) FollowerStopper-style smoothing -- Stern et al. (2018), "Dissipation of stop-and-go waves via
      control of autonomous vehicles". Rather than chase every lead fluctuation, drive toward the
      flow's average speed and let the gap absorb the rest. Here: IDM's desired speed v0 is a slow
      low-pass of the lead's speed in slow traffic, faded back to the cruise target by 70 km/h so
      open-road catch-up (the 09-11 complaint) is untouched. Off for mode 0 since the 10/01 road
      test (FLOW_SMOOTHING): it made launches and slow following lag far behind the lead.
  (2) is how it is judged -- OpenACC-style string-stability metrics in closedloop_sim.py.

LongResearchMode 1 swaps (3) for the time-headway FollowerStopper itself, as published in the
CIRCLES code (nathanlct/trajectory-training-icra, trajectory/env/accel_controllers.py): three
distance bands pick a commanded speed between 0, the lead's speed and the desired speed, and a
velocity loop tracks it. Its middle band is the equilibrium gap, so here that band is pinned to the
MPC's own equilibrium gap for the current gap-stalk position -- the 7-step knob keeps its meaning.
"""
import math

import numpy as np

# Kesting 2010 Table 1 ('car'): v0 120 km/h, delta 4, T 1.5 s, s0 2.0 m, a 1.4, b 2.0, c 0.99.
# T and s0 are moved to this car's current gap-1 feel (1.0 s, 4.5 m -- the stop distance it already
# uses); a, b, delta and c are the paper's.
IDM_T = 1.0            # s, time gap            (paper 1.5); fallback only, the gap stalk sets it
IDM_S0 = 4.5           # m, standstill gap      (paper 2.0); fallback only, StopDistance sets it
IDM_A = 2.0            # m/s^2, max acceleration (paper 1.4; 22 launch replays: 8 s speed 33.1 -> 35.9 km/h, above 2.0 the carrot accel limit binds)
IDM_B = 2.0            # m/s^2, comfortable deceleration
IDM_DELTA = 4.0
CAH_COOLNESS = 0.99    # Kesting 2010's c
BLEND_TIME = 0.5       # s, cross-fade between this path and the MPC's output on hand-over
EIDM_JMAX = 3.0        # m/s^3, Salles et al. 2020 Table 3
# How far below the CAH acceleration eq. (2.4) may go. Kesting uses b (2.0) throughout: when IIDM wants
# far more than CAH says is needed, the result is up to CAH - b. With seconds to spare that over-brakes
# -- a slower car met at 40 m after a lane change (TTC ~5 s) got -3.1 where the MPC asked -1.7 -- so
# the reach is CAH_BLEND_B once time-to-contact is CAH_BLEND_TTC[1] or more, and the paper's b by
# CAH_BLEND_TTC[0]. CAH only just reaches contact, so b is the margin when time is short.
#
# Gated on TTC, not on how hard CAH brakes: an earlier version relaxed whenever CAH asked for little,
# which is also every slow final approach to a stopped car, and openpilot's own longitudinal maneuvers
# (selfdrive/test/longitudinal_maneuvers, run through closedloop_sim.py with a synthetic lead) then
# stopped 1.4-2.0 m short of the lead instead of 3.1-3.5 and let a 15 m cut-in close to 4.1 m.
#   23 replays (17 lane changes, 6 stop-and-go): worst braking -3.12 -> -1.95 (MPC -2.31),
#     scenes braking >0.5 harder than the MPC 7 -> 3, amplification and jerk unchanged.
#   16 synthetic maneuvers: stops 3.0-3.4 m (paper's b 3.1-3.5), 15 m cut-in 7.1 m (7.9).
CAH_BLEND_B = 0.75
CAH_BLEND_TTC = (3.0, 5.0)
# The reach while the lead is pulling away (v_lead > v). Then CAH is the lead's own acceleration and
# IIDM's (s*/s)^2 holds us back until the gap has opened -- from a stop the gap starts at s0, so the
# launch waits on the gap. None = CAH_BLEND_B.
# 22 launch replays (10/01 + 10/02, flow smoothing off, IDM_A 2.0): pull-away delay 0.77 -> 0.22 s
# (MPC 0.55), first-4 s accel +1.02 -> +1.28 (MPC +1.27), gap opened 19.3 -> 11.6 m (MPC 12.3). The
# speed 8 s in is 36.4 against the MPC's 38.0 because the MPC overshoots the lead there and brakes
# back (median scene: 41 km/h against a 36 km/h lead, then -0.39); this peaks at 39.6. 0.2: 0.30 s,
# +1.22, 13.2 m. Braking toward a slower car is untouched -- it needs v_lead > v.
CAH_OPEN_B = 0.1
# Output jerk limit, ISO 15622's ACC comfort bound: 2.5 m/s^3 above 20 m/s, 5 below 5 m/s. EIDM only
# limits rising acceleration, so the brake side and input noise went straight through: jerk RMS 1.00
# against the MPC's 0.42 over the 23 replays. Two exceptions keep it from costing safety: never brake
# less than CAH's collision-avoidance minimum (JerkLimiter), and no brake-side limit at all under
# JERK_FREE_TTC seconds to contact -- without that a 15 m cut-in closed to 6.0 m instead of 7.1.
# Replays: plan jerk RMS 1.00 -> 0.75, scenes braking >0.5 harder than the MPC 3 -> 1; the synthetic
# maneuvers' distances are unchanged. Measured on the car's actual acceleration (LongControl and the
# fitted actuator lag in the loop) the research path's jerk is 0.47 against the MPC's 0.57.
#
# USE_VLEADK's own effect is small. An earlier figure (0.75 -> 0.71) came from leadTwo alone: the sim
# copied the raw speed over leadOne's vLeadK. Fixed and re-run: plan jerk 0.75 -> 0.68, actual jerk
# 0.48 -> 0.47, worst actual decel -1.97 -> -2.07 (the filter's lag), brake onset unchanged.
JERK_LIMIT_BP = (5.0, 20.0)
JERK_LIMIT_V = (5.0, 2.5)
JERK_FREE_TTC = 3.0
USE_VLEADK = True        # the lead's Kalman-filtered speed (radard) rather than the raw radar speed
FLOW_TAU = 5.0         # s, low-pass on lead speed that defines the "flow" speed
FLOW_MARGIN = 1.5      # m/s allowed above the flow speed
FLOW_FADE_KPH = (50.0, 70.0)   # full smoothing below, none above
FREE_DECEL_FLOOR = -0.5        # exceeding the flow speed only ever coasts, never brakes hard
# Flow smoothing on IIDM's v0 (mode 0). Off: from a stop the flow is ~0, so v0 starts near 5 km/h and
# climbs with FLOW_TAU -- the 10/01 road test's "doesn't follow" (launch 8 s speed 24.5 km/h against
# the MPC's 38.0, time gap 5.0 s during launches). Off: 33.1 km/h; the 25 stop-and-go replays brake
# -2.11 instead of -1.93 on average (MPC -2.49). FollowerStopper (mode 1) still uses the flow.
FLOW_SMOOTHING = False

# Time-headway FollowerStopper (CIRCLES reference implementation). Band k sits at
#   dv_-^2 / (2 d_k) + max(dx0_k, h_k * v),   dv_- = min(v_lead - v, 0)
# with h = 0.4/0.6/0.8 s, dx0 = 4.5/5.25/6.0 m, d = 1.5/1.0/0.5 m/s^2. Here the middle band's
# max(dx0, h v) is replaced by the MPC's equilibrium gap s_eq (stop distance + t_follow v + k v^2),
# and the outer two keep the paper's ratios to it: the standstill part scales by dx0_k/dx0_2 and the
# speed part by h_k/h_2.
FS_H = (0.4, 0.6, 0.8)
FS_DX0 = (4.5, 5.25, 6.0)
FS_D = (1.5, 1.0, 0.5)
FS_MAX_ACCEL = 1.5     # m/s^2, reference max_accel
FS_MAX_DECEL = 3.0     # m/s^2, reference max_deaccel (also the failsafe's braking assumption)
FS_MIN_GAP = 2.5       # m, reference safe_velocity min_gap
# The reference steps its sim at 0.1 s and commands (v_cmd - v)/0.1 -- instant speed tracking; on
# the road (Stern 2018) v_cmd went to the car's own cruise loop. This is that loop's gain.
FS_KV = 1.0            # 1/s
FS_REACTION = 0.1      # s, the failsafe's reaction delay (the reference's one sim step)
# Stern 2018 sets v_des to the speed the wave should be smoothed to, not the driver's set speed; the
# flow estimate below (faded to the cruise target above 70 km/h) plays that role.
FS_USE_FLOW = True
# Stern 2018 fed v_cmd to the stock cruise loop, which answers like a first-order lag, not the
# instant (v_cmd - v)/0.1 of the sim reference. Used directly as an accel command the piecewise
# v_cmd law is a jerk source (RMS 2-8 m/s^3 in replay), so this lag stands in for that loop. It
# fixes the jerk but the failsafe then acts through the lag: 1 s left 0.4 m to the lead (000000f8
# seg 20) and 2 s left 0.1 m (seg 22) in replay. Off; kept only to reproduce that.
FS_LOOP_TAU = 0.0      # s, 0 = off


def eidm_limit_ratio(z: float, z_prev: float | None, dt: float, a: float, j_max: float = EIDM_JMAX) -> float:
  """Salles et al. 2020, eq. (20): limit how fast s*/s may FALL.

  z_t = sqrt(z_{t-dt}^2 - dt*j_max/a)   if z_{t-dt}^2 - z_t^2 > dt*j_max/a,   else z_t.

  The interaction term is a*z^2, so this caps how quickly acceleration can rise when the gap jumps
  open or the desired gap drops -- the paper's case of junctions and lane changes, where s and s*
  change instantaneously. A rising z (gap closing, a cut-in) passes straight through, so braking is
  never delayed. Motivating replay: 00000178 seg 18, where the lead track alternated each frame
  between a car 36 m ahead and one 25 m ahead and the unlimited output flipped +0.29 <-> -2.10.
  """
  if z_prev is None:
    return z
  step = dt * j_max / max(a, 0.1)
  if z_prev * z_prev - z * z > step:
    return math.sqrt(max(z_prev * z_prev - step, 0.0))
  return z


def iidm_accel(v: float, dv: float, s: float, v0: float, a: float = IDM_A, b: float = IDM_B,
               T: float = IDM_T, s0: float = IDM_S0, delta: float = IDM_DELTA,
               z_prev: float | None = None, dt: float = 0.05) -> tuple[float, float]:
  """Improved IDM (Treiber & Kesting 2013), the car-following core of their ACC model.

  Plain IDM's a*[1 - (v/v0)^d - (s*/s)^2] lets the free-road term shrink the equilibrium gap's
  stiffness near v0, so followers settle ever further back -- the "unrealistic equilibrium gaps at
  high speed" the IDM review lists. IIDM splits the state space in four so the equilibrium gap is
  exactly s* whatever v0 is:
    v <= v0:  z >= 1 -> a(1 - z^2)            z < 1 -> a_free(1 - z^(2a/a_free))
    v >  v0:  z >= 1 -> a_free + a(1 - z^2)   z < 1 -> a_free
  with z = s*/s, s* = s0 + max(0, vT + v*dv/(2 sqrt(ab))), and above v0 a_free = -b(1-(v0/v)^(a d/b)).
  """
  s = max(s, 0.5)
  v0 = max(v0, 0.5)
  s_star = s0 + max(0.0, v * T + v * dv / (2.0 * math.sqrt(a * b)))
  z = eidm_limit_ratio(s_star / s, z_prev, dt, a)
  if v <= v0:
    a_free = a * (1.0 - (v / v0) ** delta)
    if z >= 1.0:
      return a * (1.0 - z * z), z
    if a_free <= 1e-6:
      return 0.0, z
    return a_free * (1.0 - z ** (2.0 * a / a_free)), z
  a_free = max(FREE_DECEL_FLOOR, -b * (1.0 - (v0 / v) ** (a * delta / b)))
  if z >= 1.0:
    return a_free + a * (1.0 - z * z), z
  return a_free, z


def cah_accel(v: float, v_lead: float, a_lead: float, s: float, a: float = IDM_A) -> float:
  """Constant-acceleration heuristic, Kesting 2010 eq. (2.3)."""
  s = max(s, 0.5)
  al = min(a_lead, a)
  dv = v - v_lead
  den = v_lead ** 2 - 2.0 * s * al
  # den -> 0 only for a lead that is stopped with zero acceleration, where the first case is 0/0. Its
  # limit is the second case, -v^2/(2s): stop within the gap. (MovSim's ACC.java takes the same
  # branch.) Returning al there said a stopped car needs no braking at all.
  if v_lead * dv <= -2.0 * s * al and den > 1e-3:
    return v * v * al / den
  return al - (dv ** 2) * (1.0 if dv > 0 else 0.0) / (2.0 * s)


def idm_cah_accel(v: float, v_lead: float, a_lead: float, s: float, v0: float,
                  a_max: float = IDM_A, b: float | None = None, c: float | None = None,
                  z_prev: float | None = None, dt: float = 0.05, return_z: bool = False,
                  T: float = IDM_T, s0: float = IDM_S0):
  """Kesting 2010 eq. (2.4) ACC acceleration, with IIDM in place of IDM as in the authors' book,
  and the EIDM jerk limit (Salles 2020 eq. 20) on the IIDM's gap ratio."""
  b = IDM_B if b is None else b
  c = CAH_COOLNESS if c is None else c
  a = min(IDM_A, max(a_max, 0.1))
  a_cah = cah_accel(v, v_lead, a_lead, s, a=a)
  blend_b = b
  if CAH_BLEND_B is not None:
    ttc = max(s, 0.5) / max(v - v_lead, 1e-3)
    blend_b = float(np.interp(ttc, CAH_BLEND_TTC, [b, CAH_BLEND_B]))
    if CAH_OPEN_B is not None and v_lead > v:
      blend_b = CAH_OPEN_B
  def acc(zp):
    a_iidm, z = iidm_accel(v, v - v_lead, s, v0, a=a, b=b, T=T, s0=s0, z_prev=zp, dt=dt)
    if a_iidm >= a_cah:
      return float(a_iidm), z
    return float((1.0 - c) * a_iidm + c * (a_cah + blend_b * math.tanh((a_iidm - a_cah) / blend_b))), z

  out, z = acc(z_prev)
  if return_z == 'cah':
    return out, z, a_cah
  return (out, z) if return_z else out


class JerkLimiter:
  """Rate-limit the command, but never below what collision avoidance needs: when holding back the
  brake would leave it gentler than CAH's minimum, the CAH deceleration goes through at once."""
  def __init__(self, dt: float):
    self.dt = dt
    self.prev = None

  def reset(self):
    self.prev = None

  def limit(self, a: float, a_floor: float, v: float, ttc: float = 99.0) -> float:
    if JERK_LIMIT_V is None or self.prev is None:
      self.prev = a
      return a
    step = float(np.interp(v, JERK_LIMIT_BP, JERK_LIMIT_V)) * self.dt
    lo = -1e9 if ttc < JERK_FREE_TTC else self.prev - step
    out = min(max(a, lo), self.prev + step)
    if out > a and out > a_floor:
      out = max(a, a_floor)
    self.prev = out
    return out


def gap_idm_params(v: float, s_eq: float, stop_distance: float) -> tuple[float, float]:
  """IDM's s0 and T from the gap stalk: s0 is the stop distance and T the time gap that puts IDM's
  steady-state gap s0 + vT on the MPC's own equilibrium gap s_eq for this gap position and speed.
  s_eq <= 0 means no gap information (tests, callers that predate it): the fixed defaults."""
  if s_eq <= 0.0:
    return IDM_T, IDM_S0
  s0 = max(stop_distance, 0.5)
  return max(s_eq - s0, 0.0) / max(v, 0.1), s0


def flow_v0(flow_speed: float, v_cruise: float) -> float:
  """IDM's desired speed: the flow speed (+margin) in slow traffic, the cruise target above."""
  w = float(np.interp(flow_speed * 3.6, FLOW_FADE_KPH, [1.0, 0.0]))
  smoothed = min(v_cruise, flow_speed + FLOW_MARGIN)
  return w * smoothed + (1.0 - w) * v_cruise


def fs_bands(v: float, v_lead: float, s_eq: float, stop_distance: float) -> tuple[float, float, float]:
  """FollowerStopper's three distance thresholds, middle one at the MPC's equilibrium gap."""
  dvm = min(v_lead - v, 0.0)
  span = max(s_eq - stop_distance, 0.0)
  return tuple(dvm * dvm / (2.0 * d) + stop_distance * dx0 / FS_DX0[1] + span * h / FS_H[1]
               for h, dx0, d in zip(FS_H, FS_DX0, FS_D, strict=True))


def fs_safe_velocity(v_lead: float, s: float) -> float:
  """Continuous form of the reference failsafe: the fastest speed from which, after FS_REACTION,
  braking at FS_MAX_DECEL still stops FS_MIN_GAP short of where the lead stops at the same rate."""
  b, tr = FS_MAX_DECEL, FS_REACTION
  room = s + v_lead * v_lead / (2.0 * b) - FS_MIN_GAP
  if room <= 0.0:
    return 0.0
  return -b * tr + math.sqrt((b * tr) ** 2 + 2.0 * b * room)


def follower_stopper_accel(v: float, v_lead: float, s: float, v_des: float, s_eq: float,
                           stop_distance: float, a_max: float = FS_MAX_ACCEL) -> float:
  dx1, dx2, dx3 = fs_bands(v, v_lead, s_eq, stop_distance)
  vl = min(max(v_lead, 0.0), v_des)
  if s <= dx1:
    v_cmd = 0.0
  elif s <= dx2:
    v_cmd = vl * (s - dx1) / max(dx2 - dx1, 1e-3)
  elif s <= dx3:
    v_cmd = vl + (v_des - vl) * (s - dx2) / max(dx3 - dx2, 1e-3)
  else:
    v_cmd = v_des
  v_cmd = min(v_cmd, fs_safe_velocity(v_lead, s))
  return float(np.clip(FS_KV * (v_cmd - v), -FS_MAX_DECEL, min(FS_MAX_ACCEL, max(a_max, 0.1))))


def more_binding(*accels: float | None) -> float | None:
  """The acceleration that respects every lead: the smallest of those that have an opinion.

  leadTwo carries what the MPC already treats as a second obstacle -- a car the radar-only cut-in
  detector sees merging (cut_in.py) and the target lane's lead during our own lane change. Following
  only leadOne here would silently switch that anticipation off whenever this path is in charge.
  """
  vals = [a for a in accels if a is not None]
  return min(vals) if vals else None


class ResearchLongitudinal:
  def __init__(self, dt: float):
    self.dt = dt
    self.flow = None
    self.weight = 0.0        # 1 = this path, 0 = MPC; ramps over BLEND_TIME
    self.last_a = 0.0
    self.z_prev = None       # EIDM: previous gap ratio
    self.fs_a = None         # FollowerStopper: lagged output
    self.last_cah = 0.0      # CAH acceleration of the last update (the jerk limiter's floor)

  def reset(self):
    self.flow = None
    self.z_prev = None
    self.fs_a = None

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float,
             v_cruise: float, a_max: float, mode: int = 0, s_eq: float = 0.0,
             stop_distance: float = IDM_S0) -> float | None:
    """Proposed acceleration while following a lead, or None when this path has nothing to say.

    mode 0: IIDM + CAH with flow smoothing, its time gap and standstill gap taken from s_eq, the
    MPC's equilibrium gap at the current gap position. mode 1: FollowerStopper around s_eq."""
    if not lead_present or v_cruise <= 0.0:
      self.flow = None
      self.z_prev = None
      self.fs_a = None
      return None
    if self.flow is None:
      self.flow = v_lead
    self.flow += (v_lead - self.flow) * self.dt / (FLOW_TAU + self.dt)
    v0 = flow_v0(max(self.flow, 0.0), v_cruise)
    if mode == 1:
      a_fs = follower_stopper_accel(v_ego, v_lead, d_rel, v0 if FS_USE_FLOW else v_cruise, s_eq,
                                    stop_distance, a_max)
      if FS_LOOP_TAU <= 0.0 or self.fs_a is None:
        self.fs_a = a_fs
      else:
        self.fs_a += (a_fs - self.fs_a) * self.dt / (FS_LOOP_TAU + self.dt)
      return self.fs_a
    T, s0 = gap_idm_params(v_ego, s_eq, stop_distance)
    if not FLOW_SMOOTHING:
      v0 = v_cruise
    a_out, self.z_prev, self.last_cah = idm_cah_accel(v_ego, v_lead, a_lead, d_rel, v0, a_max,
                                                      z_prev=self.z_prev, dt=self.dt, return_z='cah', T=T, s0=s0)
    return a_out

  def blend(self, a_research: float | None, a_mpc: float) -> float:
    """Cross-fade to and from the MPC so a hand-over never steps the command.

    Not from the papers -- they have one controller. Here two coexist, and the hand-over produced
    jerk spikes (RMS 0.21 -> 1.55 on a highway replay) until it was faded.
    """
    step = self.dt / BLEND_TIME
    if a_research is not None:
      self.last_a = a_research
      self.weight = min(1.0, self.weight + step)
    else:
      self.weight = max(0.0, self.weight - step)
    if self.weight <= 0.0:
      return a_mpc
    return self.weight * self.last_a + (1.0 - self.weight) * a_mpc
