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
      open-road catch-up (the 09-11 complaint) is untouched.
  (2) is how it is judged -- OpenACC-style string-stability metrics in closedloop_sim.py.
"""
import math

import numpy as np

# Kesting 2010 Table 1 ('car'): v0 120 km/h, delta 4, T 1.5 s, s0 2.0 m, a 1.4, b 2.0, c 0.99.
# T and s0 are moved to this car's current gap-1 feel (1.0 s, 4.5 m -- the stop distance it already
# uses); a, b, delta and c are the paper's.
IDM_T = 1.0            # s, time gap            (paper 1.5)
IDM_S0 = 4.5           # m, standstill gap      (paper 2.0)
IDM_A = 1.4            # m/s^2, max acceleration
IDM_B = 2.0            # m/s^2, comfortable deceleration
IDM_DELTA = 4.0
CAH_COOLNESS = 0.99    # Kesting 2010's c
BLEND_TIME = 0.5       # s, cross-fade between this path and the MPC's output on hand-over
EIDM_JMAX = 3.0        # m/s^3, Salles et al. 2020 Table 3
FLOW_TAU = 5.0         # s, low-pass on lead speed that defines the "flow" speed
FLOW_MARGIN = 1.5      # m/s allowed above the flow speed
FLOW_FADE_KPH = (50.0, 70.0)   # full smoothing below, none above
FREE_DECEL_FLOOR = -0.5        # exceeding the flow speed only ever coasts, never brakes hard


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
  if v_lead * dv <= -2.0 * s * al:
    den = v_lead ** 2 - 2.0 * s * al
    return (v * v * al / den) if den > 1e-3 else al
  return al - (dv ** 2) * (1.0 if dv > 0 else 0.0) / (2.0 * s)


def idm_cah_accel(v: float, v_lead: float, a_lead: float, s: float, v0: float,
                  a_max: float = IDM_A, b: float = IDM_B, c: float = CAH_COOLNESS,
                  z_prev: float | None = None, dt: float = 0.05, return_z: bool = False):
  """Kesting 2010 eq. (2.4) ACC acceleration, with IIDM in place of IDM as in the authors' book,
  and the EIDM jerk limit (Salles 2020 eq. 20) on the IIDM's gap ratio."""
  a = min(IDM_A, max(a_max, 0.1))
  a_iidm, z = iidm_accel(v, v - v_lead, s, v0, a=a, b=b, z_prev=z_prev, dt=dt)
  a_cah = cah_accel(v, v_lead, a_lead, s, a=a)
  if a_iidm >= a_cah:
    out = float(a_iidm)
  else:
    out = float((1.0 - c) * a_iidm + c * (a_cah + b * math.tanh((a_iidm - a_cah) / b)))
  return (out, z) if return_z else out


def flow_v0(flow_speed: float, v_cruise: float) -> float:
  """IDM's desired speed: the flow speed (+margin) in slow traffic, the cruise target above."""
  w = float(np.interp(flow_speed * 3.6, FLOW_FADE_KPH, [1.0, 0.0]))
  smoothed = min(v_cruise, flow_speed + FLOW_MARGIN)
  return w * smoothed + (1.0 - w) * v_cruise


class ResearchLongitudinal:
  def __init__(self, dt: float):
    self.dt = dt
    self.flow = None
    self.weight = 0.0        # 1 = this path, 0 = MPC; ramps over BLEND_TIME
    self.last_a = 0.0
    self.z_prev = None       # EIDM: previous gap ratio

  def reset(self):
    self.flow = None
    self.z_prev = None

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float,
             v_cruise: float, a_max: float) -> float | None:
    """Proposed acceleration while following a lead, or None when this path has nothing to say."""
    if not lead_present or v_cruise <= 0.0:
      self.flow = None
      self.z_prev = None
      return None
    if self.flow is None:
      self.flow = v_lead
    self.flow += (v_lead - self.flow) * self.dt / (FLOW_TAU + self.dt)
    v0 = flow_v0(max(self.flow, 0.0), v_cruise)
    a_out, self.z_prev = idm_cah_accel(v_ego, v_lead, a_lead, d_rel, v0, a_max,
                                       z_prev=self.z_prev, dt=self.dt, return_z=True)
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
