"""An alternative lead-following path built from the car-following literature, off by default.

The carrot MPC stays in charge of everything else; this only proposes the acceleration while a
lead is being followed, and the MPC keeps the last word on hard braking.

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

IDM_T = 1.0            # s, time gap
IDM_S0 = 3.5           # m, standstill gap
IDM_B = 2.0            # m/s^2, comfortable deceleration
IDM_DELTA = 4.0
CAH_COOLNESS = 0.99    # Kesting 2010's c
FLOW_TAU = 5.0         # s, low-pass on lead speed that defines the "flow" speed
FLOW_MARGIN = 1.5      # m/s allowed above the flow speed
FLOW_FADE_KPH = (50.0, 70.0)   # full smoothing below, none above
FREE_DECEL_FLOOR = -0.5        # exceeding the flow speed only ever coasts, never brakes hard
MPC_OVERRIDE_BELOW = -2.0      # the MPC takes over whenever it wants harder braking than this


def idm_cah_accel(v: float, v_lead: float, a_lead: float, s: float, v0: float,
                  a_max: float, b: float = IDM_B, T: float = IDM_T, s0: float = IDM_S0,
                  delta: float = IDM_DELTA, c: float = CAH_COOLNESS) -> float:
  """Kesting 2010's ACC acceleration: IDM blended with the constant-acceleration heuristic."""
  a = max(a_max, 0.1)
  s = max(s, 0.5)
  v0 = max(v0, 0.5)
  dv = v - v_lead
  s_star = s0 + max(0.0, v * T + v * dv / (2.0 * math.sqrt(a * b)))
  if v <= v0:
    a_free = a * (1.0 - (v / v0) ** delta)
  else:
    # Treiber's IIDM form above v0, floored: running a little over the flow speed should coast
    a_free = max(FREE_DECEL_FLOOR, -b * (1.0 - (v0 / v) ** (a * delta / b)))
  a_idm = a_free - a * (s_star / s) ** 2

  al = min(a_lead, a)
  if v_lead * dv <= -2.0 * s * al:
    den = v_lead ** 2 - 2.0 * s * al
    a_cah = (v * v * al / den) if den > 1e-3 else al
  else:
    a_cah = al - (dv ** 2) * (1.0 if dv > 0 else 0.0) / (2.0 * s)

  if a_idm >= a_cah:
    return float(a_idm)
  return float((1.0 - c) * a_idm + c * (a_cah + b * math.tanh((a_idm - a_cah) / b)))


def flow_v0(flow_speed: float, v_cruise: float) -> float:
  """IDM's desired speed: the flow speed (+margin) in slow traffic, the cruise target above."""
  w = float(np.interp(flow_speed * 3.6, FLOW_FADE_KPH, [1.0, 0.0]))
  smoothed = min(v_cruise, flow_speed + FLOW_MARGIN)
  return w * smoothed + (1.0 - w) * v_cruise


def combine_with_mpc(a_research: float, a_mpc: float) -> float:
  """Research path by default; the MPC wins whenever it asks for braking beyond the override."""
  if a_mpc < MPC_OVERRIDE_BELOW and a_mpc < a_research:
    return a_mpc
  return a_research


class ResearchLongitudinal:
  def __init__(self, dt: float):
    self.dt = dt
    self.flow = None

  def reset(self):
    self.flow = None

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float,
             v_cruise: float, a_max: float) -> float | None:
    """Proposed acceleration while following a lead, or None when this path has nothing to say."""
    if not lead_present or v_cruise <= 0.0:
      self.flow = None
      return None
    if self.flow is None:
      self.flow = v_lead
    self.flow += (v_lead - self.flow) * self.dt / (FLOW_TAU + self.dt)
    v0 = flow_v0(max(self.flow, 0.0), v_cruise)
    return idm_cah_accel(v_ego, v_lead, a_lead, d_rel, v0, a_max)
