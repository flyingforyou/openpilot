"""What happens to the MPC's acceleration in acc mode before it reaches the car, in one place.

The planner and tests/test_follow_quality.py both run this, so the quality test exercises exactly
the chain and the exact parameter mapping the car drives with. Order matters and is the planner's:
lead-brake accel cap, brake-follow, release-on-opening, brake-onset limiter, approach coast.
(The factory cut-in cap stays in the planner: it needs carState's factory objects.)
"""
from openpilot.selfdrive.controls.lib import carrot_t_follow as ctf
from openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc import desired_follow_distance


def configure(get_int, get_bool, stop_distance: float) -> None:
  """Params -> module switches. get_int/get_bool take a param name."""
  bj = get_int("BrakeOnsetJerk")
  ctf.BRAKE_ONSET_JERK = bj / 10.0 if bj > 0 else None
  # Start braking when the lead does. 1 = sized to the buffer, 2..100 = a fixed share of the
  # lead's deceleration, 0 = off.
  k = get_int("LeadBrakeFollow")
  # the gap left after the lead stops: LeadBrakeBufferCm when set, otherwise the stop distance
  keep = get_int("LeadBrakeBufferCm") / 100.0
  ctf.LEAD_BRAKE_BUFFER = (-0.5, keep if keep > 0 else float(stop_distance), 0.25, 60.0) if k == 1 else None
  ctf.LEAD_BRAKE_FOLLOW = (k / 100.0, -0.5, 40.0) if k > 1 else None
  ctf.RELEASE_ON_OPENING = (0.3, 4.0, 0.0) if (k == 1 and ctf.RELEASE_WITH_BUFFER) else None
  ctf.APPROACH_COAST = ctf.APPROACH_COAST_ON if get_bool("ApproachCoast") else None


class FollowShaping:
  def __init__(self, dt: float):
    self.brake_follow = ctf.BrakeFollow(dt)
    self.release_open = ctf.ReleaseOnOpening(dt)
    self.brake_onset = ctf.BrakeOnsetLimiter(dt)
    self.approach_coast = ctf.ApproachCoast(dt)

  def update(self, a_mpc: float, lead, lead_two, v_ego: float, stop_distance: float, desired_distance: float,
             t_follow: float, comfort_brake: float, comfort_brake_2: float) -> float:
    """lead / lead_two: radarState leads (present, radar, dRel, vLead, aLeadK)."""
    a = ctf.lead_brake_accel_cap(a_mpc, bool(lead.present), float(lead.dRel), float(lead.aLeadK))
    a = self.brake_follow.update(a, bool(lead.present), float(lead.dRel), float(lead.aLeadK), v_ego, float(lead.vLead))
    a = self.release_open.update(a, bool(lead.present), float(lead.dRel), v_ego, float(lead.vLead), float(desired_distance))
    # time to contact with the nearer of leadOne / leadTwo, for BrakeOnsetLimiter's gate
    ttc_min, d_min = 99.0, 999.0
    for ld in (lead, lead_two):
      if ld.present:
        d_min = min(d_min, float(ld.dRel))
        closing = v_ego - float(ld.vLead)
        if closing > 0.5:
          ttc_min = min(ttc_min, float(ld.dRel) / closing)
    a = self.brake_onset.update(a, bool(lead.present), float(lead.dRel), float(lead.aLeadK), v_ego, float(lead.vLead),
                                float(stop_distance), ttc_min, d_min)
    # leadTwo too when a radar track backs it: that is the car merging in (cut_in.py) or the one
    # ahead in the lane being entered (target_lane_lead) -- neither is worth accelerating into.
    near = []
    if ctf.APPROACH_COAST is not None:
      for ld in (lead, lead_two):
        if ld.present and (ld is lead or ld.radar):
          v_l = float(ld.vLead)
          near.append((float(ld.dRel), float(ld.aLeadK), v_l,
                       float(desired_follow_distance(v_l, v_l, comfort_brake, stop_distance, t_follow, comfort_brake_2)), v_ego))
    return self.approach_coast.update(a, near)
