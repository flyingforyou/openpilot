#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, COMFORT_BRAKE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, MAX_LATERAL_ACCEL_NO_ROLL
from openpilot.selfdrive.controls.lib.curve_speed.curve_speed_controller import CurveSpeedController
from openpilot.selfdrive.controls.lib.curve_speed.lateral_load_governor import LateralLoadGovernor
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
PARAM_REFRESH_FRAMES = int(0.5 / DT_MDL)  # params hit disk; don't read every frame
MIN_ALLOW_THROTTLE_SPEED = 2.5
V_SOFT_LEAD_SPEED = 1.0  # m/s; v_soft only engages approaching a stopped/slow lead, not a moving one

# Grey-zone throttle hold. radard publishes a lead only once the model's leadsV3[0].prob clears
# 0.5, so below that the planner sees clear road and accelerates. Route 0000001b seg 10: engaged
# with no lead, accelerated at the full 2.0 m/s^2 for 5.4s (51 -> 65 km/h) while prob crept
# 0.01 -> 0.47 toward a STOPPED car, which was finally published at 61m closing at 17.6 m/s. The
# decel needed then ran past ACCEL_MIN 0.5s later and the driver had to brake. Holding 51 km/h
# would have needed only ~2.0 m/s^2. So when there is real but sub-threshold evidence of
# something ahead, stop adding speed -- braking stays vision-gated, since the radar carries
# in-path stationary clutter continuously and braking on it would be phantom braking.
# The band is cheap: 0.2 <= prob < 0.5 is 2.5% of engaged driving over 148 logged minutes.
GREY_LEAD_PROB_MAX = 0.5   # radard's publish gate; above this there is a real lead to plan against
LEAD_CREEP_CONFIRM_TIME = 0.20      # s; reject one-frame radar velocity spikes
LEAD_CREEP_MAX_EGO_SPEED = 2.5      # m/s; this is only a stop-and-go aid
LEAD_CREEP_MAX_CLOSING_SPEED = 0.5  # m/s; never suppress a stop while rapidly closing
LEAD_CREEP_MIN_ROOM = 1.0           # m beyond configured stop distance before stop suppression is allowed
LEAD_CREEP_TRACK_JUMP_D = 5.0       # m; vision-only leads all use track id -1, so also guard continuity
LEAD_CREEP_TRACK_JUMP_V = 3.0       # m/s; a different lead must restart confirmation

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego, launch_accel: float = A_CRUISE_MAX_VALS[0]):
  """Ceiling on requested acceleration, by speed.

  launch_accel raises the standstill end of the curve, scaling the sub-10m/s points with it and
  leaving the highway end alone. Pulling away from a stop is where the stock table felt slow:
  1.6 m/s^2 against 3.5 of braking authority, and panda allows 2.0. Deceleration is untouched --
  the two are separate knobs on purpose, since only acceleration was the complaint.
  """
  scale = launch_accel / A_CRUISE_MAX_VALS[0]
  vals = [min(v * scale, ACCEL_MAX) if bp <= 10.0 else v
          for v, bp in zip(A_CRUISE_MAX_VALS, A_CRUISE_MAX_BP, strict=True)]
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, vals))

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


def combine_should_stop(mpc_should_stop: bool, e2e_should_stop: bool, experimental_mode: bool,
                        creep_follow_active: bool, mpc_stop_is_lead: bool) -> bool:
  """Creep-follow may clear only a lead0/MPC stop; preserve cruise/forced and e2e stops."""
  clear_mpc_stop = creep_follow_active and mpc_stop_is_lead
  mpc_should_stop = mpc_should_stop and not clear_mpc_stop
  return (e2e_should_stop or mpc_should_stop) if experimental_mode else mpc_should_stop


def is_mpc_stop_clearable_by_creep(source, force_slow_decel: bool, v_cruise: float,
                                   v_ego_stopping: float) -> bool:
  """Only an ordinary lead0 stop may be released by a confirmed creeping lead."""
  return (source == LongitudinalPlanSource.lead0 and
          not force_slow_decel and
          np.isfinite(v_cruise) and
          v_cruise > v_ego_stopping)


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True
    self.curve = CurveSpeedController()     # feedforward curve-speed profile + backward pass
    self.governor = LateralLoadGovernor()   # reactive lateral-load backstop + throttle-fade interlock

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    # Tesla legacy gap setting doesn't reliably re-announce itself on every boot (may be a
    # momentary request rather than a continuous status). Persist the last known value so a
    # fresh boot uses it immediately instead of silently falling back to the personality tFollow.
    self.params = Params()
    self.tesla_last_gap_adjust = self.params.get("TeslaLastGapAdjust", return_default=True) if CP.brand == "tesla" else 0

    self.frame = 0
    self.launch_accel = A_CRUISE_MAX_VALS[0]
    self.lead_creep_frames = 0
    self.lead_creep_track_id = -1
    self.lead_creep_prev_d_rel = 0.0
    self.lead_creep_prev_v_lead_k = 0.0
    self.lead_creep_active = False
    self.grey_lead_hold = False
    self.refresh_tuning()

  def refresh_tuning(self) -> None:
    """Only called while disengaged, so a change made mid-drive lands at the next engage
    rather than shifting the target distance under the car that is already following."""
    gap_profile = int(self.params.get("GapProfile", return_default=True) or 0)
    rise_pct = int(self.params.get("TFollowRiseRatePct", return_default=True) or 35)
    stop_cm = int(self.params.get("StopDistanceCm", return_default=True) or 600)
    launch_cms = int(self.params.get("LaunchAccelCms", return_default=True) or 160)
    self.launch_accel = min(launch_cms / 100.0, ACCEL_MAX)
    # Dynamic follow: how many seconds of tFollow the lead's jerk may add/remove. 0 disables it.
    dyn_tf = int(self.params.get("DynamicTFollowGain", return_default=True) or 0)
    self.mpc.set_tuning(gap_profile, rise_pct / 100.0, stop_cm / 100.0, dyn_tf / 100.0)
    # Roll with a still-moving lead: minimum lead speed (m/s) above which a full stop is not latched,
    # so the car keeps a low crawl behind a creeping lead instead of stopping then catching up. 0 = off.
    self.lead_creep_follow = int(self.params.get("LeadCreepFollowCms", return_default=True) or 0) / 100.0
    # Precise stop (paired with the velocity PID): v_soft caps the approach speed to a stopped lead.
    self.precise_stop = self.CP.brand == "tesla" and self.params.get_bool("TeslaVelocityPid")
    # Grey-zone throttle hold: model lead probability (%) at which to stop adding speed even
    # though radard has not published a lead yet. 0 = off.
    self.grey_lead_prob = int(self.params.get("GreyLeadProbPct", return_default=True) or 0) / 100.0
    self.grey_lead_accel_cap = int(self.params.get("GreyLeadAccelCapCms", return_default=True) or 0) / 100.0

    # Curve-speed lateral-accel budget: how hard the car loads the steering in curves. Clamped
    # below the LateralLoadGovernor ceiling (MAX_LATERAL_ACCEL_NO_ROLL = 3.0) so the reactive
    # backstop keeps its headroom regardless of what the driver picks. A value <= 0 turns the whole
    # curve-speed feature off -- both the feedforward profile and the reactive lateral-load governor.
    curve_cms = int(self.params.get("CurveSpeedLatAccelCms", return_default=True) or 220)
    curve_on = curve_cms > 0
    self.curve.set_tuning(min(max(curve_cms, 1) / 100.0, MAX_LATERAL_ACCEL_NO_ROLL - 0.4), enabled=curve_on)
    self.governor.set_enabled(curve_on)

  def grey_lead_ahead(self, model_msg, lead) -> bool:
    """Sub-threshold but real model evidence of something ahead, with no lead published yet."""
    if self.grey_lead_prob <= 0.0 or lead.status:
      return False
    leads = model_msg.leadsV3
    if len(leads) == 0:
      return False
    prob = float(leads[0].prob)
    return np.isfinite(prob) and self.grey_lead_prob <= prob < GREY_LEAD_PROB_MAX

  def reset_lead_creep(self, lead=None) -> None:
    valid = lead is not None and lead.status
    self.lead_creep_frames = 0
    self.lead_creep_track_id = int(lead.radarTrackId) if valid else -1
    self.lead_creep_prev_d_rel = float(lead.dRel) if valid else 0.0
    self.lead_creep_prev_v_lead_k = float(lead.vLeadK) if valid else 0.0
    self.lead_creep_active = False

  def update_lead_creep(self, lead, v_ego: float, enabled: bool) -> bool:
    """Confirm a genuinely creeping lead before suppressing the discrete stop latch.

    A single vLeadK threshold is unsafe near a stopped car: the legacy radar can report short
    positive velocity spikes. Require persistence, low ego speed, no rapid closing, enough room,
    and track continuity.
    """
    finite_lead = lead.status and np.all(np.isfinite([lead.dRel, lead.vLeadK, lead.vRel]))
    track_id = int(lead.radarTrackId) if finite_lead else -1
    initialized = self.lead_creep_frames > 0 or self.lead_creep_active
    discontinuous = initialized and (
      track_id != self.lead_creep_track_id or
      abs(lead.dRel - self.lead_creep_prev_d_rel) > LEAD_CREEP_TRACK_JUMP_D or
      abs(lead.vLeadK - self.lead_creep_prev_v_lead_k) > LEAD_CREEP_TRACK_JUMP_V
    )
    if self.lead_creep_follow <= 0.0 or not enabled or not finite_lead or discontinuous:
      self.reset_lead_creep(lead if finite_lead else None)
      return False

    room = lead.dRel - self.mpc.stop_distance
    braking_room = max(v_ego, 0.0) ** 2 / (2.0 * COMFORT_BRAKE)
    candidate = (v_ego <= LEAD_CREEP_MAX_EGO_SPEED and
                 lead.vLeadK > self.lead_creep_follow and
                 lead.vRel > -LEAD_CREEP_MAX_CLOSING_SPEED and
                 room > LEAD_CREEP_MIN_ROOM + braking_room)
    confirm_frames = max(1, int(round(LEAD_CREEP_CONFIRM_TIME / self.dt)))
    self.lead_creep_frames = min(self.lead_creep_frames + 1, confirm_frames) if candidate else 0
    self.lead_creep_active = self.lead_creep_frames >= confirm_frames
    self.lead_creep_track_id = track_id
    self.lead_creep_prev_d_rel = float(lead.dRel)
    self.lead_creep_prev_v_lead_k = float(lead.vLeadK)
    return self.lead_creep_active

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    self.frame += 1
    if not sm['selfdriveState'].enabled and self.frame % PARAM_REFRESH_FRAMES == 0:
      self.refresh_tuning()

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego, self.launch_accel)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Don't add speed toward something the model half-sees but radard cannot publish yet. This
    # only lowers the acceleration ceiling -- it never commands braking, so a false positive
    # costs speed, not a phantom stop.
    self.grey_lead_hold = self.grey_lead_ahead(sm['modelV2'], sm['radarState'].leadOne)
    if self.grey_lead_hold:
      accel_clip[1] = min(accel_clip[1], self.grey_lead_accel_cap)

    # Curve-aware longitudinal: the feedforward curve-speed profile, with the reactive lateral-load
    # governor folded in as a tighter cap (whichever wants the lower speed). Always enabled; both
    # return V_UNSET when they are not constraining, so this can only ever slow the car down.
    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override
    self.curve.update(sm, long_enabled, long_override, self.v_desired_filter.x, self.a_desired, v_cruise)
    self.governor.update(sm, long_enabled, long_override, v_ego)

    curve_v, curve_a = self.curve.output_v_target, self.curve.output_a_target
    if self.governor.output_v_target < curve_v:
      curve_v, curve_a = self.governor.output_v_target, min(curve_a, 0.0)
    if curve_v < v_cruise:
      v_cruise, self.a_desired = curve_v, curve_a

    # Throttle-fade interlock: don't add throttle while the steering is near or at its lateral limit.
    if self.a_desired > 0.0:
      self.a_desired *= self.governor.throttle_scale()

    if force_slow_decel:
      v_cruise = 0.0
    lead = sm['radarState'].leadOne
    creep_follow_active = self.update_lead_creep(lead, v_ego, not reset_state and not force_slow_decel)
    # v_soft: approaching a stopped/slow lead, cap the speed on a physics stop curve by the room left
    # to the target gap, so the car eases down to it instead of coasting past (CarrotPilot v_soft).
    # If a creeping lead is confirmed, converge to its speed rather than incorrectly targeting zero.
    if self.precise_stop and lead.status and lead.vLeadK < V_SOFT_LEAD_SPEED:
      stop_gap = max(lead.dRel - self.mpc.stop_distance - 1.0, 0.0)
      lead_target_speed = max(lead.vLeadK, 0.0) if creep_follow_active else 0.0
      v_cruise = min(v_cruise, lead_target_speed + math.sqrt(2.0 * COMFORT_BRAKE * stop_gap))

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality, lead=lead)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    gap_adjust = int(sm['carState'].cruiseState.gapAdjust)
    if self.CP.brand == "tesla":
      if gap_adjust != 0:
        if gap_adjust != self.tesla_last_gap_adjust:
          self.tesla_last_gap_adjust = gap_adjust
          self.params.put_nonblocking("TeslaLastGapAdjust", gap_adjust)
      else:
        gap_adjust = self.tesla_last_gap_adjust
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality, gap_adjust=gap_adjust)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    experimental_mode = sm['selfdriveState'].experimentalMode
    # A driver-monitor/soft-disable force-decel or an external zero-speed cap must win even if
    # lead0 happens to be the nearest obstacle. Creep-follow may clear only an ordinary lead0 stop.
    mpc_stop_is_lead = is_mpc_stop_clearable_by_creep(self.mpc.source, force_slow_decel,
                                                      v_cruise, self.CP.vEgoStopping)
    if experimental_mode:
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
    # Creep-follow is a lead0-following aid. It must not clear a cruise/forced stop, a leadTwo stop,
    # or an experimental/e2e stop-line or traffic-light request.
    self.output_should_stop = combine_should_stop(output_should_stop_mpc, output_should_stop_e2e, experimental_mode,
                                                  creep_follow_active, mpc_stop_is_lead)

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
