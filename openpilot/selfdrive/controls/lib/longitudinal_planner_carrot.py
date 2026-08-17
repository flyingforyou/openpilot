#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  """carrot's variant, which also reports the trajectory's current and peak speed.

  Local rather than shared: the stock planner takes the two-value form from drive_helpers, and
  widening that would mean editing a call path this port is meant to leave alone.
  """
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / action_t - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
    v_max = np.max(speeds)
  else:
    v_now = a_now = v_target = v_target_1sec = a_target = v_max = 0.0
  should_stop = (v_target < vEgoStopping and v_target_1sec < vEgoStopping)
  return a_target, should_stop, v_now, v_max
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.carrot_params import TypedParams
from openpilot.selfdrive.controls.lib.carrot_functions import CarrotPlanner


LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -2.0 #-1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5
RESET_DECEL_RAMP_TIME = 2.0
TURN_CURVATURE_LOOKAHEAD = 1.0
TURN_CURVATURE_MIN_SPEED = 3.0


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def get_future_curvature(model_msg, fallback_curvature, lookahead=TURN_CURVATURE_LOOKAHEAD):
  if (len(model_msg.orientationRate.z) != ModelConstants.IDX_N or
      len(model_msg.velocity.x) != ModelConstants.IDX_N):
    return fallback_curvature

  yaw_rate_future = float(np.interp(lookahead, ModelConstants.T_IDXS, model_msg.orientationRate.z))
  velocity_future = float(np.interp(lookahead, ModelConstants.T_IDXS, model_msg.velocity.x))
  if not (np.isfinite(yaw_rate_future) and np.isfinite(velocity_future)):
    return fallback_curvature

  return yaw_rate_future / max(abs(velocity_future), TURN_CURVATURE_MIN_SPEED)

def limit_accel_in_turns(v_ego, curvature, a_target, a_lat_max,
                         safety_ratio=0.70,   # 0.60~0.85 (작을수록 더 얌전)
                         min_v=0.1):
  """
  v_ego    : m/s
  curvature: 1/m  (sign 포함)
  a_target : [a_min, a_max] (m/s^2)
  a_lat_max: 허용 최대 횡가속 (m/s^2)

  safety_ratio:
    a_lat_max에 소프트 마진을 주는 비율.
    예) a_lat_max=4, safety_ratio=0.7 -> 실사용 한계 2.8로 계산.

  return   : [a_min, 제한된 a_max]
  """
  if v_ego < min_v or a_lat_max <= 0.0:
    return a_target

  a_lat_eff = abs(a_lat_max) * float(safety_ratio)

  # 횡가속
  a_y_abs = abs((v_ego * v_ego) * curvature)

  # 남은 종가속 여유 (원형 경계)
  if a_y_abs >= a_lat_eff:
    a_x_allowed = 0.0
  else:
    a_x_allowed = math.sqrt(a_lat_eff * a_lat_eff - a_y_abs * a_y_abs)

  # a_target = [min, max] 중 max만 제한
  return [a_target[0], min(a_target[1], a_x_allowed)]

class CarrotLongitudinalPlanner:
  """CarrotPilot's longitudinal planner, wrapped to the interface plannerd already uses.

  Upstream this takes a CarrotPlanner as a second argument to update() and publish(), because
  carrot runs it from its own process. Here it owns one instead, so plannerd can hold either
  planner behind the same two calls and the choice stays a single line at startup.
  """

  def __init__(self, CP, dt=DT_MDL):
    self.planner = _CarrotLongitudinalPlannerImpl(CP, dt)
    self.carrot = CarrotPlanner()

  def update(self, sm):
    self.planner.update(sm, self.carrot)

  def publish(self, sm, pm):
    self.planner.publish(sm, pm, self.carrot)

  def __getattr__(self, name):
    # plannerd and the debug tools read planner state directly (v_desired_filter, mpc, ...)
    return getattr(self.planner, name)


class _CarrotLongitudinalPlannerImpl:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    # VW MEB(ID.4/ID.5)에서만 True. 가속 오버라이드 중 출발 FCW 오탐을 끄기 위한 게이트.
    # VW MEB (ID.4/ID.5) only: a closed-curvature platform whose FCW threshold and
    # gas-override suppression differ. Not this car, and the flag it keys off does not
    # exist in this tree.
    self.is_vw_meb = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_v_target_now = 0.0
    self.output_j_target_now = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.vCluRatio = 1.0
    self.reset_decel_timer = 0
    self.reset_decel_start_a = 0.0
    
    self.v_cruise_kph = 0.0

    self.params = TypedParams()

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

  def update(self, sm, carrot):
    self.mpc.mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)

    self.v_cruise_kph = carrot.update(sm, v_cruise_kph, self.mpc.mode)
    self.mpc.mode = carrot.mode
    v_cruise = self.v_cruise_kph * CV.KPH_TO_MS

    # vCluRatio scales cruise speed to the cluster's own display rounding. This car does not
    # report it, so v_cruise is used as given -- same as the ratio being 1.0 upstream.

    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized or carrot.soft_hold_active

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits = [A_CRUISE_MIN, carrot.get_carrot_accel(v_ego)]
      curvature_future = get_future_curvature(sm['modelV2'], sm['controlsState'].desiredCurvature)
      a_lat_max = 3.0
      accel_limits_turns = limit_accel_in_turns(v_ego, curvature_future, accel_limits, a_lat_max)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## carrot

      self.reset_decel_timer = int(RESET_DECEL_RAMP_TIME / self.dt)
      self.reset_decel_start_a = self.a_desired

    elif self.reset_decel_timer > 0:
      ramp_steps = max(1, int(RESET_DECEL_RAMP_TIME / self.dt))
      t = float(np.clip(self.reset_decel_timer / ramp_steps, 0.0, 1.0))
      self.reset_decel_timer -= 1

      # 2초 동안 감속 하한을 현재 가속도 근처에서 원래 limit으로 천천히 복귀
      soft_min_accel = min(0.0, self.reset_decel_start_a - 0.05)
      ramped_min_accel = soft_min_accel * t + accel_limits_turns[0] * (1.0 - t)

      accel_limits_turns[0] = max(
        accel_limits_turns[0],
        min(0.0, ramped_min_accel)
      )

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = True #throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_limits_turns[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_limits_turns[1], clipped_accel_coast])
      accel_limits_turns[1] = min(accel_limits_turns[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality, jerk_factor = carrot.jerk_factor_apply, a_change_cost_starting = carrot.aChangeCostStarting)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(carrot, reset_state, sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    # VW MEB: 운전자가 가속페달을 밟는 중(오버라이드)이면 출발 FCW 오탐을 억제. 정지 후 급출발 시
    # 레이더 dRel 지연(stale)·vLead 음수 아티팩트로 한 프레임 충돌예측이 떴음. gasPressed 동안엔
    # 운전자가 직접 가속 제어 중이라 경고 의미가 약하고, FCW는 경고표시일 뿐 제동에는 영향 없음.
    # 또 MEB는 crash_cnt 문턱을 높여(>15 ≈ 0.8s 지속) 잠깐 스치는 marginal 예측의 nuisance "삐"를
    # 줄인다. 실제 제동(MPC)은 fcw와 무관하게 동작하므로 안전성 저하 없음. 타 차종은 기존 >2 유지.
    # MEB 추가 억제: 차가 '가속 중'(aEgo>1.0)이면 충돌 임박 상태가 아니므로(임박하면 감속부터 함)
    # 출발/따라잡기 중 nuisance "삐"를 끈다. gasPressed가 순간 누락돼도 이걸로 커버. 진짜 위험은
    # 차가 감속(aEgo<0)으로 들어가면서 그때 FCW가 뜨고, 제동은 fcw와 무관하게 이미 작동한다.
    fcw_crash_thresh = 15 if self.is_vw_meb else 2
    meb_suppress = self.is_vw_meb and (sm['carState'].gasPressed or sm['carState'].aEgo > 1.0)
    self.fcw = (self.mpc.crash_cnt > fcw_crash_thresh and not sm['carState'].standstill and not reset_state
                and not meb_suppress)
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    longitudinalActuatorDelay = self.params.get_float("LongActuatorDelay")*0.01
    vEgoStopping = self.params.get_float("VEgoStopping") * 0.01
    action_t =  longitudinalActuatorDelay + DT_MDL

    output_a_target_mpc, output_should_stop_mpc, output_v_target_mpc, _ = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    # This tree's model does not publish action.desiredVelocity; carrot's does. Without it the
    # blended branch below would take min(mpc, 0) and report a target of zero, so fall back to
    # the MPC's own value, which is what the blend reduces to when the model has no opinion.
    output_v_target_now_e2e = output_v_target_mpc

    if self.mpc.mode == 'acc':
      output_a_target = output_a_target_mpc
      output_v_target_now = output_v_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      output_v_target_now = min(output_v_target_mpc, output_v_target_now_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    #for idx in range(2):
    #  accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    #self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    #self.prev_accel_clip = accel_clip
    self.output_a_target = output_a_target
    self.output_v_target_now = output_v_target_now
    self.output_j_target_now = self.j_desired_trajectory[0]

  def publish(self, sm, pm, carrot):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.plannerSource = 'carrot'
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.vTargetNow = float(self.output_v_target_now)
    longitudinalPlan.jTargetNow = float(self.output_j_target_now)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    longitudinalPlan.xState = carrot.xState.value
    longitudinalPlan.trafficState = carrot.trafficState.value
    longitudinalPlan.cruiseTarget = self.v_cruise_kph
    longitudinalPlan.cruiseCeiling = float(carrot.cruiseCeiling)
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.desiredDistance = float(self.mpc.desired_distance)
    longitudinalPlan.events = carrot.events.to_msg()
    longitudinalPlan.myDrivingMode = carrot.myDrivingMode.value

    pm.send('longitudinalPlan', plan_send)
