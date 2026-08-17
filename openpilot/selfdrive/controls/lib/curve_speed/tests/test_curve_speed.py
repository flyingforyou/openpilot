import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.curve_speed import V_UNSET
from openpilot.selfdrive.controls.lib.curve_speed.curve_speed_controller import (
  CurveSpeedController, CurveState, A_LAT_MAX, A_DECEL)
from openpilot.selfdrive.controls.lib.curve_speed.lateral_load_governor import (
  LateralLoadGovernor, A_LAT_CEILING)

T_IDXS = np.array(ModelConstants.T_IDXS)
N = len(T_IDXS)


def make_model(yaw_rate, speed: float):
  """yaw_rate: scalar (constant over the horizon) or a per-point sequence."""
  yaws = [float(yaw_rate)] * N if np.isscalar(yaw_rate) else [float(y) for y in yaw_rate]
  model = messaging.new_message('modelV2')
  orientation_rate = log.XYZTData.new_message()
  orientation_rate.z = yaws
  model.modelV2.orientationRate = orientation_rate
  velocity = log.XYZTData.new_message()
  velocity.x = [float(speed)] * N
  model.modelV2.velocity = velocity
  return model


def make_controls_state(which='torqueState', saturated=False, actual=0.0, desired=0.0):
  cs = messaging.new_message('controlsState')
  sub = cs.controlsState.lateralControlState.init(which)
  sub.saturated = bool(saturated)
  if which == 'torqueState':
    sub.actualLateralAccel = float(actual)
    sub.desiredLateralAccel = float(desired)
  return cs


def make_car_state(v_ego=20.0, yaw_rate=0.0):
  cs = messaging.new_message('carState')
  cs.carState.vEgo = float(v_ego)
  cs.carState.yawRate = float(yaw_rate)
  return cs


def sm_for(yaw_rate, speed, which='torqueState', saturated=False, actual=0.0, desired=0.0, v_ego=20.0):
  return {
    'modelV2': make_model(yaw_rate, speed).modelV2,
    'controlsState': make_controls_state(which, saturated, actual, desired).controlsState,
    # pick a yaw rate that reproduces `actual` as measured lateral accel
    'carState': make_car_state(v_ego, actual / max(v_ego, 1.0)).carState,
  }


class TestCurveSpeedController:
  def setup_method(self):
    self.scc = CurveSpeedController()

  def test_initial_state(self):
    assert self.scc.state == CurveState.disabled
    assert self.scc.output_v_target == V_UNSET

  def test_straight_inactive(self):
    sm = sm_for(yaw_rate=0.001, speed=25.0, v_ego=25.0)
    self.scc.update(sm, True, False, 25.0, 0.0, 30.0)
    assert not self.scc.is_active
    assert self.scc.output_v_target == V_UNSET
    assert self.scc.state == CurveState.cruise

  def test_curve_slows(self):
    # sustained yaw rate -> a curve through the whole horizon -> target below set speed
    sm = sm_for(yaw_rate=0.07, speed=15.0, v_ego=24.0)
    self.scc.update(sm, True, False, 24.0, 0.0, 30.0)
    assert self.scc.is_active
    assert self.scc.output_v_target < 30.0
    # physics check: v_target ~= sqrt(a_lat_max / kappa) for the sustained curve
    kappa = 0.07 / 15.0
    assert np.isclose(self.scc.v_target, (A_LAT_MAX / kappa) ** 0.5, rtol=0.05)
    assert self.scc.state in (CurveState.slowing, CurveState.curve)

  def test_backward_pass_brakes_early(self):
    # straight for the near half of the horizon, sharp curve after: the backward pass must already
    # be pulling speed down now, but not all the way to the curve speed yet.
    i_curve, speed = 16, 25.0
    yaws = [0.0] * i_curve + [0.25] * (N - i_curve)
    sm = sm_for(yaw_rate=yaws, speed=speed, v_ego=speed)
    self.scc.update(sm, True, False, speed, 0.0, 30.0)

    v_curve = (A_LAT_MAX / (0.25 / speed)) ** 0.5
    s_curve = speed * T_IDXS[i_curve]
    assert self.scc.is_active
    assert v_curve < self.scc.v_target < 30.0
    assert np.isclose(self.scc.v_target, (v_curve ** 2 + 2 * A_DECEL * s_curve) ** 0.5, rtol=0.05)

  def test_never_raises_target(self):
    # a set speed below the curve speed must stay the binding constraint
    sm = sm_for(yaw_rate=0.02, speed=20.0, v_ego=20.0)
    self.scc.update(sm, True, False, 20.0, 0.0, 15.0)
    assert self.scc.output_v_target >= 15.0 or self.scc.output_v_target == V_UNSET

  def test_long_disabled_inactive(self):
    sm = sm_for(yaw_rate=0.07, speed=15.0, v_ego=24.0)
    self.scc.update(sm, False, False, 24.0, 0.0, 30.0)
    assert not self.scc.is_active
    assert self.scc.state == CurveState.disabled

  def test_driver_override_releases(self):
    sm = sm_for(yaw_rate=0.07, speed=15.0, v_ego=24.0)
    self.scc.update(sm, True, True, 24.0, 0.0, 30.0)
    assert not self.scc.is_active
    assert self.scc.output_v_target == V_UNSET

  def test_below_min_speed_inactive(self):
    sm = sm_for(yaw_rate=0.2, speed=4.0, v_ego=4.0)
    self.scc.update(sm, True, False, 4.0, 0.0, 30.0)
    assert not self.scc.is_active


class TestLateralLoadGovernor:
  def setup_method(self):
    self.gov = LateralLoadGovernor()

  def test_low_load_passthrough(self):
    sm = sm_for(yaw_rate=0.02, speed=20.0, actual=1.0, desired=1.0, v_ego=20.0)
    self.gov.update(sm, True, False, 20.0)
    assert self.gov.output_v_target == V_UNSET
    assert self.gov.throttle_scale() == 1.0

  def test_overload_caps_speed(self):
    over = A_LAT_CEILING * 1.1
    sm = sm_for(yaw_rate=0.05, speed=24.0, actual=over, desired=over, v_ego=24.0)
    for _ in range(20):  # let the EMA converge
      self.gov.update(sm, True, False, 24.0)
    assert self.gov.is_active
    assert self.gov.output_v_target < 24.0

  def test_interlock_zeros_throttle_when_running_wide(self):
    sm = sm_for(yaw_rate=0.05, speed=24.0, saturated=True, actual=2.0, desired=3.0, v_ego=24.0)
    self.gov.update(sm, True, False, 24.0)
    assert self.gov.throttle_scale() == 0.0  # saturated + desired > actual => running wide => no throttle

  def test_saturated_but_tracking_is_not_running_wide(self):
    sm = sm_for(yaw_rate=0.05, speed=24.0, saturated=True, actual=2.0, desired=2.0, v_ego=24.0)
    self.gov.update(sm, True, False, 24.0)
    assert not self.gov.running_wide

  def test_non_torque_controller_uses_saturation_flag(self):
    # pidState/angleState publish no lateral-accel pair, so saturation alone is the running-wide signal
    for which in ('pidState', 'angleState'):
      gov = LateralLoadGovernor()
      sm = sm_for(yaw_rate=0.05, speed=24.0, which=which, saturated=True, v_ego=24.0)
      gov.update(sm, True, False, 24.0)
      assert gov.running_wide, which
      assert gov.throttle_scale() == 0.0, which

  def test_long_disabled_releases(self):
    over = A_LAT_CEILING * 1.1
    sm = sm_for(yaw_rate=0.05, speed=24.0, actual=over, desired=over, v_ego=24.0)
    self.gov.update(sm, False, False, 24.0)
    assert not self.gov.is_active
    assert self.gov.output_v_target == V_UNSET
    assert self.gov.throttle_scale() == 1.0
