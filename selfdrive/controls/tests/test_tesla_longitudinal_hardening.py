from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import J_EGO_COST, LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_planner import (LongitudinalPlanner, combine_should_stop,
                                                                  is_mpc_stop_clearable_by_creep)


def lead(*, status=True, track_id=1, d_rel=10.0, v_lead=0.5, v_rel=0.0, a_lead=0.0):
  return SimpleNamespace(status=status, radarTrackId=track_id, dRel=d_rel,
                         vLeadK=v_lead, vRel=v_rel, aLeadK=a_lead)


def dynamic_mpc():
  mpc = LongitudinalMpc.__new__(LongitudinalMpc)
  mpc.dt = 0.05
  mpc.dynamic_tf_gain = 0.5
  mpc.lead_jerk = 0.0
  mpc.prev_a_lead_k = 0.0
  mpc.dynamic_lead_initialized = False
  mpc.dynamic_lead_track_id = -1
  mpc.dynamic_lead_d_rel = 0.0
  mpc.dynamic_lead_v_lead_k = 0.0
  return mpc


def creep_planner():
  planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
  planner.dt = 0.05
  planner.lead_creep_follow = 0.3
  planner.lead_creep_frames = 0
  planner.lead_creep_track_id = -1
  planner.lead_creep_prev_d_rel = 0.0
  planner.lead_creep_prev_v_lead_k = 0.0
  planner.lead_creep_active = False
  planner.mpc = SimpleNamespace(stop_distance=6.0)
  return planner


class TestDynamicFollowHardening:
  def test_first_lead_and_track_change_seed_without_jerk(self):
    mpc = dynamic_mpc()
    base = 1.3
    assert mpc.apply_dynamic_t_follow(base, lead(track_id=1, a_lead=1.0)) == base
    assert mpc.lead_jerk == 0.0
    mpc.lead_jerk = 1.0
    assert not mpc.prepare_dynamic_lead(lead(track_id=2, a_lead=-2.0))
    assert mpc.lead_jerk == 0.0
    assert mpc.apply_dynamic_t_follow(base, lead(track_id=2, a_lead=-2.0)) == base

    # Vision-only leads share track id -1, so distance/velocity continuity must also reset.
    mpc.reset_dynamic_lead(lead(track_id=-1, d_rel=15.0, v_lead=2.0, a_lead=0.0))
    assert mpc.apply_dynamic_t_follow(base, lead(track_id=-1, d_rel=30.0, v_lead=10.0, a_lead=3.0)) == base
    assert mpc.lead_jerk == 0.0

  def test_raw_accel_jump_is_clipped(self):
    mpc = dynamic_mpc()
    mpc.reset_dynamic_lead(lead(track_id=1, a_lead=0.0))
    base = 1.3
    # Without raw-jerk clipping this 10 m/s^2 acquisition jump immediately saturates the adjust.
    assert mpc.apply_dynamic_t_follow(base, lead(track_id=1, a_lead=10.0)) == base
    assert abs(mpc.lead_jerk) < 0.5

  def test_nonfinite_lead_resets_dynamic_follow(self):
    mpc = dynamic_mpc()
    mpc.reset_dynamic_lead(lead(track_id=1, a_lead=0.0))
    assert mpc.apply_dynamic_t_follow(1.3, lead(track_id=1, a_lead=float('nan'))) == 1.3
    assert not mpc.dynamic_lead_initialized

  def test_set_weights_resets_before_using_a_new_lead(self):
    mpc = dynamic_mpc()
    mpc.reset_dynamic_lead(lead(track_id=1, a_lead=0.0))
    mpc.lead_jerk = 1.0  # old lead was pulling away, so jerk cost would otherwise be halved
    captured = {}
    mpc.set_cost_weights = lambda costs, constraints: captured.update(costs=costs, constraints=constraints)
    mpc.set_weights(lead=lead(track_id=2, a_lead=-2.0))
    assert mpc.lead_jerk == 0.0
    assert captured['costs'][-1] == J_EGO_COST


class TestLeadCreepHardening:
  def test_requires_persistent_motion(self):
    planner = creep_planner()
    moving = lead(track_id=1, d_rel=10.0, v_lead=0.5, v_rel=0.0)
    assert not planner.update_lead_creep(moving, v_ego=0.5, enabled=True)
    assert not planner.update_lead_creep(moving, v_ego=0.5, enabled=True)
    assert not planner.update_lead_creep(moving, v_ego=0.5, enabled=True)
    assert planner.update_lead_creep(moving, v_ego=0.5, enabled=True)

  def test_near_stop_noise_and_fast_closing_do_not_suppress_stop(self):
    planner = creep_planner()
    noisy_stopped = lead(track_id=1, d_rel=6.5, v_lead=0.9, v_rel=0.0)
    for _ in range(10):
      assert not planner.update_lead_creep(noisy_stopped, v_ego=0.2, enabled=True)

    closing = lead(track_id=1, d_rel=10.0, v_lead=0.5, v_rel=-1.0)
    for _ in range(10):
      assert not planner.update_lead_creep(closing, v_ego=1.0, enabled=True)

  def test_track_change_restarts_confirmation(self):
    planner = creep_planner()
    moving = lead(track_id=1, d_rel=10.0, v_lead=0.5, v_rel=0.0)
    for _ in range(4):
      active = planner.update_lead_creep(moving, v_ego=0.5, enabled=True)
    assert active
    assert not planner.update_lead_creep(lead(track_id=2, d_rel=10.0, v_lead=0.5), v_ego=0.5, enabled=True)

  def test_braking_room_and_vision_lead_jump_restart_confirmation(self):
    planner = creep_planner()
    # At 2.5 m/s, 1.5m beyond the configured stop distance is not enough room to clear stop.
    too_close = lead(track_id=1, d_rel=7.5, v_lead=0.8, v_rel=0.0)
    for _ in range(10):
      assert not planner.update_lead_creep(too_close, v_ego=2.5, enabled=True)

    vision = lead(track_id=-1, d_rel=12.0, v_lead=0.6, v_rel=0.0)
    for _ in range(4):
      active = planner.update_lead_creep(vision, v_ego=0.5, enabled=True)
    assert active
    jumped = lead(track_id=-1, d_rel=25.0, v_lead=8.0, v_rel=0.0)
    assert not planner.update_lead_creep(jumped, v_ego=0.5, enabled=True)

  def test_nonfinite_lead_and_disabled_state_reset_confirmation(self):
    planner = creep_planner()
    moving = lead(track_id=1, d_rel=10.0, v_lead=0.5, v_rel=0.0)
    assert not planner.update_lead_creep(moving, v_ego=0.5, enabled=True)
    assert not planner.update_lead_creep(lead(track_id=1, d_rel=float('nan')), v_ego=0.5, enabled=True)
    assert planner.lead_creep_frames == 0
    assert not planner.update_lead_creep(moving, v_ego=0.5, enabled=False)


class TestShouldStopSourceIsolation:
  def test_creep_follow_does_not_clear_e2e_stop(self):
    assert combine_should_stop(True, True, experimental_mode=True, creep_follow_active=True, mpc_stop_is_lead=True)
    assert combine_should_stop(False, True, experimental_mode=True, creep_follow_active=True, mpc_stop_is_lead=True)

  def test_creep_follow_only_clears_mpc_stop(self):
    assert not combine_should_stop(True, False, experimental_mode=False, creep_follow_active=True, mpc_stop_is_lead=True)
    assert not combine_should_stop(True, False, experimental_mode=True, creep_follow_active=True, mpc_stop_is_lead=True)
    assert combine_should_stop(True, False, experimental_mode=False, creep_follow_active=False, mpc_stop_is_lead=True)

  def test_creep_follow_preserves_non_lead_mpc_stop(self):
    assert combine_should_stop(True, False, experimental_mode=False, creep_follow_active=True, mpc_stop_is_lead=False)
    assert combine_should_stop(True, False, experimental_mode=True, creep_follow_active=True, mpc_stop_is_lead=False)

  def test_only_ordinary_lead0_stop_is_clearable(self):
    assert is_mpc_stop_clearable_by_creep(LongitudinalPlanSource.lead0, False, 5.0, 0.1)
    assert not is_mpc_stop_clearable_by_creep(LongitudinalPlanSource.lead1, False, 5.0, 0.1)
    assert not is_mpc_stop_clearable_by_creep(LongitudinalPlanSource.cruise, False, 5.0, 0.1)
    assert not is_mpc_stop_clearable_by_creep(LongitudinalPlanSource.lead0, True, 5.0, 0.1)
    assert not is_mpc_stop_clearable_by_creep(LongitudinalPlanSource.lead0, False, 0.0, 0.1)
