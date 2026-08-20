"""The cut-in detector's job is to be early without being wrong very often.

These pin the gates that the threshold sweep settled on, and -- more importantly -- the shapes
that must *not* fire. A detector that calls every adjacent-lane car is worse than none, because
what it feeds is a brake.
"""
import pytest

from openpilot.selfdrive.controls.lib.cut_in import (
  CONFIRM_FRAMES,
  MAX_DREL,
  MIN_DREL,
  MIN_VLEAD,
  OUTER_LANES,
  RELEASE_FRAMES,
  CutInDetector,
)

DT = 0.05          # 20Hz, radard's rate
HALF = 1.8         # a normal lane half-width


class FakeTrack:
  def __init__(self, tid, d_rel=30.0, d_path=2.5, v_lead=20.0, measured=True, half=HALF):
    self.identifier = tid
    self.dRel = d_rel
    self.dPath = d_path
    self.vLead = v_lead
    self.measured = measured
    self.lane_half_width = half


def merge(det, tid=1, *, d_path0=3.2, closing=0.5, frames=40, d_rel=30.0, v_ego=20.0,
          lead_d_rel=0.0, half=HALF, t_start=0.0):
  """Walk a track towards the lane centre at `closing` m/s, returning when it was first called."""
  called_at = None
  for i in range(frames):
    t = t_start + i * DT
    dp = max(0.0, d_path0 - closing * (t - t_start))
    tracks = {tid: FakeTrack(tid, d_rel=d_rel, d_path=dp, half=half)}
    got = det.update(tracks, t, v_ego, lead_d_rel)
    if got == tid and called_at is None:
      called_at = t - t_start
  return called_at


class TestFires:
  def test_a_steady_merge_is_called(self):
    assert merge(CutInDetector()) is not None

  def test_it_is_called_before_the_car_is_in_the_lane(self):
    """The whole point. If it only fires once dPath is inside the lane it has warned nobody."""
    det = CutInDetector()
    at = merge(det, d_path0=3.2, closing=0.5)
    assert at is not None
    assert 3.2 - 0.5 * at > HALF      # still outside the lane edge when called

  def test_confirmation_is_not_instant(self):
    det = CutInDetector()
    at = merge(det, closing=0.5)
    assert at >= CONFIRM_FRAMES * DT

  def test_a_confirmed_merge_is_held_through_a_wobble(self):
    """Crossing the line is exactly where the lateral estimate goes briefly noisy. Losing the
    call there would drop it at the one moment it is worth having."""
    det = CutInDetector()
    merge(det, frames=30)
    assert det.track_id == 1
    # the track is still seen, it just stops qualifying: dPath jumps back out, no longer closing
    for i in range(RELEASE_FRAMES - 2):
      got = det.update({1: FakeTrack(1, d_path=3.2)}, 2.0 + i * DT, 20.0)
      assert got == 1, f"dropped the call after {i} frames of wobble"

  def test_but_not_held_forever(self):
    det = CutInDetector()
    merge(det, frames=30)
    for i in range(RELEASE_FRAMES + 5):
      det.update({1: FakeTrack(1, d_path=3.2)}, 2.0 + i * DT, 20.0)
    assert det.track_id == -1

  def test_the_nearer_of_two_merges_wins(self):
    det = CutInDetector()
    for i in range(40):
      t = i * DT
      tracks = {
        1: FakeTrack(1, d_rel=40.0, d_path=max(0.0, 3.2 - 0.5 * t)),
        2: FakeTrack(2, d_rel=20.0, d_path=max(0.0, 3.2 - 0.5 * t)),
      }
      got = det.update(tracks, t, 20.0)
    assert got == 2


class TestDoesNotFire:
  def test_a_car_holding_its_own_lane(self):
    det = CutInDetector()
    assert merge(det, closing=0.0, d_path0=3.2) is None

  def test_a_car_moving_away(self):
    det = CutInDetector()
    assert merge(det, closing=-0.5, d_path0=3.2) is None

  def test_a_car_two_lanes_over(self):
    """Even closing steadily -- it has a whole lane to cross first."""
    det = CutInDetector()
    assert merge(det, d_path0=OUTER_LANES * HALF + 1.0, closing=0.5, frames=20) is None

  def test_a_car_already_in_our_lane(self):
    """That is a lead, and the lead pipeline owns it."""
    det = CutInDetector()
    assert merge(det, d_path0=HALF * 0.5, closing=0.2) is None

  def test_a_slow_drift_that_will_not_arrive(self):
    """Closing, but so gently it is minutes from the lane -- lane-keeping wander, not a merge."""
    det = CutInDetector()
    assert merge(det, d_path0=3.4, closing=0.05, frames=60) is None

  def test_something_stationary(self):
    det = CutInDetector()
    for i in range(40):
      t = i * DT
      tracks = {1: FakeTrack(1, d_path=max(0.0, 3.2 - 0.5 * t), v_lead=MIN_VLEAD - 1.0)}
      assert det.update(tracks, t, 20.0) == -1

  def test_an_unmeasured_track(self):
    det = CutInDetector()
    for i in range(40):
      t = i * DT
      tracks = {1: FakeTrack(1, d_path=max(0.0, 3.2 - 0.5 * t), measured=False)}
      assert det.update(tracks, t, 20.0) == -1

  @pytest.mark.parametrize("d_rel", [MIN_DREL - 1.0, MAX_DREL + 1.0])
  def test_out_of_range(self, d_rel):
    det = CutInDetector()
    assert merge(det, d_rel=d_rel) is None

  def test_behind_what_we_already_follow(self):
    """Merging in behind the lead changes nothing about how fast we may go."""
    det = CutInDetector()
    assert merge(det, d_rel=30.0, lead_d_rel=20.0) is None

  def test_ahead_of_the_lead_still_counts(self):
    det = CutInDetector()
    assert merge(det, d_rel=20.0, lead_d_rel=40.0) is not None

  def test_standing_still(self):
    det = CutInDetector()
    assert merge(det, v_ego=1.0) is None


class TestState:
  def test_a_vanished_track_is_forgotten(self):
    det = CutInDetector()
    merge(det, frames=40)
    det.update({}, 10.0, 20.0)
    assert det.track_id == -1
    # and its history does not leak into a track that reuses the id later
    assert merge(det, frames=CONFIRM_FRAMES - 1, t_start=20.0) is None

  def test_reset_clears_everything(self):
    det = CutInDetector()
    merge(det, frames=40)
    det.reset()
    assert det.track_id == -1

  def test_slowing_to_a_stop_resets(self):
    det = CutInDetector()
    merge(det, frames=40)
    assert det.update({1: FakeTrack(1)}, 5.0, 0.5) == -1
    assert det.track_id == -1
