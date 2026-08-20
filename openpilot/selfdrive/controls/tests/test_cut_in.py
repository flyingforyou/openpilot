"""The cut-in detector's job is to be early without being wrong very often.

These pin the gates that the threshold sweep settled on, and -- more importantly -- the shapes
that must *not* fire. A detector that calls every adjacent-lane car is worse than none, because
what it feeds is a brake.
"""
import math

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


def merge(det, tid=1, *, d_path0=3.2, closing=0.5, frames=60, d_rel=30.0, v_ego=20.0,
          lead_d_rel=0.0, half=HALF, t_start=0.0, d_path_min=0.0, yaw=0.0, changing=False):
  """Walk a track towards the lane centre at `closing` m/s, returning when it was first called.

  `d_path_min` stops the walk short of the lane. Useful where a test wants the detector left in a
  confirmed state: a track driven all the way in stops qualifying on arrival, because a car that
  is already in the lane is a lead rather than a merge.
  """
  called_at = None
  for i in range(frames):
    t = t_start + i * DT
    dp = max(d_path_min, d_path0 - closing * (t - t_start))
    tracks = {tid: FakeTrack(tid, d_rel=d_rel, d_path=dp, half=half)}
    got = det.update(tracks, t, v_ego, lead_d_rel, yaw, changing)
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
    """Crossing the line is exactly where the lateral estimate goes briefly noisy. Dropping the
    call there would lose it at the one moment it is worth having.

    The exact number of frames it survives is bookkeeping; that it survives a short one is the
    property, so this asks for half a second rather than counting to RELEASE_FRAMES."""
    det = CutInDetector()
    merge(det, frames=60, d_path_min=2.0)
    assert det.track_id == 1
    # still seen, just no longer qualifying: it sits back out and stops closing
    for i in range(10):
      got = det.update({1: FakeTrack(1, d_path=3.2)}, 3.5 + i * DT, 20.0)
      assert got == 1, f"dropped the call after {i} frames of wobble"

  def test_but_not_held_forever(self):
    """A track that stops merging must eventually stop being treated as one."""
    det = CutInDetector()
    merge(det, frames=60, d_path_min=2.0)
    for i in range(RELEASE_FRAMES * 2):
      det.update({1: FakeTrack(1, d_path=3.2)}, 3.5 + i * DT, 20.0)
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

  def sway(self, amplitude, frames=200, centre=2.8):
    det = CutInDetector()
    for i in range(frames):
      t = i * DT
      # 2s period, so the peak inward rate is well over MIN_CLOSING however small the amplitude
      dp = centre - amplitude * math.sin(2 * math.pi * t / 2.0)
      if det.update({1: FakeTrack(1, d_path=dp)}, t, 20.0) == 1:
        return t
    return None

  @pytest.mark.parametrize("amplitude", [0.10, 0.20])
  def test_ordinary_lane_wander_is_not_a_merge(self, amplitude):
    """The one this exists for. A car correcting its own position inside its lane moves inward at
    a perfectly respectable rate for half a second and then returns; the rate gate alone cannot
    tell that from a merge, so the cumulative one has to."""
    assert self.sway(amplitude) is None

  def test_but_sway_wider_than_MIN_PROGRESS_is_indistinguishable(self):
    """Stated rather than wished away: this gate asks how much ground a track took, so anything
    that takes MIN_PROGRESS of it reads as a merge whether or not it gives it back. A 0.35m
    amplitude is 0.7m of travel -- wider than a lane-keeping correction, and by then a car that
    close swinging that far is arguably worth leaving room for anyway."""
    assert self.sway(0.35) is not None

  def test_a_wobble_on_top_of_a_real_merge_still_fires(self):
    """Rejecting sway must not reject a merge that happens to sway on its way in."""
    det = CutInDetector()
    called = None
    for i in range(120):
      t = i * DT
      dp = max(0.0, 3.4 - 0.4 * t - 0.25 * math.sin(2 * math.pi * t / 2.0))
      got = det.update({1: FakeTrack(1, d_path=dp)}, t, 20.0)
      if got == 1 and called is None:
        called = t
    assert called is not None

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


class TestTurning:
  """dPath is an offset from the model's lane centre, so a hard turn swings the frame it is
  measured in and a parked car marches across the lane. Three seconds of that is exactly the
  signature the cumulative gate looks for -- which is how the first replayed call came 1.3s out
  of a hand-driven U-turn, on a track 42.5m away that was going nowhere."""

  def test_a_hard_turn_suppresses_everything(self):
    det = CutInDetector()
    assert merge(det, yaw=0.4) is None

  def test_and_keeps_suppressing_until_the_window_has_refilled(self):
    """The wait is the whole progress window: less, and the turn's own swing is still in the
    history being integrated."""
    det = CutInDetector()
    merge(det, frames=60, yaw=0.4)
    at = merge(det, frames=20, t_start=3.0, yaw=0.0)     # 1s of settled driving
    assert at is None

  def test_but_a_settled_frame_works_normally_again(self):
    det = CutInDetector()
    merge(det, frames=60, yaw=0.4)          # ends at t=2.95, so the wait runs to t=5.95
    assert merge(det, frames=60, t_start=6.0, yaw=0.0) is not None

  def test_an_ordinary_curve_does_not_suppress(self):
    """0.043 rad/s is p99.9 of the freeway drive; the guard must sit well clear of it."""
    det = CutInDetector()
    assert merge(det, yaw=0.05) is not None

  def test_a_turn_discards_the_history_it_polluted(self):
    """Waiting is not enough on its own -- the trail gathered during the turn has to go, or it
    counts as progress the moment the wait expires."""
    det = CutInDetector()
    merge(det, frames=60, yaw=0.4)
    assert det._trail == {}


class TestOurOwnLaneChange:
  """dPath cannot say which of the two cars moved. When we change lanes the centre moves with us
  and a car sitting still in the next lane sweeps into ours -- one of the eight calls replayed
  over 00000087 was exactly that, blinker on, nobody merging."""

  def test_our_lane_change_suppresses_calls(self):
    det = CutInDetector()
    assert merge(det, changing=True) is None

  def test_and_keeps_suppressing_while_the_history_refills(self):
    det = CutInDetector()
    merge(det, frames=60, changing=True)
    assert merge(det, frames=20, t_start=3.0, changing=False) is None

  def test_then_works_normally_again(self):
    det = CutInDetector()
    merge(det, frames=60, changing=True)     # ends at t=2.95, wait runs to t=5.95
    assert merge(det, frames=60, t_start=6.0, changing=False) is not None

  def test_the_polluted_history_is_discarded(self):
    det = CutInDetector()
    merge(det, frames=60, changing=True)
    assert det._trail == {}


class TestState:
  def test_a_vanished_track_is_forgotten(self):
    det = CutInDetector()
    merge(det, frames=60)
    det.update({}, 10.0, 20.0)
    assert det.track_id == -1
    # and its history does not leak into a track that reuses the id later
    assert merge(det, frames=CONFIRM_FRAMES - 1, t_start=20.0) is None

  def test_reset_clears_everything(self):
    det = CutInDetector()
    merge(det, frames=60)
    det.reset()
    assert det.track_id == -1

  def test_slowing_to_a_stop_resets(self):
    det = CutInDetector()
    merge(det, frames=60)
    assert det.update({1: FakeTrack(1)}, 5.0, 0.5) == -1
    assert det.track_id == -1
