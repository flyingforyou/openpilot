"""RadarLeadHold: bridging a vision dropout without following the wrong car.

The hold used to be scoped to close range, where a still-measured track that vision dropped is
almost certainly the car in front. Widening it to the distances the dropouts actually happen at
(44-97 m median) is only safe with the off-path check, so that is what these pin down.
"""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.radar_lead_hold import (
  RADAR_LEAD_HOLD_MAX_DPATH,
  RADAR_LEAD_HOLD_MAX_DPATH_FAR,
  RadarLeadHold,
)


def _track(d_rel=40.0, d_path=0.0, measured=True, selected_count=5):
  return SimpleNamespace(dRel=d_rel, dPath=d_path, measured=measured, selected_count=selected_count)


def _hold(hold_dist=100.0, hold_ms=1000, track_id=1):
  h = RadarLeadHold()
  h.configure(hold_dist, hold_ms)
  h.track_id = track_id
  return h


class TestRadarLeadHold:
  def test_disabled_by_default_distance(self):
    h = RadarLeadHold()          # hold_dist 0 = off
    h.track_id = 1
    assert h.candidate({1: _track()}) is None

  def test_holds_a_still_measured_confirmed_track(self):
    assert _hold().candidate({1: _track()}) is not None

  def test_drops_when_radar_stops_measuring(self):
    assert _hold().candidate({1: _track(measured=False)}) is None

  def test_drops_when_vision_never_confirmed_it(self):
    # selected_count is the continuity flag; 0 means this was never our lead
    assert _hold().candidate({1: _track(selected_count=0)}) is None

  def test_drops_beyond_configured_distance(self):
    assert _hold(hold_dist=60.0).candidate({1: _track(d_rel=80.0)}) is None

  def test_holds_out_to_the_configured_distance(self):
    assert _hold(hold_dist=150.0).candidate({1: _track(d_rel=97.0)}) is not None

  def test_budget_runs_out(self):
    h = _hold()
    h.frames = h.max_frames
    assert h.candidate({1: _track()}) is None


class TestOffPathGate:
  """Without this, widening the range means happily following a car in the next lane."""

  def test_near_track_off_path_is_dropped(self):
    t = _track(d_rel=30.0, d_path=RADAR_LEAD_HOLD_MAX_DPATH + 0.1)
    assert _hold().candidate({1: t}) is None

  def test_near_track_on_path_is_held(self):
    t = _track(d_rel=30.0, d_path=RADAR_LEAD_HOLD_MAX_DPATH - 0.1)
    assert _hold().candidate({1: t}) is not None

  def test_far_track_gets_the_looser_limit(self):
    # same offset that is rejected near is accepted past FAR_DREL, where the path estimate is looser
    d_path = RADAR_LEAD_HOLD_MAX_DPATH + 0.2
    assert _hold().candidate({1: _track(d_rel=30.0, d_path=d_path)}) is None
    assert _hold().candidate({1: _track(d_rel=90.0, d_path=d_path)}) is not None

  def test_far_track_still_has_a_limit(self):
    t = _track(d_rel=90.0, d_path=RADAR_LEAD_HOLD_MAX_DPATH_FAR + 0.1)
    assert _hold().candidate({1: t}) is None

  def test_gate_is_symmetric(self):
    for sign in (1, -1):
      t = _track(d_rel=30.0, d_path=sign * (RADAR_LEAD_HOLD_MAX_DPATH + 0.1))
      assert _hold().candidate({1: t}) is None
