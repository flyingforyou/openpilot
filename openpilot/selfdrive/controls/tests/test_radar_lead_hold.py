from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import KalmanParams, RadarLeadHold, Track, get_lead

DT = 0.05
KP = KalmanParams(DT)


def track(identifier=1, d_rel=15.0, y_rel=0.0, v_rel=0.0, v_ego=20.0, measured=True, frames=5):
  """A radar track that has been measured steadily, as if we had been following it."""
  t = Track(identifier, v_rel + v_ego, KP)
  for _ in range(frames):
    t.update(d_rel, y_rel, v_rel, v_rel + v_ego, measured)
  return t


def vision_lead(prob=0.9, x=15.0, y=0.0, v=20.0):
  # get_lead/match_vision_to_track only read these fields off leadsV3
  return SimpleNamespace(prob=prob, x=[x], y=[y], v=[v], a=[0.0],
                         xStd=[1.0], yStd=[1.0], vStd=[1.0])


def configured_hold(dist=30.0, ms=1000):
  h = RadarLeadHold()
  h.configure(dist, ms)
  return h


class TestRadarLeadHoldGating:
  def test_disabled_by_default(self):
    h = RadarLeadHold()
    assert h.candidate({1: track()}) is None

  def test_holds_a_close_measured_track_it_was_following(self):
    h = configured_hold()
    t = track(d_rel=15.0)
    t.selected_count = 5
    h.track_id = 1
    assert h.candidate({1: t}) is t

  def test_never_holds_a_track_vision_never_confirmed(self):
    # selected_count == 0 means this was never the chosen lead: radar clutter must not become one
    h = configured_hold()
    t = track(d_rel=15.0)
    t.selected_count = 0
    h.track_id = 1
    assert h.candidate({1: t}) is None

  def test_releases_beyond_the_distance_gate(self):
    h = configured_hold(dist=30.0)
    t = track(d_rel=45.0)
    t.selected_count = 5
    h.track_id = 1
    assert h.candidate({1: t}) is None

  def test_releases_when_radar_stops_measuring(self):
    h = configured_hold()
    t = track(d_rel=15.0, measured=False)
    t.selected_count = 5   # set after update(), which would have zeroed it
    h.track_id = 1
    assert h.candidate({1: t}) is None

  def test_releases_when_the_track_disappears(self):
    h = configured_hold()
    h.track_id = 7
    assert h.candidate({1: track()}) is None

  def test_track_jump_clears_continuity(self):
    # Track.update zeroes selected_count on a discontinuity, which drops the hold
    t = track(d_rel=15.0)
    t.selected_count = 5
    t.update(40.0, 0.0, 0.0, 20.0, True)   # >TRACK_JUMP_D
    assert t.selected_count == 0
    h = configured_hold()
    h.track_id = 1
    assert h.candidate({1: t}) is None

  def test_budget_expires(self):
    h = configured_hold(ms=1000)   # 20 frames at DT=0.05
    t = track(d_rel=15.0)
    t.selected_count = 5
    h.track_id = 1
    h.frames = h.max_frames
    assert h.candidate({1: t}) is None


class TestRadarLeadHoldBookkeeping:
  def test_vision_agreement_restores_the_budget(self):
    h = configured_hold()
    h.track_id, h.frames, h.used = 1, 10, False
    h.observe({'status': True, 'radar': True, 'radarTrackId': 1})
    assert h.frames == 0

  def test_holding_burns_the_budget(self):
    h = configured_hold()
    h.track_id, h.frames, h.used = 1, 3, True
    h.observe({'status': True, 'radar': True, 'radarTrackId': 1})
    assert h.frames == 4
    assert not h.used

  def test_switching_track_resets(self):
    h = configured_hold()
    h.track_id, h.frames, h.used = 1, 8, True
    h.observe({'status': True, 'radar': True, 'radarTrackId': 2})
    assert (h.track_id, h.frames) == (2, 0)

  def test_no_lead_clears(self):
    h = configured_hold()
    h.track_id, h.frames = 1, 5
    h.observe({'status': False})
    assert (h.track_id, h.frames) == (-1, 0)

  def test_vision_only_lead_clears_the_anchor(self):
    h = configured_hold()
    h.track_id, h.frames = 1, 5
    h.observe({'status': True, 'radar': False, 'radarTrackId': -1})
    assert (h.track_id, h.frames) == (-1, 0)


class TestGetLeadWithHold:
  def test_vision_dropout_keeps_the_radar_distance(self):
    """The logged failure: vision prob collapses, radar still has the car at 5.6m."""
    tracks = {1: track(identifier=1, d_rel=5.6)}
    hold = configured_hold()

    # vision confident -> normal radar-backed lead, and the hold anchors on it
    lead = get_lead(20.0, True, tracks, vision_lead(prob=0.9, x=5.6 + 1.52), 20.0, hold=hold)
    hold.observe(lead)
    assert lead['status'] and lead['radar']
    assert hold.track_id == 1

    # vision loses confidence; without the hold this returns status=False
    lead = get_lead(20.0, True, tracks, vision_lead(prob=0.2), 20.0, hold=hold)
    assert lead['status'] and lead['radar']
    assert lead['dRel'] == 5.6
    assert hold.used

  def test_hold_beats_the_long_vision_fallback(self):
    """prob>0.5 but no sane match: stock code publishes vision's longer distance."""
    tracks = {1: track(identifier=1, d_rel=5.6)}
    far = vision_lead(prob=0.9, x=30.0, v=20.0)   # too far from the track to pass dist_sane

    plain = get_lead(20.0, True, tracks, far, 20.0)
    assert plain['status'] and not plain['radar']
    assert plain['dRel'] > 20.0                   # the +5m class of over-read we measured

    hold = configured_hold()
    hold.track_id = 1
    tracks[1].selected_count = 5
    held = get_lead(20.0, True, tracks, far, 20.0, hold=hold)
    assert held['status'] and held['radar']
    assert held['dRel'] == 5.6

  def test_disabled_hold_leaves_behaviour_unchanged(self):
    tracks = {1: track(identifier=1, d_rel=5.6)}
    tracks[1].selected_count = 5
    off = RadarLeadHold()          # never configured -> hold_dist 0
    off.track_id = 1
    assert get_lead(20.0, True, tracks, vision_lead(prob=0.2), 20.0, hold=off) == \
           get_lead(20.0, True, tracks, vision_lead(prob=0.2), 20.0)

  def test_hold_does_not_invent_a_lead_from_clutter(self):
    tracks = {1: track(identifier=1, d_rel=15.0)}   # selected_count stays 0: never confirmed
    hold = configured_hold()
    hold.track_id = 1
    lead = get_lead(20.0, True, tracks, vision_lead(prob=0.1), 20.0, hold=hold)
    assert not lead['status']
