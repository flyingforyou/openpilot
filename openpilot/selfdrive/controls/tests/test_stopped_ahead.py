"""stopped_ahead: a stationary radar track becomes the lead early only with the model's backing.

Numbers are from the drives the rule was built on: 025c seg 19 (11/09), a car stopped on a straight
that the model read as moving for 3 s, and the cases it has to stay out of.
"""
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import stopped_ahead
from openpilot.selfdrive.controls.lib.stopped_ahead import COMMIT, COUNT_MAX, corroborated, update
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, KalmanParams, Track, get_lead

V_EGO = 15.8   # 57 km/h


def _track(d_rel=84.5, v_lead=0.3, path_offset=0.9, cnt=78, measured=True):
  return SimpleNamespace(dRel=d_rel, vLead=v_lead, pathOffset=path_offset, cnt=cnt, measured=measured,
                         stopped_ahead_count=0, stopped_ahead=False)


def _ok(track=None, v_ego=V_EGO, prob=0.62, vision_dist=74.0, vision_v=10.0):
  return corroborated(v_ego, prob, vision_dist, vision_v, track or _track())


class TestCorroborated:
  def test_025c_stopped_car_the_model_reads_as_moving(self):
    # radar: 84.5 m, 0.3 m/s, 0.9 m off the path; model: P 0.62, 75.5 m, 36 km/h
    assert _ok()

  def test_low_probability_is_enough(self):
    # 022e seg 14, ramp queue: P 0.26 at 82 m against a track at 80 m
    assert _ok(_track(d_rel=80.3, path_offset=-1.3, cnt=34), v_ego=15.6, prob=0.26, vision_dist=80.8, vision_v=10.8)

  def test_not_without_the_model(self):
    # a gantry on the motorway: on the path, stationary, and the model sees nothing there
    assert not _ok(prob=0.05)

  def test_not_when_the_model_reads_its_lead_as_fast_as_us(self):
    # the same car 1.5 s earlier: the model had it at 49 km/h against our 52
    assert not _ok(v_ego=14.4, vision_v=13.6)

  def test_not_when_the_model_lead_is_somewhere_else(self):
    assert not _ok(vision_dist=40.0)

  def test_not_off_the_path(self):
    assert not _ok(_track(path_offset=2.6))

  def test_not_a_moving_track(self):
    assert not _ok(_track(v_lead=3.0))

  def test_not_a_track_that_just_appeared(self):
    assert not _ok(_track(cnt=2))

  def test_not_an_unmeasured_track(self):
    assert not _ok(_track(measured=False))

  def test_not_inside_the_time_gap(self):
    # 0235 seg 13: a stationary return 1.7 m beside the path 30 m ahead at 46 km/h, with the real
    # queue behind it; picking it would have been a hard brake for the wrong object
    assert not _ok(_track(d_rel=29.8, path_offset=1.7, cnt=33), v_ego=12.8, prob=0.23, vision_dist=40.5, vision_v=7.2)

  def test_not_at_walking_pace(self):
    assert not _ok(_track(d_rel=20.0), v_ego=3.0, vision_dist=20.0, vision_v=0.0)


class TestLatch:
  def test_latches_after_commit_frames_and_not_before(self):
    tracks = {1: _track()}
    for _ in range(COMMIT - 1):
      assert update(V_EGO, 0.62, 74.0, 10.0, tracks) is None
    assert update(V_EGO, 0.62, 74.0, 10.0, tracks) is tracks[1]

  def test_survives_a_gap_as_long_as_the_evidence(self):
    tracks = {1: _track()}
    for _ in range(COMMIT + 2):
      update(V_EGO, 0.62, 74.0, 10.0, tracks)
    for _ in range(COMMIT + 1):                     # the model's probability dips
      assert update(V_EGO, 0.05, 74.0, 10.0, tracks) is tracks[1]
    assert update(V_EGO, 0.05, 74.0, 10.0, tracks) is None

  def test_evidence_is_capped(self):
    tracks = {1: _track()}
    for _ in range(100):
      update(V_EGO, 0.62, 74.0, 10.0, tracks)
    for _ in range(COUNT_MAX):
      update(V_EGO, 0.05, 74.0, 10.0, tracks)
    assert update(V_EGO, 0.05, 74.0, 10.0, tracks) is None

  def test_nearest_latched_track_wins(self):
    tracks = {1: _track(d_rel=84.5), 2: _track(d_rel=74.1, path_offset=0.2)}
    for _ in range(COMMIT):
      best = update(V_EGO, 0.62, 74.0, 10.0, tracks)
    assert best is tracks[2]

  def test_reading_without_counting_changes_nothing(self):
    tracks = {1: _track()}
    for _ in range(10):
      assert update(V_EGO, 0.62, 74.0, 10.0, tracks, count=False) is None
    assert tracks[1].stopped_ahead_count == 0


def _radar_track(d_rel=84.5, v_lead=0.3, n=10):
  t = Track(1, v_lead, KalmanParams(DT_MDL))
  for _ in range(n):
    t.update(d_rel, 0.0, v_lead - V_EGO, v_lead, True)
  t.pathOffset = 0.9
  return t


def _lead_msg(prob=0.3, x=75.5, v=10.0):
  return SimpleNamespace(prob=prob, x=[x], v=[v], y=[0.0])


class TestGetLead:
  """Below P 0.5 radard used to publish no lead at all, whatever the radar had."""

  def test_stopped_track_becomes_the_lead_below_half_probability(self):
    tracks = {1: _radar_track()}
    for i in range(COMMIT):
      lead = get_lead(V_EGO, True, tracks, _lead_msg(), V_EGO, stopped_ahead_enabled=True)
      assert lead['present'] == (i == COMMIT - 1)
    assert lead['radar'] and lead['radarTrackId'] == 1 and lead['dRel'] == 84.5

  def test_off_by_default_and_when_disabled(self):
    tracks = {1: _radar_track()}
    for _ in range(COMMIT + 2):
      assert not get_lead(V_EGO, True, tracks, _lead_msg(), V_EGO)['present']

  def test_vision_distance_is_compared_at_the_radar(self):
    # the model measures from the camera; the comparison has to take RADAR_TO_CAMERA off
    seen = []
    orig = stopped_ahead.update
    stopped_ahead.update = lambda v, p, d, vv, tr, count=True: seen.append(d)
    try:
      get_lead(V_EGO, True, {1: _radar_track()}, _lead_msg(x=75.5), V_EGO, stopped_ahead_enabled=True)
    finally:
      stopped_ahead.update = orig
    assert seen == [75.5 - RADAR_TO_CAMERA]

  def test_evidence_starts_over_when_the_radar_stops_measuring(self):
    t = _radar_track()
    tracks = {1: t}
    for _ in range(COMMIT):
      get_lead(V_EGO, True, tracks, _lead_msg(), V_EGO, stopped_ahead_enabled=True)
    assert t.stopped_ahead
    t.update(84.0, 0.0, 0.3 - V_EGO, 0.3, False)
    assert not t.stopped_ahead and t.stopped_ahead_count == 0
