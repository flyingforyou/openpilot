import pytest

from openpilot.selfdrive.debug import intervention_log as il


class Bag:
  def __init__(self, **kw):
    self.__dict__.update(kw)


class FakeSM:
  """Just the four services the recorder reads off the real SubMaster."""
  seen = {'carState': True}

  def __init__(self, brake=False, steer=False, gas=False, enabled=True,
               a_target=-1.0, a_ego=0.5, lead=True, radar=True):
    self.m = {
      'carState': Bag(brakePressed=brake, steeringDisengage=steer, gasPressed=gas,
                      aEgo=a_ego, vEgo=20.0, steeringAngleDeg=3.0),
      'longitudinalPlan': Bag(aTarget=a_target),
      'selfdriveState': Bag(enabled=enabled),
      'radarState': Bag(leadOne=Bag(status=lead, radar=radar, dRel=18.4, vRel=-3.0)),
    }

  def __getitem__(self, k):
    return self.m[k]


@pytest.fixture
def log(tmp_path, monkeypatch):
  monkeypatch.setattr(il, 'EVENT_DIR', str(tmp_path))
  monkeypatch.setattr(il, 'PRE_N', 5)
  monkeypatch.setattr(il, 'POST_N', 3)
  monkeypatch.setattr(il, 'SAMPLE_HZ', 1e6)   # the rate limit is not what we're testing
  return il.InterventionLog()


def _settle(log, n=6, **kw):
  for _ in range(n):
    log.update(FakeSM(**kw))


def test_quiet_driving_records_nothing(log, tmp_path):
  _settle(log)
  assert log.pending is None
  assert il.list_events(str(tmp_path)) == []


def test_brake_while_engaged_records_the_pair(log, tmp_path):
  _settle(log)
  log.update(FakeSM(brake=True))
  assert log.pending is not None, "the takeover has to arm an event"

  for _ in range(il.POST_N):
    log.update(FakeSM(brake=True))
  assert log.pending is None, "and flush once the after-window is full"

  events = il.list_events(str(tmp_path))
  assert len(events) == 1
  e = events[0]
  assert e['cause'] == 'brake'
  # the whole point: openpilot wanted -1.0 while the car was doing +0.5
  assert e['disagreement'] == pytest.approx(-1.5)
  assert e['leadRadar'] and e['leadDRel'] == 18.4

  full = il.read_event(e['name'], str(tmp_path))
  assert full['samples'], "the window itself has to be there for replay"
  assert 'opAccel' in full['samples'][0] and 'aEgo' in full['samples'][0]


def test_held_pedal_is_one_event(log):
  _settle(log)
  log.update(FakeSM(brake=True))
  for _ in range(il.POST_N):
    log.update(FakeSM(brake=True))
  log.update(FakeSM(brake=True))
  assert log.pending is None, "a held brake must not re-arm on every frame"


def test_takeover_while_disengaged_grades_nobody(log, tmp_path):
  _settle(log, enabled=False)
  log.update(FakeSM(brake=True, enabled=False))
  assert log.pending is None
  assert il.list_events(str(tmp_path)) == []


def test_hard_steering_override_is_its_own_cause(log):
  _settle(log)
  log.update(FakeSM(steer=True))
  assert log.pending is not None
  assert log.pending['cause'] == 'steer'


def test_read_event_refuses_traversal(tmp_path):
  assert il.read_event('../../etc/passwd', str(tmp_path)) is None
  assert il.read_event('nope', str(tmp_path)) is None
