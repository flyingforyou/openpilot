from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.stopped_lead_early import StoppedLeadEarly, fires


def _fires(v_ego=14.0, prob=0.15, x=35.0, v_lead=7.0, on_ramp=True, radar=(False, 0.0)):
  return fires(v_ego, prob, x, v_lead, 0.0, [0.0, 100.0], [0.0, 0.0], radar[0], radar[1], on_ramp)


def _model(prob, x, v):
  lead = SimpleNamespace(prob=prob, x=[x], v=[v], y=[0.0])
  return SimpleNamespace(leadsV3=[lead], position=SimpleNamespace(x=[0.0, 100.0], y=[0.0, 0.0]))


NO_RADAR = SimpleNamespace(present=False, dRel=0.0)


class TestRampLimits:
  def test_ramp_queue_fires_only_on_a_ramp(self):
    assert _fires(on_ramp=True)
    assert not _fires(on_ramp=False)

  def test_each_highway_limit_is_kept_off_the_ramp(self):
    assert not _fires(v_ego=14.0, prob=0.5, x=60.0, v_lead=1.0, on_ramp=False)   # speed floor
    assert not _fires(v_ego=20.0, prob=0.15, x=60.0, v_lead=1.0, on_ramp=False)  # probability
    assert not _fires(v_ego=20.0, prob=0.5, x=35.0, v_lead=1.0, on_ramp=False)   # near limit
    assert not _fires(v_ego=20.0, prob=0.5, x=60.0, v_lead=10.0, on_ramp=False)  # slow-lead ratio
    assert _fires(v_ego=20.0, prob=0.5, x=60.0, v_lead=1.0, on_ramp=False)

  def test_ramp_still_has_limits(self):
    assert not _fires(v_ego=9.0)
    assert not _fires(prob=0.05)
    assert not _fires(x=25.0)
    assert not _fires(v_lead=9.0)

  def test_radar_match_hands_over_on_a_ramp(self):
    assert not _fires(radar=(True, 36.0))

  def test_update_passes_the_ramp_through(self):
    model = _model(0.15, 35.0, 1.0)
    assert StoppedLeadEarly(0.05).update(14.0, model, NO_RADAR) is None
    assert StoppedLeadEarly(0.05).update(14.0, model, NO_RADAR, True) < 0.0
