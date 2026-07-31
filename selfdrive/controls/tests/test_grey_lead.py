"""radard publishes a lead only once the model's leadsV3[0].prob clears 0.5, so below that the
planner sees clear road. Route 0000001b seg 10: it accelerated at the full 2.0 m/s^2 for 5.4s
toward a stopped car while prob crept 0.01 -> 0.47, and by the time the lead was published the
required decel was already past ACCEL_MIN.
"""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import GREY_LEAD_PROB_MAX, LongitudinalPlanner


def model(*probs):
  return SimpleNamespace(leadsV3=[SimpleNamespace(prob=p) for p in probs])


def lead(status=False):
  return SimpleNamespace(status=status)


def planner(prob_pct=20, cap_cms=0):
  p = LongitudinalPlanner.__new__(LongitudinalPlanner)
  p.grey_lead_prob = prob_pct / 100.0
  p.grey_lead_accel_cap = cap_cms / 100.0
  return p


class TestGreyLeadAhead:
  def test_holds_throttle_in_the_grey_band(self):
    assert planner(20).grey_lead_ahead(model(0.35), lead(status=False))

  def test_ignores_noise_below_the_threshold(self):
    assert not planner(20).grey_lead_ahead(model(0.05), lead(status=False))

  def test_defers_once_radard_publishes_a_lead(self):
    # above the gate there is a real lead and the MPC plans against it properly
    assert not planner(20).grey_lead_ahead(model(0.9), lead(status=True))
    assert not planner(20).grey_lead_ahead(model(GREY_LEAD_PROB_MAX), lead(status=False))

  def test_a_published_lead_always_wins_even_mid_band(self):
    assert not planner(20).grey_lead_ahead(model(0.35), lead(status=True))

  def test_disabled_by_default(self):
    assert not planner(0).grey_lead_ahead(model(0.35), lead(status=False))

  def test_threshold_is_respected(self):
    assert not planner(40).grey_lead_ahead(model(0.35), lead(status=False))
    assert planner(40).grey_lead_ahead(model(0.45), lead(status=False))

  def test_no_model_leads(self):
    assert not planner(20).grey_lead_ahead(model(), lead(status=False))

  def test_nan_prob_is_not_evidence(self):
    assert not planner(20).grey_lead_ahead(model(float('nan')), lead(status=False))

  def test_logged_failure_would_have_held(self):
    """The prob sequence logged on the approach to that stopped car."""
    p = planner(20)
    approach = [0.10, 0.14, 0.17, 0.20, 0.24, 0.26, 0.47, 0.45, 0.34, 0.40, 0.37]
    held = [p.grey_lead_ahead(model(x), lead(status=False)) for x in approach]
    assert not any(held[:3])       # noise floor: still free to accelerate
    assert all(held[4:])           # from prob 0.24 on, throttle is held
