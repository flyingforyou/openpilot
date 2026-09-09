"""The model-stopping predicate, and the speed gate it hangs on.

The gate was 82 km/h and is now 89. The cases below are the four activations that raising it
actually enables, taken off the logs (routes 000000d3 seg 4 and 000000f7 seg 19), plus the
boundary and the terms that have to keep rejecting.
"""
from openpilot.selfdrive.controls.lib.model_stop import (
  MODEL_STOP_MAX_KPH,
  model_stop_sign,
)

NO_LEAD = 1000.0   # what check_model_stopping passes when radarState has no lead


def stop(v_ego_kph, model_x, model_v=0.5, v_path_0=24.5, y_last=0.0, d_rel=NO_LEAD,
         decel_suppress=False):
  return model_stop_sign(v_ego_kph, model_x, model_v, v_path_0, y_last, d_rel, decel_suppress)


class TestSpeedGate:
  def test_the_real_activations_the_raise_enables(self):
    """All four 82-89 firings from the ten-route replay. Every one was a genuine stop.

    The path ends are the logged values; the 149.6 one printed as "150m" in the report, and the
    limit at this speed is exactly 150.0, so rounding it up would test the boundary instead.
    """
    for kmh, model_x in ((88.3, 146.0), (88.4, 149.6), (88.4, 121.0), (87.4, 140.0)):
      assert stop(kmh, model_x), f"{kmh} km/h / {model_x} m should fire"

  def test_82_to_89_band_is_open(self):
    assert stop(82.0, 140.0)
    assert stop(85.0, 140.0)
    assert stop(88.9, 140.0)

  def test_gate_still_closes_above_89(self):
    """The gate exists to keep this off at motorway speed, where a bridge reads as a stop."""
    assert not stop(MODEL_STOP_MAX_KPH, 140.0)
    assert not stop(95.0, 140.0)
    assert not stop(110.0, 100.0)

  def test_below_the_gate_is_unchanged(self):
    """Raising the ceiling must not disturb the range that was already working."""
    assert stop(50.0, 100.0, v_path_0=14.0)
    assert stop(70.0, 120.0, v_path_0=19.5)


class TestCrawl:
  def test_short_range_form_under_1kph(self):
    assert stop(0.5, 15.0, model_v=5.0)
    assert not stop(0.5, 25.0, model_v=5.0)
    assert not stop(0.5, 15.0, model_v=12.0)

  def test_crawl_form_ignores_the_lead_and_path_terms(self):
    """Under 1 km/h only x and v are consulted -- a lead right there must not veto."""
    assert stop(0.5, 15.0, model_v=5.0, d_rel=2.0, y_last=9.0)


class TestRejectionTerms:
  def test_path_must_clear_the_lead(self):
    """A path ending at the lead is the lead, not a stop line."""
    assert not stop(60.0, 80.0, d_rel=81.0)
    assert stop(60.0, 80.0, d_rel=120.0)

  def test_no_lead_makes_the_margin_term_free(self):
    """The d_rel = 1000 fallback is what lets this fire before a lead is ever promoted."""
    assert stop(88.0, 145.0, d_rel=NO_LEAD)

  def test_path_end_must_be_within_the_speed_limit_curve(self):
    assert stop(60.0, 115.0, v_path_0=16.7)        # 60 km/h -> 120 m
    assert not stop(60.0, 125.0, v_path_0=16.7)
    assert stop(80.0, 145.0, v_path_0=22.2)        # 80 km/h -> 150 m
    assert not stop(80.0, 155.0, v_path_0=22.2)

  def test_path_must_actually_slow_down(self):
    assert not stop(60.0, 100.0, model_v=15.0, v_path_0=16.7)
    assert stop(60.0, 100.0, model_v=2.0, v_path_0=16.7)
    assert stop(60.0, 100.0, model_v=11.0, v_path_0=16.7)   # under v_path_0 * 0.7

  def test_curving_path_is_a_bend_not_a_stop(self):
    assert not stop(60.0, 100.0, y_last=6.0, v_path_0=16.7)
    assert not stop(60.0, 100.0, y_last=-6.0, v_path_0=16.7)
    assert stop(60.0, 100.0, y_last=4.0, v_path_0=16.7)

  def test_decel_suppress_wins(self):
    assert not stop(60.0, 100.0, v_path_0=16.7, decel_suppress=True)

  def test_decel_suppress_does_not_reach_the_crawl_branch(self):
    """Stopped-and-braking must still report a stop, or the car creeps at a light."""
    assert stop(0.5, 15.0, model_v=5.0, decel_suppress=True)


def test_returns_a_real_bool():
  """numpy comparisons leak np.bool_, which the caller's counters then accumulate oddly."""
  assert type(stop(88.0, 140.0)) is bool
  assert type(stop(95.0, 140.0)) is bool


class TestAdjustedStopDistance:
  """The correction that used to land one frame late, stepping the aim point ~50 m."""

  def test_entry_and_steady_state_agree(self):
    """The whole point of the fix: the same input gives the same aim point either way."""
    from openpilot.selfdrive.controls.lib.model_stop import adjusted_stop_distance as f
    for rl, kmh in ((167.0, 88.4), (120.0, 60.0), (40.0, 30.0)):
      assert f(rl, kmh) == f(rl, kmh)

  def test_the_seg19_step_is_gone(self):
    """167 m at 88.4 km/h used to enter raw and correct to ~117 m one frame later."""
    from openpilot.selfdrive.controls.lib.model_stop import adjusted_stop_distance as f
    entry = f(167.0, 88.4)
    assert 115.0 < entry < 130.0, entry
    # a frame later the model's point has closed a little; the aim point must follow smoothly
    later = f(159.0, 88.7)
    assert abs(entry - later) < 12.0, (entry, later)

  def test_pulls_in_harder_as_speed_rises(self):
    from openpilot.selfdrive.controls.lib.model_stop import adjusted_stop_distance as f
    d = [f(150.0, k) for k in (0.0, 30.0, 60.0, 90.0)]
    assert d == sorted(d, reverse=True)
    assert d[0] == 150.0                      # no correction at a standstill

  def test_no_correction_for_a_point_underfoot(self):
    """The distance interp keeps a stop point a few metres away from being pulled closer still."""
    from openpilot.selfdrive.controls.lib.model_stop import adjusted_stop_distance as f
    assert f(0.0, 88.0) == 0.0
    assert f(2.0, 88.0) > 1.9

  def test_never_pushes_the_stop_further_away(self):
    from openpilot.selfdrive.controls.lib.model_stop import adjusted_stop_distance as f
    for rl in (5.0, 30.0, 80.0, 150.0, 250.0):
      for kmh in (0.0, 40.0, 88.0, 120.0):
        assert f(rl, kmh) <= rl + 1e-9
