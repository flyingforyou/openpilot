"""Driver-made disengagements are not control problems; the 11/10 right turn is the case that set the rule."""
from openpilot.tools.tesla_analysis.disengage_review import Disengagement, Sample, classify


def d(after, cause='brake', events=(), kph=62.4):
  return Disengagement(0.0, kph, cause, list(events), [Sample(*s) for s in after])


# 00000263 seg 21 from 41.0 s, one sample a second: (t, km/h, wheel deg, left, right)
RIGHT_TURN_1110 = [(0, 62.4, -10.4, 0, 0), (1, 59.3, -10.7, 0, 1), (2, 52.4, -9.8, 0, 1), (3, 42.8, -11.7, 0, 1),
                   (4, 35.2, -27.9, 0, 1), (5, 26.6, -92.2, 0, 1), (6, 21.0, -237.8, 0, 1), (7, 21.3, -256.8, 0, 1)]


def test_the_1110_right_turn_is_the_drivers():
  kind, why = classify(d(RIGHT_TURN_1110))
  assert kind == 'driver_turn' and why.startswith('right turn')


def test_a_turn_counts_whatever_ended_engagement():
  for cause in ('gas', 'steer', 'cancel'):
    assert classify(d(RIGHT_TURN_1110, cause))[0] == 'driver_turn'


def test_a_fault_is_a_problem_even_inside_a_turn():
  assert classify(d(RIGHT_TURN_1110, events=['canError']))[0] == 'problem'


def test_a_lane_change_is_the_drivers():
  # 00000263 seg 14: brake tap at 104 km/h, right blinker, no turn
  assert classify(d([(0, 104, -1, 0, 0), (0.3, 104, -1, 0, 1), (3, 109, -3, 0, 1)], kph=104))[0] == 'driver_lane_change'


def test_braking_without_a_turn_or_blinker_is_reviewed():
  assert classify(d([(0, 62, -10, 0, 0), (3, 40, -12, 0, 0), (8, 20, -15, 0, 0)]))[0] == 'review'


def test_a_fast_sweeping_bend_is_not_a_turn():
  # a big wheel angle only counts at turning speed
  assert classify(d([(0, 70, -10, 0, 0), (2, 65, -120, 0, 0)]))[0] == 'review'


def test_a_turn_too_long_after_is_not_this_ones():
  assert classify(d([(0, 50, 0, 0, 0), (9.0, 30, -5, 0, 0), (12, 20, -200, 0, 1)]))[0] == 'review'
