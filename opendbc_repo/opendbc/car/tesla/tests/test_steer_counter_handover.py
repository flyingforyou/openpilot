"""DAS_steeringControl's counter across the startup handover.

The panda keeps forwarding the stock module's idle frames until one of ours continues their
counter (tesla_legacy.h), so until card reports the panda's safety mode the counter is the stock
one + 1, never a repeat, and after that it counts on by itself. Before this, the counter was
frame-derived and jumped at every start, with the EPAS also going 7-89 ms without a frame.
"""
from types import SimpleNamespace

from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, DBC


def _controller():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_X_HW1, gen_empty_fingerprint(), [], False, False, False)
  return CarController(DBC[CAR.TESLA_MODEL_X_HW1], CP)


def _cs(stock):
  return SimpleNamespace(stock_steer_counter=stock)


def test_follows_the_stock_counter_until_the_panda_switches():
  cc = _controller()
  assert [cc._next_steer_counter(_cs(s)) for s in (14, 15, 0, 1)] == [15, 0, 1, 2]


def test_a_stale_stock_view_never_repeats_a_counter():
  cc = _controller()
  # the stock frame between two of ours had not reached us yet: stock + 1 would repeat 6
  assert [cc._next_steer_counter(_cs(s)) for s in (4, 5, 5, 7)] == [5, 6, 7, 8]


def test_counts_on_by_itself_once_the_panda_has_switched():
  cc = _controller()
  assert cc._next_steer_counter(_cs(9)) == 10
  cc.steer_counter_follow_stock = False
  # the stock module keeps running on bus 2, but it is no longer ours to follow
  assert [cc._next_steer_counter(_cs(s)) for s in (3, 3, 12)] == [11, 12, 13]


def test_no_stock_module_falls_back_to_a_free_running_counter():
  cc = _controller()
  first = cc._next_steer_counter(_cs(None))
  assert [cc._next_steer_counter(_cs(None)) for _ in range(3)] == [(first + i) % 16 for i in (1, 2, 3)]
