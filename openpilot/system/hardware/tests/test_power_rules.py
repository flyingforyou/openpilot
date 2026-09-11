"""Shutting down when the car sleeps and takes the harness with it."""
from openpilot.system.hardware.power_rules import (
  DEFAULT_OFFROAD_SHUTDOWN_MIN,
  battery_only_shutdown,
)

MIN = DEFAULT_OFFROAD_SHUTDOWN_MIN


def call(started_seen=True, in_car=False, ignition=False, offroad_s=60 * 60, timeout=MIN):
  return battery_only_shutdown(started_seen, in_car, ignition, offroad_s, timeout)


class TestFires:
  def test_the_observed_case(self):
    """Drove, parked, harness dead, an hour gone by."""
    assert call()

  def test_only_after_the_timeout(self):
    assert not call(offroad_s=MIN * 60 - 1)
    assert call(offroad_s=MIN * 60 + 1)

  def test_timeout_is_honoured(self):
    assert not call(offroad_s=10 * 60, timeout=30)
    assert call(offroad_s=10 * 60, timeout=5)


class TestDoesNotFire:
  def test_never_drove_this_boot(self):
    """A bench unit that never went onroad must not power itself off."""
    assert not call(started_seen=False, offroad_s=10 * 3600)

  def test_harness_still_powered(self):
    """in_car true means upstream's own rules apply; this one must stay out of the way."""
    assert not call(in_car=True, offroad_s=10 * 3600)

  def test_ignition_on(self):
    assert not call(ignition=True, offroad_s=10 * 3600)

  def test_disabled_by_zero(self):
    assert not call(timeout=0, offroad_s=10 * 3600)

  def test_disabled_by_negative(self):
    assert not call(timeout=-5, offroad_s=10 * 3600)

  def test_just_parked(self):
    assert not call(offroad_s=30)


def test_every_guard_is_individually_sufficient():
  """Each condition alone must block it, so no single wrong reading powers the car down."""
  assert call()
  for kw in ({"started_seen": False}, {"in_car": True}, {"ignition": True}, {"timeout": 0}):
    assert not call(**kw), kw


def test_returns_a_real_bool():
  assert type(call()) is bool
  assert type(call(in_car=True)) is bool
