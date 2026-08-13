"""Telling openpilot's own stalk presses apart from the driver's.

With cluster sync on, openpilot drives the stalk to make the car's MAX number match the map's
target, and those writes come back looking exactly like the driver turning it. Getting this wrong
is not a cosmetic bug: an override replaces the map's target outright, so one misread press stops
the map deciding the speed at all. On a real drive it did, for 98% of it.

The distinction is direction rather than identity. An override means the driver disagrees with the
map; a stalk moving toward the map's target is not a disagreement with it, whoever turned it.

These are written as the scenarios that have to be told apart rather than as unit assertions on a
formula, because the formula is only interesting insofar as it separates them.
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.map_cruise import MapCruiseController

MPH = CV.MPH_TO_MS


def controller(v_output_mph, stalk_mph, sync=True):
  c = MapCruiseController()
  c.sync_cluster = sync
  c.v_output = v_output_mph * MPH
  c.v_target = v_output_mph * MPH
  c.stalk_last = stalk_mph * MPH
  return c


def walk(c, values_mph):
  """Feed a sequence of stalk readings, as the DI would report them one detent at a time."""
  for v in values_mph:
    c._update_override(v * MPH)


class TestSyncEcho:
  def test_a_whole_ramp_is_not_an_override(self):
    """The bug. Sync walks the stalk from 40 to the map's 65 one detent at a time; every step
    along the way sits far from the target, and the old check -- has the stalk arrived yet --
    called each one a disagreement."""
    c = controller(65, 40)
    walk(c, [41, 42, 43, 48, 53, 58, 63, 65])
    assert c.override == 0.0

  def test_a_ramp_downward_is_not_an_override(self):
    c = controller(35, 60)
    walk(c, [59, 54, 49, 44, 39, 35])
    assert c.override == 0.0

  def test_the_driver_pulling_away_from_the_target_is_an_override(self):
    """Sitting on the map's target, the driver dials down. Nothing else could have done that."""
    c = controller(65, 65)
    walk(c, [64])
    assert c.override > 0.0

  def test_the_driver_pushing_past_the_target_is_an_override(self):
    """Overshooting is as clear a disagreement as pulling away -- 'no, faster than that here'."""
    c = controller(65, 64)
    walk(c, [65, 66, 67])
    assert c.override > 0.0

  def test_the_driver_reversing_mid_ramp_is_an_override(self):
    """Half way up a ramp the driver turns the other way. The steps so far are ours; the reversal
    is not, and it must be caught without waiting for the ramp to finish."""
    c = controller(65, 40)
    walk(c, [45, 50, 55])
    assert c.override == 0.0
    walk(c, [54])
    assert c.override > 0.0

  def test_a_press_the_car_ignores_is_not_an_override(self):
    """The DI answers 95-99% of presses, but not every one. A press that produces no movement
    must not look like anything at all."""
    c = controller(65, 50)
    walk(c, [50, 50, 50])
    assert c.override == 0.0

  def test_the_target_moving_under_a_still_stalk_is_not_an_override(self):
    """The map revising its own target does not mean the driver did anything."""
    c = controller(65, 50)
    c.v_output = 45 * MPH
    walk(c, [50])
    assert c.override == 0.0

  def test_the_driver_agreeing_with_the_map_is_not_an_override(self):
    """A driver winding toward the same number the map wants is not disagreeing with it. This is
    indistinguishable from our own presses and does not need to be distinguished: the override
    exists to record disagreement, and there is none here."""
    c = controller(65, 40)
    walk(c, [45, 50])
    assert c.override == 0.0

  def test_without_sync_every_change_is_the_driver(self):
    """Nothing but the driver touches the stalk when sync is off, so the direction rule must not
    apply and swallow a real override."""
    c = controller(65, 40, sync=False)
    walk(c, [41])
    assert c.override > 0.0
