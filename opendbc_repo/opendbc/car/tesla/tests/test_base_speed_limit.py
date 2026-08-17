import pytest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.carstate import snap_base_speed_limit

MPH = CV.MPH_TO_MS
KPH = CV.KPH_TO_MS
QUANTUM = 0.25   # UI_baseMapSpeedLimitMPS: 8 bits, scale 0.25, unit m/s


def as_received(limit_ms: float) -> float:
  """What the gateway actually puts on the bus: truncated onto the 0.25 m/s grid."""
  return int(limit_ms / QUANTUM) * QUANTUM


class TestSnapBaseSpeedLimit:
  @pytest.mark.parametrize("mph", [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75])
  def test_every_mph_limit_comes_back_exact(self, mph):
    assert snap_base_speed_limit(as_received(mph * MPH), False) == pytest.approx(mph * MPH)

  @pytest.mark.parametrize("kph", [30, 40, 50, 60, 70, 80, 90, 100, 110, 120])
  def test_every_kph_limit_comes_back_exact(self, kph):
    assert snap_base_speed_limit(as_received(kph * KPH), True) == pytest.approx(kph * KPH)

  @pytest.mark.parametrize("raw_ms,mph", [(15.50, 35), (17.75, 40), (20.00, 45), (29.00, 65)])
  def test_the_values_the_car_actually_sent(self, raw_ms, mph):
    """Straight off the 0000006b drive, where these read 34.673 / 39.706 / 44.739 / 64.871."""
    assert snap_base_speed_limit(raw_ms, False) / MPH == pytest.approx(mph, abs=0.01)

  def test_a_forty_clears_the_offset_split(self):
    """The whole point: truncation put a 40 zone under OFFSET_SPLIT, so the ladder paid +5."""
    from openpilot.selfdrive.controls.lib.map_cruise import OFFSET_SPLIT
    assert as_received(40 * MPH) < OFFSET_SPLIT
    assert snap_base_speed_limit(as_received(40 * MPH), False) >= OFFSET_SPLIT

  def test_no_value_stays_no_value(self):
    # 0 means "this source has no limit here", which is a real state on a ramp -- not a speed.
    assert snap_base_speed_limit(0.0, False) == 0.0
    assert snap_base_speed_limit(-1.0, False) == 0.0

  def test_correction_is_never_bigger_than_half_a_step(self):
    """It may only undo the truncation, never move a limit onto its neighbour."""
    for n in range(1, 256):
      raw = n * QUANTUM
      snapped = snap_base_speed_limit(raw, False)
      assert abs(snapped - raw) <= 2.5 * MPH + 1e-9
