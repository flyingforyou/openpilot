import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.carstate import decode_tesla_gap
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR


@pytest.mark.parametrize("raw_gap, expected", [
  (0, 1),
  (33, 2),
  (66, 3),
  (100, 4),
  (133, 5),
  (166, 6),
  (200, 7),
  (255, 0),
  (1, 0),
  (99, 0),
])
def test_decode_tesla_gap(raw_gap, expected):
  assert decode_tesla_gap(raw_gap) == expected


class TestTeslaFingerprint:
  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC
