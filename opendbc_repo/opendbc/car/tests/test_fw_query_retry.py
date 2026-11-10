from unittest.mock import patch

from opendbc.car import car_helpers
from opendbc.car.structs import CarParams

EPS = b'1057658-00-00B' + b'\x00' * 10


def fw(brand):
  f = CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=EPS, address=0x730, responseAddress=0x738, brand=brand)
  f.request = [b'\x22\xf1\x88']
  return f


def run(answers, ecus=frozenset({(0x738, None, 0)})):
  """answers: what each successive ordered FW query returns."""
  calls = []

  def ordered(*args, **kwargs):
    calls.append(1)
    return answers[min(len(calls), len(answers)) - 1]

  with patch.object(car_helpers, 'get_vin', lambda *a, **k: (-1, -1, '0' * 17)), \
       patch.object(car_helpers, 'get_present_ecus', lambda *a, **k: set(ecus)), \
       patch.object(car_helpers, 'get_fw_versions_ordered', ordered), \
       patch.object(car_helpers, 'can_fingerprint', lambda *a, **k: (None, {})):
    fp, _, _, _, source, _ = car_helpers.fingerprint(lambda *a, **k: [], lambda *a: None, lambda *a: None, None)
  return fp, source, len(calls)


def test_a_missed_tesla_window_is_asked_again():
  # the 11/10 07:06 start: the EPS reply landed under mazda/ford, not tesla
  fp, source, n = run([[fw('mazda'), fw('ford')], [fw('tesla')]])
  assert fp == 'TESLA_MODEL_X_HW1' and source == CarParams.FingerprintSource.fw and n == 2


def test_a_good_first_answer_is_not_repeated():
  assert run([[fw('tesla')]])[2] == 1


def test_retries_are_bounded():
  fp, _, n = run([[fw('mazda')]])
  assert fp is None and n == 1 + car_helpers.FW_QUERY_RETRIES


def test_no_ecus_present_is_not_retried():
  assert run([[]], ecus=frozenset())[2] == 1
