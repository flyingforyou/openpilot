"""The IC integration runs inside carcontroller.update(), i.e. inside the control process. A crash
here does not just blank the cluster -- it takes card down, which stops all CAN TX, and the car's
own TACC faults ~0.4s later for lack of an ACC command. That is exactly what shipped once:
update_ic read self.ic_model after the modelV2 plumbing had been dropped, and because its body
first runs the frame openpilot engages (it early-returns while disengaged), the AttributeError
crashed card precisely at engage, every drive.

So the thing worth pinning is not the drawn output but that update_ic never raises through the
engage transition, whatever optional inputs (ic_model, ic_radar) the plumbing does or does not set.
Run before every deploy: `pytest opendbc/car/tesla/tests/`.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, DBC, TeslaFlags

HW1 = (CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_HW1)


def _carcontroller(fingerprint, ic_integration=True):
  CP = CarInterface.get_non_essential_params(fingerprint)
  if ic_integration:
    CP.flags |= TeslaFlags.IC_INTEGRATION.value
  return CarController(DBC[fingerprint], CP)


def _carstate(**overrides):
  """The handful of fields update_ic reads off the python CarState (not CS.out). Defaults are a
  frame the factory has been seen -- both the lane and status frames are fresh -- so update_ic
  runs its full body rather than skipping on a None."""
  cs = SimpleNamespace(das_lanes={}, das_lanes_nanos=1, autopilot_status={},
                       autopilot_status_nanos=1, das_vehicles={}, hands_on_level=0)
  cs.__dict__.update(overrides)
  return cs


class TestICNeverCrashesControl:
  @pytest.mark.parametrize("fingerprint", HW1)
  def test_update_ic_at_engage_does_not_raise(self, fingerprint):
    """The exact shape of the shipped crash: armed, engaging, with a fresh status frame to send,
    and no modelV2 plumbing (ic_model unset by card). Must return frames, not raise."""
    cc = _carcontroller(fingerprint)
    sends = cc.update_ic(SimpleNamespace(enabled=True), _carstate())
    assert isinstance(sends, list) and len(sends) >= 1

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_update_ic_disengaged_is_silent(self, fingerprint):
    cc = _carcontroller(fingerprint)
    assert cc.update_ic(SimpleNamespace(enabled=False), _carstate()) == []

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_update_ic_with_leads_does_not_raise(self, fingerprint):
    """The lead path (ic_enabled + a radarState) must survive too."""
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    lead = SimpleNamespace(present=True, dRel=30.0, vRel=-2.0, yRel=0.5)
    cc.ic_radar = SimpleNamespace(leadOne=lead, leadTwo=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0))
    cc.frame = 10  # frame % 10 == 0 so send_leads fires
    sends = cc.update_ic(SimpleNamespace(enabled=True), _carstate())
    assert isinstance(sends, list)

  def test_ignored_off_hw1(self):
    """A non-HW1 fingerprint must never enter the IC body even if the flag is set."""
    cc = _carcontroller(CAR.TESLA_MODEL_3, ic_integration=True)
    assert cc.update_ic(SimpleNamespace(enabled=True), _carstate()) == []
