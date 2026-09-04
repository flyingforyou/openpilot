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


def _cc(enabled=True, left=False, right=False):
  """The CarControl fields update_ic reads. controlsd only sets the blinkers while a lane change
  is running, and builds a fresh CarControl each cycle, so False/False is the normal case."""
  return SimpleNamespace(enabled=enabled, leftBlinker=left, rightBlinker=right)


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
    sends = cc.update_ic(_cc(), _carstate())
    assert isinstance(sends, list) and len(sends) >= 1

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_update_ic_disengaged_is_silent(self, fingerprint):
    cc = _carcontroller(fingerprint)
    assert cc.update_ic(_cc(enabled=False), _carstate()) == []

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_update_ic_with_leads_does_not_raise(self, fingerprint):
    """The lead path (ic_enabled + a radarState) must survive too."""
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    lead = SimpleNamespace(present=True, dRel=30.0, vRel=-2.0, yRel=0.5)
    cc.ic_radar = SimpleNamespace(leadOne=lead, leadTwo=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0))
    cc.frame = 10  # frame % 10 == 0 so send_leads fires
    sends = cc.update_ic(_cc(), _carstate())
    assert isinstance(sends, list)

  def test_ignored_off_hw1(self):
    """A non-HW1 fingerprint must never enter the IC body even if the flag is set."""
    cc = _carcontroller(CAR.TESLA_MODEL_3, ic_integration=True)
    assert cc.update_ic(_cc(), _carstate()) == []


class TestLaneChangeDashedLine:
  """The cluster draws the crossed line dashed off DAS_autoLaneChangeState. A stock capture
  (0000009f seg 1 t+56.5s) holds ALC_IN_PROGRESS_L for the manoeuvre with the blinker on, so that
  is what we emit. This was dead code for a while -- it keyed off an ic_model nothing set."""

  ALC_AVAILABLE_BOTH, ALC_IN_PROGRESS_L, ALC_IN_PROGRESS_R = 8, 9, 10

  def _alc(self, cc, CC):
    from opendbc.can import CANParser
    sends = cc.update_ic(CC, _carstate())
    frames = [(addr, bytes(dat), 0) for addr, dat, _bus in sends if addr == 0x399]
    assert frames, "no AutopilotStatus frame emitted"
    parser = CANParser("tesla_can", [("AutopilotStatus", 25)], 0)
    parser.update([(1_000_000, frames)])
    return int(parser.vl["AutopilotStatus"]["DAS_autoLaneChangeState"])

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_no_blinker_is_available_both(self, fingerprint):
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    assert self._alc(cc, _cc()) == self.ALC_AVAILABLE_BOTH

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_left_lane_change_goes_dashed_left(self, fingerprint):
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    assert self._alc(cc, _cc(left=True)) == self.ALC_IN_PROGRESS_L

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_right_lane_change_goes_dashed_right(self, fingerprint):
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    assert self._alc(cc, _cc(right=True)) == self.ALC_IN_PROGRESS_R

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_status_stays_active_through_the_change(self, fingerprint):
    """Stock kept autopilotStatus at 3 and handsOnState at 2 throughout; only ALC moved."""
    from opendbc.can import CANParser
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    sends = cc.update_ic(_cc(left=True), _carstate())
    frames = [(addr, bytes(dat), 0) for addr, dat, _bus in sends if addr == 0x399]
    parser = CANParser("tesla_can", [("AutopilotStatus", 25)], 0)
    parser.update([(1_000_000, frames)])
    assert int(parser.vl["AutopilotStatus"]["autopilotStatus"]) == 3


class TestFarLeadDisplayClamp:
  """TeslaICLeadMaxM pulls a distant lead in so the cluster will draw it. Display only -- nothing
  here may ever reach control, and it must never push a lead further away than it really is."""

  def _lead(self, d_rel):
    return SimpleNamespace(present=True, dRel=d_rel, vRel=-2.0, yRel=0.0)

  def test_off_is_truthful(self):
    assert CarController._ic_lead(self._lead(100.0), 0.0, 0.0)[0] == 100.0

  def test_far_lead_is_pulled_in(self):
    shown = CarController._ic_lead(self._lead(100.0), 0.0, 75.0)[0]
    assert 75.0 < shown < 100.0, "pulled in, but not parked on the threshold"

  def test_far_leads_keep_approaching(self):
    """A hard clamp put every far lead on exactly the threshold, so the car sat frozen there.
    Compressed distances must still shrink as the real one does."""
    shown = [CarController._ic_lead(self._lead(d), 0.0, 75.0)[0] for d in (120.0, 100.0, 90.0, 80.0)]
    assert shown == sorted(shown, reverse=True) and len(set(shown)) == 4

  def test_near_lead_is_untouched(self):
    assert CarController._ic_lead(self._lead(40.0), 0.0, 75.0)[0] == 40.0

  def test_never_reports_further_than_reality(self):
    for d in (10.0, 40.0, 74.9, 75.0, 90.0, 126.0):
      shown = CarController._ic_lead(self._lead(d), 0.0, 75.0)[0]
      assert shown <= d, "a lead must never be drawn further away than it is"

  def test_absent_lead_stays_absent(self):
    assert CarController._ic_lead(SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0), 0.0, 75.0) is None

  def test_field_range_is_respected(self):
    # DAS_objVehDx tops out at 127.5 and 127.5 is the "no object" saturation value
    assert CarController._ic_lead(self._lead(200.0), 0.0, 0.0)[0] <= 126.0


class TestNoEmptyLeadFrames:
  """DAS_object is shared with the factory, which is never blocked on it. An empty frame from us
  is not neutral -- it contradicts a car the factory is drawing -- so we stay quiet when we have
  nothing to add."""

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_silent_when_no_lead(self, fingerprint):
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    cc.frame = 10
    cc.ic_radar = SimpleNamespace(leadOne=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0),
                                  leadTwo=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0))
    sends = cc.update_ic(_cc(), _carstate())
    assert not any(addr == 0x309 for addr, _dat, _bus in sends), "sent an empty object frame"

  @pytest.mark.parametrize("fingerprint", HW1)
  def test_speaks_when_it_has_a_lead(self, fingerprint):
    cc = _carcontroller(fingerprint)
    cc.ic_enabled = True
    cc.frame = 10
    cc.ic_radar = SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=45.0, vRel=-1.0, yRel=0.0),
                                  leadTwo=SimpleNamespace(present=False, dRel=0.0, vRel=0.0, yRel=0.0))
    sends = cc.update_ic(_cc(), _carstate())
    assert any(addr == 0x309 for addr, _dat, _bus in sends)
