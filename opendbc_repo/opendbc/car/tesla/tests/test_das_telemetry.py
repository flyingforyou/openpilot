"""DAS_telemetry and GTW_ESP1, pinned against frames captured from the car.

DAS_telemetry is the camera grading its own view: how confidently it is tracking the lane
markings either side. Nothing else available reports on the camera's health directly -- its own
DAS_pmmCameraFaultReason reads NO_FAULT even through stretches where it reports no vehicles at
all -- so this is the one instrument that can tell a degraded camera from an empty road.

GTW_ESP1 is here for the ambient temperature, which is what makes it possible to test the
condensation explanation for those stretches against something other than memory.
"""
from opendbc.can import CANParser

DBC = 'tesla_can'


def decode(msg_name: str, addr: int, frame_hex: str):
  parser = CANParser(DBC, [(msg_name, 0)], 0)
  parser.update([0, [(addr, bytes.fromhex(frame_hex), 0)]])
  return parser.vl[msg_name]


class TestDasTelemetry:
  def test_both_markers_tracked_well(self):
    """Solid markings both sides: same lane type, right marker at full confidence."""
    vl = decode('DAS_telemetry', 937, '001b030000000000')
    assert vl['DAS_telemetryMultiplexer'] == 0
    assert vl['DAS_telLeftLaneType'] == 3
    assert vl['DAS_telRightLaneType'] == 3
    assert vl['DAS_telRightMarkerQuality'] == 3
    assert vl['DAS_telLeftLaneCrossing'] == 0
    assert vl['DAS_telRightLaneCrossing'] == 0

  def test_quality_is_reported_per_side(self):
    """The two sides disagree here -- left at 1, right at 3 -- which is the whole point of the
    signal and would be invisible if the two fields overlapped."""
    vl = decode('DAS_telemetry', 937, '005b130000000000')
    assert vl['DAS_telLeftMarkerQuality'] == 1
    assert vl['DAS_telRightMarkerQuality'] == 3

  def test_seeing_nothing(self):
    """An all-zero frame: no lane type, no confidence either side. This is what the message looks
    like when the camera has nothing, and it has to stay distinguishable from good tracking."""
    vl = decode('DAS_telemetry', 937, '0000000000000000')
    assert vl['DAS_telLeftLaneType'] == 0
    assert vl['DAS_telRightLaneType'] == 0
    assert vl['DAS_telLeftMarkerQuality'] == 0
    assert vl['DAS_telRightMarkerQuality'] == 0

  def test_marker_colour_does_not_bleed_into_quality(self):
    """Colour sits directly above the quality fields, so a one-bit slip swaps them. This frame
    has a coloured left marker and a quality that must stay independent of it."""
    vl = decode('DAS_telemetry', 937, '0044080000000000')
    assert vl['DAS_telLeftMarkerColor'] == 2
    assert vl['DAS_telLeftMarkerQuality'] == 1
    assert vl['DAS_telRightMarkerQuality'] == 0


class TestGtwEsp1:
  def test_ambient_temperature(self):
    """Three consecutive frames a half-degree apart, checksum tracking with them -- the scaling is
    0.5 degC per bit off a -40 offset, and getting the offset wrong still yields a road-plausible
    number, so the steps are what pin it."""
    for frame, expected in (('0f73008c', 17.5), ('0f74008d', 18.0), ('0f75008e', 18.5)):
      vl = decode('GTW_ESP1', 520, frame)
      assert abs(vl['GTW_ambientTemperature'] - expected) < 1e-6

  def test_flags_alongside_temperature(self):
    vl = decode('GTW_ESP1', 520, '0f73008c')
    assert vl['GTW_hillStartAssistEnabled'] == 1
    assert vl['GTW_brakeFluidLow'] == 0
    assert vl['GTW_espModeSwitch'] == 0
