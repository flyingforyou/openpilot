import pytest

from tesla_diag.uds import isotp
from tesla_diag.uds.responses import UdsError, parse_dtcs, parse_read_did, parse_response


def test_single_frame_roundtrip():
  f = isotp.encode_single(bytes([0x22, 0xF1, 0x90]))
  assert f == bytes([3, 0x22, 0xF1, 0x90, 0, 0, 0, 0])
  assert isotp.Reassembler().feed(f) == bytes([0x22, 0xF1, 0x90])


def test_multi_frame_roundtrip_and_flow_control_flag():
  pdu = bytes([0x62, 0xF1, 0x90]) + b'ABCDEFGHIJKLMNOPQ'
  frames = isotp.encode_multi(pdu)
  r = isotp.Reassembler()
  assert r.feed(frames[0]) is None and r.need_flow_control
  out = None
  for f in frames[1:]:
    out = r.feed(f)
  assert out == pdu and not r.active


def test_sequence_number_wraps_at_16():
  pdu = bytes(range(256)) * 1
  frames = isotp.encode_multi(pdu)
  r = isotp.Reassembler()
  out = None
  for f in frames:
    out = r.feed(f)
  assert out == pdu


def test_wrong_sequence_is_error_and_drops_state():
  frames = isotp.encode_multi(bytes(30))
  r = isotp.Reassembler()
  r.feed(frames[0])
  with pytest.raises(isotp.IsoTpError, match='sequence'):
    r.feed(frames[2])
  assert not r.active


@pytest.mark.parametrize('bad', [b'', b'\x00\xaa', b'\x08' + bytes(7), b'\x10\x04' + bytes(6), b'\x10\x20\x01', b'\x35\x00\x00', b'\x40' + bytes(7)])
def test_bad_frames(bad):
  with pytest.raises(isotp.IsoTpError):
    isotp.parse_pci(bad)


def test_cf_without_ff():
  with pytest.raises(isotp.IsoTpError):
    isotp.Reassembler().feed(bytes([0x21]) + bytes(7))


def test_flow_control_encoding():
  p = isotp.parse_pci(isotp.encode_flow_control(isotp.FC_WAIT, 8, 5))
  assert (p.kind, p.fs, p.block_size, p.st_min) == (isotp.FC, 1, 8, 5)


def test_positive_and_did():
  r = parse_response(bytes([0x62, 0xF1, 0x90]) + b'XYZ', 0x22)
  assert r.kind == 'positive' and parse_read_did(r, 0xF190) == b'XYZ'
  with pytest.raises(UdsError):
    parse_read_did(r, 0xF191)


@pytest.mark.parametrize('nrc,name', [(0x11, 'serviceNotSupported'), (0x12, 'subFunctionNotSupported'),
                                       (0x13, 'incorrectMessageLengthOrInvalidFormat'), (0x22, 'conditionsNotCorrect'),
                                       (0x31, 'requestOutOfRange'), (0x33, 'securityAccessDenied')])
def test_negative(nrc, name):
  r = parse_response(bytes([0x7F, 0x22, nrc]), 0x22)
  assert r.kind == 'negative' and r.nrc_name == name


def test_pending_0x78_is_not_final():
  r = parse_response(bytes([0x7F, 0x22, 0x78]), 0x22)
  assert r.is_pending and r.kind == 'pending'


def test_mismatched_and_malformed():
  for bad in (b'', bytes([0x7F, 0x22]), bytes([0x59, 0x02]), bytes([0x7F, 0x19, 0x11])):
    with pytest.raises(UdsError):
      parse_response(bad, 0x22)


def test_dtc_list():
  r = parse_response(bytes([0x59, 0x02, 0xFF, 0x01, 0x02, 0x03, 0x2F, 0xAA, 0xBB, 0xCC, 0x08]), 0x19)
  assert parse_dtcs(r) == [{'dtc': '010203', 'status': 0x2F}, {'dtc': 'AABBCC', 'status': 0x08}]
