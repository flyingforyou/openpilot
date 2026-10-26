import pytest

from tesla_diag.can.frames import CanFrame
from tesla_diag.can.parser import parse_line, protocol_id_format


def test_11bit_spaced_and_unspaced():
  a = parse_line('102 94 38 10 27', '11')
  b = parse_line('1029438' + '1027', '11')
  assert a.frame.id == 0x102 and a.frame.data.hex() == '94381027'
  assert b.frame.id == 0x102 and b.frame.data == a.frame.data


def test_29bit_spaced_is_four_tokens_and_unspaced_is_eight_chars():
  a = parse_line('18 DA F1 10 06 41 00', '29')
  b = parse_line('18DAF110064100', '29')
  assert a.frame.id == b.frame.id == 0x18DAF110 and a.frame.extended
  assert a.frame.data.hex() == '064100'


def test_extra_whitespace_and_lowercase():
  assert parse_line('  7e8   03  41  0c  ', '11').frame.data == bytes([3, 0x41, 0x0C])


@pytest.mark.parametrize('bad', ['ZZZ 01', '102 1', '102 123', '1', '102 01 02 03 04 05 06 07 08 09', '10 01 02', 'x' * 300])
def test_malformed_is_garbage_not_a_frame(bad):
  p = parse_line(bad, '11')
  assert p.frame is None and p.garbage


@pytest.mark.parametrize('word', ['BUFFER FULL', 'NO DATA', 'STOPPED', 'CAN ERROR', 'OK', '?'])
def test_status_words(word):
  p = parse_line(word)
  assert p.status == word and p.frame is None


def test_rtr_frame():
  p = parse_line('102 RTR', '11')
  assert p.frame.rtr and p.frame.data == b''


def test_empty_line_is_neither():
  p = parse_line('   ')
  assert p.frame is None and not p.garbage and p.status is None


def test_protocol_id_format():
  assert protocol_id_format('6') == '11' and protocol_id_format('9') == '29'
  with pytest.raises(ValueError):
    protocol_id_format('1')


def test_frame_range_checks():
  with pytest.raises(ValueError):
    CanFrame(0, 0x800, b'')
  with pytest.raises(ValueError):
    CanFrame(0, 1, bytes(9))
