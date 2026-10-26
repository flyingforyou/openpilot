import os

from tesla_diag.can import decoder, logger
from tesla_diag.can.frames import CanFrame
from tesla_diag.profiles import load_profile

FIX = os.path.join(os.path.dirname(__file__), '..', 'fixtures', 'synthetic_can.jsonl')
PROFILE = os.path.join(os.path.dirname(__file__), '..', 'tesla_diag', 'profiles', 'example_unverified.yaml')


def test_jsonl_roundtrip_keeps_raw(tmp_path):
  p = tmp_path / 'c.jsonl'
  fr = [CanFrame(0.5, 0x102, b'\x01\x02', raw='102 01 02'), CanFrame(1.0, 0x18DAF110, b'\xaa', True, raw='18 DA F1 10 AA')]
  with logger.JsonlWriter(str(p), {'x': 1}) as w:
    for f in fr:
      w.write(f)
  meta, back = logger.read_jsonl(str(p))
  assert meta['x'] == 1 and back == fr


def test_csv_and_candump(tmp_path):
  fr = [CanFrame(0.25, 0x102, b'\xde\xad')]
  logger.write_csv(fr, str(tmp_path / 'a.csv'))
  assert '102' in (tmp_path / 'a.csv').read_text()
  lines = list(logger.candump_lines(fr))
  assert lines == ['(0.250000) can0 102#DEAD']
  (tmp_path / 'a.log').write_text(lines[0] + '\n')
  assert logger.read_candump(str(tmp_path / 'a.log')) == fr


def test_fixture_is_marked_synthetic():
  meta, _ = logger.read_jsonl(FIX)
  assert meta['SYNTHETIC'] is True


def test_extract_little_and_big_endian():
  d = bytes([0x34, 0x12, 0, 0, 0, 0, 0, 0])
  assert decoder.extract(d, 0, 16) == 0x1234
  assert decoder.extract(bytes([0x12, 0x34]), 7, 16, 'big') == 0x1234
  assert decoder.extract(bytes([0xFF]), 0, 8, signed=True) == -1


def test_profile_signals_decode_the_synthetic_fixture():
  _, frames = logger.read_jsonl(FIX)
  res = load_profile(PROFILE).signal_table().decode_frames(frames)
  assert abs(res['BMS_packVoltage']['last'] - 380.04) < 1e-6
  assert res['BMS_socUI']['last'] == 80.0 and res['BMS_socMin']['last'] == 79.0
  assert res['BMS_socUI']['status'] == 'community-unverified'


def test_short_frame_is_not_decoded():
  s = decoder.Signal('x', 0x102, 0, 16, min_dlc=2)
  assert decoder.decode(s, CanFrame(0, 0x102, b'\x01')) is None
