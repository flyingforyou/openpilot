"""Listen-only capture: the ELM327 sniff, the summary, and a replay through the decoders."""
import pytest

from openpilot.system.btobd import capture
from openpilot.system.btobd.elm327 import Elm327
from openpilot.system.btobd.tests.test_elm327 import FakeDongle

SOC = '30282DF0D'                     # 0x302: 82 DF 0D -> socMin 89.8, socUI 88.7 (wk057)
ENERGY = '382FC9AEA69A900A000'        # 0x382 example frame from wk057
V102 = '1025C999F833C07'


def dongle(**kw):
  return FakeDongle({0x302: [SOC] * 3, 0x382: [ENERGY], 0x102: [V102] * 5}, **kw)


def test_sniff_hears_every_id_and_summarises():
  d = dongle()
  e = Elm327(d)
  e.init()
  frames, full, unparsed = e.sniff(0.05)
  assert {f[1] for f in frames} == {0x302, 0x382, 0x102} and not full and not unparsed
  s = capture.summarize(frames, 2.0)
  by = {i['id']: i for i in s['ids']}
  assert by['302']['n'] == 3 and by['302']['hz'] == 1.5 and by['302']['known'] == 'bms_soc'
  assert s['decoded']['socUI'] == pytest.approx(88.7)
  assert s['decoded']['nominalFullPackEnergy'] == pytest.approx(76.4)
  assert '6F2' in ' '.join(s['warnings'])          # a core battery ID that was not heard


def test_sniff_refuses_without_silent_monitoring():
  d = dongle(silent_ok=False)
  e = Elm327(d)
  e.init()
  with pytest.raises(RuntimeError):
    e.sniff(0.05)
  assert 'ATMA' not in d.sent


def test_buffer_full_is_reported_not_hidden():
  d = dongle()
  d.overflow = True
  e = Elm327(d)
  e.init()
  frames, full, _ = e.sniff(0.05)
  assert full and any('넘쳤' in w for w in capture.summarize(frames, 1.0, full)['warnings'])


def test_empty_capture_points_at_the_connection():
  s = capture.summarize([], 5.0)
  assert s['frames'] == 0 and '어댑터 케이블' in s['warnings'][0]


def test_only_at_commands_ever_reach_the_dongle():
  """Init, per-ID monitoring and a full sniff: every line sent is an AT command or a bare CR
  (which stops ATMA). No OBD, UDS or any other request frame is possible through this client."""
  d = dongle()
  e = Elm327(d)
  e.init()
  e.monitor(0x302, 0.05)
  e.sniff(0.05)
  assert d.sent and all(c == '' or c.startswith('AT') for c in d.sent), d.sent


def test_save_and_replay_round_trip(tmp_path):
  frames = [(0.1, 0x302, bytes.fromhex('82DF0D')), (0.2, 0x382, bytes.fromhex('FC9AEA69A900A000'))]
  name = capture.save(frames, {'protocol': 6}, str(tmp_path))
  assert capture.NAME_RE.fullmatch(name)
  back = capture.load(str(tmp_path / name))
  assert back == frames
  assert capture.summarize(back, 1.0)['decoded']['socUI'] == pytest.approx(88.7)


def test_old_captures_are_pruned(tmp_path):
  for i in range(capture.KEEP + 4):
    (tmp_path / f'capture-20260101-0000{i:02d}.jsonl').write_text('{}')
  capture.save([], {}, str(tmp_path))
  assert len(list(tmp_path.iterdir())) == capture.KEEP
