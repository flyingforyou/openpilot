import pytest

from tesla_diag.transport import elm327 as E
from tests.fakes import FakeSerial

FRAMES = ['102 70 94 10 27', '302 16 83 0C', 'garbage!!', '102 71 94 10 27']


def mon(fake, seconds=0.3, protocol='6'):
  d = E.Elm327(fake)
  d.reset()
  d.setup_display()
  return d, list(d.monitor(protocol, seconds))


def test_probe_reports_commands_and_never_sends_a_request():
  f = FakeSerial(unsupported={'ATCFC0'})
  caps = E.Elm327(f).probe()
  assert caps.version == 'ELM327 v1.5' and caps.voltage == '12.4V'
  assert caps.commands['ATCFC0'] == 'unsupported' and caps.silent_monitoring
  assert f.requests == []


def test_probe_without_silent_monitoring():
  caps = E.Elm327(FakeSerial(silent_ok=False)).probe()
  assert not caps.silent_monitoring


def test_monitor_yields_frames_garbage_and_stops_cleanly():
  f = FakeSerial(FRAMES)
  d, ps = mon(f)
  assert [p.frame.id for p in ps if p.frame] == [0x102, 0x302, 0x102]
  assert sum(p.garbage for p in ps) == 1
  assert d.mode is E.Mode.COMMAND and f.requests == []
  assert f.writes[-1] == ''          # the bare CR that stops ATMA


def test_monitor_refused_without_silent_confirmation_and_never_sends_atma():
  f = FakeSerial(FRAMES, silent_ok=False)
  d = E.Elm327(f)
  d.reset()
  with pytest.raises(E.DongleError, match='silent'):
    list(d.monitor('6', 0.2))
  assert 'ATMA' not in f.writes


def test_buffer_full_is_reported():
  _, ps = mon(FakeSerial(FRAMES, overflow=True))
  assert any(p.status == 'BUFFER FULL' for p in ps)


def test_at_refused_while_monitoring():
  d = E.Elm327(FakeSerial(FRAMES))
  d.reset()
  d.start_monitor('6')
  with pytest.raises(E.StateError):
    d.at('ATI')
  d.stop_monitor()
  d.at('ATI')


def test_missing_prompt_times_out_clearly():
  d = E.Elm327(FakeSerial(drop_prompt_for={'ATE0'}), cmd_timeout=0.3)
  d.reset()
  with pytest.raises(E.DongleTimeout):
    d.at('ATE0')


def test_disconnect_mid_session():
  f = FakeSerial(FRAMES, disconnect_after_writes=3)
  d = E.Elm327(f)
  with pytest.raises(E.DongleDisconnected):
    d.reset()
    d.setup_display()


def test_question_mark_is_rejected_not_ok():
  d = E.Elm327(FakeSerial(unsupported={'ATFOO'}))
  d.reset()
  with pytest.raises(E.DongleRejected):
    d.at('ATFOO')


def test_non_at_text_cannot_use_at():
  d = E.Elm327(FakeSerial())
  with pytest.raises(E.StateError):
    d.at('2701')


def test_only_plain_can_protocols():
  with pytest.raises(ValueError):
    E.Elm327(FakeSerial()).select_protocol('A')


def test_reply_parsing_single_multi_and_errors():
  assert E.parse_elm_reply(['62 F1 90 41']) == bytes([0x62, 0xF1, 0x90, 0x41])
  assert E.parse_elm_reply(['008', '0: 62 F1 90 41 42 43', '1: 44 45']) == bytes.fromhex('62F1904142434445')[:8]
  with pytest.raises(E.DongleTimeout):
    E.parse_elm_reply(['NO DATA'])
  with pytest.raises(E.DongleError):
    E.parse_elm_reply(['CAN ERROR'])
  with pytest.raises(E.DongleError, match='short'):
    E.parse_elm_reply(['010', '0: 62 F1 90'])
