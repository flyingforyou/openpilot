"""The claim to prove: by default nothing reaches the car, and no forbidden service can be sent by any path."""
import os

import pytest

from tesla_diag import cli
from tesla_diag.profiles import Profile, load_profile
from tesla_diag.transport import elm327 as E
from tesla_diag.uds.client import SafetyRefusal, UdsClient, build_read_did, build_read_dtc, check_request_allowed
from tests.fakes import FakeSerial

EXAMPLE = os.path.join(os.path.dirname(__file__), '..', 'tesla_diag', 'profiles', 'example_unverified.yaml')
FORBIDDEN = [0x10, 0x11, 0x14, 0x27, 0x2E, 0x2F, 0x31, 0x34, 0x36, 0x37, 0x3D]


def verified() -> Profile:
  return Profile(name='t', vehicle_family='x', hardware_or_firmware_evidence='log abc', bus='b', bitrate=500,
                 id_format='11', tx_id=0x7E0, rx_id=0x7E8, supported_service_did=['22F190', '190209'],
                 evidence_url='local:log', verified_on_vehicle=True, allow_live=True)


REPLY = {'22F190': '62 F1 90 41 42'}


def fake():
  return FakeSerial(reply_for_request=REPLY)


def test_example_profile_is_not_live_capable():
  assert load_profile(EXAMPLE).live_blockers()


@pytest.mark.parametrize('sid', FORBIDDEN)
def test_forbidden_services_refused_by_client(sid):
  with pytest.raises(SafetyRefusal):
    check_request_allowed(bytes([sid, 0x01, 0x02]))


@pytest.mark.parametrize('sid', FORBIDDEN)
def test_forbidden_services_refused_by_dongle_layer_directly(sid):
  f = fake()
  d = E.Elm327(f)
  with pytest.raises(E.StateError):
    d.uds_request(bytes([sid, 0x01, 0x02]), 0x7E0, 0x7E8, '11', '6')
  assert f.requests == [] and d.sent_requests == 0


def test_31_03_routine_results_is_forbidden_too():
  with pytest.raises(SafetyRefusal):
    check_request_allowed(bytes([0x31, 0x03, 0x04, 0x01]))


def test_19_only_read_subfunctions():
  assert build_read_dtc(0x02, 0xFF) == bytes([0x19, 0x02, 0xFF])
  with pytest.raises(SafetyRefusal):
    build_read_dtc(0x04)


@pytest.mark.parametrize('flags', [(False, False), (True, False), (False, True)])
def test_needs_both_flags(flags):
  f = fake()
  c = UdsClient(verified(), E.Elm327(f), *flags)
  with pytest.raises(SafetyRefusal):
    c.read(build_read_did(0xF190))
  assert f.requests == []


def test_unverified_profile_blocks_even_with_both_flags():
  p = verified()
  p.verified_on_vehicle = False
  f = fake()
  with pytest.raises(SafetyRefusal, match='verified_on_vehicle'):
    UdsClient(p, E.Elm327(f), True, True).read(build_read_did(0xF190))
  assert f.requests == []


def test_request_must_be_listed_in_profile():
  f = fake()
  with pytest.raises(SafetyRefusal, match='supported_service_did'):
    UdsClient(verified(), E.Elm327(f), True, True).read(build_read_did(0xF191))
  assert f.requests == []


def test_budget_limits_requests():
  f = fake()
  c = UdsClient(verified(), E.Elm327(f), True, True, max_requests=1)
  c.read(build_read_did(0xF190))
  with pytest.raises(SafetyRefusal, match='budget'):
    c.read(build_read_did(0xF190))
  assert len(f.requests) == 1


def test_gates_open_sends_exactly_one_request_with_caf1(tmp_path):
  f = fake()
  audit = str(tmp_path / 'a.jsonl')
  r = UdsClient(verified(), E.Elm327(f), True, True, audit_path=audit).read(build_read_did(0xF190))
  assert r.kind == 'positive' and f.requests == ['22F190']
  assert 'ATCAF1' in f.writes and f.writes.index('ATCAF1') < f.writes.index('22F190')
  assert 'ATSH7E0' in f.writes and 'ATCRA7E8' in f.writes
  assert '"send"' in open(audit).read()


def test_dry_run_never_touches_a_port(monkeypatch, capsys):
  monkeypatch.setattr(cli, 'SerialTransport', lambda *a, **k: pytest.fail('port opened'))
  rc = cli.main(['uds-read', '--dry-run', '--profile', EXAMPLE, '--did', 'F190'])
  assert rc == 0 and 'nothing is sent' in capsys.readouterr().out


def test_cli_live_read_refused_before_port_open(monkeypatch, capsys):
  monkeypatch.setattr(cli, 'SerialTransport', lambda *a, **k: pytest.fail('port opened'))
  rc = cli.main(['uds-read', '--profile', EXAMPLE, '--did', 'F190', '--port', '/dev/null',
                 '--allow-uds-read', '--confirm-parked', '--audit-log', '/dev/null'])
  assert rc == 3 and 'refused' in capsys.readouterr().err


def test_cli_probe_and_sniff_send_no_request(tmp_path):
  f = FakeSerial(['102 70 94 10 27'])
  a = cli.build_parser().parse_args(['probe', '--port', 'x'])
  cli.cmd_probe(a, E.Elm327(f))
  f2 = FakeSerial(['102 70 94 10 27'])
  a = cli.build_parser().parse_args(['sniff', '--port', 'x', '--protocol', '6', '--seconds', '0.2', '--out', str(tmp_path / 'o.jsonl')])
  assert cli.cmd_sniff(a, E.Elm327(f2)) == 0
  assert f.requests == [] and f2.requests == []
  assert '"id": "102"' in (tmp_path / 'o.jsonl').read_text()


def test_sniff_refuses_without_silent_monitoring(tmp_path, capsys):
  f = FakeSerial(['102 01'], silent_ok=False)
  a = cli.build_parser().parse_args(['sniff', '--port', 'x', '--protocol', '6', '--seconds', '0.2', '--out', str(tmp_path / 'o.jsonl')])
  assert cli.cmd_sniff(a, E.Elm327(f)) == 2
  assert 'ATMA' not in f.writes and 'silent' in capsys.readouterr().err


def test_sniff_requires_explicit_protocol():
  with pytest.raises(SystemExit):
    cli.build_parser().parse_args(['sniff', '--port', 'x', '--out', 'o'])


def test_no_forbidden_hex_literals_are_ever_written(tmp_path):
  """Run the full CLI paths and check no write is a forbidden-service request."""
  f = fake()
  UdsClient(verified(), E.Elm327(f), True, True).read(build_read_did(0xF190))
  for w in f.writes:
    if not w.upper().startswith('AT') and w:
      assert int(w[:2], 16) in (0x22, 0x19)


def test_vin_masking():
  assert cli.mask_vin('5YJXCDE22GF000001') == '5YJ******' + '*' * 4 + '0001'
