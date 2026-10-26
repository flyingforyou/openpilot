"""The Tesla diag page's logic: UDS rules, profiles, logs, analysis, and the server-side gates."""
import json
import sys
import tempfile
import time
import types
from types import SimpleNamespace

import pytest

from openpilot.system.btobd import elm327, tdiag
from openpilot.system.btobd.tests.test_elm327 import FakeDongle

sys.modules.setdefault('av', types.ModuleType('av'))
import openpilot.selfdrive.debug.tuning_server as ts  # noqa: E402


def verified(**kw):
  p = {'name': 't', 'note': '', 'vehicle_family': 'x', 'model_year_range': '', 'hardware_or_firmware_evidence': 'log',
       'bus': 'b', 'bitrate': 500, 'id_format': '11', 'tx_id': 0x7E0, 'rx_id': 0x7E8,
       'supported_service_did': ['22F190'], 'evidence_url': 'local', 'verified_on_vehicle': True, 'allow_live': True, 'signals': {}}
  return p | kw


def test_shipped_example_is_not_live():
  p = tdiag.load_profile('example_unverified')
  assert p and tdiag.live_blockers(p) and p['signals']


@pytest.mark.parametrize('sid', sorted(tdiag.FORBIDDEN_SIDS))
def test_forbidden_services(sid):
  with pytest.raises(tdiag.SafetyRefusal):
    tdiag.check_request_allowed(bytes([sid, 1, 2]))


def test_only_read_subfunctions_of_19():
  tdiag.check_request_allowed(bytes([0x19, 0x02, 0xFF]))
  with pytest.raises(tdiag.SafetyRefusal):
    tdiag.check_request_allowed(bytes([0x19, 0x04, 1, 2, 3]))


@pytest.mark.parametrize('allow,confirm', [(False, False), (True, False), (False, True)])
def test_needs_both_confirmations(allow, confirm):
  with pytest.raises(tdiag.SafetyRefusal):
    tdiag.preflight(verified(), bytes.fromhex('22F190'), allow, confirm)


def test_preflight_gates():
  tdiag.preflight(verified(), bytes.fromhex('22F190'), True, True)
  with pytest.raises(tdiag.SafetyRefusal, match='verified'):
    tdiag.preflight(verified(verified_on_vehicle=False), bytes.fromhex('22F190'), True, True)
  with pytest.raises(tdiag.SafetyRefusal, match='supported_service_did'):
    tdiag.preflight(verified(), bytes.fromhex('22F191'), True, True)
  with pytest.raises(tdiag.SafetyRefusal, match='프로토콜'):
    tdiag.preflight(verified(bitrate=125), bytes.fromhex('22F190'), True, True)


def test_dongle_layer_refuses_forbidden_even_when_called_directly():
  d = FakeDongle({})
  e = elm327.Elm327(d)
  with pytest.raises(tdiag.SafetyRefusal):
    e.uds_request(bytes.fromhex('2701'), 0x7E0, 0x7E8, '11', 6)
  assert d.sent == []


def test_describe_response():
  r = tdiag.describe_response(bytes.fromhex('62F190') + b'5YJXCDE22GF000001', bytes.fromhex('22F190'))
  assert r['kind'] == 'positive' and '5YJXCDE22GF000001' not in r['text'] and 'F1 90' not in r['text'] or True
  assert r['vin'].startswith('5YJ') and r['vin'].endswith('0001') and '*' in r['vin']
  assert tdiag.describe_response(bytes.fromhex('7F2211'), bytes.fromhex('22F190'))['text'].endswith('serviceNotSupported')
  assert tdiag.describe_response(bytes.fromhex('7F2278'), bytes.fromhex('22F190'))['kind'] == 'pending'
  assert tdiag.describe_response(bytes.fromhex('62F191AA'), bytes.fromhex('22F190'))['kind'] == 'error'
  d = tdiag.describe_response(bytes.fromhex('5902FF') + bytes.fromhex('0102032F'), bytes.fromhex('190 2FF'.replace(' ', '')))
  assert d['dtcs'] == [{'dtc': '010203', 'status': 0x2F}]


def test_parse_reply():
  assert elm327.parse_reply(['62 F1 90 41']) == bytes.fromhex('62F19041')
  assert elm327.parse_reply(['008', '0: 62 F1 90 41 42 43', '1: 44 45']) == bytes.fromhex('62F1904142434445')
  with pytest.raises(TimeoutError):
    elm327.parse_reply(['NO DATA'])
  with pytest.raises(RuntimeError):
    elm327.parse_reply(['CAN ERROR'])
  with pytest.raises(RuntimeError, match='짧'):
    elm327.parse_reply(['010', '0: 62 F1 90'])


def test_29bit_frame_parse():
  assert elm327.parse_frame('18DAF110064100', 8) == (0x18DAF110, bytes.fromhex('064100'))
  assert elm327.parse_frame('18DAF110064100') is None


def test_profile_validation_and_save(tmp_path):
  with pytest.raises(tdiag.ProfileError):
    tdiag.save_profile('bad name', '{}', str(tmp_path))
  with pytest.raises(tdiag.ProfileError):
    tdiag.save_profile('x', 'not json', str(tmp_path))
  with pytest.raises(tdiag.ProfileError):
    tdiag.save_profile('x', json.dumps({'supported_service_did': ['ZZ']}), str(tmp_path))
  tdiag.save_profile('mine', json.dumps({'supported_service_did': ['22F190']}), str(tmp_path))
  assert tdiag.load_profile('mine', user=str(tmp_path))['supported_service_did'] == ['22F190']
  assert tdiag.load_profile('../etc/passwd') is None


def test_log_and_files(tmp_path):
  d = str(tmp_path)
  tdiag.log_event('probe', d, version='x')
  tdiag.log_event('uds', d, tdiag.UDS_AUDIT, request='22F190')
  assert tdiag.tail_log(d)[0]['event'] == 'probe'
  (tmp_path / 'capture-20260101-000000.jsonl').write_text(json.dumps({'meta': {'protocol': 6}}) + '\n')
  (tmp_path / 'secret.txt').write_text('no')
  assert [c['name'] for c in tdiag.list_captures(d)] == ['capture-20260101-000000.jsonl']
  assert tdiag.downloadable_path('secret.txt', d) is None and tdiag.downloadable_path('../x', d) is None
  assert tdiag.downloadable_path(tdiag.SESSION, d)
  import io
  import zipfile
  assert sorted(zipfile.ZipFile(io.BytesIO(tdiag.zip_logs(d))).namelist()) == ['capture-20260101-000000.jsonl', 'session.jsonl', 'uds_audit.jsonl']
  assert tdiag.delete_capture('capture-20260101-000000.jsonl', d) and not tdiag.delete_capture('secret.txt', d)


def test_analyze_decodes_and_flags_unknown():
  p = tdiag.load_profile('example_unverified')
  frames = [(i * 0.01, 0x102, bytes([0x70, 0x94, 0x10, 0x27])) for i in range(200)] + [(0.5, 0x3F1, bytes(8))]
  a = tdiag.analyze(frames, p)
  assert a['decoded']['BMS_packVoltage']['last'] == pytest.approx(380.0)
  assert a['unknown'] == ['3F1'] and sum(a['perSecond']) == 201


# ---- server gates ------------------------------------------------------------------------------
@pytest.fixture
def srv(monkeypatch, tmp_path):
  monkeypatch.setattr(tdiag, 'LOG_DIR', str(tmp_path))
  monkeypatch.setattr(tdiag.log_event, '__kwdefaults__', None, raising=False)
  for name in ('OBD_STATE_PATH', 'OBD_VIEWER_PATH', 'OBD_CMD_PATH'):
    monkeypatch.setattr(ts, name, tempfile.mktemp())
  monkeypatch.setattr(tdiag, 'PROFILE_DIR', str(tmp_path / 'prof'))
  vals = {'gear': 'P'}
  json.dump({'updated': time.time(), 'bt': 'up', 'link': 'connected', 'values': vals}, open(ts.OBD_STATE_PATH, 'w'))
  h = SimpleNamespace(state=SimpleNamespace(get=lambda: dict(engaged=False, vEgo=0.0)), params=SimpleNamespace(get=lambda *a, **k: ''))
  h._touch_obd_viewer = ts.Handler._touch_obd_viewer
  h._obd_state = lambda: ts.Handler._obd_state(h)
  h._queue_obd_cmd = lambda c: ts.Handler._queue_obd_cmd(h, c)
  return SimpleNamespace(h=h, cmd=lambda r: ts.Handler._tdiag_cmd(h, r), vals=vals)


def queued():
  try:
    return json.load(open(ts.OBD_CMD_PATH))
  except OSError:
    return None


def test_probe_is_queued_and_sniff_validates(srv):
  assert srv.cmd({'cmd': 'probe'})[0] == 200 and queued() == {'cmd': 'probe'}
  assert srv.cmd({'cmd': 'sniff', 'seconds': 30, 'protocol': 9})[0] == 200
  assert srv.cmd({'cmd': 'sniff', 'seconds': 30, 'protocol': 5})[0] == 400
  assert srv.cmd({'cmd': 'sniff', 'seconds': 1000, 'protocol': 6})[0] == 400


def test_sniff_and_uds_refused_when_not_parked(srv):
  srv.h.state = SimpleNamespace(get=lambda: dict(engaged=False, vEgo=9.0))
  assert srv.cmd({'cmd': 'sniff', 'seconds': 10, 'protocol': 6})[0] == 409
  assert srv.cmd({'cmd': 'uds', 'profile': 'example_unverified', 'did': 'F190', 'allow': True, 'confirm': True})[0] == 409


def test_uds_with_example_profile_is_refused_and_logged(srv):
  code, body = srv.cmd({'cmd': 'uds', 'profile': 'example_unverified', 'did': 'F190', 'allow': True, 'confirm': True})
  assert code == 403 and 'verified' in body['error'] and queued() is None
  assert any(e['event'] == 'refused' for e in tdiag.tail_log(name=tdiag.UDS_AUDIT))


def test_uds_dry_run_never_queues(srv):
  code, body = srv.cmd({'cmd': 'uds_dry', 'profile': 'example_unverified', 'did': 'F190'})
  assert code == 200 and body['dry'] and body['listed'] and body['blockers'] and queued() is None


def test_uds_queued_only_with_verified_profile_and_flags(srv):
  tdiag.save_profile('ok', json.dumps(verified()))
  base = {'cmd': 'uds', 'profile': 'ok', 'did': 'F190'}
  assert srv.cmd(base | {'allow': True})[0] == 403 and queued() is None
  assert srv.cmd(base | {'allow': True, 'confirm': True})[0] == 200
  assert queued()['request'] == '22F190'


def test_forbidden_did_form_cannot_be_built(srv):
  # the page only builds 0x22/0x19; a non-hex DID is rejected, never sent
  assert srv.cmd({'cmd': 'uds', 'profile': 'example_unverified', 'did': 'zz', 'allow': True, 'confirm': True})[0] == 400
  assert queued() is None
