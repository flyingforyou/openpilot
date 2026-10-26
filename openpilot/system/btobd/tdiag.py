"""Everything the /tdiag page needs that is not the dongle itself: the session log, capture and
profile files, offline analysis, and the read-only UDS rules.

The UDS rules are the same ones tools/tesla_diag enforces and are checked twice -- by the page's
server before a command is queued and again by Elm327.uds_request before a byte is written. Only
0x22 (ReadDataByIdentifier) and 0x19 (ReadDTCInformation, read subfunctions) can ever be sent.
"""
from __future__ import annotations

import io
import json
import os
import re
import time
import zipfile
from collections import Counter, defaultdict

LOG_DIR = '/data/btobd_logs'
PROFILE_DIR = '/data/btobd_profiles'
SHIPPED_DIR = os.path.join(os.path.dirname(__file__), 'profiles')
SESSION = 'session.jsonl'
UDS_AUDIT = 'uds_audit.jsonl'
SESSION_MAX = 2_000_000
PROFILE_RE = re.compile(r'[A-Za-z0-9_-]{1,40}')
READ_SIDS = frozenset({0x22, 0x19})
FORBIDDEN_SIDS = frozenset({0x10, 0x11, 0x14, 0x27, 0x2E, 0x2F, 0x31, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3D, 0x85})
DTC_SUBS = frozenset({0x01, 0x02, 0x0A})
PROTOCOLS = {6: ('11', 500), 7: ('29', 500), 8: ('11', 250), 9: ('29', 250)}
NRC = {
  0x10: 'generalReject', 0x11: 'serviceNotSupported', 0x12: 'subFunctionNotSupported',
  0x13: 'incorrectMessageLengthOrInvalidFormat', 0x14: 'responseTooLong', 0x21: 'busyRepeatRequest',
  0x22: 'conditionsNotCorrect', 0x24: 'requestSequenceError', 0x31: 'requestOutOfRange',
  0x33: 'securityAccessDenied', 0x78: 'responsePending', 0x7E: 'subFunctionNotSupportedInActiveSession',
  0x7F: 'serviceNotSupportedInActiveSession',
}


# ---- session log ---------------------------------------------------------------------------------
def log_event(event: str, directory: str | None = None, name: str = SESSION, **kw) -> None:
  """One JSON line per thing the page did or the car answered. Never raises: a full disk must not
  take the dongle loop down."""
  directory = directory or LOG_DIR
  try:
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    if os.path.exists(path) and os.path.getsize(path) > SESSION_MAX:
      os.replace(path, path + '.1')
    with open(path, 'a') as f:
      f.write(json.dumps({'t': round(time.time(), 3), 'event': event, **kw}, ensure_ascii=False) + '\n')
    os.chmod(path, 0o644)
  except OSError:
    pass


def tail_log(directory: str | None = None, name: str = SESSION, n: int = 60) -> list[dict]:
  directory = directory or LOG_DIR
  try:
    with open(os.path.join(directory, name)) as f:
      lines = f.readlines()[-n:]
  except OSError:
    return []
  out = []
  for ln in lines:
    try:
      out.append(json.loads(ln))
    except ValueError:
      pass
  return out


# ---- files ---------------------------------------------------------------------------------------
CAP_RE = re.compile(r'capture-\d{8}-\d{6}\.jsonl')
DOWNLOADABLE = (SESSION, UDS_AUDIT)


def list_captures(directory: str | None = None) -> list[dict]:
  directory = directory or LOG_DIR
  out = []
  try:
    names = sorted(n for n in os.listdir(directory) if CAP_RE.fullmatch(n))
  except OSError:
    return out
  for n in names:
    p = os.path.join(directory, n)
    meta = {}
    try:
      with open(p) as f:
        meta = json.loads(f.readline()).get('meta', {})
    except (OSError, ValueError):
      pass
    out.append({'name': n, 'size': os.path.getsize(p), 'mtime': os.path.getmtime(p), 'meta': meta})
  return out[::-1]


def downloadable_path(name: str, directory: str | None = None) -> str | None:
  directory = directory or LOG_DIR
  if not (CAP_RE.fullmatch(name) or name in DOWNLOADABLE):
    return None
  p = os.path.join(directory, name)
  return p if os.path.isfile(p) else None


def delete_capture(name: str, directory: str | None = None) -> bool:
  directory = directory or LOG_DIR
  if not CAP_RE.fullmatch(name):
    return False
  try:
    os.remove(os.path.join(directory, name))
    return True
  except OSError:
    return False


def zip_logs(directory: str | None = None) -> bytes:
  directory = directory or LOG_DIR
  buf = io.BytesIO()
  with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
    try:
      names = sorted(os.listdir(directory))
    except OSError:
      names = []
    for n in names:
      if CAP_RE.fullmatch(n) or n in DOWNLOADABLE or n == SESSION + '.1':
        z.write(os.path.join(directory, n), n)
  return buf.getvalue()


# ---- profiles ------------------------------------------------------------------------------------
def _hexint(v):
  if v in (None, ''):
    return None
  return v if isinstance(v, int) else int(str(v), 16)


class ProfileError(ValueError):
  pass


def parse_profile(d: dict, name: str = '') -> dict:
  """Validate a profile dict; returns it normalised. Raises ProfileError."""
  if not isinstance(d, dict):
    raise ProfileError('프로파일은 JSON 객체여야 합니다')
  try:
    reqs = [str(x).replace(' ', '').upper() for x in d.get('supported_service_did') or []]
    for r in reqs:
      bytes.fromhex(r)
    sigs = {}
    for n, s in (d.get('signals') or {}).items():
      sig = {'id': _hexint(s['id']), 'start': int(s['start']), 'length': int(s['length']),
             'byte_order': s.get('byte_order', 'little'), 'signed': bool(s.get('signed', False)),
             'scale': float(s.get('scale', 1.0)), 'offset': float(s.get('offset', 0.0)), 'unit': s.get('unit', ''),
             'min_dlc': int(s.get('min_dlc', 8)), 'source': s.get('source', ''), 'status': s.get('status', 'unverified')}
      if sig['byte_order'] not in ('little', 'big') or not 1 <= sig['length'] <= 64:
        raise ProfileError(f'신호 {n}: byte_order/length 오류')
      sigs[n] = sig
    out = {
      'name': name or d.get('name', ''), 'note': d.get('note', ''), 'vehicle_family': d.get('vehicle_family', ''),
      'model_year_range': str(d.get('model_year_range', '')),
      'hardware_or_firmware_evidence': d.get('hardware_or_firmware_evidence') or '',
      'bus': d.get('bus') or '', 'bitrate': int(d.get('bitrate') or 0), 'id_format': str(d.get('id_format', '11')),
      'tx_id': _hexint(d.get('tx_id')), 'rx_id': _hexint(d.get('rx_id')), 'supported_service_did': reqs,
      'evidence_url': d.get('evidence_url') or '', 'verified_on_vehicle': bool(d.get('verified_on_vehicle', False)),
      'allow_live': bool(d.get('allow_live', False)), 'signals': sigs}
  except ProfileError:
    raise
  except (KeyError, ValueError, TypeError, AttributeError) as e:
    raise ProfileError(f'프로파일 형식 오류: {e!r}') from None
  return out


def live_blockers(p: dict) -> list[str]:
  why = []
  if not p['verified_on_vehicle']:
    why.append('verified_on_vehicle이 false')
  if not p['allow_live']:
    why.append('allow_live가 false')
  if not p['evidence_url']:
    why.append('evidence_url 없음')
  if not p['hardware_or_firmware_evidence']:
    why.append('hardware_or_firmware_evidence 없음')
  if p['tx_id'] is None or p['rx_id'] is None:
    why.append('tx_id/rx_id 없음')
  if not any(v == (p['id_format'], p['bitrate']) for v in PROTOCOLS.values()):
    why.append('id_format/bitrate가 6~9번 프로토콜 중 어느 것에도 해당하지 않음')
  if not p['supported_service_did']:
    why.append('supported_service_did 비어 있음')
  return why


def list_profiles(shipped: str | None = None, user: str | None = None) -> list[dict]:
  shipped = shipped or SHIPPED_DIR
  user = user or PROFILE_DIR
  out = []
  for src, d in (('내장', shipped), ('사용자', user)):
    try:
      names = sorted(n for n in os.listdir(d) if n.endswith('.json'))
    except OSError:
      continue
    for n in names:
      try:
        with open(os.path.join(d, n)) as f:
          p = parse_profile(json.load(f), n[:-5])
      except (OSError, ValueError):
        out.append({'name': n[:-5], 'source': src, 'error': '읽을 수 없는 파일'})
        continue
      out.append({'name': p['name'], 'source': src, 'note': p['note'], 'blockers': live_blockers(p),
                  'requests': p['supported_service_did'], 'signals': len(p['signals']),
                  'tx_id': p['tx_id'], 'rx_id': p['rx_id'], 'id_format': p['id_format'], 'bitrate': p['bitrate']})
  return out


def load_profile(name: str, shipped: str | None = None, user: str | None = None) -> dict | None:
  shipped = shipped or SHIPPED_DIR
  user = user or PROFILE_DIR
  if not PROFILE_RE.fullmatch(name):
    return None
  for d in (user, shipped):          # a user file of the same name wins
    p = os.path.join(d, name + '.json')
    if os.path.isfile(p):
      try:
        with open(p) as f:
          return parse_profile(json.load(f), name)
      except (OSError, ValueError):
        return None
  return None


def save_profile(name: str, text: str, user: str | None = None) -> dict:
  user = user or PROFILE_DIR
  if not PROFILE_RE.fullmatch(name):
    raise ProfileError('이름은 영문/숫자/_/- 40자 이내')
  if len(text) > 40_000:
    raise ProfileError('프로파일이 너무 큽니다')
  try:
    d = json.loads(text)
  except ValueError as e:
    raise ProfileError(f'JSON이 아닙니다: {e}') from None
  p = parse_profile(d, name)
  os.makedirs(user, exist_ok=True)
  with open(os.path.join(user, name + '.json'), 'w') as f:
    json.dump(d, f, ensure_ascii=False, indent=1)
  return p


# ---- UDS rules -----------------------------------------------------------------------------------
class SafetyRefusal(Exception):
  pass


def build_request(did: str = '', dtc: bool = False, dtc_mask: str = 'FF') -> bytes:
  try:
    if dtc:
      return bytes([0x19, 0x02, int(dtc_mask, 16) & 0xFF])
    n = int(did, 16)
  except ValueError:
    raise SafetyRefusal('DID를 16진수로 입력하세요') from None
  if not 0 <= n <= 0xFFFF:
    raise SafetyRefusal('DID 범위 오류')
  return bytes([0x22]) + n.to_bytes(2, 'big')


def check_request_allowed(payload: bytes) -> None:
  if not payload:
    raise SafetyRefusal('빈 요청')
  sid = payload[0]
  if sid in FORBIDDEN_SIDS:
    raise SafetyRefusal(f'서비스 0x{sid:02X}는 금지입니다')
  if sid not in READ_SIDS:
    raise SafetyRefusal(f'서비스 0x{sid:02X}는 허용 목록에 없습니다')
  if sid == 0x22 and (len(payload) < 3 or len(payload) % 2 == 0):
    raise SafetyRefusal('0x22는 2바이트 DID 하나 이상')
  if sid == 0x19 and (len(payload) < 2 or payload[1] not in DTC_SUBS):
    raise SafetyRefusal('0x19 하위기능이 허용 목록에 없습니다')


def preflight(p: dict, payload: bytes, allow: bool, confirm: bool) -> None:
  """Every gate that needs no dongle. Raises SafetyRefusal with the reason."""
  check_request_allowed(payload)
  if not (allow and confirm):
    raise SafetyRefusal('"실차 읽기 허용"과 "주차 상태 확인" 둘 다 체크해야 합니다')
  b = live_blockers(p)
  if b:
    raise SafetyRefusal('이 프로파일은 실차에 쓸 수 없습니다: ' + ', '.join(b))
  if payload.hex().upper() not in p['supported_service_did']:
    raise SafetyRefusal(f'요청 {payload.hex().upper()}이(가) 프로파일의 supported_service_did에 없습니다')


def protocol_for(p: dict) -> int:
  return next(k for k, v in PROTOCOLS.items() if v == (p['id_format'], p['bitrate']))


def describe_response(pdu: bytes, request: bytes) -> dict:
  """Parse a UDS answer into something the page can show."""
  if not pdu:
    return {'kind': 'error', 'text': '빈 응답'}
  if pdu[0] == 0x7F:
    if len(pdu) != 3 or pdu[1] != request[0]:
      return {'kind': 'error', 'text': f'이상한 부정응답 {pdu.hex(" ").upper()}'}
    kind = 'pending' if pdu[2] == 0x78 else 'negative'
    return {'kind': kind, 'nrc': pdu[2], 'text': f'NRC 0x{pdu[2]:02X} {NRC.get(pdu[2], "?")}'}
  if pdu[0] != request[0] + 0x40:
    return {'kind': 'error', 'text': f'요청과 맞지 않는 응답 SID 0x{pdu[0]:02X}'}
  body = pdu[1:]
  if request[0] == 0x22:
    if body[:2] != request[1:3]:
      return {'kind': 'error', 'text': 'DID가 응답에 되풀이되지 않았습니다'}
    data = body[2:]
    out = {'kind': 'positive', 'hex': data.hex(' ').upper(), 'text': f'DID {request[1:3].hex().upper()} = {data.hex(" ").upper()}'}
    if request[1:3] == b'\xf1\x90':
      s = data.decode('ascii', 'replace')
      out['vin'] = s[:3] + '*' * max(0, len(s) - 7) + s[-4:] if len(s) > 7 else '*' * len(s)
      out['text'] += f'  ({out["vin"]})'     # VIN is masked; the raw hex above is what the car sent
    return out
  recs = body[2:]
  if len(recs) % 4:
    return {'kind': 'error', 'text': 'DTC 목록 길이 오류'}
  dtcs = [{'dtc': recs[i:i + 3].hex().upper(), 'status': recs[i + 3]} for i in range(0, len(recs), 4)]
  return {'kind': 'positive', 'dtcs': dtcs, 'text': f'DTC {len(dtcs)}개' + (': ' + ', '.join(d['dtc'] for d in dtcs) if dtcs else '')}


# ---- offline analysis ----------------------------------------------------------------------------
def extract(data: bytes, start: int, length: int, byte_order: str = 'little', signed: bool = False) -> int:
  total = len(data) * 8
  if byte_order == 'little':
    if start + length > total:
      raise ValueError
    raw = (int.from_bytes(data, 'little') >> start) & ((1 << length) - 1)
  else:
    byte, bit = divmod(start, 8)
    pos = byte * 8 + (7 - bit)
    if pos + length > total:
      raise ValueError
    raw = (int.from_bytes(data, 'big') >> (total - pos - length)) & ((1 << length) - 1)
  if signed and raw & (1 << (length - 1)):
    raw -= 1 << length
  return raw


def read_capture(path: str) -> tuple[dict, list[tuple[float, int, bytes]]]:
  meta, frames = {}, []
  with open(path) as f:
    for line in f:
      d = json.loads(line)
      if 'meta' in d:
        meta = d['meta']
      elif 'id' in d:
        frames.append((float(d['t']), int(d['id'], 16), bytes.fromhex(d['data'])))
  return meta, frames


def analyze(frames, profile: dict | None, decoders: dict | None = None) -> dict:
  """ID table (rate, DLCs, unknown), per-second frame counts, and decoded signals."""
  per: dict[int, list] = defaultdict(list)
  for t, i, d in frames:
    per[i].append((t, d))
  span = (max(f[0] for f in frames) - min(f[0] for f in frames)) if frames else 0.0
  sigs = (profile or {}).get('signals', {})
  by_id: dict[int, list] = defaultdict(list)
  for n, s in sigs.items():
    by_id[s['id']].append((n, s))
  known = set(by_id) | set(decoders or {})
  ids = [{'id': f'{i:03X}' if i <= 0x7FF else f'{i:08X}', 'n': len(fs), 'hz': round(len(fs) / span, 1) if span else None,
          'dlc': sorted({len(d) for _, d in fs}), 'last': fs[-1][1].hex(' ').upper(),
          'known': i in known, 'builtin': (decoders or {}).get(i)} for i, fs in sorted(per.items())]
  t0 = min((f[0] for f in frames), default=0.0)
  buckets = Counter(int(t - t0) for t, _, _ in frames)
  per_second = [buckets.get(s, 0) for s in range(int(span) + 1)] if frames else []
  decoded = {}
  for i, lst in by_id.items():
    for t, d in per.get(i, ()):
      for n, s in lst:
        if len(d) < s['min_dlc']:
          continue
        try:
          v = extract(d, s['start'], s['length'], s['byte_order'], s['signed']) * s['scale'] + s['offset']
        except ValueError:
          continue
        e = decoded.setdefault(n, {'n': 0, 'min': v, 'max': v, 'unit': s['unit'], 'status': s['status'], 'source': s['source']})
        e['n'] += 1
        e['last'] = round(v, 4)
        e['min'], e['max'] = round(min(e['min'], v), 4), round(max(e['max'], v), 4)
  return {'frames': len(frames), 'span': round(span, 2), 'ids': ids, 'perSecond': per_second, 'decoded': decoded,
          'unknown': [r['id'] for r in ids if not r['known']]}
