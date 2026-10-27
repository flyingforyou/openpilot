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
LIVE_LOG = 'live.jsonl'       # btobdd's decoded values, one line per publish while connected
SESSION_MAX = 2_000_000
LIVE_MAX = 50_000_000         # ~3 KB/line at ~1 Hz: ~5 h per file, ~10 h with the .1 rotation
PROFILE_RE = re.compile(r'[A-Za-z0-9_-]{1,40}')
from openpilot.system.btobd import uds_services as us

# The service tiers live in uds_services. Kept as names here for the rest of the module and tests.
READ_SIDS = us.READ_SIDS                 # page enables these once a profile is verified
CONTROL_SIDS = us.CONTROL_SIDS           # ported, page keeps buttons disabled, same gate as reads
FLASH_SIDS = us.FLASH_SIDS               # offline parse only -- never a live transmit path
LIVE_SIDS = us.LIVE_SIDS
DTC_SUBS = us.DTC_SUBS
PROTOCOLS = {6: ('11', 500), 7: ('29', 500), 8: ('11', 250), 9: ('29', 250)}
NRC = us.NRC


# ---- session log ---------------------------------------------------------------------------------
def log_event(event: str, directory: str | None = None, name: str = SESSION, max_size: int = SESSION_MAX, **kw) -> None:
  """One JSON line per thing the page did or the car answered. Never raises: a full disk must not
  take the dongle loop down."""
  directory = directory or LOG_DIR
  try:
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    if os.path.exists(path) and os.path.getsize(path) > max_size:
      os.replace(path, path + '.1')
    _repair_tail(path)
    with open(path, 'a') as f:
      f.write(json.dumps({'t': round(time.time(), 3), 'event': event, **kw}, ensure_ascii=False) + '\n')
    os.chmod(path, 0o644)
  except OSError:
    pass


def _repair_tail(path: str) -> None:
  """Cut a torn last line so the next append starts on a line of its own. Pulling the device out of
  the car mid-write left live.jsonl ending in 1,990 NUL bytes (2026-10-27); appending after that
  would glue the next record onto them and lose it too."""
  try:
    with open(path, 'rb+') as f:
      size = f.seek(0, os.SEEK_END)
      if size == 0:
        return
      f.seek(max(0, size - 4096))
      tail = f.read()
      if tail.endswith(b'\n'):
        return
      cut = tail.rfind(b'\n')
      if cut < 0 and size > len(tail):
        f.write(b'\n')                   # a torn line longer than 4 KB: leave it, just start fresh
        return
      f.truncate(size - len(tail) + cut + 1)
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
DOWNLOADABLE = (SESSION, UDS_AUDIT, LIVE_LOG, LIVE_LOG + '.1')


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


def build_request(did: str = '', dtc: bool = False, dtc_mask: str = 'FF', service: str = '', raw: str = '',
                  args: dict | None = None) -> bytes:
  """Build a request payload. `service`/`raw`/`args` drive the full catalogue; `did`/`dtc` keep the
  old read shortcuts working. The builders live in uds_services; this only maps the page's inputs."""
  a = args or {}
  try:
    if raw:
      return us.build_from_hex(raw)
    if service:
      return _build_service(service, a)
    if dtc:
      return us.read_dtc(0x02, int(dtc_mask, 16))
    return us.read_did(did)
  except us.BuildError as e:
    raise SafetyRefusal(str(e)) from None


def _build_service(service: str, a: dict) -> bytes:
  def h(k, d=0):
    v = a.get(k)
    if v in (None, ''):
      return d
    try:
      return int(str(v), 16)
    except ValueError:
      raise us.BuildError(f'{k}가 16진수가 아닙니다: {v!r}') from None
  if service == 'read_did':
    return us.read_did(str(a.get('did', '')))
  if service == 'read_dtc':
    return us.read_dtc(h('sub', 0x02), h('mask', 0xFF))
  if service == 'vehicle_info':
    return us.read_vehicle_info(h('info', 0))
  if service == 'tester_present':
    return us.tester_present(bool(a.get('suppress', True)))
  if service == 'session':
    return us.session_control(h('session', 0x03))
  if service == 'ecu_reset':
    return us.ecu_reset(h('kind', 0x01))
  if service == 'clear_dtc':
    return us.clear_dtc(h('group', 0xFFFFFF))
  if service == 'sec_seed':
    return us.security_request_seed(h('level', 0x01))
  if service == 'sec_key':
    return us.security_send_key(h('level', 0x02), us._hexbytes(str(a.get('key', ''))))
  if service == 'write_did':
    return us.write_did(h('did'), us._hexbytes(str(a.get('data', ''))))
  if service == 'io_control':
    return us.io_control(h('did'), h('control'), us._hexbytes(str(a.get('data', ''))))
  if service == 'routine':
    return us.routine_control(h('sub', 0x01), h('routine'), us._hexbytes(str(a.get('data', ''))))
  if service == 'dtc_setting':
    return us.control_dtc_setting(bool(a.get('on', True)))
  raise us.BuildError(f'알 수 없는 서비스: {service}')


def check_request_allowed(payload: bytes, live: bool = True) -> None:
  """Service-level floor. Independent of any profile. `live=False` allows flash SIDs (offline parse)."""
  if not payload:
    raise SafetyRefusal('빈 요청')
  sid = payload[0]
  if sid in FLASH_SIDS:
    if live:
      raise SafetyRefusal(f'서비스 0x{sid:02X}({us.SERVICE_NAMES.get(sid, "?")})는 차로 전송할 수 없습니다 (오프라인 해석 전용)')
    return
  if sid not in LIVE_SIDS:
    raise SafetyRefusal(f'서비스 0x{sid:02X}는 지원 목록에 없습니다')
  if sid == 0x22 and (len(payload) < 3 or len(payload) % 2 == 0):
    raise SafetyRefusal('0x22는 2바이트 DID 하나 이상')
  if sid == 0x19 and (len(payload) < 2 or payload[1] not in DTC_SUBS):
    raise SafetyRefusal('0x19 하위기능이 읽기 목록에 없습니다')
  if sid == 0x27 and len(payload) < 2:
    raise SafetyRefusal('0x27은 레벨이 필요합니다')


def preflight(p: dict, payload: bytes, allow: bool, confirm: bool, verify: bool = False) -> None:
  """Every gate that needs no dongle. Raises SafetyRefusal with the reason.

  Normal mode: the profile must be live-capable (evidence-backed) and must list this exact request.
  verify mode: a READ-ONLY request may run against an as-yet-unverified profile so the user can EARN
  verification -- the profile only needs a usable address/bitrate, and a positive answer promotes it.
  Control and flash requests are never allowed in verify mode."""
  check_request_allowed(payload, live=True)
  if not (allow and confirm):
    raise SafetyRefusal('"실차 전송 허용"과 "주차(P)·안전 확인" 둘 다 체크해야 합니다')
  if verify:
    if payload[0] not in READ_SIDS:
      raise SafetyRefusal('검증은 읽기 서비스로만 합니다 (쓰기/제어는 검증된 프로파일에서만)')
    for why in _addressable(p):
      raise SafetyRefusal('검증하려면 먼저 ' + why)
    return
  b = live_blockers(p)
  if b:
    raise SafetyRefusal('이 프로파일은 실차에 쓸 수 없습니다: ' + ', '.join(b))
  if payload.hex().upper() not in p['supported_service_did']:
    raise SafetyRefusal(f'요청 {payload.hex().upper()}이(가) 프로파일의 supported_service_did에 없습니다')


def _addressable(p: dict) -> list[str]:
  """What a profile needs just to send a read at all (for verification), short of being verified."""
  why = []
  if p['tx_id'] is None or p['rx_id'] is None:
    why.append('tx_id/rx_id를 넣으세요')
  if not any(v == (p['id_format'], p['bitrate']) for v in PROTOCOLS.values()):
    why.append('id_format/bitrate를 6~9번 프로토콜에 맞추세요')
  return why


def protocol_for(p: dict) -> int:
  return next(k for k, v in PROTOCOLS.items() if v == (p['id_format'], p['bitrate']))


def describe_response(pdu: bytes, request: bytes) -> dict:
  """Parse a UDS answer into something the page can show (full catalogue, via uds_services)."""
  return us.parse(pdu, request)


def promote_profile(name: str, request: bytes, response: bytes, device: str = '',
                    user: str | None = None) -> dict | None:
  """Turn a draft user profile into a verified one, backed by a real positive read from THIS car.

  Called after a successful verify-mode read. Only touches user profiles. Adds the request to
  supported_service_did, records the response as evidence, and sets verified_on_vehicle/allow_live.
  Returns the saved profile, or None if it could not (shipped profile, parse fail, negative answer).
  """
  user = user or PROFILE_DIR
  path = os.path.join(user, name + '.json')
  if not PROFILE_RE.fullmatch(name) or not os.path.isfile(path):
    return None
  parsed = us.parse(response, request)
  if parsed['kind'] != 'positive':
    return None
  try:
    with open(path) as f:
      d = json.load(f)
  except (OSError, ValueError):
    return None
  reqs = {str(x).replace(' ', '').upper() for x in d.get('supported_service_did') or []}
  reqs.add(request.hex().upper())
  d['supported_service_did'] = sorted(reqs)
  d['verified_on_vehicle'] = True
  d['allow_live'] = True
  stamp = time.strftime('%Y-%m-%d %H:%M')
  d['hardware_or_firmware_evidence'] = (
    f'{stamp} {device or "comma"}에서 {request.hex().upper()} -> {response.hex().upper()} 긍정응답')
  d['evidence_url'] = d.get('evidence_url') or f'log:{UDS_AUDIT}'
  p = parse_profile(d, name)
  with open(path, 'w') as f:
    json.dump(d, f, ensure_ascii=False, indent=1)
  return p


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
