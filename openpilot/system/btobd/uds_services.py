"""UDS (ISO 14229-1) service catalogue, ported from the public Tesla diagnostic projects named in
docs/RESEARCH (tm3diag, UDS-TESLA, Battery-Emulator, PiCanScanMyTesla) but rewritten here, not
copied. Every service has a request builder and a response parser. What differs is whether the
service may ever reach the car:

  READ      0x22 0x19 0x09 0x3E     -- the page enables these once a profile is verified.
  CONTROL   0x10 0x11 0x14 0x27     -- fully ported (builder + parser + live path), but the page
            0x2E 0x2F 0x31 0x85        leaves their buttons DISABLED. They stay gated exactly like
                                        reads (verified profile + request listed + both confirms).
  FLASH     0x34 0x35 0x36 0x37     -- builders/parsers exist for OFFLINE decoding of a captured
            0x38                        flash session only. There is NO live transmit path, with or
                                        without a profile: Elm327.uds_request refuses these.

Two lines held regardless of profile, matching the project's own spec:
  * No seed->key derivation. 0x27 can request a seed and send a key, but the key bytes come from
    the profile (`security_key`, user-supplied / a provider you are lawfully entitled to use, as in
    tm3diag's SECURITY_PROVIDER). Nothing here computes or guesses a key.
  * No live firmware transfer (the FLASH tier above).
"""
from __future__ import annotations

READ_SIDS = frozenset({0x22, 0x19, 0x09, 0x3E})
CONTROL_SIDS = frozenset({0x10, 0x11, 0x14, 0x27, 0x2E, 0x2F, 0x31, 0x85})
FLASH_SIDS = frozenset({0x34, 0x35, 0x36, 0x37, 0x38})        # offline parse only, never live
LIVE_SIDS = READ_SIDS | CONTROL_SIDS                           # may reach the car, if fully gated
ALL_SIDS = LIVE_SIDS | FLASH_SIDS

# DTC read subfunctions (ReadDTCInformation). The write-ish 0x19 subs do not exist; it is read-only.
DTC_SUBS = frozenset({0x01, 0x02, 0x04, 0x06, 0x0A})

SERVICE_NAMES = {
  0x22: 'ReadDataByIdentifier', 0x19: 'ReadDTCInformation', 0x09: 'ReadVehicleInformation',
  0x3E: 'TesterPresent', 0x10: 'DiagnosticSessionControl', 0x11: 'ECUReset',
  0x14: 'ClearDiagnosticInformation', 0x27: 'SecurityAccess', 0x2E: 'WriteDataByIdentifier',
  0x2F: 'InputOutputControlByIdentifier', 0x31: 'RoutineControl', 0x85: 'ControlDTCSetting',
  0x34: 'RequestDownload', 0x35: 'RequestUpload', 0x36: 'TransferData', 0x37: 'RequestTransferExit',
  0x38: 'RequestFileTransfer',
}

NRC = {
  0x10: 'generalReject', 0x11: 'serviceNotSupported', 0x12: 'subFunctionNotSupported',
  0x13: 'incorrectMessageLengthOrInvalidFormat', 0x14: 'responseTooLong', 0x21: 'busyRepeatRequest',
  0x22: 'conditionsNotCorrect', 0x24: 'requestSequenceError', 0x25: 'noResponseFromSubnetComponent',
  0x26: 'failurePreventsExecutionOfRequestedAction', 0x31: 'requestOutOfRange',
  0x33: 'securityAccessDenied', 0x35: 'invalidKey', 0x36: 'exceededNumberOfAttempts',
  0x37: 'requiredTimeDelayNotExpired', 0x70: 'uploadDownloadNotAccepted', 0x71: 'transferDataSuspended',
  0x72: 'generalProgrammingFailure', 0x73: 'wrongBlockSequenceCounter',
  0x78: 'responsePending', 0x7E: 'subFunctionNotSupportedInActiveSession',
  0x7F: 'serviceNotSupportedInActiveSession',
}


class BuildError(ValueError):
  pass


def tier(sid: int) -> str:
  if sid in READ_SIDS:
    return 'read'
  if sid in CONTROL_SIDS:
    return 'control'
  if sid in FLASH_SIDS:
    return 'flash'
  return 'unknown'


def _hexbytes(s: str) -> bytes:
  s = (s or '').replace(' ', '')
  try:
    return bytes.fromhex(s)
  except ValueError:
    raise BuildError(f'16진수가 아닙니다: {s!r}') from None


# ---- request builders ----------------------------------------------------------------------------
def read_did(dids: str) -> bytes:
  """0x22 with one or more 2-byte DIDs, e.g. 'F190' or 'F190 F18C'."""
  ids = [t for t in (dids or '').replace(',', ' ').split() if t]
  if not ids:
    raise BuildError('DID를 입력하세요')
  out = bytearray([0x22])
  for d in ids:
    try:
      n = int(d, 16)
    except ValueError:
      raise BuildError(f'DID가 16진수가 아닙니다: {d}') from None
    if not 0 <= n <= 0xFFFF:
      raise BuildError(f'DID 범위 오류: {d}')
    out += n.to_bytes(2, 'big')
  return bytes(out)


def read_dtc(sub: int = 0x02, mask: int = 0xFF) -> bytes:
  if sub not in DTC_SUBS:
    raise BuildError(f'0x19 하위기능 0x{sub:02X}는 읽기 전용 목록에 없습니다')
  if sub in (0x02, 0x01):
    return bytes([0x19, sub, mask & 0xFF])
  if sub == 0x06:                                   # reportDTCExtDataRecordByDTCNumber needs a DTC
    raise BuildError('0x19 06은 DTC 번호가 필요합니다; 프로파일 요청으로 넣으세요')
  return bytes([0x19, sub])


def read_vehicle_info(info_type: int) -> bytes:
  return bytes([0x09, info_type & 0xFF])


def tester_present(suppress: bool = True) -> bytes:
  return bytes([0x3E, 0x80 if suppress else 0x00])


def session_control(session: int) -> bytes:
  return bytes([0x10, session & 0xFF])


def ecu_reset(kind: int = 0x01) -> bytes:
  return bytes([0x11, kind & 0xFF])


def clear_dtc(group: int = 0xFFFFFF) -> bytes:
  return bytes([0x14]) + (group & 0xFFFFFF).to_bytes(3, 'big')


def security_request_seed(level: int = 0x01) -> bytes:
  if level % 2 == 0:
    raise BuildError('seed 요청은 홀수 레벨입니다')
  return bytes([0x27, level & 0xFF])


def security_send_key(level: int, key: bytes) -> bytes:
  """Send a key. The key is supplied, never derived here. `level` is the even send-key subfunction."""
  if level % 2 == 1:
    raise BuildError('key 전송은 짝수 레벨입니다')
  if not key:
    raise BuildError('키가 없습니다 (프로파일 security_key 또는 외부 provider 필요; 여기서 키를 만들지 않습니다)')
  return bytes([0x27, level & 0xFF]) + key


def write_did(did: int, data: bytes) -> bytes:
  return bytes([0x2E]) + (did & 0xFFFF).to_bytes(2, 'big') + data


def io_control(did: int, control: int, data: bytes = b'') -> bytes:
  return bytes([0x2F]) + (did & 0xFFFF).to_bytes(2, 'big') + bytes([control & 0xFF]) + data


def routine_control(sub: int, routine: int, data: bytes = b'') -> bytes:
  """sub: 0x01 start, 0x02 stop, 0x03 requestResults."""
  return bytes([0x31, sub & 0xFF]) + (routine & 0xFFFF).to_bytes(2, 'big') + data


def control_dtc_setting(on: bool) -> bytes:
  return bytes([0x85, 0x01 if on else 0x02])


def build_from_hex(raw: str) -> bytes:
  """A profile lists whole request payloads as hex; this is the generic builder for them."""
  p = _hexbytes(raw)
  if not p:
    raise BuildError('빈 요청')
  return p


# ---- response parsing ----------------------------------------------------------------------------
def parse(pdu: bytes, request: bytes) -> dict:
  """Turn a UDS answer into a dict the page can render. Covers every service; flash responses are
  parsed too (for offline capture decoding)."""
  if not pdu:
    return {'kind': 'error', 'text': '빈 응답'}
  sid = request[0] if request else 0
  if pdu[0] == 0x7F:
    if len(pdu) < 3:
      return {'kind': 'error', 'text': f'이상한 부정응답 {pdu.hex(" ").upper()}'}
    nrc = pdu[2]
    kind = 'pending' if nrc == 0x78 else 'negative'
    return {'kind': kind, 'nrc': nrc, 'text': f'NRC 0x{nrc:02X} {NRC.get(nrc, "?")}', 'echoSid': pdu[1]}
  if request and pdu[0] != sid + 0x40:
    return {'kind': 'error', 'text': f'요청(0x{sid:02X})과 맞지 않는 응답 SID 0x{pdu[0]:02X}'}
  body = pdu[1:]
  name = SERVICE_NAMES.get(sid, f'0x{sid:02X}')
  if sid == 0x22:
    return _parse_did(body, request)
  if sid == 0x19:
    return _parse_dtc(body)
  if sid == 0x27 and len(body) >= 1:
    # odd subfunction -> seed follows; even -> key accepted
    kind = 'seed' if body[0] % 2 == 1 else 'keyok'
    seed = body[1:].hex(' ').upper()
    return {'kind': 'positive', 'sub': 'seed' if kind == 'seed' else 'key', 'seed': seed,
            'text': (f'시드 {seed}' if kind == 'seed' else '키 수락됨')}
  if sid in (0x10, 0x11, 0x3E, 0x14, 0x85, 0x2E, 0x2F, 0x31, 0x09):
    return {'kind': 'positive', 'hex': body.hex(' ').upper(),
            'text': f'{name} 긍정응답' + (f' · {body.hex(" ").upper()}' if body else '')}
  if sid in FLASH_SIDS:
    return {'kind': 'positive', 'flash': True, 'hex': body.hex(' ').upper(),
            'text': f'{name} 응답(오프라인 해석) {body.hex(" ").upper()}'}
  return {'kind': 'positive', 'hex': body.hex(' ').upper(), 'text': f'{name} {body.hex(" ").upper()}'}


def _parse_did(body: bytes, request: bytes) -> dict:
  reqd = request[1:] if request else b''
  if reqd and body[:2] != reqd[:2]:
    return {'kind': 'error', 'text': 'DID가 응답에 되풀이되지 않았습니다'}
  did = body[:2]
  data = body[2:]
  out = {'kind': 'positive', 'did': did.hex().upper(), 'hex': data.hex(' ').upper(),
         'text': f'DID {did.hex().upper()} = {data.hex(" ").upper()}'}
  if did == b'\xf1\x90':                            # VIN, masked
    s = data.decode('ascii', 'replace')
    out['vin'] = s[:3] + '*' * max(0, len(s) - 7) + s[-4:] if len(s) > 7 else '*' * len(s)
    out['text'] += f'  ({out["vin"]})'
  return out


def _parse_dtc(body: bytes) -> dict:
  if len(body) < 1:
    return {'kind': 'error', 'text': '빈 DTC 응답'}
  sub = body[0]
  if sub in (0x01,):                                # reportNumberOfDTCByStatusMask
    return {'kind': 'positive', 'text': f'DTC 개수 보고 {body.hex(" ").upper()}'}
  recs = body[2:] if sub == 0x02 else body[1:]
  if sub == 0x02 and len(recs) % 4 == 0:
    dtcs = [{'dtc': recs[i:i + 3].hex().upper(), 'status': recs[i + 3]} for i in range(0, len(recs), 4)]
    return {'kind': 'positive', 'dtcs': dtcs,
            'text': f'DTC {len(dtcs)}개' + (': ' + ', '.join(d['dtc'] for d in dtcs) if dtcs else '')}
  return {'kind': 'positive', 'hex': body.hex(' ').upper(), 'text': f'DTC(0x{sub:02X}) {body.hex(" ").upper()}'}
