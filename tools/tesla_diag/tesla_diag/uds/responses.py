"""UDS (ISO 14229-1) response parsing."""
from __future__ import annotations

from dataclasses import dataclass

NRC = {
  0x10: 'generalReject', 0x11: 'serviceNotSupported', 0x12: 'subFunctionNotSupported',
  0x13: 'incorrectMessageLengthOrInvalidFormat', 0x14: 'responseTooLong', 0x21: 'busyRepeatRequest',
  0x22: 'conditionsNotCorrect', 0x24: 'requestSequenceError', 0x25: 'noResponseFromSubnetComponent',
  0x26: 'failurePreventsExecutionOfRequestedAction', 0x31: 'requestOutOfRange',
  0x33: 'securityAccessDenied', 0x35: 'invalidKey', 0x36: 'exceededNumberOfAttempts',
  0x37: 'requiredTimeDelayNotExpired', 0x70: 'uploadDownloadNotAccepted', 0x71: 'transferDataSuspended',
  0x72: 'generalProgrammingFailure', 0x73: 'wrongBlockSequenceCounter',
  0x78: 'requestCorrectlyReceived-ResponsePending', 0x7E: 'subFunctionNotSupportedInActiveSession',
  0x7F: 'serviceNotSupportedInActiveSession',
}


class UdsError(Exception):
  pass


@dataclass(frozen=True)
class Response:
  kind: str                 # 'positive' | 'negative' | 'pending'
  sid: int                  # service the response answers
  payload: bytes = b''      # positive: bytes after the response SID
  nrc: int | None = None

  @property
  def nrc_name(self) -> str:
    return NRC.get(self.nrc, f'unknown(0x{self.nrc:02X})') if self.nrc is not None else ''

  @property
  def is_pending(self) -> bool:
    return self.kind == 'pending'


def parse_response(pdu: bytes, request_sid: int | None = None) -> Response:
  if not pdu:
    raise UdsError('empty response')
  if pdu[0] == 0x7F:
    if len(pdu) != 3:
      raise UdsError(f'negative response must be 3 bytes, got {len(pdu)}')
    sid, nrc = pdu[1], pdu[2]
    if request_sid is not None and sid != request_sid:
      raise UdsError(f'negative response for service {sid:02X}, asked {request_sid:02X}')
    return Response('pending' if nrc == 0x78 else 'negative', sid, nrc=nrc)
  sid = pdu[0] - 0x40
  if sid < 0 or sid > 0xBF:
    raise UdsError(f'0x{pdu[0]:02X} is not a response SID')
  if request_sid is not None and sid != request_sid:
    raise UdsError(f'response SID 0x{pdu[0]:02X} does not answer request 0x{request_sid:02X}')
  return Response('positive', sid, payload=bytes(pdu[1:]))


def parse_read_did(resp: Response, did: int) -> bytes:
  """0x62 response: echoed DID then data."""
  if resp.kind != 'positive' or resp.sid != 0x22:
    raise UdsError('not a positive ReadDataByIdentifier response')
  if len(resp.payload) < 2 or int.from_bytes(resp.payload[:2], 'big') != did:
    raise UdsError(f'response does not echo DID {did:04X}')
  return resp.payload[2:]


def parse_dtcs(resp: Response) -> list[dict]:
  """0x59 02 response: availability mask then 4-byte records (3-byte DTC + status)."""
  if resp.kind != 'positive' or resp.sid != 0x19:
    raise UdsError('not a positive ReadDTCInformation response')
  body = resp.payload
  if len(body) < 2 or body[0] != 0x02:
    raise UdsError('only reportDTCByStatusMask (0x02) responses are parsed')
  recs = body[2:]
  if len(recs) % 4:
    raise UdsError('DTC record list is not a multiple of 4 bytes')
  return [{'dtc': recs[i:i + 3].hex().upper(), 'status': recs[i + 3]} for i in range(0, len(recs), 4)]
