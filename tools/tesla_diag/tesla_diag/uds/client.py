"""Read-only UDS client. Every gate is checked here AND again inside Elm327.uds_request.

A request leaves only if: the service is on the allowlist (0x22, 0x19 with a read subfunction), the
exact request bytes appear in the profile's supported_service_did, the profile passes live_blockers(),
--allow-uds-read and --confirm-parked were both given, and the per-session request budget is not spent.
dry_run never touches a link at all.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

from tesla_diag.profiles import Profile
from tesla_diag.transport.elm327 import FORBIDDEN_SIDS, PROTOCOLS, READ_SIDS
from tesla_diag.uds.responses import Response, UdsError, parse_response

DTC_READ_SUBFUNCTIONS = frozenset({0x01, 0x02, 0x0A})   # count by mask, list by mask, supported list


class SafetyRefusal(Exception):
  pass


def build_read_did(did: int) -> bytes:
  if not 0 <= did <= 0xFFFF:
    raise ValueError('DID out of range')
  return bytes([0x22]) + did.to_bytes(2, 'big')


def build_read_dtc(sub: int = 0x02, mask: int = 0xFF) -> bytes:
  if sub not in DTC_READ_SUBFUNCTIONS:
    raise SafetyRefusal(f'0x19 subfunction 0x{sub:02X} is not on the allowlist')
  return bytes([0x19, sub] + ([mask] if sub in (0x01, 0x02) else []))


def check_request_allowed(payload: bytes) -> None:
  """Service-level allowlist; independent of any profile."""
  if not payload:
    raise SafetyRefusal('empty request')
  sid = payload[0]
  if sid in FORBIDDEN_SIDS:
    raise SafetyRefusal(f'service 0x{sid:02X} is forbidden for live use')
  if sid not in READ_SIDS:
    raise SafetyRefusal(f'service 0x{sid:02X} is not on the read allowlist')
  if sid == 0x22 and (len(payload) < 3 or len(payload) % 2 == 0):
    raise SafetyRefusal('0x22 takes one or more 2-byte DIDs')
  if sid == 0x19 and (len(payload) < 2 or payload[1] not in DTC_READ_SUBFUNCTIONS):
    raise SafetyRefusal('0x19 subfunction not on the allowlist')


@dataclass
class DryRun:
  request: bytes
  tx_id: int | None
  rx_id: int | None
  note: str


class UdsClient:
  def __init__(self, profile: Profile, dongle=None, allow_uds_read: bool = False, confirm_parked: bool = False,
               max_requests: int = 3, p2: float = 2.0, audit_path: str | None = None):
    self.profile, self.dongle = profile, dongle
    self.allow_uds_read, self.confirm_parked = allow_uds_read, confirm_parked
    self.max_requests, self.p2, self.audit_path = max_requests, p2, audit_path
    self.sent = 0

  def _audit(self, event: str, **kw) -> None:
    if self.audit_path:
      os.makedirs(os.path.dirname(self.audit_path) or '.', exist_ok=True)
      with open(self.audit_path, 'a') as f:
        f.write(json.dumps({'t': time.strftime('%Y-%m-%dT%H:%M:%S'), 'event': event, **kw}) + '\n')

  def dry_run(self, payload: bytes) -> DryRun:
    check_request_allowed(payload)
    p = self.profile
    listed = payload.hex().upper() in p.supported_service_did
    note = 'listed in profile' if listed else 'NOT in profile supported_service_did (would be refused live)'
    blockers = p.live_blockers()
    if blockers:
      note += '; live blocked: ' + ', '.join(blockers)
    return DryRun(payload, p.tx_id, p.rx_id, note)

  def preflight(self, payload: bytes) -> None:
    """Every gate that does not need a dongle; the CLI runs it before a port is opened."""
    check_request_allowed(payload)
    p = self.profile
    if not (self.allow_uds_read and self.confirm_parked):
      raise SafetyRefusal('live read needs both --allow-uds-read and --confirm-parked')
    blockers = p.live_blockers()
    if blockers:
      raise SafetyRefusal('profile cannot be used live: ' + ', '.join(blockers))
    if payload.hex().upper() not in p.supported_service_did:
      raise SafetyRefusal(f'request {payload.hex().upper()} is not in the profile supported_service_did')
    if not any(v[1] == p.id_format and v[2] == p.bitrate for v in PROTOCOLS.values()):
      raise SafetyRefusal(f'no plain-CAN dongle protocol for {p.id_format}-bit/{p.bitrate} kbps')
    if self.sent >= self.max_requests:
      raise SafetyRefusal(f'request budget of {self.max_requests} spent')

  def _gate(self, payload: bytes) -> None:
    self.preflight(payload)
    if self.dongle is None:
      raise SafetyRefusal('no dongle attached')

  def read(self, payload: bytes) -> Response:
    try:
      self._gate(payload)
    except SafetyRefusal as e:
      self._audit('refused', req=payload.hex().upper(), why=str(e))
      raise
    p = self.profile
    protocol = next(k for k, v in PROTOCOLS.items() if v[1] == p.id_format and v[2] == p.bitrate)
    self.sent += 1
    self._audit('send', req=payload.hex().upper(), tx=p.tx_id, rx=p.rx_id, protocol=protocol)
    pdu = self.dongle.uds_request(payload, p.tx_id, p.rx_id, p.id_format, protocol, self.p2)
    self._audit('recv', resp=pdu.hex().upper())
    try:
      return parse_response(pdu, payload[0])
    except UdsError:
      self._audit('bad-response', resp=pdu.hex().upper())
      raise
