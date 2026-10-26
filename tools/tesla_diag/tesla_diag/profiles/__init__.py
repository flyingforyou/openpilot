"""Vehicle/bus profiles. A profile is the only thing that can unlock a live UDS read, and only when
it carries evidence and says it was verified on a car."""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml

from tesla_diag.can.decoder import Signal, SignalTable

PROFILE_DIR = os.path.dirname(__file__)


class ProfileError(Exception):
  pass


def _hexint(v) -> int | None:
  if v is None or v == '':
    return None
  return v if isinstance(v, int) else int(str(v), 16)


@dataclass
class Profile:
  name: str = 'unnamed'
  vehicle_family: str = ''
  model_year_range: str = ''
  hardware_or_firmware_evidence: str = ''
  bus: str = ''
  bitrate: int = 0
  id_format: str = '11'
  tx_id: int | None = None
  rx_id: int | None = None
  supported_service_did: list[str] = field(default_factory=list)   # exact request payloads, hex
  evidence_url: str = ''
  verified_on_vehicle: bool = False
  allow_live: bool = False
  signals: list[Signal] = field(default_factory=list)

  def signal_table(self) -> SignalTable:
    return SignalTable(self.signals)

  def live_blockers(self) -> list[str]:
    """Why this profile cannot be used for a live read; empty only if every gate passes."""
    why = []
    if not self.verified_on_vehicle:
      why.append('verified_on_vehicle is false')
    if not self.allow_live:
      why.append('allow_live is false')
    if not self.evidence_url:
      why.append('no evidence_url')
    if not self.hardware_or_firmware_evidence:
      why.append('no hardware_or_firmware_evidence')
    if self.tx_id is None or self.rx_id is None:
      why.append('tx_id/rx_id not set')
    if not self.bitrate:
      why.append('bitrate not set')
    if self.id_format not in ('11', '29'):
      why.append('id_format must be 11 or 29')
    if not self.supported_service_did:
      why.append('supported_service_did is empty')
    return why


def load_profile(path: str) -> Profile:
  if not os.path.exists(path):
    alt = os.path.join(PROFILE_DIR, path)
    path = alt if os.path.exists(alt) else path
  try:
    with open(path) as f:
      d = yaml.safe_load(f) or {}
  except (OSError, yaml.YAMLError) as e:
    raise ProfileError(f'{path}: {e}') from None
  if not isinstance(d, dict):
    raise ProfileError(f'{path}: top level must be a mapping')
  known = set(Profile.__dataclass_fields__)
  unknown = set(d) - known
  if unknown:
    raise ProfileError(f'{path}: unknown keys {sorted(unknown)}')
  try:
    sigs = [Signal.from_dict(n, s) for n, s in (d.get('signals') or {}).items()]
  except (KeyError, ValueError, TypeError) as e:
    raise ProfileError(f'{path}: bad signal ({e!r})') from None
  reqs = [str(x).replace(' ', '').upper() for x in d.get('supported_service_did') or []]
  for r in reqs:
    try:
      bytes.fromhex(r)
    except ValueError:
      raise ProfileError(f'{path}: supported_service_did entry {r!r} is not hex') from None
  return Profile(
    name=d.get('name', os.path.basename(path)), vehicle_family=d.get('vehicle_family', ''),
    model_year_range=str(d.get('model_year_range', '')),
    hardware_or_firmware_evidence=d.get('hardware_or_firmware_evidence', '') or '',
    bus=d.get('bus', '') or '', bitrate=int(d.get('bitrate') or 0), id_format=str(d.get('id_format', '11')),
    tx_id=_hexint(d.get('tx_id')), rx_id=_hexint(d.get('rx_id')), supported_service_did=reqs,
    evidence_url=d.get('evidence_url', '') or '', verified_on_vehicle=bool(d.get('verified_on_vehicle', False)),
    allow_live=bool(d.get('allow_live', False)), signals=sigs)
