"""Signal decoding from a profile's signal table. Nothing is built in: every signal carries its
source and status in the profile, and a signal whose status is not 'verified' is printed with that
label so a guessed layout cannot pass for a confirmed one."""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from tesla_diag.can.frames import CanFrame


@dataclass(frozen=True)
class Signal:
  name: str
  msg_id: int
  start: int            # bit index; for little-endian (Intel) the LSB position, for big-endian the MSB
  length: int
  byte_order: str = 'little'   # 'little' | 'big'
  signed: bool = False
  scale: float = 1.0
  offset: float = 0.0
  unit: str = ''
  min_dlc: int = 8
  source: str = ''
  status: str = 'unverified'   # 'verified' only with a log from the target car

  @classmethod
  def from_dict(cls, name: str, d: dict) -> Signal:
    mid = d['id'] if isinstance(d['id'], int) else int(str(d['id']), 16)
    s = cls(name=name, msg_id=mid, start=int(d['start']), length=int(d['length']),
            byte_order=d.get('byte_order', 'little'), signed=bool(d.get('signed', False)),
            scale=float(d.get('scale', 1.0)), offset=float(d.get('offset', 0.0)), unit=d.get('unit', ''),
            min_dlc=int(d.get('min_dlc', 8)), source=d.get('source', ''), status=d.get('status', 'unverified'))
    if s.byte_order not in ('little', 'big'):
      raise ValueError(f'{name}: byte_order must be little or big')
    if not 1 <= s.length <= 64:
      raise ValueError(f'{name}: length out of range')
    return s


def extract(data: bytes, start: int, length: int, byte_order: str = 'little', signed: bool = False) -> int:
  n = int.from_bytes(data, 'little' if byte_order == 'little' else 'big')
  total = len(data) * 8
  if byte_order == 'little':
    if start + length > total:
      raise ValueError('signal outside frame')
    raw = (n >> start) & ((1 << length) - 1)
  else:
    # DBC "Motorola" numbering: start is the MSB, bits run downwards within a byte and on to the next byte
    byte, bit = divmod(start, 8)
    pos = byte * 8 + (7 - bit)                  # position counting from the MSB of byte 0
    if pos + length > total:
      raise ValueError('signal outside frame')
    raw = (int.from_bytes(data, 'big') >> (total - pos - length)) & ((1 << length) - 1)
  if signed and raw & (1 << (length - 1)):
    raw -= 1 << length
  return raw


def decode(sig: Signal, fr: CanFrame) -> float | None:
  if fr.id != sig.msg_id or fr.rtr or len(fr.data) < sig.min_dlc:
    return None
  try:
    return extract(fr.data, sig.start, sig.length, sig.byte_order, sig.signed) * sig.scale + sig.offset
  except ValueError:
    return None


class SignalTable:
  def __init__(self, signals: list[Signal]):
    self.signals = signals
    self.by_id: dict[int, list[Signal]] = defaultdict(list)
    for s in signals:
      self.by_id[s.msg_id].append(s)

  def decode_frames(self, frames):
    """Latest value, count and source per signal over a capture."""
    out: dict[str, dict] = {}
    for fr in frames:
      for s in self.by_id.get(fr.id, ()):
        v = decode(s, fr)
        if v is None:
          continue
        e = out.setdefault(s.name, {'n': 0, 'min': v, 'max': v, 'unit': s.unit, 'status': s.status, 'source': s.source})
        e['n'] += 1
        e['last'] = v
        e['min'] = min(e['min'], v)
        e['max'] = max(e['max'], v)
    return out
