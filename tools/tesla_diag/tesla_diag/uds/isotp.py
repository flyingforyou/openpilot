"""ISO 15765-2 (ISO-TP) framing, offline. Used to parse captures and to simulate what an ELM327
does on its own with ATCAF1. The live path never builds PCI bytes: with ATCAF1 the dongle frames the
request itself, so doing it here as well would double-frame it."""
from __future__ import annotations

from dataclasses import dataclass

SF, FF, CF, FC = 0, 1, 2, 3
FC_CTS, FC_WAIT, FC_OVFLW = 0, 1, 2
MAX_PDU = 4095     # 12-bit length in the first frame


class IsoTpError(ValueError):
  pass


@dataclass(frozen=True)
class Pci:
  kind: int
  length: int = 0        # SF payload length / FF total length
  seq: int = 0           # CF sequence number
  fs: int = 0            # FC flow status
  block_size: int = 0
  st_min: int = 0
  payload: bytes = b''   # data bytes carried by this frame


def parse_pci(data: bytes) -> Pci:
  if not data:
    raise IsoTpError('empty frame')
  kind = data[0] >> 4
  if kind == SF:
    n = data[0] & 0x0F
    if n == 0 or n > 7 or n > len(data) - 1:
      raise IsoTpError(f'bad single-frame length {n} for {len(data)}-byte frame')
    return Pci(SF, length=n, payload=bytes(data[1:1 + n]))
  if kind == FF:
    if len(data) < 8:
      raise IsoTpError('first frame shorter than 8 bytes')
    total = (data[0] & 0x0F) << 8 | data[1]
    if total < 8:
      raise IsoTpError(f'first-frame length {total} fits a single frame')
    return Pci(FF, length=total, payload=bytes(data[2:8]))
  if kind == CF:
    return Pci(CF, seq=data[0] & 0x0F, payload=bytes(data[1:]))
  if kind == FC:
    if len(data) < 3:
      raise IsoTpError('flow control shorter than 3 bytes')
    fs = data[0] & 0x0F
    if fs > 2:
      raise IsoTpError(f'reserved flow status {fs}')
    return Pci(FC, fs=fs, block_size=data[1], st_min=data[2])
  raise IsoTpError(f'unknown PCI type {kind}')


def encode_single(payload: bytes, pad: bool = True) -> bytes:
  if not 1 <= len(payload) <= 7:
    raise IsoTpError('single frame carries 1..7 bytes')
  f = bytes([len(payload)]) + payload
  return f.ljust(8, b'\x00') if pad else f


def encode_flow_control(fs: int = FC_CTS, block_size: int = 0, st_min: int = 0) -> bytes:
  return bytes([0x30 | fs, block_size, st_min]).ljust(8, b'\x00')


def encode_multi(payload: bytes) -> list[bytes]:
  """FF + CFs, for offline simulation only."""
  if len(payload) <= 7:
    return [encode_single(payload)]
  if len(payload) > MAX_PDU:
    raise IsoTpError('payload exceeds 4095 bytes')
  frames = [bytes([0x10 | len(payload) >> 8, len(payload) & 0xFF]) + payload[:6]]
  seq = 1
  for i in range(6, len(payload), 7):
    frames.append((bytes([0x20 | seq]) + payload[i:i + 7]).ljust(8, b'\x00'))
    seq = (seq + 1) & 0x0F
  return frames


class Reassembler:
  """Feed it the frames of ONE direction of ONE ECU pair. `need_flow_control` is true right after a
  first frame: the receiver owes a flow-control frame, which passive monitoring can only observe."""

  def __init__(self):
    self.buf = bytearray()
    self.total = 0
    self.next_seq = 1
    self.need_flow_control = False

  @property
  def active(self) -> bool:
    return self.total > 0

  def reset(self) -> None:
    self.buf.clear()
    self.total = 0
    self.next_seq = 1
    self.need_flow_control = False

  def feed(self, data: bytes) -> bytes | None:
    """Returns the complete PDU when one finishes, else None. Raises IsoTpError on a violation and
    drops the half-built PDU."""
    p = parse_pci(data)
    if p.kind == SF:
      self.reset()
      return p.payload
    if p.kind == FF:
      self.reset()
      self.total = p.length
      self.buf += p.payload
      self.next_seq = 1
      self.need_flow_control = True
      return None
    if p.kind == CF:
      if not self.active:
        raise IsoTpError('consecutive frame with no first frame')
      self.need_flow_control = False
      if p.seq != self.next_seq:
        got, want = p.seq, self.next_seq
        self.reset()
        raise IsoTpError(f'sequence error: got {got}, expected {want}')
      self.next_seq = (self.next_seq + 1) & 0x0F
      self.buf += p.payload[:self.total - len(self.buf)]
      if len(self.buf) >= self.total:
        out = bytes(self.buf)
        self.reset()
        return out
      return None
    return None     # flow control belongs to the other direction; ignored here
