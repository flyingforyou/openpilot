"""Normalised CAN frame model. Everything downstream (logger, decoder, ISO-TP) works on this."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CanFrame:
  ts: float            # seconds since capture start (monotonic), or file time on replay
  id: int
  data: bytes
  extended: bool = False
  rtr: bool = False
  raw: str = ''        # the untouched text the dongle produced, kept for audit

  @property
  def dlc(self) -> int:
    return len(self.data)

  @property
  def id_hex(self) -> str:
    return f'{self.id:08X}' if self.extended else f'{self.id:03X}'

  def __post_init__(self):
    limit = 0x1FFFFFFF if self.extended else 0x7FF
    if not 0 <= self.id <= limit:
      raise ValueError(f'CAN id {self.id:#x} out of range for {"29" if self.extended else "11"}-bit')
    if len(self.data) > 8:
      raise ValueError('classic CAN carries at most 8 data bytes')
