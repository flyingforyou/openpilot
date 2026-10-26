"""FakeSerial: a scripted ELM327 clone. Implements the transport interface (write/read/close)."""
from __future__ import annotations

from tesla_diag.transport.elm327 import DongleDisconnected


class FakeSerial:
  def __init__(self, stream_lines=(), silent_ok=True, drop_prompt_for=(), disconnect_after_writes=None,
               overflow=False, reply_for_request=None, unsupported=()):
    self.stream_lines = list(stream_lines)
    self.silent_ok = silent_ok
    self.drop_prompt_for = set(drop_prompt_for)
    self.disconnect_after = disconnect_after_writes
    self.overflow = overflow
    self.reply_for_request = reply_for_request or {}
    self.unsupported = set(unsupported)
    self.out = b''
    self.writes: list[str] = []
    self.monitoring = False
    self.closed = False

  def write(self, data: bytes) -> None:
    if self.disconnect_after is not None and len(self.writes) >= self.disconnect_after:
      raise DongleDisconnected('link lost')
    cmd = data.decode().strip()
    self.writes.append(cmd)
    if self.monitoring:                       # any byte stops ATMA
      self.monitoring = False
      self.out += b'STOPPED\r\r>'
      return
    up = cmd.upper()
    if up in self.drop_prompt_for:
      return
    if up in self.unsupported:
      self.out += b'?\r\r>'
    elif up == 'ATZ':
      self.out += b'\r\rELM327 v1.5\r\r>'
    elif up == 'ATI':
      self.out += b'ELM327 v1.5\r\r>'
    elif up == 'AT@1':
      self.out += b'OBDII to RS232 Interpreter\r\r>'
    elif up == 'ATRV':
      self.out += b'12.4V\r\r>'
    elif up == 'ATDP':
      self.out += b'ISO 15765-4 (CAN 11/500)\r\r>'
    elif up == 'ATCSM1':
      self.out += (b'OK' if self.silent_ok else b'?') + b'\r\r>'
    elif up == 'ATMA':
      self.monitoring = True
      self.out += b'\r'.join(s.encode() for s in self.stream_lines) + b'\r'
      if self.overflow:
        self.out += b'BUFFER FULL\r'
    elif up.startswith('AT'):
      self.out += b'OK\r\r>'
    else:                                     # a UDS request
      reply = self.reply_for_request.get(up, 'NO DATA')
      self.out += reply.replace('\n', '\r').encode() + b'\r\r>'

  def read(self, n: int, timeout: float) -> bytes:
    chunk, self.out = self.out[:n], self.out[n:]
    return chunk

  def close(self) -> None:
    self.closed = True

  @property
  def requests(self) -> list[str]:
    """Writes that are not AT commands and not the CR that stops the monitor."""
    return [w for w in self.writes if w and not w.upper().startswith('AT')]
