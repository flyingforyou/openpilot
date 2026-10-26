"""ELM327 client for listen-only CAN monitoring on a Model S/X.

Nothing is requested from the car. (Whether Scan My Tesla works the same way is not documented on
its site -- it only says it records "all packets as fast as the car sends them" and filters.) The adapter is set to ISO 15765-4 CAN 11-bit 500 kbps (ATSP6),
silent monitoring where the firmware supports it (ATCSM1: no ACKs from the dongle on the bus), and
then pointed at one ID at a time (ATCRA <id>; ATMA) for a short dwell. Cheap clones overflow if
asked to monitor everything on a busy bus, so filtering per ID is the only reliable way.

The transport only needs send(bytes) and recv(n) with a timeout, so tests drive it with a fake.
"""
from __future__ import annotations

import re
import time

PROMPT = b'>'
HEX_RE = re.compile(r'^[0-9A-F]+$')
INIT = ['ATZ', 'ATE0', 'ATL0', 'ATS0', 'ATH1', 'ATAL', 'ATSP6', 'ATCAF0']
OPTIONAL = ['ATCSM1']   # CAN silent monitoring; ELM327 v1.4+, '?' on older clones is fine
NOISE = ('SEARCHING', 'STOPPED', 'OK', 'BUFFER FULL', 'CAN ERROR', 'NO DATA', '?', 'DATA ERROR',
         'BUS BUSY', 'BUS ERROR', 'FB ERROR', '<RX ERROR', 'ELM327')


def parse_frame(line: str) -> tuple[int, bytes] | None:
  """One monitor line with headers on -> (11-bit id, data). Accepts spaced or unspaced output."""
  s = line.strip().upper().replace(' ', '')
  if len(s) < 5 or not HEX_RE.match(s) or (len(s) - 3) % 2:
    return None
  data = bytes.fromhex(s[3:])
  if not 1 <= len(data) <= 8:
    return None
  return int(s[:3], 16), data


class Elm327:
  def __init__(self, transport, cmd_timeout: float = 2.0):
    self.t = transport
    self.cmd_timeout = cmd_timeout
    self.version = ''
    self.protocol = ''
    self.silent = False
    self._buf = b''

  def _read_until_prompt(self, timeout: float) -> list[str]:
    end = time.monotonic() + timeout
    while PROMPT not in self._buf:
      left = end - time.monotonic()
      if left <= 0:
        raise TimeoutError('ELM327: no prompt')
      chunk = self.t.recv(256, left)
      if chunk:
        self._buf += chunk
    head, _, self._buf = self._buf.partition(PROMPT)
    return [ln.strip() for ln in head.decode(errors='replace').replace('\r', '\n').split('\n') if ln.strip()]

  def cmd(self, c: str, timeout: float | None = None) -> list[str]:
    self._buf = b''
    self.t.send((c + '\r').encode())
    return self._read_until_prompt(timeout or self.cmd_timeout)

  def init(self) -> None:
    lines = self.cmd('ATZ', 4.0)
    self.version = next((ln for ln in lines if 'ELM' in ln.upper()), lines[-1] if lines else '')
    for c in INIT[1:]:
      self.cmd(c)
    for c in OPTIONAL:
      self.silent = 'OK' in [ln.upper() for ln in self.cmd(c)]
    dp = self.cmd('ATDP')
    self.protocol = dp[-1] if dp else ''

  def monitor(self, msg_id: int, dwell: float, max_frames: int = 64) -> list[tuple[int, bytes]]:
    """Frames of one ID heard within `dwell` seconds."""
    self.cmd(f'ATCRA{msg_id:03X}')
    self._buf = b''
    self.t.send(b'ATMA\r')
    frames = []
    end = time.monotonic() + dwell
    while time.monotonic() < end and len(frames) < max_frames:
      chunk = self.t.recv(256, max(0.01, end - time.monotonic()))
      if not chunk:
        continue
      self._buf += chunk
      *lines, rest = self._buf.replace(b'\r', b'\n').split(b'\n')
      self._buf = rest
      for ln in lines:
        f = parse_frame(ln.decode(errors='replace'))
        if f and f[0] == msg_id:
          frames.append(f)
    # Any character stops ATMA; the adapter answers with STOPPED and a prompt.
    self.t.send(b'\r')
    try:
      self._read_until_prompt(1.0)
    except TimeoutError:
      pass
    self._buf = b''
    return frames

  def sniff(self, seconds: float, protocol: int = 6, max_frames: int = 20000):
    """Hear every frame on the bus for `seconds`. Returns (frames, buffer_full, unparsed).

    Refuses unless the adapter confirmed silent monitoring (ATCSM1): without it nothing here can
    promise the dongle is not acknowledging frames on the car's bus. frames are (seconds since
    start, 11-bit id, data). Only AT commands and a bare CR (to stop ATMA) are ever sent.
    """
    if not self.silent:
      raise RuntimeError('이 동글은 무음 감시(ATCSM1)를 지원하지 않아 전체 감시를 시작하지 않습니다')
    self.cmd(f'ATSP{protocol}')
    if any('?' in ln for ln in self.cmd('ATCRA')):      # no argument clears the receive filter
      self.cmd('ATCM000')
      self.cmd('ATCF000')
    self._buf = b''
    self.t.send(b'ATMA\r')
    frames, buffer_full, unparsed = [], False, 0
    t0 = time.monotonic()
    end = t0 + seconds
    while time.monotonic() < end and len(frames) < max_frames:
      chunk = self.t.recv(512, max(0.01, end - time.monotonic()))
      if not chunk:
        continue
      self._buf += chunk
      *lines, rest = self._buf.replace(b'\r', b'\n').split(b'\n')
      self._buf = rest
      for ln in lines:
        text = ln.decode(errors='replace').strip()
        if not text:
          continue
        f = parse_frame(text)
        if f:
          frames.append((time.monotonic() - t0, f[0], f[1]))
        elif 'BUFFER FULL' in text.upper():
          buffer_full = True
        elif not any(n in text.upper() for n in NOISE):
          unparsed += 1
    self.t.send(b'\r')
    try:
      self._read_until_prompt(1.5)
    except TimeoutError:
      pass
    self._buf = b''
    self.cmd('ATSP6')
    return frames, buffer_full, unparsed
