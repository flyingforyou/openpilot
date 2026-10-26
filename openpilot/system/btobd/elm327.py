"""ELM327 client for CAN monitoring, the way Scan My Tesla drives a dongle on a Model S/X.

Nothing is requested from the car. The adapter is set to ISO 15765-4 CAN 11-bit 500 kbps (ATSP6),
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
