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
SNIFF_DWELL = 1.5      # seconds per ID bucket before sniff moves to the next one
INIT = ['ATZ', 'ATE0', 'ATL0', 'ATS0', 'ATH1', 'ATAL', 'ATSP6', 'ATCAF0']
OPTIONAL = ['ATCSM1']   # CAN silent monitoring; ELM327 v1.4+, '?' on older clones is fine
NOISE = ('SEARCHING', 'STOPPED', 'OK', 'BUFFER FULL', 'CAN ERROR', 'NO DATA', '?', 'DATA ERROR',
         'BUS BUSY', 'BUS ERROR', 'FB ERROR', '<RX ERROR', 'ELM327')


def parse_frame(line: str, id_chars: int = 3) -> tuple[int, bytes] | None:
  """One monitor line with headers on -> (id, data). Accepts spaced or unspaced output.

  id_chars is 3 for 11-bit and 8 for 29-bit; the id length comes from the protocol that was
  selected, never from the text.
  """
  s = line.strip().upper().replace(' ', '')
  if len(s) < id_chars + 2 or not HEX_RE.match(s) or (len(s) - id_chars) % 2:
    return None
  data = bytes.fromhex(s[id_chars:])
  if not 1 <= len(data) <= 8:
    return None
  return int(s[:id_chars], 16), data


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

    Runs whether or not the adapter confirmed silent monitoring (ATCSM1); the caller records which.
    Without it the dongle ACKs frames it hears -- but a CAN controller ACKs at the bit level for
    every valid frame regardless of the receive filter, so monitor() (the battery page's per-ID
    ATCRA+ATMA, which never required CSM) already ACKs everything on the bus the whole time it is
    connected. Refusing only this full capture added no protection. Every other node on the bus
    ACKs each frame too, so one more dominant ACK bit is ordinary bus behaviour. frames are
    (seconds since start, 11-bit id, data). Only AT commands and a bare CR (to stop ATMA) are sent.

    Unfiltered ATMA on this car's bus overflowed a v1.5 clone in 0.16 s, and an ELM327 stops
    monitoring on BUFFER FULL -- a 30 s capture came back with 21 frames, all from the first 0.16 s.
    So 11-bit captures rotate the receive filter through eight buckets of the ID space (ATCRA 0XX ..
    7XX) to cut the rate the adapter has to keep up with, and an overflow restarts ATMA instead of
    ending the capture. Each bucket hears 1/8 of the time, so a 0.1 Hz message needs ~80 s or more.
    """
    self.cmd(f'ATSP{protocol}')
    id_chars = 8 if protocol in (7, 9) else 3
    buckets: list[str | None] = [None]
    if id_chars == 3 and not any('?' in ln for ln in self.cmd('ATCRA0XX')):
      buckets = [f'{h:X}XX' for h in range(8)]
    elif any('?' in ln for ln in self.cmd('ATCRA')):      # no argument clears the receive filter
      self.cmd('ATCM000')
      self.cmd('ATCF000')
    frames: list = []
    buffer_full, unparsed = False, 0
    t0 = time.monotonic()
    end = t0 + seconds
    dwell = min(SNIFF_DWELL, seconds / len(buckets))
    i = 0
    while time.monotonic() < end and len(frames) < max_frames:
      bucket = buckets[i % len(buckets)]
      i += 1
      if bucket is not None:
        self.cmd(f'ATCRA{bucket}')
      dwell_end = min(end, time.monotonic() + dwell)
      while len(frames) < max_frames:
        full, bad = self._listen_all(dwell_end, id_chars, t0, frames, max_frames)
        buffer_full |= full
        unparsed += bad
        if not full or time.monotonic() >= dwell_end:
          break
    self._buf = b''
    self.cmd('ATSP6')
    return frames, buffer_full, unparsed

  def _listen_all(self, until: float, id_chars: int, t0: float, frames: list, max_frames: int) -> tuple[bool, int]:
    """One ATMA run into `frames` until `until`, max_frames or an overflow. Returns (overflowed, unparsed)."""
    self._buf = b''
    self.t.send(b'ATMA\r')
    unparsed = 0
    last_was_frame = False
    while time.monotonic() < until and len(frames) < max_frames:
      chunk = self.t.recv(512, max(0.01, until - time.monotonic()))
      if not chunk:
        continue
      self._buf += chunk
      *lines, rest = self._buf.replace(b'\r', b'\n').split(b'\n')
      self._buf = rest
      for ln in lines:
        text = ln.decode(errors='replace').strip()
        if not text:
          continue
        f = parse_frame(text, id_chars)
        if f:
          frames.append((time.monotonic() - t0, f[0], f[1]))
          last_was_frame = True
          continue
        if 'BUFFER FULL' in text.upper():
          # the line just before the overflow is the one the adapter cut short ("102 17" in a real
          # capture: one byte of an 8-byte frame)
          if last_was_frame:
            frames.pop()
          # the adapter has already stopped and is back at its prompt; a bare CR there would re-run
          # the last command rather than stop anything, so just collect the prompt
          try:
            self._read_until_prompt(1.0)
          except TimeoutError:
            pass
          return True, unparsed
        if not any(n in text.upper() for n in NOISE):
          unparsed += 1
        last_was_frame = False
    self.t.send(b'\r')
    try:
      self._read_until_prompt(1.5)
    except TimeoutError:
      pass
    return False, unparsed

  # ---- Tesla diag page additions ---------------------------------------------------------------
  def probe(self) -> dict:
    """AT layer only: what this firmware accepts. Nothing is sent to the car."""
    lines = self.cmd('ATZ', 4.0)
    out = {'version': next((ln for ln in lines if 'ELM' in ln.upper()), lines[-1] if lines else ''), 'commands': {}}
    for c in ('ATE0', 'ATL0', 'ATS0', 'ATH1'):
      self.cmd(c)
    for c, key in (('ATI', 'ati'), ('AT@1', 'description'), ('ATRV', 'voltage')):
      out[key] = ' '.join(ln for ln in self.cmd(c) if ln != '?')
    for c in ('ATCAF0', 'ATCAF1', 'ATCFC0', 'ATCFC1', 'ATSH7E0', 'ATCRA7E8', 'ATCRA', 'ATCSM1', 'ATD'):
      lines = self.cmd(c)
      out['commands'][c] = 'unsupported' if '?' in lines else ('ok' if 'OK' in [ln.upper() for ln in lines] else ' '.join(lines))
    out['silent'] = out['commands']['ATCSM1'] == 'ok'
    self.silent = out['silent']
    self.restore()
    return out

  def restore(self) -> None:
    """Back to the monitor settings the battery page polls with."""
    for c in INIT[1:]:
      self.cmd(c)
    if self.silent:
      self.cmd('ATCSM1')

  def uds_request(self, payload: bytes, tx_id: int, rx_id: int, id_format: str, protocol: int, p2: float = 2.0) -> bytes:
    """One read-only request; the dongle frames it (ATCAF1) so no PCI is added here."""
    from openpilot.system.btobd import tdiag
    tdiag.check_request_allowed(payload)
    self.cmd(f'ATSP{protocol}')
    for c in ('ATCAF1', 'ATCFC1', 'ATH0', 'ATS1'):
      self.cmd(c)
    self.cmd(f'ATSH{tx_id:03X}' if id_format == '11' else f'ATSH{tx_id:08X}')
    self.cmd(f'ATCRA{rx_id:03X}' if id_format == '11' else f'ATCRA{rx_id:08X}')
    try:
      lines = self.cmd(payload.hex().upper(), p2 + 2.0)
      return parse_reply(lines)
    finally:
      try:
        self.restore()
      except (TimeoutError, OSError):
        pass


  def uds_discover(self, payload: bytes, tx_id: int, protocol: int, rx_pattern: str) -> list[tuple[int, bytes]]:
    """One read-only request with a wildcard receive filter (ATCRA 6XX) and headers on, to learn
    which ID a module answers on when only its request ID is known. Returns every (id, frame) heard
    in the response window -- PCI byte included, broadcasts on matching IDs too; the caller picks
    the answer out by content. 11-bit only. Keep the request single-frame-answered (3E00, short
    DIDs): with the filter open, the dongle cannot run flow control for a multi-frame reply."""
    from openpilot.system.btobd import tdiag
    tdiag.check_request_allowed(payload)
    self.cmd(f'ATSP{protocol}')
    for c in ('ATCAF1', 'ATCFC1', 'ATH1', 'ATS1'):
      self.cmd(c)
    self.cmd(f'ATSH{tx_id:03X}')
    self.cmd(f'ATCRA{rx_pattern}')
    try:
      lines = self.cmd(payload.hex().upper(), 4.0)
    finally:
      try:
        self.restore()
      except (TimeoutError, OSError):
        pass
    return [f for f in (parse_frame(ln) for ln in lines) if f]


MULTI_HEAD = re.compile(r'^[0-9A-F]{3}$')
MULTI_LINE = re.compile(r'^[0-9A-F]:\s*(.*)$')
BAD_REPLIES = ('NO DATA', 'UNABLE TO CONNECT', 'BUS BUSY', 'CAN ERROR', 'BUFFER FULL', 'BUS ERROR', 'FB ERROR', 'DATA ERROR', 'STOPPED')


def parse_reply(lines: list[str]) -> bytes:
  """Reply of a CAF1 dongle: 'XX XX ..' for one frame, or '014' + '0: ..' '1: ..' for several."""
  lines = [ln for ln in lines if not ln.upper().startswith(('SEARCHING', 'BUS INIT'))]
  for ln in lines:
    u = ln.upper()
    if u == '?':
      raise RuntimeError('동글이 ?로 답했습니다')
    for bad in BAD_REPLIES:
      if u.startswith(bad):
        raise TimeoutError(bad) if bad == 'NO DATA' else RuntimeError(bad)
  if not lines:
    raise TimeoutError('빈 응답')
  if MULTI_HEAD.match(lines[0].replace(' ', '')) and len(lines) > 1 and MULTI_LINE.match(lines[1]):
    total, body = int(lines[0], 16), b''
    for ln in lines[1:]:
      m = MULTI_LINE.match(ln)
      if m:
        body += bytes.fromhex(m.group(1).replace(' ', ''))
    if len(body) < total:
      raise RuntimeError(f'멀티프레임 응답이 짧습니다 ({len(body)}/{total})')
    return body[:total]
  try:
    return bytes.fromhex(lines[-1].replace(' ', ''))
  except ValueError:
    raise RuntimeError(f'해석할 수 없는 응답 {lines!r}') from None
