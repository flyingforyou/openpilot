"""ELM327 over a byte transport (pyserial on /dev/rfcomm0, or a fake in tests).

Two state machines, deliberately separate:
  COMMAND  line-oriented; every command is answered up to a '>' prompt.
  MONITOR  after ATMA the dongle streams frames with no prompt until any byte is sent; the stream
           ends with 'STOPPED' and a prompt. AT commands are refused in this state.

The only non-AT thing this class can ever send is a UDS request, through `uds_request`, which
re-checks the service against a hard allowlist no matter who calls it. Everything else is `AT...`
or the single CR that stops the monitor.
"""
from __future__ import annotations

import enum
import re
import time
from dataclasses import dataclass, field

from tesla_diag.can.frames import CanFrame
from tesla_diag.can.parser import Parsed, parse_line, protocol_id_format

PROMPT = b'>'
READ_SIDS = frozenset({0x22, 0x19})
# Anything here must never reach the wire, from any caller. Kept explicit so a test can pin it.
FORBIDDEN_SIDS = frozenset({0x10, 0x11, 0x14, 0x27, 0x2E, 0x2F, 0x31, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3D, 0x85})
PROTOCOLS = {
  '6': ('ISO 15765-4 CAN', '11', 500), '7': ('ISO 15765-4 CAN', '29', 500),
  '8': ('ISO 15765-4 CAN', '11', 250), '9': ('ISO 15765-4 CAN', '29', 250),
}


class DongleError(Exception):
  pass


class DongleDisconnected(DongleError):
  pass


class DongleTimeout(DongleError):
  pass


class DongleRejected(DongleError):
  """The dongle answered '?' -- the command is unknown to this firmware."""


class StateError(DongleError):
  pass


class Mode(enum.Enum):
  COMMAND = 'command'
  MONITOR = 'monitor'


class SerialTransport:
  def __init__(self, port: str, baud: int = 115200):
    import serial
    self._serial = serial
    try:
      self.s = serial.Serial(port, baud, timeout=0.05, write_timeout=2)
    except serial.SerialException as e:
      raise DongleError(f'cannot open {port}: {e}') from None

  def write(self, data: bytes) -> None:
    try:
      self.s.write(data)
      self.s.flush()
    except (self._serial.SerialException, OSError) as e:
      raise DongleDisconnected(str(e)) from None

  def read(self, n: int, timeout: float) -> bytes:
    self.s.timeout = max(0.0, timeout)
    try:
      return self.s.read(n)
    except (self._serial.SerialException, OSError) as e:
      raise DongleDisconnected(str(e)) from None

  def close(self) -> None:
    try:
      self.s.close()
    except Exception:
      pass


@dataclass
class Capabilities:
  version: str = ''
  device: str = ''
  voltage: str = ''
  commands: dict = field(default_factory=dict)    # AT command -> 'ok' | 'unsupported' | 'error: ...'

  @property
  def silent_monitoring(self) -> bool:
    return self.commands.get('ATCSM1') == 'ok'


class Elm327:
  def __init__(self, transport, cmd_timeout: float = 2.0):
    self.t = transport
    self.cmd_timeout = cmd_timeout
    self.mode = Mode.COMMAND
    self.buf = b''
    self.silent_confirmed = False
    self.protocol: str | None = None
    self.sent_requests = 0          # UDS requests actually written
    self.log: list[str] = []        # every AT command / request, for the audit trail

  # ---- command mode -------------------------------------------------------------------------
  def _read_prompt(self, timeout: float) -> list[str]:
    end = time.monotonic() + timeout
    while PROMPT not in self.buf:
      left = end - time.monotonic()
      if left <= 0:
        raise DongleTimeout('no prompt from dongle (is it powered, paired and on the right port?)')
      chunk = self.t.read(256, min(left, 0.25))
      if chunk:
        self.buf += chunk
    head, _, self.buf = self.buf.partition(PROMPT)
    text = head.decode(errors='replace').replace('\r', '\n')
    return [ln.strip() for ln in text.split('\n') if ln.strip()]

  def at(self, cmd: str, timeout: float | None = None, allow_error: bool = False) -> list[str]:
    c = cmd.strip().upper()
    if not c.startswith('AT'):
      raise StateError(f'only AT commands go through at(): {cmd!r}')
    if self.mode is not Mode.COMMAND:
      raise StateError('dongle is in monitor mode; stop_monitor() first')
    self.buf = b''
    self.log.append(c)
    self.t.write((cmd.strip() + '\r').encode())
    lines = self._read_prompt(timeout or self.cmd_timeout)
    if lines and lines[0].upper() == c:          # echo still on
      lines = lines[1:]
    if not allow_error and any(ln == '?' for ln in lines):
      raise DongleRejected(f'{c}: dongle answered ?')
    return lines

  def try_at(self, cmd: str) -> str:
    """'ok' | 'unsupported' | 'error: <why>' without raising for the first two."""
    try:
      lines = self.at(cmd, allow_error=True)
    except DongleTimeout:
      return 'error: timeout'
    if '?' in lines:
      return 'unsupported'
    return 'ok' if any(ln.upper() == 'OK' for ln in lines) or cmd.upper().startswith(('ATRV', 'ATI', 'AT@1', 'ATDP')) else f'error: {lines}'

  def reset(self) -> Capabilities:
    lines = self.at('ATZ', 5.0, allow_error=True)
    caps = Capabilities(version=next((ln for ln in lines if 'ELM' in ln.upper()), lines[-1] if lines else ''))
    self.silent_confirmed = False
    return caps

  def setup_display(self) -> None:
    for c in ('ATE0', 'ATL0', 'ATS1', 'ATH1'):
      self.at(c)

  def probe(self) -> Capabilities:
    """AT layer only. Never selects a bus action, never sends an OBD/UDS request."""
    caps = self.reset()
    self.setup_display()
    for c, key in (('ATI', 'device'), ('AT@1', 'device_desc'), ('ATRV', 'voltage')):
      lines = self.at(c, allow_error=True)
      val = ' '.join(ln for ln in lines if ln != '?')
      if key == 'device' and not caps.version:
        caps.version = val
      elif key == 'voltage':
        caps.voltage = val
      elif key == 'device_desc':
        caps.device = val
    for c in ('ATCAF0', 'ATCAF1', 'ATCFC0', 'ATCFC1', 'ATSH7E0', 'ATCRA7E8', 'ATCRA', 'ATCSM1', 'ATD'):
      caps.commands[c] = self.try_at(c)
    caps.commands['ATDP'] = self.try_at('ATDP')
    return caps

  def select_protocol(self, protocol: str) -> None:
    if protocol not in PROTOCOLS:
      raise ValueError(f'protocol {protocol!r}: only 6/7/8/9 (plain CAN) are exposed')
    self.at(f'ATSP{protocol}')
    self.protocol = protocol

  # ---- monitor mode -------------------------------------------------------------------------
  def start_monitor(self, protocol: str, require_silent: bool = True) -> None:
    """ATCSM1 first; if the dongle will not confirm silent monitoring, nothing starts."""
    if self.mode is Mode.MONITOR:
      raise StateError('already monitoring')
    st = self.try_at('ATCSM1')
    self.silent_confirmed = st == 'ok'
    if require_silent and not self.silent_confirmed:
      raise DongleError(f'ATCSM1 -> {st}: this dongle did not confirm CAN silent monitoring, so listening is refused '
                        '(a non-silent monitor would ACK frames on the car bus)')
    self.select_protocol(protocol)
    self.at('ATCAF0')                  # raw frames: we want the PCI bytes too
    self.at('ATCRA')                   # no argument: clear the receive filter
    self.buf = b''
    self.log.append('ATMA')
    self.t.write(b'ATMA\r')
    self.mode = Mode.MONITOR

  def read_monitor(self, seconds: float, id_format: str, max_frames: int = 1_000_000):
    """Yield (Parsed) for each line for up to `seconds`. Garbage lines are yielded, not hidden."""
    if self.mode is not Mode.MONITOR:
      raise StateError('not monitoring')
    t0 = time.monotonic()
    end = t0 + seconds
    n = 0
    while time.monotonic() < end and n < max_frames:
      chunk = self.t.read(512, min(0.2, max(0.0, end - time.monotonic())))
      if not chunk:
        continue
      self.buf += chunk
      if len(self.buf) > 4096 and b'\r' not in self.buf:
        self.buf = b''                         # no line ends in sight: framing lost, resync
        yield Parsed(garbage=True)
        continue
      *lines, self.buf = self.buf.replace(b'\n', b'\r').split(b'\r')
      for ln in lines:
        text = ln.decode(errors='replace')
        if not text.strip():
          continue
        p = parse_line(text, id_format, time.monotonic() - t0)
        if p.frame:
          n += 1
        yield p

  def stop_monitor(self) -> list[str]:
    """Any byte stops ATMA. A bare CR is used. The dongle then prints STOPPED and a prompt."""
    if self.mode is not Mode.MONITOR:
      return []
    self.t.write(b'\r')
    try:
      lines = self._read_prompt(2.0)
    except DongleTimeout:
      lines = []
    self.buf = b''
    self.mode = Mode.COMMAND
    return lines

  def monitor(self, protocol: str, seconds: float, require_silent: bool = True):
    """start -> yield frames -> stop, always stopping."""
    fmt = protocol_id_format(protocol)
    self.start_monitor(protocol, require_silent)
    try:
      yield from self.read_monitor(seconds, fmt)
    finally:
      try:
        self.stop_monitor()
      except DongleError:
        self.mode = Mode.COMMAND

  # ---- UDS (read-only) ----------------------------------------------------------------------
  def uds_request(self, payload: bytes, tx_id: int, rx_id: int, id_format: str, protocol: str,
                  p2: float = 2.0) -> bytes:
    """One request, one answer, with the dongle doing the ISO-TP framing (ATCAF1/ATCFC1)."""
    if not payload or payload[0] not in READ_SIDS or payload[0] in FORBIDDEN_SIDS:
      raise StateError(f'service 0x{payload[0] if payload else 0:02X} is not on the read allowlist')
    if self.mode is not Mode.COMMAND:
      raise StateError('stop monitoring before a UDS request')
    self.select_protocol(protocol)
    self.at('ATCAF1')
    self.at('ATCFC1')
    self.at('ATH0')                                  # data only; no ids in the reply
    self.at('ATS1')
    self.at(f'ATSH{tx_id:03X}' if id_format == '11' else f'ATSH{tx_id:08X}')
    self.at(f'ATCRA{rx_id:03X}' if id_format == '11' else f'ATCRA{rx_id:08X}')
    self.buf = b''
    req = payload.hex().upper()
    self.log.append(f'REQ {req}')
    self.sent_requests += 1
    self.t.write((req + '\r').encode())
    lines = self._read_prompt(p2 + 2.0)
    try:
      return parse_elm_reply(lines)
    finally:
      self.at('ATH1', allow_error=True)
      self.at('ATCRA', allow_error=True)

  def close(self) -> None:
    try:
      self.stop_monitor()
    except DongleError:
      pass
    self.t.close()


_MULTI_HEAD = re.compile(r'^[0-9A-F]{3}$')
_MULTI_LINE = re.compile(r'^[0-9A-F]:\s*(.*)$')


def parse_elm_reply(lines: list[str]) -> bytes:
  """Reply of a CAF1 dongle: single frame 'XX XX ..' or '014' + '0: ..' '1: ..' for multi-frame."""
  for ln in lines:
    u = ln.upper()
    for bad, exc in (('NO DATA', DongleTimeout), ('UNABLE TO CONNECT', DongleError), ('BUS BUSY', DongleError),
                     ('CAN ERROR', DongleError), ('BUFFER FULL', DongleError), ('BUS ERROR', DongleError),
                     ('FB ERROR', DongleError), ('DATA ERROR', DongleError), ('STOPPED', DongleError)):
      if u.startswith(bad):
        raise exc(bad)
    if u == '?':
      raise DongleRejected('dongle answered ?')
  lines = [ln for ln in lines if not ln.upper().startswith('SEARCHING') and not ln.upper().startswith('BUS INIT')]
  if not lines:
    raise DongleTimeout('empty reply')
  if _MULTI_HEAD.match(lines[0].replace(' ', '')) and len(lines) > 1 and _MULTI_LINE.match(lines[1]):
    total = int(lines[0], 16)
    body = b''
    for ln in lines[1:]:
      m = _MULTI_LINE.match(ln)
      if m:
        body += bytes.fromhex(m.group(1).replace(' ', ''))
    if len(body) < total:
      raise DongleError(f'multi-frame reply short: {len(body)} of {total} bytes')
    return body[:total]
  # a pending (7F xx 78) is handled by the dongle itself; if one is still printed, return it as is
  last = lines[-1].replace(' ', '')
  try:
    return bytes.fromhex(last)
  except ValueError:
    raise DongleError(f'unparseable reply {lines!r}') from None
