"""Elm327 against a scripted fake dongle: init sequence, per-ID monitoring, and line parsing."""
from openpilot.system.btobd.elm327 import Elm327, parse_frame


class FakeDongle:
  """Answers AT commands like an ELM327 v1.5 clone; ATMA streams the frames of the filtered ID."""

  def __init__(self, frames: dict[int, list[str]], silent_ok=True):
    self.frames = frames
    self.silent_ok = silent_ok
    self.sent = []
    self.out = b''
    self.filter = None
    self.overflow = False
    self.monitoring = False

  def send(self, data: bytes):
    cmd = data.decode().strip()
    self.sent.append(cmd)
    if self.monitoring:            # any character stops ATMA
      self.monitoring = False
      self.out += b'STOPPED\r\r>'
      return
    if cmd == 'ATZ':
      self.out += b'\r\rELM327 v1.5\r\r>'
    elif cmd.startswith('ATCRA'):
      self.filter = int(cmd[5:], 16) if cmd[5:] else None
      self.out += b'OK\r\r>'
    elif cmd == 'ATDP':
      self.out += b'ISO 15765-4 (CAN 11/500)\r\r>'
    elif cmd == 'ATMA':
      self.monitoring = True
      for fid, lines in self.frames.items():
        if self.filter is None or fid == self.filter:
          for f in lines:
            self.out += f.encode() + b'\r'
      if self.overflow:
        self.out += b'BUFFER FULL\r'
    elif cmd == 'ATCSM1':
      self.out += (b'OK' if self.silent_ok else b'?') + b'\r\r>'
    else:
      self.out += b'OK\r\r>'

  def recv(self, n, timeout):
    chunk, self.out = self.out[:n], self.out[n:]
    return chunk


def test_parse_frame_spaced_and_unspaced():
  assert parse_frame('3020') is None                       # odd data length
  assert parse_frame('3020D') == (0x302, b'\x0d')
  assert parse_frame('30282DF0D') == (0x302, bytes.fromhex('82DF0D'))
  assert parse_frame('302 82 DF 0D') == (0x302, bytes.fromhex('82DF0D'))
  assert parse_frame('BUFFER FULL') is None
  assert parse_frame('STOPPED') is None


def test_init_sends_monitor_setup():
  d = FakeDongle({})
  e = Elm327(d)
  e.init()
  assert e.version == 'ELM327 v1.5'
  assert e.silent
  for c in ('ATE0', 'ATH1', 'ATSP6', 'ATS0'):
    assert c in d.sent


def test_old_clone_without_silent_monitoring():
  e = Elm327(FakeDongle({}, silent_ok=False))
  e.init()
  assert not e.silent


def test_monitor_filters_one_id():
  d = FakeDongle({0x382: ['382FC9AEA69A900A000', '382FC9AEA69A900A000'], 0x102: ['1026697']})
  e = Elm327(d)
  e.init()
  frames = e.monitor(0x382, dwell=0.05)
  assert frames == [(0x382, bytes.fromhex('FC9AEA69A900A000'))] * 2
  assert 'ATCRA382' in d.sent
  # the adapter was stopped and the next command still gets a clean prompt
  assert e.monitor(0x102, dwell=0.05) == [(0x102, bytes.fromhex('6697'))]
