"""Elm327 against a scripted fake dongle: init sequence, per-ID monitoring, and line parsing."""
from openpilot.system.btobd.elm327 import Elm327, parse_frame


class FakeDongle:
  """Answers AT commands like an ELM327 v1.5 clone; ATMA streams the frames of the filtered ID."""

  def __init__(self, frames: dict[int, list[str]], silent_ok=True, wildcard_ok=True, wildcard_ignored=False,
               mask_ok=True):
    self.frames = frames
    self.silent_ok = silent_ok
    self.wildcard_ok = wildcard_ok      # ATCRA with X digits (0XX); older clones answer '?'
    self.wildcard_ignored = wildcard_ignored   # the car's clone: OK to ATCRA0XX, then passes everything
    self.mask_ok = mask_ok              # ATCF/ATCM filter + mask
    self.uds_replies: dict[str, list[str]] = {}   # request hex -> lines the dongle prints
    self.sent = []
    self.out = b''
    self.filter = None
    self.cf = self.cm = 0
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
      arg = cmd[5:]
      if 'X' in arg and not self.wildcard_ok:
        self.out += b'?\r\r>'
        return
      self.filter = None if not arg or ('X' in arg and self.wildcard_ignored) else arg
      self.cf = self.cm = 0
      self.out += b'OK\r\r>'
    elif cmd[:4] in ('ATCF', 'ATCM'):
      if not self.mask_ok:
        self.out += b'?\r\r>'
        return
      setattr(self, cmd[2:4].lower(), int(cmd[4:], 16))
      self.filter = None
      self.out += b'OK\r\r>'
    elif cmd == 'ATDP':
      self.out += b'ISO 15765-4 (CAN 11/500)\r\r>'
    elif cmd == 'ATMA':
      self.monitoring = True
      for fid, lines in self.frames.items():
        if self.matches(fid):
          for f in lines:
            self.out += f.encode() + b'\r'
      if self.overflow:             # a real ELM327 stops monitoring and returns to its prompt
        self.out += b'10217\rBUFFER FULL\r\r>'   # with the line before it cut short
        self.monitoring = False
    elif cmd in self.uds_replies:
      self.out += ''.join(ln + '\r' for ln in self.uds_replies[cmd]).encode() + b'\r>'
    elif cmd == 'ATCSM1':
      self.out += (b'OK' if self.silent_ok else b'?') + b'\r\r>'
    else:
      self.out += b'OK\r\r>'

  def matches(self, fid: int) -> bool:
    if self.filter is None:
      return fid & self.cm == self.cf & self.cm
    return all(p == 'X' or p == c for p, c in zip(self.filter.upper(), f'{fid:03X}', strict=False))

  def recv(self, n, timeout):
    chunk, self.out = self.out[:n], self.out[n:]
    return chunk


def test_parse_frame_spaced_and_unspaced():
  assert parse_frame('3020') is None                       # odd data length
  assert parse_frame('3020D') == (0x302, b'\x0d')
  assert parse_frame('30282DF0D') == (0x302, bytes.fromhex('82DF0D'))
  assert parse_frame('302 82 DF 0D') == (0x302, bytes.fromhex('82DF0D'))
  assert parse_frame('BUFFER FULL') is None
  assert parse_frame('F01743900000000') is None          # seen in a real capture: not an 11-bit id
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


def test_uds_discover_reports_who_answers():
  d = FakeDongle({})
  d.uds_replies['3E00'] = ['6F2 01 02 03 04 05 06 07 08', '65C 02 7E 00']   # a broadcast and the answer
  e = Elm327(d)
  frames, lines = e.uds_discover(bytes.fromhex('3E00'), 0x64C, 6, '6XX')
  assert lines == ['6F2 01 02 03 04 05 06 07 08', '65C 02 7E 00']
  assert (0x65C, bytes.fromhex('027E00')) in frames
  assert 'ATSH64C' in d.sent and 'ATCRA6XX' in d.sent and 'ATH1' in d.sent


def test_init_resets_a_dongle_left_monitoring():
  # the 'A' of ATZ only stops a running ATMA; the reset has to be sent again
  d = FakeDongle({})
  d.monitoring = True
  e = Elm327(d)
  e.init()
  assert e.version == 'ELM327 v1.5'
  assert d.sent.count('ATZ') == 2
