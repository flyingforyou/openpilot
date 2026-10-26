"""Bluetooth on comma four without bluez: bring-up, the kernel mgmt API, and RFCOMM.

Needs the BT-enabled AGNOS kernel (commaai/agnos-kernel-sdm845 PR #106) and root. The rootfs is
read-only, so there is no bluetoothd to answer PIN requests or keep link keys; the mgmt control
channel does both here, the same interface bluetoothd itself sits on.

Bring-up follows commaai/agnos-builder PR #542's bt_init.sh except for the attach protocol: its
hciattach "qualcomm" init times out on this WCN3990, while plain H4 at 115200 with RTS/CTS works --
the chip answers HCI_Reset there and runs from ROM.
"""
from __future__ import annotations

import ctypes
import errno
import fcntl
import glob
import json
import os
import select
import socket
import struct
import termios
import threading
import time

BTPOWER = '/dev/btpower'
BT_UART = '/dev/ttyHS1'
BTPOWER_IOCTL = 0xBFAD
TIOCSETD = 0x5423
N_HCI = 15
HCIUARTSETPROTO = 0x400455C8   # _IOW('U', 200, int)
HCIUARTSETFLAGS = 0x400455CB   # _IOW('U', 203, int)
HCI_UART_H4 = 0

AF_BLUETOOTH = 31
BTPROTO_HCI = 1
HCI_DEV_NONE = 0xFFFF
HCI_CHANNEL_CONTROL = 3

# mgmt opcodes / events (Documentation: bluez doc/mgmt-api.txt)
OP_SET_POWERED, OP_SET_CONNECTABLE, OP_SET_BONDABLE, OP_SET_SSP = 0x0005, 0x0007, 0x0009, 0x000B
OP_LOAD_LINK_KEYS, OP_PIN_REPLY, OP_SET_IO_CAP = 0x0012, 0x0016, 0x0018
OP_USER_CONFIRM_REPLY, OP_START_DISCOVERY = 0x001C, 0x0023
EV_CMD_COMPLETE, EV_CMD_STATUS, EV_NEW_LINK_KEY = 0x0001, 0x0002, 0x0009
EV_CONNECTED, EV_DISCONNECTED, EV_PIN_REQUEST, EV_CONFIRM_REQUEST = 0x000B, 0x000C, 0x000E, 0x000F
EV_AUTH_FAILED, EV_DEVICE_FOUND, EV_DISCOVERING = 0x0011, 0x0012, 0x0013
IO_NO_INPUT_NO_OUTPUT = 3
BREDR = 0


def available() -> bool:
  return os.path.exists(BTPOWER) and os.path.exists(BT_UART)


def addr_to_bytes(addr: str) -> bytes:
  return bytes(int(x, 16) for x in reversed(addr.split(':')))


def bytes_to_addr(b: bytes) -> str:
  return ':'.join(f'{x:02X}' for x in reversed(b[:6]))


def parse_eir(eir: bytes) -> dict:
  """Name and class of device out of an EIR blob."""
  out, i = {}, 0
  while i < len(eir):
    n = eir[i]
    if n == 0 or i + 1 + n > len(eir):
      break
    typ, data = eir[i + 1], eir[i + 2:i + 1 + n]
    if typ in (0x08, 0x09):
      out['name'] = data.decode(errors='replace')
    elif typ == 0x0D and len(data) == 3:
      out['class'] = int.from_bytes(data, 'little')
    i += 1 + n
  return out


HCI_RESET = bytes([0x01, 0x03, 0x0C, 0x00])
HCI_RESET_DONE = bytes([0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00])


def _reset_raw(tty: int, attempts: int = 5) -> None:
  """HCI_Reset over the raw UART until it is acknowledged, before the kernel owns the line.

  The btpower off/on does not always reset the chip: after a reboot it came back still running an
  LE scan from the previous session, streaming advertising reports, and the kernel's own init
  commands timed out in that flood (hci0 0x1003/0x1001/0x1009 tx timeout). A reset stops whatever
  the chip was doing; draining until it is acknowledged leaves the line in sync.
  """
  for _ in range(attempts):
    os.write(tty, HCI_RESET)
    buf, end = b'', time.monotonic() + 0.6
    while time.monotonic() < end:
      r, _, _ = select.select([tty], [], [], 0.05)
      if r:
        buf += os.read(tty, 4096)
    if HCI_RESET_DONE in buf:
      time.sleep(0.1)
      termios.tcflush(tty, termios.TCIOFLUSH)
      return
  raise OSError('Bluetooth chip did not acknowledge HCI_Reset')


def power_up() -> int:
  """Power the chip, attach the UART as an HCI line discipline. Returns the tty fd, which must stay
  open for as long as the adapter should exist."""
  fd = os.open(BTPOWER, os.O_RDWR)
  try:
    try:
      fcntl.ioctl(fd, BTPOWER_IOCTL, 0)
    except OSError:
      pass
    time.sleep(2)
    fcntl.ioctl(fd, BTPOWER_IOCTL, 1)
  finally:
    os.close(fd)
  time.sleep(3)
  for r in glob.glob('/sys/class/rfkill/rfkill*'):
    try:
      if open(f'{r}/type').read().strip() == 'bluetooth':
        open(f'{r}/soft', 'w').write('0')
    except OSError:
      pass
  time.sleep(1)
  tty = os.open(BT_UART, os.O_RDWR | os.O_NOCTTY)
  a = termios.tcgetattr(tty)
  a[0] = a[1] = a[3] = 0
  a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.CRTSCTS
  a[4] = a[5] = termios.B115200
  termios.tcsetattr(tty, termios.TCSANOW, a)
  termios.tcflush(tty, termios.TCIOFLUSH)
  _reset_raw(tty)
  fcntl.ioctl(tty, TIOCSETD, struct.pack('i', N_HCI))
  fcntl.ioctl(tty, HCIUARTSETFLAGS, 0)
  fcntl.ioctl(tty, HCIUARTSETPROTO, HCI_UART_H4)
  return tty


class Mgmt:
  """The kernel's Bluetooth management channel, with a reader thread that answers pairing."""

  def __init__(self, pin_provider, keys_path: str):
    self.pin_provider = pin_provider
    self.keys_path = keys_path
    self.devices: dict[str, dict] = {}
    self.discovering = False
    self.connected: set[str] = set()
    self.last_auth_error = ''
    self._replies: dict[int, tuple] = {}
    self._cv = threading.Condition()
    self.sock = socket.socket(AF_BLUETOOTH, socket.SOCK_RAW | socket.SOCK_CLOEXEC, BTPROTO_HCI)
    addr = struct.pack('<HHH', AF_BLUETOOTH, HCI_DEV_NONE, HCI_CHANNEL_CONTROL)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.bind(self.sock.fileno(), ctypes.c_char_p(addr), len(addr)) != 0:
      e = ctypes.get_errno()
      raise OSError(e, f'mgmt bind: {os.strerror(e)}')
    self._run = True
    threading.Thread(target=self._reader, daemon=True).start()

  def close(self):
    self._run = False
    self.sock.close()

  def command(self, op: int, params: bytes = b'', index: int = 0, timeout: float = 5.0) -> tuple[int, bytes]:
    with self._cv:
      self._replies.pop(op, None)
      self.sock.send(struct.pack('<HHH', op, index, len(params)) + params)
      if not self._cv.wait_for(lambda: op in self._replies, timeout):
        raise TimeoutError(f'mgmt op 0x{op:04x}')
      return self._replies.pop(op)

  def setup(self) -> None:
    # hci0 registers and runs its own init a moment after the line discipline is attached; until
    # then Set Powered answers Invalid Index (0x11).
    for _ in range(20):
      st, _ = self.command(OP_SET_POWERED, bytes([1]), timeout=10.0)
      if st == 0:
        break
      time.sleep(0.5)
    else:
      raise OSError(f'mgmt set powered status {st}')
    for op, val in ((OP_SET_CONNECTABLE, 1), (OP_SET_BONDABLE, 1), (OP_SET_SSP, 1)):
      st, _ = self.command(op, bytes([val]))
      if st not in (0, 0x0C):   # 0x0C: not supported / already set
        raise OSError(f'mgmt 0x{op:04x} status {st}')
    self.command(OP_SET_IO_CAP, bytes([IO_NO_INPUT_NO_OUTPUT]))
    keys = self._load_keys()
    if keys:
      blob = b''.join(addr_to_bytes(k['addr']) + bytes([BREDR, k['type']]) + bytes.fromhex(k['val']) +
                      bytes([k['pin_len']]) for k in keys)
      self.command(OP_LOAD_LINK_KEYS, struct.pack('<BH', 0, len(keys)) + blob)

  def start_discovery(self) -> int:
    self.devices = {a: d for a, d in self.devices.items() if d.get('paired')}
    st, _ = self.command(OP_START_DISCOVERY, bytes([1]))   # BR/EDR only
    return st

  def _load_keys(self) -> list:
    try:
      return json.load(open(self.keys_path))
    except (OSError, ValueError):
      return []

  def _store_key(self, addr: str, ktype: int, val: bytes, pin_len: int) -> None:
    keys = [k for k in self._load_keys() if k['addr'] != addr]
    keys.append({'addr': addr, 'type': ktype, 'val': val.hex(), 'pin_len': pin_len})
    tmp = self.keys_path + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(keys, f)
    os.replace(tmp, self.keys_path)

  def known(self, addr: str) -> bool:
    return any(k['addr'] == addr for k in self._load_keys())

  def _reader(self) -> None:
    while self._run:
      try:
        r, _, _ = select.select([self.sock], [], [], 0.5)
        if not r:
          continue
        pkt = self.sock.recv(1024)
      except OSError:
        return
      if len(pkt) < 6:
        continue
      ev, _idx, ln = struct.unpack('<HHH', pkt[:6])
      p = pkt[6:6 + ln]
      if ev in (EV_CMD_COMPLETE, EV_CMD_STATUS) and len(p) >= 3:
        op, st = struct.unpack('<HB', p[:3])
        with self._cv:
          self._replies[op] = (st, p[3:])
          self._cv.notify_all()
      elif ev == EV_PIN_REQUEST and len(p) >= 7:
        pin = (self.pin_provider() or '1234').encode()[:16]
        self.sock.send(struct.pack('<HHH', OP_PIN_REPLY, 0, 24) + p[:7] + bytes([len(pin)]) + pin.ljust(16, b'\0'))
      elif ev == EV_CONFIRM_REQUEST and len(p) >= 7:
        self.sock.send(struct.pack('<HHH', OP_USER_CONFIRM_REPLY, 0, 7) + p[:7])
      elif ev == EV_NEW_LINK_KEY and len(p) >= 26:
        addr = bytes_to_addr(p[1:7])
        self._store_key(addr, p[8], p[9:25], p[25])
        self.devices.setdefault(addr, {})['paired'] = True
      elif ev == EV_DEVICE_FOUND and len(p) >= 14:
        addr = bytes_to_addr(p[0:6])
        rssi = struct.unpack('<b', p[7:8])[0]
        eir_len = struct.unpack('<H', p[12:14])[0]
        d = self.devices.setdefault(addr, {})
        d.update(parse_eir(p[14:14 + eir_len]))
        d['rssi'] = rssi
        d['seen'] = time.time()
      elif ev == EV_DISCOVERING and len(p) >= 2:
        self.discovering = bool(p[1])
      elif ev == EV_CONNECTED and len(p) >= 6:
        self.connected.add(bytes_to_addr(p[0:6]))
      elif ev == EV_DISCONNECTED and len(p) >= 6:
        self.connected.discard(bytes_to_addr(p[0:6]))
      elif ev == EV_AUTH_FAILED and len(p) >= 8:
        self.last_auth_error = f'{bytes_to_addr(p[0:6])} status {p[7]}'


class RfcommTransport:
  """An RFCOMM stream as the send/recv(n, timeout) pair Elm327 expects."""

  def __init__(self, addr: str, channels=(1, 2, 3), timeout: float = 15.0):
    err = None
    for ch in channels:
      s = socket.socket(AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
      s.settimeout(timeout)
      try:
        s.connect((addr, ch))
        self.sock, self.channel = s, ch
        return
      except OSError as e:
        err = e
        s.close()
        if e.errno in (errno.EHOSTDOWN, errno.EHOSTUNREACH, errno.ETIMEDOUT):
          break   # the device is not there at all; other channels will not help
    raise err or OSError('rfcomm connect failed')

  def send(self, b: bytes) -> None:
    self.sock.sendall(b)

  def recv(self, n: int, timeout: float) -> bytes:
    self.sock.settimeout(max(0.01, timeout))
    try:
      data = self.sock.recv(n)
    except socket.timeout:
      return b''
    if not data:
      raise ConnectionError('rfcomm closed')
    return data

  def close(self) -> None:
    self.sock.close()
