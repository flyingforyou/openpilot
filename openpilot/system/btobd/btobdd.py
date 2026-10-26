#!/usr/bin/env python3
"""btobdd: Bluetooth ELM327 dongle on the diagnostic port -> Model S/X battery and drive data.

Started by the manager whenever the BT-enabled kernel is present. Re-execs itself under sudo,
because the HCI line discipline, rfkill and the mgmt channel all need root.

It only holds a link while the car is on or the /diag page is open: a live Bluetooth session keeps
the dongle awake on the 12 V battery, and a parked car is better left alone.

IPC with the tuning page, both in /dev/shm:
  btobd.json      state, rewritten about once a second
  btobd_cmd.json  one pending command from the page: {"cmd": "scan"|"connect"|"disconnect", ...}
  btobd_viewer    touched by the page on every poll; its mtime is how this knows someone is looking
"""
from __future__ import annotations

import json
import os
import sys
import time

STATE_PATH = '/dev/shm/btobd.json'
CMD_PATH = '/dev/shm/btobd_cmd.json'
VIEWER_PATH = '/dev/shm/btobd_viewer'
KEYS_PATH = '/data/bt/linkkeys.json'
PARAMS_DIR = '/data/params/d'
VIEWER_TIMEOUT = 60.0
RETRY_S = 15.0
DWELL = 0.15
CELL_DWELL = 1.2
CELL_EVERY = 6        # cycles between brick-voltage sweeps
RAW_KEEP = 40


def _param(key: str, default: str = '') -> str:
  # Read the file directly: this runs as root, and going through Params would risk writing
  # root-owned files into the params store the rest of openpilot owns.
  try:
    return open(os.path.join(PARAMS_DIR, key)).read().strip() or default
  except OSError:
    return default


def _write_state(state: dict) -> None:
  tmp = STATE_PATH + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(state, f)
  os.chmod(tmp, 0o644)
  os.replace(tmp, STATE_PATH)


def _take_cmd() -> dict | None:
  try:
    with open(CMD_PATH) as f:
      cmd = json.load(f)
  except (OSError, ValueError):
    return None
  try:
    os.remove(CMD_PATH)
  except OSError:
    pass
  return cmd


def _wanted() -> bool:
  if _param('BtObdEnabled', '1') != '1':
    return False
  if os.path.exists(os.path.join(PARAMS_DIR, 'IsOnroad')) and _param('IsOnroad') == '1':
    return True
  try:
    return time.time() - os.path.getmtime(VIEWER_PATH) < VIEWER_TIMEOUT
  except OSError:
    return False


class Daemon:
  def __init__(self):
    from openpilot.system.btobd import hci
    from openpilot.system.btobd.tesla_sx import TeslaSXState, WATCH_IDS
    self.hci = hci
    self.watch = WATCH_IDS
    self.car = TeslaSXState()
    self.tty = None
    self.mgmt = None
    self.elm = None
    self.link = None
    self.bt = 'off'
    self.link_state = 'idle'
    self.error = ''
    self.next_try = 0.0
    self.cycle = 0
    self.raw: dict[int, dict] = {}
    self.target = ''
    self.started = time.time()

  # --- Bluetooth adapter -------------------------------------------------------------------------
  def ensure_bt(self) -> bool:
    if self.mgmt is not None:
      return True
    if time.monotonic() < self.next_try:
      return False
    try:
      self.bt = 'starting'
      self._publish()
      os.makedirs(os.path.dirname(KEYS_PATH), exist_ok=True)
      self.tty = self.hci.power_up()
      time.sleep(1.0)
      self.mgmt = self.hci.Mgmt(lambda: _param('BtObdPin', '1234'), KEYS_PATH)
      self.mgmt.setup()
      self.bt = 'up'
      self.error = ''
      return True
    except Exception as e:
      self.bt = 'error'
      self.error = f'블루투스 시작 실패: {type(e).__name__}: {e}'
      self._teardown_bt()
      self.next_try = time.monotonic() + RETRY_S
      return False

  def _teardown_bt(self):
    if self.mgmt:
      self.mgmt.close()
    self.mgmt = None
    if self.tty is not None:
      try:
        os.close(self.tty)
      except OSError:
        pass
    self.tty = None

  # --- dongle link -------------------------------------------------------------------------------
  def connect(self, addr: str) -> None:
    from openpilot.system.btobd.elm327 import Elm327
    self.link_state = 'connecting'
    self._publish()
    try:
      self.link = self.hci.RfcommTransport(addr)
      self.elm = Elm327(self.link)
      self.elm.init()
      self.link_state = 'connected'
      self.error = ''
    except Exception as e:
      self.disconnect()
      self.link_state = 'error'
      auth = self.mgmt.last_auth_error if self.mgmt else ''
      self.error = f'동글 연결 실패: {type(e).__name__}: {e}' + (f' (인증 실패 {auth})' if auth else '')
      self.next_try = time.monotonic() + RETRY_S

  def disconnect(self) -> None:
    if self.link:
      try:
        self.link.close()
      except OSError:
        pass
    self.link = self.elm = None
    if self.link_state == 'connected':
      self.link_state = 'idle'

  def poll_once(self) -> None:
    ids = [i for i in self.watch if i != 0x6F2]
    if self.cycle % CELL_EVERY == 0:
      ids.append(0x6F2)
    self.cycle += 1
    for msg_id in ids:
      frames = self.elm.monitor(msg_id, CELL_DWELL if msg_id == 0x6F2 else DWELL,
                                max_frames=64 if msg_id == 0x6F2 else 4)
      now = time.monotonic()
      for fid, data in frames:
        self.car.feed(fid, data, now)
        r = self.raw.setdefault(fid, {'n': 0})
        r.update(data=data.hex(' ').upper(), len=len(data), t=time.time())
        r['n'] += 1
      self._handle_cmd()
      if self.elm is None:
        return

  # --- page ------------------------------------------------------------------------------------
  def _handle_cmd(self) -> None:
    cmd = _take_cmd()
    if not cmd:
      return
    c = cmd.get('cmd')
    if c == 'scan' and self.mgmt:
      st = self.mgmt.start_discovery()
      if st:
        self.error = f'검색 시작 실패 (status {st})'
    elif c == 'connect':
      self.disconnect()
      self.next_try = 0.0
    elif c == 'disconnect':
      self.disconnect()
      self.link_state = 'idle'

  def _publish(self) -> None:
    now = time.monotonic()
    devices = []
    if self.mgmt:
      for a, d in sorted(self.mgmt.devices.items(), key=lambda kv: -kv[1].get('rssi', -127)):
        devices.append({'addr': a, 'name': d.get('name', ''), 'rssi': d.get('rssi'),
                        'paired': self.mgmt.known(a)})
    state = {
      'updated': time.time(), 'bt': self.bt, 'link': self.link_state, 'error': self.error,
      'target': self.target, 'wanted': _wanted(),
      'discovering': bool(self.mgmt and self.mgmt.discovering), 'devices': devices,
      'elm': {'version': self.elm.version if self.elm else '', 'silent': bool(self.elm and self.elm.silent),
              'channel': getattr(self.link, 'channel', None)},
      'raw': {f'{k:03X}': v for k, v in sorted(self.raw.items())},
      **self.car.snapshot(now),
    }
    _write_state(state)

  def run(self) -> None:
    last_pub = 0.0
    while True:
      self.target = _param('BtObdAddress')
      wanted = _wanted()
      bt_ok = self.ensure_bt()   # the adapter itself comes up at boot; only the dongle link is gated
      if bt_ok:
        self._handle_cmd()
      if wanted and bt_ok:
        if self.elm is None and self.target and time.monotonic() >= self.next_try:
          self.connect(self.target)
        if self.elm is not None:
          try:
            self.poll_once()
          except Exception as e:
            self.error = f'통신 끊김: {type(e).__name__}: {e}'
            self.disconnect()
            self.link_state = 'error'
            self.next_try = time.monotonic() + RETRY_S
      else:
        if not bt_ok:
          _take_cmd()   # a command for a dead adapter would only fire long after it was meant
        if self.elm is not None:
          self.disconnect()
        time.sleep(0.5)
      if time.monotonic() - last_pub > 0.8 or self.elm is None:
        self._publish()
        last_pub = time.monotonic()
      if self.elm is None:
        time.sleep(0.5)


def main():
  if os.geteuid() != 0:
    env = ['env', f"PYTHONPATH={os.environ.get('PYTHONPATH', '')}"]
    os.execvp('sudo', ['sudo', '-n', *env, sys.executable, '-m', 'openpilot.system.btobd.btobdd'])
  Daemon().run()


if __name__ == '__main__':
  main()
