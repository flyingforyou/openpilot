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
    self.sniff: dict | None = None
    self.probe_result: dict | None = None
    self.uds_result: dict | None = None

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
    elif c == 'sniff':
      self.run_sniff(float(cmd.get('seconds', 10)), int(cmd.get('protocol', 6)))
    elif c == 'probe':
      self.run_probe()
    elif c == 'uds':
      self.run_uds(cmd)
    elif c == 'connect':
      self.disconnect()
      self.next_try = 0.0
    elif c == 'disconnect':
      self.disconnect()
      self.link_state = 'idle'

  def run_sniff(self, seconds: float, protocol: int) -> None:
    """Hear everything on the bus for a few seconds and keep it. Listen-only: see Elm327.sniff."""
    from openpilot.system.btobd import capture
    if self.elm is None:
      self.sniff = {'status': 'error', 'error': '동글이 연결돼 있지 않습니다'}
      return
    seconds = min(max(seconds, 3.0), 300.0)
    self.sniff = {'status': 'running', 'seconds': seconds, 'protocol': protocol, 'started': time.time()}
    self._publish()
    try:
      frames, full, unparsed = self.elm.sniff(seconds, protocol, max_frames=60000)
      summary = capture.summarize(frames, seconds, full, unparsed)
      filt = self.elm.sniff_filter
      if filt is None and protocol in (6, 8):
        summary['warnings'].append('이 동글은 ID 구간 필터가 먹지 않아 전체 버스를 한꺼번에 들었습니다. 버퍼가 자주 넘쳐 '
                                   '프레임 대부분이 빠지므로 Hz는 실제보다 훨씬 낮게 나옵니다.')
      name = capture.save(frames, {'protocol': protocol, 'frames': len(frames), 'idFormat': '29' if protocol in (7, 9) else '11', 'seconds': seconds, 'dongle': self.elm.version,
                                   'adapterProtocol': self.elm.protocol, 'silent': self.elm.silent,
                                   'filter': filt, 'saved': time.time()})
      self.sniff = {'status': 'done', 'protocol': protocol, 'file': name, 'finished': time.time(), **summary}
      from openpilot.system.btobd import tdiag
      tdiag.log_event('sniff', protocol=protocol, seconds=seconds, file=name, frames=summary['frames'], filter=filt,
                      unique=summary['unique'], bufferFull=full, unparsed=unparsed, silent=self.elm.silent)
    except Exception as e:
      self.sniff = {'status': 'error', 'error': f'{type(e).__name__}: {e}'}
      if isinstance(e, (OSError, TimeoutError, ConnectionError)):
        self.disconnect()
        self.link_state = 'error'
        self.next_try = time.monotonic() + RETRY_S

  def _link_lost(self, e: Exception) -> None:
    if isinstance(e, (OSError, ConnectionError)):
      self.disconnect()
      self.link_state = 'error'
      self.next_try = time.monotonic() + RETRY_S

  def run_probe(self) -> None:
    """AT-layer check of the dongle (Tesla diag page). Sends nothing to the car."""
    from openpilot.system.btobd import tdiag
    if self.elm is None:
      self.probe_result = {'status': 'error', 'error': '동글이 연결돼 있지 않습니다', 'time': time.time()}
      return
    self.probe_result = {'status': 'running', 'time': time.time()}
    self._publish()
    try:
      res = self.elm.probe()
      self.probe_result = {'status': 'done', 'time': time.time(), **res}
      tdiag.log_event('probe', **res)
    except Exception as e:
      self.probe_result = {'status': 'error', 'error': f'{type(e).__name__}: {e}', 'time': time.time()}
      tdiag.log_event('probe-error', error=str(e))
      self._link_lost(e)

  def run_uds(self, cmd: dict) -> None:
    """One UDS request. The gates run again here even though the page's server already checked them:
    this is the process that actually writes to the dongle. `verify` is a read-only handshake that
    promotes a draft profile on a positive answer; control services are only reached when a profile
    is already verified and lists them (the page keeps their buttons disabled regardless)."""
    from openpilot.system.btobd import tdiag
    name = str(cmd.get('profile', ''))
    verify = bool(cmd.get('verify'))
    prof = tdiag.load_profile(name)
    try:
      if prof is None:
        raise tdiag.SafetyRefusal('프로파일을 찾을 수 없습니다')
      payload = bytes.fromhex(str(cmd.get('request', '')))
      tdiag.preflight(prof, payload, bool(cmd.get('allow')), bool(cmd.get('confirm')), verify=verify)
      if self.elm is None:
        raise tdiag.SafetyRefusal('동글이 연결돼 있지 않습니다')
    except (tdiag.SafetyRefusal, ValueError) as e:
      self.uds_result = {'status': 'refused', 'error': str(e), 'time': time.time()}
      tdiag.log_event('refused', name=tdiag.UDS_AUDIT, request=str(cmd.get('request', '')), why=str(e), verify=verify)
      tdiag.log_event('uds-refused', request=str(cmd.get('request', '')), why=str(e), verify=verify)
      return
    proto = tdiag.protocol_for(prof)
    discover = str(cmd.get('discover') or '')
    if discover:
      self.run_uds_discover(prof, payload, proto, discover)
      return
    self.uds_result = {'status': 'running', 'request': payload.hex().upper(), 'verify': verify, 'time': time.time()}
    self._publish()
    tdiag.log_event('send', name=tdiag.UDS_AUDIT, request=payload.hex().upper(), tx=prof['tx_id'], rx=prof['rx_id'],
                    protocol=proto, profile=name, tier=tdiag.us.tier(payload[0]), verify=verify)
    try:
      pdu = self.elm.uds_request(payload, prof['tx_id'], prof['rx_id'], prof['id_format'], proto)
      res = tdiag.describe_response(pdu, payload)
      promoted = None
      if verify and res['kind'] == 'positive':
        p = tdiag.promote_profile(name, payload, pdu, device=getattr(self.elm, 'version', ''))
        promoted = bool(p)
        tdiag.log_event('verified', name=tdiag.UDS_AUDIT, profile=name, request=payload.hex().upper(), ok=promoted)
      self.uds_result = {'status': 'done', 'request': payload.hex().upper(), 'raw': pdu.hex(' ').upper(),
                         'verify': verify, 'promoted': promoted, 'time': time.time(), **res}
      tdiag.log_event('recv', name=tdiag.UDS_AUDIT, request=payload.hex().upper(), response=pdu.hex().upper(), kind=res['kind'])
      tdiag.log_event('uds', request=payload.hex().upper(), response=pdu.hex().upper(), kind=res['kind'], verify=verify)
    except Exception as e:
      self.uds_result = {'status': 'error', 'request': payload.hex().upper(), 'error': f'{type(e).__name__}: {e}', 'time': time.time()}
      tdiag.log_event('uds-error', request=payload.hex().upper(), error=str(e))
      self._link_lost(e)

  def run_uds_discover(self, prof: dict, payload: bytes, proto: int, rx_pattern: str) -> None:
    """Which ID answers prof's request ID: a read sent with the receive filter opened to rx_pattern.
    An answer is a frame whose payload is this service's positive (SID+0x40) or negative (7F SID)
    response; anything else heard on a matching ID in the window is broadcast traffic."""
    from openpilot.system.btobd import tdiag
    req = payload.hex().upper()
    self.uds_result = {'status': 'running', 'request': req, 'discover': rx_pattern, 'time': time.time()}
    self._publish()
    tdiag.log_event('send', name=tdiag.UDS_AUDIT, request=req, tx=prof['tx_id'], rx=rx_pattern, protocol=proto,
                    profile=prof['name'], tier=tdiag.us.tier(payload[0]), discover=True)
    try:
      frames, lines = self.elm.uds_discover(payload, prof['tx_id'], proto, rx_pattern)
    except Exception as e:
      self.uds_result = {'status': 'error', 'request': req, 'discover': rx_pattern, 'error': f'{type(e).__name__}: {e}',
                         'time': time.time()}
      tdiag.log_event('uds-error', request=req, error=str(e), discover=rx_pattern)
      self._link_lost(e)
      return
    sid = payload[0]
    answers = sorted({f'{fid:03X}' for fid, data in frames
                      if len(data) >= 2 and (data[1] == sid + 0x40 or (data[1] == 0x7F and len(data) >= 3 and data[2] == sid))})
    heard = [{'id': f'{fid:03X}', 'data': data.hex(' ').upper()} for fid, data in frames]
    self.uds_result = {'status': 'done', 'request': req, 'discover': rx_pattern, 'answers': answers, 'heard': heard,
                       'lines': lines[:60], 'kind': 'discover', 'time': time.time()}
    tdiag.log_event('recv', name=tdiag.UDS_AUDIT, request=req, discover=rx_pattern, answers=answers, heard=heard,
                    lines=lines[:60])

  def _publish(self) -> None:
    now = time.monotonic()
    snap = self.car.snapshot(now)
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
              'protocol': self.elm.protocol if self.elm else '', 'channel': getattr(self.link, 'channel', None)},
      'sniff': self.sniff, 'probe': self.probe_result, 'uds': self.uds_result,
      'raw': {f'{k:03X}': v for k, v in sorted(self.raw.items())},
      **snap,
    }
    _write_state(state)
    # btobd.json above is tmpfs and overwritten every publish; this is the persistent copy for
    # offline analysis. Only while a dongle is actually feeding values -- idle publishes would
    # just fill the log with empties.
    if self.elm is not None and snap.get('values'):
      from openpilot.system.btobd import tdiag
      tdiag.log_event('live', name=tdiag.LIVE_LOG, max_size=tdiag.LIVE_MAX, values=snap['values'], cellsV=snap.get('cellsV'),
                      cellsT=snap.get('cellsT'), raw={f'{k:03X}': v['data'] for k, v in self.raw.items()})

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
