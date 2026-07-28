"""Decode every CAN frame on the bus against the car's DBC, for the web viewer.

cabana does this far better offline, but it needs the log pulled off the car first. This is for
the moment you are sitting in the car wondering which signal moves when you press something --
it decodes live and flags what changed, so a knob can be found in one pass instead of a drive
plus an analysis session.
"""
import threading
import time
from collections import defaultdict

import cereal.messaging as messaging
from opendbc.can.dbc import DBC
from opendbc.can.parser import CANDefine, get_raw_value


class CanDecoder:
  """Tracks every (bus, address) seen, decoded where the DBC knows the message."""

  # A signal is "recently changed" for this long, so a change is still visible after glancing up
  CHANGE_HOLD_S = 3.0

  def __init__(self, dbc_names: list[str]):
    self.lock = threading.Lock()
    self.msgs: dict[tuple[int, int], dict] = {}
    self.dbc_name = None
    self.dbcs: list[tuple[DBC, dict]] = []

    for name in dbc_names:
      try:
        self.dbcs.append((DBC(name), CANDefine(name).dv))
        self.dbc_name = self.dbc_name or name
      except Exception:
        continue

    self.started = time.monotonic()
    threading.Thread(target=self._run, daemon=True).start()

  def _lookup(self, address: int):
    for dbc, dv in self.dbcs:
      msg = dbc.addr_to_msg.get(address)
      if msg is not None:
        return msg, dv
    return None, None

  def _decode(self, msg, dv, dat: bytes) -> dict[str, dict]:
    out = {}
    for sig in msg.sigs.values():
      try:
        raw = get_raw_value(dat, sig)
        if sig.is_signed:
          raw -= ((raw >> (sig.size - 1)) & 0x1) * (1 << sig.size)
        value = raw * sig.factor + sig.offset
      except Exception:
        continue
      enum = (dv.get(msg.address) or {}).get(sig.name, {}).get(int(value)) if dv else None
      out[sig.name] = {'v': round(value, 4), 'raw': raw, 'enum': enum}
    return out

  def _run(self):
    sm = messaging.SubMaster(['can'])
    counts: dict[tuple[int, int], int] = defaultdict(int)

    while True:
      sm.update(100)
      if not sm.updated['can']:
        continue
      now = time.monotonic()

      for frame in sm['can']:
        key = (frame.src, frame.address)
        counts[key] += 1
        dat = bytes(frame.dat)

        with self.lock:
          entry = self.msgs.get(key)
          if entry is None:
            msg, dv = self._lookup(frame.address)
            entry = {
              'bus': frame.src, 'address': frame.address,
              'name': msg.name if msg else None,
              'len': len(dat), 'hex': dat.hex(), 'signals': {},
              'changed': {}, 'count': 0, 'last': now, 'hz': 0.0,
              '_msg': msg, '_dv': dv, '_first': now,
            }
            self.msgs[key] = entry

          prev_sigs = entry['signals']
          sigs = self._decode(entry['_msg'], entry['_dv'], dat) if entry['_msg'] else {}
          for n, s in sigs.items():
            if n in prev_sigs and prev_sigs[n]['raw'] != s['raw']:
              entry['changed'][n] = now

          entry.update(hex=dat.hex(), signals=sigs, count=counts[key], last=now)
          span = max(now - entry['_first'], 1e-3)
          entry['hz'] = round(counts[key] / span, 1)

  def snapshot(self, changed_only: bool = False) -> dict:
    now = time.monotonic()
    out = []
    with self.lock:
      for entry in self.msgs.values():
        changed = {n for n, t in entry['changed'].items() if now - t < self.CHANGE_HOLD_S}
        if changed_only and not changed:
          continue
        out.append({
          'bus': entry['bus'], 'address': entry['address'], 'name': entry['name'],
          'hex': entry['hex'], 'hz': entry['hz'], 'count': entry['count'],
          'age': round(now - entry['last'], 2),
          'signals': [
            {'name': n, **s, 'changed': n in changed}
            for n, s in sorted(entry['signals'].items())
          ],
          'anyChanged': bool(changed),
        })
    out.sort(key=lambda m: (m['bus'], m['address']))
    return {'messages': out, 'dbc': self.dbc_name, 'total': len(out)}
