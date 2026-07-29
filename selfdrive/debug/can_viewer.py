"""Decode every CAN frame on the bus against the car's DBC, for the web viewer.

cabana does this far better offline, but it needs the log pulled off the car first. This is for
the moment you are sitting in the car wondering which signal moves when you press something --
it decodes live and flags what changed, so a knob can be found in one pass instead of a drive
plus an analysis session.
"""
import re
import threading
import time
from collections import defaultdict

import cereal.messaging as messaging
from opendbc.can.dbc import DBC
from opendbc.can.parser import CANDefine, get_raw_value


# Counters and checksums change on every single frame by design. Flagging them as "changed"
# buries the one signal you are actually hunting for. This DBC types none of them (every
# signal is DEFAULT), so match on the naming instead.
NOISE_SIGNAL_RE = re.compile(r"(^CRC_|^MC_|Checksum$|Counter$|_CRC$|_Cnt$)", re.IGNORECASE)


def is_noise_signal(sig) -> bool:
  return getattr(sig, 'type', 0) != 0 or bool(NOISE_SIGNAL_RE.search(sig.name))


class CanDecoder:
  """Tracks every (bus, address) seen, decoded where the DBC knows the message."""

  # A signal is "recently changed" for this long, so a change is still visible after glancing up
  CHANGE_HOLD_S = 3.0

  def __init__(self, dbc_names: list[str], start: bool = True):
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

    self.counts: dict[tuple[int, int], int] = defaultdict(int)
    self.started = time.monotonic()
    if start:
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
      # VAL_ tables key off the raw value, not the scaled one. Same thing for the many signals
      # with factor 1 and no offset, but not for e.g. AirTemp_Outsd, where raw 255 "SNA" scales
      # to 87.5 and the label silently never matched.
      enum = (dv.get(msg.address) or {}).get(sig.name, {}).get(raw) if dv else None
      out[sig.name] = {'v': round(value, 4), 'raw': raw, 'enum': enum,
                       'noise': is_noise_signal(sig)}
    return out

  def ingest(self, frames, now: float | None = None) -> None:
    """Take a batch of CAN frames. Split out from the live loop so it can be driven from a
    recorded log as well as the bus."""
    now = time.monotonic() if now is None else now

    for frame in frames:
      key = (frame.src, frame.address)
      self.counts[key] += 1
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
        for n, sig in sigs.items():
          if sig['noise']:
            continue
          if n in prev_sigs and prev_sigs[n]['raw'] != sig['raw']:
            entry['changed'][n] = now

        entry.update(hex=dat.hex(), signals=sigs, count=self.counts[key], last=now)
        span = now - entry['_first']
        entry['hz'] = round((self.counts[key] - 1) / span, 1) if self.counts[key] > 1 and span > 0 else 0.0

  def _run(self):
    sm = messaging.SubMaster(['can'])
    while True:
      sm.update(100)
      if sm.updated['can']:
        self.ingest(sm['can'])

  def catalog(self) -> list[dict]:
    """Every message the DBC defines, so a signal that never arrives is visibly absent rather
    than just missing from the list."""
    out, seen = [], set()
    for dbc, _ in self.dbcs:
      for address, msg in dbc.addr_to_msg.items():
        if address in seen:
          continue
        seen.add(address)
        out.append({
          'bus': None, 'address': address, 'name': msg.name,
          'hex': None, 'hz': 0.0, 'count': 0, 'age': None, 'seen': False,
          'signals': [{'name': sig.name, 'v': None, 'raw': None, 'enum': None,
                       'noise': is_noise_signal(sig), 'changed': False}
                      for sig in sorted(msg.sigs.values(), key=lambda s: s.name)],
          'anyChanged': False,
        })
    return out

  def snapshot(self, changed_only: bool = False, now: float | None = None,
               include_unseen: bool = True) -> dict:
    now = time.monotonic() if now is None else now
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
          'seen': True,
        })

    if include_unseen and not changed_only:
      live = {m['address'] for m in out}
      out += [m for m in self.catalog() if m['address'] not in live]

    out.sort(key=lambda m: (m['bus'] is None, m['bus'] or 0, m['address']))
    return {
      'messages': out, 'dbc': self.dbc_name, 'total': len(out),
      'seen': sum(1 for m in out if m['seen']),
      'known': sum(1 for m in out if m['name']),
    }
