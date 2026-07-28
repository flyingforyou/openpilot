"""Where the CAN viewer gets its frames: the live bus, or a route recorded on this device.

Without a car connected there is no CAN and no CarParams, so the viewer has nothing to show and
no DBC to show it with -- which makes it impossible to work on the page unless you are sitting
in the car. Replaying a stored route fixes both: the log carries its own CarParams, so the
right DBC is picked from the recording rather than from whatever is plugged in.
"""
import os
import threading
import time

import capnp
import zstandard

from cereal import log as capnp_log, car, messaging
from openpilot.common.params import Params
from openpilot.selfdrive.debug.can_viewer import CanDecoder

REALDATA = '/data/media/0/realdata'
MAX_SLEEP = 0.2   # don't stall on gaps between segments


def list_routes(base: str = REALDATA) -> list[dict]:
  """Routes on this device, newest first, as {name, segments}."""
  if not os.path.isdir(base):
    return []

  counts: dict[str, int] = {}
  mtimes: dict[str, float] = {}
  for entry in os.listdir(base):
    seg = os.path.join(base, entry, 'rlog.zst')
    if '--' not in entry or not os.path.isfile(seg):
      continue
    name = entry.rsplit('--', 1)[0]
    counts[name] = counts.get(name, 0) + 1
    mtimes[name] = max(mtimes.get(name, 0), os.path.getmtime(seg))

  return [{'name': n, 'segments': counts[n]}
          for n in sorted(counts, key=lambda n: -mtimes[n])]


def _segments(route: str, base: str = REALDATA) -> list[str]:
  paths = []
  for entry in os.listdir(base):
    if not entry.startswith(route + '--'):
      continue
    seg = os.path.join(base, entry, 'rlog.zst')
    if os.path.isfile(seg):
      paths.append((int(entry.rsplit('--', 1)[1]), seg))
  return [p for _, p in sorted(paths)]


def _read_events(path: str):
  with open(path, 'rb') as f:
    data = zstandard.ZstdDecompressor().stream_reader(f).read()
  try:
    yield from capnp_log.Event.read_multiple_bytes(data)
  except capnp.KjException:
    pass  # segments truncated by power loss end mid-message


def dbcs_for_fingerprint(fingerprint: str) -> list[str]:
  from opendbc.car.values import PLATFORMS
  platform = PLATFORMS.get(fingerprint)
  if platform is None:
    return []
  return sorted({v for v in (platform.config.dbc_dict or {}).values() if v})


class CanSource:
  """Serves a CanDecoder fed either from the live bus or from a recorded route."""

  def __init__(self, params: Params):
    self.params = params
    self.lock = threading.Lock()
    self.live: CanDecoder | None = None
    self.replay: CanDecoder | None = None
    self.route: str | None = None
    self.status = ''
    self._stop = threading.Event()

  # ---- live ----

  def _live_decoder(self) -> CanDecoder | None:
    if self.live is None:
      raw = self.params.get("CarParams")
      if raw is None:
        return None
      try:
        cp = messaging.log_from_bytes(raw, car.CarParams)
        names = dbcs_for_fingerprint(cp.carFingerprint)
        self.live = CanDecoder(names) if names else None
      except Exception:
        return None
    return self.live

  # ---- replay ----

  def start_replay(self, route: str) -> str | None:
    """Returns an error message, or None on success."""
    segs = _segments(route)
    if not segs:
      return f'{route}: 세그먼트를 찾을 수 없습니다'

    fingerprint = None
    for evt in _read_events(segs[0]):
      if evt.which() == 'carParams':
        fingerprint = evt.carParams.carFingerprint
        break
    if fingerprint is None:
      return f'{route}: carParams가 없어 DBC를 정할 수 없습니다'

    names = dbcs_for_fingerprint(fingerprint)
    if not names:
      return f'{fingerprint}: DBC를 찾을 수 없습니다'

    self.stop_replay()
    with self.lock:
      self.replay = CanDecoder(names, start=False)
      self.route = route
      self.status = f'{fingerprint} · 준비 중'
      self._stop = threading.Event()
    threading.Thread(target=self._run_replay, args=(route, segs, self._stop), daemon=True).start()
    return None

  def stop_replay(self) -> None:
    self._stop.set()
    with self.lock:
      self.replay = None
      self.route = None
      self.status = ''

  def _run_replay(self, route: str, segs: list[str], stop: threading.Event) -> None:
    while not stop.is_set():
      for i, path in enumerate(segs):
        if stop.is_set():
          return
        with self.lock:
          if self.replay is None:
            return
          self.status = f'재생 중 · 세그먼트 {i + 1}/{len(segs)}'
          dec = self.replay

        last = None
        for evt in _read_events(path):
          if stop.is_set():
            return
          if evt.which() != 'can':
            continue
          t = evt.logMonoTime / 1e9
          if last is not None:
            time.sleep(min(max(t - last, 0.0), MAX_SLEEP))
          last = t
          dec.ingest(evt.can, now=time.monotonic())

  # ---- serving ----

  def get(self) -> CanDecoder | None:
    with self.lock:
      if self.replay is not None:
        return self.replay
    return self._live_decoder()

  def state(self) -> dict:
    with self.lock:
      return {
        'mode': 'replay' if self.replay is not None else 'live',
        'route': self.route,
        'status': self.status,
      }
