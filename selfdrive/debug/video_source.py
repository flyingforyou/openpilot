"""Recorded video for the web viewer.

Every segment keeps a qcamera.ts preview of the road camera next to the full-resolution HEVC
files. Neither plays in a browser as it sits: .ts is not a container browsers accept, and the
.hevc files are raw elementary streams with no container and no timestamps at all. Remuxing
qcamera into MP4 fixes it -- a stream copy, so a 60s segment costs about 0.1s, loses nothing,
and comes out seekable. The HEVC files are left alone and offered as downloads: transcoding
1344x760 HEVC on this device would take longer than the drive did.
"""
import os
import threading

import av

REALDATA = '/data/media/0/realdata'
CACHE = '/data/tmp/qcam-mp4'   # /tmp is a small tmpfs here
CACHE_BUDGET = 400 * 1024 * 1024

PREVIEW = 'qcamera.ts'
# full-res sources, in the order worth offering them
DOWNLOADS = [
  ('fcamera.hevc', '전방'),
  ('ecamera.hevc', '광각'),
  ('dcamera.hevc', '운전자'),
  ('rlog.zst', '로그'),
]


def _route_of(entry: str) -> tuple[str, int] | None:
  name, _, num = entry.rpartition('--')
  if not name or not num.isdigit():
    return None
  return name, int(num)


def list_videos(base: str = REALDATA) -> list[dict]:
  """Routes that have something to play, newest first."""
  if not os.path.isdir(base):
    return []

  routes: dict[str, dict] = {}
  for entry in sorted(os.listdir(base)):
    parsed = _route_of(entry)
    if parsed is None:
      continue
    name, num = parsed
    preview = os.path.join(base, entry, PREVIEW)
    if not os.path.isfile(preview):
      continue

    r = routes.setdefault(name, {'name': name, 'segments': [], 'bytes': 0, 'mtime': 0})
    r['segments'].append({
      'seg': num,
      'bytes': os.path.getsize(preview),
      'downloads': [{'file': f, 'label': lab, 'bytes': os.path.getsize(os.path.join(base, entry, f))}
                    for f, lab in DOWNLOADS if os.path.isfile(os.path.join(base, entry, f))],
    })
    r['bytes'] += sum(os.path.getsize(os.path.join(base, entry, f))
                      for f in os.listdir(os.path.join(base, entry)))
    r['mtime'] = max(r['mtime'], os.path.getmtime(preview))

  out = []
  for r in routes.values():
    r['segments'].sort(key=lambda s: s['seg'])
    r['count'] = len(r['segments'])
    out.append(r)
  return sorted(out, key=lambda r: -r['mtime'])


def _seg_dir(route: str, seg: int, base: str = REALDATA) -> str:
  path = os.path.join(base, f'{route}--{seg}')
  if os.path.sep in route or '..' in route or not os.path.isdir(path):
    raise FileNotFoundError(f'{route}--{seg}')
  return path


def raw_path(route: str, seg: int, name: str, base: str = REALDATA) -> str:
  if name not in {f for f, _ in DOWNLOADS} | {PREVIEW}:
    raise FileNotFoundError(name)
  path = os.path.join(_seg_dir(route, seg, base), name)
  if not os.path.isfile(path):
    raise FileNotFoundError(path)
  return path


class Mp4Cache:
  """Remuxed segments, kept on disk under a size budget.

  Remuxing is cheap but not free, and the player re-requests a segment on every seek and
  replay, so hold on to the result. Keyed by source mtime so a segment still being written
  doesn't get served from a stale copy.
  """

  def __init__(self, cache_dir: str = CACHE, budget: int = CACHE_BUDGET):
    self.dir = cache_dir
    self.budget = budget
    self.locks: dict[str, threading.Lock] = {}
    self.guard = threading.Lock()

  def _lock(self, key: str) -> threading.Lock:
    with self.guard:
      return self.locks.setdefault(key, threading.Lock())

  def get(self, route: str, seg: int, base: str = REALDATA) -> str:
    src = raw_path(route, seg, PREVIEW, base)
    key = f'{route}--{seg}--{int(os.path.getmtime(src))}.mp4'
    dst = os.path.join(self.dir, key)

    with self._lock(key):
      if not os.path.isfile(dst):
        os.makedirs(self.dir, exist_ok=True)
        tmp = dst + f'.{os.getpid()}.part'
        try:
          _remux(src, tmp)
          os.replace(tmp, dst)
        finally:
          if os.path.exists(tmp):
            os.remove(tmp)
        self._evict(keep=dst)
    os.utime(dst, None)
    return dst

  def _evict(self, keep: str) -> None:
    """keep is the file about to be served: a budget smaller than one segment must not
    delete it out from under the response."""
    files = [(os.path.join(self.dir, f), os.path.getatime(os.path.join(self.dir, f)),
              os.path.getsize(os.path.join(self.dir, f)))
             for f in os.listdir(self.dir)
             if f.endswith('.mp4') and os.path.join(self.dir, f) != keep]
    total = sum(s for _, _, s in files)
    for path, _, size in sorted(files, key=lambda f: f[1]):
      if total <= self.budget:
        break
      try:
        os.remove(path)
        total -= size
      except OSError:
        pass


def _remux(src: str, dst: str) -> None:
  with av.open(src) as inp:
    stream = inp.streams.video[0]
    with av.open(dst, 'w', format='mp4', options={'movflags': 'faststart'}) as out:
      ostream = out.add_stream(template=stream)
      for packet in inp.demux(stream):
        if packet.dts is None:   # demuxer flush packet
          continue
        packet.stream = ostream
        out.mux(packet)
