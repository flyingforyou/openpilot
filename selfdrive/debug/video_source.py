"""Recorded video for the web viewer.

Every segment keeps a qcamera.ts preview of the road camera next to the full-resolution HEVC
files. Neither plays in a browser as it sits: .ts is not a container browsers accept, and the
.hevc files are raw elementary streams with no container and no timestamps at all. Remuxing
qcamera into MP4 fixes it -- a stream copy, so a 60s segment costs about 0.1s, loses nothing,
and comes out seekable.

qcamera is 526x330 though, which is enough to find a moment and not enough to read one, so the
full-resolution cameras are served as well -- see CAMERAS and CODECS.
"""
import os
import subprocess
import threading

import av

REALDATA = '/data/media/0/realdata'
CACHE = '/data/tmp/qcam-mp4'   # /tmp is a small tmpfs here; this is on /data's disk, not RAM
# H.264 transcoding is the expensive path (~40s/segment, no hardware encoder on this device), so
# an evicted segment is not a cache miss, it is another 40s wait. 400MB held about 13 segments --
# less than one route's worth of scanning -- so browsing a scan strip kept re-paying that cost on
# segments already visited. /data has 21GB free; this trades a slice of it for not doing that.
CACHE_BUDGET = 4 * 1024 * 1024 * 1024

PREVIEW = 'qcamera.ts'

# The three recorded views, all 1344x760 HEVC. qcamera is kept only as a fallback for segments
# that predate a camera file -- at 526x330 it finds a moment without showing one, so nothing
# offers it by choice.
CAMERAS = {
  'road': 'fcamera.hevc',
  'wide': 'ecamera.hevc',
  'driver': 'dcamera.hevc',
}

# Two ways to hand a camera file to a browser, both measured here over a 60s segment:
#   copy  -- 0.75s, 36MB, lossless, but HEVC in MP4 only plays where the browser decodes it
#   h264  -- 41s, 18MB, lossy, plays anywhere
# The page asks canPlayType and picks; copy is preferred because it is the original bitstream.
CODECS = ('copy', 'h264')
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

  def get(self, route: str, seg: int, base: str = REALDATA,
          cam: str = 'road', codec: str = 'copy') -> str:
    if codec not in CODECS:
      codec = 'copy'
    name = CAMERAS.get(cam)
    if name is None:
      cam, name = 'preview', PREVIEW
    src = raw_path(route, seg, name, base)
    key = f'{route}--{seg}--{cam}--{codec}--{int(os.path.getmtime(src))}.mp4'
    dst = os.path.join(self.dir, key)

    with self._lock(key):
      if not os.path.isfile(dst):
        os.makedirs(self.dir, exist_ok=True)
        tmp = dst + f'.{os.getpid()}.part'
        try:
          if cam == 'preview':
            _remux(src, tmp)
          else:
            _from_camera(src, tmp, transcode=(codec == 'h264'))
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


def _from_camera(src: str, dst: str, transcode: bool) -> None:
  """Put a full-resolution camera file in a container a browser will accept.

  The .hevc files are raw elementary streams with no container and no timestamps at all, so the
  frame rate has to be asserted on the way in -- loggerd writes them at 20fps. Copying the stream
  costs nothing but leaves HEVC, which not every browser decodes; transcoding costs about 40s a
  segment at ultrafast and plays everywhere.
  """
  # -f mp4 is not optional: the destination is written to a .part file first so a killed run
  # cannot leave a half-muxed segment in the cache, and ffmpeg picks its muxer from the
  # extension, which .part is not.
  out = subprocess.run(
    ['ffmpeg', '-y', '-v', 'error', '-r', '20', '-i', src]
    + (['-c', 'copy'] if transcode is False else
       ['-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '28'])
    + ['-movflags', 'faststart', '-f', 'mp4', dst],
    capture_output=True, text=True)
  if out.returncode != 0:
    raise RuntimeError(f'ffmpeg failed ({out.returncode}): {out.stderr.strip()[:300]}')


def _remux(src: str, dst: str) -> None:
  with av.open(src) as inp:
    stream = inp.streams.video[0]
    with av.open(dst, 'w', format='mp4', options={'movflags': 'faststart'}) as out:
      ostream = out.add_stream(template=stream)
      # A transport stream's first timestamp is wherever the broadcast clock happened to be, not
      # zero, and copying it through leaves an MP4 whose movie header spans start_pts..end while
      # the track only holds a minute of samples. The browser then reports a duration minutes
      # longer than the video and starts currentTime at the offset instead of 0, which silently
      # breaks anything lining data up against playback position. Rebase onto zero.
      first = None
      for packet in inp.demux(stream):
        if packet.dts is None:   # demuxer flush packet
          continue
        if first is None:
          first = packet.dts
        packet.dts -= first
        if packet.pts is not None:
          packet.pts -= first
        packet.stream = ostream
        out.mux(packet)
