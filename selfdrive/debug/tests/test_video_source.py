import os
import time

import av
import numpy as np
import pytest

from openpilot.selfdrive.debug import video_source
from openpilot.selfdrive.debug.video_source import Mp4Cache, list_videos, raw_path

FPS = 20


def write_ts(path: str, seconds: float = 1.0):
  """A real MPEG-TS with a real H.264 stream -- the remux has nothing to prove against a stub."""
  with av.open(path, 'w', format='mpegts') as c:
    s = c.add_stream('libx264', rate=FPS)
    s.width, s.height, s.pix_fmt = 64, 48, 'yuv420p'
    s.options = {'preset': 'ultrafast', 'tune': 'zerolatency'}
    for i in range(int(seconds * FPS)):
      frame = av.VideoFrame.from_ndarray(np.full((48, 64, 3), i * 4 % 256, dtype=np.uint8), format='rgb24')
      c.mux(s.encode(frame))
    c.mux(s.encode())


@pytest.fixture
def realdata(tmp_path):
  base = tmp_path / 'realdata'
  for route, segs in [('00000000--aaaa', [0, 1, 2]), ('00000001--bbbb', [0])]:
    for seg in segs:
      d = base / f'{route}--{seg}'
      d.mkdir(parents=True)
      write_ts(str(d / 'qcamera.ts'))
      (d / 'fcamera.hevc').write_bytes(b'\x00' * 1234)
      (d / 'rlog.zst').write_bytes(b'\x00' * 99)
  # a route mid-recording, plus the boot dir, neither of which has anything to play
  (base / '00000002--cccc--0').mkdir()
  (base / 'boot').mkdir()
  return str(base)


def test_lists_newest_first_with_only_playable_routes(realdata):
  os.utime(os.path.join(realdata, '00000001--bbbb--0', 'qcamera.ts'), (time.time(),) * 2)
  routes = list_videos(realdata)

  assert [r['name'] for r in routes] == ['00000001--bbbb', '00000000--aaaa']
  assert [r['count'] for r in routes] == [1, 3]
  assert [s['seg'] for s in routes[1]['segments']] == [0, 1, 2]
  assert routes[1]['bytes'] > 3 * 1234, 'route size counts the full-res files, not just the preview'


def test_downloads_are_the_files_that_exist(realdata):
  seg = list_videos(realdata)[0]['segments'][0]

  assert [d['file'] for d in seg['downloads']] == ['fcamera.hevc', 'rlog.zst']
  assert [d['bytes'] for d in seg['downloads']] == [1234, 99]


def test_no_segments_means_no_route(tmp_path):
  assert list_videos(str(tmp_path / 'nope')) == []


@pytest.mark.parametrize("route, seg, name", [
  ('../../../etc', 0, 'qcamera.ts'),
  ('00000000--aaaa', 0, '../../../etc/passwd'),
  ('00000000--aaaa', 0, 'secrets'),
  ('00000000--aaaa', 99, 'qcamera.ts'),
])
def test_raw_path_refuses_anything_it_did_not_offer(realdata, route, seg, name):
  with pytest.raises(FileNotFoundError):
    raw_path(route, seg, name, realdata)


def test_raw_path_serves_a_listed_file(realdata):
  assert os.path.isfile(raw_path('00000000--aaaa', 1, 'fcamera.hevc', realdata))


def test_remux_is_playable_and_keeps_the_frames(realdata, tmp_path):
  cache = Mp4Cache(str(tmp_path / 'cache'))
  out = cache.get('00000000--aaaa', 0, realdata)

  with av.open(out) as c:
    assert 'mp4' in c.format.name
    stream = c.streams.video[0]
    assert stream.codec_context.name == 'h264', 'stream copy, not a transcode'
    assert (stream.width, stream.height) == (64, 48)
    assert sum(1 for _ in c.decode(stream)) == FPS
    assert c.duration is not None, 'a duration is what lets the player show a seek bar'


def test_second_request_is_served_from_cache(realdata, tmp_path):
  cache = Mp4Cache(str(tmp_path / 'cache'))
  first = cache.get('00000000--aaaa', 0, realdata)
  before = os.path.getmtime(first)

  assert cache.get('00000000--aaaa', 0, realdata) == first
  assert os.path.getmtime(first) == before, 'not remuxed again'


def test_rewritten_segment_is_not_served_stale(realdata, tmp_path):
  """The segment being recorded right now grows; its cached copy must not outlive it."""
  cache = Mp4Cache(str(tmp_path / 'cache'))
  first = cache.get('00000000--aaaa', 0, realdata)

  src = os.path.join(realdata, '00000000--aaaa--0', 'qcamera.ts')
  write_ts(src, seconds=2.0)
  os.utime(src, (time.time() + 10,) * 2)

  second = cache.get('00000000--aaaa', 0, realdata)
  assert second != first
  with av.open(second) as c:
    assert sum(1 for _ in c.decode(c.streams.video[0])) == 2 * FPS


def test_cache_stays_under_budget(realdata, tmp_path):
  cache = Mp4Cache(str(tmp_path / 'cache'), budget=1)
  served = [cache.get('00000000--aaaa', seg, realdata) for seg in range(3)]

  assert os.listdir(cache.dir) == [os.path.basename(served[-1])], 'older entries evicted'
  assert os.path.isfile(served[-1]), 'a budget below one segment must still serve that segment'


def test_default_locations_are_the_device_ones():
  assert video_source.REALDATA == '/data/media/0/realdata'
  assert not video_source.CACHE.startswith('/tmp/'), '/tmp is a small tmpfs on this device'
