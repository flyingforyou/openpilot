"""Bus captures from the dongle: summarise what was heard, decode it, keep it as JSONL.

Listen-only. A capture is a list of (seconds, 11-bit id, data); nothing here talks to the car.
The point is to answer, before anything else, "is the dongle on a bus that carries these
frames at all, and at what rate": the ID table says that, and decoding the known IDs through the
same decoders the live page uses gives numbers to compare with Scan My Tesla's.
"""
from __future__ import annotations

import json
import os
import re
import time

from openpilot.system.btobd import tesla_sx as t

CAPTURE_DIR = '/data/btobd_logs'
KEEP = 10
NAME_RE = re.compile(r'capture-\d{8}-\d{6}\.jsonl')
CORE_IDS = (0x102, 0x302, 0x382, 0x6F2)    # what the battery page needs
NAMES = {i: d.__name__ for i, d in t.DECODERS.items()} | {t.CELL_ID: 'bricks'}


def summarize(frames, duration: float, buffer_full: bool = False, unparsed: int = 0) -> dict:
  per: dict[int, dict] = {}
  for _ts, fid, data in frames:
    e = per.setdefault(fid, {'n': 0})
    e['n'] += 1
    e['last'] = data.hex(' ').upper()
  duration = max(duration, 1e-6)
  state = t.TeslaSXState()
  for ts, fid, data in frames:
    state.feed(fid, data, ts)
  ids = [{'id': f'{fid:03X}', 'n': e['n'], 'hz': round(e['n'] / duration, 1), 'last': e['last'], 'known': NAMES.get(fid)}
         for fid, e in sorted(per.items())]
  warnings = []
  if not frames:
    warnings.append('프레임을 하나도 받지 못했습니다. 동글이 꽂힌 위치(진단 커넥터용 어댑터 케이블), '
                    'CAN 프로토콜·속도, 차가 깨어 있는지를 확인하세요.')
  if buffer_full:
    warnings.append('동글 버퍼가 넘쳤습니다. 프레임이 빠졌을 수 있고, 드물게 오는 메시지일수록 잘 빠집니다.')
  if unparsed:
    warnings.append(f'해석하지 못한 줄이 {unparsed}개 있었습니다 (동글 오류나 잡음).')
  if frames:
    missing = [f'{i:03X}' for i in CORE_IDS if i not in per]
    if missing:
      warnings.append('배터리 화면에 필요한 ID 중 듣지 못한 것: ' + ', '.join(missing) +
                      ' (연결한 버스에 없거나, 짧은 시간 안에 오지 않았거나, 동글이 놓쳤을 수 있습니다)')
  return {'frames': len(frames), 'duration': round(duration, 1), 'unique': len(per),
          'fps': round(len(frames) / duration), 'ids': ids, 'decoded': state.values,
          'warnings': warnings, 'bufferFull': buffer_full}


def save(frames, meta: dict, directory: str = CAPTURE_DIR) -> str:
  os.makedirs(directory, exist_ok=True)
  name = time.strftime('capture-%Y%m%d-%H%M%S.jsonl')
  path = os.path.join(directory, name)
  with open(path, 'w') as f:
    f.write(json.dumps({'meta': meta}) + '\n')
    for ts, fid, data in frames:
      f.write(json.dumps({'t': round(ts, 4), 'id': f'{fid:03X}', 'data': data.hex(' ').upper()}) + '\n')
  os.chmod(path, 0o644)
  for old in sorted(n for n in os.listdir(directory) if NAME_RE.fullmatch(n))[:-KEEP]:
    os.remove(os.path.join(directory, old))
  return name


def load(path: str):
  """Frames of a saved capture, for offline decoding and replay."""
  frames = []
  with open(path) as f:
    for line in f:
      d = json.loads(line)
      if 'id' in d:
        frames.append((d['t'], int(d['id'], 16), bytes.fromhex(d['data'])))
  return frames
