"""Capture files. JSONL is the native format and keeps the raw dongle text next to the normalised
fields, so a parser bug can be fixed later and the capture re-read. CSV and candump-style are
export formats."""
from __future__ import annotations

import csv
import json
import re
import time
from collections.abc import Iterable, Iterator
from typing import IO

from tesla_diag.can.frames import CanFrame

SCHEMA = 'tesla-diag-capture/1'


class JsonlWriter:
  def __init__(self, path: str, meta: dict | None = None, mask_vin: bool = True):
    self.f: IO[str] = open(path, 'w')
    self.mask_vin = mask_vin
    self.n = 0
    header = {'schema': SCHEMA, 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'), **(meta or {})}
    self.f.write(json.dumps({'meta': header}) + '\n')

  def write(self, fr: CanFrame) -> None:
    rec = {'t': round(fr.ts, 6), 'id': fr.id_hex, 'ext': fr.extended, 'dlc': fr.dlc,
           'data': fr.data.hex(' ').upper(), 'raw': fr.raw}
    if fr.rtr:
      rec['rtr'] = True
    self.f.write(json.dumps(rec) + '\n')
    self.n += 1

  def close(self) -> None:
    self.f.close()

  def __enter__(self):
    return self

  def __exit__(self, *a):
    self.close()


def read_jsonl(path: str) -> tuple[dict, list[CanFrame]]:
  meta: dict = {}
  frames = []
  with open(path) as f:
    for n, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        d = json.loads(line)
      except ValueError as e:
        raise ValueError(f'{path}:{n}: not JSON ({e})') from None
      if 'meta' in d:
        meta = d['meta']
        continue
      frames.append(CanFrame(float(d['t']), int(d['id'], 16), bytes.fromhex(d.get('data', '')),
                             bool(d.get('ext', False)), bool(d.get('rtr', False)), d.get('raw', '')))
  return meta, frames


def write_csv(frames: Iterable[CanFrame], path: str) -> None:
  with open(path, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'id', 'extended', 'dlc', 'data', 'raw'])
    for fr in frames:
      w.writerow([f'{fr.ts:.6f}', fr.id_hex, int(fr.extended), fr.dlc, fr.data.hex(' ').upper(), fr.raw])


def candump_lines(frames: Iterable[CanFrame], iface: str = 'can0') -> Iterator[str]:
  """can-utils log format: (ts) iface ID#DATA."""
  for fr in frames:
    yield f'({fr.ts:.6f}) {iface} {fr.id_hex}#{"R" if fr.rtr else fr.data.hex().upper()}'


CANDUMP_RE = re.compile(r'\((?P<t>[\d.]+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)#(?P<d>[0-9A-Fa-f]*)')


def read_candump(path: str) -> list[CanFrame]:
  out = []
  with open(path) as f:
    for line in f:
      m = CANDUMP_RE.match(line.strip())
      if m:
        out.append(CanFrame(float(m['t']), int(m['id'], 16), bytes.fromhex(m['d']), extended=len(m['id']) > 3))
  return out


def load_any(path: str) -> tuple[dict, list[CanFrame]]:
  if path.endswith('.log') or path.endswith('.candump'):
    return {}, read_candump(path)
  return read_jsonl(path)
