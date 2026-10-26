"""ELM327 monitor text -> CanFrame.

The id length is not guessed from the text: ATSP6/8 are 11-bit, ATSP7/9 are 29-bit, and the caller
passes that in. With ATH1 + ATS1 an 11-bit id is one 3-digit token, a 29-bit id is four 2-digit
tokens. With ATS0 the run is unspaced and parity tells them apart only if the id format is known,
which it is. Anything that is not a frame (status words, prompts, garbage) returns a Classified
non-frame so the caller can count it instead of silently dropping it.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

from tesla_diag.can.frames import CanFrame

HEX = re.compile(r'^[0-9A-F]+$')
MAX_LINE = 200     # a classic CAN line is < 40 chars; anything longer is a framing accident

# dongle status words, longest first so 'BUFFER FULL' wins over 'FULL'
STATUS_WORDS = ('BUFFER FULL', 'CAN ERROR', 'BUS ERROR', 'BUS BUSY', 'DATA ERROR', 'FB ERROR',
                'LV RESET', 'NO DATA', 'STOPPED', 'SEARCHING', 'UNABLE TO CONNECT', 'ERR', '<RX ERROR',
                'ACT ALERT', 'OK', '?')


@dataclass(frozen=True)
class Parsed:
  frame: CanFrame | None = None
  status: str | None = None     # one of STATUS_WORDS when the line is a dongle message
  garbage: bool = False         # neither a frame nor a known status


def protocol_id_format(protocol: str) -> str:
  """'11' or '29' for the CAN protocol numbers the ELM327 offers (6-9)."""
  p = protocol.upper().lstrip('A')
  if p in ('6', '8'):
    return '11'
  if p in ('7', '9'):
    return '29'
  raise ValueError(f'protocol {protocol!r} is not a plain CAN protocol (6/7/8/9)')


def parse_line(line: str, id_format: str = '11', ts: float = 0.0) -> Parsed:
  text = line.strip().upper()
  if not text:
    return Parsed(garbage=False)
  if len(text) > MAX_LINE:
    return Parsed(garbage=True)
  for w in STATUS_WORDS:
    if text == w or text.startswith(w):
      return Parsed(status=w)
  rtr = text.endswith('RTR')
  body = text[:-3].strip() if rtr else text
  tokens = body.split()
  id_chars = 3 if id_format == '11' else 8
  try:
    if len(tokens) > 1:                         # ATS1: spaced
      if id_format == '11':
        idtok, data_toks = tokens[0], tokens[1:]
        if len(idtok) != 3:
          return Parsed(garbage=True)
      else:
        if len(tokens) < 4 or any(len(t) != 2 for t in tokens[:4]):
          return Parsed(garbage=True)
        idtok, data_toks = ''.join(tokens[:4]), tokens[4:]
      if any(len(t) != 2 for t in data_toks):
        return Parsed(garbage=True)
      hexdata = ''.join(data_toks)
    else:                                       # ATS0: one unspaced run
      run = tokens[0]
      if len(run) < id_chars or (len(run) - id_chars) % 2:
        return Parsed(garbage=True)
      idtok, hexdata = run[:id_chars], run[id_chars:]
    if not HEX.match(idtok) or (hexdata and not HEX.match(hexdata)):
      return Parsed(garbage=True)
    data = bytes.fromhex(hexdata)
    if len(data) > 8 or (not data and not rtr):
      return Parsed(garbage=True)
    return Parsed(frame=CanFrame(ts, int(idtok, 16), data, extended=id_format == '29', rtr=rtr, raw=line.strip()))
  except ValueError:
    return Parsed(garbage=True)
