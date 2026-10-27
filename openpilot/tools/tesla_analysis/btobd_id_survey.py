#!/usr/bin/env python3
"""Match a btobd capture's IDs against talas9/tesla_can_signals and decode what matches.

  python btobd_id_survey.py <capture.jsonl> <talas9 ModelS dir>

The talas9 dir holds ModelS_<BUS>.compact.json.json (fetch from github.com/talas9/tesla_can_signals).
For each bus it reports how many capture IDs exist there with the same frame length -- on the
2026-10-27 capture PT matched 17/17, so the diagnostic port carries the powertrain bus -- then
decodes every matched signal (mux-aware). A match is an ID + length coincidence, not proof: check
the observed count against the listed cycle time and that the values are physically sensible.
Signals with VIN in the name are not printed.
"""
import collections
import json
import os
import sys

BUSES = ['PT', 'ETH', 'CH', 'BFT', 'BDY', 'OBDII']


def load_capture(path):
  per = collections.defaultdict(list)
  with open(path) as f:
    for line in f:
      d = json.loads(line)
      if 'id' in d:
        per[int(d['id'], 16)].append(bytes.fromhex(d['data'].replace(' ', '')))
  return per


def ext(d, s):
  w, st = s['width'], s['start_position']
  if s['endianness'] == 'LITTLE':
    v = (int.from_bytes(d.ljust(8, b'\0'), 'little') >> st) & ((1 << w) - 1)
  else:
    byte, bit = divmod(st, 8)
    pos = byte * 8 + (7 - bit)
    v = (int.from_bytes(d.ljust(8, b'\0'), 'big') >> (64 - pos - w)) & ((1 << w) - 1)
  if s['signedness'] == 'SIGNED' and v >> (w - 1):
    v -= 1 << w
  return v


def describe(s, vals):
  vd = {v: k for k, v in (s.get('value_description') or {}).items()}

  def show(v):
    return vd[v] if v in vd else f"{v * s['scale'] + s['offset']:.4g}"
  uniq = sorted(set(vals))
  if len(uniq) <= 6:
    return ', '.join(show(v) for v in uniq)
  ph = [v * s['scale'] + s['offset'] for v in vals if v not in vd] or [0]
  return f'{min(ph):.4g}..{max(ph):.4g} (last {show(vals[-1])})'


def main(capture, talas9):
  per = load_capture(capture)
  dlc = {i: collections.Counter(len(d) for d in ds).most_common(1)[0][0] for i, ds in per.items()}
  buses = {}
  for b in BUSES:
    p = os.path.join(talas9, f'ModelS_{b}.compact.json.json')
    if os.path.exists(p):
      with open(p) as f:
        buses[b] = {v['message_id']: (k, v) for k, v in json.load(f)['messages'].items()}
  for b, msgs in buses.items():
    hit = [i for i in per if i in msgs]
    print(f'{b}: {len(msgs)} messages, {len(hit)} capture IDs present, '
          f'{sum(msgs[i][1]["length_bytes"] == dlc[i] for i in hit)} with matching length')
  for b, msgs in buses.items():
    for i in sorted(per):
      if i not in msgs or msgs[i][1]['length_bytes'] != dlc[i]:
        continue
      name, m = msgs[i]
      ds = [d for d in per[i] if len(d) == dlc[i]]
      sig = m['signals']
      muxer = next((s for s in sig.values() if s.get('is_muxer')), None)
      print(f'\n[{b}] {i:03X} {name}  n={len(ds)} cycle={m.get("cycle_time")}ms from={m.get("senders")}')
      for sn, s in sig.items():
        if 'VIN' in sn.upper():
          continue
        use = [d for d in ds if ext(d, muxer) == s['mux_id']] if 'mux_id' in s and muxer else ds
        if not use:
          print(f'   {sn:42s} (mux {s["mux_id"]} not heard)')
          continue
        print(f'   {sn:42s} {describe(s, [ext(d, s) for d in use])} {s.get("units", "")}  [n={len(use)}]')
  unmatched = [i for i in sorted(per) if not any(i in m and m[i][1]['length_bytes'] == dlc[i] for m in buses.values())]
  print(f'\nunmatched: {len(unmatched)} IDs')


if __name__ == '__main__':
  main(*sys.argv[1:3])
