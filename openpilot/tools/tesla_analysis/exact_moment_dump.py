#!/usr/bin/env python3
"""Frame-by-frame dump of DAS_lanes (ours or genuine) plus our own model's laneLineProbs, around a
specific segment-relative timestamp the user identified as a moment where lines should render but
don't."""

import os
import sys

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log
from opendbc.can.dbc import DBC as DBCLoader
from opendbc.can.parser import get_raw_value

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
dbc = DBCLoader(os.path.join(REPO_ROOT, 'opendbc_repo', 'opendbc', 'dbc', 'tesla_can.dbc'))
lanes_msg = dbc.addr_to_msg[0x239]
status_sig = dbc.addr_to_msg[0x399].sigs['autopilotStatus']
LANE_FIELDS = ['DAS_leftLaneExists', 'DAS_rightLaneExists', 'DAS_virtualLaneWidth',
               'DAS_virtualLaneViewRange', 'DAS_virtualLaneC0', 'DAS_virtualLaneC1',
               'DAS_virtualLaneC2', 'DAS_virtualLaneC3', 'DAS_leftLineUsage', 'DAS_rightLineUsage']


def phys(sig, raw):
  return raw * sig.factor + sig.offset


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'w-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def dump(route, seg, center_t, half_window, our_src):
  p = os.path.join(LOG_ROOT, f'{route}--{seg}', 'rlog.zst')
  events = list(read_events(p))
  # events[0].logMonoTime is NOT segment-relative zero -- some messages (initData/carParams) keep
  # a stale timestamp from route start. Anchor to the first 'can'/selfdriveState sample instead,
  # which is what actually starts this segment's 60s window.
  t0 = min(e.logMonoTime for e in events if e.which() in ('can', 'selfdriveState'))
  lo, hi = center_t - half_window, center_t + half_window

  print(f'\n===== {route}--{seg}  window [{lo:.1f}s, {hi:.1f}s] (our_src={our_src}) =====')
  enabled = False
  for evt in events:
    t = (evt.logMonoTime - t0) / 1e9
    if t < lo or t > hi:
      continue
    w = evt.which()
    if w == 'selfdriveState':
      new_en = evt.selfdriveState.enabled
      if new_en != enabled:
        print(f'  t={t:6.2f}s  ENGAGED -> {new_en}')
      enabled = new_en
    elif w == 'modelV2':
      probs = list(evt.modelV2.laneLineProbs)
      if len(probs) >= 3:
        print(f'  t={t:6.2f}s  modelV2 laneLineProbs: left={probs[1]:.3f} right={probs[2]:.3f}  n_laneLines={len(evt.modelV2.laneLines)}')
        if len(evt.modelV2.laneLines) >= 3:
          xs = list(evt.modelV2.laneLines[1].x)
          print(f'          laneLines[1].x len={len(xs)}  sample={xs[:3]}...{xs[-3:] if len(xs) > 3 else []}')
    elif w == 'can':
      for c in evt.can:
        if c.address == 0x399 and c.src in (2, 128, our_src):
          st = int(phys(status_sig, get_raw_value(bytes(c.dat), status_sig)))
          print(f'  t={t:6.2f}s  src={c.src:3d} AutopilotStatus={st}')
        if c.address == 0x239 and c.src in (2, our_src):
          vals = {f: phys(lanes_msg.sigs[f], get_raw_value(bytes(c.dat), lanes_msg.sigs[f])) for f in LANE_FIELDS}
          tag = 'OURS' if c.src == our_src else 'genuine(bus2)'
          print(f'  t={t:6.2f}s  src={c.src:3d} [{tag}] DAS_lanes: '
                f'L={vals["DAS_leftLaneExists"]:.0f} R={vals["DAS_rightLaneExists"]:.0f} '
                f'Lu={vals["DAS_leftLineUsage"]:.0f} Ru={vals["DAS_rightLineUsage"]:.0f} '
                f'width={vals["DAS_virtualLaneWidth"]:.2f} vr={vals["DAS_virtualLaneViewRange"]:.1f} '
                f'C0={vals["DAS_virtualLaneC0"]:.3f} C1={vals["DAS_virtualLaneC1"]:.4f}')


dump('0000009e--7f4078b620', 2, 0.0, 8.0, our_src=128)
dump('0000009f--b644363276', 1, 18.0, 8.0, our_src=None)
