#!/usr/bin/env python3
"""Does the panda 'interruptRateCan2' fault correlate with openpilot being engaged (i.e. with our
own bus2->bus0 blocking logic running), or is it present regardless (e.g. just genuine AP computer
traffic load)?"""

import os
import sys
from collections import Counter

LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')

import capnp
import zstandard
from openpilot.cereal import log as capnp_log


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


def scan(route, n, label):
  enabled = False
  fault_by_enabled = Counter()
  can2_err_samples = []
  can1_err_samples = []
  tx_blocked_samples = []
  n_samples = 0
  for i in range(n):
    p = os.path.join(LOG_ROOT, f'{route}--{i}', 'rlog.zst')
    if not os.path.exists(p):
      continue
    for evt in read_events(p):
      w = evt.which()
      if w == 'selfdriveState':
        enabled = evt.selfdriveState.enabled
      elif w == 'pandaStates':
        n_samples += 1
        ps = evt.pandaStates[0]
        faults = list(ps.faults)
        fault_by_enabled[(enabled, tuple(sorted(faults)))] += 1
        can2_err_samples.append((enabled, ps.canState2.totalErrorCnt))
        can1_err_samples.append((enabled, ps.canState1.totalErrorCnt))
        tx_blocked_samples.append((enabled, ps.safetyTxBlocked))

  print(f'\n=== {label} ({route}) === n_pandaState_samples={n_samples}')
  print('fault sets by enabled state:')
  for k, v in sorted(fault_by_enabled.items(), key=lambda kv: -kv[1]):
    print(f'  enabled={k[0]!s:5s} faults={k[1]}  n={v}')
  if can2_err_samples:
    last_by_en = {}
    for en, v in can2_err_samples:
      last_by_en[en] = v
    print(f'  canState2 (bus2) totalErrorCnt: last seen per state = {last_by_en}')
  if can1_err_samples:
    last_by_en = {}
    for en, v in can1_err_samples:
      last_by_en[en] = v
    print(f'  canState1 (bus1) totalErrorCnt: last seen per state = {last_by_en}')
  if tx_blocked_samples:
    last_by_en = {}
    for en, v in tx_blocked_samples:
      last_by_en[en] = v
    print(f'  safetyTxBlocked: last seen per state = {last_by_en}')


scan('0000009e--7f4078b620', 6, 'OUR drive (openpilot engaged at times)')
scan('0000009f--b644363276', 3, 'dashcam drive (never engaged)')
scan('0000009d--94840ba29c', 20, 'earlier dashcam drive (never engaged)')
