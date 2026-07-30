#!/usr/bin/env python3
"""Hunt for battery signals on a bus by behaviour rather than by name.

The harness taps the powertrain CAN, but none of the DBCs we have name the BMS
messages on this generation. So instead of guessing addresses, brute-force every
bit field on the bus and keep the ones that behave the way battery signals have
to behave:

  - pack current / power track motor torque almost exactly (r close to +1)
  - pack voltage sags under draw and rises under regen (r close to -1)
  - SOC / energy remaining drift slowly and nearly monotonically across a route

Counters, checksums and constants are thrown out first. Feed it a whole route --
the drift test needs minutes, and the correlation test needs some real throttle.
"""

import os

# Where the copied route segments live, and where to stage decompressed rlogs.
# Override with OP_LOG_ROOT / OP_SCRATCH rather than editing this.
LOG_ROOT = os.environ.get('OP_LOG_ROOT', os.path.expanduser('~/op-logs'))
SCRATCH = os.environ.get('OP_SCRATCH', '/tmp/op-analysis')
import io
import sys
from collections import defaultdict

import capnp
import numpy as np
import zstandard
from cereal import log as capnp_log

BUS = int(os.environ.get('OP_BUS', 0))

# DI_torque1 on the legacy party bus: motor torque is the reference the battery
# has to answer to. 13 bits signed at bit 16, 0.25 Nm/bit.
TORQUE_ADDR = 0x108
TORQUE_START, TORQUE_WIDTH = 16, 13

# Addresses openpilot already decodes, plus the radar/DAS chatter. Nothing here
# is going to turn out to be the BMS.
KNOWN = {0x003, 0x00E, 0x045, 0x06D, 0x101, 0x108, 0x118, 0x135, 0x155, 0x175,
         0x211, 0x214, 0x218, 0x238, 0x239, 0x246, 0x283, 0x2B8, 0x2B9, 0x2C8,
         0x2D8, 0x2E8, 0x2F8, 0x30E, 0x318, 0x338, 0x348, 0x368, 0x370, 0x388,
         0x389, 0x398, 0x399, 0x3C8, 0x3D8, 0x3E8, 0x3E9, 0x3EE, 0x428, 0x438,
         0x488,
         # Both of these top the correlation table on every route and neither is
         # the battery. 0x186 repeats DI_torqueMotor at 100Hz on the same
         # 0.25 Nm/bit scale (the value appears twice, at bit 0 and bit 16);
         # 0x145 is the ESP's longitudinal accelerometer at 50Hz.
         0x145, 0x186}

WIDTHS = (8, 10, 12, 14, 16)
GRID_HZ = 10.0


def whole_messages(buf):
  """Byte length of the leading run of complete capnp messages.

  The final segment of a route is whatever loggerd had written when the car shut
  down, so it usually ends mid-message. capnp aborts the process on that rather
  than raising something catchable, so walk the stream framing first and hand it
  only what is complete: a 4-byte segment count, one 4-byte word count per
  segment, padding to 8 bytes, then the segment data.
  """
  end = 0
  n = len(buf)
  while end < n:
    if end + 4 > n:
      break
    count = int.from_bytes(buf[end:end + 4], 'little') + 1
    header = 4 + 4 * count
    header += -header % 8
    if end + header > n:
      break
    sizes = buf[end + 4:end + 4 + 4 * count]
    words = sum(int.from_bytes(sizes[i:i + 4], 'little') for i in range(0, len(sizes), 4))
    total = header + 8 * words
    if end + total > n:
      break
    end += total
  return end


def read_events(path):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'battery-{os.getpid()}.rlog')
  try:
    buf = io.BytesIO()
    with open(path, 'rb') as src:
      zstandard.ZstdDecompressor().copy_stream(src, buf)
    raw = buf.getvalue()
    good = whole_messages(raw)
    if good < len(raw):
      print(f'  {os.path.basename(os.path.dirname(path))}: dropped '
            f'{len(raw) - good} trailing bytes (truncated segment)', file=sys.stderr)
    with open(tmp, 'wb') as dst:
      dst.write(raw[:good])
    with open(tmp, 'rb') as f:
      try:
        yield from capnp_log.Event.read_multiple(f)
      except capnp.KjException:
        pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def collect(paths):
  """(bus, addr) -> (times, payload matrix of uint8)."""
  times = defaultdict(list)
  data = defaultdict(list)
  for path in paths:
    for evt in read_events(path):
      if evt.which() != 'can':
        continue
      t = evt.logMonoTime / 1e9
      for f in evt.can:
        if f.src != BUS:
          continue
        times[f.address].append(t)
        data[f.address].append(bytes(f.dat))

  out = {}
  for addr, payloads in data.items():
    width = max(len(p) for p in payloads)
    # A message whose length wobbles is a diagnostic response, not a signal.
    if min(len(p) for p in payloads) != width or len(payloads) < 50:
      continue
    mat = np.frombuffer(b''.join(payloads), dtype=np.uint8).reshape(-1, width)
    # A shell glob hands segments over lexically -- --1, --10, --11, --2 -- so
    # the frames arrive badly out of order. np.interp needs increasing x, and
    # without this a 49-segment route silently collapses to the ~10 minutes that
    # happen to land before the first backwards jump.
    order = np.argsort(np.array(times[addr]), kind='stable')
    out[addr] = (np.array(times[addr])[order], mat[order])
  return out


def unpack(mat, little):
  """Bit matrix for a whole message. Hoisted out of the sweep -- unpacking once
  per address instead of once per candidate field is the difference between
  seconds and minutes."""
  return np.unpackbits(mat, axis=1, bitorder='little' if little else 'big').astype(np.int64)


def field(bits, start, width, little, signed):
  """Pull one bit field out of every row.

  The little-endian case matches DBC `@1` bit numbering exactly. The big-endian
  case is a plain MSB-first slice, not Motorola `@0` numbering -- since every
  start bit gets swept it still covers the same set of fields, but a printed
  `@0` spec has to be re-derived before it goes in a DBC.
  """
  sel = bits[:, start:start + width]
  if little:
    vals = sel @ (1 << np.arange(width))
  else:
    vals = sel @ (1 << np.arange(width - 1, -1, -1))
  if signed:
    vals = np.where(vals >= (1 << (width - 1)), vals - (1 << width), vals)
  return vals


def extract(mat, start, width, little, signed):
  return field(unpack(mat, little), start, width, little, signed)


def is_counter(vals, width):
  """Rolling counters step by a constant amount and wrap. Nothing else does."""
  d = np.diff(vals.astype(np.int64))
  d = np.where(d < 0, d + (1 << width), d)
  if len(d) == 0:
    return False
  step = np.bincount(np.clip(d, 0, 16)).argmax()
  return step != 0 and (d == step).mean() > 0.9


def is_noise(vals):
  """Checksums and CRCs hop across their whole range every frame."""
  span = vals.max() - vals.min()
  if span == 0:
    return True
  return np.abs(np.diff(vals)).mean() > 0.25 * span


def resample(t, vals, grid):
  return np.interp(grid, t, vals)


# Correlation alone is not enough -- 0x145 tracks motor torque at r=+0.95 and is
# the ESP's longitudinal accelerometer. A battery signal also has to land in a
# physically possible range under some believable scale.
SCALES = (1.0, 0.5, 0.25, 0.2, 0.1, 0.05, 0.025, 0.02, 0.01, 0.005, 0.002, 0.001)

# 85/90kWh Model X pack: ~96s, so ~250V empty-and-sagging to ~420V on regen.
VOLT_RANGE = (250.0, 430.0)
SOC_RANGE = (0.0, 100.0)


def plausible(lo, hi, window, min_span):
  """Scales under which the whole series sits inside a physical window."""
  out = []
  for s in SCALES:
    if window[0] <= lo * s and hi * s <= window[1] and (hi - lo) * s >= min_span:
      out.append(s)
  return out


def main(paths):
  frames = collect(paths)
  if TORQUE_ADDR not in frames:
    print(f'no 0x{TORQUE_ADDR:03X} on bus {BUS} -- wrong bus, or not a legacy car')
    return

  tt, tmat = frames[TORQUE_ADDR]
  torque = extract(tmat, TORQUE_START, TORQUE_WIDTH, True, True) * 0.25

  t0, t1 = tt[0], tt[-1]
  grid = np.arange(t0, t1, 1.0 / GRID_HZ)
  # Segments handed in on the command line are rarely contiguous, and np.interp
  # draws a straight line across every hole. Two such ramps correlate with each
  # other beautifully and with nothing real -- that is what produced a 328-366 V
  # "pack voltage" on one route and r=0 on the other four. Drop the holes.
  gaps = np.diff(tt)
  for i in np.flatnonzero(gaps > max(1.0, 10 * np.median(gaps))):
    grid = grid[(grid <= tt[i]) | (grid >= tt[i + 1])]
  ref = resample(tt, torque, grid)
  ref = ref - ref.mean()
  ref_norm = np.linalg.norm(ref)
  duration = t1 - t0

  print(f'bus {BUS}, {duration:.0f}s, {len(frames)} addresses, '
        f'motor torque {torque.min():.0f}..{torque.max():.0f} Nm')
  if torque.max() - torque.min() < 100:
    print('  WARNING: little torque variation in this log -- the correlation '
          'test will not separate anything. Use a log with real acceleration.')
  print()

  # Best candidate per address per category -- the sweep reports the same
  # underlying field a dozen times at different widths, which buries the signal.
  volts, current, soc = {}, {}, {}

  def keep(bag, addr, score, row):
    if addr not in bag or score > bag[addr][0]:
      bag[addr] = (score, row)

  for addr, (t, mat) in sorted(frames.items()):
    if addr in KNOWN:
      continue
    nbits = mat.shape[1] * 8
    packed = {le: unpack(mat, le) for le in (True, False)}
    for width in WIDTHS:
      for start in range(0, nbits - width + 1):
        for little in (True, False):
          for signed in (False, True):
            if signed and width == 8:
              continue
            vals = field(packed[little], start, width, little, signed)
            if len(np.unique(vals)) < 8 or is_counter(vals, width) or is_noise(vals):
              continue

            y = resample(t, vals.astype(float), grid)
            yc = y - y.mean()
            yn = np.linalg.norm(yc)
            if yn == 0:
              continue
            r = float(yc @ ref / (yn * ref_norm))
            lo, hi = float(vals.min()), float(vals.max())
            desc = (f'0x{addr:03X} {start:2d}|{width}@{1 if little else 0}'
                    f'{"-" if signed else "+"}')

            # Pack voltage: sags under draw, and sits in a real pack's window.
            if r < -0.6:
              for s in plausible(lo, hi, VOLT_RANGE, 5.0):
                keep(volts, addr, -r, f'r={r:+.3f}  {desc}  x{s} -> '
                                      f'{lo * s:.1f}..{hi * s:.1f} V')
                break

            # Pack current: signed, centred near zero, follows torque closely.
            if r > 0.9 and signed and lo < 0 < hi and abs(lo + hi) < 0.5 * (hi - lo):
              keep(current, addr, r, f'r={r:+.3f}  {desc}  raw {lo:.0f}..{hi:.0f}')

            # SOC and energy walk one direction and barely move per second.
            steps = np.diff(y)
            up, down = int((steps > 0).sum()), int((steps < 0).sum())
            if up + down > 20 and hi - lo > 2:
              monotone = max(up, down) / (up + down)
              if monotone > 0.95:
                for s in plausible(lo, hi, SOC_RANGE, 0.5):
                  keep(soc, addr, monotone,
                       f'monotone={monotone:.2f}  {desc}  x{s} -> '
                       f'{vals[0] * s:.1f} to {vals[-1] * s:.1f} %  '
                       f'({(hi - lo) * s / duration * 3600:.1f} %/h)')
                  break

  def report(title, bag, empty):
    print(f'=== {title} ===')
    if not bag:
      print(f'  {empty}')
    for _, row in sorted(bag.values(), reverse=True)[:12]:
      print(f'  {row}')
    print()

  report('pack voltage (falls under draw, lands in 250-430 V)', volts,
         'nothing in a pack-voltage range moves against torque')
  report('pack current (signed, zero-centred, follows torque)', current,
         'no zero-centred field tracks torque above r=0.9')
  report('SOC / energy remaining (one-way drift, 0-100 %)', soc,
         'nothing drifting one way -- log may be too short to see SOC move')


if __name__ == '__main__':
  main(sys.argv[1:])
