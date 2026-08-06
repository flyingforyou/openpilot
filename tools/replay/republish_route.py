#!/usr/bin/env python3
"""Republish recorded messages onto the live message bus.

A minimal stand-in for the compiled tools/replay/replay binary, for environments where it
isn't built. Lets a real, running consumer (e.g. selfdrive.ui.ui) treat a recording as if it
were live, so UI behavior can be checked against real recorded data without a car connected.

Prefer a window produced by extract_ui_window.py: reading an rlog directly holds a whole
segment of decoded capnp objects in memory, which alongside the UI and the ffmpeg recorder was
enough to reboot a 3.5GB device.

  PYTHONPATH=. python3 tools/replay/republish_route.py <window.pkl|rlog.zst> [--loop] [--block a,b]

--block drops services from the replay so a live process can own them instead. Two publishers on
one service is an error, so replaying longitudinalPlan while plannerd runs would fail outright --
and it is exactly what you want to block when the point is to watch the live planner work.
"""
import pickle
import sys
import time

import cereal.messaging as messaging

SERVICES = [
  'carState', 'radarState', 'modelV2', 'liveCalibration', 'selfdriveState',
  'carControl', 'carParams', 'controlsState', 'longitudinalPlan', 'deviceState',
  'pandaStates', 'onroadEvents', 'driverMonitoringState', 'liveParameters', 'carOutput',
  'roadCameraState', 'wideRoadCameraState',
]


def blocked_from_argv() -> set[str]:
  if '--block' not in sys.argv:
    return set()
  i = sys.argv.index('--block')
  return set(sys.argv[i + 1].split(',')) if i + 1 < len(sys.argv) else set()


def load(path: str):
  """Return [(mono_time, service, payload_bytes)], from a prebuilt window or straight from a log."""
  if path.endswith('.pkl'):
    with open(path, 'rb') as f:
      return pickle.load(f)

  from tools.lib.logreader import LogReader
  return [(m.logMonoTime, m.which(), m.as_builder().to_bytes())
          for m in LogReader(path) if m.which() in SERVICES]


def main(path: str, loop: bool, blocked: set[str] | None = None):
  blocked = blocked or set()
  events = load(path)
  # Do not even open a socket for a blocked service: binding it is what would collide with the
  # live process meant to own it.
  socks = {s: messaging.pub_sock(s) for s in SERVICES if s not in blocked}
  if blocked:
    print(f"blocked (left to live processes): {', '.join(sorted(blocked))}")
  print(f"republishing {len(events)} messages" + (" (looping)" if loop else ""))

  n = 0
  while True:
    last_t = None
    for t, which, payload in events:
      sock = socks.get(which)
      if sock is None:
        continue
      if last_t is not None:
        dt = (t - last_t) / 1e9
        if 0 < dt < 1.0:
          time.sleep(dt)
      last_t = t
      sock.send(payload)
      n += 1
    if not loop:
      break

  print(f"done, republished {n} messages")


if __name__ == "__main__":
  argv = sys.argv[1:]
  blocked = blocked_from_argv()
  # Drop the flags and --block's own value, so the remaining word is the path.
  args, skip = [], False
  for a in argv:
    if skip:
      skip = False
      continue
    if a == '--block':
      skip = True
      continue
    if not a.startswith('--'):
      args.append(a)
  if len(args) != 1:
    print(f"usage: {sys.argv[0]} <window.pkl|rlog.zst> [--loop] [--block a,b]")
    sys.exit(1)
  main(args[0], '--loop' in sys.argv, blocked)
