#!/usr/bin/env python3
"""Republish recorded messages onto the live message bus.

A minimal stand-in for the compiled tools/replay/replay binary, for environments where it
isn't built. Lets a real, running consumer (e.g. selfdrive.ui.ui) treat a recording as if it
were live, so UI behavior can be checked against real recorded data without a car connected.

Prefer a window produced by extract_ui_window.py: reading an rlog directly holds a whole
segment of decoded capnp objects in memory, which alongside the UI and the ffmpeg recorder was
enough to reboot a 3.5GB device.

  PYTHONPATH=. python3 tools/replay/republish_route.py <window.pkl|rlog.zst> [--loop]
"""
import pickle
import sys
import time

import cereal.messaging as messaging

SERVICES = [
  'carState', 'radarState', 'modelV2', 'liveCalibration', 'selfdriveState',
  'carControl', 'carParams', 'controlsState', 'longitudinalPlan', 'deviceState',
  'pandaStates', 'onroadEvents', 'driverMonitoringState', 'liveParameters', 'carOutput',
]


def load(path: str):
  """Return [(mono_time, service, payload_bytes)], from a prebuilt window or straight from a log."""
  if path.endswith('.pkl'):
    with open(path, 'rb') as f:
      return pickle.load(f)

  from tools.lib.logreader import LogReader
  return [(m.logMonoTime, m.which(), m.as_builder().to_bytes())
          for m in LogReader(path) if m.which() in SERVICES]


def main(path: str, loop: bool):
  events = load(path)
  socks = {s: messaging.pub_sock(s) for s in SERVICES}
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
  args = [a for a in sys.argv[1:] if not a.startswith('--')]
  if len(args) != 1:
    print(f"usage: {sys.argv[0]} <window.pkl|rlog.zst> [--loop]")
    sys.exit(1)
  main(args[0], '--loop' in sys.argv)
