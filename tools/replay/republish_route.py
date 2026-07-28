#!/usr/bin/env python3
"""Republish a recorded route's messages onto the live message bus.

A minimal stand-in for the compiled tools/replay/replay binary, for environments where it
isn't built. Lets a real, running consumer (e.g. selfdrive.ui.ui) treat a recorded rlog as if
it were live, so UI behavior can be checked against real recorded data without a car connected.

Usage:
  PYTHONPATH=. python3 tools/replay/republish_route.py /path/to/segment/rlog.zst
"""
import sys
import time

import cereal.messaging as messaging
from tools.lib.logreader import LogReader

SERVICES = [
  'carState', 'radarState', 'modelV2', 'liveCalibration', 'selfdriveState',
  'carControl', 'carParams', 'controlsState', 'longitudinalPlan', 'deviceState',
  'pandaStates', 'onroadEvents', 'driverMonitoringState', 'driverStateV2',
  'liveParameters', 'carOutput', 'gpsLocationExternal',
]


def main(rlog_path: str):
  socks = {s: messaging.pub_sock(s) for s in SERVICES}
  lr = LogReader(rlog_path)

  last_t = None
  n_sent = 0
  for msg in lr:
    which = msg.which()
    sock = socks.get(which)
    if sock is None:
      continue

    t = msg.logMonoTime
    if last_t is not None:
      dt = (t - last_t) / 1e9
      if 0 < dt < 1.0:
        time.sleep(dt)
    last_t = t

    sock.send(msg.as_builder().to_bytes())
    n_sent += 1

  print(f"done, republished {n_sent} messages")


if __name__ == "__main__":
  if len(sys.argv) != 2:
    print(f"usage: {sys.argv[0]} /path/to/segment/rlog.zst")
    sys.exit(1)
  main(sys.argv[1])
