#!/usr/bin/env python3
"""Extract a short window of a recorded route into a compact file for UI replay.

republish_route.py streams straight out of LogReader, which holds a whole segment of decoded
capnp objects in memory. Together with the UI process and the ffmpeg recorder that was enough
to push a 3.5GB device into rebooting, so the read and the replay are split in two: this script
does the heavy read once and writes only the raw bytes the UI actually consumes, and
republish_route.py then plays that small file back with almost no memory of its own.

  PYTHONPATH=. python3 tools/replay/extract_ui_window.py <rlog.zst> <out.pkl> [duration_s]

Picks a window where openpilot is engaged and radarState.leadOne.status is true. Both matter:
the mici renderer hides the lane lines, the path and the lead chevron entirely while
disengaged, so a disengaged window replays as a blank screen.
"""
import pickle
import sys

from tools.lib.logreader import LogReader

SERVICES = [
  'carState', 'radarState', 'modelV2', 'liveCalibration', 'selfdriveState',
  'carControl', 'carParams', 'controlsState', 'longitudinalPlan', 'deviceState',
  'pandaStates', 'onroadEvents', 'driverMonitoringState', 'liveParameters', 'carOutput',
  'roadCameraState', 'wideRoadCameraState',
]


def main(rlog_path: str, out_path: str, duration_s: float):
  # First pass: find where openpilot is engaged AND a lead is present, so the window shows
  # the lane lines, the chevron and its R/V label rather than a blank disengaged screen.
  lead_start = None
  engaged = False
  for msg in LogReader(rlog_path):
    which = msg.which()
    if which == 'selfdriveState':
      engaged = msg.selfdriveState.enabled
    elif which == 'radarState' and engaged and msg.radarState.leadOne.status:
      lead_start = msg.logMonoTime
      break

  if lead_start is None:
    print("no engaged-with-lead window in this segment, starting from the beginning")

  out = []
  t_start = None
  for msg in LogReader(rlog_path):
    which = msg.which()
    if which not in SERVICES:
      continue

    t = msg.logMonoTime
    if lead_start is not None and t < lead_start:
      continue
    if t_start is None:
      t_start = t
    if (t - t_start) / 1e9 > duration_s:
      break

    out.append((t, which, msg.as_builder().to_bytes()))

  with open(out_path, 'wb') as f:
    pickle.dump(out, f, protocol=pickle.HIGHEST_PROTOCOL)

  span = (out[-1][0] - out[0][0]) / 1e9 if out else 0.0
  print(f"wrote {len(out)} messages spanning {span:.1f}s to {out_path}")


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print(f"usage: {sys.argv[0]} <rlog.zst> <out.pkl> [duration_s]")
    sys.exit(1)
  main(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 15.0)
