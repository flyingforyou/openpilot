#!/usr/bin/env python3
"""Where to look on video for every cut-in the car would have called.

This runs the shipped CutInDetector itself rather than a copy of its rules, so what it prints is
what the car does -- a replay that reimplements the detector drifts from it the first time a
constant moves, and then quietly reports a car that does not exist.

Tracks are rebuilt from radarTracks with dPath recomputed off the model's lane lines, the same
pair radard.d_path uses. Output is route, segment and mm:ss into that segment, which is what it
takes to actually find the moment in a clip.

  ./cut_in_clips.py op-logs/0000007f--ed761c79a7--*
"""
import sys

import numpy as np

from openpilot.selfdrive.controls.lib.cut_in import CutInDetector
from openpilot.tools.lib.logreader import LogReader

SEG_S = 60.0


class Track:
  """Only the attributes CutInDetector reads."""
  __slots__ = ('identifier', 'dRel', 'dPath', 'vLead', 'measured', 'lane_half_width')

  def __init__(self, tid, d_rel, d_path, v_lead, measured, half):
    self.identifier, self.dRel, self.dPath = tid, d_rel, d_path
    self.vLead, self.measured, self.lane_half_width = v_lead, measured, half


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def route_of(path: str) -> str:
  return path.rstrip('/').rsplit('/', 1)[-1].rsplit('--', 1)[0]


def lane_frame(md):
  if len(md.laneLines) < 3:
    return None
  xs = np.asarray(md.laneLines[1].x, dtype=float)
  if xs.size == 0:
    return None
  return xs, np.asarray(md.laneLines[1].y, dtype=float), np.asarray(md.laneLines[2].y, dtype=float)


def main(paths):
  paths = sorted(paths, key=seg_no)
  det = CutInDetector()
  frame, v_ego, lead_d = None, 0.0, 0.0
  calls = []          # (segment, absolute t, dRel, dPath, mph)

  seg_start: dict[int, float] = {}
  for p in paths:
    seg = seg_no(p)
    last_id = -1
    for msg in LogReader(f'{p}/rlog.zst'):
      w = msg.which()
      # initData carries the *route* start time in every segment, so anchoring on the first
      # message read makes every offset route-relative. Anchor on the stream instead, and take
      # a running minimum because the file is only roughly time-ordered.
      if w not in ('carState', 'modelV2', 'radarState', 'radarTracks'):
        continue
      t = msg.logMonoTime / 1e9
      seg_start[seg] = min(seg_start.get(seg, t), t)
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'modelV2':
        frame = lane_frame(msg.modelV2)
      elif w == 'radarState':
        L = msg.radarState.leadOne
        lead_d = float(L.dRel) if L.present else 0.0
      elif w == 'radarTracks' and frame is not None:
        xs, ly, ry = frame
        tracks = {}
        for pt in msg.radarTracks.points:
          d = float(pt.dRel)
          left, right = float(np.interp(d, xs, ly)), float(np.interp(d, xs, ry))
          half = max(0.1, abs(right - left) / 2.0)
          tracks[int(pt.trackId)] = Track(int(pt.trackId), d, float(pt.yRel) + (left + right) / 2.0,
                                          float(pt.vLead), bool(pt.measured), half)
        tid = det.update(tracks, t, v_ego, lead_d)
        # report the moment it starts, not every frame it stays true
        if tid >= 0 and tid != last_id:
          tr = tracks[tid]
          calls.append((seg, t, tr.dRel, tr.dPath, v_ego * 2.23694))
        last_id = tid

  route = route_of(paths[0])
  print(f"route {route}   {len(paths)} segments\n")
  if not calls:
    print("  no cut-ins called")
    return

  print(f"  {'segment':>9} {'at':>7}   {'dRel':>6} {'dPath':>7} {'mph':>5}   video")
  for (seg, abs_t, d_rel, d_path, mph) in calls:
    off = abs_t - seg_start[seg]
    # a merge is worth watching from a few seconds before the call
    start = max(0.0, off - 4.0)
    where = f"{route}--{seg}  seek {start:.0f}s"
    print(f"  {seg:>9} {int(off // 60):01d}:{off % 60:04.1f}   {d_rel:5.1f}m {d_path:+6.2f}m {mph:5.0f}   {where}")

  segs = sorted({c[0] for c in calls})
  print(f"\n  {len(calls)} cut-ins across segments {segs}")
  print(f"\n  on the device the clips are /data/media/0/realdata/{route}--<segment>/")
  print("  qcamera.ts is the small one and plays anywhere; fcamera.hevc is full resolution.")


if __name__ == '__main__':
  main(sys.argv[1:])
