#!/usr/bin/env python3
"""Could a merge have been seen coming, from this car's forward radar alone?

The planner reacts to a cut-in only once the gap has already collapsed, because that is the first
moment anything tells it. CarrotPilot answers this earlier by watching each radar track move
sideways towards the lane rather than waiting for it to arrive in front -- but its detector is
built on corner radars, which this car does not have.

So this replays a front-radar-only version over recorded drives before any of it is written into
the planner. It rebuilds dPath the way radard does, from the model's own lane lines, projects
each track sideways at its measured yvRel, and asks which tracks it would have called. Every
call is then scored against what actually happened to the followed gap a moment later, which is
the only thing that separates a detector from a wish.

  ./cut_in_detect_replay.py op-logs/0000007f--ed761c79a7--*
"""
import sys
from collections import defaultdict, deque

import numpy as np

from openpilot.tools.lib.logreader import LogReader

# Deliberately conservative: this fires a brake, so a miss costs comfort and a false call costs
# trust. Every one of these is tighter than CarrotPilot's equivalent.
MIN_DREL, MAX_DREL = 6.0, 50.0     # carrot: 5..50
MIN_VLEAD = 4.0                    # moving traffic, not roadside furniture
GAP_COLLAPSE_M = 6.0               # what counts as the followed gap actually collapsing
LOOKAHEAD_S = 8.0                  # how long after a call to look for that collapse

# The first version of this asked whether a track would be *inside* the lane within 1.5s, and so
# could only ever fire once the car was nearly there -- median warning 0.3s, which is no warning.
# Measured against the drive, a merging car is visible for a median 13.4s beforehand and is first
# seen outside the lane every time, so the ceiling is not the sensor. What it takes is asking for
# sustained progress towards the lane rather than arrival in it.
RATE_WINDOW_S = 1.0                # |dPath| closing is measured over this, not frame to frame
MIN_CLOSING = 0.15                 # m/s towards the lane centre, sustained
MAX_LANE_ENTRY_S = 4.0             # ...and on track to reach our lane edge within this
OUTER_LANES = 2.5                  # ignore anything further out than this many half-widths
CONFIRM_FRAMES = 10                # 0.5s at 20Hz


def seg_no(path: str) -> int:
  return int(path.rstrip('/').rsplit('--', 1)[-1])


def lane_frame(md):
  """(xs, left_ys, right_ys) from the model, or None -- the same pair radard.d_path uses."""
  if len(md.laneLines) < 3:
    return None
  xs = np.asarray(md.laneLines[1].x, dtype=float)
  if xs.size == 0:
    return None
  return xs, np.asarray(md.laneLines[1].y, dtype=float), np.asarray(md.laneLines[2].y, dtype=float)


def d_path(frame, d_rel, y_rel):
  xs, ly, ry = frame
  left = float(np.interp(d_rel, xs, ly))
  right = float(np.interp(d_rel, xs, ry))
  half = max(0.1, abs(right - left) / 2.0)
  return y_rel + (left + right) / 2.0, half


def main(paths):
  paths = sorted(paths, key=seg_no)
  frame = None
  v_ego = 0.0
  lead_d = np.nan
  t0 = None

  hist = defaultdict(lambda: {'n': 0, 'dp': deque()})   # track id -> confirmation + |dPath| trail
  calls = []          # (t, track id, dRel, dPath, inward, projected, lead dRel then)
  lead_trace = []     # (t, leadOne dRel)

  for p in paths:
    for msg in LogReader(f'{p}/rlog.zst'):
      t = msg.logMonoTime / 1e9
      t0 = t if t0 is None else t0
      w = msg.which()
      if w == 'carState':
        v_ego = float(msg.carState.vEgo)
      elif w == 'modelV2':
        frame = lane_frame(msg.modelV2)
      elif w == 'radarState':
        lead = msg.radarState.leadOne
        lead_d = float(lead.dRel) if lead.present else np.nan
        lead_trace.append((t - t0, lead_d))
      elif w == 'radarTracks' and frame is not None and v_ego > 5.0:
        seen = set()
        for pt in msg.radarTracks.points:
          tid = int(pt.trackId)
          seen.add(tid)
          st = hist[tid]
          d_rel, y_rel = float(pt.dRel), float(pt.yRel)
          if not (pt.measured and MIN_DREL < d_rel < MAX_DREL and float(pt.vLead) > MIN_VLEAD):
            st['n'] = 0
            st['dp'].clear()
            continue
          dp, half = d_path(frame, d_rel, y_rel)

          # |dPath| over the last second. Measuring the rate across a window rather than from
          # yvRel frame to frame is what makes a slow merge visible: the instantaneous lateral
          # rate of a car easing over one lane is inside the radar's own noise.
          trail = st['dp']
          trail.append((t - t0, abs(dp)))
          while trail and (t - t0) - trail[0][0] > RATE_WINDOW_S:
            trail.popleft()
          if len(trail) < 3 or (t - t0) - trail[0][0] < RATE_WINDOW_S * 0.5:
            st['n'] = 0
            continue
          span = (t - t0) - trail[0][0]
          closing = (trail[0][1] - abs(dp)) / span          # + means coming towards our lane

          to_edge = abs(dp) - half
          entry_s = to_edge / closing if closing > 1e-3 else 1e3
          # only interesting if it would end up ahead of what we are already following
          relevant = not np.isfinite(lead_d) or d_rel < lead_d - 1.0
          if (half < abs(dp) < OUTER_LANES * half and closing > MIN_CLOSING
              and entry_s < MAX_LANE_ENTRY_S and relevant):
            st['n'] += 1
            if st['n'] == CONFIRM_FRAMES:
              calls.append((t - t0, tid, d_rel, dp, closing, entry_s, lead_d))
          else:
            st['n'] = 0
        for tid in list(hist):
          if tid not in seen:
            hist.pop(tid, None)

  if not calls:
    print("no cut-ins called -- either the gates are too tight or this drive had none")
    return
  lt = np.array(lead_trace)

  print(f"{len(lead_trace)} lead frames, {len(calls)} cut-in calls\n")
  head = f"  {'when':>8} {'id':>6} {'dRel':>6} {'dPath':>7} {'closing':>8} {'enters in':>10}"
  print(f"{head} {'lead was':>9}   {'gap collapsed after':>20}")

  leads, no_follow = [], 0
  for (ts, tid, d_rel, dp, closing, entry_s, ld) in calls:
    # did the gap we actually follow collapse within the lookahead? that is the merge landing
    win = (lt[:, 0] >= ts) & (lt[:, 0] <= ts + LOOKAHEAD_S)
    seg = lt[win]
    drop = None
    if seg.size and np.isfinite(ld):
      hit = np.where(seg[:, 1] < ld - GAP_COLLAPSE_M)[0]
      if hit.size:
        drop = seg[hit[0], 0] - ts
    if drop is None:
      no_follow += 1
    else:
      leads.append(drop)
    verdict = 'never' if drop is None else f'{drop:.1f}s'
    left = f"  {ts/60:7.1f}m {tid:6} {d_rel:5.1f}m {dp:+6.2f}m {closing:+7.2f} {entry_s:9.1f}s"
    print(f"{left} {ld:8.1f}m   {verdict:>20}")

  print(f"\n  followed by a real gap collapse: {len(leads)} of {len(calls)}")
  if leads:
    a = np.array(leads)
    spread = f"p10 {np.percentile(a,10):.1f}s  max {a.max():.1f}s ahead of the collapse"
    print(f"  warning given                    median {np.median(a):.1f}s  {spread}")
  cost = f"({100*no_follow/len(calls):.0f}% -- these are the cost of the feature)"
  print(f"  called with nothing following it: {no_follow}   {cost}")
  mins = lt[-1, 0] / 60.0 if lt.size else 0.0
  if mins:
    print(f"  call rate                        {len(calls)/mins:.2f} per minute over {mins:.0f} min")


if __name__ == '__main__':
  main(sys.argv[1:])
