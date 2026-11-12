"""See the car in front of the lead, and expect the lead to brake before it does.

The planner reacts to the lead's own deceleration, which the radar reports about half a second
after it starts; the brake then arrives 0.75 s after the lead's. A driver watches the car beyond:
when that one slows, the car in between is about to.

The forward radar sees under and around the lead more often than not. Over routes 0250-0266 a
second track on our path beyond the lead was present in 65 % of following frames (54 % above
90 km/h, 76 % in town), and at 127 of the 182 moments the lead began braking past 1 m/s^2. Where
that track had been held for two seconds, it had already lost speed first in 60 of 80, a median
0.7 s ahead of the lead and more than a second in 25.

So the lead is given the deceleration it will need: the constant rate that leaves it KEEP_HEADWAY_S
behind the car ahead of it, at that car's speed. Only the lead's acceleration estimate is lowered,
never raised, never past MAX_DECEL, and only once the need is past ENGAGE for HOLD_FRAMES -- so the
planner starts easing off, and everything else about the lead (distance, speed, which track it is)
is untouched. If the call is wrong the cost is an early lift behind a lead that did not brake.
"""
import numpy as np

MIN_AHEAD = 6.0           # m beyond the lead: nearer is the lead's own second reflection
MAX_AHEAD = 90.0
MAX_PATH_OFFSET = 1.5     # m from the model's path at the track's distance
MIN_AGE = 20              # frames the track has existed
MIN_CLOSING = 1.0         # m/s the lead is faster than the car ahead of it
KEEP_HEADWAY_S = 1.0      # what the lead is assumed to keep behind that car
KEEP_STANDSTILL = 4.0     # m
PREDICT_S = 1.0           # the car ahead's own braking, carried this far forward
ENGAGE = -0.8             # m/s^2: a need gentler than this is not worth acting on (-0.5: 3.9 shallow
                          # brake applications a minute in steady following against 3.4 without; -0.8: 3.6)
MAX_DECEL_BP = (15.0, 25.0)   # m/s, lead speed
MAX_DECEL_V = (-1.5, -0.8)    # m/s^2: the most the lead is ever assumed to be about to brake
HOLD_FRAMES = 3
# A lead that is itself pulling away is not about to brake for the car beyond it: in a queue moving
# off, that car is always the slower one, and expecting the lead to brake made launches sluggish
# (speed 8 s after the lead moves 37.7 -> 36.4 km/h over the 12 launch scenes).
MAX_LEAD_ACCEL = 0.3      # m/s^2
MIN_LEAD_V = 5.0          # m/s
RELEASE_RATE = 2.0        # m/s^3 back up to the measured value


def find_ahead(tracks: dict, lead_track_id: int, lead_d_rel: float):
  best = None
  for tid, t in tracks.items():
    if tid == lead_track_id or t.cnt < MIN_AGE or abs(t.pathOffset) > MAX_PATH_OFFSET:
      continue
    if lead_d_rel + MIN_AHEAD < t.dRel < lead_d_rel + MAX_AHEAD and (best is None or t.dRel < best.dRel):
      best = t
  return best


def needed_lead_accel(v_lead: float, d_between: float, v_ahead: float, a_ahead: float) -> float:
  """The constant deceleration that leaves the lead KEEP_HEADWAY_S behind the car ahead of it."""
  v_ahead = max(v_ahead + min(a_ahead, 0.0) * PREDICT_S, 0.0)
  closing = v_lead - v_ahead
  if closing < MIN_CLOSING:
    return 0.0
  room = d_between - (KEEP_STANDSTILL + KEEP_HEADWAY_S * v_ahead)
  floor = float(np.interp(v_lead, MAX_DECEL_BP, MAX_DECEL_V))
  return max(-(closing * closing) / (2.0 * max(room, 1.0)), floor)


class LeadAhead:
  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.count = 0
    self.a = None        # the lowered value while active
    self.track_id = -1

  def update(self, tracks: dict, lead: dict) -> float | None:
    """Returns the lead acceleration to use instead of the measured one, or None."""
    if not lead.get("present") or not lead.get("radar"):
      self.reset()
      return None
    measured = float(lead["aLeadK"])
    ahead = find_ahead(tracks, int(lead["radarTrackId"]), float(lead["dRel"]))
    need = 0.0
    if ahead is not None and measured < MAX_LEAD_ACCEL and float(lead["vLead"]) > MIN_LEAD_V:
      need = needed_lead_accel(float(lead["vLead"]), ahead.dRel - float(lead["dRel"]), float(ahead.vLead), float(ahead.aLeadK))
      self.track_id = ahead.identifier
    self.count = self.count + 1 if need < ENGAGE else 0
    want = min(measured, need) if self.count >= HOLD_FRAMES else measured
    if self.a is None:
      if want >= measured:
        return None
      self.a = want
    else:
      # down at once, back up at RELEASE_RATE
      self.a = min(want, self.a + RELEASE_RATE * self.dt)
      if self.a >= measured:
        self.a = None
        return None
    return self.a
