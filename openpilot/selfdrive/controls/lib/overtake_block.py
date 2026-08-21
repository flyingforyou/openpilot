"""Hold a lane change while a vehicle we just overtook is still level with us.

Nothing on this car reports a vehicle that is exactly abreast. The factory camera tracks the next
lane down to about 4-5 m and then loses it; the blind spot flags are named Rear and behave like it,
set on 2% of the frames where the camera still has a vehicle inside 9 m. Between those two is a
vehicle nobody can see, at the one moment moving into it would hit it.

Waiting for the blind spot to pick it up does not work, and the logs are unambiguous about why:
after the camera lost a vehicle we were passing, the flag fired on 32% of them on one route and
66% on another, and when it did fire it came a median 1.3-2.3 s later than the closing speed said
it should, matching within a second only a fifth of the time. A gate waiting on that flag stays
shut forever on the third to the half where it never comes.

So this waits on nothing. The camera hands over a last distance and a closing speed, which is
enough to say how long the vehicle needs to clear our tail, and the block runs exactly that long
and then ends by itself. It is not accurate -- but the direction it is wrong in is bounded, and
"stuck shut" is not one of the ways it can fail.

Measured over three drives it holds a side for a median 2.2 s, at most 10.2 s, and blocks
somewhere around 10-14% of the driving time on a road with this much overtaking.

There is a second, simpler half. While the camera can still see a vehicle in that lane and it is
close enough to be level with us, no inference is needed at all -- and the first version did not
hold for it, so on the road the side only locked out once the vehicle had already disappeared,
which is later than it should be. ALONGSIDE_DX covers that.

What neither half can do is a vehicle that overtakes *us* into the same place. It was never in
front to be seen, and the radar is forward-only. That case stays invisible and this does not
pretend otherwise.
"""

# Only vehicles lost close in are ones we are passing rather than ones that drove away ahead.
MAX_LOST_DX = 15.0

# We must actually be overtaking. Above this the vehicle is falling back relative to us.
MIN_CLOSING = 0.5

# Where the vehicle has to reach before a lane change is no longer into it: behind our own tail,
# with a little room. Measured from the camera's last report, which is roughly at our bumper.
CLEAR_DX = -6.0

# Nothing derived from one distance and one speed should be trusted for longer than this. 12 s
# never binds on the measured drives and 5 s cuts short 41-62% of the windows, which throws away
# the calculation the block is built on; 8 s trims the tail without doing that.
MAX_BLOCK_S = 8.0

# How long without a refresh before the camera has stopped reporting it, rather than being between
# updates. DAS_object gives each group about 6.7 Hz.
LOST_AFTER_S = 0.6

# Close enough that moving over would be moving into it, while the camera can still see it. The
# camera stops reporting the next lane at about 4-5 m, so this covers from there to just ahead --
# and no further, because a vehicle 15 m up the next lane is something you change lanes behind,
# not into. Measured cost of holding on it: 1.6-7.6% of a drive on the left, 4.3-4.8% on the
# right, against 5-18% at 15 m and 7-28% at 20 m, which would be refusing ordinary lane changes.
ALONGSIDE_DX = 10.0

GROUP_LEFT, GROUP_RIGHT = 1, 2
SIDES = {GROUP_LEFT: 'left', GROUP_RIGHT: 'right'}


class OvertakeBlock:
  """Which sides are held, and until when. Fed the factory object list every frame."""

  def __init__(self) -> None:
    self._seen: dict[tuple[int, int], tuple[float, float, float]] = {}   # key -> (t, dx, vx_rel)
    self._until: dict[str, float] = {'left': 0.0, 'right': 0.0}
    self._alongside: dict[str, bool] = {'left': False, 'right': False}

  def update(self, das_objects, t: float, v_ego: float) -> None:
    if v_ego <= 0.0:
      self.reset()
      return

    self._alongside = {'left': False, 'right': False}
    for obj in das_objects or []:
      group = int(obj.group)
      if group not in SIDES:
        continue
      dx = float(obj.dx)
      self._seen[(group, int(obj.objId))] = (t, dx, float(obj.vxRel))
      # No inference needed while it is plainly visible and level with us.
      if 0.0 < dx < ALONGSIDE_DX:
        side = SIDES[group]
        self._alongside[side] = True
        # Carry the hold across the gap between the last sighting and noticing it has gone.
        # Without this the side comes free for up to LOST_AFTER_S at exactly the wrong moment:
        # the vehicle has just drawn level, which is when the camera stops reporting it.
        self._until[side] = max(self._until[side], t + LOST_AFTER_S)

    for key in list(self._seen):
      last_t, dx, vx_rel = self._seen[key]
      if t - last_t < LOST_AFTER_S:
        continue
      del self._seen[key]

      # Lost while close, and while we were passing it. Anything else is a vehicle that left the
      # camera's view going away from us, which is not alongside anything.
      if dx > MAX_LOST_DX or vx_rel > -MIN_CLOSING:
        continue
      side = SIDES[key[0]]
      hold = min((dx - CLEAR_DX) / abs(vx_rel), MAX_BLOCK_S)
      self._until[side] = max(self._until[side], t + hold)

  def blocked(self, t: float) -> tuple[bool, bool]:
    """(left, right) -- a vehicle level with us on that side, seen or inferred."""
    return (self._alongside['left'] or t < self._until['left'],
            self._alongside['right'] or t < self._until['right'])

  def reset(self) -> None:
    self._seen.clear()
    self._until = {'left': 0.0, 'right': 0.0}
    self._alongside = {'left': False, 'right': False}
