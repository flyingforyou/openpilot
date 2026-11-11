"""A firmer start to a lane change, most of all toward the outside of a bend.

The model makes the lane change by itself once it is given the desire, and on a straight it has the
line under the car 2.5 s later. Toward the outside of a bend it is slower and the driver ends up
helping. Over the 291 lane changes of routes 0201-0263 (line under the car / moved in the first 2 s /
share where the driver added torque the same way):

    straight (under 0.3 m/s^2 of bend)        2.5 s   0.83 m    8 %
    in a bend (>= 1.0), toward the inside     2.1 s   0.88 m    8 %
    in a bend (>= 1.0), toward the outside    3.0 s   0.60 m   53 %

The extra curvature the model itself asks for is the same in all three (0.7-0.8 m/s^2 at its peak),
so going outward it is doing the same thing against a car that sits 0.2-0.25 m toward the inside of
the bend to begin with and has to unwind the wheel to get out of it.

This adds a short push of lateral acceleration in the direction of the change while the model starts
it: BASE everywhere, plus OUTSIDE_GAIN of the bend's own lateral acceleration when the change goes
outward. It ramps in, holds until PUSH_S and fades, so it adds sideways speed early and leaves the
arrival in the new lane to the model, which sees the speed it has and plans from there. It ends early
once the line is nearly under the car. It is a curvature added above clip_curvature, so the ISO jerk
and lateral limits, the panda's and the EPAS's are what they were.

Not measured on the car: whether the model takes some of the push back by easing its own. The
replay figure (line under the car 2.95 -> 2.0 s going outward in a bend, 2.5 -> 2.2 s on a straight;
0.66 and 0.2 m/s more sideways speed at that moment) assumes it does not.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL

BASE = 0.15            # m/s^2, every lane change
OUTSIDE_GAIN = 0.30    # of the bend's lateral acceleration, when the change goes to the outside
BEND_MAX = 1.5         # m/s^2 of bend the gain is applied to
MAX_PUSH = 0.5         # m/s^2, before the driver's setting
RAMP_S = 0.3
PUSH_S = 1.2           # s from the start of the change to the start of the fade
FADE_TAU = 0.3
LINE_NEAR = 0.6        # m: the line the car is crossing is this close to its middle -> stop pushing
ROAD_TAU = 1.0         # s, low-pass on the model's curvature that stands for "the bend we are in"
MIN_V = 10.0


class LaneChangeBoost:
  def __init__(self) -> None:
    self._road_curvature = 0.0
    self._t = -1.0          # s since the change started, -1 when there is none
    self._push = 0.0        # m/s^2 chosen at the start, signed (+ right)
    self._out = 0.0         # m/s^2 being added now
    self._released = False

  def reset(self) -> None:
    self._t = -1.0
    self._push = 0.0
    self._out = 0.0
    self._released = False

  @staticmethod
  def push_for(road_curvature: float, v_ego: float, right: bool, gain: float) -> float:
    """Signed lateral acceleration (+ right) to add for a change that starts now."""
    sign = 1.0 if right else -1.0
    bend = road_curvature * v_ego ** 2          # + is a right-hand bend
    outward = bend * sign < 0.0
    push = BASE + (OUTSIDE_GAIN * min(abs(bend), BEND_MAX) if outward else 0.0)
    return sign * min(push, MAX_PUSH) * max(gain, 0.0)

  def update(self, model_curvature, model_v2, v_ego, lat_active, gain) -> float:
    """Curvature to add to the model's (0.0 outside a lane change)."""
    try:
      model_curvature = float(model_curvature)
      v_ego = float(v_ego)
      gain = float(gain)
      state = model_v2.meta.laneChangeState
      direction = model_v2.meta.laneChangeDirection
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return 0.0
    if not np.isfinite([model_curvature, v_ego, gain]).all() or not lat_active or gain <= 0.0 or v_ego < MIN_V:
      self.reset()
      return 0.0

    starting = state == log.LaneChangeState.laneChangeStarting and direction != log.LaneChangeDirection.none
    if not starting:
      # the bend is only learned outside a change: during one the curvature is the change itself
      if state == log.LaneChangeState.off:
        self._road_curvature += (model_curvature - self._road_curvature) * DT_CTRL / ROAD_TAU
      self._t = -1.0
      self._released = False
      self._out += (0.0 - self._out) * DT_CTRL / FADE_TAU
      return self._out / v_ego ** 2

    right = direction == log.LaneChangeDirection.right
    if self._t < 0.0:
      self._t = 0.0
      self._released = False
      self._push = self.push_for(self._road_curvature, v_ego, right, gain)
    else:
      self._t += DT_CTRL

    if not self._released:
      try:
        line_y = float(model_v2.laneLines[2 if right else 1].y[0])
        if np.isfinite(line_y) and abs(line_y) < LINE_NEAR:
          self._released = True
      except (AttributeError, IndexError, TypeError, ValueError):
        pass
      if self._t >= PUSH_S:
        self._released = True

    if self._released:
      self._out += (0.0 - self._out) * DT_CTRL / FADE_TAU
    else:
      self._out = self._push * min(self._t / RAMP_S, 1.0)
    return self._out / v_ego ** 2
