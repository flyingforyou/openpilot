"""Where the model aims in a bend: nearer the middle of the lane, by moving what it is shown.

The model picks a line toward the inside of every bend -- 0.22 m at motorway speed over routes
0240-0263, a third of the time more than 0.3 m. Adding curvature on top of its output to push the
car back out (LaneCenteringCurveHold, 11/11) made the bends unsteady: the model keeps steering back
to its own line, lateral accel swing +20 %, jerk +18 % on the road. Slower versions of that trim
lose the swing in the closed-loop replay but still add 10-16 % jerk above 1 Hz, because a car held
off the model's line gets a stream of small corrections whatever is holding it there.

So nothing is added to the output. The frames the model sees are warped as if the camera stood a
little further toward the inside than it does (a ground-plane homography folded into the warp
matrix modeld builds anyway), and the model, on its own line and unopposed, puts the car that much
further out. In the replay (tools/tesla_analysis/latsim.py, five bends): inside 0.26 -> 0.10 m,
lateral accel swing x0.98, jerk x1.00.

The shift is a slow integral of how far the lane centre is beside the car beyond a deadband --
slow on purpose: a person lets a moment's drift go and answers only a steady lean. It lets go on a
blinker or a lane change, and is bounded. A pure sideways shift leaves the horizon where it is, so
the calibration built on the same model's pose is not disturbed.

The model's lean comes and goes with the bend; an integral does not. Left to itself it was still
pushing 0.07-0.30 m outward when each of 11 bends on 11/15 (routes 0272-0277) opened out, and the
car ran 0.13-0.60 m toward the outside line after it; between the two halves of an S it pushed
into the inside of the second. So the shift is only allowed toward the outside of the bend the
model's own plan holds from now to AIM_LOOKAHEAD s on, and no further than that bend is hard
(AIM_PER_LAT): it comes off as the exit comes into the plan, and is gone before a bend the other
way. On a straight there is none.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.common.transformations.model import calib_from_medmodel, calib_from_sbigmodel
from openpilot.common.transformations.orientation import rot_from_euler

AIM_GAIN = 0.6            # 1/s: m of shift per second, per m of lane centre beside the car
AIM_DEADBAND = 0.08       # m beside the car that is left alone
AIM_MAX = 0.30            # m
AIM_PER_LAT = 0.20        # m of shift allowed per m/s2 of bend; the model leans 0.3 m at 1.45 m/s2 (0272, 0273)
AIM_LOOKAHEAD = 2.0       # s of the model's plan the bend has to last; the car is 1-2 s behind the shift
AIM_BOUND_TAU = 0.4       # s, how fast the shift follows that bound down
AIM_LEAK_TAU = 20.0       # s
AIM_RELEASE_TAU = 1.5     # s, on a blinker, a lane change, without lines or when not steering
AIM_MIN_V = 10.0          # m/s
AIM_MIN_PROB = 0.5
AIM_LANE_WIDTH = (2.6, 4.6)   # m; outside it the two lines are not one lane
DEFAULT_HEIGHT = 1.22     # m, camera above the road when the calibration has not said


def shifted_warp(device_from_calib_euler, intrinsics, bigmodel_frame: bool, shift: float, height: float) -> np.ndarray:
  """get_warp_matrix for a camera `shift` m to the right of where it is (exact for the road surface)."""
  calib_from_model = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ rot_from_euler(device_from_calib_euler)
  # calibrated frame: x forward, y right, z down; the road is the plane z = height
  ground = np.eye(3)
  ground[1, 2] = shift / max(height, 0.5)
  return camera_from_calib @ ground @ calib_from_model


def to_road_frame(model_v2, shift: float) -> None:
  """Put what the model saw through the shift back where it is on the road, in place.

  A point the model reports at y lies at y + shift (+ right). That holds for everything out in the
  world -- lane lines, road edges, leads -- and those are moved. It does not hold for the model's
  own path (position): that is where the car will go from where it is, which no warp changes, so it
  stays. The lane centring trim (lane_centering.py) was tuned and replayed against the lines as the
  model saw them, and takes the shift back off (meta.laneAimShift) to keep doing exactly that.
  """
  model_v2.meta.laneAimShift = float(shift)
  if shift == 0.0:
    return
  for group in (model_v2.laneLines, model_v2.roadEdges, model_v2.leadsV3):
    for item in group:
      item.y = [float(y) + shift for y in item.y]


def lat_accel_ahead(model_v2) -> tuple[float, float]:
  """The least and the most lateral accel (m/s2, + in a bend to the right) in the model's plan from now to AIM_LOOKAHEAD s."""
  rate, vel = model_v2.orientationRate, model_v2.velocity
  lat = [float(r) * float(v) for t, r, v in zip(rate.t, rate.z, vel.x) if t <= AIM_LOOKAHEAD]
  return (min(lat), max(lat)) if lat else (0.0, 0.0)


class LaneAim:
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.shift = 0.0        # m to the right, as shown to the model on the next frame

  def reset(self) -> None:
    self.shift = 0.0

  def update(self, left_y: float, right_y: float, left_prob: float, right_prob: float, v_ego: float,
             enabled: bool, lat_active: bool, turn_signal: bool, lane_change: bool, lat_ahead: tuple[float, float]) -> float:
    """left_y, right_y: the lane lines beside the car as the model, shown the current shift, reports them
    (on the road they are `shift` further right; see to_road_frame). lat_ahead: lat_accel_ahead()."""
    if not enabled:
      self.shift = 0.0
      return self.shift
    width = right_y - left_y
    usable = (lat_active and not turn_signal and not lane_change and v_ego > AIM_MIN_V
              and min(left_prob, right_prob) > AIM_MIN_PROB and AIM_LANE_WIDTH[0] < width < AIM_LANE_WIDTH[1])
    if not usable:
      self.shift -= self.shift * min(1.0, self.dt / AIM_RELEASE_TAU)
      return self.shift
    # where the lane centre really is beside the car: what was reported, less what was shown
    centre = 0.5 * (left_y + right_y) + self.shift
    error = 0.0 if abs(centre) <= AIM_DEADBAND else float(np.sign(centre) * (abs(centre) - AIM_DEADBAND))
    held = self.shift
    self.shift -= AIM_GAIN * error * self.dt
    self.shift -= self.shift * self.dt / AIM_LEAK_TAU
    # a bend to the right (+) is answered with a shift to the right (+), which sends the car left, to its outside
    lowest, highest = lat_ahead
    high = min(AIM_MAX, AIM_PER_LAT * max(0.0, lowest))
    low = -min(AIM_MAX, AIM_PER_LAT * max(0.0, -highest))
    # a shift beyond what the bend now allows comes down to it, whatever the integral asks; it is not cut
    ease = min(1.0, self.dt / AIM_BOUND_TAU)
    high = max(high, held + (high - held) * ease)
    low = min(low, held + (low - held) * ease)
    self.shift = float(np.clip(self.shift, low, high))
    return self.shift
