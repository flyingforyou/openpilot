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
slow on purpose: a person lets a moment's drift go and answers only a steady lean. It leaks away
on a straight, lets go on a blinker or a lane change, and is bounded. A pure sideways shift leaves
the horizon where it is, so the calibration built on the same model's pose is not disturbed.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.common.transformations.model import calib_from_medmodel, calib_from_sbigmodel
from openpilot.common.transformations.orientation import rot_from_euler

AIM_GAIN = 0.6            # 1/s: m of shift per second, per m of lane centre beside the car
AIM_DEADBAND = 0.08       # m beside the car that is left alone
AIM_MAX = 0.30            # m
AIM_LEAK_TAU = 20.0       # s, so that what a bend needed does not stay on the straight after it
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


class LaneAim:
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.shift = 0.0        # m to the right, as shown to the model on the next frame

  def reset(self) -> None:
    self.shift = 0.0

  def update(self, left_y: float, right_y: float, left_prob: float, right_prob: float, v_ego: float,
             enabled: bool, lat_active: bool, turn_signal: bool, lane_change: bool) -> float:
    """left_y, right_y: the lane lines beside the car as the model, shown the current shift, reports them
    (on the road they are `shift` further right; see to_road_frame)."""
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
    self.shift -= AIM_GAIN * error * self.dt
    self.shift -= self.shift * self.dt / AIM_LEAK_TAU
    self.shift = float(np.clip(self.shift, -AIM_MAX, AIM_MAX))
    return self.shift
