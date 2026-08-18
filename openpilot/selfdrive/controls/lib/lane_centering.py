"""Nudge the model's path back toward the middle of the lane it is in.

The end-to-end model is not asked to stay centred, and on a curve it will happily take the
line it thinks is best -- which reads from the seat as drifting wide on entry and cutting the
apex. This adds a small curvature correction toward the midpoint of the two lane lines, so the
model still decides where to go and this only trims how far off centre it is allowed to sit.

The correction is deliberately weak (a few degrees at the wheel at most) and it is the first
thing to give up: every gate below returns the model's own curvature untouched rather than
guessing, and the correction is faded out rather than dropped so nothing steps in the steering.

Ported from StarPilot's selfdrive/controls/lib/lane_centering.py, thresholds unchanged --
they are what that fork tuned, and retuning them blind here would only lose that.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

# Below this there are no lane lines worth trusting and the model is manoeuvring anyway.
MIN_V_EGO = 5.0

# What counts as a lane line worth steering to.
MIN_LANE_PROB = 0.6
MAX_LANE_STD = 0.3
MIN_LANE_WIDTH = 2.6
MAX_LANE_WIDTH = 4.8

# How far off centre the driver may deliberately sit, and how close that is ever allowed to
# put the car to a line.
MAX_OFFSET = 0.3
MIN_CENTER_TO_LINE = 1.1

# The correction is a curvature, capped before the gain so the gain stays a plain fraction of
# the geometrically required amount. 0.004 * 0.30 is ~0.75 m/s^2 of lateral at 90 km/h.
MAX_RAW_CORRECTION = 0.004
MAX_GAIN = 0.30

SMOOTH_TAU = 0.4
SIGNAL_RELEASE_TAU = 0.20
CONFIDENCE_RELEASE_TAU = 0.20

# Centimetres of offset the model is simply allowed to have.
CENTER_ERROR_DEADBAND = 0.08

# How the model's own confidence buys it the right to leave the centre. Below the start it is
# always corrected; past full it keeps whatever share of the deviation e2e_authority grants.
E2E_MAX_PATH_STD = 0.35
E2E_BREAK_IN_START = 0.15
E2E_BREAK_IN_FULL = 0.50


class LaneCenteringController:
  def __init__(self) -> None:
    self._correction = 0.0

  def reset(self) -> None:
    self._correction = 0.0

  def update(self, model_curvature, model_v2, v_ego, enabled, offset, e2e_authority, lat_active,
             model_valid, pause_on_signal=False, turn_signal_active=False) -> float:
    """Return the model's curvature with the centring trim added, or unchanged."""
    model_curvature = float(model_curvature)

    try:
      v_ego = float(v_ego)
      offset = float(offset)
      e2e_authority = float(e2e_authority)
    except (TypeError, ValueError):
      self.reset()
      return model_curvature

    if not np.isfinite([v_ego, offset, e2e_authority]).all():
      self.reset()
      return model_curvature

    if not model_valid or not enabled or not lat_active or v_ego < MIN_V_EGO:
      self.reset()
      return model_curvature

    # A signalled move is the driver's, not a deviation to correct -- fade out instead of
    # fighting it, and keep the faded value so the release is smooth on both sides.
    if pause_on_signal and turn_signal_active:
      self._correction = float(smooth_value(0.0, self._correction, SIGNAL_RELEASE_TAU, dt=DT_CTRL))
      return model_curvature + self._correction

    try:
      if model_v2.meta.laneChangeState != log.LaneChangeState.off:
        self.reset()
        return model_curvature
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return model_curvature

    valid, raw_correction = self._raw_correction(
      model_v2,
      v_ego,
      float(np.clip(offset, -MAX_OFFSET, MAX_OFFSET)),
      float(np.clip(e2e_authority, 0.0, 1.0)),
    )
    if not valid:
      self._correction = float(smooth_value(0.0, self._correction, CONFIDENCE_RELEASE_TAU, dt=DT_CTRL))
      return model_curvature + self._correction

    target = float(np.clip(raw_correction, -MAX_RAW_CORRECTION, MAX_RAW_CORRECTION)) * MAX_GAIN
    self._correction = float(smooth_value(target, self._correction, SMOOTH_TAU, dt=DT_CTRL))
    return model_curvature + self._correction

  @staticmethod
  def _valid_path(x, y) -> bool:
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    return bool(x[0] <= distance <= x[-1])

  def _raw_correction(self, model_v2, v_ego: float, offset: float, e2e_authority: float) -> tuple[bool, float]:
    """Curvature that would put the model's path on the lane centre one lookahead ahead."""
    try:
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        return False, 0.0
      # 1 and 2 are the two lines of the lane the car is in; 0 and 3 are the adjacent ones.
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        return False, 0.0
      if np.any(probs[[1, 2]] < MIN_LANE_PROB) or np.any(probs[[1, 2]] > 1.0):
        return False, 0.0
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > MAX_LANE_STD):
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      if not (self._valid_path(left_x, left_y) and self._valid_path(right_x, right_y) and self._valid_path(pos_x, pos_y)):
        return False, 0.0

      # Look further ahead the faster you go, so the trim is a heading change rather than a
      # sideways shove.
      lookahead = float(np.clip(v_ego, 8.0, 35.0))
      if not all(self._covers(x, lookahead) for x in (left_x, right_x, pos_x)):
        return False, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      if not MIN_LANE_WIDTH <= width <= MAX_LANE_WIDTH:
        return False, 0.0

      # A deliberate offset never gets to put the car nearer a line than MIN_CENTER_TO_LINE,
      # so asking for 30cm in a narrow lane quietly gives back less.
      max_safe_offset = min(MAX_OFFSET, max(0.0, width * 0.5 - MIN_CENTER_TO_LINE))
      target_y = 0.5 * (left + right) + float(np.clip(offset, -max_safe_offset, max_safe_offset))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
      error = target_y - model_y
      error_abs = abs(error)
      if error_abs <= CENTER_ERROR_DEADBAND:
        error = 0.0
      else:
        error = np.copysign(error_abs - CENTER_ERROR_DEADBAND, error)

      # A confident model that is a long way off centre is usually going somewhere on purpose
      # -- around a parked car, or its own line through a bend. e2e_authority is how much of
      # that intent to honour: 0 corrects everything, 1 leaves big deviations alone.
      try:
        pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)
        if self._valid_path(pos_x, pos_y_std):
          path_std = float(np.interp(lookahead, pos_x, pos_y_std))
          if 0.0 <= path_std <= E2E_MAX_PATH_STD:
            break_in = np.clip(
              (error_abs - E2E_BREAK_IN_START) / (E2E_BREAK_IN_FULL - E2E_BREAK_IN_START),
              0.0,
              1.0,
            )
            error *= 1.0 - e2e_authority * float(break_in)
      except (AttributeError, TypeError, ValueError):
        pass

      # y = k x^2 / 2 over the lookahead, solved for k.
      return True, float(2.0 * error / lookahead ** 2)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0
