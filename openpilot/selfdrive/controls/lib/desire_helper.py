from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# How often to re-read AutoLaneChange, in update() calls at 20 Hz. Params.get hits disk.
PARAM_REFRESH_FRAMES = 50

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Seconds the blinker must be on before the lane change starts by itself. 0 keeps stock
    # behaviour, where only the driver's nudge starts it.
    self.auto_lane_change_delay = 0.0
    self.auto_lane_change_timer = 0.0
    self._param_frame = PARAM_REFRESH_FRAMES
    # Imported here rather than at module scope: Params needs a native library that only exists
    # after a build, and this module is worth being able to import and test without one.
    try:
      from openpilot.common.params import Params
      self._params = Params()
    except Exception:
      self._params = None

  def _read_params(self) -> None:
    self._param_frame += 1
    if self._param_frame < PARAM_REFRESH_FRAMES or self._params is None:
      return
    self._param_frame = 0
    try:
      value = self._params.get("AutoLaneChange", return_default=True)
      self.auto_lane_change_delay = max(0, int(value)) * 0.1 if value is not None else 0.0
    except Exception:
      self.auto_lane_change_delay = 0.0

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob):
    self._read_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.auto_lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # Start on the blinker alone once it has been held long enough and that side is clear.
        # The delay is the whole safety of it: a blinker is also what you use for a turn, and
        # above the 20 mph gate this is the only thing between flicking one on and the car
        # deciding to change lanes. A blindspot resets the wait rather than pausing it, so a car
        # that clears late does not hand over a countdown that has already run.
        if self.auto_lane_change_delay > 0 and not blindspot_detected:
          self.auto_lane_change_timer += DT_MDL
        else:
          self.auto_lane_change_timer = 0.0
        auto_start = (self.auto_lane_change_delay > 0 and
                      self.auto_lane_change_timer >= self.auto_lane_change_delay)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self.auto_lane_change_timer = 0.0
        elif (torque_applied or auto_start) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0
          self.auto_lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

    self.desire = log.Desire.none
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
