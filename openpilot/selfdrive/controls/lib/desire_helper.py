from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.lane_change_guards import lane_exists, side_lead_unsafe

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# How often to re-read AutoLaneChange, in update() calls at 20 Hz. Params.get hits disk.
PARAM_REFRESH_FRAMES = 50

# Seconds still to wait after the blindspot clears. A full restart of the configured delay
# punishes a long setting for a car that was never in the way for long; leaving the elapsed time
# alone would let a car that has only just moved out hand over a countdown that already ran.
BLINDSPOT_REARM_S = 1.0

# How long after committing a lane change the blindspot can still call it off.
#
# Once laneChangeStarting is entered there was no way out of it: the state only left when the
# model said the move was finished, which on route 00000087 segment 13 took 5.6 s. Nothing was
# re-checked in that time -- not the blindspot, not the target lane's nearest car -- so a vehicle
# arriving alongside after the start had no way to stop anything. Traced from a real crossing:
# the driver nudged right while a car in the right lane was moving left into the lane being
# vacated, and the guard that would have caught it disappeared precisely because that car was
# crossing (get_side_leads looks in a 1.8-5.4 m band, which a crossing car leaves).
#
# Aborting late would be its own hazard -- reversing out of a change already half made is worse
# than finishing it -- so this only covers the beginning. At 60 mph a lane change takes 4-5 s to
# cover 3.6 m, so one second in the car has moved about 0.7 m and is still inside its own lane:
# returning to centre from there costs nothing. After that the move stands.
LANE_CHANGE_ABORT_S = 1.0

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
    # A brake press means the driver is reacting to something the car may not have seen, and one
    # automatic change per blinker is enough -- neither should be undone by the blinker simply
    # staying on. Both clear when the state machine leaves the lane change entirely.
    self.auto_blocked_by_brake = False
    self.auto_already_changed = False
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

  @staticmethod
  def _occupied(carstate, overtaking, direction) -> bool:
    """Is that side occupied -- seen by the blindspot, or known to be from an overtake."""
    if direction == LaneChangeDirection.left:
      return bool(carstate.leftBlindspot or overtaking[0])
    if direction == LaneChangeDirection.right:
      return bool(carstate.rightBlindspot or overtaking[1])
    return False

  def _reset_auto(self) -> None:
    self.auto_lane_change_timer = 0.0
    self.auto_blocked_by_brake = False
    self.auto_already_changed = False

  def update(self, carstate, lateral_active, lane_change_prob, lane_side=None, side_lead=None,
             overtaking=(False, False)):
    """lane_side and side_lead describe the side the blinker points at, or are None when the
    caller has nothing to say -- in which case those gates simply do not apply.

    lane_side is (outer_line_prob, road_edge_distance_m); side_lead is that lane's nearest
    radar lead. Both are looked up by direction by the caller, which is the only place that
    knows left from right before this decides the direction itself.

    `overtaking` is (left, right) from OvertakeBlock: a vehicle we have just passed that nothing
    can currently see but which has not yet had time to fall behind us. It is treated exactly as
    a blindspot rather than as its own concept, so both the start gate and the early abort get it
    without either having to learn a second way of being told the lane is occupied.
    """
    self._read_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self._reset_auto()
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # A fresh blinker is a fresh intent: whatever blocked the last one does not carry over.
        self._reset_auto()
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        # Braking with the blinker on means the driver is reacting to something; whatever it is,
        # the car should not pick that moment to change lanes by itself.
        if getattr(carstate, 'brakePressed', False):
          self.auto_blocked_by_brake = True

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((self._occupied(carstate, overtaking, LaneChangeDirection.left) and
                               self.lane_change_direction == LaneChangeDirection.left) or
                              (self._occupied(carstate, overtaking, LaneChangeDirection.right) and
                               self.lane_change_direction == LaneChangeDirection.right))

        # Start on the blinker alone once it has been held long enough and everything the car
        # can see about that side says go. The delay is the core of it: a blinker is also what
        # you use for a turn, and above the speed gate it is the only thing between flicking one
        # on and the car deciding to move over.
        self.auto_lane_change_timer += DT_MDL
        if blindspot_detected:
          # Leave at least BLINDSPOT_REARM_S on the clock rather than restarting the whole wait.
          self.auto_lane_change_timer = min(self.auto_lane_change_timer,
                                            self.auto_lane_change_delay - BLINDSPOT_REARM_S)

        # Only the automatic start has to satisfy these. A nudge is the driver saying they have
        # looked, and second-guessing that with the model's opinion of the lane is not this
        # feature's job.
        room_that_side = lane_side is None or lane_exists(*lane_side)
        side_clear = side_lead is None or not side_lead_unsafe(side_lead, v_ego)
        auto_start = (self.auto_lane_change_delay > 0 and
                      self.auto_lane_change_timer >= self.auto_lane_change_delay and
                      not self.auto_blocked_by_brake and not self.auto_already_changed and
                      room_that_side and side_clear)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self._reset_auto()
        elif (torque_applied or auto_start) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0
          self.auto_lane_change_timer = 0.0
          if auto_start:
            self.auto_already_changed = True

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        # Someone arrived alongside after we committed. Early enough that the car has barely
        # moved, this goes back to waiting rather than carrying on into them.
        started_blindspot = self._occupied(carstate, overtaking, self.lane_change_direction)
        if started_blindspot and self.lane_change_timer < LANE_CHANGE_ABORT_S:
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_timer = 0.0
          # Do not let the abort hand back a countdown that has already run: the automatic start
          # has to earn its delay again, or it would re-commit the moment the blindspot blinks.
          self.auto_lane_change_timer = 0.0

        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
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
