import time

import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.common.filter_simple import BounceFilter, FirstOrderFilter
from openpilot.cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds
METER_TO_FOOT = 3.28084

# Gap popup, styled to match the lane change alerts in AlertRenderer.
GAP_ALERT_MARGIN = 18                # AlertRenderer.ALERT_MARGIN
GAP_ALERT_BG_HEIGHT_FRAC = 0.583     # AlertRenderer's "small alert" height
GAP_ALERT_FONT_SIZE = 82             # what AlertRenderer picks for text this short
GAP_ALERT_TEXT_Y = 11                # AlertRenderer's text1_y_offset for this font size
GAP_ALERT_SLIDE = 50                 # AlertRenderer slides text in from rect.y - 50
GAP_POPUP_SECONDS = 2.0

# Left column strip between the driver monitor icon (16,10 sized 60x60) and the
# steering wheel (centered at x=46, y=height-39), used for the lead distance.
LEAD_DIST_CENTER_X = 46
LEAD_DIST_CENTER_Y = 123
LEAD_DIST_FONT_SIZE = 32.0


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


# CarrotPilot's traffic-light state, published on longitudinalPlan.
TRAFFIC_OFF, TRAFFIC_RED, TRAFFIC_GREEN = 0, 1, 2
# xState: which longitudinal state its planner is in. Only the stopping ones matter here.
XSTATE_E2E_STOP, XSTATE_E2E_STOPPED = 3, 5

# carState.navMap.roadClass / rampType, decoded off the Tesla gateway's own map view -- see
# NavMapData in car.capnp. Ramp takes priority over road class when both are present: roadClass
# reads as the road just left for the whole length of a ramp, so showing it there is showing
# stale information (see map_cruise.py).
ROAD_CLASS_LABEL = {1: 'FREEWAY', 4: 'ARTERIAL', 5: 'COLLECTOR', 6: 'LOCAL'}
RAMP_ON, RAMP_OFF = 1, 2
RAMP_LABEL = {RAMP_ON: 'ON-RAMP', RAMP_OFF: 'OFF-RAMP'}
NAV_STATE_FONT_SIZE = 22.0
# Icon spans (16,10)-(76,70); this starts 10px clear of it and roughly centers the text on the
# icon's vertical middle.
NAV_STATE_X = 86
NAV_STATE_Y = 28
# Below this the plan is neither accelerating nor braking enough to draw an arrow for.
ACCEL_DEADBAND = 0.15  # m/s^2
# Small corner badge rather than a headline element -- it confirms what the model saw, it
# doesn't need to be read from across the cabin. Sized against the 60x60 driver-monitor icon.
LIGHT_RADIUS = 20
# AlertRenderer draws the lane-change turn-signal icon flush against the top-right corner
# (104px wide, 2px margin from the edge -- see alert_renderer.py's _draw_icons), so the light
# has to clear rect.width - 106 with room for its own halo, not just sit near the corner.
LIGHT_MARGIN_X = 130
LIGHT_MARGIN_Y = 20


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  LIGHT_RED = rl.Color(226, 44, 44, 255)
  LIGHT_GREEN = rl.Color(60, 200, 110, 255)
  ACCEL_UP = rl.Color(128, 216, 166, 255)
  ACCEL_DOWN = rl.Color(226, 120, 90, 255)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self._gap_adjust: int = 0
    self._last_gap_adjust: int = 0
    self._gap_popup_until: float = 0.0
    self._lead_d_rel: float | None = None

    self._nav_valid: bool = False
    self._road_class: int = 0
    self._ramp_type: int = 0
    self._nav_limit: float = 0.0  # m/s, 0 = none posted
    self._nav_fleet_median: float = 0.0  # m/s, 0 = none reported

    self._traffic_state: int = TRAFFIC_OFF
    self._x_state: int = 0
    self._plan_accel: float = 0.0
    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._gap_label = UnifiedLabel(text="", font_size=GAP_ALERT_FONT_SIZE, font_weight=FontWeight.DISPLAY,
                                   line_height=0.86)
    # Same slide-in-and-fade as AlertRenderer, so it enters and leaves like a real alert
    self._gap_alert_y_filter = BounceFilter(0, 0.1, 1 / gui_app.target_fps)
    self._gap_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 44, 44)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self._gap_adjust = 0
      self._last_gap_adjust = 0
      self._gap_popup_until = 0.0
      self._lead_d_rel = None
      self._nav_valid = False
      self._traffic_state = TRAFFIC_OFF
      self._x_state = 0
      self._plan_accel = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']
    plan = sm['longitudinalPlan']

    nav = car_state.navMap
    self._nav_valid = bool(nav.valid)
    self._road_class = int(nav.roadClass)
    self._ramp_type = int(nav.rampType)
    self._nav_limit = float(nav.baseSpeedLimit) if nav.baseSpeedLimit > 0 else float(nav.mapSpeedLimit)
    self._nav_fleet_median = float(nav.fleetMedianSpeed)

    # carrot's own target (eco control, the Tesla map auto-speed override, ...) takes priority
    # over the car's reported cluster value, the same way controlsd now feeds it to the car's
    # own dash -- so an auto speed change pops this box exactly like a lever change does,
    # instead of only being visible on the physical cluster.
    if sm.valid['longitudinalPlan'] and plan.cruiseTarget > 0:
      set_speed = plan.cruiseTarget
    else:
      v_cruise_cluster = car_state.vCruiseCluster
      set_speed = (
        controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
      )
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    gap_adjust = int(car_state.cruiseState.gapAdjust)
    if 1 <= gap_adjust <= 7:
      # Do not show a popup for the first valid value after UI startup.
      if self._last_gap_adjust != 0 and gap_adjust != self._last_gap_adjust:
        self._gap_popup_until = time.monotonic() + GAP_POPUP_SECONDS
      self._last_gap_adjust = gap_adjust
      self._gap_adjust = gap_adjust
    else:
      self._gap_adjust = 0

    lead_one = sm['radarState'].leadOne if sm.valid['radarState'] else None
    self._lead_d_rel = lead_one.dRel if (lead_one and lead_one.present) else None

    # Only the carrot planner fills these; the stock one leaves them zero, which reads as
    # "no traffic light seen" and draws nothing.
    self._traffic_state = int(plan.trafficState)
    self._x_state = int(plan.xState)
    self._plan_accel = float(plan.aTarget)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    if self.is_cruise_set:
      self._draw_set_speed(rect)

    self._draw_nav_state(rect)

    self._draw_steering_wheel(rect)

    # Only while engaged, matching the lane lines and path in the model renderer
    if self._engaged and self._lead_d_rel is not None:
      self._draw_lead_distance(rect)

    # Filters must run every frame, not just while showing, so the exit animates too.
    show_gap = (self._can_draw_top_icons and self._gap_adjust != 0 and
                time.monotonic() < self._gap_popup_until)
    self._gap_alert_y_filter.update(rect.y if show_gap else rect.y - GAP_ALERT_SLIDE)
    self._gap_alpha_filter.update(1 if show_gap else 0)

    if self._gap_alpha_filter.x > 0.01:
      self._draw_gap_popup(rect)

    self._draw_traffic_light(rect)

  def _draw_traffic_light(self, rect: rl.Rectangle) -> None:
    """A lamp for what the model sees, and an arrow for what the plan does about it.

    Drawn only when there is something to say: no lamp when no light is seen, no arrow when the
    plan is neither accelerating nor braking. A permanently-lit indicator stops being read.
    """
    if self._traffic_state == TRAFFIC_OFF:
      return

    cx = int(rect.x + rect.width - LIGHT_RADIUS - LIGHT_MARGIN_X)
    cy = int(rect.y + LIGHT_RADIUS + LIGHT_MARGIN_Y)

    lit = COLORS.LIGHT_RED if self._traffic_state == TRAFFIC_RED else COLORS.LIGHT_GREEN
    rl.draw_circle(cx, cy, LIGHT_RADIUS + 3, COLORS.BLACK_TRANSLUCENT)
    rl.draw_circle(cx, cy, LIGHT_RADIUS, lit)
    rl.draw_circle_lines(cx, cy, LIGHT_RADIUS, COLORS.WHITE_TRANSLUCENT)

    # The planner can still be holding a stop after the light goes green -- show that, rather
    # than a green lamp over a car that is not moving, which reads as a fault.
    if self._x_state in (XSTATE_E2E_STOP, XSTATE_E2E_STOPPED):
      rl.draw_circle_lines(cx, cy, LIGHT_RADIUS + 6, lit)

    if abs(self._plan_accel) < ACCEL_DEADBAND:
      return

    up = self._plan_accel > 0
    colour = COLORS.ACCEL_UP if up else COLORS.ACCEL_DOWN
    ay = cy + LIGHT_RADIUS + 16
    half, height = 9, 12
    tip = rl.Vector2(cx, ay - height / 2 if up else ay + height / 2)
    left = rl.Vector2(cx - half, ay + height / 2 if up else ay - height / 2)
    right = rl.Vector2(cx + half, ay + height / 2 if up else ay - height / 2)
    # raylib fills triangles wound counter-clockwise; swapping the base covers both directions.
    if up:
      rl.draw_triangle(tip, left, right, colour)
    else:
      rl.draw_triangle(tip, right, left, colour)

  def _draw_nav_state(self, rect: rl.Rectangle) -> None:
    """What the car's own map is saying right now, as text next to the driver-monitor icon.

    The traffic-light lamp and the MAX popup only mark the instant something changes; this is
    the opposite -- a live readout of the current state, so a freeway -> ramp -> surface-street
    transition is visible continuously rather than as a blink easy to miss on a 536x240 screen.
    Shares the driver-monitor icon's hide condition so the two never draw over each other.
    """
    if not self._nav_valid or self.drawing_top_icons() or not self._can_draw_top_icons:
      return

    if self._ramp_type in RAMP_LABEL:
      text = RAMP_LABEL[self._ramp_type]
      color = COLORS.ACCEL_DOWN if self._ramp_type == RAMP_OFF else COLORS.ACCEL_UP
    else:
      text = ROAD_CLASS_LABEL.get(self._road_class)
      if text is None:
        return
      color = COLORS.WHITE_TRANSLUCENT

    if self._nav_limit > 0:
      speed = self._nav_limit * (CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH)
      text = f"{text} {round(speed)}"

    # What the fleet actually drives here, alongside the posted limit -- the two can disagree
    # by a lot (a ramp has no posted limit at all), and seeing both is the point.
    if self._nav_fleet_median > 0:
      fleet_speed = self._nav_fleet_median * (CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH)
      text = f"{text}/{round(fleet_speed)}"

    x = rect.x + NAV_STATE_X
    y = rect.y + NAV_STATE_Y
    rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(x + 1, y + 1), NAV_STATE_FONT_SIZE, 0, rl.Color(0, 0, 0, 200))
    rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(x, y), NAV_STATE_FONT_SIZE, 0, color)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    # pos
    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    # color and draw
    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      # Draw exclamation point icon
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    # draw drop shadow
    circle_radius = 162 // 2
    rl.draw_circle_gradient(int(x + circle_radius), int(y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("MAX")
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

  def _draw_lead_distance(self, rect: rl.Rectangle) -> None:
    """Distance to the lead, in the left strip between the driver monitor and the wheel."""
    if self._lead_d_rel is None:
      return

    d = self._lead_d_rel
    text = f"{d:.0f}m" if ui_state.is_metric else f"{d * METER_TO_FOOT:.0f}ft"
    text_size = measure_text_cached(self._font_bold, text, LEAD_DIST_FONT_SIZE)

    x = rect.x + LEAD_DIST_CENTER_X - text_size.x / 2
    y = rect.y + LEAD_DIST_CENTER_Y - text_size.y / 2

    rl.draw_text_ex(self._font_bold, text, rl.Vector2(x + 2, y + 2), LEAD_DIST_FONT_SIZE, 0, rl.Color(0, 0, 0, 220))
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(x, y), LEAD_DIST_FONT_SIZE, 0, COLORS.WHITE)

  def _draw_gap_popup(self, rect: rl.Rectangle) -> None:
    """Briefly show the Tesla gap setting after the driver changes it.

    Styled like a lane change alert: same black-to-transparent top gradient, and the same
    lowercase display font at the same margin, so it reads as part of the alert language
    rather than a separate widget.
    """
    alpha = self._gap_alpha_filter.x

    # Background: solid for the top fifth, then fade out. Matches AlertRenderer's
    # small alert (0.583 of height) used by the lane change alerts.
    bg_height = round(rect.height * GAP_ALERT_BG_HEIGHT_FRAC)
    solid_height = round(bg_height * 0.2)
    color = rl.Color(0, 0, 0, int(255 * 0.90 * alpha))
    translucent = rl.Color(0, 0, 0, 0)

    rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), solid_height, color)
    rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y + solid_height), int(rect.width),
                                 int(bg_height - solid_height), color, translucent)

    text = f"Gap {self._gap_adjust}"
    text_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    self._gap_label.set_text(text)
    self._gap_label.set_text_color(text_color)
    self._gap_label.set_font_size(GAP_ALERT_FONT_SIZE)
    self._gap_label.set_alignment(rl.GuiTextAlignment.TEXT_ALIGN_LEFT)
    self._gap_label.render(rl.Rectangle(rect.x + GAP_ALERT_MARGIN,
                                        self._gap_alert_y_filter.x + GAP_ALERT_TEXT_Y,
                                        rect.width - GAP_ALERT_MARGIN, rect.height))

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
