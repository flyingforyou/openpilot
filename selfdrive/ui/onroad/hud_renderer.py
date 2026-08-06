import time

import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


# CarrotPilot's traffic-light state, published on longitudinalPlan.
TRAFFIC_OFF, TRAFFIC_RED, TRAFFIC_GREEN = 0, 1, 2
# xState: which of its longitudinal states the planner is in. Only the stopping ones matter here.
XSTATE_E2E_STOP, XSTATE_E2E_STOPPED = 3, 5
# Below this the plan is neither accelerating nor braking in any way worth drawing an arrow for.
ACCEL_DEADBAND = 0.15  # m/s^2


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  LIGHT_RED = rl.Color(226, 44, 44, 255)
  LIGHT_GREEN = rl.Color(60, 200, 110, 255)
  LIGHT_OFF = rl.Color(80, 80, 80, 170)
  ACCEL_UP = rl.Color(128, 216, 166, 255)
  ACCEL_DOWN = rl.Color(226, 120, 90, 255)


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._gap_adjust: int = 0
    self._last_gap_adjust: int = 0
    self._gap_popup_until: float = 0.0
    self._traffic_state: int = TRAFFIC_OFF
    self._x_state: int = 0
    self._plan_accel: float = 0.0

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

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
      self._traffic_state = TRAFFIC_OFF
      self._x_state = 0
      self._plan_accel = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    gap_adjust = int(car_state.cruiseState.gapAdjust)
    if 1 <= gap_adjust <= 7:
      # Do not show a popup for the first valid value after UI startup.
      if self._last_gap_adjust != 0 and gap_adjust != self._last_gap_adjust:
        self._gap_popup_until = time.monotonic() + 1.0
      self._last_gap_adjust = gap_adjust
      self._gap_adjust = gap_adjust
    else:
      self._gap_adjust = 0

    # Only the carrot planner fills these; the stock one leaves them at zero, which reads as
    # "no traffic light seen" and draws nothing.
    plan = sm['longitudinalPlan']
    self._traffic_state = int(plan.trafficState)
    self._x_state = int(plan.xState)
    self._plan_accel = float(plan.aTarget)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    # Left of the experimental button, which already owns the top-right corner.
    self._draw_traffic_light(button_x - UI_CONFIG.border_size, button_y)

  def _draw_traffic_light(self, right_x: float, top_y: float) -> None:
    """A lamp for what the model sees, and an arrow for what the plan is doing about it.

    Drawn only when there is something to say: no lamp when no light is seen, no arrow when the
    plan is neither accelerating nor braking. A permanently-lit indicator stops being read.
    """
    if self._traffic_state == TRAFFIC_OFF:
      return

    radius = 44
    cx = int(right_x - radius)
    cy = int(top_y + UI_CONFIG.button_size / 2)

    lit = COLORS.LIGHT_RED if self._traffic_state == TRAFFIC_RED else COLORS.LIGHT_GREEN
    rl.draw_circle(cx, cy, radius + 6, COLORS.BLACK_TRANSLUCENT)
    rl.draw_circle(cx, cy, radius, lit)
    rl.draw_circle_lines(cx, cy, radius, COLORS.WHITE_TRANSLUCENT)

    # The planner can be stopping for a light it no longer calls red -- show that it is still
    # holding, rather than a green lamp over a car that is not moving.
    if self._x_state in (XSTATE_E2E_STOP, XSTATE_E2E_STOPPED):
      rl.draw_circle_lines(cx, cy, radius + 12, lit)

    if abs(self._plan_accel) < ACCEL_DEADBAND:
      return

    up = self._plan_accel > 0
    colour = COLORS.ACCEL_UP if up else COLORS.ACCEL_DOWN
    ax = cx
    ay = cy + radius + 34
    half, height = 20, 26
    tip = rl.Vector2(ax, ay - height / 2 if up else ay + height / 2)
    left = rl.Vector2(ax - half, ay + height / 2 if up else ay - height / 2)
    right = rl.Vector2(ax + half, ay + height / 2 if up else ay - height / 2)
    # raylib fills triangles wound counter-clockwise; swapping the base corners covers both.
    if up:
      rl.draw_triangle(tip, left, right, colour)
    else:
      rl.draw_triangle(tip, right, left, colour)

    if self._gap_adjust != 0 and time.monotonic() < self._gap_popup_until:
      self._draw_gap_popup(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

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

  def _draw_gap_popup(self, rect: rl.Rectangle) -> None:
    """Draw a transient 'GAP N' popup after the driver changes the Tesla gap setting."""
    text = f"GAP {self._gap_adjust}"
    font_size = 64
    padding_x = 42
    height = 104

    text_size = measure_text_cached(self._font_bold, text, font_size)
    width = text_size.x + padding_x * 2

    x = rect.x + (rect.width - width) / 2
    y = rect.y + 340

    popup_rect = rl.Rectangle(x, y, width, height)
    rl.draw_rectangle_rounded(popup_rect, 0.35, 10, rl.Color(0, 0, 0, 210))
    rl.draw_rectangle_rounded_lines_ex(popup_rect, 0.35, 10, 5, rl.Color(255, 255, 255, 100))

    text_pos = rl.Vector2(x + (width - text_size.x) / 2, y + (height - text_size.y) / 2)
    rl.draw_text_ex(self._font_bold, text, text_pos, font_size, 0, COLORS.WHITE)
