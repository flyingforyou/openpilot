import colorsys
import time
import numpy as np
import pyray as rl
from openpilot.cereal import messaging
from opendbc.car.structs import car
from dataclasses import dataclass, field
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.mici.onroad.side_vehicles import (
  DEFAULT_HALF_WIDTH,
  HALF_WIDTH,
  side_vehicles,
)
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

# Adjacent-lane markers. The width of one is the vehicle's real width, so it needs a reference:
# a car, against which a motorcycle draws narrow and a lorry wide.
SIDE_REFERENCE_HALF_WIDTH = 0.90

# A motorcycle is drawn at 0.44 of a car's width, which is the information -- but scaled straight
# it also comes out the smallest marker on screen for the vehicle hardest to see in reality. The
# floor keeps a narrow one legible without flattening the difference: a car still draws wider.
SIDE_MIN_HALF_PX = 11.0

# The lead label replaced the chevron, so it is the indicator rather than an annotation on one,
# and it was sized to sit beside a shape that is no longer drawn.
# One size for every number and label on a marker. Distances are always in metres: the markers are
# read against each other, and mixing units or sizes between them would say one is the more
# important, which it is not.
MARKER_FONT_SIZE = 26.0

# The lead's own type, off the factory camera's lead group, matched on distance like the side ones.
DAS_GROUP_LEAD = 0
DAS_LEAD_MATCH_M = 4.0
SIDE_LINE_PX = 4.0
SIDE_VEHICLE_COLOR = rl.Color(255, 255, 255, 180)
SIDE_CUTIN_COLOR = rl.Color(226, 44, 44, 235)
SIDE_SHADOW_COLOR = rl.Color(0, 0, 0, 190)

# radarState.leadOne/leadTwo.present flickers false for single frames on a real lead -- checked
# against today's actual driving data (route 00000003) and found 56 drop/recover episodes in
# just the first 10 minutes while engaged, almost all under 1s and clustered on leads 100m+ out
# where the return is weakest. The chevron used to reset every one of those to nothing; holding
# the last known position for a short grace period bridges the flicker without touching what the
# controller itself sees -- this only changes what gets drawn, not radarState or any MPC input.
LEAD_HOLD_SECONDS = 0.5

# radarState.leadOne/leadTwo.vehicleClass, carried through from the Bosch radar's own per-point
# classifier (see car.capnp's RadarPoint.VehicleClass and radar_interface.py). "unknown" and
# anything vision-only (no radar match at all) show nothing -- a guess isn't better than the
# plain chevron. classProb mirrors the 50% floor radar_interface.py already uses to accept a
# point as real in the first place, so the bar for showing a label matches the bar for showing
# the point at all.
VEHICLE_CLASS_LABEL = {
  'fourWheel': 'CAR',
  'twoWheel': 'BIKE',
  'pedestrian': 'PED',
  'constructionElement': 'CONST',
}
VEHICLE_CLASS_MIN_PROB = 0.5

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]

LANE_LINE_COLORS = {
  UIStatus.DISENGAGED: rl.Color(200, 200, 200, 255),
  UIStatus.OVERRIDE: rl.Color(255, 255, 255, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 64, 255),
}


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  d_rel: float = 0.0
  half_width: float = 0.90
  fill_alpha: int = 0
  source: str = ""
  vehicle_class: str = ""  # "" = not reported or not confident enough to show


class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._left_blindspot = False
    self._right_blindspot = False
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._lead_last_seen = [0.0, 0.0]
    self._side_vehicles: list = []
    self._das_objects: list = []
    self._path_offset_z = HEIGHT_INIT[0]

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    self._acceleration_x_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._acceleration_x_filter2 = FirstOrderFilter(0.0, 1, 1 / gui_app.target_fps)

    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._ll_color_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

    # Check if data is up-to-date
    if (sm.recv_frame["extrinsicsCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode
    car_state = sm['carState'] if sm.valid['carState'] else None
    # An overtake hold is a lane that is occupied as far as the lane change is concerned, so it
    # flashes the same way. Drawing it differently would ask the driver to learn a second signal
    # for a side they cannot use either way.
    meta = sm['modelV2'].meta
    self._left_blindspot = bool((car_state and car_state.leftBlindspot) or meta.overtakeHoldLeft)
    self._right_blindspot = bool((car_state and car_state.rightBlindspot) or meta.overtakeHoldRight)

    extrinsics_calibration = sm['extrinsicsCalibration']
    self._path_offset_z = extrinsics_calibration.height[0] if extrinsics_calibration.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      # Before the leads, which read it for the lead's own type.
      self._das_objects = list(car_state.dasObjects) if car_state is not None else []

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array)
      self._update_side_vehicles(car_state, path_x_array)
      self._transform_dirty = False

    # Draw elements (hide when disengaged)
    if ui_state.status != UIStatus.DISENGAGED:
      self._draw_lane_lines()
      self._draw_path(sm)

      if render_lead_indicator and radar_state:
        self._draw_lead_indicator()

      self._draw_side_vehicles()

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array):
    """Update positions of lead vehicles"""
    leads = [radar_state.leadOne, radar_state.leadTwo]
    now = time.monotonic()

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.present:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          source = "R" if lead_data.radar else "V"
          vehicle_class = ""
          if lead_data.classProb >= VEHICLE_CLASS_MIN_PROB:
            vehicle_class = VEHICLE_CLASS_LABEL.get(str(lead_data.vehicleClass), "")
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect, source, vehicle_class)
          self._lead_last_seen[i] = now
      elif now - self._lead_last_seen[i] > LEAD_HOLD_SECONDS:
        # Only actually clear it once the grace period is up -- a status flicker on the same
        # frame this runs would otherwise blank the chevron immediately, every time.
        self._lead_vehicles[i] = LeadVehicle()

  def _update_side_vehicles(self, car_state, path_x_array):
    """Project the adjacent-lane vehicles into the image, where they actually are.

    Drawn in car space like the lead chevron rather than parked at the edge of the screen: a
    marker that does not move with the vehicle it describes is a legend, not an indicator, and
    at this size it is easier to ignore than to read.

    Width is the vehicle's own, scaled by the same perspective factor the chevron uses, so a
    motorcycle stays narrow at any distance and a lorry stays wide. That is the whole reason the
    factory camera's type is worth reading -- the radar cannot tell them apart.
    """
    self._side_vehicles = []
    if car_state is None:
      return

    for veh in side_vehicles(car_state.dasObjects):
      idx = self._get_path_length_idx(path_x_array, veh.d_rel)
      z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
      # dasObjects' dy is right-positive, the same sense _map_to_screen is given for a lead
      # (which passes -yRel, and yRel is left-positive). No sign flip here.
      point = self._map_to_screen(veh.d_rel, veh.dy, z + self._path_offset_z)
      sz = np.clip((25 * 30) / (veh.d_rel / 3 + 30), 15.0, 30.0)
      half = max(SIDE_MIN_HALF_PX, sz * (veh.half_width / SIDE_REFERENCE_HALF_WIDTH))

      if point is None:
        # Beside us, or close enough that the road under it projects off the bottom of the frame.
        # This is exactly when a driver most wants to know, so it does not go unshown: the marker
        # goes to the edge it is on, at the bottom, where "alongside" reads without a legend.
        x = half + 4.0 if veh.dy < 0 else self._rect.width - half - 4.0
        y = self._rect.height - 8.0
      else:
        x, y = point[0], min(point[1], self._rect.height - 8.0)
      self._side_vehicles.append((x, y, half, sz * 0.42, veh.is_cutin, veh.d_rel))

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    line_width_factor = 0.12
    for i, lane_line in enumerate(self._lane_lines):
      if i in (1, 2):
        line_width_factor = 0.16
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, line_width_factor * self._lane_line_probs[i], 0.0, max_idx
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, line_width_factor, 0.0, max_idx)

    # Update path using raw points
    if lead and lead.present:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    soon_acceleration = self._acceleration_x[len(self._acceleration_x) // 4] if len(self._acceleration_x) > 0 else 0
    self._acceleration_x_filter.update(soon_acceleration)
    self._acceleration_x_filter2.update(soon_acceleration)

    # make path width wider/thinner when initially braking/accelerating
    if self._experimental_mode and False:
      high_pass_acceleration = self._acceleration_x_filter.x - self._acceleration_x_filter2.x
      y_off = np.interp(high_pass_acceleration, [-1, 0, 1], [0.9 * 2, 0.9, 0.9 / 2])
    else:
      y_off = 0.9

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, y_off, self._path_offset_z, max_idx, allow_invert=False
    )

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient.colors = segment_colors
    self._exp_gradient.stops = gradient_stops

  def _lead_half_width(self, d_rel):
    """The lead's real half-width, from the factory camera's own lead group when it has one.

    Same table and same reason as the side markers: the radar cannot type a vehicle on this car,
    and drawing every lead the width of a saloon throws away the one thing that makes the marker
    say what is in front rather than just where.
    """
    best, gap = DEFAULT_HALF_WIDTH, DAS_LEAD_MATCH_M
    for obj in self._das_objects:
      if int(obj.group) != DAS_GROUP_LEAD:
        continue
      if abs(float(obj.dx) - d_rel) < gap:
        gap = abs(float(obj.dx) - d_rel)
        best = HALF_WIDTH.get(int(obj.objType), DEFAULT_HALF_WIDTH)
    return best

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect, source, vehicle_class=""):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 1
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha), source=source,
                       vehicle_class=vehicle_class, d_rel=float(d_rel),
                       half_width=self._lead_half_width(d_rel))

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    alpha = np.clip(prob, 0.0, 0.7)
    if adjacent:
      _base_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED])
      color = rl.Color(_base_color.r, _base_color.g, _base_color.b, int(alpha * 255))

      # turn adjacent lls orange if torque is high
      torque = self._torque_filter.x
      high_torque = abs(torque) > 0.6
      if high_torque and (left == (torque > 0)):
        color = blend_colors(
          color,
          rl.Color(255, 115, 0, int(alpha * 255)),  # orange
          np.interp(abs(torque), [0.6, 0.8], [0.0, 1.0])
        )
    else:
      color = rl.Color(255, 255, 255, int(alpha * 255))

    if ui_state.status == UIStatus.DISENGAGED:
      color = rl.Color(0, 0, 0, int(alpha * 255))

    return color

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    """Two closest lines should be green (lane line or road edges)"""
    # laneLines[1] and laneLines[2] are the ego lane's left and right boundaries.
    # Toggle every 250 ms, resulting in a 2 Hz warning flash.
    blindspot_flash_on = int(time.monotonic() * 4.0) % 2 == 0

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      blindspot_active = (i == 1 and self._left_blindspot) or (i == 2 and self._right_blindspot)
      if blindspot_active:
        color = rl.Color(255, 35, 35, 255 if blindspot_flash_on else 45)
      else:
        color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      # if closest lane lines are not confident, make road edges green
      color = self._get_ll_color(float(1.0 - self._road_edge_stds[i]), float(self._lane_line_probs[i + 1]) < 0.25, i == 0)
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if self._experimental_mode:
      # Draw with acceleration coloring
      if ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      elif len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )

      if ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(0, 0, 0, 90))
      else:
        draw_polygon(self._rect, self._path.projected_points, gradient=gradient)

  def _draw_bracket(self, x, y, half, height, colour):
    """A bracket standing on the road at (x, y), `half` wide. Open at the top so it reads as
    sitting under the vehicle rather than boxing it in, which at this size is just a smudge.

    Returns the y of its top edge, so a caller can stack text above it."""
    left, right, top, bottom = x - half, x + half, y - height, y
    sides = ((left, bottom, right, bottom), (left, bottom, left, top), (right, bottom, right, top))
    for (x0, y0, x1, y1) in sides:
      rl.draw_line_ex(rl.Vector2(x0 + 1, y0 + 1), rl.Vector2(x1 + 1, y1 + 1), SIDE_LINE_PX, SIDE_SHADOW_COLOR)
    for (x0, y0, x1, y1) in sides:
      rl.draw_line_ex(rl.Vector2(x0, y0), rl.Vector2(x1, y1), SIDE_LINE_PX, colour)
    return top

  def _draw_marker_text(self, text, x, anchor_y, colour, above=True):
    """One line centred on x, stacked above or below anchor_y. Returns the edge it now occupies.

    Everything on a marker is the same size, lead and adjacent lane alike -- the numbers are read
    together and a difference in size would say one mattered more, which is not the case.
    """
    size = measure_text_cached(self._font_bold, text, MARKER_FONT_SIZE)
    ty = anchor_y - size.y - 2 if above else anchor_y + 2
    tx = x - size.x / 2
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(tx + 1, ty + 1), MARKER_FONT_SIZE, 0, SIDE_SHADOW_COLOR)
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(tx, ty), MARKER_FONT_SIZE, 0, colour)
    return ty if above else ty + size.y

  def _draw_side_vehicles(self):
    """A bracket sitting on the road where the vehicle is, its width the vehicle's own."""
    for (x, y, half, height, is_cutin, d_rel) in self._side_vehicles:
      colour = SIDE_CUTIN_COLOR if is_cutin else SIDE_VEHICLE_COLOR
      top = self._draw_bracket(x, y, half, height, colour)
      self._draw_marker_text(f"{d_rel:.0f}m", x, top, colour)
  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.chevron or not lead.source:
        continue

      # The label is the whole indicator now. The chevron and its glow were the largest thing on
      # a 536-wide screen and said nothing the label does not: where the lead is, which the label
      # already sits on, and how close, which the distance readout gives in metres.
      #
      # R = matched to a radar track, V = vision only. vehicle_class, when the radar reported one
      # with enough confidence, rides along in the same label rather than a second text draw --
      # one readable line beats two small ones at this size. Use a loaded font: rl.draw_text()
      # would draw nothing, since raylib's built-in default font isn't populated in this app.
      apex_x, apex_y = lead.chevron[1]
      sz = max(lead.chevron[0][1] - apex_y, 1.0)
      colour = rl.Color(80, 200, 255, 255) if lead.source == "R" else rl.Color(255, 190, 50, 255)

      # Drawn the same way as the side markers, so the road reads as one picture: a bracket under
      # the vehicle, its distance above it, and above that what the lead is and where it came
      # from. The distance used to live in the left column and is here now because this is where
      # it is being looked at.
      half = max(SIDE_MIN_HALF_PX, sz * (lead.half_width / SIDE_REFERENCE_HALF_WIDTH))
      bottom = apex_y + sz
      top = self._draw_bracket(apex_x, bottom, half, sz * 0.42, colour)

      # Distance above the bracket, exactly as the adjacent-lane markers place theirs, and the
      # source below it -- which of the two is wanted at a glance is the distance.
      self._draw_marker_text(f"{lead.d_rel:.0f}m", apex_x, top, colour)
      label = f"{lead.source} {lead.vehicle_class}" if lead.vehicle_class else lead.source
      self._draw_marker_text(label, apex_x, bottom, colour, above=False)

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_height: float) -> int:
    """Get the index corresponding to the given path height"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_height)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]
    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  @staticmethod
  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]
