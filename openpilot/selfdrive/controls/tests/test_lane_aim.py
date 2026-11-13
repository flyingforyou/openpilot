"""lane_aim.py: the shift itself. What it does to a bend is in test_lateral_replay.py."""
import numpy as np
import pytest

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib import lane_aim as la

RPY = np.array([0.0045, -0.088, -0.014])
K = DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road.intrinsics


def run(aim, centre, n, v=30.0, enabled=True, active=True, signal=False, lane_change=False, prob=0.9, width=3.6):
  """`centre`: where the lane centre really is beside the car (m, + right); the model reports it less the shift."""
  for _ in range(n):
    c = centre - aim.shift
    aim.update(c - width / 2, c + width / 2, prob, prob, v, enabled, active, signal, lane_change)
  return aim.shift


class TestWarp:
  @pytest.mark.parametrize("big", [False, True])
  def test_no_shift_is_the_stock_matrix(self, big):
    np.testing.assert_allclose(la.shifted_warp(RPY, K, big, 0.0, 1.37), get_warp_matrix(RPY, K, big), rtol=1e-9)

  def test_a_point_on_the_road_moves_by_the_shift(self):
    # a model pixel, cast onto the road from the shifted camera, lands `shift` m further right than from the real one
    h, shift = 1.37, 0.2
    px = np.array([300.0, 180.0, 1.0])                       # below the horizon of the model frame
    ground = lambda ray: ray * h / ray[2]                    # calibrated frame: z down
    stock = np.linalg.inv(get_warp_matrix(RPY, K, False))    # camera pixel -> model pixel
    cam_px = la.shifted_warp(RPY, K, False, shift, h) @ px   # where the shifted view samples the real image
    from openpilot.common.transformations.model import calib_from_medmodel
    seen = ground(calib_from_medmodel @ (stock @ cam_px))    # the road point the real camera has there
    asked = ground(calib_from_medmodel @ px)                 # the road point that pixel means to the model
    np.testing.assert_allclose(seen - asked, [0.0, shift, 0.0], atol=1e-6)

  def test_the_horizon_does_not_move(self):
    from openpilot.common.transformations.model import medmodel_frame_from_calib_frame
    far = medmodel_frame_from_calib_frame[:, :3] @ np.array([1.0, 0.05, 0.0])     # a direction, level with the road
    a = la.shifted_warp(RPY, K, False, 0.3, 1.37) @ far
    b = get_warp_matrix(RPY, K, False) @ far
    np.testing.assert_allclose(a / a[2], b / b[2], atol=1e-6)


class TestShift:
  def test_a_steady_lean_is_answered_slowly(self):
    aim = la.LaneAim()
    # the car sits 0.25 m right of the centre (the centre is 0.25 m to its left)
    one = run(aim, -0.25, 20)
    assert 0.0 < one < 0.12                                  # after a second: on its way, no jump
    assert one == pytest.approx(la.AIM_GAIN * (0.25 - la.AIM_DEADBAND), rel=0.1)
    # shown to the right of where it is, the model moves the car left; here the car has not moved, so it keeps growing
    assert run(aim, -0.25, 200) == pytest.approx(la.AIM_MAX, abs=1e-6)

  def test_the_other_side(self):
    assert run(la.LaneAim(), +0.25, 20) < 0.0

  def test_inside_the_deadband_nothing_builds(self):
    assert run(la.LaneAim(), -0.07, 400) == 0.0

  def test_it_settles_when_the_car_comes_back(self):
    aim = la.LaneAim()
    run(aim, -0.25, 60)
    held = aim.shift
    after = run(aim, -0.05, 100)                             # the car is now within the deadband
    assert 0.5 * held < after < held                         # kept, less the leak
    assert run(aim, 0.0, 2000) < 0.01                        # and gone on a long straight

  @pytest.mark.parametrize("kw", [dict(active=False), dict(signal=True), dict(lane_change=True), dict(prob=0.3),
                                  dict(v=5.0), dict(width=5.5)])
  def test_lets_go(self, kw):
    aim = la.LaneAim()
    run(aim, -0.3, 100)
    full = aim.shift
    assert run(aim, -0.3, 60, **kw) < 0.2 * full             # 3 s

  def test_off_is_off(self):
    aim = la.LaneAim()
    run(aim, -0.3, 100)
    assert run(aim, -0.3, 1, enabled=False) == 0.0


def _model(shift=0.0):
  from openpilot.cereal import messaging
  msg = messaging.new_message('modelV2')
  m = msg.modelV2
  xs = [float(x) for x in range(0, 66, 2)]
  m.init('laneLines', 4)
  for i, c in enumerate((-5.4, -1.6, 2.0, 5.6)):
    m.laneLines[i].x = xs
    m.laneLines[i].y = [c - shift + 0.002 * x * x for x in xs]
  m.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  m.laneLineStds = [0.1, 0.1, 0.1, 0.1]
  m.init('roadEdges', 2)
  for i, c in enumerate((-7.0, 7.0)):
    m.roadEdges[i].x = xs
    m.roadEdges[i].y = [c - shift for _ in xs]
  m.init('leadsV3', 1)
  m.leadsV3[0].x = [40.0, 42.0]
  m.leadsV3[0].y = [0.5 - shift, 0.6 - shift]
  m.position.x = xs
  m.position.y = [0.001 * x * x for x in xs]
  m.position.yStd = [0.05 for _ in xs]
  return m


class TestRoadFrame:
  """modelV2 reports what the model saw through the shift where it is on the road."""

  def test_the_world_moves_back_and_the_cars_path_does_not(self):
    m = _model(shift=0.25)
    path = list(m.position.y)
    la.to_road_frame(m, 0.25)
    assert m.meta.laneAimShift == pytest.approx(0.25)
    np.testing.assert_allclose(m.laneLines[1].y[0], -1.6, atol=1e-6)
    np.testing.assert_allclose(m.laneLines[2].y[0], 2.0, atol=1e-6)
    np.testing.assert_allclose(m.roadEdges[0].y[5], -7.0, atol=1e-6)
    np.testing.assert_allclose(list(m.leadsV3[0].y), [0.5, 0.6], atol=1e-6)
    np.testing.assert_allclose(list(m.position.y), path)

  def test_no_shift_touches_nothing(self):
    m = _model()
    before = list(m.laneLines[1].y)
    la.to_road_frame(m, 0.0)
    assert list(m.laneLines[1].y) == before and m.meta.laneAimShift == 0.0

  def test_lane_centring_still_sees_what_the_model_saw(self):
    # the trim was tuned and replayed on the model's view; reporting the road must not change it
    from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
    seen = _model(shift=0.25)                  # as the model saw it, nothing put back
    road = _model(shift=0.25)
    la.to_road_frame(road, 0.25)
    a, b = LaneCenteringController(), LaneCenteringController()
    for _ in range(40):
      ka = a.update(0.001, seen, 30.0, True, 0.0, 0.0, True, True, True, False, False)
      kb = b.update(0.001, road, 30.0, True, 0.0, 0.0, True, True, True, False, False)
    assert a._correction != 0.0
    assert kb == pytest.approx(ka, abs=1e-9)
