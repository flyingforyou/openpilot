from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.lane_change_accel import (
  LaneChangeAccel, MAX_CLOSING, MIN_HEADWAY_S, RAMP_S, START_S, TARGET_LANE_SLOWER_M,
)

V = 28.0          # ~100 km/h
V_CRUISE = 33.0   # room to go faster
A_MAX = 1.0


def lead(d=40.0, v=V, a=0.0, present=True):
  return SimpleNamespace(present=present, dRel=d, vLead=v, aLeadK=a)


def run(seconds, a_target=0.0, changing=True, side_clear=True, ld=None, side=None):
  lca = LaneChangeAccel(DT_MDL)
  out = a_target
  for _ in range(int(seconds / DT_MDL)):
    out = lca.update(a_target, changing, side_clear, V, V_CRUISE, A_MAX, ld or lead(), side)
  return out, lca


def test_nothing_before_start():
  out, _ = run(START_S - 0.1)
  assert out == 0.0


def test_ramps_up_toward_free_road_accel():
  mid, _ = run(START_S + RAMP_S / 2)
  full, lca = run(START_S + RAMP_S + 0.2)
  assert 0.0 < mid < full
  assert lca.weight == 1.0
  assert full <= A_MAX


def test_never_lowers_a_command():
  out, _ = run(START_S + RAMP_S + 0.2, a_target=1.5)
  assert out == 1.5


def test_off_when_not_changing():
  out, lca = run(3.0, changing=False)
  assert out == 0.0 and lca.weight == 0.0


def test_off_when_the_side_is_not_clear():
  out, _ = run(3.0, side_clear=False)
  assert out == 0.0


def test_off_behind_a_close_or_braking_or_slower_lead():
  assert run(3.0, ld=lead(d=MIN_HEADWAY_S * V - 1.0))[0] == 0.0
  assert run(3.0, ld=lead(a=-1.0))[0] == 0.0
  assert run(3.0, ld=lead(v=V - MAX_CLOSING - 0.5))[0] == 0.0


def test_off_with_a_slower_car_in_the_target_lane():
  assert run(3.0, side=lead(d=TARGET_LANE_SLOWER_M - 10, v=V - 3.0))[0] == 0.0
  # a faster one there is fine
  assert run(3.0, side=lead(d=50.0, v=V + 2.0))[0] > 0.0


def test_no_lead_at_all_is_fine():
  assert run(3.0, ld=lead(present=False))[0] > 0.0
