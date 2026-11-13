"""drive_report.py: a verdict needs its scene, one drive is a small sample, and known-bad drives must read bad."""
import json
import os

import numpy as np
import pytest

from openpilot.tools.tesla_analysis import drive_report as dr

BASE = json.load(open(dr.BASELINE))
HOLD_DRIVES = ['0000026c', '0000026d', '0000026e', '0000026f', '00000270']     # 11/13, curve hold on: the bends swung
have_logs = pytest.mark.skipif(not all(dr.segments(r) for r in HOLD_DRIVES + BASE['routes'][:1]), reason='needs the route logs')


def drive(seconds=600.0, params=None, **cols):
  """A straight, empty motorway at 30 m/s with everything on; cols overwrite columns (value or array)."""
  n = int(seconds / dr.DT)
  a = np.zeros((n, len(dr.COLS)))
  base = dict(t=np.arange(n) * dr.DT, v=30.0, enabled=1, lat_active=1, long_active=1, lane_l=-1.8, lane_r=1.8,
              prob_l=0.9, prob_r=0.9, exec_ms=25.0, sec=np.arange(n) * dr.DT % 60, seg=np.arange(n) * dr.DT // 60)
  for k, v in {**base, **cols}.items():
    a[:, dr.C[k]] = v
  return dr.Drive(a, [], [], {}, ['abc'], params or {}, ['00000001--x'], np.zeros(n, dtype=int))


def status(d, name):
  return next(c for c in dr.checks(dr.measure(d), BASE, d.params) if c.name == name).status


def test_runs_stop_at_a_jump_in_time():
  t = np.array([0, .05, .10, 5.0, 5.05])
  assert dr.runs(np.ones(5), t) == [(0, 3), (3, 5)]
  assert dr.runs(np.array([1, 1, 0, 1, 1]), t, min_len=2) == [(0, 2), (3, 5)]


def test_grade_fails_only_beyond_every_reference_drive():
  spread = [0.12, 0.17, 0.185]
  assert dr.grade(0.170, 0.160, spread) == dr.PASS          # within +10 % of the pooled reference
  assert dr.grade(0.180, 0.160, spread) == dr.WATCH         # above it, but a reference drive read worse
  assert dr.grade(0.200, 0.160, spread) == dr.FAIL
  assert dr.grade(1.20, 1.30, [1.0, 1.3], margin=0.15, lower_is_worse=True) == dr.PASS
  assert dr.grade(1.10, 1.30, [1.0, 1.3], margin=0.15, lower_is_worse=True) == dr.WATCH
  assert dr.grade(0.90, 1.30, [1.0, 1.3], margin=0.15, lower_is_worse=True) == dr.FAIL


def test_nothing_is_judged_without_its_scene():
  d = drive()
  got = {c.name: c.status for c in dr.checks(dr.measure(d), BASE, d.params)}
  for name in ('커브 출렁임', '커브 차선 위치', '차선변경', '레이더·카메라 매칭', '정지차 인식', '추종 잔제동', '앞차 따라 출발',
               '진입램프 속도', '진출 전 감속', '시동 직후 조향 프레임 공백'):
    assert got[name] == dr.NONE, name
  assert dr.FAIL not in got.values(), got


def test_one_brake_application_is_one_hard_brake():
  n = int(600 / dr.DT)
  a = np.zeros(n)
  a[2000:2100] = -2.5 + 0.6 * np.sin(np.arange(100))     # hovering around -2: still one application
  a[6000:6050] = -3.0
  m = dr.measure(drive(a_ego=a))
  assert len(m['scenes']['hard']) == 2


def test_the_aim_is_judged_only_where_the_drive_had_it():
  assert not any(c.name == '에임 이동량' for c in dr.checks(dr.measure(drive()), BASE, {}))
  assert status(drive(params={'LaneCenteringAim': '1'}), '에임 이동량') == dr.FAIL            # on, and never moved
  n = int(600 / dr.DT)
  small = 0.03 * np.sin(np.arange(n) / 400)
  assert status(drive(params={'LaneCenteringAim': '1'}, aim=small, lane_l=-1.8 - small, lane_r=1.8 - small), '에임 이동량') == dr.PASS
  assert status(drive(params={'LaneCenteringAim': '1'}, aim=0.30, lane_l=-2.1, lane_r=1.5), '에임 이동량') == dr.FAIL   # stuck at its limit


def test_a_stopped_car_met_late_fails():
  n = int(600 / dr.DT)
  lead, d_rel, a = np.zeros(n), np.zeros(n), np.zeros(n)
  lead[4000:4200], d_rel[4000:4200] = 1, np.linspace(60, 5, 200)
  a[4000:4100] = -4.5
  d = drive(v=20.0, lead=lead, d_rel=d_rel, a_ego=a)
  assert status(d, '정지차 인식') == dr.FAIL
  a[4000:4100] = -2.5
  assert status(drive(v=20.0, lead=lead, d_rel=d_rel, a_ego=a), '정지차 인식') == dr.PASS


def test_an_on_ramp_held_under_the_traffic_fails():
  n = int(600 / dr.DT)
  ramp = np.zeros(n)
  ramp[2000:2300] = 1
  assert status(drive(v=17.8, ramp=ramp, fleet=23.0, nav_valid=1), '진입램프 속도') == dr.FAIL       # 64 km/h, traffic at 83
  assert status(drive(v=22.0, ramp=ramp, fleet=23.0, nav_valid=1), '진입램프 속도') == dr.PASS


def test_a_fault_while_engaged_fails_and_start_up_noise_does_not():
  d = drive()
  d.events = [(10.0, 'canError', 0, False)]
  assert status(d, '경고·폴트') == dr.PASS
  d.events = [(10.0, 'canError', 0, True)]
  assert status(d, '경고·폴트') == dr.FAIL


@have_logs
def test_the_drives_with_the_curve_hold_fail_on_bends():
  d = dr.load(HOLD_DRIVES)
  assert status(d, '커브 출렁임') == dr.FAIL


@have_logs
def test_no_reference_drive_fails_what_it_is_the_reference_for():
  for r in BASE['routes']:
    if not dr.segments(r):
      continue
    d = dr.load([r])
    got = {c.name: c.status for c in dr.checks(dr.measure(d), BASE, d.params)}
    for name in ('커브 출렁임', '직선 안정', '앞차 따라 출발'):
      assert got[name] != dr.FAIL, (r, name)
