"""Bends, driven again in closed loop by the real driving model (tools/tesla_analysis/latsim.py).

Why: the curve hold (LaneCenteringCurveHold) shipped on 11/11 with a unit test that passed against a
stand-in car -- a spring I had assumed. On the road it took the car off the inside of bends as meant
and made them unsteady: lateral accel swing +20 %, lateral jerk +18 % (routes 0267-0270). Nothing
that does not run the model can see that, because the swing is the model answering the trim.

Here nine recorded bends (tools/tesla_analysis/lateral_windows.txt; both directions, 96-121 km/h,
four driven without the hold and five with it) are replayed with the car free to leave the logged
path. What the replay is trusted for is tested first, against the road:
  the drive itself, replayed, stays within 0.13 m of the logged path in all nine
  without the hold the car sits 0.20 m inside the bend (road, 0240-0263: 0.22)
  with it, 0.04 m (road: 0.08), lateral accel swing +24 %, jerk +16 % (road: +20 %, +18 %)
Then the gate: what ships must not swing or jerk more in these bends than the plain lane centring
did when this was written (data/lateral_replay_baseline.json), and the curve hold must fail it.
What ships now is lane_aim.py (LaneCenteringAim), which moves what the model is shown instead of
adding to what it asks for: it has to pass the same gate and bring the car nearer the middle.

The replay needs what the repo does not carry -- the model compiled for this CPU, a clang for
tinygrad, and the segments' video (see latsim.py) -- and is skipped without them. It is 27 runs of
about 450 model frames: some 15 minutes on 16 cores the first time, then read from a cache keyed on
the lateral sources until one of them changes.
"""
import hashlib
import json
import os
import pickle
import re
from multiprocessing import get_context

import numpy as np
import pytest

from openpilot.tools.tesla_analysis import latsim

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
WINDOWS = os.path.join(ROOT, 'tools', 'tesla_analysis', 'lateral_windows.txt')
BASELINE = os.path.join(HERE, 'data', 'lateral_replay_baseline.json')
# whatever decides the curvature the car is asked for
SOURCES = ('selfdrive/controls/lib/lane_centering.py', 'selfdrive/controls/lib/lane_aim.py', 'selfdrive/controls/lib/drive_helpers.py',
           'selfdrive/modeld/modeld.py', 'tools/tesla_analysis/latsim.py', 'tools/tesla_analysis/lateral_windows.txt')
LEAD_IN = 5.0            # s of straight between closing the loop and the bend

MAX_OFF_PATH = 0.25      # m, the drive replayed against its own log
MAX_INSIDE_ERROR = 0.08  # m, where the replay puts the car in the lane against where it was
LIMITS = {'swing': 1.10, 'jerk': 1.08}   # shipped / baseline, mean over the bends


def windows():
  rows = [line.split() for line in open(WINDOWS) if line.strip() and not line.startswith('#')]
  return [(seg, float(a), float(b)) for seg, a, b in rows]


def shipped() -> tuple[bool, bool]:
  """(curve hold, aim) as the defaults have them."""
  keys = open(os.path.join(ROOT, 'common', 'params_keys.h')).read()
  on = lambda name: re.search(r'"%s",\s*\{[^}]*"(\d)"\}' % name, keys).group(1) == '1'
  return on('LaneCenteringCurveHold'), on('LaneCenteringAim')


OFF, HOLD, AIM = (False, False), (True, False), (False, True)


def _job(job):
  seg, start, end, cfg = job
  r = latsim.run(seg, hold=cfg[0], aim=cfg[1], closed=True, tclose=start, t0=max(0.0, start - 3.0), t1=end)
  return seg, cfg, latsim.metrics(r, start + LEAD_IN), bool(latsim.load(seg)[2]['hold_driven'])


@pytest.fixture(scope='module')
def replays():
  """{(segment, (hold, aim)): metrics}, and {segment: whether the drive had the hold}."""
  wins = windows()
  if not all(latsim.available(seg) for seg, _, _ in wins):
    pytest.skip('lateral replay needs the compiled model, clang and the segments\' video (see latsim.py)')
  h = hashlib.sha256()
  for src in SOURCES:
    h.update(open(os.path.join(ROOT, src), 'rb').read())
  pkl = os.path.join(latsim.S, 'driving_tinygrad.pkl')
  h.update(f'{os.path.getsize(pkl)} {latsim.ACT_DELAY} {latsim.ACT_TAU} {LEAD_IN}'.encode())
  cache = os.path.join(latsim.S, 'cache', h.hexdigest()[:16] + '.pkl')
  if os.path.isfile(cache):
    return pickle.load(open(cache, 'rb'))
  jobs = [(seg, a, b, cfg) for seg, a, b in wins for cfg in sorted({OFF, HOLD, AIM, shipped()})]
  with get_context('spawn').Pool(min(os.cpu_count() or 1, len(jobs))) as pool:
    out = pool.map(_job, jobs, chunksize=1)
  res = ({(seg, cfg): m for seg, cfg, m, _ in out}, {seg: driven for seg, _, _, driven in out})
  os.makedirs(os.path.dirname(cache), exist_ok=True)
  pickle.dump(res, open(cache, 'wb'))
  return res


def mean(runs, cfg, key):
  return float(np.mean([runs[(seg, cfg)][key] for seg, _, _ in windows()]))


class TestTheReplayIsTheRoad:
  def test_the_drive_replayed_stays_on_its_own_path(self, replays):
    runs, driven = replays
    for seg, _, _ in windows():
      m = runs[(seg, HOLD if driven[seg] else OFF)]
      assert m['off_path'] < MAX_OFF_PATH, f'{seg}: {m["off_path"]:.2f} m from the logged path'
      assert abs(m['inside'] - m['inside_log']) < MAX_INSIDE_ERROR, f'{seg}: lane position {m["inside"]:+.2f} vs {m["inside_log"]:+.2f} m'

  def test_both_kinds_of_drive_are_in_the_set(self, replays):
    _, driven = replays
    assert 3 <= sum(driven.values()) <= len(driven) - 3

  def test_without_the_hold_the_car_sits_inside_as_measured(self, replays):
    runs, _ = replays
    assert mean(runs, OFF, 'inside') == pytest.approx(0.22, abs=0.06)      # road, 0240-0263

  def test_the_hold_does_what_it_did_on_the_road(self, replays):
    runs, _ = replays
    assert mean(runs, HOLD, 'inside') < mean(runs, OFF, 'inside') - 0.10   # road: 0.22 -> 0.08
    assert mean(runs, HOLD, 'swing') / mean(runs, OFF, 'swing') == pytest.approx(1.20, abs=0.12)
    assert mean(runs, HOLD, 'jerk') / mean(runs, OFF, 'jerk') == pytest.approx(1.18, abs=0.10)


class TestBendsStaySteady:
  def ratios(self, runs, cfg):
    base = json.load(open(BASELINE))
    return {k: mean(runs, cfg, k) / float(np.mean([base[seg][k] for seg, _, _ in windows()])) for k in LIMITS}

  def test_what_ships_is_no_busier_than_plain_lane_centring(self, replays):
    runs, _ = replays
    got = self.ratios(runs, shipped())
    worse = {k: round(v, 3) for k, v in got.items() if v > LIMITS[k]}
    assert not worse, f'bends are less steady than the baseline: {worse} (limits {LIMITS})'

  def test_the_curve_hold_as_shipped_on_1111_fails(self, replays):
    runs, _ = replays
    got = self.ratios(runs, HOLD)
    assert got['swing'] > LIMITS['swing'] and got['jerk'] > LIMITS['jerk'], got

  def test_the_baseline_is_the_plain_lane_centring(self, replays):
    # if this moves, the lane centring itself changed: look at why before refreshing the file
    runs, _ = replays
    got = self.ratios(runs, OFF)
    assert got['swing'] == pytest.approx(1.0, abs=0.10) and got['jerk'] == pytest.approx(1.0, abs=0.08), got


class TestAimKeepsTheCarNearTheMiddle:
  def test_it_is_what_ships(self):
    assert shipped() == AIM

  def test_nearer_the_middle_in_every_bend_that_leaned(self, replays):
    runs, _ = replays
    for seg, _, _ in windows():
      off, aim = runs[(seg, OFF)]['inside'], runs[(seg, AIM)]['inside']
      assert aim < 0.16, f'{seg}: {aim:+.2f} m inside'
      if off > 0.15:
        assert aim < off - 0.05, f'{seg}: {off:+.2f} -> {aim:+.2f} m'
      assert aim > -0.08, f'{seg}: pushed {-aim:.2f} m to the outside'
    assert mean(runs, AIM, 'inside') < 0.12

  def test_as_steady_as_without_it(self, replays):
    runs, _ = replays
    assert mean(runs, AIM, 'swing') / mean(runs, OFF, 'swing') < 1.05
    assert mean(runs, AIM, 'jerk') / mean(runs, OFF, 'jerk') < 1.04
    for seg, _, _ in windows():
      assert runs[(seg, AIM)]['jerk'] / runs[(seg, OFF)]['jerk'] < 1.10, seg
