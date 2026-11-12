"""Ride quality of the acc-mode follow chain (lib/follow_shaping.py) on recorded following.

Why: on 11/12 the car felt busier than before -- more small brake applications, a tug from behind --
and every unit test still passed, because each piece was tested for doing its own job, not for what
it costs a passenger in ordinary following. These tests replay real minutes of steady following
through the chain with the shipped defaults and hold it to the planner's own smoothness.

Fixtures (data/follow_quality.npz) are 20 Hz longitudinalPlan frames from routes 0250-0266, driven
with LeadBrakeFollow, ApproachCoast and LeadAheadBrake off, so aTarget there is the planner alone:
  steady_*     14 one-minute windows of engaged following, 8 above 85 km/h
  leadbrake_*  10 sixteen-second windows around a lead braking past 1 m/s^2
columns: vEgo, aTarget, lead(present, radar, dRel, vLead, aLeadK), leadTwo(same), tFollow, desiredDistance
The metrics are lib/ride_quality.py's vector, the same one tools/tesla_analysis/commit_quality.py
scores whole commits with (closed loop, morning-commute minutes). Each limit is "no worse than the
planner alone" by a margin, and each has a configuration that has actually shipped and fails it:
  11/11 evening  brake applications +4.6 a minute
  11/12 morning  brake applications +0.94, command jerk x1.58  ("busier", "a tug from behind")
  11/12 evening  throttle applications +0.31 a minute, jerk x1.09 (ApproachCoast giving back at 1 m/s^3)
  (11/12 late: rate 0.25 and no braking-lead prediction -- found by commit_quality.py on 7.6 h of commutes)
"""
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib import carrot_t_follow as ctf
from openpilot.selfdrive.controls.lib import follow_shaping as fs
from openpilot.selfdrive.controls.lib import lead_ahead as la
from openpilot.selfdrive.controls.lib import ride_quality as rq

DT = 0.05
FIXTURES = os.path.join(os.path.dirname(__file__), 'data', 'follow_quality.npz')
STOP_DISTANCE = 5.5
COMFORT_BRAKE_2 = 2.37

# What following may cost on top of the planner's own output (ride_quality vector).
LIMITS = {
  'brake_on': ('+', 0.2),      # brake applications a minute
  'ubon': ('+', 0.1),          # ... of them with the lead not braking or closing
  'accel_on': ('+', 0.15),     # throttle applications a minute
  'rev': ('+', 0.3),           # accel <-> brake reversals a minute
  'cmd_jerk': ('x', 1.06),     # RMS command jerk
  'calm_brake': ('+', 0.5),    # % of calm time spent braking
}


def defaults() -> dict:
  p = Params()
  return {k: p.get(k, return_default=True) for k in ("BrakeOnsetJerk", "LeadBrakeFollow", "LeadBrakeBufferCm", "ApproachCoast", "ComfortBrake")}


@pytest.fixture
def chain(monkeypatch):
  """Configure the chain from the shipped param defaults, restoring the module afterwards."""
  for k in ("BRAKE_ONSET_JERK", "LEAD_BRAKE_BUFFER", "LEAD_BRAKE_FOLLOW", "RELEASE_ON_OPENING", "APPROACH_COAST"):
    monkeypatch.setattr(ctf, k, getattr(ctf, k))

  def make(overrides=None):
    cfg = {**defaults(), **(overrides or {})}
    fs.configure(lambda k: int(cfg[k]), lambda k: bool(cfg[k]), STOP_DISTANCE)
    return cfg
  return make


def lead(r, o):
  return NS(present=bool(r[o]), radar=bool(r[o + 1]), dRel=float(r[o + 2]), vLead=float(r[o + 3]), aLeadK=float(r[o + 4]))


def replay(rows, cfg):
  sh = fs.FollowShaping(DT)
  cb = int(cfg["ComfortBrake"]) / 100.0
  return np.array([sh.update(float(r[1]), lead(r, 2), lead(r, 7), float(r[0]), STOP_DISTANCE, float(r[13]), float(r[12]),
                             cb, COMFORT_BRAKE_2) for r in rows])


def load(prefix):
  data = np.load(FIXTURES)
  return {k: data[k] for k in data.files if k.startswith(prefix)}


def parts(fx, cmds):
  return [rq.components(c, r[:, 0], r[:, 4], r[:, 5], r[:, 6], DT) for r, c in zip(fx.values(), cmds, strict=True)]


def steady_quality(chain_cfg):
  """(planner vector, chain vector) over the steady-following fixtures."""
  fx = load('steady_')
  ins = [rows[:, 1].astype(float) for rows in fx.values()]
  outs = [replay(rows, chain_cfg) for rows in fx.values()]
  return rq.vector(parts(fx, ins)), rq.vector(parts(fx, outs))


def violations(chain_cfg) -> dict:
  base, out = steady_quality(chain_cfg)
  bad = {}
  for k, (kind, lim) in LIMITS.items():
    worse = out[k] - base[k] if kind == '+' else out[k] / max(base[k], 1e-9)
    if worse > lim:
      bad[k] = (round(base[k], 3), round(out[k], 3))
  return bad


def test_fixtures_are_what_the_docstring_says():
  steady, brakes = load('steady_'), load('leadbrake_')
  assert len(steady) == 14 and len(brakes) == 10
  assert sum(len(a) for a in steady.values()) * DT / 60 > 12
  assert sum(np.median(a[:, 0]) * 3.6 > 85 for a in steady.values()) == 8


def test_steady_following_is_no_busier_than_the_planner(chain):
  assert violations(chain()) == {}


@pytest.mark.parametrize("name, engage, release, rate, broken", [
  ("11/11 evening", (-0.7, -0.2), True, 2.0, 'brake_on'),
  ("11/12 morning", (-1.2, -0.6), True, 2.0, 'cmd_jerk'),
  ("11/12 evening", (-2.5, -1.5), False, 1.0, 'accel_on'),
])
def test_the_builds_that_felt_busy_fail(chain, monkeypatch, name, engage, release, rate, broken):
  monkeypatch.setattr(ctf, 'APPROACH_COAST_ON', (12.0, 3.0, 2.0))   # all three predicted a braking lead 3 s ahead
  """The gate has to be able to fail: every configuration the driver called out, or that the
  commit comparison showed busier, fails it on the metric that moved."""
  monkeypatch.setattr(ctf, 'BF_ENGAGE', engage)
  monkeypatch.setattr(ctf, 'RELEASE_WITH_BUFFER', release)
  monkeypatch.setattr(ctf, 'APPROACH_RATE', rate)
  assert broken in violations(chain({"LeadBrakeFollow": 1, "ApproachCoast": 1})), name


def test_the_vector_counts_what_it_says():
  a = np.array([0.0, 0.5, 0.0, -0.5, 0.0, 0.5, -0.5])
  v = rq.vector([rq.components(a, [20.0] * 7, [30.0] * 7, [20.0] * 7, [0.0] * 7, 10.0)])
  assert v['minutes'] == pytest.approx(70 / 60)
  assert v['brake_on'] * v['minutes'] == 2 and v['accel_on'] * v['minutes'] == 2 and v['rev'] * v['minutes'] == 3
  assert v['ubon'] * v['minutes'] == 2          # the lead never braked
  braking_lead = rq.components(a, [20.0] * 7, [30.0] * 7, [20.0] * 7, [-1.0] * 7, 10.0)
  assert braking_lead['ubon'] == 0 and braking_lead['calm'] == 0


def test_lead_brake_scenes_are_not_softened(chain):
  """Smoothness must not be bought by braking less when the lead really brakes."""
  cfg = chain()
  for name, rows in load('leadbrake_').items():
    out, inp = replay(rows, cfg), rows[:, 1]
    assert out.min() <= inp.min() + 0.05, name
    first_in = np.argmax(inp < -0.5) if (inp < -0.5).any() else len(inp)
    first_out = np.argmax(out < -0.5) if (out < -0.5).any() else len(out)
    assert first_out <= first_in + 2, name


def test_brake_follow_still_acts_on_a_hard_stop(chain):
  """A lead stopping at 3 m/s^2 from 20 m/s, 30 m ahead, while the planner is still at -0.5."""
  cfg = chain()
  if int(cfg["LeadBrakeFollow"]) != 1:
    pytest.skip("brake-follow off by default")
  sh = fs.FollowShaping(DT)
  out = [sh.update(-0.5, NS(present=True, radar=True, dRel=30.0, vLead=18.0, aLeadK=-3.0),
                   NS(present=False, radar=False, dRel=0.0, vLead=0.0, aLeadK=0.0), 20.0, STOP_DISTANCE, 28.0, 1.0,
                   int(cfg["ComfortBrake"]) / 100.0, COMFORT_BRAKE_2) for _ in range(20)]
  assert min(out) < -1.5


def test_lead_ahead_cannot_engage_brake_follow():
  """LeadAheadBrake lowers the lead's aLeadK to at most its floor; that alone must never reach the
  brake-follow band, or a slowing car two ahead turns into brake taps (the 11/12 interaction)."""
  floor = min(la.MAX_DECEL_V)
  assert floor >= ctf.BF_ENGAGE[1]
