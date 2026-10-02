import numpy as np
from types import SimpleNamespace as NS

import openpilot.selfdrive.controls.lib.longitudinal_mpc_carrot.long_mpc as M

T6 = M.LEAD_MODEL_T


def mpc(v_ego=10.0):
  m = M.LongitudinalMpc.__new__(M.LongitudinalMpc)
  m.x0 = np.array([0.0, v_ego, 0.0])
  m.lead_model_predict = True
  m.lead_model_used = []
  return m


def radar(d=20.0, v=10.0, a=0.0, radar=True):
  return NS(present=True, radar=radar, dRel=d, vLead=v, aLeadK=a, aLeadTau=1.5)


def model(d=20.0, v=10.0, dv=(0, 0, 0, 0, 0, 0), prob=0.9):
  vv = np.array([v + x for x in dv])
  xx = d + M.LEAD_MODEL_RADAR_TO_CAMERA + np.concatenate(([0], np.cumsum(np.diff(T6) * (vv[:-1] + vv[1:]) / 2)))
  return NS(prob=prob, x=list(xx), v=list(vv))


def test_falls_back_when_not_the_same_car_or_unsure():
  m = mpc()
  assert m.model_lead_traj(radar(), model(d=30.0)) is None           # 10 m apart
  assert m.model_lead_traj(radar(), model(v=14.0)) is None           # 4 m/s apart
  assert m.model_lead_traj(radar(), model(prob=0.3)) is None
  assert m.model_lead_traj(radar(radar=False), model()) is None      # vision-only lead
  assert m.model_lead_traj(radar(v=1.0), model(v=1.0)) is None       # stopped lead: radar
  assert m.model_lead_traj(radar(), None) is None


def test_starts_at_radar_and_adds_the_model_change():
  m = mpc()
  xv = m.model_lead_traj(radar(d=20.0, v=10.0), model(d=21.0, v=9.2, dv=(0, -1, -2, -2, -2, -2)))
  assert xv[0, 0] == 20.0 and xv[0, 1] == 10.0                       # model's own offsets cancel
  assert np.interp(2.0, M.T_IDXS, xv[:, 1]) == np.float64(9.0)


def test_cautious_mode_never_puts_the_lead_further_than_radar():
  m = mpc()
  lead = radar(d=20.0, v=10.0, a=-1.0)
  xv, _ = m.process_lead(lead, 0.0, model(d=20.0, v=10.0, dv=(0, 2, 4, 4, 4, 4)))
  rx, _ = m.radar_lead_traj(lead, 0.0)
  assert np.all(xv <= rx + 1e-9)
  assert m.lead_model_used == [True]


def test_off_is_the_radar_path():
  m = mpc(); m.lead_model_predict = False
  lead = radar(a=-1.0)
  xv, _ = m.process_lead(lead, 0.0, model(dv=(0, -3, -6, -8, -8, -8)))
  rx, _ = m.radar_lead_traj(lead, 0.0)
  assert np.array_equal(xv, rx) and m.lead_model_used == []
