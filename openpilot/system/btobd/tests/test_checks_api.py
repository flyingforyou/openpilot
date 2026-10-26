"""The /api/checks handler: the parked gate lives on the server, so a stale tab or a hand-made
request cannot run the check while driving."""
import json
import sys
import tempfile
import time
import types
from types import SimpleNamespace

import pytest

sys.modules.setdefault('av', types.ModuleType('av'))   # only video_source needs it
import openpilot.selfdrive.debug.tuning_server as ts  # noqa: E402
from openpilot.system.btobd.tests.test_checks import healthy  # noqa: E402


class _Params:
  def get(self, k, return_default=False):
    return {'BtObdPackNewKwh': '85.8'}.get(k, '')


@pytest.fixture
def api(monkeypatch):
  v, cells, temps, age = healthy()
  v['gear'] = 'P'
  dev = {'updated': time.time(), 'bt': 'up', 'link': 'connected', 'values': v, 'age': age, 'cellsV': cells, 'cellsT': temps}
  for name in ('OBD_STATE_PATH', 'OBD_VIEWER_PATH', 'CHECK_HISTORY_PATH'):
    monkeypatch.setattr(ts, name, tempfile.mktemp())
  h = SimpleNamespace(state=SimpleNamespace(get=lambda: dict(engaged=False, vEgo=0.0)), params=_Params())
  h._touch_obd_viewer = ts.Handler._touch_obd_viewer
  h._obd_state = lambda: ts.Handler._obd_state(h)

  def write():
    json.dump(dev, open(ts.OBD_STATE_PATH, 'w'))
  write()
  return SimpleNamespace(h=h, dev=dev, v=v, cells=cells, write=write,
                         check=lambda save=False: ts.Handler._checks(h, save))


def test_parked_run_is_saved(api):
  r = api.check(True)
  assert r['parked'] and r['saved'] and r['report']['overall'] == 'ok'
  assert len(ts._read_check_history()) == 1


@pytest.mark.parametrize('state', [dict(engaged=False, vEgo=8.0), dict(engaged=True, vEgo=0.0)])
def test_moving_or_engaged_is_refused_and_not_recorded(api, state):
  api.h.state = SimpleNamespace(get=lambda: state)
  r = api.check(True)
  assert not r['parked'] and 'report' not in r and not ts._read_check_history()


def test_not_in_park_is_refused(api):
  api.v['gear'] = 'D'
  api.write()
  assert not api.check(True)['parked']


def test_waits_for_all_cells_before_saving(api):
  api.dev['cellsV'] = api.cells[:50] + [None] * 46
  api.write()
  r = api.check(True)
  assert r['waiting'] and not r.get('saved') and not ts._read_check_history()


def test_stale_dongle_data_is_not_judged(api):
  api.dev['updated'] = time.time() - 60
  api.write()
  r = api.check(True)
  assert r['parked'] and 'report' not in r


@pytest.mark.parametrize('state', [dict(engaged=False, vEgo=8.0), dict(engaged=True, vEgo=0.0)])
def test_capture_is_refused_unless_parked(api, state):
  api.h.state = SimpleNamespace(get=lambda: state)
  code, body = ts.Handler._sniff_request(api.h, {'seconds': 10, 'protocol': 6})
  assert code == 409 and 'error' in body


def test_capture_request_is_validated(api):
  ok = lambda req: ts.Handler._sniff_request(api.h, req)
  assert ok({'seconds': 10, 'protocol': 6}) == (200, {'seconds': 10.0, 'protocol': 6})
  assert ok({'seconds': 61})[0] == 400 and ok({'seconds': 1})[0] == 400
  assert ok({'protocol': 7})[0] == 400                      # only the 11-bit CAN modes are offered
  assert ok({'seconds': 'abc'})[0] == 400
