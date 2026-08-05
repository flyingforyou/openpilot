from openpilot.common.params import Params


class TypedParams:
  """carrot reads params through get_int/get_float; this tree's Params.get already returns the
  type declared in params_keys.h. Thin adapter so the ported code stays as written."""

  def __init__(self):
    self._p = Params()

  def _raw(self, key):
    return self._p.get(key, return_default=True)

  def get_int(self, key) -> int:
    v = self._raw(key)
    return int(v) if v is not None else 0

  def get_float(self, key) -> float:
    v = self._raw(key)
    return float(v) if v is not None else 0.0

  def get_bool(self, key) -> bool:
    return bool(self._p.get_bool(key))

  def __getattr__(self, name):
    return getattr(self._p, name)

import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.selfdrive.controls.lib.carrot_t_follow import ramp_t_follow
from openpilot.selfdrive.selfdrived.events import Events

EventName = log.OnroadEvent.EventName
LaneChangeState = log.LaneChangeState
