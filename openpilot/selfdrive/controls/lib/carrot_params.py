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
    # Not Params.get_bool: it reads the file and calls an absent one False, ignoring the default
    # declared in params_keys.h. get_int and get_float above already honour that default, so a
    # bool that had never been written was the one type here that silently disagreed with what
    # the header said it was -- TeslaMapCurveUseMap ships as "1" and read back as off.
    return bool(self._raw(key))    # None (no value, no default) is False, which is the old answer

  def __getattr__(self, name):
    return getattr(self._p, name)
