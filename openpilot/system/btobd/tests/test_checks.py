"""Check rules: start from a healthy parked pack and break one thing at a time."""
import copy

from openpilot.system.btobd import checks as c


def healthy():
  cells = [3.800 + (i % 5) * 0.001 for i in range(96)]       # 4 mV spread
  temps = [22.0 + (i % 3) * 0.3 for i in range(32)]
  v = {'packVoltage': round(sum(cells), 2), 'packCurrent': 0.4, 'socUI': 60.0, 'socMin': 59.5,
       'nominalFullPackEnergy': 84.0, 'maxDischargePower': 320.0, 'maxRegenPower': 60.0,
       'cellMin': min(cells), 'cellMax': max(cells), 'cellAvg': sum(cells) / 96, 'cellDelta': max(cells) - min(cells),
       'cellCount': 96, 'negTerminalTemp': 22.5, 'dcdcOutputVoltage': 13.6, 'rearInverter12V': 13.5,
       'inverterTemp': 24, 'motorTemp': 25}
  age = {k: 1.0 for k in v}
  return v, cells, temps, age


def run(mut=None, ref=85.8):
  v, cells, temps, age = healthy()
  if mut:
    mut(v, cells, temps, age)
  return {i['id']: i for i in c.evaluate(v, cells, temps, age, ref)['items']}


def test_healthy_pack_has_nothing_bad_or_warn():
  rep = c.evaluate(*healthy(), 85.8)
  assert rep['overall'] == c.OK
  assert rep['counts'][c.BAD] == 0 and rep['counts'][c.WARN] == 0


def test_balance_thresholds_and_rest_gate():
  def spread(mv, cur=0.4):
    def f(v, cells, t, a):
      cells[0] = max(cells) - mv / 1000
      v['cellDelta'] = max(cells) - min(cells)
      v['packCurrent'] = cur
    return f
  assert run(spread(15))['balance']['status'] == c.OK
  assert run(spread(30))['balance']['status'] == c.WARN
  assert run(spread(55))['balance']['status'] == c.BAD
  # same spread under load is held back, not failed
  assert run(spread(55, cur=-85.0))['balance']['status'] == c.NA


def test_weak_cell_is_named():
  def f(v, cells, t, a):
    cells[41] -= 0.060
    v['cellAvg'] = sum(cells) / 96
  it = run(f)['weak']
  assert it['status'] == c.BAD and '#42' in it['value']


def test_cell_range():
  def low(v, cells, t, a): cells[3] = 2.6
  def edge(v, cells, t, a): cells[3] = 2.95
  assert run(low)['cell_range']['status'] == c.BAD
  assert run(edge)['cell_range']['status'] == c.WARN


def test_pack_voltage_must_match_cell_sum():
  assert run(lambda v, *_: v.update(packVoltage=v['packVoltage'] * 1.02))['pack_vs_cells']['status'] == c.WARN
  assert run(lambda v, *_: v.update(packVoltage=v['packVoltage'] * 1.06))['pack_vs_cells']['status'] == c.BAD


def test_12v_and_consistency():
  assert run(lambda v, *_: v.update(dcdcOutputVoltage=11.2))['dcdc']['status'] == c.WARN
  assert run(lambda v, *_: v.update(rearInverter12V=11.8))['dcdc_vs_inv']['status'] in (c.WARN, c.BAD)


def test_temperature_checks():
  def hot_module(v, cells, t, a): t[5] = 36.0
  assert run(hot_module)['module_spread']['status'] == c.BAD
  assert run(lambda v, *_: v.update(negTerminalTemp=40.0))['terminal']['status'] == c.BAD
  assert run(lambda v, *_: v.update(inverterTemp=110))['inverterTemp']['status'] == c.BAD


def test_discharge_limit_only_judged_when_warm_and_charged():
  assert run(lambda v, *_: v.update(maxDischargePower=70.0))['discharge']['status'] == c.WARN
  assert run(lambda v, *_: v.update(maxDischargePower=30.0))['discharge']['status'] == c.BAD
  def cold(v, cells, t, a):
    t[:] = [-3.0] * 32
    v['maxDischargePower'] = 30.0
  assert run(cold)['discharge']['status'] == c.NA        # cold packs are limited by design


def test_degradation_needs_a_reference():
  assert run(ref=None)['degradation']['status'] == c.NA
  assert run(ref=85.8)['degradation']['status'] == c.OK       # 84.0 / 85.8 = 2.1%
  assert run(ref=110.0)['degradation']['status'] == c.BAD     # 84 / 110 = 23.6%


def test_stale_and_missing_signals():
  assert run(lambda v, c_, t, a: a.update(packVoltage=60.0))['signals']['status'] == c.WARN
  assert run(lambda v, c_, t, a: v.pop('socUI'))['signals']['status'] == c.NA


def test_incomplete_cells_hold_back_cell_verdicts():
  def f(v, cells, t, a): cells[10:] = [None] * 86
  r = run(f)
  assert r['cells_rx']['status'] == c.NA
  assert r['balance']['status'] == c.NA and r['weak']['status'] == c.NA and r['pack_vs_cells']['status'] == c.NA


def test_parked_gate():
  assert c.parked({'engaged': False, 'vEgo': 0.0}, {'gear': 'P'}) == (True, '')
  assert c.parked({'engaged': False, 'vEgo': 0.0}, {}) == (True, '')            # gear unknown: speed decides
  assert not c.parked({'engaged': True, 'vEgo': 0.0}, {'gear': 'P'})[0]
  assert not c.parked({'engaged': False, 'vEgo': 5.0}, {'gear': 'P'})[0]
  assert not c.parked({'engaged': False, 'vEgo': 0.0}, {'gear': 'D'})[0]
  assert not c.parked({'engaged': False, 'vEgo': 0.0}, {'gear': 'N'})[0]


def test_record_keeps_trend_numbers():
  v, cells, temps, age = healthy()
  rep = c.evaluate(v, cells, temps, age, 85.8)
  r = c.record(rep, v, 123.0)
  assert r['t'] == 123.0 and r['overall'] == c.OK and r['metrics']['cellDelta'] == v['cellDelta']
  assert copy.deepcopy(r) == r
