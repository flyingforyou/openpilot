"""Parked-car checks on the listen-only Model S/X data from the diagnostic-port dongle.

Everything here is a pure function of what tesla_sx.TeslaSXState already decoded: nothing is
requested from the car. Each item carries the rule it was judged by, so the page can print the
criterion next to the verdict. Thresholds are rules of thumb for 18650-cell packs and for how this
data is reported; they are meant to point at things worth a closer look, not to condemn a part.
"""
from __future__ import annotations

OK, WARN, BAD, NA = 'ok', 'warn', 'bad', 'na'

CELL_V_OK = (3.0, 4.25)        # a healthy cell sits in here at any SOC
CELL_V_BAD = (2.7, 4.30)
BALANCE_OK_MV, BALANCE_BAD_MV = 20.0, 40.0
REST_CURRENT_A = 10.0          # the spread only means something near zero current
WEAK_CELL_MV = 30.0
PACK_VS_CELLS = (1.0, 3.0)     # percent
DCDC_V = (12.0, 14.8)
DCDC_VS_INVERTER_V = (0.8, 1.5)
MODULE_SPREAD_C = (5.0, 10.0)
TERMINAL_VS_MODULES_C = (8.0, 15.0)
DRIVE_TEMP_C = (85.0, 105.0)
DISCHARGE_LIMIT_KW = (100.0, 50.0)   # (warn below, bad below), only judged warm and not near empty
DEGRADATION_PCT = (10.0, 20.0)
KEY_SIGNAL_MAX_AGE_S = 15.0


def _item(id_, group, label, status, value, rule, note=''):
  return {'id': id_, 'group': group, 'label': label, 'status': status, 'value': value, 'rule': rule, 'note': note}


def _grade(x, warn_at, bad_at):
  return OK if x < warn_at else WARN if x < bad_at else BAD


def evaluate(values: dict, cells_v: list, cells_t: list, age: dict, pack_new_kwh: float | None = None) -> dict:
  V = values
  out: list[dict] = []
  cv = [c for c in cells_v if c is not None]
  ct = [c for c in cells_t if c is not None]
  cur = V.get('packCurrent')
  at_rest = cur is not None and abs(cur) <= REST_CURRENT_A

  # --- signals first: a verdict on stale data is worse than none -----------------------------------
  G = '신호'
  missing = [k for k in ('packVoltage', 'socUI', 'nominalFullPackEnergy') if k not in V]
  stale = [k for k in ('packVoltage', 'socUI', 'nominalFullPackEnergy') if age.get(k, 0) > KEY_SIGNAL_MAX_AGE_S]
  if missing:
    out.append(_item('signals', G, '핵심 신호 수신', NA, '없음: ' + ', '.join(missing),
                     '팩 전압·SOC·공칭 용량이 최근 15초 안에 들어와야 함', '동글 연결과 시동(차 깨움) 상태를 확인하세요'))
  elif stale:
    out.append(_item('signals', G, '핵심 신호 수신', WARN, '끊김: ' + ', '.join(stale),
                     '팩 전압·SOC·공칭 용량이 최근 15초 안에 들어와야 함', '차가 잠들었거나 동글이 멈췄을 수 있습니다'))
  else:
    out.append(_item('signals', G, '핵심 신호 수신', OK, '정상', '팩 전압·SOC·공칭 용량이 최근 15초 안에 들어와야 함'))
  out.append(_item('cells_rx', G, '셀 데이터 수신', OK if len(cv) == 96 and len(ct) == 32 else NA,
                   f'전압 {len(cv)}/96 · 온도 {len(ct)}/32', '96개 셀 전압과 32개 모듈 온도를 모두 받아야 셀 점검 가능',
                   '' if len(cv) == 96 else '셀 값은 몇십 초에 걸쳐 차례로 들어옵니다'))

  # --- cells ----------------------------------------------------------------------------------------
  G = '배터리 셀'
  if cv:
    lo, hi = min(cv), max(cv)
    st = BAD if lo < CELL_V_BAD[0] or hi > CELL_V_BAD[1] else OK if CELL_V_OK[0] <= lo and hi <= CELL_V_OK[1] else WARN
    out.append(_item('cell_range', G, '셀 전압 범위', st, f'{lo:.3f} ~ {hi:.3f} V',
                     f'{CELL_V_OK[0]}~{CELL_V_OK[1]} V 안이면 정상, {CELL_V_BAD[0]} V 미만·{CELL_V_BAD[1]} V 초과는 위험',
                     '' if st == OK else '범위를 벗어난 셀이 있습니다. 셀 번호는 진단 페이지의 격자에서 확인하세요'))
  else:
    out.append(_item('cell_range', G, '셀 전압 범위', NA, '–', f'{CELL_V_OK[0]}~{CELL_V_OK[1]} V 안이면 정상'))
  if len(cv) == 96 and V.get('cellDelta') is not None:
    mv = V['cellDelta'] * 1000
    if at_rest:
      out.append(_item('balance', G, '셀 밸런스 (최고−최저)', _grade(mv, BALANCE_OK_MV, BALANCE_BAD_MV), f'{mv:.0f} mV',
                       f'{BALANCE_OK_MV:.0f} mV 미만 좋음 · {BALANCE_BAD_MV:.0f} mV 미만 보통 · 이상 나쁨 (전류 ±{REST_CURRENT_A:.0f} A 이내에서만)',
                       'SOC가 아주 낮거나(20% 미만) 높을 때(90% 이상) 편차가 잘 드러납니다'))
    else:
      out.append(_item('balance', G, '셀 밸런스 (최고−최저)', NA, f'{mv:.0f} mV (부하 중)',
                       f'전류 ±{REST_CURRENT_A:.0f} A 이내에서만 판정', '공조·충전 등을 끄고 다시 점검하세요'))
    avg = V.get('cellAvg')
    weak = [(i + 1, (avg - c) * 1000) for i, c in enumerate(cells_v) if c is not None and avg is not None and (avg - c) * 1000 >= WEAK_CELL_MV]
    out.append(_item('weak', G, '약한 셀', BAD if weak else OK,
                     ', '.join(f'#{n} (−{d:.0f} mV)' for n, d in weak[:8]) if weak else '없음',
                     f'평균보다 {WEAK_CELL_MV:.0f} mV 이상 낮은 셀', '같은 셀이 매번 나오면 그 셀의 노화를 의심할 수 있습니다' if weak else ''))
  else:
    for id_, label in (('balance', '셀 밸런스 (최고−최저)'), ('weak', '약한 셀')):
      out.append(_item(id_, G, label, NA, '–', '96개 셀을 모두 받아야 판정'))

  # --- consistency ----------------------------------------------------------------------------------
  G = '전압·전류 일관성'
  if len(cv) == 96 and V.get('packVoltage'):
    s = sum(cv)
    pct = abs(s - V['packVoltage']) / V['packVoltage'] * 100
    out.append(_item('pack_vs_cells', G, '팩 전압 ↔ 셀 합', _grade(pct, *PACK_VS_CELLS),
                     f'팩 {V["packVoltage"]:.1f} V · 셀 합 {s:.1f} V · 차이 {pct:.2f}%',
                     f'차이 {PACK_VS_CELLS[0]}% 미만 정상 · {PACK_VS_CELLS[1]}% 미만 주의 · 이상 이상',
                     '' if pct < PACK_VS_CELLS[0] else '전압 센서나 셀 측정선, 또는 해독이 어긋났을 수 있습니다'))
  else:
    out.append(_item('pack_vs_cells', G, '팩 전압 ↔ 셀 합', NA, '–', '96개 셀을 모두 받아야 비교 가능'))
  if cur is not None:
    out.append(_item('rest', G, '휴지 상태 (전류)', OK if at_rest else NA, f'{cur:.1f} A',
                     f'±{REST_CURRENT_A:.0f} A 이내면 휴지 상태', '' if at_rest else '부하가 있어 일부 판정을 보류합니다'))
  v12 = V.get('dcdcOutputVoltage')
  if v12 is not None:
    st = OK if DCDC_V[0] <= v12 <= DCDC_V[1] else WARN
    out.append(_item('dcdc', G, '12V 계통 (DC-DC 출력)', st, f'{v12:.1f} V', f'{DCDC_V[0]}~{DCDC_V[1]} V',
                     '' if st == OK else '12V 보조 배터리나 DC-DC 컨버터를 확인해 볼 만합니다'))
    inv = V.get('rearInverter12V')
    if inv is not None:
      d = abs(v12 - inv)
      out.append(_item('dcdc_vs_inv', G, '12V 일관성 (DC-DC ↔ 인버터)', _grade(d, *DCDC_VS_INVERTER_V), f'차이 {d:.1f} V',
                       f'{DCDC_VS_INVERTER_V[0]} V 미만 정상 · {DCDC_VS_INVERTER_V[1]} V 미만 주의', '12V 배선 접촉 불량이면 벌어집니다'))
  else:
    out.append(_item('dcdc', G, '12V 계통 (DC-DC 출력)', NA, '–', f'{DCDC_V[0]}~{DCDC_V[1]} V'))

  # --- temperatures ---------------------------------------------------------------------------------
  G = '온도'
  if len(ct) >= 8:
    spread = max(ct) - min(ct)
    out.append(_item('module_spread', G, '모듈 온도 편차', _grade(spread, *MODULE_SPREAD_C), f'{spread:.1f} °C ({min(ct):.1f} ~ {max(ct):.1f})',
                     f'{MODULE_SPREAD_C[0]:.0f} °C 미만 좋음 · {MODULE_SPREAD_C[1]:.0f} °C 미만 보통 · 이상 나쁨',
                     '주행이나 충전 직후에는 일시적으로 커집니다'))
    neg = V.get('negTerminalTemp')
    if neg is not None:
      d = neg - sum(ct) / len(ct)
      out.append(_item('terminal', G, '음극 단자 ↔ 모듈 평균', _grade(abs(d), *TERMINAL_VS_MODULES_C), f'{d:+.1f} °C (단자 {neg:.1f} °C)',
                       f'차이 {TERMINAL_VS_MODULES_C[0]:.0f} °C 미만 정상 · {TERMINAL_VS_MODULES_C[1]:.0f} °C 미만 주의',
                       '단자만 뜨거우면 접촉 저항이 의심됩니다' if d > TERMINAL_VS_MODULES_C[0] else ''))
  else:
    out.append(_item('module_spread', G, '모듈 온도 편차', NA, '–', '모듈 온도를 받아야 판정'))
  for key, label in (('inverterTemp', '인버터 온도'), ('motorTemp', '모터 온도')):
    t = V.get(key)
    if t is not None:
      out.append(_item(key, G, label, _grade(t, DRIVE_TEMP_C[0], DRIVE_TEMP_C[1]), f'{t:.0f} °C',
                       f'{DRIVE_TEMP_C[0]:.0f} °C 이하 정상 · {DRIVE_TEMP_C[1]:.0f} °C 이하 주의', '주차 중에는 보통 주변 온도에 가깝습니다'))

  # --- BMS limits -----------------------------------------------------------------------------------
  G = 'BMS 출력 제한'
  dis = V.get('maxDischargePower')
  warm = ct and min(ct) >= 15
  soc = V.get('socUI')
  if dis is not None and warm and soc is not None and soc >= 20:
    st = BAD if dis < DISCHARGE_LIMIT_KW[1] else WARN if dis < DISCHARGE_LIMIT_KW[0] else OK
    out.append(_item('discharge', G, '최대 방전 출력', st, f'{dis:.0f} kW',
                     f'모듈 15 °C 이상·SOC 20% 이상일 때 {DISCHARGE_LIMIT_KW[0]:.0f} kW 이상이 정상',
                     '' if st == OK else 'BMS가 출력을 제한하고 있습니다. 경고 직전 신호인 경우가 많습니다'))
  else:
    out.append(_item('discharge', G, '최대 방전 출력', NA, f'{dis:.0f} kW' if dis is not None else '–',
                     '모듈 15 °C 이상·SOC 20% 이상에서만 판정', '추울 때나 SOC가 낮을 때는 제한이 정상입니다'))

  # --- degradation ----------------------------------------------------------------------------------
  G = '열화'
  nfp = V.get('nominalFullPackEnergy')
  if nfp is not None and pack_new_kwh:
    d = max(0.0, (1 - nfp / pack_new_kwh) * 100)
    out.append(_item('degradation', G, '용량 열화', _grade(d, *DEGRADATION_PCT), f'{d:.1f}% ({nfp:.1f} / {pack_new_kwh:.1f} kWh)',
                     f'{DEGRADATION_PCT[0]:.0f}% 미만 좋음 · {DEGRADATION_PCT[1]:.0f}% 미만 보통 · 이상 나쁨',
                     'BMS 추정치라 몇 %는 충전·방전 패턴에 따라 오르내립니다'))
  else:
    out.append(_item('degradation', G, '용량 열화', NA, f'{nfp:.1f} kWh' if nfp is not None else '–',
                     '신품 기준 용량을 진단 페이지에서 정해야 계산', '팩 종류를 고르면 열화를 계산합니다'))

  counts = {s: sum(1 for i in out if i['status'] == s) for s in (OK, WARN, BAD, NA)}
  overall = BAD if counts[BAD] else WARN if counts[WARN] else OK if counts[OK] else NA
  return {'items': out, 'counts': counts, 'overall': overall}


def parked(state: dict, values: dict) -> tuple[bool, str]:
  """Whether the car is parked: openpilot off, not moving, and in P when the dongle says so."""
  if state.get('engaged'):
    return False, 'openpilot이 켜져 있습니다'
  if abs(state.get('vEgo') or 0.0) > 0.3:
    return False, '차가 움직이고 있습니다'
  gear = values.get('gear')
  if gear is not None and gear != 'P':
    return False, f'기어가 P가 아닙니다 (현재 {gear})'
  return True, ''


def record(report: dict, values: dict, t: float) -> dict:
  """One line of history: the verdict plus the few numbers worth trending."""
  keys = ('socUI', 'packVoltage', 'cellMin', 'cellMax', 'cellDelta', 'nominalFullPackEnergy', 'moduleTempMin', 'moduleTempMax')
  return {'t': t, 'overall': report['overall'], 'counts': report['counts'], 'metrics': {k: values.get(k) for k in keys if k in values}}
