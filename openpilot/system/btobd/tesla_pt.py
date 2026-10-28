"""Charging, thermal and identity broadcasts on the diagnostic port, summarised from a saved capture.

The diag connector carries the powertrain bus plus messages the gateway forwards from the ETH side
(2026-10-27 capture: 17/17 PT IDs matched talas9 by ID and length, 22 ETH). Signal positions are
from talas9/tesla_can_signals (Model S, PT and ETH tables). They were checked on that one capture,
taken parked and charging on AC, against each other: pilot +6 V / -11.8 V is J1772 state C, and
pack power over the AC volts gives a believable 28 A. Firmware changes can move fields, so each row
keeps its ID and frame count.

VIN, clock and firmware strings were decoded by hand from the same capture. The serial part of the
VIN is masked here and in anything the page shows.
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone

from openpilot.system.btobd.tesla_sx import TeslaSXState


@dataclass(frozen=True)
class Sig:
  key: str
  label: str
  msg: int
  start: int
  width: int
  scale: float = 1.0
  offset: float = 0.0
  unit: str = ''
  enum: dict = field(default_factory=dict)
  sna: int | None = None             # raw "signal not available" value
  mux: tuple[int, int, int] | None = None  # (start, width, value) of the muxer this signal needs
  signed: bool = False
  bad: tuple = ()                    # enum values that deserve a warning
  min_len: int = 1


BMS_STATE = {0: '대기', 1: '주행', 2: '지원', 3: '완속 충전', 4: '급속 충전', 5: '충전기 전압 맞춤',
             6: '폴트 해제', 7: '폴트', 8: '용접 검사', 15: 'SNA'}
PILOT = {0: '없음', 1: '폴트', 2: '완속 충전', 3: '급속 충전', 4: '대기', 5: '무효', 6: '미사용', 7: 'SNA'}
PROXIMITY = {0: 'SNA', 1: '분리됨', 2: '꽂힘 · 잠기지 않음', 3: '꽂힘 · 잠김'}
CP_TYPE = {0: '북미 Tesla', 1: '유럽 IEC', 2: '중국 GB'}
FC_CONTACTOR = {0: '열림', 1: '닫힘', 2: '용접됨', 3: '정적 시험 통과', 4: '초기화', 7: 'SNA'}
PHASE = {0: '단상', 1: '3상', 2: '2상 델타(모듈 2개)', 3: '2상 델타(모듈 3개)', 4: 'SNA'}
HEATER = {0: '꺼짐', 1: '시작', 2: '가열 점검', 3: '가동', 4: '과열', 5: '일시 중지', 6: 'HV 제한', 7: 'SNA'}
HVIL = {0: '끊김', 1: '정상'}
INPOS = {0: '이동 중', 1: '위치 도달'}

CHARGE = [
  Sig('bmsState', 'BMS 상태', 0x212, 20, 4, enum=BMS_STATE, bad=(7, 8), min_len=3),
  Sig('pilot', '충전 파일럿', 0x77D, 24, 3, enum=PILOT, bad=(1, 5), min_len=4),
  Sig('proximity', '커넥터 근접', 0x77D, 27, 2, enum=PROXIMITY, min_len=4),
  Sig('pilotHigh', '파일럿 전압 (high)', 0x51F, 40, 12, 0.01, unit='V', mux=(0, 2, 2), signed=True, min_len=7),
  Sig('pilotLow', '파일럿 전압 (low)', 0x51F, 28, 12, 0.01, unit='V', mux=(0, 2, 2), signed=True, min_len=7),
  Sig('acVolts1', '충전기 1 AC 입력', 0x207, 8, 9, unit='V', min_len=3),
  Sig('acVolts2', '충전기 2 AC 입력', 0x209, 8, 9, unit='V', min_len=3),
  Sig('acVolts3', '충전기 3 AC 입력', 0x20B, 8, 9, unit='V', min_len=3),
  Sig('phase', '입력 상', 0x21C, 29, 3, enum=PHASE, min_len=4),
  Sig('hvil1', 'HVIL (충전기 1)', 0x42C, 37, 1, enum=HVIL, bad=(0,), min_len=5),
  Sig('hvil2', 'HVIL (충전기 2)', 0x43C, 37, 1, enum=HVIL, bad=(0,), min_len=5),
  Sig('hvil3', 'HVIL (충전기 3)', 0x44C, 37, 1, enum=HVIL, bad=(0,), min_len=5),
  Sig('fcContactor', '급속충전 접촉기', 0x25C, 3, 3, enum=FC_CONTACTOR, bad=(2,), min_len=1),
  Sig('portType', '충전 포트 종류', 0x20E, 2, 2, enum=CP_TYPE, min_len=1),
]

THERMAL = [
  Sig('heaterState', '배터리 히터', 0x26A, 20, 3, enum=HEATER, bad=(4,), min_len=3),
  Sig('heaterTemp', '히터 온도', 0x26A, 0, 11, 0.125, -40, '°C', sna=2047, min_len=2),
  Sig('pump1', '배터리 펌프 1', 0x32A, 0, 7, unit='%', sna=127),
  Sig('pump2', '배터리 펌프 2', 0x32A, 8, 7, unit='%', sna=127, min_len=2),
  Sig('ptPump', '파워트레인 펌프', 0x32A, 16, 7, unit='%', sna=127, min_len=3),
  Sig('pump1Req', '배터리 펌프 1 요청', 0x39A, 32, 7, unit='%', sna=127, min_len=5),
  Sig('heaterPct', '히터 출력', 0x32A, 40, 7, unit='%', sna=127, min_len=6),
  Sig('radBypass', '라디에이터 바이패스', 0x37A, 24, 7, unit='%', sna=127, min_len=4),
  Sig('chillerBypass', '칠러 바이패스', 0x37A, 32, 7, unit='%', sna=127, min_len=5),
  Sig('compCooling', '컴프레서 냉방 요청', 0x20A, 16, 7, unit='%', sna=127, min_len=3),
  Sig('refrigPressure', '냉매 압력', 0x20A, 40, 8, 0.125, unit='bar', sna=255, min_len=6),
  Sig('dischargeTemp', '냉매 토출 온도', 0x33A, 0, 11, 0.125, -40, '°C', sna=2047, min_len=2),
  Sig('suctionTemp', '냉매 흡입 온도', 0x33A, 16, 8, 0.5, -40, '°C', sna=255, min_len=3),
  Sig('suctionPressure', '냉매 흡입 압력', 0x33A, 24, 7, 0.125, unit='bar', sna=127, min_len=4),
]

# Seen on 2026-10-27 with plausible scales but no second source yet.
CANDIDATES = [
  Sig('tpms1', '타이어 공기압? 1', 0x3F8, 0, 16, 1 / 16, unit='psi', min_len=2),
  Sig('tpms2', '타이어 공기압? 2', 0x3F8, 16, 16, 1 / 16, unit='psi', min_len=4),
  Sig('tpms3', '타이어 공기압? 3', 0x3F8, 32, 16, 1 / 16, unit='psi', min_len=6),
  Sig('tpms4', '타이어 공기압? 4', 0x3F8, 48, 16, 1 / 16, unit='psi', min_len=8),
  Sig('temp467', '온도? (0x467)', 0x467, 0, 16, 0.01, unit='°C', min_len=2),
  Sig('temp469', '온도? (0x469)', 0x469, 0, 16, 0.01, unit='°C', min_len=2),
  Sig('temp46B', '온도? (0x46B)', 0x46B, 0, 16, 0.01, unit='°C', min_len=2),
]

# 0x246 UI_solarData, same layout as opendbc tesla_can.dbc (the gateway forwards it to this port too).
# On 2026-10-27 15:50 UTC it read azimuth 119 / elevation 14, the sun's position over the Bay Area at
# that moment; on a chassis-bus drive log azimuth - carRef matched the GPS bearing within 2 degrees.
SOLAR = [
  Sig('sunElevation', '태양 고도', 0x246, 32, 8, unit='°', signed=True, min_len=5),
  Sig('sunAzimuth', '태양 방위각', 0x246, 0, 16, unit='°', signed=True, min_len=2),
  Sig('sunCarRef', '태양 방향 (차 기준)', 0x246, 16, 9, unit='°', signed=True, min_len=3),
]
COMPASS = ['북', '북북동', '북동', '동북동', '동', '동남동', '남동', '남남동', '남', '남남서', '남서', '서남서', '서', '서북서', '북서', '북북서']

VIN_IDS = (0x508, 0x538)
CLOCK_ID = 0x528
FW_STRING_IDS = (0x542, 0x552, 0x558)
BMS_VERSION_ID = 0x512
VIN_YEARS = dict(zip('ABCDEFGHJKLMNPRSTVWXY', range(2010, 2031), strict=True))
VIN_PLANTS = {'F': '프리몬트', 'P': '팔로알토', 'A': '오스틴', 'B': '베를린', 'C': '상하이', 'R': '리노'}
VIN_MODELS = {'S': 'Model S', 'X': 'Model X', '3': 'Model 3', 'Y': 'Model Y'}


def _raw(d: bytes, start: int, width: int) -> int:
  """Little-endian (Intel) field, the only byte order these tables use."""
  return (int.from_bytes(d.ljust(8, b'\0'), 'little') >> start) & ((1 << width) - 1)


def decode(s: Sig, d: bytes):
  """Raw-scaled value of one signal, or None when the frame doesn't carry it."""
  if len(d) < s.min_len or (s.mux and _raw(d, s.mux[0], s.mux[1]) != s.mux[2]):
    return None
  v = _raw(d, s.start, s.width)
  if s.sna is not None and v == s.sna:
    return None
  if s.signed and v >> (s.width - 1):
    v -= 1 << s.width
  return v if s.enum else v * s.scale + s.offset


def _fmt(v: float) -> str:
  return f'{v:.4g}' if abs(v) < 1000 else f'{v:.0f}'


def _row(s: Sig, vals: list) -> dict:
  row = {'key': s.key, 'label': s.label, 'id': f'{s.msg:03X}', 'n': len(vals), 'unit': s.unit, 'status': 'ok'}
  if not vals:
    return {**row, 'value': '–', 'detail': '들리지 않음', 'status': 'na'}
  last = vals[-1]
  if s.enum:
    seen = list(dict.fromkeys(vals))
    row['value'] = s.enum.get(last, f'값 {last}')
    row['detail'] = '' if len(seen) == 1 else '바뀜: ' + ' → '.join(s.enum.get(v, str(v)) for v in seen)
    row['status'] = 'bad' if any(v in s.bad for v in vals) else 'ok'
  else:
    lo, hi = min(vals), max(vals)
    row['value'] = _fmt(last)
    row['detail'] = '' if hi - lo < 1e-9 else f'처음 {_fmt(vals[0])} → 끝 {_fmt(last)} · 범위 {_fmt(lo)}~{_fmt(hi)}'
  return row


def mask_vin(vin: str) -> str:
  return vin[:11] + '•' * (len(vin) - 11)


def _vin(frames_by_id: dict) -> tuple[str, int]:
  """VIN from 0x508 or 0x538: byte 0 is the part index (0/1/2), bytes 1-7 seven ASCII characters."""
  for i in VIN_IDS:
    parts = {}
    for _, d in frames_by_id.get(i, ()):
      if len(d) >= 2 and d[0] <= 2:
        parts[d[0]] = d[1:]
    if len(parts) == 3:
      text = b''.join(parts[k] for k in range(3)).decode('ascii', 'replace').replace('\0', '').strip()
      if len(text) >= 17 and text[:17].isalnum():
        return text[:17], i
  return '', 0


def _ascii(d: bytes) -> str:
  return ''.join(chr(b) for b in d if 32 <= b < 127)


def summarize(frames: list[tuple[float, int, bytes]], meta: dict | None = None) -> dict:
  """Group the useful broadcasts of one capture into rows the page can show as-is."""
  meta = meta or {}
  by_id: dict[int, list] = defaultdict(list)
  for t, i, d in frames:
    by_id[i].append((t, d))

  def rows(sigs):
    out = []
    for s in sigs:
      vals = [v for _, d in by_id.get(s.msg, ()) if (v := decode(s, d)) is not None]
      out.append(_row(s, vals))
    return out

  charge = rows(CHARGE)
  pack = TeslaSXState()
  for t, i, d in frames:
    if i == 0x102:
      pack.feed(i, d, t)
  pv, pc = pack.values.get('packVoltage'), pack.values.get('packCurrent')
  ac = [v for k in ('acVolts1', 'acVolts2', 'acVolts3') for r in charge if r['key'] == k and r['n'] for v in [float(r['value'])]]
  if pv is not None and pc is not None:
    kw = pv * pc / 1000
    charge.append({'key': 'packPower', 'label': '팩 전력 (0x102)', 'id': '102', 'n': len(by_id[0x102]), 'unit': 'kW',
                   'value': f'{kw:.2f}', 'detail': f'{pv:.1f} V × {pc:.1f} A', 'status': 'ok'})
    ac_v = [v for v in ac if v > 80]
    if ac_v and abs(kw) > 0.3:
      volts = sum(ac_v) / len(ac_v)
      charge.append({'key': 'acCurrent', 'label': 'AC 입력 전류 (추정)', 'id': '', 'n': 0, 'unit': 'A',
                     'value': f'{abs(kw) * 1000 / volts:.0f}', 'status': 'ok',
                     'detail': f'팩 전력 ÷ AC {volts:.0f} V, 충전기 손실 무시 (실제는 몇 % 더 큼)'})

  ident = []
  vin, vin_id = _vin(by_id)
  if vin:
    ident.append({'key': 'vin', 'label': 'VIN (뒤 6자리 가림)', 'id': f'{vin_id:03X}', 'n': len(by_id[vin_id]),
                  'value': mask_vin(vin), 'unit': '', 'status': 'ok',
                  'detail': ' · '.join(x for x in (VIN_MODELS.get(vin[3], ''), f'{VIN_YEARS[vin[9]]}년식' if vin[9] in VIN_YEARS else '',
                                                   VIN_PLANTS.get(vin[10], '')) if x)})
  else:
    ident.append({'key': 'vin', 'label': 'VIN', 'id': '508', 'n': len(by_id.get(0x508, ())), 'value': '–', 'unit': '',
                  'status': 'na', 'detail': '세 조각(0/1/2)을 다 듣지 못함'})
  clock = [(t, int.from_bytes(d[:4], 'big')) for t, d in by_id.get(CLOCK_ID, ()) if len(d) >= 4]
  if clock:
    t, car = clock[-1]
    # the page re-renders `ts` in the viewer's local time
    row = {'key': 'clock', 'label': '차량 시계', 'id': f'{CLOCK_ID:03X}', 'n': len(clock), 'ts': car,
           'value': datetime.fromtimestamp(car, timezone.utc).strftime('%m/%d %H:%M:%S'), 'unit': 'UTC', 'status': 'ok', 'detail': ''}
    end_t = max(f[0] for f in frames)
    if meta.get('saved'):
      skew = car - (meta['saved'] - end_t + t)
      row['detail'] = f'기기 시계와 {skew:+.0f} 초 차이 (캡처 저장 시각 기준, ±2초는 정상)'
      row['status'] = 'ok' if abs(skew) < 30 else 'warn'
    ident.append(row)
  for i in FW_STRING_IDS:
    fs = by_id.get(i, ())
    if fs:
      ident.append({'key': f'fw{i:03X}', 'label': '모듈 식별 문자열?', 'id': f'{i:03X}', 'n': len(fs),
                    'value': _ascii(fs[-1][1]) or '–', 'unit': '', 'status': 'ok', 'detail': 'HW/SW 식별자 또는 git 해시로 추정'})
  ver = [d[7] for _, d in by_id.get(BMS_VERSION_ID, ()) if len(d) >= 8 and d[7] != 255]
  if ver:
    ident.append({'key': 'bmsHw', 'label': 'BMS 하드웨어 ID', 'id': f'{BMS_VERSION_ID:03X}', 'n': len(ver),
                  'value': str(ver[-1]), 'unit': '', 'status': 'ok', 'detail': ''})

  where = rows(SOLAR)
  sun = [(az, ref) for _, d in by_id.get(0x246, ())
         if (az := decode(SOLAR[1], d)) is not None and (ref := decode(SOLAR[2], d)) is not None]
  if sun:
    heading = (sun[-1][0] - sun[-1][1]) % 360
    where.insert(0, {'key': 'heading', 'label': '차 방향', 'id': '246', 'n': len(sun), 'unit': '°', 'status': 'ok',
                     'value': f'{heading:.0f}', 'detail': f'{COMPASS[round(heading / 22.5) % 16]} · 태양 방위각 − 차 기준 각도 (GPS 방향과 ±2° 확인)'})

  groups = [
    {'key': 'charge', 'title': '충전', 'rows': charge,
     'note': 'BMS 상태·파일럿·근접·HVIL이 충전이 안 되거나 멈출 때 원인을 가리킵니다. J1772 파일럿은 +9 V 꽂힘, +6 V 충전 중, +3 V 환기 필요입니다.'},
    {'key': 'thermal', 'title': '배터리 열관리', 'rows': rows(THERMAL),
     'note': '급속충전·고속주행 때 펌프·칠러·컴프레서가 도는지 봅니다. 주차 중엔 대부분 쉬는 값입니다.'},
    {'key': 'ident', 'title': '차량 식별 · 시각', 'rows': ident, 'note': ''},
    {'key': 'where', 'title': '방향 · 태양', 'rows': where,
     'note': '차 화면이 위치로 계산해 내보내는 값입니다. 차 방향은 주차 중에도 나옵니다. 태양 고도가 음수면 해가 진 뒤입니다.'},
    {'key': 'candidate', 'title': '후보 (확인 필요)', 'rows': rows(CANDIDATES),
     'note': '0x3F8은 계기판 타이어 공기압 화면과, 0x467~0x46B는 실내/외기 온도와 한 번 대조하면 확정됩니다.'},
  ]
  for g in groups:
    g['heard'] = sum(r['status'] != 'na' for r in g['rows'])
  return {'groups': groups, 'frames': len(frames), 'protocol': meta.get('protocol'), 'filter': meta.get('filter'),
          'saved': meta.get('saved'), 'seconds': meta.get('seconds')}
