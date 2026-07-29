"""A readable vehicle status view, built from the same decoded CAN the /can page shows.

/can answers "which signal moved when I pressed that", which means it shows all 671 messages and
makes you find things. This answers "what is the car doing right now", so it is a fixed, curated
list: the handful of signals worth a glance while sitting in the car, then everything else.

Only signals actually observed on this car's chassis bus are listed -- tesla_can.dbc also defines
messages a Model X HW1 never sends (SDM1, DriverSeat, EPAS3P_sysStatus), and a row that is
permanently blank is worse than no row.
"""
from openpilot.selfdrive.debug.can_viewer import CanDecoder

# Enum labels come out of the DBC upper-cased. Anything not mapped falls through as-is, so an
# unexpected value stays visible rather than being hidden behind a friendly word.
DOOR = {'CLOSED': '닫힘', 'OPEN': '열림', 'INIT': '초기화', 'SNA': '없음'}
LATCH = {'CLS': '닫힘', 'OPN': '열림', 'NDEF0': '-', 'SNA': '없음'}
GEAR = {'DI_GEAR_P': 'P', 'DI_GEAR_R': 'R', 'DI_GEAR_N': 'N', 'DI_GEAR_D': 'D',
        'DI_GEAR_INVALID': '-', 'DI_GEAR_SNA': '없음'}
BUCKLE = {0: '미착용', 1: '착용', 2: '고장', 3: '없음'}
ONOFF = {0: '꺼짐', 1: '켜짐'}
CRUISE = {'OFF': '꺼짐', 'STANDBY': '대기', 'ENABLED': '작동', 'STANDSTILL': '정차 유지',
          'OVERRIDE': '오버라이드', 'FAULT': '고장', 'PRE_FAULT': '고장 직전', 'PRE_CANCEL': '해제 직전'}
LIGHT = {0: '꺼짐', 1: '켜짐', 2: '고장', 3: '없음'}
WIPER = {'OFF': '꺼짐', 'INTERVAL1': '간헐 1', 'INTERVAL2': '간헐 2', 'INTERVAL3': '간헐 3',
         'INTERVAL4': '간헐 4', 'STAGE1': '저속', 'STAGE2': '고속', 'SNA': '없음'}

# (label, address, signal, options)
#   unit/dec  -- numeric display
#   enum      -- map the DBC label (or the raw int when the DBC has no VAL_ table) to Korean
#   warn      -- displayed values that should stand out
SECTIONS = [
  ('core', '중요', [
    ('주행', [
      ('기어', 0x118, 'DI_gear', {'enum': GEAR}),
      ('속도', 0x155, 'ESP_vehicleSpeed', {'unit': 'km/h', 'dec': 1}),
      ('모터', 0x108, 'DI_motorRPM', {'unit': 'rpm', 'dec': 0}),
      ('가속 페달', 0x108, 'DI_pedalPos', {'unit': '%', 'dec': 0}),
      ('브레이크', 0x118, 'DI_brakePedal', {'enum': {'APPLIED': '밟음', 'NOT_APPLIED': '뗌'},
                                            'warn': {'밟음'}}),
      ('조향각', 0x003, 'StW_Angl', {'unit': '°', 'dec': 1}),
      ('주차 브레이크', 0x283, 'MPkBrk_Stat', {'enum': {'ENGG': '체결', 'RELS': '해제'},
                                               'warn': {'체결'}}),
    ]),
    ('크루즈', [
      ('상태', 0x368, 'DI_cruiseState', {'enum': CRUISE}),
      ('설정 속도', 0x368, 'DI_cruiseSet', {'unit': 'km/h', 'dec': 0}),
      ('차간 거리 단수', 0x045, 'DTR_Dist_Rq', {'enum': {f'ACC_DIST_{i}': f'{i}단' for i in range(1, 8)}}),
    ]),
    ('문 · 개폐', [
      ('운전석', 0x318, 'DOOR_STATE_FL', {'enum': DOOR, 'warn': {'열림'}}),
      ('조수석', 0x318, 'DOOR_STATE_FR', {'enum': DOOR, 'warn': {'열림'}}),
      ('뒷좌석 좌', 0x318, 'DOOR_STATE_RL', {'enum': DOOR, 'warn': {'열림'}}),
      ('뒷좌석 우', 0x318, 'DOOR_STATE_RR', {'enum': DOOR, 'warn': {'열림'}}),
      ('트렁크', 0x318, 'BOOT_STATE', {'enum': DOOR, 'warn': {'열림'}}),
      ('프렁크', 0x318, 'DOOR_STATE_FrontTrunk', {'enum': DOOR, 'warn': {'열림'}}),
      ('보닛', 0x283, 'EngHd_Stat', {'enum': LATCH, 'warn': {'열림'}}),
    ]),
    ('안전벨트', [
      ('운전석', 0x211, 'RCM_buckleDriverStatus', {'enum': BUCKLE, 'warn': {'미착용'}}),
      ('조수석', 0x211, 'RCM_bucklePassengerStatus', {'enum': BUCKLE, 'warn': {'미착용'}}),
      ('2열', 0x211, 'RCM_buckle2ndRowStatus', {'enum': BUCKLE}),
    ]),
    ('에어 서스펜션', [
      ('앞 좌', 0x10B, 'FL_Lvl', {'unit': 'mm', 'dec': 0}),
      ('앞 우', 0x10B, 'FR_Lvl', {'unit': 'mm', 'dec': 0}),
      ('뒤 좌', 0x10B, 'RL_Lvl', {'unit': 'mm', 'dec': 0}),
      ('뒤 우', 0x10B, 'RR_Lvl', {'unit': 'mm', 'dec': 0}),
    ]),
    ('온도', [
      ('실내', 0x283, 'AirTemp_Insd', {'unit': '°C', 'dec': 1}),
      ('외기', 0x283, 'AirTemp_Outsd', {'unit': '°C', 'dec': 1}),
    ]),
  ]),
  ('more', '그 외', [
    ('등화', [
      ('헤드램프 좌', 0x318, 'BC_headLightLStatus', {'enum': LIGHT}),
      ('헤드램프 우', 0x318, 'BC_headLightRStatus', {'enum': LIGHT}),
      ('방향지시등 좌', 0x318, 'BC_indicatorLStatus', {'enum': LIGHT}),
      ('방향지시등 우', 0x318, 'BC_indicatorRStatus', {'enum': LIGHT}),
      ('상향등', 0x283, 'HiBm_On', {'enum': ONOFF}),
      ('하향등 요청', 0x283, 'LoBm_On_Rq', {'enum': ONOFF}),
      ('조도 센서', 0x283, 'LgtSens_Night', {'enum': {'DAY': '주간', 'NIGHT': '야간'}}),
    ]),
    ('와이퍼', [
      ('스위치', 0x045, 'WprSw6Posn', {'enum': WIPER}),
      ('워셔', 0x045, 'WprWashSw_Psd', {'enum': {'NPSD': '없음', 'TIPWIPE': '한번', 'WASH': '분사',
                                                 'SNA': '없음'}}),
    ]),
    ('구동계', [
      ('운전자 토크', 0x108, 'DI_torqueDriver', {'unit': 'Nm', 'dec': 1}),
      ('모터 토크', 0x108, 'DI_torqueMotor', {'unit': 'Nm', 'dec': 1}),
      ('토크 추정', 0x118, 'DI_torqueEstimate', {'unit': 'Nm', 'dec': 1}),
      ('회생 표시등', 0x368, 'DI_regenLight', {'enum': ONOFF}),
      ('차량 홀드', 0x368, 'DI_vehicleHoldState', {}),
      ('시스템 상태', 0x368, 'DI_systemState', {}),
    ]),
    ('제동 · 안정성', [
      ('ABS 작동', 0x135, 'ESP_absBrakeEvent', {'enum': {'ACTIVE': '작동', 'NOT_ACTIVE': '해제'}}),
      ('브레이크 램프', 0x135, 'ESP_brakeLamp', {'enum': {'ON': '켜짐', 'OFF': '꺼짐'}}),
      ('ESP 경고등', 0x135, 'ESP_espFaultLamp', {'enum': {'ON': '켜짐', 'OFF': '꺼짐'}, 'warn': {'켜짐'}}),
      ('ABS 경고등', 0x135, 'ESP_absFaultLamp', {'enum': {'ON': '켜짐', 'OFF': '꺼짐'}, 'warn': {'켜짐'}}),
      ('ESP OFF 등', 0x135, 'ESP_espOffLamp', {'enum': {'ON': '켜짐', 'OFF': '꺼짐'}}),
      ('언덕 출발 보조', 0x135, 'ESP_hillStartAssistActive', {'enum': {'ACTIVE': '작동', 'INACTIVE': '해제',
                                                                      'NOT_AVAILABLE': '불가', 'SNA': '없음'}}),
    ]),
    ('조향계', [
      ('조향 각속도', 0x003, 'StW_AnglSpd', {'unit': '°/s', 'dec': 1}),
      ('토션바 토크', 0x370, 'EPAS_torsionBarTorque', {'unit': 'Nm', 'dec': 2}),
      ('핸즈온 레벨', 0x370, 'EPAS_handsOnLevel', {}),
      ('EAC 상태', 0x370, 'EPAS_eacStatus', {}),
    ]),
    ('운전 보조', [
      ('사각 후좌', 0x399, 'DAS_blindSpotRearLeft', {}),
      ('사각 후우', 0x399, 'DAS_blindSpotRearRight', {}),
      ('전방 충돌 경고', 0x399, 'DAS_forwardCollisionWarning', {}),
      ('차선 이탈 경고', 0x399, 'DAS_laneDepartureWarning', {}),
      ('오토파일럿 상태', 0x399, 'autopilotStatus', {}),
      ('오토파크 준비', 0x399, 'DAS_autoparkReady', {'enum': {'AUTOPARK_READY': '준비됨',
                                                              'AUTOPARK_UNAVAILABLE': '불가'}}),
      ('오토파크 완료', 0x399, 'DAS_autoParked', {'enum': {0: '아니오', 1: '예'}}),
    ]),
    ('위치', [
      ('위도', 0x3D8, 'MCU_latitude', {'unit': '°', 'dec': 6}),
      ('경도', 0x3D8, 'MCU_longitude', {'unit': '°', 'dec': 6}),
      ('GPS 정확도', 0x3D8, 'MCU_gpsAccuracy', {'unit': 'm', 'dec': 1}),
    ]),
    ('차량 사양', [
      ('4륜 구동', 0x398, 'GTW_fourWheelDrive', {}),
      ('에어 서스펜션', 0x398, 'GTW_airSuspensionInstalled', {}),
      ('오토파일럿', 0x398, 'GTW_autopilot', {}),
      ('전방 레이더', 0x398, 'GTW_forwardRadarHw', {}),
      ('우핸들', 0x398, 'GTW_rhd', {'enum': ONOFF}),
      ('국가', 0x398, 'GTW_country', {'ascii': True}),
      ('휠 타입', 0x398, 'GTW_wheelType', {}),
    ]),
    ('차량 시계', [
      ('연', 0x318, 'YEAR', {'dec': 0}),
      ('월', 0x318, 'MONTH', {'dec': 0}),
      ('일', 0x318, 'DAY', {'dec': 0}),
      ('시', 0x318, 'Hour', {'dec': 0}),
      ('분', 0x318, 'MINUTE', {'dec': 0}),
      ('초', 0x318, 'SECOND', {'dec': 0}),
    ]),
  ]),
]


def _index(dec: CanDecoder) -> dict[int, dict]:
  """address -> {signal: entry}, preferring the lowest bus.

  The chassis traffic is mirrored onto bus 2 by the panda's forwarding, so the same address
  arrives twice; bus 0 is the car itself and is what should be displayed.
  """
  out: dict[int, tuple[int, dict]] = {}
  snap = dec.snapshot(include_unseen=False)
  for msg in snap['messages']:
    prev = out.get(msg['address'])
    if prev is None or msg['bus'] < prev[0]:
      out[msg['address']] = (msg['bus'], {s['name']: s for s in msg['signals']})
  return {addr: sigs for addr, (_, sigs) in out.items()}


def _render(sig: dict | None, opts: dict) -> dict:
  if sig is None:
    return {'value': None, 'warn': False}

  if opts.get('ascii'):
    # GTW_country is two ASCII bytes, not a number: 21843 is "US"
    raw = int(sig['raw'])
    try:
      text = raw.to_bytes(2, 'big').decode('ascii')
    except (OverflowError, UnicodeDecodeError):
      text = str(raw)
    return {'value': text, 'warn': False}

  enum_map = opts.get('enum')
  if enum_map is not None:
    # DBC label first, raw int for the signals the DBC never gave a VAL_ table
    key = sig['enum'] if sig['enum'] is not None else int(sig['raw'])
    text = enum_map.get(key, sig['enum'] if sig['enum'] is not None else str(sig['raw']))
  elif sig['enum'] is not None:
    text = sig['enum']            # SNA / INIT and friends beat a nonsense number
  elif 'unit' in opts or 'dec' in opts:
    dec_places = opts.get('dec', 1)
    text = f"{sig['v']:.{dec_places}f}"
    if opts.get('unit'):
      text += f" {opts['unit']}"
  else:
    text = str(sig['raw'])

  return {'value': text, 'warn': text in opts.get('warn', ())}


def build(dec: CanDecoder | None) -> dict:
  """The whole page's data, as {sections: [...], missing: n}."""
  if dec is None:
    return {'sections': [], 'missing': 0, 'error': '차량 미연결 · 저장된 route를 재생해서 볼 수 있습니다'}

  by_addr = _index(dec)
  sections, missing = [], 0

  for sec_id, sec_title, groups in SECTIONS:
    out_groups = []
    for group_title, rows in groups:
      out_rows = []
      for label, addr, signal, opts in rows:
        sig = (by_addr.get(addr) or {}).get(signal)
        if sig is None:
          missing += 1
        out_rows.append({'label': label, 'addr': f'0x{addr:03X}', 'signal': signal,
                         **_render(sig, opts)})
      out_groups.append({'title': group_title, 'rows': out_rows,
                         'warn': any(r['warn'] for r in out_rows),
                         'seen': any(r['value'] is not None for r in out_rows)})
    sections.append({'id': sec_id, 'title': sec_title, 'groups': out_groups})

  total = sum(len(rows) for _, _, groups in SECTIONS for _, rows in groups)
  return {'sections': sections, 'missing': missing, 'total': total}
