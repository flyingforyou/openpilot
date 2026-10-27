# 미해독 ID 조사 — 2026-10-27 캡처 (주차·충전 중)

`capture-20261027-155057.jsonl`(120초, ID 216개, 디코더가 아는 것 9개)의 나머지를 조사한 결과.
재현: `python tools/tesla_analysis/btobd_id_survey.py <캡처> <talas9 ModelS 폴더>`
(talas9 폴더는 github.com/talas9/tesla_can_signals 의 `ModelS/*.compact.json.json`).

주의: 이 캡처는 필터 없이 찍혀 개수가 표본이다(README "캡처" 정정 참고). 아래 "n"은 120초에 건진 개수.
100 ms 메시지는 대략 70~160개, 1 s 메시지는 5~18개로 나온다 — 이 비율이 talas9의 주기와 맞는지를
ID 매칭의 교차검증으로 썼다.

## 이 포트는 PT(파워트레인) 버스

| talas9 버스 | 캡처에 있는 ID | 길이까지 일치 |
|---|---|---|
| **PT** | 17 | **17** |
| ETH | 28 | 22 |
| CH / BFT | 1 / 1 | 1 / 1 (우연으로 판단) |

PT가 완전히 일치하고 ETH가 상당수 겹친다 → 게이트웨이가 PT와 ETH 쪽 메시지를 진단 커넥터로 내보낸다.
talas9는 Model S·다른 펌웨어 기준이라 신호 위치는 "값이 물리적으로 그럴듯함"까지만 확인됐다.

## 유용한 것 (확인 수준 순)

### 1. 충전 — PT, 값이 서로 정확히 맞음

| ID | 메시지 | 이날 값 | 의미 |
|---|---|---|---|
| 0x212 | BMS_status | `BMS_CHARGER` | BMS 상태 (주행/충전/급속/폴트/용접검사…) |
| 0x207/0x209/0x20B | CHGPH1/2/3_lineStatus | 205~207 V ×3 | 차량 충전기 3개 모듈의 AC 입력 전압 |
| 0x21C | CHG_status (ETH) | `SINGLE_PHASE` | 단상 입력(3모듈 병렬) |
| 0x77D | PLC_comboStatus | pilot `LINE_CHARGE`, proximity `LATCHED` | 충전 커넥터 체결·충전 허가 |
| 0x51F mux | PLCRLY_sensorData | pilot +6.0 V / −11.8 V | J1772 상태 C(충전 중)와 정확히 일치 |
| 0x20E | CP_status | `CP_TYPE_US_TESLA` | 충전 포트 종류 |
| 0x25C | CHG_FCstatus | `FC_CONTACTOR_STATE_OPEN` | 급속충전 접촉기(완속이라 열림이 맞음) |
| 0x42C/43C/44C | CHGVI_PHx_chargerRequest | hvilStatus 1 | HVIL(고전압 인터록) 정상 |

교차확인: 팩 쪽 15.6 A × 368 V ≈ 5.7 kW, AC 206 V면 입력 약 28 A → 32 A급 충전기와 맞는다.
**충전이 왜 안 되나/왜 멈췄나**를 진단할 때 이 묶음이 핵심(파일럿 전압, 근접, HVIL, BMS 상태).

### 2. 배터리 열관리 — ETH, 값 그럴듯함

| ID | 메시지 | 이날 값 |
|---|---|---|
| 0x26A | THC_batteryHeaterStatus | 히터 OFF, 히터 온도 23.5 °C (모듈 22~24 °C와 일치) |
| 0x32A / 0x39A | THC_status1 / status5 | 배터리 펌프 1·2, PT 펌프 28~30 % (요청 30 %) |
| 0x37A | THC_status4 | 라디에이터 바이패스 90 %, 모드밸브 parallel |
| 0x20A | THC_compressorStatus | 컴프레서 냉방 0 %, 냉매 압력 3.9 bar |
| 0x33A | THC_status2 | 냉매 토출 24.6 °C / 흡입 21 °C, 흡입 압력 5.1 bar |

급속충전·고속주행 때 배터리 냉각이 제대로 도는지(펌프·칠러·컴프레서) 볼 수 있다. 이날은 다 쉬는 상태.

### 3. 차량 식별·시각 — 직접 해독, 확실

| ID | 내용 | 근거 |
|---|---|---|
| 0x508 / 0x538 | **VIN**, mux 0/1/2에 7+7+3글자 ASCII | `5YJXC…GF0·····` (이 문서에서는 가림) — Model X, 2016년식(G), 프리몬트(F) |
| 0x528 | **UTC 유닉스 시각**, 4바이트 빅엔디언, 약 0.2 Hz | 캡처 시각과 2초 이내 일치 |
| 0x562 / 0x5D8 | 배터리 주행거리 (같은 값) | 0x5D8은 0x562 복사본 |
| 0x542 / 0x552 / 0x558 | ASCII `T16H005`, `9685`, `cba7fc3f` | 모듈 HW/SW 식별 문자열·git 해시로 추정 |
| 0x512 | BMS_version | BMS HW id 81 (0x368 GTW_packConfig 81과 같음) |

VIN이 브로드캐스트로 들린다는 건, UDS(0x22 F190)가 안 되는 이 포트에서도 차를 식별할 수 있다는 뜻.
op-logs의 두 대의 Model X(0x398/0x359 지문으로 구분 중)를 VIN으로 직접 가를 수 있다.

### 4. 후보 (검증 필요)

| ID | 관찰 | 가설 | 확인 방법 |
|---|---|---|---|
| 0x3F8 | LE u16 4개 = 715/737/691/712, 120초간 넷 다 같이 2~3씩 감소 | /16 하면 44.7/46.1/43.2/44.5 → **타이어 공기압(psi)** (Model X 권장 ≈45 psi), 주행 후 식는 중 | 계기판 TPMS 화면과 대조 |
| 0x467 / 0x469 / 0x46B | LE u16 2424~2430 | /100 → 24.2~24.3 °C 온도 3개 | 실내/외기 온도와 대조 |
| 0x00E | STW_ANGLHP_STAT (opendbc tesla_can) | 조향각 −9.4°, 주차 고정 | 이미 차량 하네스 버스에도 있음 |
| 0x0202 | BCFDM_status (ETH) | 외부 도어 릴리스 `PRESSED`가 120초 내내 고정 | 매칭이 틀렸을 가능성 — 쓰지 말 것 |

### 매칭됐지만 우연으로 판단 (쓰지 말 것)

- 0x3D2: talas9 CH의 EPAS3P_angleCalibration과 ID가 같지만, 이 버스에서는 수명 충·방전량(wk057)으로 이미 맞게 해독됨.
- 0x7E2 / 0x738 / 0x7EA: UDS 요청·응답 이름과 ID가 같지만 내용이 UDS 형식이 아님.
- 0x318: PT RCCM_THCHvacSensors와 ETH GTW_carConfig 둘 다 1 s 주기라 구분 불가.
- 0x368: PT GTW_carConfig(100 ms)와 ETH BCFDM_warningMatrix1(1 s) — 관측 121개라 **PT 쪽이 맞음**.

## 아직 모르는 것

- 177개 ID는 어느 표에도 없음. 대부분 0x3xx·0x7xx에 값 0 고정(주차 중 꺼진 모듈)이라 정보가 없다.
- 이날은 주차·충전만 했다. 주행(구동·회생·브레이크), 급속충전, 공조 가동 상태의 캡처가 있어야 나머지가 움직인다.
