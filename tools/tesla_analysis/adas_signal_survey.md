# ADAS/지도 신호 실주행 서베이 (2026-08-06)

`/shadow` 페이지에 새로 추가한 raw-CAN 디코드(0x3E8/0x3C8/0x238/0x2C8/0x2E8/0x2D8/0x2B8 --
`selfdrive/debug/shadow_replay.py`의 `_AdasDecoder`)로 실주행 로그를 훑어서, 신호별로 실제 값이
변하는지/상수인지/한 번도 안 오는지를 확인한 결과. "이름만 보고 유망해 보이는 것"과 "실제로 이
차·이 로그에서 쓸모 있는 것"이 꽤 달랐다.

## 방법

`/api/shadow` (solve=false, MPC 재계산 없이 로그만 읽음)를 12개 route에서 각 2세그먼트씩 호출,
총 22세그먼트 · 24,323프레임 (두 차량 모두 포함 -- [[project_tesla_two_cars]] 참고). 각
(주소, 신호)별로 관측값 집합·min·max를 집계. 표본 routes: `00000030,0000002c,00000028,00000024,
00000020,0000001b,00000017,00000011,0000000d,00000009,00000005,0000001d`.

## 1군 -- 실제로 변하고, 플래너에 바로 쓸 수 있음

| 신호 | 관측 범위 | 비고 |
|---|---|---|
| `UI_roadCurvC0..C3` (0x2C8) | C0 -20.4..20.4 등 전부 변함 | 도로 곡률 다항식. `curve_speed_controller`에 비전 모델 예측 보완/대체로 바로 투입 가능 |
| `UI_roadCurvRange`/`Health` (0x2C8) | Range 0..252m, Health {0,2,3} | 곡률의 유효 거리·품질 플래그 |
| `UI_mapSpeedLimit` (0x3C8) | 1..31 | 실제로 변함. `mapSpeedLimitType`(=1 고정)/`mapSpeedUnits`(=0 고정)/`countryCode`(=840 고정)이라 이 표본에선 단위 문제 없음 |
| `UI_controlledAccess` (0x3C8) | 0/1 전환 | 고속도로 vs 일반도로 -- gap profile/ACC-from-zero 기본값 분기 근거 |
| fleet 속도 (0x238 mux3/4: `bottomQrtl`/`topQrtlFleetSpeedMPS`, `meanFleetSplineSpeedMPS`, `medianFleetSpeedMPS`) | 대략 3~35 m/s대 | "이 도로에서 실제 Tesla 운전자들이 내는 속도" -- 제한속도보다 이 포크의 로그-실증 튜닝 철학에 더 맞을 수 있음. `rampType`도 mux4에서 같이 변함 |
| `UI_splineLocConfidence`/`UI_gpsRoadMatch` (0x238/0x3C8) | 0..99 / 0·1 | 지도 매칭 신뢰도 -- 위 지도 신호들을 쓸 때 반드시 같이 게이팅해야 함 |
| `UI_nextBranchDist` + `Left/RightOffRamp` (0x3C8) | 넓게 변함 | 분기 거리 -- 램프 감속 준비용 |

## 2군 -- 변하긴 하는데 아직 확신 없음

| 신호 | 상태 | 메모 |
|---|---|---|
| `UI_csaRoadCurvC2/C3`, `UI_csaOfframpCurvC2/C3` (0x2E8/0x2D8) | 변함, 단 `UsingTspline`은 0x2D8에서 표본 내내 0 | `UI_roadCurvature`와 상당 부분 중복. 실시간 소비보다는 "순정 Autopilot이 이 커브에서 어떻게 감속했을지" 검증용 ground truth로 더 가치 있어 보임 |
| `UI_radarTargetDx`/`DxEnd`/`TrustMap` (0x2B8) | -75..160 / 0..255 / 0·1 | 레이더-비전 fallback 거리 점프 문제(기존 진단, [[project_tesla_gap_ui_status]])에 세 번째 신뢰도 소스로 쓸 여지 |
| **`UI_radarEnableBraking` (0x2B8)** | **24,323프레임 전부 상수 0** | 가장 기대했던 신호인데 이번 12-route 표본에서 한 번도 안 켜짐. 지도가 정지 장애물 구간을 아는 특수 상황이 이 드라이브들엔 없었던 듯. 실제로 언제 켜지는지 더 좁혀 확인 전엔 안전 로직에 절대 쓰지 말 것 |
| `UI_autosteerRestricted`/`pmmEnabled`/`scaEnabled` (0x3C8) | 0/1 변함 | 변하는 건 확인했지만 정확히 도로유형/날씨/지도 커버리지 중 뭘 반영하는지 미검증 |

## 3군 -- 이번 표본에서 사실상 무의미

- **0x3E8 `UI_driverAssistControl` 43개 신호 중 42개**: 12개 route · 24,202프레임 내내 전부
  상수. 순간순간 바뀌는 주행 신호가 아니라 이 차의 AP 기능/권한을 한 번 방송하는 static config
  메시지였다. 플래너엔 쓸모없음. (유일하게 변한 `UI_summonReverseDist`는 summon 전용이라 일반
  주행과 무관.)
- 0x3C8의 `reject*` 계열(LeftLane/RightLane/HPP/Nav/FreeSpace/Autosteer/HandsOn) -- 표본 내내 0
  고정. 완전히 죽은 신호라 단정할 수는 없음(해당 상황 자체가 없었을 수도) -- 지금까진 정보 없음.

## 다음에 볼 것

곡률(1군) -> 지도 speed limit + fleet 속도 + controlled-access(2군 중 gap/커브진입 튜닝에 바로
쓸 것들) 순으로 구현 우선순위. `UI_radarEnableBraking`이 켜지는 조건은 지도에 정지 장애물이
있다고 알려진 구간을 지나는 드라이브를 따로 찾아서 확인 필요.
