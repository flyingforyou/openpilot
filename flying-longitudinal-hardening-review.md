# flying 종방향 제어 점검 및 패치 설명

대상 브랜치: `flyingforyou/openpilot`의 `feature/tesla-gap-long-ui`

패치 파일: `flying-longitudinal-hardening.patch`

SHA256: `67eff8416cab216f8a7d286ee56e4ad896beb58e88cf5597ee7f5a96afb6a8f7`

## 결론

현재 gap 1~7 테이블, 마지막 gap 저장, gap 확대만 천천히 반영하는 비대칭 slew, dynamic tFollow의 일시적 0.50초 하한, FCW 상수는 의도된 설계이므로 변경하지 않았다.

실제 코드 흐름과 기존 주행 로그를 함께 보면 수정 가치가 명확한 부분은 다음 네 가지다.

~ Tesla 속도 PID가 현재 시점의 `speeds[0]`을 목표로 받지만 `aTarget`은 actuator delay 이후 시점에서 계산되어, 속도 오차와 feedforward가 서로 다른 시점을 가리킨다.

~ experimental mode에서 e2e 가속도가 선택되어도 공개되는 speed trajectory는 MPC 것이므로, e2e 가속도와 MPC 속도 목표를 섞을 수 있다.

~ `LeadCreepFollowCms`가 순간적인 `vLeadK` 한 값만 보고 전체 `shouldStop`을 지워, 정지차 속도 노이즈, cut-in, 빠른 접근, e2e 정지선, leadTwo 또는 강제 감속 상황에서 정지 요청을 잘못 풀 수 있다.

~ dynamic follow가 앞차를 처음 잡거나 다른 트랙으로 바뀐 순간에도 이전 `aLeadK`와 새 `aLeadK`를 차분할 수 있어, 가짜 jerk와 완화된 jerk cost가 새 앞차에 한 프레임 이상 이어질 수 있다.

테스트 코드와 웹 설명도 현재 1.10~1.60초 gap 테이블 및 실제 안전 가드와 맞지 않는 부분이 있어 함께 정리했다.

## 패치 내용

### 1. 속도 PID의 목표 시점과 source 정렬

수정 파일:

~ `selfdrive/controls/controlsd.py`

~ `selfdrive/controls/lib/longcontrol.py`

변경:

~ MPC source에서는 `longitudinalActuatorDelay + DT_MDL` 시점의 계획 속도를 보간해 velocity PID 목표로 사용한다.

~ e2e acceleration이 최종 `aTarget`으로 선택된 경우에는 현재 속도에서 선택된 가속도를 짧은 actuator horizon만큼 적분한 목표를 사용한다.

~ speed plan이 비어 있거나 길이가 맞지 않거나 NaN이면 0m/s가 아니라 현재 차량 속도로 fallback한다. 계획 데이터 누락 때문에 갑작스럽게 정지 목표가 들어가는 것을 막는다.

~ `TeslaVelocityPid`가 꺼져 있으면 기존 acceleration PID 경로는 그대로다.

### 2. creep-follow 정지 해제 조건 강화

수정 파일:

~ `selfdrive/controls/lib/longitudinal_planner.py`

변경:

~ 같은 앞차가 0.20초 이상 계속 임계 속도를 넘을 때만 활성화한다.

~ ego speed가 2.5m/s 이하인 stop-and-go 구간에서만 허용한다.

~ 상대속도 기준으로 0.5m/s보다 빠르게 접근 중이면 정지를 풀지 않는다.

~ 설정된 stop distance 외에 1m와 현재 속도에 필요한 comfort-brake 거리까지 남아 있어야 한다.

~ radar track ID가 바뀌거나, vision-only lead에서 거리 또는 속도가 크게 점프하면 확인 카운터를 다시 시작한다.

~ NaN이나 lead loss, disengage, forceDecel에서는 즉시 reset한다.

~ creep-follow가 풀 수 있는 정지는 평범한 `lead0` 기반 MPC stop으로 제한한다.

~ e2e stop-line 또는 traffic-light stop, `leadTwo`, cruise/외부 zero-speed cap, forceDecel 요청은 절대 지우지 않는다.

~ 정밀정지용 `v_soft`도 creep이 확인된 경우 0m/s가 아니라 앞차의 확인된 저속으로 수렴하게 한다.

### 3. dynamic-follow 앞차 연속성 보호

수정 파일:

~ `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`

변경:

~ 첫 앞차 인식은 jerk를 계산하지 않고 현재 `aLeadK`를 기준값으로만 저장한다.

~ radar track ID 변경, radar↔vision 전환, 거리 5m 이상 점프, 속도 5m/s 이상 점프에서는 jerk EMA를 reset한다.

~ lead loss와 비정상 수치, MPC solver reset에서도 dynamic lead 상태를 reset한다.

~ raw jerk를 ±5m/s³로 제한한다. 단일 Kalman acceleration 점프로 dynamic tFollow나 jerk cost가 바로 포화되는 것을 막는다.

~ `set_weights()`에서 새 앞차의 연속성을 먼저 확인한다. cut-in 직후 이전 앞차의 `jerk_factor × 0.5`가 새 앞차에 이어지지 않는다.

~ 기존 dynamic tFollow의 방향, EMA, 일시적 0.50~2.00초 clip, 기본 off 동작은 유지한다.

### 4. 테스트 및 설명 정합성 수정

수정 파일:

~ `selfdrive/controls/tests/test_following_distance.py`

~ `selfdrive/controls/tests/test_longcontrol.py`

~ `selfdrive/controls/tests/test_tesla_longitudinal_hardening.py`

~ `selfdrive/debug/tuning_server.py`

변경:

~ 오래된 0.80~1.75초 기대값을 현재 1.10~1.60초 테이블에 맞춘다.

~ personality fallback도 현재 aggressive 1.10, standard 1.30, relaxed 1.45에 맞춘다.

~ velocity PID actuator-horizon 보간과 e2e source target을 테스트한다.

~ 앞차 첫 인식, track 전환, vision-only 점프, NaN, raw jerk clip, cut-in 전 weight reset을 테스트한다.

~ creep persistence, 정지차 속도 노이즈, 빠른 closing, 부족한 제동 공간, track change, force/source 격리를 테스트한다.

~ 웹 튜닝 설명의 gap 1↔7 전환 시간을 현재 0.50초 범위에 맞게 5.0초, 약 1.4초, 1.0초로 수정한다.

~ “위험 없음” 같은 절대 표현을 제거하고 실제 가드 및 저속 실차 확인 방법을 설명한다.

## 변경하지 않은 항목과 이유

~ gap 1~7의 `1.10, 1.20, 1.30, 1.38, 1.45, 1.52, 1.60초`: 이전 시내 FCW 분석 뒤 의도적으로 올린 값이다.

~ gap 확대만 rate-limit하고 축소는 즉시 반영하는 동작: 목표거리를 확대할 때 급감속을 줄이기 위한 명시적인 설계다.

~ `TeslaLastGapAdjust`: 차량이 gap 상태를 즉시 재전송하지 않는 경우를 위한 fallback이다.

~ dynamic tFollow의 0.50초 일시적 floor: 상시 base gap이 아니라 jerk transient 보정이며 옵션 기본값은 off다.

~ FCW의 `CRASH_DISTANCE`와 `LEAD_DANGER_FACTOR`: gap table 수정 이후에는 경고 민감도보다 차간 설정을 유지하는 편이 맞다.

~ ACADOS solver와 generated code: 이번 수정은 solver parameter 형식을 바꾸지 않으므로 재생성하지 않는다.

~ DBC, cereal schema, Tesla CAN parser: 이번 문제를 고치는 데 새로운 신호가 필요하지 않다.

## 적용 방법

브랜치가 패치 작성 시점의 `feature/tesla-gap-long-ui`와 같다면:

```bash
cd /data/openpilot
git status --short
git am /path/to/flying-longitudinal-hardening.patch
```

충돌이 발생하면:

```bash
git am --abort
```

이 패치는 `git am` 형식이며 한 개 커밋으로 적용된다.

## 검증 결과

완료한 검증:

~ 수정 Python 파일 AST 구문 검사

~ 변경된 핵심 함수를 실제 소스에서 추출해 실행한 단위 검증

~ velocity target horizon, e2e target, fallback 검증

~ creep persistence, distance/closing/track/source guard 검증

~ dynamic lead acquisition, cut-in, NaN, jerk clip 검증

~ `git diff --check`

~ 새 baseline worktree에 `git am` 재적용 후 patch integrity 검사

현재 환경에서 완료하지 못한 검증:

~ 전체 openpilot 빌드

~ ACADOS와 cereal을 포함한 전체 pytest suite

~ 실제 rlog replay

~ Model X HW1 실차 주행

따라서 첫 실차 검증은 반드시 낮은 속도와 충분한 제동 여유에서 진행해야 한다.

## 권장 실차 테스트 순서

### 1. 회귀 기준

~ `DynamicTFollowGain=0`

~ `LeadCreepFollowCms=0`

~ `TeslaVelocityPid=0`

기존과 동일한지 확인한다. 이번 패치는 세 옵션이 모두 꺼져 있을 때 주행 동작을 바꾸지 않아야 한다.

### 2. 정밀 정지

~ `TeslaVelocityPid=1`

~ 나머지 두 옵션 off

정지차에 20~30km/h 이하로 접근해 10m, 6m, 최종 정지 시점의 vEgo와 dRel을 기록한다. 계획 속도 누락이나 experimental mode 전환 때 갑작스러운 브레이크가 없는지 확인한다.

### 3. creep-follow 단독

~ `LeadCreepFollowCms=0.50`부터 시작

~ Dynamic off

앞차가 0.3~1.0m/s로 계속 움직이는 정체에서 0.2초 후 저속 추종이 이어지는지 확인한다. 앞차가 멈추거나 cut-in, 빠른 closing, 정지거리 근처에서는 즉시 정상 정지로 돌아와야 한다.

### 4. dynamic-follow 단독

~ `DynamicTFollowGain=0.30`

~ Creep off

앞차 가속과 감속 전환에서만 반응하는지 본다. 새 앞차가 끼어든 첫 순간에는 간격이나 jerk cost가 이전 앞차 상태를 이어받지 않아야 한다.

### 5. 병행

개별 검증이 끝난 뒤에만:

~ `TeslaVelocityPid=1`

~ `LeadCreepFollowCms=0.50`

~ `DynamicTFollowGain=0.30`

정체, cut-in, 앞차 완전정지, 재출발, experimental mode 정지선을 순서대로 확인한다.
