#!/usr/bin/env python3
"""Live viewer and tuning switchboard, served from the device over WiFi.

Real-car A/B testing otherwise means a laptop in the passenger seat. This serves a page you can
open on a phone: current lead perception state on the left, the switches that change it on the
right, so a run can be set up and its effect watched without stopping to SSH in.

  PYTHONPATH=/data/openpilot python3 selfdrive/debug/tuning_server.py
  # then open http://<device-ip>:8088 from anything on the same network

Pages: / index, /live lead perception and settings, /can every decoded CAN signal,
/videos the recorded road video.

Settings are saved whenever you press 반영, but radard and the longitudinal planner only
re-read them while disengaged. So a change made mid-drive lands at the next engage rather than
moving the target distance under a car that is already following one.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

# The shadow replay's background thread spends most of a solve inside C calls (zstandard,
# capnp, acados) that hold the GIL for the default 5ms slice before the interpreter checks
# whether another thread wants it -- long enough that a poll from the page waiting on
# /api/shadow could sit for multiple seconds behind it. This makes that check happen more
# often, so the HTTP threads stay responsive while a solve is running, at a small unmeasurable
# cost to solve throughput.
sys.setswitchinterval(0.001)
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.debug import vehicle_state, video_source
from openpilot.selfdrive.debug.can_source import CanSource, list_routes
from openpilot.selfdrive.debug import shadow_replay
from openpilot.selfdrive.debug.intervention_log import (InterventionLog, list_events,
                                                        locate_segment, read_event)
from openpilot.selfdrive.modeld.model_manager import ModelManager

# Options rather than free-form numbers: a typo in a text box goes straight into the braking
# path, and named choices are also what makes an A/B run reproducible afterwards.
# CarrotPilot's own longitudinal knobs. Grouped separately because none of them do anything
# unless CarrotLongEnabled is on -- they are read by its planner, not this tree's. Defaults are
# carrot's; the labels explain what each does rather than restating the parameter name.
# The map auto-cruise family, kept in its own table so the page can give it its own
# section next to the live map readout instead of burying five related toggles in the
# middle of the general list.
MAP_SETTINGS = {
  "TeslaMapAutoSpeed": {
    "label": "지도 자동 크루즈", "type": "bool",
    "help": "매 프레임 스토크 값(차가 보고하는 vCruise)에서 시작해, 아래 상황 중 해당하는 것이 "
            "그 값을 대체합니다. 대체된 값이 longitudinalPlan.cruiseTarget으로 나가고, 계기판 "
            "표시(DAS_setSpeed)와 MPC 목표 둘 다 이 값을 씁니다."
            "<br><br><b>① 표지판을 믿을 때</b> — baseSpeedLimit(가장 먼저 갱신)을 우선하고, "
            "없으면 mapSpeedLimit·mppSpeedLimit·fusedSpeedLimit 순으로 봅니다. 위치 신뢰도가 "
            "낮거나(splineConfidence&lt;60) GPS가 도로에 안 붙었으면 아예 안 믿습니다. 표지판이 "
            "도로등급과 안 맞으면(예: 진출로인데 65mph 그대로) 방금 나온 도로의 값으로 보고 버립니다."
            "<br><br><b>② 램프(진입·진출)</b> — 램프엔 표지판이 없고 도로등급도 두 도로 사이라 "
            "애매하므로, 그 지점을 실제로 달리는 플릿 속도(fleetSplineSpeed, 위치 기준이라 "
            "구간이 아니라 연속으로 움직임)를 그대로 목표로 씁니다. 진입로는 지금 속도보다 "
            "낮게는 절대 안 잡습니다(합류 중 감속 요청 방지)."
            "<br><br><b>②-1 커브 감속</b> — 아래 '커브 감속' 옵션을 켜면 <b>모델이 보는 곡률</b>로 "
            "코너 진입 속도를 <b>상한으로만</b> 제한합니다(램프 포함 전 구간). 플릿 속도는 "
            "여기 쓰지 않습니다 — 램프(②)에서만 씁니다. 예전에는 플릿을 모든 도로의 상한으로도 "
            "썼는데, 실측 결과 곡률과 무관한 신호였습니다: 비램프 18.3만 프레임에서 "
            "횡가속도와 (플릿−표지판)의 상관이 <b>+0.043</b>, 가장 조이는 구간에서도 플릿이 "
            "표지판보다 +4.5mph 높게 읽혔습니다. 실제로 잡히는 건 교통·신호이고 그건 이미 "
            "레이더 추종이 처리하므로, 직선 구간 목표만 93~97% 깎고 있었습니다."
            "<br><br><b>③ 표지판도 램프도 아닐 때</b> — 지도가 아직 안 잡혔거나 미매핑 도로. "
            "마지막 목표를 유지하되 도로등급별 상한(고속 75mph·간선 50·집산 35·국지 30)으로만 "
            "묶어서, 고속도로 속도가 국지도로까지 안 넘어오게 합니다."
            "<br><br><b>④ 운전자가 레버를 움직일 때</b> — \"여긴 아니고 이 속도\"로 그대로 "
            "받아들여 목표를 대체합니다(내리는 것뿐 아니라 올리는 것도). 지도 목표가 그 뒤로 "
            "약 5mph 이상 다른 구간으로 실제로 옮겨가기 전까진 이 값을 유지합니다 — 다음 신호에 "
            "풀리는 게 아니라 다음 존에서 풀립니다."
            "<br><br><b>상한</b> — 이 차는 스토크 값이 곧 크루즈 설정속도라서(pcmCruise) 스토크를 "
            "상한으로 못 씁니다. 지난 도로에서 30에 맞춰뒀으면 고속도로에 들어서도 30에 묶이기 "
            "때문입니다. 그래서 상한은 TeslaMapAutoSpeedMax 하나뿐입니다."
            "<br><br><b>변화 속도</b> — 내려갈 때 1.0m/s², 올라갈 때 0.5(진입 합류는 1.5, 대기 "
            "없이 바로). 올라가는 쪽은 3초 대기 후 시작하되(지도가 먼저 다음 구간을 알려도 차는 "
            "아직 진입로 위) 진출로에선 대기 없이 바로 내려갑니다. 커브 감속기가 이 결과를 뒤에서 "
            "한 번 더 낮출 수 있습니다(올리진 않음)."
            "",
    "options": [(0, "사용 안 함 (기본)"), (1, "사용")],
  },
  "TeslaMapAutoSpeedMax": {
    "label": "지도 자동 크루즈 상한", "type": "int",
    "help": "자동으로 올라갈 수 있는 최대 속도입니다. 이 차는 스토크 값이 곧 크루즈 설정속도라서 "
            "스토크를 상한으로 쓰면 시내에서 맞춰둔 값에 묶여 고속도로에서 못 올라갑니다. "
            "그래서 상한은 여기서 한 번만 정합니다.",
    "options": [(105, "65mph"), (113, "70mph"), (121, "75mph"), (129, "80mph (기본)")],
  },
  "TeslaMapAutoSpeedRatio": {
    "label": "지도 제한속도 비율", "type": "int",
    "help": "게시된 제한속도의 몇 %를 목표로 삼을지입니다. 속도대별 오프셋(40mph 미만 +5, "
            "이상 +10)은 이 비율을 적용한 뒤에 더해집니다.",
    "options": [(90, "90% 여유"), (100, "100% 표지판대로 (기본)"), (105, "105%"), (110, "110%")],
  },
  "TeslaMapAutoSpeedCurve": {
    "label": "커브 감속 (곡률 기준)", "type": "bool",
    "help": "<b>모델이 보는 곡률</b>로 코너 진입 속도를 <b>상한으로만</b> 제한합니다. 램프를 "
            "포함한 전 구간에 적용되고, 구간 평균이 아니라 한 지점에서 읽으므로 도로가 펴지는 "
            "즉시 풀립니다."
            "<br><br><b>플릿 속도는 여기 쓰지 않습니다.</b> 예전에는 플릿 속도"
            "(UI_meanFleetSplineSpeedMPS)를 모든 도로의 상한으로도 썼지만, 실측 결과 곡률과 "
            "무관한 신호였습니다 — 비램프 18.3만 프레임에서 횡가속도와 (플릿−표지판)의 상관이 "
            "<b>+0.043</b>, 부호도 반대였고, 가장 조이는 구간(3.0m/s² 이상)에서도 플릿이 표지판보다 "
            "<b>+4.5mph 높게</b> 읽혔습니다. 플릿이 표지판 아래로 내려가는 비율은 도로가 조여질수록 "
            "오히려 줄었습니다(직선 22% → 급커브 0%). 실제로 잡히는 건 교통·신호이고 그건 이미 "
            "레이더 추종이 처리하므로, 직선 구간에서 93~97% 발동해 목표만 깎고 있었습니다"
            "(고속도로 3.3mph, 정체 간선 11.7mph — 후자는 표지판 아래까지). 플릿은 표지판이 없는 "
            "<b>램프에서만</b> 목표로 씁니다."
            "<br><br><b>횡가속도 기준이란</b> — 반경 R인 커브를 속도 v로 돌면 바깥으로 v²/R 만큼의 "
            "가속도가 걸립니다. 이 값을 얼마까지 허용할지 정하면 그 커브의 한계속도가 "
            "<code>√(기준 × R)</code>로 나옵니다. 기준이 낮을수록 더 일찍·더 많이 감속합니다. "
            "참고로 이 차의 실측 횡가속도는 중앙값 0.40, p99 3.43, p99.9 4.54m/s² 였고, "
            "조향 자체의 한계(MAX_LATERAL_ACCEL_NO_ROLL)는 5.0입니다. 3.0은 일상 주행의 "
            "상위 1% 언저리 — 평소엔 안 걸리고 급커브에서만 걸리는 값입니다."
            "<br><br><b>올리지는 않습니다</b> — 목표를 대체하지 않고 낮추기만 합니다. 감속 자체는 "
            "기존 슬루(내려갈 때 1.0m/s²)를 그대로 따라갑니다.",
    "options": [(0, "사용 안 함"), (1, "사용 (기본)")],
  },
  "TeslaMapCurveLatAccel": {
    "label": "커브 감속 · 쏠림 기준", "type": "int",
    "help": "커브를 돌 때 <b>몸이 옆으로 쏠리는 세기</b>를 어디까지 허용할지입니다. 위 '커브 감속'을 "
            "켰을 때 그 상한을 정하는 값입니다."
            "<br><br>반경 R인 커브를 속도 v로 돌면 <code>v² ÷ R</code> 만큼 쏠립니다. 속도가 "
            "<b>제곱</b>으로 들어가서, 2배 빠르면 쏠림은 4배입니다. 이 값을 정하면 커브마다 "
            "한계속도가 <code>√(기준 × R)</code>로 나옵니다."
            "<br><br><b>왜 플릿을 안 쓰는가</b> — 플릿 스플라인은 구간 평균이라 제일 조인 지점이 "
            "뭉개집니다. 반경 31m 헤어핀 실측에서 플릿은 28.5mph까지만 내려갔는데, 3.0 기준 "
            "곡률한계는 21.4mph였고 실제로는 <b>17.3mph</b>로 돌았습니다. 그리고 애초에 곡률과 "
            "상관이 없습니다(비램프 18.3만 프레임, 상관 +0.043). 그래서 커브는 이 기준 하나가 "
            "전담하고, 직선에서는 곡률한계가 100mph쯤이라 저절로 아무 구속이 없습니다."
            "<br><br><b>기준값 고르기</b> — 이 차 실측 쏠림은 중앙값 0.40, 상위10% 2.00, "
            "상위1% 3.43, 상위0.1% 4.54 m/s² 이고 조향 자체 한계는 5.0입니다. "
            "<b>3.0은 상위 1% 언저리</b>라 평소엔 안 걸리고 급커브에서만 걸립니다. 실제로 "
            "상한이 주행을 막는 시간이 3.0에서 <b>3.4%</b>, 2.5로 낮추면 <b>7.6%</b>로 두 배가 "
            "됩니다 — 낮출수록 평범한 커브에서도 개입해 답답해집니다."
            "<br><br>전방 <b>4초</b>를 봅니다. <b>램프에도 적용됩니다</b> — 진입 램프의 루프 "
            "구간(전체의 14.2%)에서 플릿은 38.6mph로 읽는데 실제로는 27.8mph로 돌아야 했고, "
            "이 기준은 27.3mph로 맞췄습니다. 램프의 목표 자체는 플릿이 잡으므로, 이 기준이 그 "
            "루프를 잡아주는 유일한 장치입니다. 합류 직선에서는 곡률이 없어 한계가 113mph쯤이라 "
            "저절로 풀립니다.",
    "options": [(0, "사용 안 함 (커브 감속 없음)"), (250, "2.5 일찍 감속"),
                (300, "3.0 (기본)"), (350, "3.5 늦게 감속")],
  },
  "TeslaMapCurveUseMap": {
    "label": "커브 감속 · 먼 곡률은 지도에서", "type": "bool",
    "help": "위 '커브 감속'이 읽는 곡률을 <b>60m 안쪽은 모델, 바깥은 차량 지도</b>로 나눠서 "
            "받습니다. 둘 중 <b>더 조이는 쪽</b>을 쓰므로 속도를 낮추기만 하고 올리지는 "
            "않습니다. 게이트웨이가 쏘는 <code>UI_roadCurvature</code>(0x2C8)의 3차 곡선이 "
            "지도 쪽 출처입니다."
            "<br><br><b>왜 나누는가</b> — 실측입니다. 두 출처가 d미터 앞을 어떻게 예측했는지를, "
            "차가 실제로 그 d미터를 달렸을 때 <b>정말 돌았던 곡률</b>(조향각 기반 "
            "<code>controlsState.curvature</code>, 어느 쪽 출력도 아님)과 맞춰봤습니다."
            "<br><br>60m 안쪽에서는 <b>모델이 더 정확합니다</b> — 카메라가 그 도로를 실제로 보고 "
            "있으니 당연합니다. 그런데 100m에서는 모델이 사실상 눈을 감습니다: 실제 커브 835개 중 "
            "<b>15%만</b> 잡았고, 지도는 <b>72%</b>를 잡았습니다. 잡아낸 커브의 오차도 모델이 78% "
            "더 컸습니다(0.00148 vs 0.00083 1/m). 예측 배열 끝단이라 먼 커브가 직선으로 펴집니다."
            "<br><br><b>왜 이게 중요한가</b> — 커브 감속은 전방 <b>4초</b>를 봅니다. 시속 60마일에서 "
            "4초는 <b>107m</b>입니다. 즉 고속 커브에서 이 기능이 물어보는 바로 그 거리가 모델이 못 "
            "보는 거리였습니다. 시내(30mph)에서는 4초가 54m라 60m 안쪽이고, 그때는 지도가 아무것도 "
            "보태지 않습니다 — 모델로 충분하기 때문입니다."
            "<br><br><b>느려지지 않습니다</b> — 지도가 직선을 커브라고 우기는 비율은 100m에서 "
            "<b>1.1%</b>였고, 직선에서 이 값으로 계산한 한계속도는 하위 1%에서도 58mph였습니다. "
            "45mph 아래로 내려간 직선은 10,886개 중 <b>8개(0.07%)</b>입니다."
            "<br><br><b>솔직히, 지금은 거의 발동 안 합니다.</b> 기록된 주행 3개(00000073, "
            "0000006e, 0000007d)에 그대로 재생해보면 기본값 3.0에서는 실제 주행속도보다 낮게 "
            "잡히는 프레임이 <b>거의 0%</b>였습니다. 2.5로 낮추면 0.2~0.3% 구간에서 걸리고 "
            "그때 깎이는 폭이 중앙값 3.5~5.4mph(75mph 부근)입니다. 그 세 주행에 이게 필요할 만큼 "
            "조인 고속 커브가 없었다는 뜻이지, 안 걸린다는 뜻은 아닙니다. 안 쓰던 안전장치가 "
            "하나 생긴 것으로 보시면 됩니다."
            "<br><br>메시지가 <b>0.5초</b>만 끊겨도 즉시 무시합니다. CAN 파서는 송신이 멈춰도 "
            "마지막 값을 계속 들고 있어서, 굳어버린 '급커브' 값이 차를 영영 붙잡을 수 있습니다. "
            "게이트웨이가 스스로 붙이는 health 플래그가 0이거나 유효거리가 60m에 못 미쳐도 "
            "쓰지 않습니다.",
    "options": [(0, "모델만 사용"), (1, "60m 밖은 지도 (기본)")],
  },
}


# Sealed, not deleted. These params are still declared in params_keys.h, still read by
# carrot_functions.py, and whatever is stored on the device still drives the car. Dropping them
# from this table only takes them off the page and out of `known`, so the write endpoint rejects
# them too -- the stored values become read-only rather than gone:
#
#   TFollowGap1..7   the gap table. Its numbers are not headway seconds on their own: they are the
#                    *slope* of `k*v**2 + t_follow*v + stop_distance`, so they only mean something
#                    paired with ComfortBrake (curvature) and StopDistanceCarrot (intercept).
#                    Tuning one of the three in isolation is what makes this page dangerous, and
#                    the stalk already picks a position at runtime -- there is nothing to adjust.
#   CruiseMaxVals0..6  the accel curve. Never moved off its defaults in any recorded drive.
#   ComfortBrake2    two parameters for the single scalar k. Fixed at its 2.50 default, which is
#                    also DEFAULT_COMFORT_BRAKE_2 and the MPC's own COMFORT_BRAKE.
#   MyDrivingMode / MyDrivingModeAuto
#                    a hidden multiplier layer: mySafeFactor scales t_follow *and* comfort_brake
#                    (carrot_functions.py:393 and :708), so Eco/Safe quietly bend the same gap
#                    curve this page tunes explicitly. Sat on Normal for every frame of every
#                    recorded drive.
CARROT_SETTINGS = {
  "DynamicTFollow": {
    "label": "앞차 거동 따라 간격 조정", "type": "int",
    "help": "앞차가 감속하면 간격을 벌리고 멀어지면 좁힙니다. 0 이면 사용 안 함.",
    "options": [(0, "사용 안 함 (기본)"), (50, "약하게 0.5"), (100, "표준 1.0"), (150, "강하게 1.5")],
  },
  "DynamicTFollowLC": {
    "label": "차선변경 시 간격", "type": "int",
    "help": "차선을 바꾸는 동안 추종 시간에 곱합니다. 캐롯 원본은 좁히는 용도로만 설계됐고 "
            "(범위 20~100%), 넓히는 옵션은 없습니다.",
    "options": [(60, "많이 좁게 0.6"), (80, "좁게 0.8"), (100, "그대로 1.0 (기본)")],
  },
  "EnableSpeedTF": {
    "label": "속도별 간격 조정", "type": "int",
    "help": "음수(-1~-3)면 Gap 1~4 를 그 구간표로 다시 배분합니다 (예: -1 은 0/30/60/90km/h "
            "구간). 양수(1~50)면 정지 상태에서 그 % 만큼 간격을 줄이고 100km/h 에서 원래 "
            "값으로 돌아옵니다 -- 캐롯 원본 범위가 -3~50 이라 1 은 사실상 티가 안 나고, "
            "체감하려면 최소 10 이상을 넣어야 합니다. 0 이면 사용 안 함."
            "<br><br><b>고속에는 영향이 없습니다</b> — 100km/h(62mph) 이상에서 배율이 정확히 1.0 "
            "이라 저속 구간만 좁힙니다."
            "<br><br><b>⚠ 금방 포화합니다.</b> 줄어든 t_follow 는 <code>_clip_t_follow</code> 에서 "
            "갭 표의 최솟값으로 하한이 걸립니다. 25 만 넣어도 저속 구간이 대부분 하한에 닿아, "
            "50 으로 올려도 30mph 목표거리는 0.4m 남짓밖에 안 줄어듭니다. 하한을 더 내리려면 "
            "갭 표 자체를 낮춰야 하는데 <b>그 표는 봉인되어 있으므로</b>, 이 설정으로 얻을 수 있는 "
            "폭은 여기까지가 전부입니다.",
    "options": [(-3, "0/50/100/150 단계"), (-2, "0/40/80/120 단계"), (-1, "0/30/60/90 단계"),
                (0, "사용 안 함 (기본)"), (10, "저속 10% 감소"), (25, "저속 25% 감소"),
                (50, "저속 최대 50% 감소")],
  },
  "TFollowDecelBoost": {
    "label": "감속 시 간격 확대", "type": "int",
    "help": "내가 감속 중일 때 목표 간격을 추가로 벌리는 정도입니다.",
    "options": [(0, "없음"), (50, "표준 0.5 (기본)"), (100, "크게 1.0")],
  },
  "ComfortBrake": {
    "label": "편안한 제동 가정", "type": "int",
    "help": "플래너가 <b>내가 편안하게 낼 수 있다고 가정하는 감속도</b>입니다. 목표거리 곡선의 "
            "<b>곡률</b>을 정합니다:"
            "<br><code>목표 = k×속도² + 추종시간×속도 + 정지간격</code>, "
            "<code>k = 1/(2×이값) − 1/5</code>"
            "<br><br>뒤의 1/5 은 앞차분 계수(ComfortBrake2)로, <b>2.50 에 봉인되어 있습니다</b> — "
            "MPC 의 COMFORT_BRAKE 와 같은 값입니다. 이 값은 항상 2.50 이하로 자동 제한되므로 "
            "k ≥ 0 이 보장됩니다. 넘어가면 목표거리가 속도의 제곱으로 <i>줄어들어</i> "
            "2026-08-14 에 74mph 에서 앞차와 12m 까지 붙은 것이 정확히 그 경우입니다."
            "<br><br>실측(연속 추적된 리드만, 정속 추종 중앙값): "
            "2.4 → <b>1.72초</b>, 2.8 → 1.12초, 3.2 → <b>0.87초</b>."
            "<br><br><b>⚠ 이제 이 페이지에서 유일하게 남은 거리 곡선 손잡이입니다.</b> 기울기(갭 1~7단)와 "
            "가속 곡선은 봉인되어 있으므로, 이 값을 움직이면 상쇄할 수단이 없습니다. 2.16 → 2.50 은 "
            "45mph 기준 목표거리를 약 13m 줄입니다 — 현재 갭 표는 2.16 에 맞춰 0.64초 낮춰둔 값이라 "
            "<b>둘은 한 세트입니다.</b>"
            "<br><br>앞차가 급정거할 때의 과잉 제동을 줄이는 용도로는 쓸 수 없습니다 — "
            "줄어드는 제동량이 곧 줄어드는 차간거리입니다.",
    "options": [(200, "2.0 곡률 크게"), (208, "2.08"), (216, "2.16 (권장)"), (225, "2.25"),
                (240, "2.4"), (250, "2.5 = 평평 (upstream)")],
  },
  "StopDistanceCarrot": {
    "label": "정지 시 앞차 간격", "type": "int",
    "help": "앞차 뒤에 멈출 때 남기는 거리입니다.",
    "options": [(450, "4.5m"), (550, "5.5m (기본)"), (600, "6.0m"), (700, "7.0m")],
  },
  "LongTuningKpV": {
    "label": "종방향 PID · Kp", "type": "int",
    "help": "목표 속도와 실제 속도의 차이에 즉시 반응하는 크기입니다. 이 포트는 Kp/Ki 가 0 이라 "
            "지금까지 피드포워드만으로 굴러왔습니다. 크면 오차를 빨리 잡지만 가감속이 흔들릴 수 "
            "있습니다. 저장값 100 = 1.00.",
    "options": [(0, "0 (사용 안 함)"), (50, "0.50"), (100, "1.00 (캐롯 기본)"), (150, "1.50")],
  },
  "LongTuningKiV": {
    "label": "종방향 PID · Ki", "type": "int",
    "help": "오래 남는 오차를 누적해서 없앱니다. 크면 오버슈트와 진동이 생길 수 있습니다. "
            "저장값은 1/1000 단위라 100 = 0.100 입니다.",
    "options": [(0, "0 (캐롯 기본)"), (50, "0.050"), (100, "0.100"), (200, "0.200")],
  },
  "LongTuningKf": {
    "label": "종방향 PID · 피드포워드", "type": "int",
    "help": "계획된 가속도를 그대로 얼마나 실어 보낼지입니다. 100 이면 계획값 그대로 나가고, "
            "이 트리가 지금까지 해오던 동작입니다. 올리면 가속·감속 양쪽이 함께 강해집니다.",
    "options": [(80, "0.80"), (100, "1.00 (기본)"), (120, "1.20"), (150, "1.50")],
  },
  "RadarLatFactor": {
    "label": "앞차 횡방향 예측 시간", "type": "int",
    "help": "레이더 트랙이 우리 차선으로 들어오는지 판단할 때 몇 초 앞을 내다볼지입니다. "
            "0 이면 예측을 끕니다. 캐롯 원본은 0.60초로 하드코딩돼 있어 그 값이 기본입니다. "
            "크면 컷인을 일찍 잡지만 오탐도 늘 수 있습니다.",
    "options": [(0, "사용 안 함"), (30, "0.30초"), (60, "0.60초 (캐롯 기본)"), (100, "1.00초")],
  },
  "StoppingAccel": {
    "label": "정지 진입 감속 기준", "type": "int",
    "help": "정지 계획이 나와도 차가 이미 이만큼 제동 중일 때까지 기다렸다가 정지 램프로 넘어갑니다. "
            "일찍 넘어가면 마지막 제동을 그 램프가 하게 되어 '멀리서 갑자기 서는' 느낌이 납니다. "
            "0 이면 차종 기본값을 씁니다. 0 에 가까울수록 마지막 제동이 약해집니다.",
    "options": [(0, "차종 기본값"), (-30, "-0.30 약하게"), (-50, "-0.50 (캐롯 기본)"),
                (-70, "-0.70"), (-100, "-1.00 강하게")],
  },
  "RadarReactionFactor": {
    "label": "앞차 가속 지속 가정", "type": "int",
    "help": "앞차가 지금 내는 가감속이 앞으로 얼마나 이어질지 가정하는 정도입니다. 낮추면 앞차 "
            "변화에 더 빨리 반응하고, 높이면 곧 사라질 것으로 보고 부드럽지만 늦게 반응합니다. "
            "너무 낮으면 레이더 노이즈에도 민감해집니다.",
    "options": [(60, "60% 민감하게"), (80, "80%"), (100, "100% (기본)"),
                (140, "140%"), (200, "200% 둔하게")],
  },
  "JLeadFactor3": {
    "label": "앞차 저크 반영", "type": "int",
    "help": "앞차의 가속도 변화율을 계획에 얼마나 반영할지입니다. 브레이크를 막 밟은 앞차와 "
            "이미 풀고 있는 앞차를 구분하는 값이라, 이 포팅이 레이더 쪽 jLead 를 함께 "
            "가져온 이유이기도 합니다. 0 이면 사용 안 함.",
    "options": [(0, "사용 안 함 (기본)"), (50, "약하게 0.5"), (100, "표준 1.0")],
  },
  "AChangeCostStarting": {
    "label": "출발 시 가속 상승 자유도", "type": "int",
    "help": "정지에서 출발할 때 가속을 얼마나 자유롭게 올릴지입니다. 낮을수록 빠르게 붙습니다.",
    "options": [(10, "자유롭게 10 (기본)"), (40, "보통 40"), (100, "부드럽게 100"), (200, "매우 부드럽게 200")],
  },
  "TrafficLightGreenHold": {
    "label": "출발 전 초록불 유지 시간", "type": "int",
    "help": "정지 상태에서 출발하기 전에 초록불 판정이 <b>끊기지 않고</b> 얼마나 유지되어야 하는지입니다. "
            "기본값 없이는 초록으로 바뀐 <b>첫 프레임에 바로 출발</b>합니다."
            "<br><br>실주행 844개 구간 측정: 신호 정지 후 자동 출발 11건 중 <b>9건은 초록이 끝까지 "
            "유지</b>(0.60초~31초)됐고, <b>2건은 0.20초·0.35초 만에 사라지고 각각 0.26초·0.40초 뒤 "
            "다시 빨간불</b>이 됐습니다. 0.35와 0.60 사이가 이 값을 두는 자리입니다."
            "<br><br>카메라 판정이 유일한 근거라서 필요한 값입니다 — 당근 원본은 내비 서비스의 "
            "신호 상태(carrotMan.trafficState)와 교차 검증하지만 이 포트엔 그 서비스가 없습니다. "
            "너무 길게 잡으면 실제 초록불에서 출발이 그만큼 늦어집니다.",
    "options": [(0, "즉시 (기존 동작)"), (3, "0.3초"), (5, "0.5초 (기본)"), (8, "0.8초"), (12, "1.2초")],
  },
  "TrafficStopDistanceAdjust": {
    "label": "정지선 앞 여유", "type": "int",
    "help": "정지선에서 얼마나 앞뒤로 멈출지 조정합니다. 음수면 더 앞에 섭니다.",
    "options": [(-250, "2.5m 앞"), (-150, "1.5m 앞 (기본)"), (0, "정지선"), (150, "1.5m 뒤")],
  },
  "TeslaCutInLead": {
    "label": "끼어들기 선반영", "type": "bool",
    "help": "옆 차선에서 <b>들어오는 중인 차</b>를 앞에 오기 전에 플래너에 넘겨서 미리 거리를 "
            "벌립니다. 지금은 갭이 무너진 <b>뒤에야</b> 알게 됩니다 — 실측(0000007f)에서 목표보다 "
            "25m 부족한 상태(헤드웨이 0.74초)인데 요구 감속이 <b>-0.18m/s²</b>였습니다. 7건 모두 "
            "회복되긴 했지만 대부분 <b>앞차가 멀어져서</b>지 우리가 감속해서가 아니었습니다."
            "<br><br><b>어떻게 미리 아는가</b> — 레이더 트랙마다 차선 중심까지의 거리(dPath)가 "
            "<b>1초 창</b>에서 얼마나 줄고 있는지 봅니다. 한 프레임씩 보는 횡속도는 레이더 노이즈에 "
            "묻혀서, 한 차선을 천천히 넘어오는 차가 안 보입니다. 끼어드는 차는 리드가 되기 "
            "<b>중앙값 13.4초 전부터</b> 보이고 매번 차선 밖에서 먼저 잡힙니다 — 센서가 아니라 "
            "질문이 문제였습니다."
            "<br><br><b>얼마나 미리</b> — 3개 주행 재생 결과 <b>중앙값 2.1초</b>(하위10% 1.1초, "
            "최대 7.1초) 먼저 잡습니다. 호출은 분당 0.24~0.96회이고, 그중 실제로 갭이 무너진 비율이 "
            "<b>73% / 80% / 54%</b>입니다. 54%가 고속도로 주행인데, 빗나간 건 다차선에서 결국 "
            "안 들어온 옆 차선 차들입니다."
            "<br><br><b>틀렸을 때 비용</b> — 감지된 차를 <b>두 번째 장애물(leadTwo)</b>로 넘길 뿐이라, "
            "잘못 잡아도 <b>실제로 거기 있는 실제 차</b>에게 거리를 조금 더 주는 게 전부입니다. "
            "따라가던 리드(leadOne)는 건드리지 않습니다 — 끼어들 때가 비전이 가장 불확실한 "
            "순간이라 일부러 분리했습니다."
            "<br><br>임계값은 취향이 아니라 3개 주행 스윕으로 정했습니다: <b>옆 차선 하나까지만</b>"
            "(차선 반폭의 2배 이내 — 2.5배로 넓히면 고속도로에서 24건 54%가 34건 47%로 나빠지고 "
            "늘어난 건 전부 헛것), 차선 가장자리까지 <b>3초 이내</b>로 좁혀오는 중, 0.5초 연속 확인.",
    "options": [(0, "사용 안 함"), (1, "사용 (기본)")],
  },
  "RadarLeadHoldCm": {
    "label": "근거리 레이더 유지", "type": "int",
    "help": "비전 신뢰도가 잠깐 떨어져도 이 거리 안쪽이면 따라가던 레이더 트랙을 계속 씁니다. "
            "비전으로 넘어가면 거리를 median +4.9m 멀게 읽습니다. 0이면 사용 안 함.",
    "options": [(0, "사용 안 함"), (2000, "20m 이내"), (3000, "30m 이내"), (4000, "40m 이내")],
  },
  "RadarLeadHoldMs": {
    "label": "레이더 유지 시간", "type": "int",
    "help": "위 유지가 최대 얼마나 이어질지입니다. 길면 끊김에 강하고, 짧으면 오래된 트랙을 덜 붙듭니다.",
    "options": [(500, "0.5초"), (1000, "표준 1.0초"), (2000, "2.0초")],
  },
}

SETTINGS = {
  "TeslaStockLong": {
    "label": "순정 ACC 사용", "type": "bool",
    "help": "속도 제어를 차의 순정 ACC에 맡기고 openpilot은 조향만 합니다. 순정 ACC는 이미 "
            "다듬어져 있으므로 롱 튜닝을 아예 건너뛰는 선택지입니다. 재시작해야 반영됩니다.",
    "options": [(0, "CarrotPilot 롱 (기본)"), (1, "순정 ACC")],
  },
  "TeslaCoopSteer": {
    "label": "핸들 같이 돌리기", "type": "bool",
    "help": "운전자가 핸들을 돌리면 손을 놓는 대신 목표 각도를 그쪽으로 옮깁니다. EPS가 조향을 "
            "끊을 만큼 세게 밀 일이 없어집니다. 재시작해야 반영됩니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "사용")],
  },
  "TeslaCoopMaxTorqueCNm": {
    "label": "협조 조향 최대 토크", "type": "int",
    "help": "이 토크에서 목표가 가장 많이 이동합니다. 낮을수록 가볍게 반응합니다.",
    "options": [(150, "가볍게 1.5Nm"), (250, "표준 2.5Nm"), (350, "묵직하게 3.5Nm")],
  },
  "TeslaCoopLatAccelCms": {
    "label": "협조 조향 이동량", "type": "int",
    "help": "최대 토크일 때 목표가 얼마나 옮겨갈지입니다. 횡가속도 기준이라 속도가 붙으면 각도는 줄어듭니다.",
    "options": [(100, "조금 1.0m/s²"), (150, "표준 1.5m/s²"), (220, "많이 2.2m/s²")],
  },
  "TeslaSyncClusterSpeed": {
    "label": "계기판 MAX 속도 동기화", "type": "bool",
    "help": "계기판의 MAX 숫자를 openpilot이 실제로 목표하는 속도에 맞춥니다."
            "<br><br><b>왜 지금까지 안 움직였나</b> — 계기판은 <code>DAS_setSpeed</code>를 보지 "
            "않습니다. MAX 숫자는 DI가 자체적으로 들고 있는 setpoint이고(<code>DI_state."
            "DI_digitalSpeed</code>), 그걸 바꾸는 건 <b>크루즈 레버(STW_ACTN_RQ)</b>뿐입니다. "
            "실측 확인: 레버 DN_2ND에 70→65→60, UP_2ND에 60→65로 움직였고 그동안 실제 차속은 "
            "60~64mph로 무관했습니다."
            "<br><br>그래서 openpilot이 <b>레버를 사람처럼 누릅니다</b> — 한 번에 ±1 또는 ±5씩, "
            "DI가 반영할 시간(약 0.2초)을 두고 목표에 닿을 때까지. 70→63mph면 −5 한 번에 −1 두 번으로 "
            "0.93초, 65→40mph면 −5 다섯 번 1.55초입니다. 실주행에서 목표값이 가장 빠르게 움직인 "
            "10.85mph/s에 대해 16.1mph/s로 따라갑니다. 순정 SCCM 프레임을 복사해서 레버 필드만 "
            "바꾸므로 방향지시등·와이퍼· "
            "차간거리 설정은 그대로 나갑니다."
            "<br><br><b>기본 꺼짐</b> — 이 메시지는 Model S/X에서 <b>기어 셀렉터</b>이기도 합니다"
            "(같은 필드에 FWD·RWD). panda가 속도 4단계와 IDLE 외의 값은 전부 거부하도록 막아뒀지만, "
            "레버를 쓰는 기능이라 직접 켜서 쓰시게 했습니다. panda 재플래시가 필요합니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "사용")],
  },
  "TeslaStockAutopark": {
    "label": "순정 오토파크 허용", "type": "bool",
    "help": "openpilot이 해제된 동안 순정 자동주차 모듈이 차를 몰 수 있게 버스를 넘겨줍니다. "
            "재시작해야 반영됩니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "허용")],
  },
  "DriverMonitorBypass": {
    "label": "드라이버 모니터링 우회", "type": "bool",
    "help": "운전자 주의 감시를 끕니다. 얼굴 미검출·주의 분산 경고와 강제 해제가 발생하지 않습니다. "
            "카메라와 모델은 그대로 돌아가므로 녹화는 유지되고, 전력 절감 효과는 없습니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "우회")],
  },
  "AutoLaneChange": {
    "label": "자동 차선변경", "type": "int",
    "help": "깜빡이만 켜면 핸들을 살짝 트는 동작 없이 차선변경이 시작됩니다. 여기서 고른 시간만큼 "
            "깜빡이를 유지해야 출발하고, 그 사이 사각지대에 차가 잡히면 타이머가 처음부터 다시 "
            "갑니다. 핸들을 트는 기존 방식도 그대로 동작하며, 그쪽은 기다림 없이 바로 시작합니다."
            "<br><br><b>대기시간이 이 기능의 안전장치 전부입니다.</b> 깜빡이는 차선변경뿐 아니라 "
            "교차로 회전에도 켜는 것이라, 32km/h(20mph) 속도 제한 위에서는 깜빡이를 켠 것과 "
            "차가 차선을 옮기기로 정하는 것 사이에 이 시간 말고는 아무것도 없습니다. 회전 전에 "
            "깜빡이를 오래 켜두는 습관이 있으시면 짧게 두지 마세요."
            "<br><br>사각지대 감지와 20mph 하한은 기존과 동일하게 계속 막습니다.",
    "options": [(0, "사용 안 함 (기본)"), (5, "0.5초"), (10, "1초"), (20, "2초"), (30, "3초")],
  },
  "LaneCentering": {
    "label": "차선 중앙 보정", "type": "bool",
    "help": "모델은 차선 중앙을 지키라고 배우지 않아서, 커브에서 자기가 좋다고 판단한 라인을 "
            "탑니다 — 진입에서 바깥으로 벌어졌다가 정점을 깎는 식입니다. 이 옵션은 양쪽 차선의 "
            "중앙을 향해 아주 작은 곡률을 더해서, 갈 곳은 모델이 정하되 중앙에서 얼마나 "
            "벗어나도 되는지만 제한합니다."
            "<br><br>보정량은 최대 0.0012 1/m으로, 90km/h에서 횡가속 0.75m/s²·핸들 3° 정도입니다. "
            "양쪽 차선이 확실할 때(확률 0.6↑, 표준편차 0.3↓, 폭 2.6~4.8m)만 동작하고, "
            "차선변경 중이거나 5m/s 미만이면 부드럽게 풀립니다. ISO 저크·횡가속 한계 위에서 "
            "목표 곡률만 손대므로 조향 한계는 그대로입니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "사용")],
  },
  "LaneCenteringE2EAuthority": {
    "label": "모델 이탈 허용도", "type": "int",
    "help": "모델이 중앙에서 크게 벗어났을 때 그걸 의도로 볼지 실수로 볼지입니다. 15cm 미만은 "
            "항상 되당기고, 50cm 이상부터 이 값만큼 모델에게 양보합니다. 단 모델이 스스로 "
            "확신할 때(경로 표준편차 0.35 이하)만 양보하므로, 헷갈리는 상황에서는 그대로 잡습니다."
            "<br><br>낮출수록 커브 아웃인아웃을 강하게 잡고, 높일수록 주차차량 회피 같은 "
            "의도적인 이탈을 살려둡니다. StarPilot 기본값은 100입니다.",
    "options": [(0, "전부 되당김 0%"), (50, "절반 (기본)"), (100, "모델에 맡김 100%")],
  },
  "LaneCenterOffset": {
    "label": "차선 내 위치", "type": "int",
    "help": "중앙 대신 일부러 한쪽으로 붙여 달립니다. 차선이 좁으면 차선까지 1.1m는 남기도록 "
            "요청한 값보다 적게 들어갑니다. 음수가 왼쪽입니다.",
    "options": [(-20, "왼쪽 20cm"), (-10, "왼쪽 10cm"), (0, "중앙 (기본)"),
                (10, "오른쪽 10cm"), (20, "오른쪽 20cm")],
  },
  "LaneCenteringPauseOnSignal": {
    "label": "깜빡이 중 보정 정지", "type": "bool",
    "help": "깜빡이가 켜져 있는 동안 보정을 풉니다. 차선변경으로 인식되지 않는 손수 하는 "
            "차선 이동을 붙잡지 않게 합니다.",
    "options": [(0, "계속 보정"), (1, "정지 (기본)")],
  },
}

STATE_SERVICES = ['carState', 'radarState', 'selfdriveState', 'longitudinalPlan', 'deviceState']

def _git_commit() -> str:
  """Which build is actually serving. Stale processes are hard to spot otherwise."""
  try:
    return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=os.path.dirname(__file__),
                          capture_output=True, text=True, timeout=3).stdout.strip() or 'unknown'
  except Exception:
    return 'unknown'


GIT_COMMIT = _git_commit()


class State:
  """Polls the live message bus in the background so HTTP requests never block on it."""

  def __init__(self):
    self.lock = threading.Lock()
    self.data: dict = {'onroad': False, 'connected': False}
    self.interventions = InterventionLog()
    threading.Thread(target=self._run, daemon=True).start()

  def _run(self):
    sm = messaging.SubMaster(STATE_SERVICES)
    while True:
      sm.update(100)
      # Same subscription already carries everything an intervention record needs, including
      # longitudinalPlan -- which is the shadow answer from the controller that was not driving.
      try:
        self.interventions.update(sm)
      except Exception:
        cloudlog.exception("intervention_log update failed")
      cs, rs = sm['carState'], sm['radarState']
      lead = rs.leadOne
      with self.lock:
        self.data = {
          'connected': sm.seen['carState'],
          'onroad': sm['deviceState'].started,
          'engaged': sm['selfdriveState'].enabled,
          'vEgo': round(cs.vEgo, 2),
          'gap': int(cs.cruiseState.gapAdjust),
          'blindspot': [bool(cs.leftBlindspot), bool(cs.rightBlindspot)],
          'aTarget': round(sm['longitudinalPlan'].aTarget, 2),
          # What is actually running, not what the parameter says -- the parameter is read
          # once at startup, so the two disagree between a change and the next restart. None
          # when plannerd has not published yet: the enum's zero is 'stock', and reporting that
          # for "no data" would be the same misleading answer the parameter already gives.
          'planner': (str(sm['longitudinalPlan'].plannerSource)
                      if sm.seen['longitudinalPlan'] else None),
          'lead': {
            'status': bool(lead.present),
            'source': ('R' if lead.radar else 'V') if lead.present else None,
            'trackId': int(lead.radarTrackId),
            'dRel': round(lead.dRel, 1),
            'vLead': round(lead.vLead, 1),
            'prob': round(lead.modelProb, 2),
          },
          # What the auto cruise speed is being decided from. cruiseTarget is what the planner
          # settled on after map_cruise, the curve controller and everything else had their say,
          # so seeing it next to the raw sources is the whole point -- the interesting moments
          # are the ones where they disagree.
          'map': {
            'valid': bool(cs.navMap.valid),
            'base': round(cs.navMap.baseSpeedLimit, 2),
            'posted': round(cs.navMap.mapSpeedLimit, 2),
            'fleet': round(cs.navMap.fleetSplineSpeed, 2),
            'roadClass': int(cs.navMap.roadClass),
            'ramp': int(cs.navMap.rampType),
            'conf': int(cs.navMap.splineConfidence),
            'offset': round(cs.navMap.speedOffset, 2),
            'vSet': round(cs.cruiseState.speed, 2),
            'target': round(sm['longitudinalPlan'].cruiseTarget, 1),
            # The road-ahead cubic, as curvature at 60m and at 150m rather than as coefficients
            # -- c2 and c3 mean nothing at a glance, and what the cap actually reads is the
            # curvature across that window. 0 range or 0 health means it is not being used.
            'curv60': round(2 * cs.navMap.curvC2 + 6 * cs.navMap.curvC3 * 60.0, 5),
            'curv150': round(2 * cs.navMap.curvC2 + 6 * cs.navMap.curvC3 * 150.0, 5),
            'curvRange': round(cs.navMap.curvRange, 0),
            'curvHealth': int(cs.navMap.curvHealth),
          },
        }

  def get(self):
    with self.lock:
      return dict(self.data)


class Handler(BaseHTTPRequestHandler):
  state: State
  params: Params
  can: 'CanSource'
  videos: 'video_source.Mp4Cache'
  models: 'ModelManager'

  def log_message(self, *a):
    pass  # don't spam the console on every poll

  def _send(self, code, body, ctype='application/json'):
    payload = body.encode() if isinstance(body, str) else body
    self.send_response(code)
    self.send_header('Content-Type', ctype)
    self.send_header('Content-Length', str(len(payload)))
    self.send_header('Cache-Control', 'no-store')
    self.end_headers()
    self.wfile.write(payload)

  def _send_file(self, path: str, ctype: str, download: str | None = None):
    """Serve a file, honouring Range -- without it a <video> can't seek and Safari won't
    play at all."""
    size = os.path.getsize(path)
    start, end = 0, size - 1
    rng = self.headers.get('Range', '')
    partial = rng.startswith('bytes=') and '-' in rng
    if partial:
      lo, _, hi = rng[6:].partition('-')
      start = int(lo) if lo else 0
      end = int(hi) if hi else size - 1
      end = min(end, size - 1)
      if start > end:
        self.send_response(416)
        self.send_header('Content-Range', f'bytes */{size}')
        self.end_headers()
        return

    self.send_response(206 if partial else 200)
    self.send_header('Content-Type', ctype)
    self.send_header('Content-Length', str(end - start + 1))
    self.send_header('Accept-Ranges', 'bytes')
    if partial:
      self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
    if download:
      self.send_header('Content-Disposition', f'attachment; filename="{download}"')
    self.end_headers()

    remaining = end - start + 1
    with open(path, 'rb') as f:
      f.seek(start)
      while remaining > 0:
        chunk = f.read(min(256 * 1024, remaining))
        if not chunk:
          break
        try:
          self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
          return   # the player seeked away; nothing to report
        remaining -= len(chunk)

  def do_GET(self):
    if self.path.startswith('/api/version'):
      return self._send(200, json.dumps({'commit': GIT_COMMIT}))

    if self.path.startswith('/api/state'):
      return self._send(200, json.dumps({**self.state.get(), 'commit': GIT_COMMIT}))

    if self.path.startswith('/api/models'):
      return self._send(200, json.dumps(self.models.snapshot(self._onroad())))

    if self.path.startswith('/api/routes'):
      return self._send(200, json.dumps({'routes': list_routes(), **self.can.state()}))

    if self.path.startswith('/api/can'):
      dec = self.can.get()
      if dec is None:
        return self._send(200, json.dumps({
          'messages': [], 'dbc': None, 'total': 0, **self.can.state(),
          'error': '차량 미연결 · 저장된 route를 재생해서 볼 수 있습니다'}))
      snap = dec.snapshot('changed=1' in self.path,
                          include_unseen='unseen=0' not in self.path)
      return self._send(200, json.dumps({**snap, **self.can.state()}))

    if self.path.startswith('/api/vehicle'):
      return self._send(200, json.dumps({**vehicle_state.build(self.can.get()), **self.can.state()}))

    if self.path.startswith('/api/events'):
      # ?event=<name> returns the full sample window; without it, just the index
      qs = self.path.split('?', 1)[1] if '?' in self.path else ''
      name = next((p[6:] for p in qs.split('&') if p.startswith('event=')), None)
      if name:
        ev = read_event(name)
        if ev:
          # Resolved on read, not at record time: the segment the event landed in is only
          # knowable once loggerd has closed it, which is after the event is written.
          ev['segment'] = locate_segment(ev.get('route', ''), ev.get('wallTime', 0))
        return self._send(200 if ev else 404, json.dumps(ev or {'error': 'not found'}))
      return self._send(200, json.dumps({'events': list_events()}))

    if self.path.startswith('/api/settings'):
      def pack(table):
        out = {}
        for k, cfg in table.items():
          try:
            # not get_bool(): it ignores the declared default and reads False until first write
            v = self.params.get(k, return_default=True)
            v = int(bool(v)) if cfg['type'] == 'bool' else int(v)
          except Exception:
            v = None
          out[k] = {'value': v, 'label': cfg['label'], 'help': cfg['help'],
                    'options': [{'v': ov, 'label': ol} for ov, ol in cfg['options']]}
        return out
      return self._send(200, json.dumps({
        'settings': pack(SETTINGS),
        'map': pack(MAP_SETTINGS),
        'carrot': pack(CARROT_SETTINGS),
        # The carrot block only does anything when its planner is the one running, and that is
        # read at startup -- so the page can say plainly whether these are live right now.
        'carrotActive': bool(self.params.get('CarrotLongEnabled', return_default=True)),
        'engaged': bool(self.state.get().get('engaged')),
      }))

    if self.path.startswith('/api/shadow/scan'):
      qs = self.path.split('?', 1)[1] if '?' in self.path else ''
      route = next((p[6:] for p in qs.split('&') if p.startswith('route=')), None)
      st = self.shadow_scan.state()
      if route and st.get('route') != route and st.get('status') != 'running':
        st = {'status': 'idle'}                      # a stale result from a different route
      return self._send(200, json.dumps(st))

    if self.path.startswith('/api/shadow'):
      qs = self.path.split('?', 1)[1] if '?' in self.path else ''
      route = next((p[6:] for p in qs.split('&') if p.startswith('route=')), None)
      if route:
        return self._send(200, json.dumps({'segments': shadow_replay.list_segments(route),
                                           **shadow_replay.route_floor(route)}))
      st = self.shadow.state()
      st['routes'] = [r['name'] for r in video_source.list_videos()]
      st['engaged'] = bool(self.state.get().get('engaged'))
      st['commit'] = GIT_COMMIT
      return self._send(200, json.dumps(st))

    if self.path.startswith('/api/videos'):
      return self._send(200, json.dumps({'routes': video_source.list_videos()}))

    page = self.path.split('?')[0].rstrip('/')

    # /v/<route>/<seg>.mp4[?cam=road|wide|driver][&q=copy|h264] -- in a container a browser plays
    if page.startswith('/v/'):
      qs = self.path.split('?', 1)[1] if '?' in self.path else ''
      cam = next((p[4:] for p in qs.split('&') if p.startswith('cam=')), 'road')
      codec = next((p[2:] for p in qs.split('&') if p.startswith('q=')), 'copy')
      # Timed and printed on purpose: log_message is silenced against poll spam, but a video
      # load is rare enough, and expensive enough when it misses cache, to be worth a line --
      # this is the only way to tell a slow build from a slow network hop after the fact.
      t0 = time.monotonic()
      try:
        route, seg = page[3:].rsplit('/', 1)
        path = self.videos.get(route, int(seg.removesuffix('.mp4')), cam=cam, codec=codec)
      except (ValueError, FileNotFoundError):
        return self._send(404, '{}')
      except Exception as e:
        return self._send(500, json.dumps({'error': f'{type(e).__name__}: {e}'}))
      build_ms = int((time.monotonic() - t0) * 1000)
      print(f'[video] {route}--{seg} cam={cam} q={codec} build={build_ms}ms size={os.path.getsize(path)}',
            flush=True)
      return self._send_file(path, 'video/mp4')

    # /dl/<route>/<seg>/<file> -- the originals, untouched
    if page.startswith('/dl/'):
      try:
        route, seg, name = page[4:].rsplit('/', 2)
        path = video_source.raw_path(route, int(seg), name)
      except (ValueError, FileNotFoundError):
        return self._send(404, '{}')
      return self._send_file(path, 'application/octet-stream', f'{route}--{seg}-{name}')

    if page == '/live':
      return self._send(200, PAGE_LIVE, 'text/html; charset=utf-8')
    if page == '/events':
      return self._send(200, PAGE_EVENTS, 'text/html; charset=utf-8')
    if page == '/can':
      return self._send(200, PAGE_CAN, 'text/html; charset=utf-8')
    if page == '/vehicle':
      return self._send(200, PAGE_VEHICLE, 'text/html; charset=utf-8')
    if page == '/videos':
      return self._send(200, PAGE_VIDEO, 'text/html; charset=utf-8')
    if page == '/shadow':
      return self._send(200, PAGE_SHADOW, 'text/html; charset=utf-8')
    return self._send(200, PAGE_INDEX, 'text/html; charset=utf-8')

  def _onroad(self) -> bool:
    return bool(self.state.get().get('onroad'))

  def _read_json(self):
    n = int(self.headers.get('Content-Length', 0))
    return json.loads(self.rfile.read(n) or b'{}')

  def do_POST(self):
    if self.path.startswith('/api/models/'):
      try:
        req = self._read_json()
      except json.JSONDecodeError:
        return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))
      # Checked here rather than only in the page: a stale tab, a second phone or a curl would
      # otherwise be able to swap the model out from under a car that is driving.
      if self._onroad():
        return self._send(409, json.dumps({'error': '주행 중에는 모델을 변경할 수 없습니다'}))
      action = self.path.split('?')[0].rsplit('/', 1)[-1]
      model_id = str(req.get('id') or '')
      if action == 'download':
        code, out = self.models.download(model_id)
      elif action == 'select':
        code, out = self.models.select(model_id)
      elif action == 'delete':
        code, out = self.models.delete(model_id, self._onroad())
      else:
        code, out = 404, {'error': f'알 수 없는 요청: {action}'}
      return self._send(code, json.dumps(out))

    if self.path.startswith('/api/shadow/scan'):
      n = int(self.headers.get('Content-Length', 0))
      try:
        req = json.loads(self.rfile.read(n) or b'{}')
      except json.JSONDecodeError:
        return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))
      route = req.get('route')
      if not route:
        return self._send(400, json.dumps({'error': '주행을 지정하세요'}))
      out = self.shadow_scan.start(route)
      return self._send(409 if 'error' in out else 200, json.dumps(out))

    if self.path.startswith('/api/shadow'):
      n = int(self.headers.get('Content-Length', 0))
      try:
        req = json.loads(self.rfile.read(n) or b'{}')
      except json.JSONDecodeError:
        return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))
      route, seg = req.get('route'), req.get('seg')
      if not route or seg is None:
        return self._send(400, json.dumps({'error': '경로와 세그먼트를 지정하세요'}))
      # solve defaults true so an old client still gets what it expects; the page sends false
      # when it only wants what the drive recorded.
      out = self.shadow.start(route, int(seg), solve=bool(req.get('solve', True)))
      return self._send(409 if 'error' in out else 200, json.dumps(out))

    if self.path.startswith('/api/replay'):
      n = int(self.headers.get('Content-Length', 0))
      try:
        req = json.loads(self.rfile.read(n) or b'{}')
      except json.JSONDecodeError:
        return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))

      route = req.get('route')
      if not route:
        self.can.stop_replay()
        return self._send(200, json.dumps({'ok': True, **self.can.state()}))

      err = self.can.start_replay(route)
      if err:
        return self._send(400, json.dumps({'error': err}))
      return self._send(200, json.dumps({'ok': True, **self.can.state()}))

    if not self.path.startswith('/api/settings'):
      return self._send(404, '{}')

    n = int(self.headers.get('Content-Length', 0))
    try:
      req = json.loads(self.rfile.read(n) or b'{}')
    except json.JSONDecodeError:
      return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))

    changes = req.get('changes') or {}
    known = {**SETTINGS, **MAP_SETTINGS, **CARROT_SETTINGS}
    unknown = [k for k in changes if k not in known]
    if unknown:
      return self._send(400, json.dumps({'error': f'알 수 없는 설정: {", ".join(unknown)}'}))

    for key, value in changes.items():
      cfg = known[key]
      if value not in [v for v, _ in cfg['options']]:
        return self._send(400, json.dumps({'error': f'{cfg["label"]}: 허용되지 않은 값'}))
      try:
        # Params is typed: BOOL wants a real bool and INT a real int, not their string forms
        self.params.put(key, bool(value) if cfg['type'] == 'bool' else int(value))
      except (TypeError, ValueError) as e:
        return self._send(400, json.dumps({'error': f'{cfg["label"]} 저장 실패: {e}'}))

    # Written either way. radard and the planner only re-read while disengaged, so a change
    # made mid-drive takes effect at the next engage instead of moving under the driver.
    engaged = bool(self.state.get().get('engaged'))
    return self._send(200, json.dumps({'ok': True, 'count': len(changes), 'engaged': engaged}))




PAGE_EVENTS = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>개입 기록</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--vision:#F5B942;--ok:#4CC38A;--bad:#E5484D;
--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--vision:#9A6210;--ok:#1B7F53;--bad:#C42B30}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:16px;padding-bottom:calc(16px + env(safe-area-inset-bottom))}
h1{font-size:15px;margin:0 0 4px;letter-spacing:-.01em}
.back{display:inline-block;font-family:var(--m);font-size:11px;color:var(--dim);text-decoration:none;margin-bottom:10px}.back:hover{color:var(--radar)}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:10px;cursor:pointer}
.card:hover{border-color:var(--radar)}
.row{display:flex;justify-content:space-between;align-items:baseline;gap:10px;flex-wrap:wrap}
.when{font-family:var(--m);font-size:12px}
.tag{font-family:var(--m);font-size:9.5px;letter-spacing:.1em;text-transform:uppercase;
padding:2px 7px;border-radius:5px;border:1px solid var(--line);color:var(--mut)}
.tag.brake{color:var(--bad);border-color:var(--bad)}
.tag.steer{color:var(--vision);border-color:var(--vision)}
.tag.gas{color:var(--ok);border-color:var(--ok)}
.d{font-family:var(--m);font-size:19px;font-variant-numeric:tabular-nums}
.d.neg{color:var(--bad)}
.meta{font-family:var(--m);font-size:11px;color:var(--dim);margin-top:8px}
.meta b{color:var(--mut);font-weight:400}
.empty{font-family:var(--m);font-size:12px;color:var(--dim);text-align:center;padding:40px 0}
canvas{width:100%;height:150px;display:block;margin-top:10px}
.legend{font-family:var(--m);font-size:10px;color:var(--dim);margin-top:6px}
.legend i{display:inline-block;width:9px;height:2px;vertical-align:middle;margin-right:4px}
a.vid{font-family:var(--m);font-size:11px;color:var(--radar);text-decoration:none;margin-right:14px}
</style></head><body>
<a class="back" href="/">&larr; 인덱스</a>
<h1>개입 기록</h1>
<div class="sub">운전자가 시스템을 끈 순간 · 그때 두 제어기가 각각 원한 것</div>
<div id="list"><div class="empty">불러오는 중…</div></div>
<script>
const fmt = t => new Date(t*1000).toLocaleString('ko-KR',{month:'2-digit',day:'2-digit',
  hour:'2-digit',minute:'2-digit',second:'2-digit'});
const CAUSE = {brake:'브레이크', steer:'조향', gas:'가속'};

function draw(cv, s){
  const ctx = cv.getContext('2d'), W = cv.width = cv.clientWidth*2, H = cv.height = 300;
  ctx.clearRect(0,0,W,H);
  const vals = s.flatMap(p => [p.opAccel, p.aEgo]);
  const lo = Math.min(-1, ...vals), hi = Math.max(1, ...vals);
  const y = v => H - ((v-lo)/(hi-lo))*(H-20) - 10, x = i => (i/(s.length-1))*W;
  ctx.strokeStyle = getComputedStyle(document.body).getPropertyValue('--line');
  ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0,y(0)); ctx.lineTo(W,y(0)); ctx.stroke();
  const line = (key, col) => {
    ctx.strokeStyle = col; ctx.lineWidth = 3; ctx.beginPath();
    s.forEach((p,i) => i ? ctx.lineTo(x(i), y(p[key])) : ctx.moveTo(x(i), y(p[key])));
    ctx.stroke();
  };
  const cs = getComputedStyle(document.body);
  line('aEgo', cs.getPropertyValue('--mut'));      // 실제로 일어난 일
  line('opAccel', cs.getPropertyValue('--radar')); // openpilot이 원한 것
}

async function open_(name, el){
  if (el.dataset.open) { el.querySelector('.detail')?.remove(); delete el.dataset.open; return; }
  const r = await fetch('/api/events?event=' + encodeURIComponent(name));
  const e = await r.json();
  if (!e.samples) return;
  // Deep-link straight to the segment the event landed in, and to the moment inside it. Without
  // the segment this was a link to the video index and a minute of scrubbing to find the event.
  let seg = '';
  if (e.route) {
    const s = e.segment;
    const q = new URLSearchParams({route: e.route});
    if (s) {
      q.set('seg', s.seg);
      if (s.offset != null) q.set('t', s.offset);
    }
    const label = s ? `영상 보기 · 세그 ${s.seg}` : '영상 보기';
    seg = `<a class="vid" href="/videos?${q}">${label}</a>`;
  }
  const d = document.createElement('div');
  d.className = 'detail';
  d.innerHTML = `<canvas></canvas>
    <div class="legend"><i style="background:var(--radar)"></i>openpilot 요구
      &nbsp;&nbsp;<i style="background:var(--mut)"></i>실제 가속도</div>
    <div class="meta">${seg}<a class="vid" href="/can">CAN 리플레이</a>
      <b>route</b> ${e.route || '-'}${e.segment ? ` <b>세그</b> ${e.segment.seg}` +
        (e.segment.offset != null ? ` (+${e.segment.offset}s)` : '') : ''}</div>`;
  el.appendChild(d);
  el.dataset.open = '1';
  draw(d.querySelector('canvas'), e.samples);
}

async function load(){
  const r = await fetch('/api/events');
  const {events} = await r.json();
  const box = document.getElementById('list');
  if (!events.length){ box.innerHTML = '<div class="empty">아직 기록된 개입이 없습니다</div>'; return; }
  box.innerHTML = '';
  for (const e of events){
    const el = document.createElement('div');
    el.className = 'card';
    const lead = e.leadStatus ? `${e.leadDRel}m ${e.leadRadar?'R':'V'}` : '없음';
    el.innerHTML = `<div class="row">
        <span class="when">${fmt(e.wallTime)}</span>
        <span class="tag ${e.cause}">${CAUSE[e.cause]||e.cause}</span>
        <span class="d ${e.disagreement<0?'neg':''}">${e.disagreement>0?'+':''}${e.disagreement}</span>
      </div>
      <div class="meta"><b>속도</b> ${e.vEgo}m/s &nbsp; <b>앞차</b> ${lead}
        &nbsp; <b>롱</b> ${e.stockLong?'순정 ACC':'openpilot'}
        &nbsp; <b>op</b> ${e.opAccel} <b>실제</b> ${e.aEgo}</div>`;
    el.onclick = () => open_(e.name, el);
    box.appendChild(el);
  }
}
load();
</script></body></html>"""


PAGE_LIVE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>openpilot tuning</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--vision:#F5B942;--ok:#4CC38A;--bad:#E5484D;
--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--vision:#9A6210;--ok:#1B7F53;--bad:#C42B30}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:16px;padding-bottom:calc(16px + env(safe-area-inset-bottom))}
h1{font-size:15px;margin:0 0 4px;letter-spacing:-.01em}
.back{display:inline-block;font-family:var(--m);font-size:11px;color:var(--dim);text-decoration:none;margin-bottom:10px}.back:hover{color:var(--radar)}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;
margin-bottom:12px}
.h{font-family:var(--m);font-size:10px;letter-spacing:.14em;text-transform:uppercase;
color:var(--dim);margin-bottom:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(88px,1fr));gap:12px}
.k{font-family:var(--m);font-size:9.5px;letter-spacing:.1em;text-transform:uppercase;
color:var(--dim);margin-bottom:4px}
.v{font-family:var(--m);font-size:20px;font-variant-numeric:tabular-nums;line-height:1.1}
.v small{font-size:11px;color:var(--mut);margin-left:2px}
.src{display:inline-flex;align-items:center;justify-content:center;min-width:30px;height:30px;
border-radius:8px;font-family:var(--m);font-weight:700;font-size:16px}
.src.R{background:var(--radar);color:#04121b}.src.V{background:var(--vision);color:#1b1304}
.src.none{background:var(--line);color:var(--dim);font-size:12px}
.row{display:flex;justify-content:space-between;align-items:center;gap:14px;padding:12px 0;
border-bottom:1px solid var(--line)}.row:last-child{border-bottom:0}
.row .lab{font-size:14px;margin-bottom:3px}.row .hlp{font-size:11.5px;color:var(--mut);line-height:1.45}
select{background:var(--bg);color:var(--tx);border:1px solid var(--line);border-radius:9px;
padding:9px 10px;font-size:13.5px;font-family:inherit;max-width:190px}
select:focus-visible{outline:2px solid var(--radar);outline-offset:1px}
select.dirty{border-color:var(--hot,#F5B942)}
.applybar{position:sticky;bottom:0;background:var(--bg);padding:12px 0 4px;
display:flex;gap:10px;align-items:center}
button.apply{flex:1;background:var(--ok);color:#04140c;border:0;border-radius:10px;
padding:13px;font-size:14.5px;font-weight:600;cursor:pointer;font-family:inherit}
button.apply[disabled]{background:var(--line);color:var(--dim);cursor:default}
.dirtynote{font-size:12px;color:var(--mut)}
.mrow{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:11px 0;
border-bottom:1px solid var(--line)}.mrow:last-child{border-bottom:0}
.mrow .dot{font-family:var(--m);color:var(--dim);margin-right:7px}
.mrow.on .dot{color:var(--ok)}
.mrow .nm{font-size:14px}
.mrow .st{font-family:var(--m);font-size:11px;color:var(--mut);margin-top:3px}
.mrow .st.bad{color:var(--bad)}
.mbtns{display:flex;gap:7px;flex-shrink:0}
button.mb{background:var(--bg);color:var(--tx);border:1px solid var(--line);border-radius:8px;
padding:8px 12px;font-size:12.5px;font-family:inherit;cursor:pointer}
button.mb:hover:not([disabled]){border-color:var(--radar)}
button.mb.pri{border-color:var(--ok);color:var(--ok)}
button.mb.del{border-color:var(--line);color:var(--dim)}
button.mb[disabled]{opacity:.4;cursor:default}
.bar{height:3px;background:var(--line);border-radius:2px;margin-top:6px;overflow:hidden}
.bar i{display:block;height:100%;background:var(--radar);transition:width .3s}
#msg{position:fixed;left:16px;right:16px;bottom:calc(16px + env(safe-area-inset-bottom));
background:var(--card);border:1px solid var(--line);border-radius:10px;padding:11px 14px;
font-size:13px;opacity:0;transform:translateY(8px);transition:.2s;pointer-events:none}
#msg.show{opacity:1;transform:none}#msg.err{border-color:var(--bad);color:var(--bad)}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<a class="back" href="/">← 메뉴</a>
<h1>Live · 앞차 인식과 설정</h1>
<div class="sub" id="conn">연결 중…</div>

<div class="card"><div class="h">Lead perception</div>
  <div class="grid">
    <div><div class="k">source</div><div id="src" class="src none">–</div></div>
    <div><div class="k">거리</div><div class="v"><span id="drel">–</span><small>m</small></div></div>
    <div><div class="k">앞차 속도</div><div class="v"><span id="vlead">–</span><small>m/s</small></div></div>
    <div><div class="k">track</div><div class="v" id="trk">–</div></div>
    <div><div class="k">model prob</div><div class="v" id="prob">–</div></div>
  </div>
</div>

<div class="card"><div class="h">Vehicle</div>
  <div class="grid">
    <div><div class="k">속도</div><div class="v"><span id="vego">–</span><small>mph</small></div></div>
    <div><div class="k">gap</div><div class="v" id="gap">–</div></div>
    <div><div class="k">aTarget</div><div class="v" id="atgt">–</div></div>
    <div><div class="k">상태</div><div class="v" style="font-size:13px"><span id="eng" class="pill">–</span></div></div>
    <div><div class="k">blindspot</div><div class="v" style="font-size:13px"><span id="bs" class="pill">–</span></div></div>
  </div>
</div>

<div class="card"><div class="h">지도 자동 크루즈 <span id="mapState" class="hlp"></span></div>
  <div class="grid">
    <div><div class="k">크루즈 목표</div><div class="v"><span id="mtgt">–</span><small>mph</small></div></div>
    <div><div class="k">설정속도</div><div class="v"><span id="mset">–</span><small>mph</small></div></div>
    <div><div class="k">base 제한</div><div class="v"><span id="mbase">–</span><small>mph</small></div></div>
    <div><div class="k">게시 제한</div><div class="v"><span id="mpost">–</span><small>mph</small></div></div>
    <div><div class="k">fleet 속도</div><div class="v"><span id="mfleet">–</span><small>mph</small></div></div>
    <div><div class="k">도로등급</div><div class="v" id="mrc">–</div></div>
    <div><div class="k">램프</div><div class="v" style="font-size:13px"><span id="mramp" class="pill">–</span></div></div>
    <div><div class="k">위치신뢰</div><div class="v"><span id="mconf">–</span><small>%</small></div></div>
    <div style="grid-column:1/-1"><div class="k">지도 곡률 (반경)</div>
      <div class="v" style="font-size:13px"><span id="mcurv">–</span></div></div>
  </div>
</div>

<div class="card"><div class="h">Driving Model <span id="mdlState" class="hlp"></span></div>
  <div id="models"></div>
  <div class="hlp" id="mdlNote" style="margin-top:10px"></div>
</div>

<div class="card"><div class="h">지도 자동 크루즈 설정</div><div id="mapSettings"></div></div>

<div class="card"><div class="h">Settings</div><div id="settings"></div></div>

<div class="card"><div class="h">CarrotPilot 설정 <span id="carrotState" class="hlp"></span></div>
  <div id="carrotSettings"></div>
</div>

<div class="card">
  <div class="applybar">
    <button class="apply" id="apply" disabled>반영</button>
    <span class="dirtynote" id="dirty"></span>
  </div>
</div>
<div id="msg"></div>

<script>
const $=i=>document.getElementById(i);
let engaged=false;

function toast(t,err){const m=$('msg');m.textContent=t;m.className='show'+(err?' err':'');
  clearTimeout(m._t);m._t=setTimeout(()=>m.className='',2600);}

const MPS_TO_MPH=2.23694;
// Same names map_cruise.py uses, and the same convention for zero: a limit of 0 is "this source
// has nothing here", which on a ramp is the useful answer rather than a missing one.
const ROAD_CLASS={0:'미상',1:'고속도로',4:'간선',5:'집산',6:'국지'};
const RAMP_TYPE={0:'—',1:'진입',2:'진출'};
function mph(v){return (v==null||v<=0)?'–':(v*MPS_TO_MPH).toFixed(0);}
function updateMap(m){
  m=m||{};
  const st=$('mapState');
  st.textContent=m.valid?(m.offset>0?`차량 오프셋 +${(m.offset*MPS_TO_MPH).toFixed(0)}mph`:''):'지도 데이터 없음';
  $('mtgt').textContent=m.target!=null?(m.target*0.621371).toFixed(0):'–';  // cruiseTarget is kph
  $('mset').textContent=mph(m.vSet);
  $('mbase').textContent=mph(m.base);
  $('mpost').textContent=mph(m.posted);
  $('mfleet').textContent=mph(m.fleet);
  $('mrc').textContent=ROAD_CLASS[m.roadClass]??m.roadClass??'–';
  $('mconf').textContent=m.conf??'–';
  const r=$('mramp');
  r.textContent=RAMP_TYPE[m.ramp]??'–';
  r.className='pill '+(m.ramp?'on':'off');
  // Curvature shown as the radius it means. 1/m is unreadable at a glance; "R120" is a corner.
  // Anything past a few km of radius is straight road, so it is reported as such rather than
  // as a number that would only invite reading meaning into noise.
  const rad=k=>{const a=Math.abs(k||0); return a<3e-4?'직선':'R'+Math.round(1/a);};
  $('mcurv').textContent = (m.curvHealth>0 && m.curvRange>0)
    ? `60m ${rad(m.curv60)} · 150m ${rad(m.curv150)} · 유효 ${m.curvRange}m`
    : '없음 (health 0)';
}

async function poll(){
  try{
    const s=await(await fetch('/api/state')).json();
    engaged=s.engaged;
    $('conn').textContent=s.connected?(s.onroad?'주행 중 · onroad':'정차 · offroad')
                                     :'openpilot 대기 중';
    const L=s.lead||{};
    const el=$('src');
    el.className='src '+(L.source||'none');
    el.textContent=L.source||'–';
    $('drel').textContent=L.status?L.dRel:'–';
    $('vlead').textContent=L.status?L.vLead:'–';
    $('trk').textContent=L.status&&L.trackId>=0?L.trackId:'–';
    $('prob').textContent=L.status?L.prob:'–';
    $('vego').textContent=s.vEgo!=null?(s.vEgo*2.23694).toFixed(1):'–';
    $('gap').textContent=s.gap||'–';
    $('atgt').textContent=s.aTarget??'–';
    const e=$('eng');e.textContent=s.engaged?'engaged':'disengaged';
    e.className='pill '+(s.engaged?'on':'off');
    const b=$('bs'),[l,r]=s.blindspot||[false,false];
    b.textContent=l&&r?'L R':l?'L':r?'R':'없음';
    b.className='pill '+((l||r)?'on':'off');
    updateMap(s.map);
  }catch(e){$('conn').textContent='디바이스에 연결할 수 없습니다';}
}

// Driving model selection. Applied at the next modeld start rather than swapped under a
// running model, so the card always shows both what is loaded and what was asked for.
let mdl={models:[],running:null,selected:'stock',onroad:false}, mdlBusy=false;

function modelName(id){
  const m=(mdl.models||[]).find(x=>x.id===id);
  return m?m.name:'–';
}

// Failure text comes from an exception -- a URL or a checksum, but not something to hand to
// innerHTML unescaped.
function esc(s){const d=document.createElement('div');d.textContent=s;return d.innerHTML;}

function renderModels(){
  $('mdlState').textContent=`· 현재 실행 중 ${mdl.running?modelName(mdl.running):'보고 없음'}`
    +` · 다음 실행 ${modelName(mdl.selected)}`;
  $('mdlNote').textContent=mdl.onroad
    ? '주행 중에는 모델을 변경할 수 없습니다 · 정차 후 offroad에서 변경하세요'
    : '선택은 다음 modeld 시작(다음 주행)부터 적용됩니다.';
  const box=$('models');box.innerHTML='';
  for(const m of mdl.models||[]){
    const sel=m.id===mdl.selected, dl=!!m.downloading;
    const row=document.createElement('div');
    row.className='mrow'+(sel?' on':'');
    const verifying=dl&&m.stage==='verify';
    let st;
    if(m.builtin) st='Built in';
    else if(verifying) st='Verifying · 이 빌드에서 로드되는지 확인 중';
    else if(dl) st=`Downloading ${m.progress||0}%`;
    else st=m.installed?'Installed':'Not installed';
    if(sel) st+=' · Selected';
    // Invalid outranks the rest: the file is there and intact, but this build cannot load it.
    // A warning is the weaker case -- the check could not run, so modeld's fallback decides.
    const bad=m.invalid||(!dl&&m.error);
    const txt=bad?('Invalid · '+esc(m.error||'')):(m.warn?(st+' · 확인 못함 · '+esc(m.warn)):st);
    const left=document.createElement('div');
    left.innerHTML=`<div class="nm"><span class="dot">${sel?'●':'○'}</span>${m.name}</div>`
      +`<div class="st${bad?' bad':''}">${txt}</div>`
      +(dl&&!verifying?`<div class="bar"><i style="width:${m.progress||0}%"></i></div>`:'');
    row.appendChild(left);
    const btns=document.createElement('div');btns.className='mbtns';
    const add=(label,cls,act,off)=>{
      const b=document.createElement('button');
      b.className='mb '+cls;b.textContent=label;
      b.disabled=!!(off||mdl.onroad||mdlBusy||dl);
      b.onclick=()=>modelAction(act,m.id);
      btns.appendChild(b);
    };
    if(m.installed) add('Select','pri','select',sel||m.invalid);
    if(!m.builtin && !m.installed) add('Download','pri','download');
    if(!m.builtin && m.installed) add('Delete','del','delete');
    row.appendChild(btns);
    box.appendChild(row);
  }
}

async function modelAction(action,id){
  mdlBusy=true;renderModels();
  try{
    const r=await fetch('/api/models/'+action,{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
    const d=await r.json();
    if(!r.ok) toast(d.error||'요청에 실패했습니다',1);
    else if(action==='select') toast(`${modelName(id)} 선택됨 · 다음 주행부터 적용`);
    else if(action==='download') toast(`${modelName(id)} 내려받는 중`);
    else toast(`${modelName(id)} 삭제됨`);
  }catch(e){toast('디바이스에 연결할 수 없습니다',1);}
  mdlBusy=false;
  await loadModels();
}

async function loadModels(){
  try{ mdl=await(await fetch('/api/models')).json(); }catch(e){}
  renderModels();
}

// One timer chain, rescheduled from itself -- a refresh triggered by a button press must not
// start a second one. Faster only while something is actually downloading.
function scheduleModels(){
  const busy=(mdl.models||[]).some(m=>m.downloading);
  setTimeout(async()=>{await loadModels();scheduleModels();}, busy?700:3000);
}

let cfg={}, mapCfg={}, carrotCfg={}, staged={}, carrotActive=false, livePlanner=null;

function renderSettings(){
  renderInto($('settings'), cfg);
  renderInto($('mapSettings'), mapCfg);
  renderInto($('carrotSettings'), carrotCfg);
  // These only take effect when carrot's planner is the one running, and that is decided at
  // startup -- so say so rather than letting them look live when they are not.
  // livePlanner is what plannerd actually published; carrotActive is only what the parameter
  // says. They differ between changing the toggle and restarting, which is exactly when a
  // misleading label would cost the most.
  if(livePlanner === 'carrot')      $('carrotState').textContent = '· 사용 중 (plannerd 확인됨)';
  else if(livePlanner === 'stock')  $('carrotState').textContent = carrotActive
      ? '· 켜져 있지만 아직 기존 플래너가 돌고 있습니다 — 재시작 필요'
      : '· 꺼져 있음 (기존 플래너)';
  else                              $('carrotState').textContent = carrotActive
      ? '· 켜짐 (주행 시작하면 확인됩니다)'
      : '· 꺼져 있음';
}

function renderInto(box, table){
  box.innerHTML='';
  for(const[k,c]of Object.entries(table)){
    const row=document.createElement('div');row.className='row';
    const left=document.createElement('div');
    left.innerHTML=`<div class="lab">${c.label}</div><div class="hlp">${c.help}</div>`;
    row.appendChild(left);
    const sel=document.createElement('select');
    sel.setAttribute('aria-label',c.label);
    const cur=(k in staged)?staged[k]:c.value;
    c.options.forEach(o=>{
      const op=document.createElement('option');
      op.value=o.v;op.textContent=o.label;op.selected=(o.v===cur);
      sel.appendChild(op);
    });
    if(k in staged && staged[k]!==c.value) sel.classList.add('dirty');
    sel.onchange=()=>{
      const v=parseInt(sel.value,10);
      if(v===c.value) delete staged[k]; else staged[k]=v;
      renderSettings();updateApply();
    };
    row.appendChild(sel);
    box.appendChild(row);
  }
}

function updateApply(){
  const n=Object.keys(staged).length;
  $('apply').disabled=!n;
  $('dirty').textContent=n?`${n}개 변경 대기`:
    (engaged?'제어 중 · 변경 시 다음 engage부터 적용':'');
}

async function loadSettings(){
  const d=await(await fetch('/api/settings')).json();
  cfg=d.settings; mapCfg=d.map||{}; carrotCfg=d.carrot||{}; carrotActive=!!d.carrotActive; engaged=d.engaged;
  renderSettings();updateApply();
}

$('apply').onclick=async()=>{
  const body=JSON.stringify({changes:staged});
  const r=await fetch('/api/settings',{method:'POST',
    headers:{'Content-Type':'application/json'},body});
  const d=await r.json();
  if(!r.ok){toast(d.error||'저장에 실패했습니다',1);return;}
  staged={};
  await loadSettings();
  toast(d.engaged?'저장됨 · 다음 engage부터 적용됩니다':'저장됨 · 약 0.5초 내 반영');
};

loadSettings();loadModels().then(scheduleModels);poll();setInterval(poll,300);
</script></body></html>"""


PAGE_INDEX = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>openpilot</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--ok:#4CC38A;--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--ok:#1B7F53}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:22px;padding-bottom:calc(22px + env(safe-area-inset-bottom))}
h1{font-size:17px;margin:0 0 3px;letter-spacing:-.01em}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:20px}
a.card{display:block;text-decoration:none;color:inherit;background:var(--card);
border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:12px}
a.card:hover,a.card:focus-visible{border-color:var(--radar);outline:none}
.t{font-size:15px;margin-bottom:4px}
.d{font-size:12.5px;color:var(--mut);line-height:1.5}
.st{display:flex;gap:8px;margin-top:11px;flex-wrap:wrap}
.pill{font-family:var(--m);font-size:10px;padding:3px 8px;border-radius:99px;
border:1px solid var(--line);color:var(--mut)}
.pill.on{border-color:var(--ok);color:var(--ok)}
</style></head><body>
<h1>openpilot</h1><div class="sub" id="sub">연결 중…</div>

<a class="card" href="/live">
  <div class="t">Live · 앞차 인식과 설정</div>
  <div class="d">R/V 인식 출처, 앞차 거리·속도, gap, 제동 요구를 실시간으로 보고
    정지차 매칭 보정 같은 옵션을 바꿉니다.</div>
  <div class="st"><span class="pill" id="p-eng">–</span><span class="pill" id="p-lead">–</span></div>
</a>

<a class="card" href="/events">
  <div class="t">개입 · 운전자가 끈 순간</div>
  <div class="d">브레이크나 강한 조향으로 시스템을 끈 시점을 전후 구간과 함께 기록합니다.
    그때 openpilot이 원했던 값이 같이 남으므로, 순정 ACC와 어느 쪽이 맞았는지 비교할 수 있습니다.</div>
  <div class="st"><span class="pill" id="p-evt">–</span></div>
</a>

<a class="card" href="/vehicle">
  <div class="t">차량 · 상태 한눈에</div>
  <div class="d">기어·속도·문·안전벨트·서스펜션 차고처럼 지금 차가 어떤 상태인지를
    골라서 보여줍니다. 나머지 신호는 "그 외" 탭에 있습니다.</div>
  <div class="st"><span class="pill" id="p-veh">–</span></div>
</a>

<a class="card" href="/can">
  <div class="t">CAN · 전체 신호 뷰어</div>
  <div class="d">차량이 보내는 모든 CAN 메시지를 DBC로 디코딩해서 보여줍니다.
    값이 바뀐 신호를 표시하므로, 어떤 조작이 어떤 신호를 움직이는지 찾을 때 씁니다.</div>
  <div class="st"><span class="pill" id="p-can">–</span></div>
</a>

<a class="card" href="/shadow">
  <div class="t">그림자 · 순정 vs 지금 코드</div>
  <div class="d">녹화된 주행을 지금 소스의 플래너로 다시 풀어, 순정 ACC가 실제로 한 것과
    나란히 보여줍니다. 상수를 고치고 다시 돌리면 새 선만 움직이므로, 바꾼 값이
    실제 상황에서 무엇을 바꾸는지 바로 확인할 수 있습니다.</div>
  <div class="st"><span class="pill" id="p-shd">–</span></div>
</a>

<a class="card" href="/videos">
  <div class="t">영상 · 녹화된 주행</div>
  <div class="d">디바이스에 저장된 주행 영상을 목록에서 골라 바로 재생합니다.
    세그먼트가 끝나면 다음으로 이어지고, 원본 카메라 파일은 내려받을 수 있습니다.</div>
  <div class="st"><span class="pill" id="p-vid">–</span></div>
</a>

<script>
async function tick(){
  try{
    const s=await(await fetch('/api/state')).json();
    const stateTxt=s.connected?(s.onroad?'주행 중 · onroad':'정차 · offroad'):'openpilot 대기 중';
    // Which planner plannerd actually published, straight from longitudinalPlan. Shown up top
    // because it is the one thing about a drive you cannot recover afterwards by guessing.
    if(s.planner && s.planner !== livePlanner){ livePlanner=s.planner; renderSettings(); }
    const plannerTxt = s.planner === 'carrot' ? ' · 종방향 CarrotPilot'
                     : s.planner === 'stock'  ? ' · 종방향 기존' : '';
    document.getElementById('sub').textContent=
      (s.commit?`${stateTxt} · commit ${s.commit}`:stateTxt) + plannerTxt;
    const e=document.getElementById('p-eng');
    e.textContent=s.engaged?'engaged':'disengaged';e.className='pill'+(s.engaged?' on':'');
    const L=s.lead||{},l=document.getElementById('p-lead');
    l.textContent=L.status?`앞차 ${L.source} · ${L.dRel}m`:'앞차 없음';
    l.className='pill'+(L.status?' on':'');
  }catch(e){document.getElementById('sub').textContent='디바이스에 연결할 수 없습니다';}
  try{
    const c=await(await fetch('/api/can')).json();
    const p=document.getElementById('p-can');
    p.textContent=c.dbc?`${c.total} msg · ${c.dbc}`:(c.error||'DBC 없음');
    p.className='pill'+(c.total?' on':'');
  }catch(e){}
  try{
    const v=await(await fetch('/api/vehicle')).json();
    const p=document.getElementById('p-veh');
    const n=v.error?0:(v.total-v.missing);
    p.textContent=v.error?'차량 미연결':`${n}/${v.total} 신호 수신`;
    p.className='pill'+(n?' on':'');
  }catch(e){}
}
async function once(){
  try{
    const d=await(await fetch('/api/videos')).json();
    const n=(d.routes||[]).length,s=(d.routes||[]).reduce((a,r)=>a+r.count,0);
    const p=document.getElementById('p-vid');
    p.textContent=n?`주행 ${n}개 · 세그먼트 ${s}개`:'녹화 없음';
    p.className='pill'+(n?' on':'');
  }catch(e){}
}
once();tick();setInterval(tick,1000);
</script></body></html>"""


PAGE_VEHICLE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>차량 상태</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--hot:#F5B942;--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--hot:#9A6210}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:18px;padding-bottom:calc(18px + env(safe-area-inset-bottom))}
a.back{color:var(--dim);font-family:var(--m);font-size:11px;text-decoration:none}
h1{font-size:17px;margin:8px 0 3px}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:16px}
.tabs{display:flex;gap:8px;margin-bottom:14px}
.tab{flex:1;font-size:13px;padding:9px;border-radius:9px;border:1px solid var(--line);
background:var(--card);color:var(--mut);cursor:pointer}
.tab[aria-selected=true]{border-color:var(--radar);color:var(--radar)}
.grp{background:var(--card);border:1px solid var(--line);border-radius:11px;
margin-bottom:11px;overflow:hidden}
.grp.warn{border-color:var(--hot)}
.gt{font-size:12px;color:var(--mut);padding:10px 13px;border-bottom:1px solid var(--line)}
.grp.warn .gt{color:var(--hot)}
.row{display:flex;align-items:baseline;gap:10px;padding:8px 13px;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.lb{font-size:13px;flex:1;min-width:0}
.vl{font-family:var(--m);font-size:13px;text-align:right;white-space:nowrap}
.vl.w{color:var(--hot)}
.vl.na{color:var(--dim)}
.sg{font-family:var(--m);font-size:9.5px;color:var(--dim);white-space:nowrap}
.hint{font-size:12px;color:var(--mut);line-height:1.6;background:var(--card);
border:1px solid var(--line);border-radius:11px;padding:13px;margin-bottom:11px}
</style></head><body>
<a class="back" href="/">← 홈</a>
<h1>차량 상태</h1><div class="sub" id="sub">연결 중…</div>
<div class="tabs">
  <button class="tab" id="t-core" aria-selected="true">중요</button>
  <button class="tab" id="t-more" aria-selected="false">그 외</button>
</div>
<div id="out"></div>
<script>
const $=i=>document.getElementById(i);
let sec='core',last=null,showSig=false;
for(const id of ['core','more']) $('t-'+id).onclick=()=>{
  sec=id;for(const o of ['core','more'])$('t-'+o).setAttribute('aria-selected',o===id);render(last);};

function render(d){
  if(!d) return;
  const out=$('out');
  if(d.error){out.innerHTML='<div class="hint">'+d.error+'</div>';return;}
  const s=(d.sections||[]).find(x=>x.id===sec);
  if(!s){out.innerHTML='';return;}
  let h='';
  for(const g of s.groups){
    h+='<div class="grp'+(g.warn?' warn':'')+'"><div class="gt">'+g.title+'</div>';
    for(const r of g.rows){
      const na=r.value===null;
      h+='<div class="row"><span class="lb">'+r.label+'</span>'
        +(showSig?'<span class="sg">'+r.addr+' '+r.signal+'</span>':'')
        +'<span class="vl'+(r.warn?' w':'')+(na?' na':'')+'">'+(na?'–':r.value)+'</span></div>';
    }
    h+='</div>';
  }
  out.innerHTML=h;
}
$('sub').onclick=()=>{showSig=!showSig;render(last);};

async function tick(){
  try{
    const d=await(await fetch('/api/vehicle')).json();last=d;
    const mode=d.mode==='replay'?('재생 · '+(d.route||'')):'실시간';
    $('sub').textContent=d.error?mode:(mode+' · '+(d.total-d.missing)+'/'+d.total+' 신호 수신 · 탭하면 주소 표시');
    render(d);
  }catch(e){$('sub').textContent='디바이스에 연결할 수 없습니다';}
}
tick();setInterval(tick,500);
</script></body></html>"""


PAGE_CAN = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>CAN viewer</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--hot:#F5B942;--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--hot:#9A6210}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:14px;padding-bottom:calc(14px + env(safe-area-inset-bottom))}
.back{display:inline-block;font-family:var(--m);font-size:11px;color:var(--dim);
text-decoration:none;margin-bottom:8px}.back:hover{color:var(--radar)}
h1{font-size:15px;margin:0 0 3px}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:12px}
.bar{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap;position:sticky;top:0;
background:var(--bg);padding:4px 0;z-index:5}
.srcbar{display:flex;gap:8px;margin-bottom:8px;flex-wrap:wrap}
.srcbar select{flex:1;min-width:170px;background:var(--card);color:var(--tx);
border:1px solid var(--line);border-radius:9px;padding:9px 10px;font-size:13px;font-family:inherit}
.srcbar select:focus-visible{outline:2px solid var(--radar);outline-offset:1px}
button.tg.on{border-color:var(--radar);color:var(--radar)}
input[type=search]{flex:1;min-width:150px;background:var(--card);color:var(--tx);
border:1px solid var(--line);border-radius:9px;padding:9px 11px;font-size:14px}
input:focus-visible,button:focus-visible{outline:2px solid var(--radar);outline-offset:1px}
button.tg{background:var(--card);color:var(--mut);border:1px solid var(--line);border-radius:9px;
padding:9px 12px;font-family:var(--m);font-size:11.5px;cursor:pointer}
button.tg[aria-pressed=true]{border-color:var(--hot);color:var(--hot)}
.msg{background:var(--card);border:1px solid var(--line);border-radius:10px;margin-bottom:8px;
overflow:hidden}
.msg.hot{border-color:var(--hot)}
.msg.unseen{opacity:.5}
.msg.unseen .addr{color:var(--dim)}
.mh{display:flex;align-items:baseline;gap:9px;padding:10px 12px;cursor:pointer;flex-wrap:wrap}
.addr{font-family:var(--m);font-size:13px;color:var(--radar);font-weight:600}
.nm{font-size:13px;flex:1;min-width:100px}
.nm.unk{color:var(--dim);font-style:italic}
.hz{font-family:var(--m);font-size:10.5px;color:var(--dim)}
.bytes{font-family:var(--m);font-size:11px;color:var(--mut);word-break:break-all;
padding:0 12px 10px}
.sigs{border-top:1px solid var(--line);padding:4px 12px 10px}
.sig{display:flex;justify-content:space-between;gap:12px;padding:5px 0;font-family:var(--m);
font-size:12px;border-bottom:1px solid var(--line)}
.sig:last-child{border-bottom:0}
.sig .n{color:var(--mut);word-break:break-all}
.sig .val{font-variant-numeric:tabular-nums;text-align:right;white-space:nowrap}
.sig.ch .n,.sig.ch .val{color:var(--hot)}
.sig .en{color:var(--dim);font-size:10.5px;margin-left:6px}
.sig.noise{opacity:.4}
.empty{color:var(--dim);font-size:13px;padding:24px 4px;text-align:center}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<a class="back" href="/">← 메뉴</a>
<h1>CAN 신호 뷰어</h1><div class="sub" id="sub">연결 중…</div>
<div class="srcbar">
  <select id="route" aria-label="신호 소스"><option value="">라이브 (차량 연결)</option></select>
  <button class="tg" id="play">재생</button>
</div>
<div class="bar">
  <input type="search" id="q" placeholder="메시지·신호 이름 또는 주소" aria-label="검색">
  <button class="tg" id="only" aria-pressed="false">변한 것만</button>
  <button class="tg" id="unseen" aria-pressed="true">미수신</button>
  <button class="tg" id="unknown" aria-pressed="true">DBC 외</button>
  <button class="tg" id="radar" aria-pressed="false">레이더 포인트</button>
  <button class="tg" id="pause" aria-pressed="false">일시정지</button>
</div>
<div id="list"></div>

<script>
const $=i=>document.getElementById(i);
const open=new Set(); let paused=false, onlyChanged=false, showUnseen=true, showUnknown=true, showRadar=false;

$('only').onclick=()=>{onlyChanged=!onlyChanged;$('only').setAttribute('aria-pressed',onlyChanged);};
$('unseen').onclick=()=>{showUnseen=!showUnseen;$('unseen').setAttribute('aria-pressed',showUnseen);render(last);};
$('unknown').onclick=()=>{showUnknown=!showUnknown;$('unknown').setAttribute('aria-pressed',showUnknown);render(last);};
$('radar').onclick=()=>{showRadar=!showRadar;$('radar').setAttribute('aria-pressed',showRadar);render(last);};
$('pause').onclick=()=>{paused=!paused;$('pause').setAttribute('aria-pressed',paused);};
$('q').oninput=()=>render(last);

async function loadRoutes(){
  const d=await(await fetch('/api/routes')).json();
  const sel=$('route');
  sel.innerHTML='<option value="">라이브 (차량 연결)</option>'+
    (d.routes||[]).map(r=>`<option value="${r.name}">${r.name} · ${r.segments}개</option>`).join('');
  if(d.mode==='replay'&&d.route) sel.value=d.route;
  setPlay(d.mode==='replay');
}
function setPlay(on){
  const b=$('play');
  b.textContent=on?'정지':'재생';
  b.classList.toggle('on',on);
}
$('play').onclick=async()=>{
  const route=$('route').value;
  const playing=$('play').classList.contains('on');
  const body=JSON.stringify({route:playing?null:route});
  const r=await fetch('/api/replay',{method:'POST',
    headers:{'Content-Type':'application/json'},body});
  const d=await r.json();
  if(!r.ok){$('sub').textContent=d.error||'재생을 시작할 수 없습니다';return;}
  setPlay(d.mode==='replay');
};

let last={messages:[]};
function key(m){return (m.bus===null?'x':m.bus)+':'+m.address;}

function render(d){
  last=d;
  const q=$('q').value.trim().toLowerCase();
  const list=$('list');
  let msgs=d.messages||[];
  if(!showUnseen) msgs=msgs.filter(m=>m.seen);
  if(!showUnknown) msgs=msgs.filter(m=>m.name);
  if(!showRadar) msgs=msgs.filter(m=>!/RadarPoint/i.test(m.name||''));
  if(q) msgs=msgs.filter(m=>
    (m.name||'').toLowerCase().includes(q) ||
    String(m.address).includes(q) || m.address.toString(16).includes(q) ||
    m.signals.some(s=>s.name.toLowerCase().includes(q)));
  if(!msgs.length){list.innerHTML='<div class="empty">'+
    (d.error||(q?'검색 결과가 없습니다':'수신된 CAN 메시지가 없습니다'))+'</div>';return;}

  list.innerHTML=msgs.map(m=>{
    const k=key(m), isOpen=open.has(k);
    const sigs=isOpen&&m.signals.length?'<div class="sigs">'+m.signals.map(s=>
      `<div class="sig${s.changed?' ch':''}${s.noise?' noise':''}"><span class="n">${s.name}</span>`+
      `<span class="val">${m.seen?s.v:'N/A'}${s.enum?`<span class="en">${s.enum}</span>`:''}</span></div>`
    ).join('')+'</div>':'';
    const meta=m.seen?`bus ${m.bus} · ${m.hz}Hz`:'미수신';
    const bytes=m.seen?m.hex.replace(/(..)/g,'$1 ').trim():'N/A';
    return `<div class="msg${m.anyChanged?' hot':''}${m.seen?'':' unseen'}" data-k="${k}">
      <div class="mh"><span class="addr">0x${m.address.toString(16).toUpperCase()}</span>
        <span class="nm${m.name?'':' unk'}">${m.name||'(DBC에 없음)'}</span>
        <span class="hz">${meta}</span></div>
      <div class="bytes">${bytes}</div>${sigs}</div>`;
  }).join('');

  list.querySelectorAll('.msg').forEach(el=>{
    el.querySelector('.mh').onclick=()=>{
      const k=el.dataset.k; open.has(k)?open.delete(k):open.add(k); render(last);};
  });
}

async function tick(){
  if(paused) return;
  try{
    const d=await(await fetch('/api/can'+(onlyChanged?'?changed=1':''))).json();
    const src=d.mode==='replay'?`재생: ${d.route} · ${d.status}`:'라이브';
    $('sub').textContent=d.dbc?`수신 ${d.seen}/${d.known} · 전체 ${d.total} · ${d.dbc} · ${src}`
                              :(d.error||'DBC를 찾을 수 없습니다');
    render(d);
  }catch(e){$('sub').textContent='디바이스에 연결할 수 없습니다';}
}
loadRoutes();tick();setInterval(tick,400);
</script></body></html>"""



PAGE_SHADOW = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>그림자 · 순정 vs 지금 코드</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--hot:#F5B942;--bad:#FF6B5A;--ok:#4FC98A;
--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--hot:#9A6210;--bad:#C23B28;--ok:#1E7A4B}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);padding:18px;
padding-bottom:calc(18px + env(safe-area-inset-bottom))}
a.back{color:var(--dim);font-family:var(--m);font-size:11px;text-decoration:none}
h1{font-size:17px;margin:8px 0 3px}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:11px;
margin-bottom:11px;overflow:hidden}
.card>h2{font-size:12px;margin:0;padding:10px 13px;border-bottom:1px solid var(--line);
color:var(--mut);font-weight:600}
.pad{padding:13px;display:flex;gap:8px;flex-wrap:wrap;align-items:center}
/* 영상은 그대로 두고 그래프는 아래에 둔다. 위에 겹치면 재생이 시작될 때 브라우저가 video 를
   별도 합성 레이어로 올리면서 캔버스를 덮어버린다. */
.vid{margin:13px 13px 0;background:#000;border-radius:9px;overflow:hidden}
.vid video{width:100%;max-height:52vh;display:block}
.novid{aspect-ratio:16/9;display:flex;align-items:center;justify-content:center;
color:var(--dim);font-size:12px}
select,input{background:var(--bg);border:1px solid var(--line);color:var(--tx);border-radius:8px;
padding:7px 10px;font-size:12px;font-family:var(--m)}
button{background:transparent;border:1px solid var(--line);color:var(--tx);border-radius:8px;
padding:7px 14px;font-size:12px;cursor:pointer;font-family:var(--s)}
button:hover:not([disabled]){border-color:var(--radar);color:var(--radar)}
button[disabled]{opacity:.45;cursor:default}
.lbl{font-size:11px;color:var(--dim)}
canvas{width:100%;height:250px;display:block}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:11px;color:var(--mut);padding:0 13px 12px}
.legend i{display:inline-block;width:14px;height:3px;border-radius:2px;margin-right:5px;
vertical-align:middle}
.read{display:grid;grid-template-columns:repeat(auto-fit,minmax(104px,1fr));gap:1px;
background:var(--line);border-top:1px solid var(--line)}
.rd{background:var(--card);padding:9px 11px}
.rd .k{font-size:10px;color:var(--dim);margin-bottom:2px}
.rd .v{font-family:var(--m);font-size:14px;font-variant-numeric:tabular-nums}
.rd .v small{font-size:10px;color:var(--mut);margin-left:2px}
.msg{padding:0 13px 13px;font-size:12px;color:var(--mut)}
.msg.err{color:var(--bad);font-family:var(--m);font-size:11.5px}

/* ADAS/지도 요약 -- 영상 위에 얹는 1~2줄짜리 압축 리드아웃 */
.adasStrip{font-family:var(--m);font-size:11.5px;color:var(--tx);background:var(--bg);
border-top:1px solid var(--line);padding:6px 13px;line-height:1.7}
.adasStrip .sep{color:var(--line);margin:0 8px}
.adasStrip .k{color:var(--dim)}
.adasStrip .v.na{color:var(--dim)}
.adasStrip .v.on{color:var(--radar)}
.note{font-size:11.5px;color:var(--mut);line-height:1.6;padding:13px}
.note b{color:var(--tx);font-weight:600}

/* 세그먼트 스캔 -- 로컬 shadow_viz.py 의 세그먼트 스트립과 같은 모양 */
.segs{display:flex;gap:4px;padding:11px 13px;overflow-x:auto}
.seg{flex:0 0 auto;width:52px;background:transparent;border:1px solid var(--line);
border-radius:8px;padding:6px 4px 5px;cursor:pointer;color:var(--mut);font-family:var(--m);
font-size:10px;text-align:center}
.seg:hover{border-color:var(--radar);color:var(--tx)}
.seg[aria-selected=true]{border-color:var(--radar);color:var(--radar);background:rgba(90,200,250,.08)}
.seg .n{font-size:12px;font-weight:600;display:block;line-height:1.3}
.seg .bar{height:3px;border-radius:2px;background:var(--line);margin:4px 0 3px;overflow:hidden}
.seg .bar>i{display:block;height:100%;background:var(--bad)}
.seg .pc{font-variant-numeric:tabular-nums}
.scanbar{height:3px;background:var(--line);border-radius:2px;overflow:hidden;margin:0 13px 11px}
.scanbar>i{display:block;height:100%;background:var(--radar);transition:width .2s}
</style></head><body>
<a class="back" href="/">&larr; 돌아가기</a>
<h1>그림자 · 순정 ACC vs 지금 코드</h1>
<div class="sub" id="sub">불러오는 중…</div>

<div class="card">
  <h2>세그먼트 스캔 — 순정과 지금 계획이 크게 갈린 순서로</h2>
  <div class="pad">
    <span class="lbl">스캔할 주행</span><select id="scanRoute"></select>
    <button id="scanRun">스캔</button>
    <span class="lbl" id="scanMsg" style="margin-left:auto"></span>
  </div>
  <div class="scanbar" id="scanBar" style="display:none"><i style="width:0"></i></div>
  <div class="segs" id="segs"></div>
</div>

<div class="card">
  <h2>재생할 구간</h2>
  <div class="pad">
    <span class="lbl">주행</span><select id="route"></select>
    <span class="lbl">세그먼트</span><select id="seg"></select>
    <button id="run">풀기</button>
  </div>
  <div class="msg" id="msg">주행 중에는 실행하지 않습니다. MPC 를 매 프레임 다시 풀기 때문에 60초 구간에 수 초 걸립니다.</div>
</div>

<div class="card">
  <h2 id="chartTitle">결과</h2>
  <div class="vid" id="vidwrap"></div>
  <div class="adasStrip" id="adasStrip1">–</div>
  <div class="adasStrip" id="adasStrip2">–</div>
  <canvas id="ch"></canvas>
  <div class="legend">
    <span><i style="background:#F5B942"></i>순정 ACC 실제 (aEgo)</span>
    <span><i style="background:#8A97A6"></i>주행 당시 계획 (기록된 aTarget)</span>
    <span><i style="background:#FF9F4A"></i>순정 ACC 커맨드 밴드 (bus2 직접 수신)</span>
    <span><i style="background:#4ED88A"></i>CarrotPilot 재계산</span>
    <span><span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:#4ED88A;margin-right:5px;vertical-align:1px"></span>CarrotPilot 이 정지를 결정한 순간 (e2eStop)</span>
  </div>
  <div class="pad" style="padding-top:0">
    <button id="play">▶ 재생</button>
    <button id="worst" title="CarrotPilot과 순정이 가장 크게 갈린 순간">최대 격차로</button>
    <button id="worstStop" disabled title="CarrotPilot이 정지를 결정한 첫 순간 (xState=e2eStop)">모델 정지 순간으로</button>
    <span class="lbl" id="tt" style="margin-left:auto;font-family:var(--m)">—</span>
  </div>
  <div class="read" id="read"></div>
</div>

<div class="card">
  <h2>읽는 법</h2>
  <div class="note">
    노란 선과 회색 선은 <b>로그에 있는 그대로</b>라 코드를 고쳐도 움직이지 않습니다.
    초록 선(CarrotPilot)만 지금 소스로 다시 푼 결과이므로, 상수를 바꾸고 다시 돌렸을 때
    <b>초록 선이 어떻게 달라지는지</b>가 그 변경의 효과입니다.
    <br><br>
    상태는 매 프레임 로그값으로 다시 심습니다. 그대로 굴리면 1~2초 만에 실제 상황에서
    멀어져 같은 순간을 비교하는 의미가 사라지기 때문입니다.
  </div>
</div>

<script>
const MPH=2.2369363;
let DATA=null, poll=null, vt=0, VOFF=0, worstT=null, worstStopT=null;
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();

// column 15 of a row is {addr: {sigName: value}}, decoded straight off raw CAN in shadow_replay's
// _AdasDecoder; 16/17 are cruiseState.speed(m/s)/enabled straight from carState. Curated down to
// what's actually useful for the planner -- see tools/tesla_analysis/adas_signal_survey.md for why
// most of the ~110 other decoded signals didn't make this cut (0x3E8 in particular is almost all
// static config, not per-moment driving data).
const F_BRAKE=8, F_GAS=16;   // must match shadow_replay.py's F_BRAKE/F_GAS

// DAS_fusedSpeedLimit/UI_mppSpeedLimit: factor 5 already applied server-side, so 31*5=155 means
// "NONE" and 0 means unknown/SNA -- everything else is the value itself.
function fmtSpeedLimit(v, unitFlag){
  if(v==null) return '–';
  if(v<=0) return '?';
  if(v>=155) return '없음';
  return `${v}${unitFlag===1?'km/h':'mph'}`;
}
// UI_mapSpeedLimit is a discrete band code (factor=1 in the DBC, not the *5 continuous encoding
// the other two use) -- 31=SNA, 30=무제한, 29..1 step down in 5-unit-ish bands, 0=UNKNOWN.
const MAP_SPEED_BAND=[null,5,7,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,130,140,150,160,'무제한','SNA'];
function fmtMapSpeedLimit(v, unitFlag){
  if(v==null) return '–';
  const b=MAP_SPEED_BAND[v];
  if(b==null) return '?';
  return typeof b==='number' ? `${b}${unitFlag===1?'km/h':'mph'}` : b;
}

function updateAdas(r){
  const adas=(r&&r[15])||{};
  const das399=adas[0x399]||{}, gps2f8=adas[0x2F8]||{}, map3c8=adas[0x3C8]||{};
  const unitFlag=gps2f8.UI_mapSpeedLimitUnits;

  $('adasStrip1').innerHTML =
    `<span class="k">융합제한</span> ${fmtSpeedLimit(das399.DAS_fusedSpeedLimit, unitFlag)}` +
    `<span class="sep">·</span><span class="k">MPP제한</span> ${fmtSpeedLimit(gps2f8.UI_mppSpeedLimit, unitFlag)}` +
    `<span class="sep">·</span><span class="k">지도제한</span> ${fmtMapSpeedLimit(map3c8.UI_mapSpeedLimit, map3c8.UI_mapSpeedUnits)}` +
    `<span class="sep">·</span><span class="k">GPS매칭</span> ${map3c8.UI_gpsRoadMatch===undefined?'–':(map3c8.UI_gpsRoadMatch?'예':'아니오')}` +
    `<span class="sep">·</span><span class="k">지도카운터</span> ${map3c8.UI_mapDataCounter??'–'}`;

  const cruiseOn = !!(r && r[17]), vCruise = r ? r[16] : null;
  const gasOn = !!(r && (r[7]&F_GAS)), brakeOn = !!(r && (r[7]&F_BRAKE));
  $('adasStrip2').innerHTML =
    `<span class="k">크루즈</span> <span class="v ${cruiseOn?'on':''}">${cruiseOn?'ON':'OFF'}</span>` +
    `<span class="sep">·</span><span class="k">크루즈속도</span> ${vCruise!=null?(vCruise*MPH).toFixed(0)+'mph':'–'}` +
    `<span class="sep">·</span><span class="k">가스</span> <span class="v ${gasOn?'on':''}">${gasOn?'ON':'OFF'}</span>` +
    `<span class="sep">·</span><span class="k">브레이크</span> <span class="v ${brakeOn?'on':''}">${brakeOn?'ON':'OFF'}</span>`;
}

async function boot(){
  const d=await (await fetch('/api/shadow')).json();
  $('sub').textContent = `커밋 ${d.commit||''} · 녹화 주행 ${(d.routes||[]).length}개`;
  $('route').innerHTML = (d.routes||[]).map(r=>`<option>${r}</option>`).join('');
  $('scanRoute').innerHTML = $('route').innerHTML;
  if((d.routes||[]).length) await loadSegs();
  if(d.status==='done') show(d);
  resumeScan();
}

// 세그먼트 스캔 -- 로그에 이미 있는 aTarget/aEgo 로 훑는 저비용 1차 패스. 재계산이 필요한
// 세그먼트를 고르기 위한 것이라 MPC 를 다시 풀지 않는다; 그 비용은 고른 세그먼트 하나에만 든다.
let scanPoll=null;
$('scanRun').onclick=async()=>{
  const route=$('scanRoute').value;
  $('scanRun').disabled=true; $('scanMsg').textContent='스캔 중…';
  const res=await fetch('/api/shadow/scan',{method:'POST',body:JSON.stringify({route})});
  const d=await res.json();
  if(d.error){ $('scanMsg').textContent=d.error; $('scanRun').disabled=false; return; }
  pollScan(route);
};
function resumeScan(){
  // 재접속했을 때 이미 돌고 있거나 끝나 있는 스캔을 이어받는다
  fetch('/api/shadow/scan?route='+encodeURIComponent($('scanRoute').value)).then(r=>r.json()).then(d=>{
    if(d.status==='running'){ $('scanRun').disabled=true; pollScan(d.route); }
    else if(d.status==='done'){ renderSegs(d.segments); }
  });
}
function pollScan(route){
  if(scanPoll) return;
  scanPoll=setInterval(async()=>{
    const d=await (await fetch('/api/shadow/scan?route='+encodeURIComponent(route))).json();
    if(d.status==='running'){
      $('scanBar').style.display='block';
      $('scanBar').firstElementChild.style.width=`${d.total?100*d.done/d.total:0}%`;
      $('scanMsg').textContent=`${d.done}/${d.total} 세그먼트`;
      return;
    }
    clearInterval(scanPoll); scanPoll=null; $('scanRun').disabled=false; $('scanBar').style.display='none';
    if(d.status==='error'){ $('scanMsg').textContent=d.error; return; }
    if(d.status==='done'){ $('scanMsg').textContent=`${d.segments.length}개 세그먼트`; renderSegs(d.segments); }
  }, 800);
}
function renderSegs(segments){
  $('segs').innerHTML = segments.map(s=>`
    <button class="seg" data-seg="${s.seg}" aria-selected="false"
      title="세그먼트 ${s.seg} · 리드 ${s.leadFrames}프레임 · 최대 격차 ${(s.worst*MPH).toFixed(1)} mph/s">
      <span class="n">${s.seg}</span>
      <span class="bar"><i style="width:${Math.min(100,s.disagreePct)}%"></i></span>
      <span class="pc">${s.disagreePct}%</span></button>`).join('');
  $('segs').onclick=e=>{
    const b=e.target.closest('.seg'); if(!b) return;
    document.querySelectorAll('#segs .seg').forEach(x=>x.setAttribute('aria-selected', x===b));
    $('route').value=$('scanRoute').value;
    loadSegs().then(()=>{ $('seg').value=b.dataset.seg; pickSeg(); });
  };
}
async function loadSegs(){
  const r=$('route').value;
  const d=await (await fetch('/api/shadow?route='+encodeURIComponent(r))).json();
  $('seg').innerHTML=(d.segments||[]).map(s=>`<option>${s}</option>`).join('');
  pickSeg();
}
function pickSeg(){
  if($('seg').value==='') return;
  // Video only. Solving a segment costs about 13s on this device -- 3.4 decompressing and
  // parsing the log, 8.8 replaying it through carrot's planner, which reads the log a second
  // time. Paying that on every dropdown change meant the page was unusable for picking a moment
  // to look at. It runs when asked now.
  mountVideo($('route').value, +$('seg').value);
  DATA=null; SOLVED='';
  $('run').disabled=false;
  $('run').textContent='풀기';
  $('chartTitle').textContent='기록값 읽는 중…';
  draw();
  load(false);                   // recorded lines only -- the log read, not the re-solve
}
$('route').onchange=loadSegs;
$('seg').onchange=pickSeg;

let SOLVED='';                   // 이미 푼(또는 푸는 중인) 구간+하한
let WANT_SOLVE=false;            // 이번 요청이 재계산까지 하는지
// load(false) reads the log and draws what the drive recorded -- a few seconds. load(true) adds
// the re-solve on top, which is the part that costs.
async function load(doSolve){
  const route=$('route').value, seg=$('seg').value;
  if(!route || seg==='') return;
  const key=`${route}/${seg}/${doSolve?1:0}`;
  if(key===SOLVED) return;
  if(poll) return;               // 앞선 실행이 끝나기 전에는 겹쳐 쏘지 않는다
  SOLVED=key; WANT_SOLVE=doSolve;
  DATA=null;
  $('run').disabled=true; $('msg').className='msg';
  $('msg').textContent = doSolve ? 'CarrotPilot 으로 푸는 중… (약 13초)'
                                 : '기록값 읽는 중… (약 4초)';
  const res=await fetch('/api/shadow',{method:'POST',
    body:JSON.stringify({route, seg:+seg, solve:doSolve})});
  const d=await res.json();
  if(d.error){
    $('msg').className='msg err'; $('msg').textContent=d.error;
    $('run').disabled=false; SOLVED='';
    return;
  }
  poll=setInterval(check,700);
}
$('run').onclick=()=>{ SOLVED=''; load(true); };
async function check(){
  const d=await (await fetch('/api/shadow')).json();
  if(d.status==='running') return;
  if(d.status==='partial'){ show(d); return; }        // still solving -- keep polling
  clearInterval(poll); poll=null; $('run').disabled=false;
  if(d.status==='error'){ $('msg').className='msg err'; $('msg').textContent=d.error; SOLVED=''; return; }
  if(d.status==='done'){
    $('msg').className='msg';
    $('run').textContent = d.solved===false ? '풀기' : '다시 풀기';
    show(d);
  }
}

const video=()=>document.getElementById('v');

function show(d){
  // 같은 구간의 두 번째 그림(partial -> done)인지 확인한다. 맞으면 재생 위치를 그대로 두고
  // 선만 마저 그린다 -- 계산이 끝났다고 보던 위치가 처음으로 튀면 안 된다.
  const fresh = !DATA || DATA.route!==d.route || DATA.seg!==d.seg;
  DATA=d;
  if(fresh){ vt=0; VOFF=0; }

  if(d.status==='partial'){
    $('chartTitle').textContent =
      `기록값 — ${d.route} 세그 ${d.seg} · 하한 ${(+d.accelMin).toFixed(2)} m/s² (${d.accelMinSrc||''})`
      + (WANT_SOLVE ? ' · 다시 푸는 중…' : '');
    $('msg').textContent = `${d.rows.length}프레임 기록값 표시됨`
      + (WANT_SOLVE ? ' · 지금 코드와 CarrotPilot 으로 푸는 중… (약 13초)' : '');
  } else if(d.solved===false){
    // Log read only: the recorded lines are real, the recomputed ones were never asked for.
    $('chartTitle').textContent =
      `기록값 — ${d.route} 세그 ${d.seg} · 하한 ${(+d.accelMin).toFixed(2)} m/s² (${d.accelMinSrc||''})`;
    $('msg').textContent = `${d.rows.length}프레임 · 노란/회색선은 주행이 남긴 값입니다. `
      + `"풀기"를 누르면 지금 코드와 CarrotPilot 선이 더해집니다 (약 13초).`;
  } else {
    $('chartTitle').textContent =
      `결과 — ${d.route} 세그 ${d.seg} · 제동 하한 ${(+d.accelMin).toFixed(2)} m/s² (${d.accelMinSrc||''})`
      + ` · 기록 당시 ${d.recordedPlanner==='carrot'?'CarrotPilot':'기존 플래너'} · 푸는 데 ${d.solveSec}초`;
    const stopFrames=d.rows.filter(r=>r[14]===3||r[14]===5).length;
    $('msg').textContent = `${d.rows.length}프레임`
      + (stopFrames ? ` · CarrotPilot 정지 상태(e2eStop/e2eStopped) ${stopFrames}개 (${(stopFrames/20).toFixed(1)}초)` : '');
  }

  // CarrotPilot이 순정보다 가장 많이 더 감속을 원한 순간 -- 재계산 값이 아직 없는 프레임은
  // (null-1) 이 NaN 이 되어 비교에서 자연히 걸러진다.
  let best=0; worstT=null;
  for(const r of d.rows){ const g=r[12]-r[1]; if(g<best){best=g; worstT=r[0];} }

  // CarrotPilot이 정지를 결정한 첫 순간 -- xState가 e2eStop(3)으로 넘어간 프레임.
  worstStopT=null;
  for(const r of d.rows){ if(r[14]===3){ worstStopT=r[0]; break; } }
  $('worstStop').disabled = worstStopT==null;

  if(fresh) mountVideo(d.route, d.seg);
  draw();
}

// 영상은 세그먼트를 고르는 순간 붙는다. 다시 풀어야만 나오게 두면, 계산이 끝나기 전에는
// 그 구간에 무슨 일이 있었는지 볼 방법이 없다 -- 그리고 재시작 직후처럼 결과가 없는 상태에서는
// 영상 자체가 아예 나타나지 않는다.
// 고화질은 두 경로가 있다. HEVC 무변환 복사는 0.75초면 되지만 브라우저가 HEVC 를 디코딩할 수
// 있어야 하고, H.264 재인코딩은 어디서나 재생되는 대신 세그먼트당 40초쯤 걸린다. 재생 가능한
// 쪽을 브라우저에게 직접 물어서 고른다.
function bestQuality(){
  const v=document.createElement('video');
  return v.canPlayType('video/mp4; codecs="hvc1.1.6.L120.B0"') ? 'copy' : 'h264';
}
let MOUNTED='';
function mountVideo(route, seg){
  const codec=bestQuality();
  const key=`${route}/${seg}/${codec}`;
  if(key===MOUNTED) return;
  MOUNTED=key; VOFF=0; vt=0;
  const wrap=$('vidwrap');
  if(codec==='h264') $('msg').textContent='고화질 변환 중… 세그먼트당 40초쯤 걸리고, 한 번 만들면 캐시됩니다';
  wrap.innerHTML = `<video id="v" controls preload="auto" playsinline
      src="/v/${encodeURIComponent(route)}/${seg}.mp4?cam=road&q=${codec}"></video>`;
  const v=video();
  v.onloadedmetadata=()=>{
    // qcamera 는 세그먼트 시작에서 시작하고 리먹스가 타임스탬프를 0 으로 되돌린다.
    // 되돌리지 않은 옛 캐시가 섞여 있어도 맞도록 길이 차이로 보정한다.
    const dur = DATA && DATA.rows.length ? DATA.rows[DATA.rows.length-1][0] : 60;
    if(isFinite(v.duration) && v.duration > dur+1){ VOFF=v.duration-dur; try{v.currentTime=VOFF;}catch(e){} }
  };
  v.onerror=()=>{ wrap.innerHTML='<div class="novid">이 세그먼트에는 영상이 없습니다</div>'; MOUNTED=''; };
  v.onplay =()=>$('play').textContent='❚❚ 일시정지';
  v.onpause=()=>$('play').textContent='▶ 재생';
}

function seek(t){
  if(!DATA) return;
  const dur=DATA.rows[DATA.rows.length-1][0];
  vt=Math.max(0,Math.min(dur,t));
  const v=video(); if(v) try{ v.currentTime=vt+VOFF; }catch(e){}
  draw();
}

// 루프는 멈추지 않고 돈다. 재생 이벤트로 켜는 구조는 그 이벤트를 한 번 놓치면 화면이 영영
// 갱신되지 않고 원인도 드러나지 않는다.
function tick(){
  const v=video();
  if(v && !v.paused) vt=Math.max(0, v.currentTime-VOFF);
  draw();
  requestAnimationFrame(tick);
}

function draw(){
  const cv=$('ch'), w=cv.clientWidth, h=cv.clientHeight;
  if(!DATA){
    if(w>0&&h>0){ const g=cv.getContext('2d'); g.setTransform(1,0,0,1,0,0); g.clearRect(0,0,cv.width,cv.height); }
    const v=video();
    if(v) $('tt').textContent = `${Math.max(0,v.currentTime-VOFF).toFixed(2)}s · 아직 풀지 않았습니다`;
    return;
  }
  if(!(w>0&&h>0)) return;
  const dpr=devicePixelRatio||1;
  if(cv.width!==Math.round(w*dpr)){ cv.width=Math.round(w*dpr); cv.height=Math.round(h*dpr); }
  const g=cv.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,w,h);

  const rows=DATA.rows, dur=rows.length?rows[rows.length-1][0]:1;
  const S=Math.max(Math.abs(DATA.accelMin), 2.0)*1.08;
  const L=46,R=14,T=14,B=h-24, iw=w-L-R, ih=B-T;
  const x=t=>L+iw*(t/dur), y=a=>T+ih*(0.5-(a/S)/2);

  g.font='10px '+css('--m'); g.textAlign='right'; g.lineWidth=1;
  for(const a of [2.0,0,DATA.accelMin]){
    const yy=Math.round(y(a))+.5;
    g.strokeStyle = a===DATA.accelMin ? css('--bad') : css('--line');
    g.beginPath(); g.moveTo(L,yy); g.lineTo(w-R,yy); g.stroke();
    g.fillStyle=css('--dim'); g.fillText((a*MPH).toFixed(1), L-6, yy+3);
  }
  g.textAlign='left'; g.fillStyle=css('--dim'); g.fillText('mph/s', 4, T+4);

  // 순정 ACC가 그 순간 실제로 커맨드하던 밴드 (stockAccelMin..Max) -- bus2 에서 직접 읽은
  // 값이라 openpilot 이 종방향을 잡고 있어도 채워진다. null 이면 그 순간 순정이 침묵 중이라는
  // 뜻이라, 선을 잇지 않고 창을 끊는다.
  g.fillStyle='#FF9F4A'; g.globalAlpha=.12;
  for(let i=0;i<rows.length-1;i++){
    const a=rows[i], b=rows[i+1];
    if(a[10]==null || a[11]==null) continue;
    g.fillRect(x(a[0]), y(a[11]), Math.max(1,x(b[0])-x(a[0])), y(a[10])-y(a[11]));
  }
  g.globalAlpha=1;
  g.strokeStyle='#FF9F4A'; g.lineWidth=1.4; g.beginPath(); let pen=false;
  rows.forEach(r=>{
    if(r[10]==null){ pen=false; return; }
    const px=x(r[0]), py=y(r[10]);
    pen?g.lineTo(px,py):g.moveTo(px,py); pen=true;
  });
  g.stroke();

  // col 12 (CarrotPilot 재계산) 는 재계산이 끝나기 전에는 null -- 그 프레임에서 선을 끊는다.
  // 0 으로 읽으면 계산 중인 구간이 실선으로 이어져 이미 다 푼 것처럼 보인다.
  for(const [col,color,lw] of [[1,'#F5B942',1.8],[2,'#8A97A6',1.2],[12,'#4ED88A',1.8]]){
    g.strokeStyle=color; g.lineWidth=lw; g.beginPath(); let pen=false;
    rows.forEach(r=>{
      if(r[col]==null){ pen=false; return; }
      const px=x(r[0]),py=y(r[col]);
      pen?g.lineTo(px,py):g.moveTo(px,py); pen=true;
    });
    g.stroke();
  }

  // CarrotPilot이 정지를 결정한 프레임 (xState=e2eStop) 을 아래쪽에 점으로 찍는다.
  g.fillStyle='#4ED88A';
  for(const r of rows) if(r[14]===3){
    g.beginPath(); g.arc(x(r[0]), B+10, 3, 0, 7); g.fill();
  }

  g.textAlign='center'; g.fillStyle=css('--dim');
  const step=dur>45?10:5;
  for(let t=0;t<=dur;t+=step) g.fillText(String(t), x(t), B+16);

  // 현재 재생 위치: 지나온 구간을 덮고 경계에 손잡이를 세운다
  const px=Math.round(x(Math.min(vt,dur)))+.5;
  g.fillStyle=css('--bg'); g.globalAlpha=.5; g.fillRect(L,T,Math.max(0,px-L),ih); g.globalAlpha=1;
  g.strokeStyle=css('--radar'); g.lineWidth=2;
  g.beginPath(); g.moveTo(px,T-6); g.lineTo(px,B+6); g.stroke();
  g.fillStyle=css('--radar');
  g.beginPath(); g.moveTo(px-5,T-8); g.lineTo(px+5,T-8); g.lineTo(px,T-1); g.closePath(); g.fill();
  const r=rows[Math.min(rows.length-1,Math.round(vt*20))];
  if(r) for(const [col,color] of [[1,'#F5B942'],[12,'#4ED88A'],[10,'#FF9F4A']]){
    if(r[col]==null) continue;
    g.beginPath(); g.arc(px,y(r[col]),4,0,7); g.fillStyle=color; g.fill();
    g.strokeStyle=css('--card'); g.lineWidth=1.8; g.stroke();
  }
  readout(r);
  updateAdas(r);
  $('tt').textContent=`${vt.toFixed(2)}s / ${dur.toFixed(0)}s`;
}

function readout(r){
  if(!r) return;
  const solving = '<span style="opacity:.5">계산 중…</span>';
  $('read').innerHTML=[
    ['시각',`${r[0].toFixed(1)}<small>s</small>`],
    ['순정 실제',`${(r[1]*MPH).toFixed(1)}<small>mph/s</small>`],
    [`당시 계획${DATA&&DATA.recordedPlanner==='carrot'?' (Carrot)':''}`,
      `${(r[2]*MPH).toFixed(1)}<small>mph/s</small>`],
    ['순정 커맨드 밴드',r[10]==null?'침묵':
      `<span style="color:#FF9F4A">${(r[10]*MPH).toFixed(1)} … ${(r[11]*MPH).toFixed(1)}<small>mph/s</small></span>`],
    ['속도',`${(r[5]*MPH).toFixed(0)}<small>mph</small>`],
    ['리드',r[6]==null?'—':`${(r[6]*3.28084).toFixed(0)}<small>ft</small>`],
    ['CarrotPilot',r[12]==null?(DATA&&DATA.hasCarrot===false?'<span style="opacity:.5">해당 없음</span>':solving):
      `<span style="color:#4ED88A">${(r[12]*MPH).toFixed(1)}<small>mph/s</small></span>`],
    ['tFollow (Carrot)',r[13]==null?'—':`${r[13].toFixed(2)}<small>s</small>`],
    ['xState (Carrot)',r[14]==null?'—':
      ['lead','cruise','e2eCruise','e2eStop','e2ePrepare','e2eStopped'][r[14]]||r[14]],
    ['갭',r[8]||'—'],
  ].map(([k,v])=>`<div class="rd"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
}

// 누르면 그 지점에 세워둔다. 재생은 재생 버튼으로만 -- 프레임을 하나씩 살펴보려고 누르는데
// 매번 영상이 달아나면 그 지점을 다시 잡아야 한다.
$('ch').onclick=e=>{
  const b=e.currentTarget.getBoundingClientRect(), L=46,R=14;
  const dur=DATA ? DATA.rows[DATA.rows.length-1][0]
                 : ((video() && isFinite(video().duration)) ? video().duration-VOFF : 60);
  const v=video(); if(v && !v.paused) v.pause();
  seek(Math.max(0,Math.min(1,(e.clientX-b.left-L)/(b.width-L-R)))*dur);
};
$('play').onclick=()=>{
  // Play plays. It used to kick off a solve when there was no result, which put a 17s job
  // behind a button whose label says nothing about computing anything.
  const v=video(); if(!v) return;
  if(v.paused) v.play().catch(()=>{}); else v.pause();
};
$('worst').onclick=()=>{
  if(worstT==null) return;
  const v=video(); if(v && !v.paused) v.pause();
  seek(Math.max(0, worstT-4));
};
$('worstStop').onclick=()=>{
  if(worstStopT==null) return;
  const v=video(); if(v && !v.paused) v.pause();
  seek(Math.max(0, worstStopT-4));
};
addEventListener('resize',draw);
boot(); tick();
</script>
</body></html>
"""

PAGE_VIDEO = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>openpilot 녹화 영상</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--ok:#4CC38A;--bad:#E5484D;
--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--ok:#1B7F53;--bad:#C42B30}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:16px;padding-bottom:calc(16px + env(safe-area-inset-bottom))}
h1{font-size:15px;margin:0 0 4px;letter-spacing:-.01em}
.back{display:inline-block;font-family:var(--m);font-size:11px;color:var(--dim);
text-decoration:none;margin-bottom:10px}.back:hover{color:var(--radar)}
.sub{font-family:var(--m);font-size:11px;color:var(--dim);margin-bottom:16px}
.wrap{display:grid;gap:12px;grid-template-columns:1fr}
@media(min-width:900px){.wrap{grid-template-columns:280px 1fr;align-items:start}}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;
margin-bottom:12px}
.h{font-family:var(--m);font-size:10px;letter-spacing:.14em;text-transform:uppercase;
color:var(--dim);margin-bottom:12px}
.rt{display:block;width:100%;text-align:left;background:transparent;color:inherit;
border:1px solid var(--line);border-radius:10px;padding:11px 12px;margin-bottom:8px;
cursor:pointer;font-family:inherit}
.rt:hover,.rt:focus-visible{border-color:var(--radar);outline:none}
.rt[aria-current="true"]{border-color:var(--radar);background:rgba(90,200,250,.08)}
.rt .n{font-family:var(--m);font-size:12.5px;margin-bottom:3px}
.rt .m{font-size:11px;color:var(--mut);font-variant-numeric:tabular-nums}
video{width:100%;display:block;border-radius:10px;background:#000;aspect-ratio:526/330}
.bar{display:flex;align-items:center;gap:10px;margin-top:12px;flex-wrap:wrap}
.bar .now{font-family:var(--m);font-size:12px;font-variant-numeric:tabular-nums;color:var(--mut)}
button.nav{background:var(--bg);color:var(--tx);border:1px solid var(--line);border-radius:9px;
padding:8px 13px;font-size:13px;font-family:inherit;cursor:pointer}
button.nav:hover:not([disabled]){border-color:var(--radar)}
button.nav[disabled]{color:var(--dim);cursor:default}
.segs{display:flex;flex-wrap:wrap;gap:6px;margin-top:12px}
.seg{min-width:34px;background:var(--bg);color:var(--mut);border:1px solid var(--line);
border-radius:7px;padding:6px 0;font-family:var(--m);font-size:11.5px;cursor:pointer;
font-variant-numeric:tabular-nums}
.seg:hover{border-color:var(--radar)}
.seg[aria-current="true"]{background:var(--radar);border-color:var(--radar);color:#04121b;font-weight:700}
.dl{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
.dl a{font-family:var(--m);font-size:11px;color:var(--mut);text-decoration:none;
border:1px solid var(--line);border-radius:7px;padding:6px 9px}
.dl a:hover{border-color:var(--radar);color:var(--radar)}
.note{font-size:11.5px;color:var(--dim);line-height:1.5;margin-top:12px}
.empty{font-size:13px;color:var(--mut);padding:8px 0}
label.chk{display:flex;align-items:center;gap:7px;font-size:12.5px;color:var(--mut);cursor:pointer}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<a class="back" href="/">← 메뉴</a>
<h1>녹화 영상</h1>
<div class="sub" id="sub">불러오는 중…</div>

<div class="wrap">
  <div class="card" style="margin:0">
    <div class="h">주행 기록</div>
    <div id="routes"><div class="empty">불러오는 중…</div></div>
  </div>

  <div class="card" style="margin:0">
    <div class="h" id="title">재생</div>
    <video id="v" controls playsinline preload="metadata"></video>
    <div class="bar">
      <button class="nav" id="prev">← 이전</button>
      <button class="nav" id="next">다음 →</button>
      <select id="cam" aria-label="카메라">
        <option value="road" selected>전방</option>
        <option value="wide">광각</option>
        <option value="driver">운전자</option>
      </select>
      <span class="now" id="now">주행 기록을 선택하세요</span>
      <label class="chk"><input type="checkbox" id="auto" checked> 자동 연속 재생</label>
    </div>
    <div class="segs" id="segs"></div>
    <div class="dl" id="dl"></div>
    <div class="note">전방 카메라의 저해상도 미리보기(526&times;330)를 MP4로 변환해 재생합니다.
      원본 전방·광각·운전자 카메라는 HEVC 원시 스트림이라 브라우저에서 재생되지 않아 내려받기만 제공합니다.</div>
  </div>
</div>

<script>
const $=i=>document.getElementById(i);
const v=$('v');
let routes=[],cur=null,seg=0;

const mb=b=>b>=1073741824?(b/1073741824).toFixed(1)+' GB':Math.round(b/1048576)+' MB';
const when=t=>new Date(t*1000).toLocaleString('ko-KR',
  {month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'});
const mins=n=>n>=60?`약 ${Math.floor(n/60)}시간 ${n%60}분`:`약 ${n}분`;

function drawRoutes(){
  const box=$('routes');
  if(!routes.length){box.innerHTML='<div class="empty">녹화된 영상이 없습니다.</div>';return;}
  box.innerHTML='';
  routes.forEach(r=>{
    const b=document.createElement('button');
    b.className='rt';b.setAttribute('aria-current',cur&&r.name===cur.name);
    b.innerHTML=`<div class="n">${r.name}</div>`+
      `<div class="m">${when(r.mtime)} · ${r.count}개 · ${mins(r.count)} · ${mb(r.bytes)}</div>`;
    b.onclick=()=>open_(r,0);
    box.appendChild(b);
  });
}

function drawSegs(){
  const box=$('segs');box.innerHTML='';
  if(!cur)return;
  cur.segments.forEach((s,i)=>{
    const b=document.createElement('button');
    b.className='seg';b.textContent=s.seg;b.setAttribute('aria-current',i===seg);
    b.onclick=()=>open_(cur,i);
    box.appendChild(b);
  });
  const d=$('dl');d.innerHTML='';
  (cur.segments[seg]?.downloads||[]).forEach(f=>{
    const a=document.createElement('a');
    a.href=`/dl/${cur.name}/${cur.segments[seg].seg}/${f.file}`;
    a.textContent=`${f.label} ${mb(f.bytes)}`;a.download='';
    d.appendChild(a);
  });
  $('prev').disabled=seg<=0;
  $('next').disabled=seg>=cur.segments.length-1;
  $('now').textContent=`세그먼트 ${seg+1}/${cur.segments.length}`;
  $('title').textContent=cur.name;
}

// 고화질은 두 경로다. HEVC 무변환 복사는 원본 그대로라 손실이 없고 빠르지만 브라우저가 HEVC
// 를 디코딩할 수 있어야 하고, H.264 재인코딩은 어디서나 재생되는 대신 세그먼트당 40초쯤 든다.
// 재생 가능한 쪽을 브라우저에게 물어서 고른다.
function bestQuality(){
  return document.createElement('video')
    .canPlayType('video/mp4; codecs="hvc1.1.6.L120.B0"') ? 'copy' : 'h264';
}
function open_(r,i,seekTo){
  cur=r;seg=Math.max(0,Math.min(i,r.segments.length-1));
  v.src=`/v/${r.name}/${r.segments[seg].seg}.mp4?cam=${$('cam').value}&q=${bestQuality()}`;
  if(seekTo!=null){
    // Arriving from an intervention: land on the moment, paused. Playing straight into it would
    // run past the thing you came to look at before the first frame is even decoded.
    v.onloadedmetadata=()=>{ try{ v.currentTime=seekTo; }catch(e){} v.onloadedmetadata=null; };
  }else{
    v.play().catch(()=>{});   // autoplay may be blocked; controls still work
  }
  drawRoutes();drawSegs();
}
$('cam').onchange=()=>{ if(cur) open_(cur, seg); };

function step(d){if(cur&&cur.segments[seg+d])open_(cur,seg+d);}
$('prev').onclick=()=>step(-1);
$('next').onclick=()=>step(1);
v.addEventListener('ended',()=>{if($('auto').checked)step(1);});
v.addEventListener('error',()=>{$('now').textContent='이 세그먼트를 재생할 수 없습니다';});
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  if(e.key==='ArrowRight'&&e.shiftKey){step(1);e.preventDefault();}
  if(e.key==='ArrowLeft'&&e.shiftKey){step(-1);e.preventDefault();}
});

(async()=>{
  try{
    const d=await(await fetch('/api/videos')).json();
    routes=d.routes||[];
    const segs=routes.reduce((a,r)=>a+r.count,0);
    const bytes=routes.reduce((a,r)=>a+r.bytes,0);
    $('sub').textContent=routes.length?`${routes.length}개 주행 · ${segs}개 세그먼트 · ${mb(bytes)}`
                                      :'녹화된 영상이 없습니다';
    drawRoutes();
    // ?route=&seg=&t= comes from the intervention log, which knows which segment an event
    // landed in and how far into it. Fall back to the newest route when it does not resolve --
    // an event older than the segments still on disk should not leave a blank page.
    const q=new URLSearchParams(location.search);
    const want=q.get('route'), wantSeg=q.get('seg'), wantT=q.get('t');
    let opened=false;
    if(want){
      const r=routes.find(x=>x.name===want);
      if(r){
        const i=Math.max(0, r.segments.findIndex(x=>String(x.seg)===String(wantSeg)));
        open_(r, i, wantT!=null?parseFloat(wantT):null);
        $('now').textContent=`개입 기록에서 이동 · 세그 ${wantSeg}` + (wantT!=null?` · ${wantT}초 지점`:'');
        opened=true;
      }else{
        $('sub').textContent=`${want} 의 영상이 디바이스에 없습니다 (오래되어 삭제됨)`;
      }
    }
    if(!opened && routes.length)open_(routes[0],0);
  }catch(e){$('sub').textContent='디바이스에 연결할 수 없습니다';}
})();
</script></body></html>"""


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', type=int, default=8088)
  args = ap.parse_args()

  Handler.state = State()
  Handler.params = Params()
  Handler.can = CanSource(Handler.params)
  Handler.videos = video_source.Mp4Cache()
  Handler.shadow = shadow_replay.ShadowReplay(lambda: bool(Handler.state.get().get('engaged')))
  Handler.shadow_scan = shadow_replay.ShadowScan(lambda: bool(Handler.state.get().get('engaged')))
  Handler.models = ModelManager(Handler.params)

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}  (commit {GIT_COMMIT})")
  srv.serve_forever()


if __name__ == "__main__":
  main()
