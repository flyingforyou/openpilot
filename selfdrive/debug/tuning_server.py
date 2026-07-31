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
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.debug import vehicle_state, video_source
from openpilot.selfdrive.debug.can_source import CanSource, list_routes

# Options rather than free-form numbers: a typo in a text box goes straight into the braking
# path, and named choices are also what makes an A/B run reproducible afterwards.
SETTINGS = {
  # ── 차간 · 추종 ────────────────────────────────────────────────────────────
  "GapProfile": {
    "group": "차간 · 추종",
    "label": "차간거리 프로파일", "type": "int",
    "help": "스티어링 휠 Gap 1~7이 각각 몇 초 간격을 요구할지 정합니다. 기본은 CarrotPilot 분포(1.10~1.60초)에 "
            "맞춰져 있습니다. gap 1이 1.10초 미만이면 시내에서 앞차 급감속 시 FCW(이머전시)가 뜰 수 있어 그 밑으로는 안 갑니다.",
    "options": [(0, "표준 1.10~1.60초"), (1, "가깝게 1.10~1.45초"), (2, "멀게 1.25~1.75초"), (3, "넓게 1.10~1.69초")],
  },
  "StopDistanceCm": {
    "group": "차간 · 추종",
    "label": "정지 시 앞차 간격", "type": "int",
    "help": "앞차 뒤에 멈출 때 남기는 거리입니다. 모든 속도의 추종 거리에 같은 값이 더해집니다.",
    "options": [(450, "가깝게 4.5m"), (500, "조금 가깝게 5.0m"), (600, "표준 6.0m"),
                (700, "여유 7.0m"), (800, "넓게 8.0m")],
  },
  "TFollowRiseRatePct": {
    "group": "차간 · 추종",
    "label": "Gap 확대 반영 속도", "type": "int",
    "help": "Gap을 크게 바꿨을 때 목표 거리가 늘어나는 속도입니다. 빠르면 즉각적이지만 "
            "감속이 급해질 수 있습니다. 줄일 때는 항상 즉시 반영됩니다. "
            "gap 1↔7 전 구간 전환에 0.10은 5.0초, 0.35는 약 1.4초, 0.50은 1.0초가 걸립니다.",
    "options": [(10, "느리게 0.10초/초"), (20, "보통 0.20초/초"),
                (35, "빠르게 0.35초/초 (기본)"), (50, "즉각 0.50초/초")],
  },
  "DynamicTFollowGain": {
    "group": "차간 · 추종",
    "label": "앞차 움직임 따라 차간 조절", "type": "int",
    "help": "앞차의 가감속(저크)에 맞춰 차간시간을 실시간으로 조절합니다. 앞차가 풀어주면 차간을 "
            "줄이고 동시에 가속을 더 빨리 붙여(jerk 비용 절반) 바로 따라붙습니다. 앞차가 감속하면 미리 "
            "벌립니다. 값은 차간 조절 폭(초)입니다. 가다 서다 재출발과 앞차 따라붙기가 매끄러워집니다. "
            "선택한 Gap(1~7) 위에 얹혀 동작하며, 0이면 사용 안 함. CarrotPilot의 dynamic_t_follow 이식.",
    "options": [(0, "사용 안 함 (기본)"), (30, "약하게 0.30초"),
                (50, "표준 0.50초"), (80, "강하게 0.80초")],
  },
  "LeadCreepFollowCms": {
    "group": "차간 · 추종",
    "label": "기어가는 앞차 따라가기", "type": "int",
    "help": "앞차가 이 속도 이상으로 0.2초 이상 계속 움직일 때만 완전정지하지 않고 함께 굴러갑니다. "
            "정지거리 근처, 빠르게 접근 중, 레이더 트랙이 바뀐 직후에는 안전하게 정상 정지합니다. "
            "정지차 속도 노이즈 한두 번으로 정지 latch가 풀리지 않도록 보호합니다. 0이면 사용 안 함.",
    "options": [(0, "사용 안 함 (기본)"), (30, "0.30 m/s 이상"),
                (50, "0.50 m/s 이상"), (80, "0.80 m/s 이상")],
  },
  "StoppedLeadMatchEnabled": {
    "group": "차간 · 추종",
    "label": "정지차 매칭 보정", "type": "bool",
    "help": "비전이 정지차를 달리는 차로 오독할 때 레이더 트랙을 유지합니다.",
    "options": [(1, "사용"), (0, "미사용")],
  },
  "StoppedLeadHoldMs": {
    "group": "차간 · 추종",
    "label": "정지차 확정 대기", "type": "int",
    "help": "거리·횡방향이 일치하는 상태가 이만큼 지속되면 정지차로 확정합니다. "
            "짧으면 빨리 반응하고, 길면 오검출에 보수적입니다.",
    "options": [(300, "빠르게 0.3초"), (500, "표준 0.5초"), (800, "신중히 0.8초"), (1200, "매우 신중 1.2초")],
  },
  # ── 가속 · 감속 ────────────────────────────────────────────────────────────
  "LaunchAccelCms": {
    "group": "가속 · 감속",
    "label": "출발 가속 상한", "type": "int",
    "help": "정지 상태에서 뗄 때 허용하는 최대 가속도입니다. 감속에는 영향이 없습니다. "
            "36km/h 이후로는 기존 곡선 그대로이고, panda 안전 상한이 2.0이라 그 위로는 못 갑니다. "
            "정지→30km/h 도달 시간은 1.60이 5.2초, 1.90이 4.4초, 2.00이 4.2초입니다.",
    "options": [(160, "기본 1.60 m/s²"), (180, "빠르게 1.80 m/s²"),
                (190, "더 빠르게 1.90 m/s²"), (200, "최대 2.00 m/s²")],
  },
  "CurveSpeedLatAccelCms": {
    "group": "가속 · 감속",
    "label": "커브 감속 강도", "type": "int",
    "help": "커브를 미리 인식해서 속도를 줄일 때, 코너에서 허용하는 횡가속(횡G)입니다. "
            "낮을수록 더 일찍·더 많이 감속해 부드럽고, 높을수록 속도를 더 유지해 스포티합니다. "
            "'끔'을 고르면 커브 감속을 완전히 비활성화합니다 — 미리 줄이는 피드포워드와 조향 부하 "
            "백스톱이 모두 꺼지므로, 코너에서 스스로 감속하지 않습니다. 이때는 설정한 크루즈 "
            "속도로 커브에 진입하니 직접 속도를 조절하세요. 감속만 조절하며 가속은 못 올립니다.",
    "options": [(0, "끔 · 커브 감속 안 함"), (190, "부드럽게 1.90 m/s² · 일찍 감속"),
                (220, "표준 2.20 m/s² (기본)"), (240, "스포티 2.40 m/s²"),
                (260, "최소 감속 2.60 m/s²")],
  },
  "TeslaVelocityPid": {
    "group": "가속 · 감속",
    "label": "정지 정밀도 · 속도 PID", "type": "bool",
    "help": "정지 정밀도 묶음(CarrotPilot 이식). ①속도 PID: 가속 명령 그대로 내보내기(피드포워드, "
            "게인 0이라 지나침)에서 계획 속도를 닫힌 루프로 추종으로 바꿔 계획한 지점에 정확히 섭니다. "
            "②fcw_stop: 앞차 4m 이내면 정지 램프로 즉시 커밋해 기어들지 않게 합니다. ③v_soft: 정지·저속 "
            "앞차에 접근할 때 남은 거리에 맞춰 속도를 물리 곡선으로 낮춰 매끄럽게 섭니다. 게인은 carrot "
            "기본값(kp 1.0)이라 미세 조정이 필요할 수 있습니다. 변경 후 재시동이 필요합니다.",
    "options": [(0, "피드포워드 (기본)"), (1, "속도 PID")],
  },
  # ── 조향 · 같이 돌리기 ─────────────────────────────────────────────────────
  "TeslaCoopSteer": {
    "group": "조향 · 같이 돌리기",
    "label": "핸들 같이 돌리기", "type": "bool",
    "help": "핸들을 돌리면 openpilot이 조향을 놓는 대신, 운전자가 미는 힘만큼 목표 방향을 "
            "함께 옮깁니다. 세게 밀 필요가 없어져서 차가 조향을 차단하는 지점(약 3.2Nm)까지 "
            "가지 않습니다. 그래도 확 꺾으면 기존처럼 전체 해제됩니다. 변경 후 재시동이 필요합니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "같이 돌리기")],
  },
  "TeslaCoopMaxTorqueCNm": {
    "group": "조향 · 같이 돌리기",
    "label": "같이 돌리기 · 허용 힘", "type": "int",
    "help": "운전자가 이 힘까지 미는 동안 목표를 따라 옮깁니다. 넘으면 더 못 따라가므로 "
            "차가 조향을 차단할 수 있습니다. 이 차는 약 3.2Nm에서 차단되므로 그보다 낮아야 합니다.",
    "options": [(200, "좁게 2.0 Nm"), (250, "표준 2.5 Nm"), (300, "넓게 3.0 Nm · 차단점에 근접")],
  },
  "TeslaCoopLatAccelCms": {
    "group": "조향 · 같이 돌리기",
    "label": "같이 돌리기 · 반응 강도", "type": "int",
    "help": "같은 힘으로 얼마나 많이 틀어줄지입니다. 크면 핸들이 가볍게 느껴지고 조금만 밀어도 "
            "많이 돌아갑니다. 작으면 묵직하지만 의도하지 않은 개입이 줄어듭니다.",
    "options": [(100, "묵직하게 1.0 m/s²"), (150, "표준 1.5 m/s²"), (200, "가볍게 2.0 m/s²")],
  },
  # ── 주차 · 안전 ────────────────────────────────────────────────────────────
  "TeslaStockLong": {
    "group": "주차 · 안전",
    "label": "Tesla 순정 ACC 사용", "type": "bool",
    "help": "가감속을 openpilot이 아니라 Tesla 순정 크루즈(TACC)에 맡깁니다. openpilot은 조향만 합니다. "
            "롱 튜닝(gap·정지거리·추종 등)이 전부 무의미해지는 대신 순정 ACC의 검증된 거동을 씁니다. "
            "이 HW1 세팅에서 순정 ACC가 실제로 동작하는지는 실차 확인이 필요합니다. 변경 후 재시동이 필요합니다.",
    "options": [(0, "openpilot 롱 (기본)"), (1, "순정 ACC")],
  },
  "TeslaStockAutopark": {
    "group": "주차 · 안전",
    "label": "순정 오토파크 허용", "type": "bool",
    "help": "openpilot이 해제된 동안 순정 AP1의 조향·가감속 메시지를 차로 통과시킵니다. "
            "오토파크가 동작하게 되지만, 순정 오토스티어 조향도 함께 통과합니다. "
            "openpilot이 작동 중일 때는 항상 차단됩니다. 변경 후 재시동이 필요합니다.",
    "options": [(0, "차단 (기본)"), (1, "통과")],
  },
  "DriverMonitorBypass": {
    "group": "주차 · 안전",
    "label": "드라이버 모니터링 우회", "type": "bool",
    "help": "운전자 주의 감시를 끕니다. 얼굴 미검출·주의 분산 경고와 강제 해제가 발생하지 않습니다. "
            "카메라와 모델은 그대로 돌아가므로 녹화는 유지되고, 전력 절감 효과는 없습니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "우회")],
  },
}

# One-tap A/B presets. Each stages a bundle of the low-speed lead-following knobs at once so a
# scenario can be set up with a single tap on the road, then committed with 반영 (same staged flow
# as the individual selects). Mirrors tesla-long-test-scenarios.html. Values are validated against
# each setting's options at import so a bad preset fails fast rather than at POST time.
SCENARIOS = [
  {"id": "baseline", "label": "기본 · 둘 다 off", "desc": "회귀 확인 · 예전과 동일",
   "set": {"DynamicTFollowGain": 0, "LeadCreepFollowCms": 0}},
  {"id": "creep30", "label": "A · 기어가는 앞차 0.30", "desc": "완전정지 회피(약) · 가다서다 핵심",
   "set": {"LeadCreepFollowCms": 30, "DynamicTFollowGain": 0}},
  {"id": "creep50", "label": "A · 기어가는 앞차 0.50", "desc": "완전정지 회피(표준)",
   "set": {"LeadCreepFollowCms": 50, "DynamicTFollowGain": 0}},
  {"id": "dyn50", "label": "B · 차간 동적조절 0.50", "desc": "앞차 저크로 추종 부드럽게",
   "set": {"DynamicTFollowGain": 50, "LeadCreepFollowCms": 0}},
  {"id": "combo", "label": "A+B 병행", "desc": "완전정지 회피 + 부드러움 · carrot 근접",
   "set": {"LeadCreepFollowCms": 30, "DynamicTFollowGain": 50}},
]

for _sc in SCENARIOS:
  for _k, _v in _sc["set"].items():
    assert _k in SETTINGS, f"scenario {_sc['id']}: unknown setting {_k}"
    assert _v in [_ov for _ov, _ in SETTINGS[_k]["options"]], f"scenario {_sc['id']}: {_k}={_v} not an option"

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
    threading.Thread(target=self._run, daemon=True).start()

  def _run(self):
    sm = messaging.SubMaster(STATE_SERVICES)
    while True:
      sm.update(100)
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
          'lead': {
            'status': bool(lead.status),
            'source': ('R' if lead.radar else 'V') if lead.status else None,
            'trackId': int(lead.radarTrackId),
            'dRel': round(lead.dRel, 1),
            'vLead': round(lead.vLead, 1),
            'prob': round(lead.modelProb, 2),
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
      return self._send(200, json.dumps(self.state.get()))

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

    if self.path.startswith('/api/settings'):
      out = {}
      for k, cfg in SETTINGS.items():
        try:
          # not get_bool(): it ignores the declared default and reads False until first write
          v = self.params.get(k, return_default=True)
          v = int(bool(v)) if cfg['type'] == 'bool' else int(v)
        except Exception:
          v = None
        out[k] = {'value': v, 'label': cfg['label'], 'help': cfg['help'], 'group': cfg.get('group', ''),
                  'options': [{'v': ov, 'label': ol} for ov, ol in cfg['options']]}
      return self._send(200, json.dumps({
        'settings': out,
        'scenarios': SCENARIOS,
        'engaged': bool(self.state.get().get('engaged')),
      }))

    if self.path.startswith('/api/videos'):
      return self._send(200, json.dumps({'routes': video_source.list_videos()}))

    page = self.path.split('?')[0].rstrip('/')

    # /v/<route>/<seg>.mp4 -- the road preview, remuxed so a browser will play it
    if page.startswith('/v/'):
      try:
        route, seg = page[3:].rsplit('/', 1)
        path = self.videos.get(route, int(seg.removesuffix('.mp4')))
      except (ValueError, FileNotFoundError):
        return self._send(404, '{}')
      except Exception as e:
        return self._send(500, json.dumps({'error': f'{type(e).__name__}: {e}'}))
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
    if page == '/can':
      return self._send(200, PAGE_CAN, 'text/html; charset=utf-8')
    if page == '/vehicle':
      return self._send(200, PAGE_VEHICLE, 'text/html; charset=utf-8')
    if page == '/videos':
      return self._send(200, PAGE_VIDEO, 'text/html; charset=utf-8')
    if page == '/guide':
      return self._send(200, PAGE_GUIDE, 'text/html; charset=utf-8')
    return self._send(200, PAGE_INDEX, 'text/html; charset=utf-8')

  def do_POST(self):
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
    unknown = [k for k in changes if k not in SETTINGS]
    if unknown:
      return self._send(400, json.dumps({'error': f'알 수 없는 설정: {", ".join(unknown)}'}))

    for key, value in changes.items():
      cfg = SETTINGS[key]
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
.grp{font-family:var(--m);font-size:10px;letter-spacing:.14em;text-transform:uppercase;color:var(--radar);
margin:16px 0 2px;padding-top:11px;border-top:1px solid var(--line)}
#settings .grp:first-child{border-top:0;padding-top:2px;margin-top:0}
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
.scnnote{font-size:11.5px;color:var(--mut);line-height:1.45;margin-bottom:9px}
.scnbox{display:flex;flex-direction:column;gap:8px}
.scn{display:flex;flex-direction:column;align-items:flex-start;gap:2px;text-align:left;width:100%;
background:var(--bg);border:1px solid var(--line);border-radius:10px;padding:10px 12px;color:var(--tx);
cursor:pointer;font-family:inherit}
.scn:hover,.scn:focus-visible{border-color:var(--radar);outline:none}
.scn.active{border-color:var(--ok);box-shadow:inset 0 0 0 1px var(--ok)}
.scn .sl{font-size:14px}.scn .sd{font-size:11.5px;color:var(--mut)}
.scn.active .sl::after{content:" · 적용됨";color:var(--ok);font-size:11px;font-family:var(--m)}
.scn.staged{border-color:var(--vision)}
.scn.staged .sl::after{content:" · 대기";color:var(--vision);font-size:11px;font-family:var(--m)}
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
    <div><div class="k">속도</div><div class="v"><span id="vego">–</span><small>m/s</small></div></div>
    <div><div class="k">gap</div><div class="v" id="gap">–</div></div>
    <div><div class="k">aTarget</div><div class="v" id="atgt">–</div></div>
    <div><div class="k">상태</div><div class="v" style="font-size:13px"><span id="eng" class="pill">–</span></div></div>
    <div><div class="k">blindspot</div><div class="v" style="font-size:13px"><span id="bs" class="pill">–</span></div></div>
  </div>
</div>

<div class="card"><div class="h">테스트 시나리오</div>
  <div class="scnnote">한 번 눌러 세팅을 모아 스테이징 → 아래 <b>반영</b>으로 적용. 해제 상태에서 반영해야 다음 engage부터 적용됩니다.</div>
  <div id="scenarios" class="scnbox"></div>
</div>

<div class="card"><div class="h">Settings</div><div id="settings"></div>
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
    $('vego').textContent=s.vEgo??'–';
    $('gap').textContent=s.gap||'–';
    $('atgt').textContent=s.aTarget??'–';
    const e=$('eng');e.textContent=s.engaged?'engaged':'disengaged';
    e.className='pill '+(s.engaged?'on':'off');
    const b=$('bs'),[l,r]=s.blindspot||[false,false];
    b.textContent=l&&r?'L R':l?'L':r?'R':'없음';
    b.className='pill '+((l||r)?'on':'off');
  }catch(e){$('conn').textContent='디바이스에 연결할 수 없습니다';}
}

let cfg={}, staged={}, scenarios=[];

function effVal(k){ return (k in staged)?staged[k]:(cfg[k]?cfg[k].value:null); }

function renderScenarios(){
  const box=$('scenarios');if(!box)return;box.innerHTML='';
  scenarios.forEach(sc=>{
    const keys=Object.keys(sc.set).filter(k=>k in cfg);
    const matches=keys.every(k=>effVal(k)===sc.set[k]);
    const anyStaged=keys.some(k=>k in staged);
    const b=document.createElement('button');
    b.className='scn'+(matches?(anyStaged?' staged':' active'):'');
    b.innerHTML=`<span class="sl">${sc.label}</span><span class="sd">${sc.desc}</span>`;
    b.onclick=()=>{
      for(const k of keys){
        if(sc.set[k]===cfg[k].value) delete staged[k]; else staged[k]=sc.set[k];
      }
      renderSettings();renderScenarios();updateApply();
    };
    box.appendChild(b);
  });
}

function renderSettings(){
  const box=$('settings');box.innerHTML='';
  let lastGrp=null;
  for(const[k,c]of Object.entries(cfg)){
    if(c.group && c.group!==lastGrp){
      const g=document.createElement('div');g.className='grp';g.textContent=c.group;
      box.appendChild(g);lastGrp=c.group;
    }
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
      renderSettings();renderScenarios();updateApply();
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
  cfg=d.settings;engaged=d.engaged;scenarios=d.scenarios||[];
  renderSettings();renderScenarios();updateApply();
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

loadSettings();poll();setInterval(poll,300);
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

<a class="card" href="/guide">
  <div class="t">설명 · 튜닝 옵션이 뭘 바꾸나</div>
  <div class="d">저속 추종·커브 감속 옵션이 실제로 무엇을 바꾸는지 쉬운 비유로 설명하고,
    옵션별 캐치포인트(언제 켜고, 뭘 보고, 뭘 조심할지)를 정리했습니다.</div>
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
    document.getElementById('sub').textContent=
      s.connected?(s.onroad?'주행 중 · onroad':'정차 · offroad'):'openpilot 대기 중';
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


PAGE_GUIDE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>설명 · 튜닝 옵션</title><style>
:root{--bg:#0B0F14;--card:#141C26;--line:#243040;--tx:#E4EAF0;--mut:#8A97A6;--dim:#5D6B7B;
--radar:#5AC8FA;--ok:#4CC38A;--warn:#F5B942;--bad:#E5484D;--m:ui-monospace,SFMono-Regular,Menlo,monospace;
--s:system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
@media(prefers-color-scheme:light){:root{--bg:#F4F7FA;--card:#fff;--line:#DCE3EA;--tx:#0E151D;
--mut:#54636F;--dim:#8494A2;--radar:#0A72A8;--ok:#1B7F53;--warn:#9A6210;--bad:#C42B30}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:var(--s);
padding:22px;padding-bottom:calc(30px + env(safe-area-inset-bottom));max-width:760px;margin-inline:auto}
.back{display:inline-block;font-family:var(--m);font-size:11px;color:var(--dim);text-decoration:none;margin-bottom:10px}
.back:hover{color:var(--radar)}
h1{font-size:19px;margin:0 0 3px}
.sub{font-size:12.5px;color:var(--mut);line-height:1.55;margin-bottom:18px}
h2{font-size:15.5px;margin:22px 0 9px;color:var(--radar)}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px 15px;margin:11px 0}
.card .lab{font-size:11px;font-family:var(--m);color:var(--dim);letter-spacing:.04em;margin-bottom:8px}
.blk{margin:8px 0}
.blk .t{font-size:12.5px;color:var(--mut);margin-bottom:3px}
p{margin:5px 0;font-size:14px;line-height:1.6}
.vs{display:grid;grid-template-columns:auto 1fr;gap:4px 10px;font-size:13.5px;margin:6px 0}
.vs .w{color:var(--dim);font-family:var(--m);font-size:11px;padding-top:2px}
ul{margin:6px 0 4px;padding-left:20px}li{margin:4px 0;font-size:13.5px;line-height:1.55}
b.ok{color:var(--ok)}b.warn{color:var(--warn)}b.bad{color:var(--bad)}
code{font-family:var(--m);background:var(--bg);border:1px solid var(--line);border-radius:5px;padding:1px 5px;font-size:12px}
.em{color:var(--radar)}
</style></head><body>
<a class="back" href="/">← 메뉴</a>
<h1>튜닝 옵션이 뭘 바꾸나</h1>
<div class="sub">openpilot이 앞차를 따라가는 건 = 네가 <b>줄 서서 앞사람 뒤를 걷는 것</b>과 똑같아. 이 비유로 설명할게.
옵션은 안 켜면 예전과 100% 동일하니 하나씩 켜봐.</div>

<h2>1. 기어가는 앞차 따라가기 <code>LeadCreepFollowCms</code></h2>
<div class="card">
  <div class="lab">줄이 아주 조금씩 앞으로 갈 때</div>
  <div class="vs">
    <div class="w">예전</div><div>앞사람이 찔끔 움직여도 나는 <b>완전히 멈춰 섰다가</b>, 좀 멀어지면 <b>후다닥 따라붙어.</b> → 섰다 붙었다 덜컹덜컹.</div>
    <div class="w">지금</div><div>앞사람이 계속 조금씩 움직이면 나도 <b class="em">같이 살살 걸어가.</b> 앞사람이 진짜 딱 멈추면 그때 나도 멈춤.</div>
  </div>
  <div class="blk"><div class="t">한 줄 로직</div>
  <p>예전엔 "내가 갈 속도가 거의 0이네 → <b>완전 멈춤 확정!</b>" 스위치가 앞차가 움직이든 말든 눌려버렸어.
  지금은 "앞차가 아직 이 속도 이상으로 굴러가면 → <b>멈춤 스위치 누르지 마</b>" 조건을 붙인 거야.</p></div>
  <div class="blk"><div class="t">캐치포인트</div><ul>
    <li><b class="ok">✅ 정체·가다서다에서 효과 최고.</b> "섰다 크립으로 따라잡기" 덜컹임이 사라짐.</li>
    <li><b class="warn">⚠️ 숫자 = 문턱 속도.</b> 낮을수록(0.30) 아주 느린 기어감까지 따라가고, 높을수록(0.80) 어지간히 움직여야 따라감.</li>
    <li><b class="bad">🚨 앞차가 진짜 멈췄는데 안 서는 느낌</b>이면 문턱을 올려(0.30→0.50). 그래도 이상하면 브레이크 밟고 off.</li>
    <li>실제 브레이크 세기는 그대로라 <b>앞차에 박는 위험은 없음.</b> "완전 정지 잠금"만 뺀 거.</li>
  </ul></div>
</div>

<h2>2. 앞차 움직임 따라 차간 조절 <code>DynamicTFollowGain</code></h2>
<div class="card">
  <div class="lab">앞사람 걸음이 바뀌는 순간에 간격을 잠깐 조절</div>
  <div class="vs">
    <div class="w">빨라짐</div><div>앞사람이 <b>갑자기 빨라지면</b> 간격을 <b class="em">좁히고 가속도 확 붙여</b>(jerk 절반) 바로 따라붙어.</div>
    <div class="w">느려짐</div><div>앞사람이 <b>갑자기 느려지면</b> 간격을 미리 <b class="em">벌려서</b> 급브레이크 없이 부드럽게 받아.</div>
  </div>
  <div class="blk"><div class="t">한 줄 로직</div>
  <p>"앞차 속도"가 아니라 <b>"속도가 변하는 정도(급가속/급감속)"</b>를 보고 간격을 밀고 당겨.
  정속으로 잘 갈 땐 아무 일 안 하고, <b>변화가 생기는 순간에만</b> 작동해.</p></div>
  <div class="blk"><div class="t">캐치포인트</div><ul>
    <li><b class="ok">✅ 재출발·감속 받아치기가 부드러워짐.</b> 1번이 "완전정지 자체"를 없앤다면, 이건 그 위에 <b>부드러움</b>을 얹는 보조.</li>
    <li><b class="warn">⚠️ 숫자 = 조절 폭(초).</b> 클수록 더 많이 밀당함.</li>
    <li><b class="em">🔑 일시적(과도) 효과.</b> 앞차가 일정 속도로 자리잡으면 간격은 네가 고른 Gap(1~7)으로 <b>되돌아와.</b> 정상임.</li>
    <li>정속 고속에선 거의 체감 없음. <b>변화 잦은 저속에서 진가.</b></li>
  </ul></div>
</div>

<h2>3. 커브 감속 강도 <code>CurveSpeedLatAccelCms</code></h2>
<div class="card">
  <div class="lab">코너를 미리 보고 얼마나 세게 파고들지</div>
  <div class="vs">
    <div class="w">낮게</div><div>코너에서 <b>일찍·많이 감속</b> — 편안, 안 쏠림.</div>
    <div class="w">높게</div><div><b>속도 유지</b>하며 코너 통과 — 스포티, 좀 쏠림.</div>
    <div class="w">끔</div><div>코너에서 <b class="bad">스스로 감속 안 함</b> — 내가 알아서 줄여야 함.</div>
  </div>
  <div class="blk"><div class="t">한 줄 로직</div>
  <p>예전엔 "코너에서 허용하는 쏠림 정도"가 <b>고정 숫자</b>로 코드에 박혀 있었어. 그걸 밖으로 빼서 고를 수 있게 한 거야.</p></div>
  <div class="blk"><div class="t">캐치포인트</div><ul>
    <li><b class="em">🔑 끔을 골라도 진짜 위험한 한계는 안전장치가 따로 지킴</b> — 근데 "끔"은 그것까지 꺼서 <b>코너 전 자동 감속을 아예 안 함.</b> 직접 줄여야 함.</li>
    <li><b class="warn">⚠️ 기본값(2.20)은 예전 그대로</b>라 안 만지면 예전과 똑같음.</li>
    <li>감속만 조절. 가속을 더 붙여주는 기능이 아님.</li>
  </ul></div>
</div>

<h2>공통 (전부 해당)</h2>
<div class="card"><ul>
  <li><b class="ok">🟢 셋 다 기본 off/기본값</b> → 안 켜면 예전과 100% 동일. 안전하게 하나씩.</li>
  <li><b>🔄 바꾼 값은 "해제 상태에서 반영"돼야 적용됨.</b> 주행 중 목표거리가 안 튀게 일부러 그렇게 함. 값 바꾸고 반영 → 한 번 해제했다 다시 걸기.</li>
  <li><b>🧪 한 번에 하나만</b> 바꿔서 A/B 해야 원인을 알 수 있어. (그래서 <a class="em" href="/live">Live</a>에 시나리오 버튼을 만든 것)</li>
  <li><b class="em">🎯 1번이 네 불만("완전정지 후 크립")의 진짜 해결책</b>, 2번은 부드러움 보강. 테스트도 1번부터.</li>
</ul></div>

</body></html>"""


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

function open_(r,i){
  cur=r;seg=Math.max(0,Math.min(i,r.segments.length-1));
  v.src=`/v/${r.name}/${r.segments[seg].seg}.mp4`;
  v.play().catch(()=>{});   // autoplay may be blocked; controls still work
  drawRoutes();drawSegs();
}

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
    if(routes.length)open_(routes[0],0);
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

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}  (commit {GIT_COMMIT})")
  srv.serve_forever()


if __name__ == "__main__":
  main()
