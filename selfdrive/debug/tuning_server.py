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
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.debug import vehicle_state, video_source
from openpilot.selfdrive.debug.can_source import CanSource, list_routes
from openpilot.selfdrive.debug import shadow_replay
from openpilot.selfdrive.debug.intervention_log import InterventionLog, list_events, read_event

# Options rather than free-form numbers: a typo in a text box goes straight into the braking
# path, and named choices are also what makes an A/B run reproducible afterwards.
SETTINGS = {
  "StoppedLeadMatchEnabled": {
    "label": "정지차 매칭 보정", "type": "bool",
    "help": "비전이 정지차를 달리는 차로 오독할 때 레이더 트랙을 유지합니다.",
    "options": [(1, "사용"), (0, "미사용")],
  },
  "StoppedLeadHoldMs": {
    "label": "정지차 확정 대기", "type": "int",
    "help": "거리·횡방향이 일치하는 상태가 이만큼 지속되면 정지차로 확정합니다. "
            "짧으면 빨리 반응하고, 길면 오검출에 보수적입니다.",
    "options": [(300, "빠르게 0.3초"), (500, "표준 0.5초"), (800, "신중히 0.8초"), (1200, "매우 신중 1.2초")],
  },
  "StopDistanceCm": {
    "label": "정지 시 앞차 간격", "type": "int",
    "help": "앞차 뒤에 멈출 때 남기는 거리입니다. 모든 속도의 추종 거리에 같은 값이 더해집니다.",
    "options": [(450, "가깝게 4.5m"), (500, "조금 가깝게 5.0m"), (600, "표준 6.0m"),
                (700, "여유 7.0m"), (800, "넓게 8.0m")],
  },
  "GapProfile": {
    "label": "차간거리 프로파일", "type": "int",
    "help": "스티어링 휠 Gap 1~7이 각각 몇 초 간격을 요구할지 정합니다.",
    "options": [(0, "표준 0.80~1.75초"), (1, "가깝게 0.80~1.60초"),
                (2, "멀게 0.95~1.90초"), (3, "넓게 0.80~1.90초")],
  },
  "TFollowRiseRatePct": {
    "label": "Gap 확대 반영 속도", "type": "int",
    "help": "Gap을 크게 바꿨을 때 목표 거리가 늘어나는 속도입니다. 빠르면 즉각적이지만 "
            "감속이 급해질 수 있습니다. 줄일 때는 항상 즉시 반영됩니다.",
    "options": [(10, "느리게 0.10초/초"), (35, "표준 0.35초/초"), (60, "빠르게 0.60초/초")],
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
  "LongitudinalPersonality": {
    "label": "Driving personality", "type": "int",
    "help": "Gap 신호가 없을 때의 기본 추종 시간입니다.",
    "options": [(0, "aggressive"), (1, "standard"), (2, "relaxed")],
  },
  "TeslaStockLong": {
    "label": "순정 ACC 사용", "type": "bool",
    "help": "속도 제어를 차의 순정 ACC에 맡기고 openpilot은 조향만 합니다. 순정 ACC는 이미 "
            "다듬어져 있으므로 롱 튜닝을 아예 건너뛰는 선택지입니다. 재시작해야 반영됩니다.",
    "options": [(0, "openpilot 롱 (기본)"), (1, "순정 ACC")],
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

    if self.path.startswith('/api/events'):
      # ?event=<name> returns the full sample window; without it, just the index
      qs = self.path.split('?', 1)[1] if '?' in self.path else ''
      name = next((p[6:] for p in qs.split('&') if p.startswith('event=')), None)
      if name:
        ev = read_event(name)
        return self._send(200 if ev else 404, json.dumps(ev or {'error': 'not found'}))
      return self._send(200, json.dumps({'events': list_events()}))

    if self.path.startswith('/api/settings'):
      out = {}
      for k, cfg in SETTINGS.items():
        try:
          # not get_bool(): it ignores the declared default and reads False until first write
          v = self.params.get(k, return_default=True)
          v = int(bool(v)) if cfg['type'] == 'bool' else int(v)
        except Exception:
          v = None
        out[k] = {'value': v, 'label': cfg['label'], 'help': cfg['help'],
                  'options': [{'v': ov, 'label': ol} for ov, ol in cfg['options']]}
      return self._send(200, json.dumps({
        'settings': out,
        'engaged': bool(self.state.get().get('engaged')),
      }))

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
      try:
        route, seg = page[3:].rsplit('/', 1)
        path = self.videos.get(route, int(seg.removesuffix('.mp4')), cam=cam, codec=codec)
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

  def do_POST(self):
    if self.path.startswith('/api/shadow'):
      n = int(self.headers.get('Content-Length', 0))
      try:
        req = json.loads(self.rfile.read(n) or b'{}')
      except json.JSONDecodeError:
        return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))
      route, seg = req.get('route'), req.get('seg')
      if not route or seg is None:
        return self._send(400, json.dumps({'error': '경로와 세그먼트를 지정하세요'}))
      am = req.get('accelMin')
      out = self.shadow.start(route, int(seg), float(am) if am not in (None, '') else None)
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
  const seg = e.route ? `<a class="vid" href="/videos">영상 보기</a>` : '';
  const d = document.createElement('div');
  d.className = 'detail';
  d.innerHTML = `<canvas></canvas>
    <div class="legend"><i style="background:var(--radar)"></i>openpilot 요구
      &nbsp;&nbsp;<i style="background:var(--mut)"></i>실제 가속도</div>
    <div class="meta">${seg}<a class="vid" href="/can">CAN 리플레이</a>
      <b>route</b> ${e.route || '-'}</div>`;
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

let cfg={}, staged={};

function renderSettings(){
  const box=$('settings');box.innerHTML='';
  for(const[k,c]of Object.entries(cfg)){
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
  cfg=d.settings;engaged=d.engaged;
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
.note{font-size:11.5px;color:var(--mut);line-height:1.6;padding:13px}
.note b{color:var(--tx);font-weight:600}
</style></head><body>
<a class="back" href="/">&larr; 돌아가기</a>
<h1>그림자 · 순정 ACC vs 지금 코드</h1>
<div class="sub" id="sub">불러오는 중…</div>

<div class="card">
  <h2>재생할 구간</h2>
  <div class="pad">
    <span class="lbl">주행</span><select id="route"></select>
    <span class="lbl">세그먼트</span><select id="seg"></select>
    <span class="lbl">제동 하한</span>
    <input id="amin" size="6" title="이 차의 포트값이 들어갑니다. 고쳐 넣으면 그 값으로 풉니다">
    <span class="lbl">m/s²</span>
    <button id="run">다시 풀기</button>
  </div>
  <div class="msg" id="msg">주행 중에는 실행하지 않습니다. MPC 를 매 프레임 다시 풀기 때문에 60초 구간에 수 초 걸립니다.</div>
</div>

<div class="card">
  <h2 id="chartTitle">결과</h2>
  <div class="vid" id="vidwrap"></div>
  <canvas id="ch"></canvas>
  <div class="legend">
    <span><i style="background:#F5B942"></i>순정 ACC 실제 (aEgo)</span>
    <span><i style="background:#8A97A6"></i>주행 당시 계획 (기록된 aTarget)</span>
    <span><i style="background:#5AC8FA"></i>지금 코드가 내놓는 계획</span>
    <span><i style="background:#FF6B5A"></i>하한에 붙은 구간</span>
  </div>
  <div class="pad" style="padding-top:0">
    <button id="play">▶ 재생</button>
    <button id="worst" title="지금 코드와 순정이 가장 크게 갈린 순간">최대 격차로</button>
    <span class="lbl" id="tt" style="margin-left:auto;font-family:var(--m)">—</span>
  </div>
  <div class="read" id="read"></div>
</div>

<div class="card">
  <h2>읽는 법</h2>
  <div class="note">
    노란 선과 회색 선은 <b>로그에 있는 그대로</b>라 코드를 고쳐도 움직이지 않습니다.
    파란 선만 지금 소스로 다시 푼 결과이므로, 상수를 바꾸고 다시 돌렸을 때
    <b>파란 선이 어떻게 달라지는지</b>가 그 변경의 효과입니다.
    <br><br>
    빨간 구간은 계획이 제동 하한에 닿아 <b>더 내려가지 못한</b> 구간입니다. 여기서 노란 선이
    파란 선보다 아래에 있으면, openpilot 이 보수적이었던 것이 아니라 한계에 막힌 것입니다.
    <br><br>
    상태는 매 프레임 로그값으로 다시 심습니다. 그대로 굴리면 1~2초 만에 실제 상황에서
    멀어져 같은 순간을 비교하는 의미가 사라지기 때문입니다.
  </div>
</div>

<script>
const MPH=2.2369363;
let DATA=null, poll=null, vt=0, VOFF=0, worstT=null;
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();

async function boot(){
  const d=await (await fetch('/api/shadow')).json();
  $('sub').textContent = `커밋 ${d.commit||''} · 녹화 주행 ${(d.routes||[]).length}개`;
  $('route').innerHTML = (d.routes||[]).map(r=>`<option>${r}</option>`).join('');
  if((d.routes||[]).length) await loadSegs();
  if(d.status==='done') show(d);
}
async function loadSegs(){
  const r=$('route').value;
  const d=await (await fetch('/api/shadow?route='+encodeURIComponent(r))).json();
  $('seg').innerHTML=(d.segments||[]).map(s=>`<option>${s}</option>`).join('');
  // 포트가 이 차에 주는 값을 그대로 채운다. 빈 칸이면 무엇과 비교하고 있는지 알 수 없다.
  if(d.floor!=null){ $('amin').value=(+d.floor).toFixed(2); $('amin').title=d.floorSrc||''; }
  pickSeg();
}
function pickSeg(){
  if($('seg').value==='') return;
  mountVideo($('route').value, +$('seg').value);
  solve();                       // 고르면 바로 푼다
}
$('route').onchange=loadSegs;
$('seg').onchange=pickSeg;

let SOLVED='';                   // 이미 푼(또는 푸는 중인) 구간+하한
async function solve(force){
  const route=$('route').value, seg=$('seg').value, amin=$('amin').value.trim();
  if(!route || seg==='') return;
  const key=`${route}/${seg}/${amin}`;
  if(!force && key===SOLVED) return;
  if(poll) return;               // 앞선 실행이 끝나기 전에는 겹쳐 쏘지 않는다
  SOLVED=key;
  DATA=null;
  $('run').disabled=true; $('msg').className='msg'; $('msg').textContent='푸는 중…';
  const res=await fetch('/api/shadow',{method:'POST',
    body:JSON.stringify({route, seg:+seg, accelMin:amin})});
  const d=await res.json();
  if(d.error){
    $('msg').className='msg err'; $('msg').textContent=d.error;
    $('run').disabled=false; SOLVED='';
    return;
  }
  poll=setInterval(check,700);
}
$('run').onclick=()=>solve(true);
$('amin').onchange=()=>solve();
async function check(){
  const d=await (await fetch('/api/shadow')).json();
  if(d.status==='running') return;
  clearInterval(poll); poll=null; $('run').disabled=false;
  if(d.status==='error'){ $('msg').className='msg err'; $('msg').textContent=d.error; SOLVED=''; return; }
  if(d.status==='done'){ $('msg').className='msg'; show(d); }
}

const video=()=>document.getElementById('v');

function show(d){
  DATA=d; vt=0; VOFF=0;
  $('chartTitle').textContent =
    `결과 — ${d.route} 세그 ${d.seg} · 하한 ${(+d.accelMin).toFixed(2)} m/s² (${d.accelMinSrc||''}) · 푸는 데 ${d.solveSec}초`;
  const hit=d.rows.filter(r=>r[6]&32).length;
  $('msg').textContent = `${d.rows.length}프레임, 하한에 닿은 프레임 ${hit}개 (${(hit/20).toFixed(1)}초)`;

  // 지금 코드가 순정보다 가장 많이 더 감속을 원한 순간
  let best=0; worstT=null;
  for(const r of d.rows){ const g=r[3]-r[1]; if(g<best){best=g; worstT=r[0];} }

  mountVideo(d.route, d.seg);
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

  g.fillStyle=css('--bad'); g.globalAlpha=.22;
  for(let i=0;i<rows.length-1;i++) if(rows[i][6]&32)
    g.fillRect(x(rows[i][0]),T,Math.max(1,x(rows[i+1][0])-x(rows[i][0])),ih);
  g.globalAlpha=1;

  g.font='10px '+css('--m'); g.textAlign='right'; g.lineWidth=1;
  for(const a of [2.0,0,DATA.accelMin]){
    const yy=Math.round(y(a))+.5;
    g.strokeStyle = a===DATA.accelMin ? css('--bad') : css('--line');
    g.beginPath(); g.moveTo(L,yy); g.lineTo(w-R,yy); g.stroke();
    g.fillStyle=css('--dim'); g.fillText((a*MPH).toFixed(1), L-6, yy+3);
  }
  g.textAlign='left'; g.fillStyle=css('--dim'); g.fillText('mph/s', 4, T+4);

  for(const [col,color,lw] of [[1,'#F5B942',1.8],[2,'#8A97A6',1.2],[3,'#5AC8FA',1.8]]){
    g.strokeStyle=color; g.lineWidth=lw; g.beginPath();
    rows.forEach((r,i)=>{ const px=x(r[0]),py=y(r[col]); i?g.lineTo(px,py):g.moveTo(px,py); });
    g.stroke();
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
  if(r) for(const [col,color] of [[1,'#F5B942'],[3,'#5AC8FA']]){
    g.beginPath(); g.arc(px,y(r[col]),4,0,7); g.fillStyle=color; g.fill();
    g.strokeStyle=css('--card'); g.lineWidth=1.8; g.stroke();
  }
  readout(r);
  $('tt').textContent=`${vt.toFixed(2)}s / ${dur.toFixed(0)}s`;
}

function readout(r){
  if(!r) return;
  const d=r[3]-r[1];
  $('read').innerHTML=[
    ['시각',`${r[0].toFixed(1)}<small>s</small>`],
    ['순정 실제',`${(r[1]*MPH).toFixed(1)}<small>mph/s</small>`],
    ['당시 계획',`${(r[2]*MPH).toFixed(1)}<small>mph/s</small>`],
    ['지금 코드',`<span style="color:${css('--radar')}">${(r[3]*MPH).toFixed(1)}<small>mph/s</small></span>`],
    ['지금−순정',`<span style="color:${d<0?css('--bad'):'inherit'}">${d>0?'+':''}${(d*MPH).toFixed(1)}<small>mph/s</small></span>`],
    ['속도',`${(r[4]*MPH).toFixed(0)}<small>mph</small>`],
    ['리드',r[5]==null?'—':`${(r[5]*3.28084).toFixed(0)}<small>ft</small>`],
    ['tFollow',`${r[8].toFixed(2)}<small>s</small>`],
    ['갭',r[7]||'—'],
    ['하한',(r[6]&32)?`<span style="color:${css('--bad')}">닿음</span>`:'—'],
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
  solve();                       // 결과가 없으면 재생과 함께 풀어둔다
  const v=video(); if(!v) return;
  if(v.paused) v.play().catch(()=>{}); else v.pause();
};
$('worst').onclick=()=>{
  if(worstT==null) return;
  const v=video(); if(v && !v.paused) v.pause();
  seek(Math.max(0, worstT-4));
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
function open_(r,i){
  cur=r;seg=Math.max(0,Math.min(i,r.segments.length-1));
  v.src=`/v/${r.name}/${r.segments[seg].seg}.mp4?cam=${$('cam').value}&q=${bestQuality()}`;
  v.play().catch(()=>{});   // autoplay may be blocked; controls still work
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
  Handler.shadow = shadow_replay.ShadowReplay(lambda: bool(Handler.state.get().get('engaged')))

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}  (commit {GIT_COMMIT})")
  srv.serve_forever()


if __name__ == "__main__":
  main()
