#!/usr/bin/env python3
"""Live viewer and tuning switchboard, served from the device over WiFi.

Real-car A/B testing otherwise means a laptop in the passenger seat. This serves a page you can
open on a phone: current lead perception state on the left, the switches that change it on the
right, so a run can be set up and its effect watched without stopping to SSH in.

  PYTHONPATH=/data/openpilot python3 selfdrive/debug/tuning_server.py
  # then open http://<device-ip>:8088 from anything on the same network

Pages: / index, /live lead perception and settings, /can every decoded CAN signal.

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
from openpilot.selfdrive.debug.can_source import CanSource, list_routes

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
    "options": [(0, "표준 1.10~1.75초"), (1, "가깝게 0.95~1.60초"),
                (2, "멀게 1.25~1.90초"), (3, "넓게 0.98~1.89초")],
  },
  "TFollowRiseRatePct": {
    "label": "Gap 확대 반영 속도", "type": "int",
    "help": "Gap을 크게 바꿨을 때 목표 거리가 늘어나는 속도입니다. 빠르면 즉각적이지만 "
            "감속이 급해질 수 있습니다. 줄일 때는 항상 즉시 반영됩니다.",
    "options": [(5, "느리게 0.05초/초"), (10, "표준 0.10초/초"), (20, "빠르게 0.20초/초")],
  },
  "DriverMonitorBypass": {
    "label": "드라이버 모니터링 우회", "type": "bool",
    "help": "운전자 주의 감시를 끕니다. 얼굴 미검출·주의 분산 경고와 강제 해제가 발생하지 않습니다. "
            "카메라와 모델은 그대로 돌아가므로 녹화는 유지되고, 전력 절감 효과는 없습니다.",
    "options": [(0, "사용 안 함 (기본)"), (1, "우회")],
  },
  "LongitudinalPersonality": {
    "label": "Driving personality", "type": "int",
    "help": "Gap 신호가 없을 때의 기본 추종 시간입니다.",
    "options": [(0, "aggressive"), (1, "standard"), (2, "relaxed")],
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

    page = self.path.split('?')[0].rstrip('/')
    if page == '/live':
      return self._send(200, PAGE_LIVE, 'text/html; charset=utf-8')
    if page == '/can':
      return self._send(200, PAGE_CAN, 'text/html; charset=utf-8')
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

<a class="card" href="/can">
  <div class="t">CAN · 전체 신호 뷰어</div>
  <div class="d">차량이 보내는 모든 CAN 메시지를 DBC로 디코딩해서 보여줍니다.
    값이 바뀐 신호를 표시하므로, 어떤 조작이 어떤 신호를 움직이는지 찾을 때 씁니다.</div>
  <div class="st"><span class="pill" id="p-can">–</span></div>
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
}
tick();setInterval(tick,1000);
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


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', type=int, default=8088)
  args = ap.parse_args()

  Handler.state = State()
  Handler.params = Params()
  Handler.can = CanSource(Handler.params)

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}  (commit {GIT_COMMIT})")
  srv.serve_forever()


if __name__ == "__main__":
  main()
