#!/usr/bin/env python3
"""Live viewer and tuning switchboard, served from the device over WiFi.

Real-car A/B testing otherwise means a laptop in the passenger seat. This serves a page you can
open on a phone: current lead perception state on the left, the switches that change it on the
right, so a run can be set up and its effect watched without stopping to SSH in.

  PYTHONPATH=/data/openpilot python3 selfdrive/debug/tuning_server.py
  # then open http://<device-ip>:8088 from anything on the same network

Pages: / index, /live lead perception and settings, /can every decoded CAN signal.

Control settings are refused while openpilot is engaged -- changing how the car decides to
brake while it is actively braking is a different risk than changing it with the driver in
control. Disengaged is fine, so options can be swapped at a light without shutting anything
down. Pass --allow-engaged to lift that; the page will say so.
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.debug.can_viewer import CanDecoder

# name -> (type, label, help). Kept small on purpose: an A/B switch you can reason about beats
# a form full of raw floats you can typo into the braking path.
SETTINGS = {
  "StoppedLeadMatchEnabled": ("bool", "정지차 매칭 보정",
                              "비전이 정지차를 달리는 차로 오독할 때 레이더 트랙을 유지"),
  "StoppedLeadHoldMs": ("int", "확정 대기 시간 (ms)",
                        "거리·횡방향이 일치하는 상태가 이만큼 지속되면 정지차로 확정"),
  "LongitudinalPersonality": ("int", "Driving personality",
                              "0 aggressive · 1 standard · 2 relaxed"),
}

STATE_SERVICES = ['carState', 'radarState', 'selfdriveState', 'longitudinalPlan', 'deviceState']


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
  can: 'LazyCan'
  allow_engaged: bool

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
    if self.path.startswith('/api/state'):
      return self._send(200, json.dumps(self.state.get()))

    if self.path.startswith('/api/can'):
      dec = self.can.get()
      if dec is None:
        return self._send(200, json.dumps({'messages': [], 'dbc': None, 'total': 0,
                                           'error': '차량 인식 대기 중 (CarParams 없음)'}))
      return self._send(200, json.dumps(dec.snapshot('changed=1' in self.path)))

    if self.path.startswith('/api/settings'):
      out = {}
      for k, (kind, label, help_) in SETTINGS.items():
        try:
          # not get_bool(): it ignores the declared default and reads False until first write
          v = self.params.get(k, return_default=True)
          v = bool(v) if kind == 'bool' else v
        except Exception:
          v = None
        out[k] = {'value': v, 'type': kind, 'label': label, 'help': help_}
      return self._send(200, json.dumps({'settings': out, 'allowEngaged': self.allow_engaged}))

    page = self.path.split('?')[0].rstrip('/')
    if page == '/live':
      return self._send(200, PAGE_LIVE, 'text/html; charset=utf-8')
    if page == '/can':
      return self._send(200, PAGE_CAN, 'text/html; charset=utf-8')
    return self._send(200, PAGE_INDEX, 'text/html; charset=utf-8')

  def do_POST(self):
    if not self.path.startswith('/api/settings'):
      return self._send(404, '{}')

    n = int(self.headers.get('Content-Length', 0))
    try:
      req = json.loads(self.rfile.read(n) or b'{}')
    except json.JSONDecodeError:
      return self._send(400, json.dumps({'error': '요청을 읽을 수 없습니다'}))

    key, value = req.get('key'), req.get('value')
    if key not in SETTINGS:
      return self._send(400, json.dumps({'error': f'알 수 없는 설정: {key}'}))

    if self.state.get().get('engaged') and not self.allow_engaged:
      return self._send(409, json.dumps({
        'error': 'openpilot이 제어 중일 때는 변경할 수 없습니다. 해제 후 다시 시도하세요.'}))

    kind = SETTINGS[key][0]
    try:
      # Params is typed: BOOL wants a real bool and INT a real int, not their string forms
      self.params.put(key, bool(value) if kind == 'bool' else int(value))
    except (TypeError, ValueError) as e:
      return self._send(400, json.dumps({'error': f'저장 실패: {e}'}))

    return self._send(200, json.dumps({'ok': True, 'key': key, 'value': value}))



class LazyCan:
  """CarParams only appears once the car has been identified, which is long after this server
  starts at boot. Keep retrying instead of deciding at startup that there is no car."""

  def __init__(self, params: Params):
    self.params = params
    self.decoder: CanDecoder | None = None
    self.lock = threading.Lock()

  def get(self) -> "CanDecoder | None":
    if self.decoder is None:
      with self.lock:
        if self.decoder is None:
          self.decoder = build_can_decoder(self.params)
    return self.decoder


def build_can_decoder(params: Params) -> "CanDecoder | None":
  """Decode against whatever DBCs this car actually uses."""
  raw = params.get("CarParams")
  if raw is None:
    return None
  try:
    cp = messaging.log_from_bytes(raw, car.CarParams)
    from opendbc.car.values import PLATFORMS
    platform = PLATFORMS.get(cp.carFingerprint)
    names = sorted({v for v in (platform.config.dbc_dict or {}).values() if v}) if platform else []
    return CanDecoder(names) if names else None
  except Exception:
    return None


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
.sw{position:relative;width:50px;height:29px;flex:0 0 auto;border:0;border-radius:15px;
background:var(--line);cursor:pointer;transition:background .15s}
.sw[aria-checked=true]{background:var(--ok)}
.sw::after{content:"";position:absolute;top:3px;left:3px;width:23px;height:23px;border-radius:50%;
background:#fff;transition:transform .15s}
.sw[aria-checked=true]::after{transform:translateX(21px)}
.sw:focus-visible{outline:2px solid var(--radar);outline-offset:2px}
input[type=number]{width:88px;background:var(--bg);color:var(--tx);border:1px solid var(--line);
border-radius:8px;padding:8px;font-family:var(--m);font-size:14px;text-align:right}
input:focus-visible{outline:2px solid var(--radar);outline-offset:1px}
.pill{display:inline-block;font-family:var(--m);font-size:10px;padding:3px 8px;border-radius:99px;
border:1px solid var(--line);color:var(--mut)}
.pill.on{border-color:var(--ok);color:var(--ok)}.pill.off{border-color:var(--dim)}
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

<div class="card"><div class="h">Settings</div><div id="settings"></div></div>
<div id="msg"></div>

<script>
const $=i=>document.getElementById(i);
let engaged=false, allowEngaged=false;

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

async function loadSettings(){
  const d=await(await fetch('/api/settings')).json();
  allowEngaged=d.allowEngaged;
  const box=$('settings');box.innerHTML='';
  for(const[k,c]of Object.entries(d.settings)){
    const row=document.createElement('div');row.className='row';
    const left=document.createElement('div');
    left.innerHTML=`<div class="lab">${c.label}</div><div class="hlp">${c.help}</div>`;
    row.appendChild(left);
    if(c.type==='bool'){
      const b=document.createElement('button');
      b.className='sw';b.setAttribute('role','switch');
      b.setAttribute('aria-checked',!!c.value);b.setAttribute('aria-label',c.label);
      b.onclick=()=>save(k,b.getAttribute('aria-checked')!=='true',b);
      row.appendChild(b);
    }else{
      const i=document.createElement('input');
      i.type='number';i.value=c.value??0;i.setAttribute('aria-label',c.label);
      i.onchange=()=>save(k,parseInt(i.value,10),i);
      row.appendChild(i);
    }
    box.appendChild(row);
  }
}

async function save(key,value,el){
  if(engaged&&!allowEngaged){toast('제어 중에는 변경할 수 없습니다. 해제 후 시도하세요.',1);
    loadSettings();return;}
  const r=await fetch('/api/settings',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({key,value})});
  const d=await r.json();
  if(!r.ok){toast(d.error||'저장에 실패했습니다',1);loadSettings();return;}
  if(el.classList.contains('sw'))el.setAttribute('aria-checked',!!value);
  toast('저장됨 · 약 0.5초 내 반영');
}

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
input[type=search]{flex:1;min-width:150px;background:var(--card);color:var(--tx);
border:1px solid var(--line);border-radius:9px;padding:9px 11px;font-size:14px}
input:focus-visible,button:focus-visible{outline:2px solid var(--radar);outline-offset:1px}
button.tg{background:var(--card);color:var(--mut);border:1px solid var(--line);border-radius:9px;
padding:9px 12px;font-family:var(--m);font-size:11.5px;cursor:pointer}
button.tg[aria-pressed=true]{border-color:var(--hot);color:var(--hot)}
.msg{background:var(--card);border:1px solid var(--line);border-radius:10px;margin-bottom:8px;
overflow:hidden}
.msg.hot{border-color:var(--hot)}
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
.empty{color:var(--dim);font-size:13px;padding:24px 4px;text-align:center}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<a class="back" href="/">← 메뉴</a>
<h1>CAN 신호 뷰어</h1><div class="sub" id="sub">연결 중…</div>
<div class="bar">
  <input type="search" id="q" placeholder="메시지·신호 이름 또는 주소" aria-label="검색">
  <button class="tg" id="only" aria-pressed="false">변한 것만</button>
  <button class="tg" id="pause" aria-pressed="false">일시정지</button>
</div>
<div id="list"></div>

<script>
const $=i=>document.getElementById(i);
const open=new Set(); let paused=false, onlyChanged=false;

$('only').onclick=()=>{onlyChanged=!onlyChanged;$('only').setAttribute('aria-pressed',onlyChanged);};
$('pause').onclick=()=>{paused=!paused;$('pause').setAttribute('aria-pressed',paused);};
$('q').oninput=()=>render(last);

let last={messages:[]};
function key(m){return m.bus+':'+m.address;}

function render(d){
  last=d;
  const q=$('q').value.trim().toLowerCase();
  const list=$('list');
  let msgs=d.messages||[];
  if(q) msgs=msgs.filter(m=>
    (m.name||'').toLowerCase().includes(q) ||
    String(m.address).includes(q) || m.address.toString(16).includes(q) ||
    m.signals.some(s=>s.name.toLowerCase().includes(q)));
  if(!msgs.length){list.innerHTML='<div class="empty">'+
    (d.error||(q?'검색 결과가 없습니다':'수신된 CAN 메시지가 없습니다'))+'</div>';return;}

  list.innerHTML=msgs.map(m=>{
    const k=key(m), isOpen=open.has(k);
    const sigs=isOpen&&m.signals.length?'<div class="sigs">'+m.signals.map(s=>
      `<div class="sig${s.changed?' ch':''}"><span class="n">${s.name}</span>`+
      `<span class="val">${s.v}${s.enum?`<span class="en">${s.enum}</span>`:''}</span></div>`
    ).join('')+'</div>':'';
    return `<div class="msg${m.anyChanged?' hot':''}" data-k="${k}">
      <div class="mh"><span class="addr">0x${m.address.toString(16).toUpperCase()}</span>
        <span class="nm${m.name?'':' unk'}">${m.name||'(DBC에 없음)'}</span>
        <span class="hz">bus ${m.bus} · ${m.hz}Hz</span></div>
      <div class="bytes">${m.hex.replace(/(..)/g,'$1 ').trim()}</div>${sigs}</div>`;
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
    $('sub').textContent=d.dbc?`${d.total}개 메시지 · ${d.dbc}`:(d.error||'DBC를 찾을 수 없습니다');
    render(d);
  }catch(e){$('sub').textContent='디바이스에 연결할 수 없습니다';}
}
tick();setInterval(tick,400);
</script></body></html>"""


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', type=int, default=8088)
  ap.add_argument('--allow-engaged', action='store_true',
                  help='허용 시 openpilot 제어 중에도 설정 변경 가능 (권장하지 않음)')
  args = ap.parse_args()

  Handler.state = State()
  Handler.params = Params()
  Handler.allow_engaged = args.allow_engaged
  Handler.can = LazyCan(Handler.params)

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}"
        + ("  (writes allowed while ENGAGED)" if args.allow_engaged
           else "  (writes blocked while engaged)"))
  srv.serve_forever()


if __name__ == "__main__":
  main()
