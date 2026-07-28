#!/usr/bin/env python3
"""Live viewer and tuning switchboard, served from the device over WiFi.

Real-car A/B testing otherwise means a laptop in the passenger seat. This serves a page you can
open on a phone: current lead perception state on the left, the switches that change it on the
right, so a run can be set up and its effect watched without stopping to SSH in.

  PYTHONPATH=/data/openpilot python3 selfdrive/debug/tuning_server.py
  # then open http://<device-ip>:8088 from anything on the same network

Control settings are refused while onroad by default -- changing how the car decides to brake
mid-drive is a different risk than changing it in a parking lot. Pass --allow-onroad to lift
that; the page will say so.
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cereal.messaging as messaging
from openpilot.common.params import Params

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
  allow_onroad: bool

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
      return self._send(200, json.dumps({'settings': out, 'allowOnroad': self.allow_onroad}))

    return self._send(200, PAGE, 'text/html; charset=utf-8')

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

    if self.state.get().get('onroad') and not self.allow_onroad:
      return self._send(409, json.dumps({
        'error': '주행 중에는 변경할 수 없습니다. 정차 후 다시 시도하세요.'}))

    kind = SETTINGS[key][0]
    try:
      # Params is typed: BOOL wants a real bool and INT a real int, not their string forms
      self.params.put(key, bool(value) if kind == 'bool' else int(value))
    except (TypeError, ValueError) as e:
      return self._send(400, json.dumps({'error': f'저장 실패: {e}'}))

    return self._send(200, json.dumps({'ok': True, 'key': key, 'value': value}))


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
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
<h1>openpilot tuning</h1>
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
let onroad=false, allowOnroad=false;

function toast(t,err){const m=$('msg');m.textContent=t;m.className='show'+(err?' err':'');
  clearTimeout(m._t);m._t=setTimeout(()=>m.className='',2600);}

async function poll(){
  try{
    const s=await(await fetch('/api/state')).json();
    onroad=s.onroad;
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
  allowOnroad=d.allowOnroad;
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
  if(onroad&&!allowOnroad){toast('주행 중에는 변경할 수 없습니다. 정차 후 시도하세요.',1);
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


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', type=int, default=8088)
  ap.add_argument('--allow-onroad', action='store_true',
                  help='허용 시 주행 중에도 설정 변경 가능 (권장하지 않음)')
  args = ap.parse_args()

  Handler.state = State()
  Handler.params = Params()
  Handler.allow_onroad = args.allow_onroad

  srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
  print(f"serving on http://0.0.0.0:{args.port}"
        + ("  (onroad writes ALLOWED)" if args.allow_onroad else "  (onroad writes blocked)"))
  srv.serve_forever()


if __name__ == "__main__":
  main()
