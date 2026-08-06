"""Re-run a recorded drive through the longitudinal planner that is on the device right now.

A drive done on the factory ACC already carries a matched pair at every instant: what the car did
(carState.aEgo) and what openpilot's planner asked for at the time (longitudinalPlan.aTarget).
Reading both back is useful, but it only ever shows the code that was running during the drive.

This solves the MPC again, here, with whatever the source says today. Edit a constant -- the
braking floor, the gap table, a cost -- and the recomputed line moves while the two recorded ones
stay put, so a change can be judged against a real closure instead of a guess. That is the reason
this lives on the device: the acados solver is built here, and only here is "the current code"
the same thing the car would run.

Not free: a 60s segment is ~1200 solves plus decompressing 12MB of log. It runs in a worker
thread, one at a time, and refuses to start while openpilot is engaged -- the car comes first.
"""

import os
import threading
import time

import capnp
import numpy as np
import zstandard

from cereal import log as capnp_log, messaging
from openpilot.common.swaglog import cloudlog
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

REALDATA = '/data/media/0/realdata'
SCRATCH = '/data/tmp/shadow'

# The handful of Tesla ADAS/map messages the /shadow page's "풀기" table decodes alongside the
# usual planner columns. Names are the DBC message names in tesla_can.dbc.
ADAS_DBC = 'tesla_can'
ADAS_ADDRS = (0x3E8, 0x3C8, 0x238, 0x2C8, 0x2E8, 0x2D8, 0x2B8, 0x399, 0x2F8)

# UI_driverAssistRoadSign (0x238) multiplexes unrelated map data onto the same bits via
# UI_roadSign -- decoding every signal unconditionally (like the other 6 messages) would read
# whichever mux group isn't active as garbage. Only the fields for the current mux value get
# decoded; the rest are simply absent from that frame's snapshot.
ROAD_SIGN_COMMON = ('UI_roadSign', 'UI_splineLocConfidence', 'UI_splineID')
ROAD_SIGN_MUX = {
  0: ('UI_dummyData',),
  1: ('UI_stopSignStopLineDist', 'UI_stopSignStopLineConf'),
  2: ('UI_trafficLightStopLineDist', 'UI_trafficLightStopLineConf'),
  3: ('UI_baseMapSpeedLimitMPS', 'UI_bottomQrtlFleetSpeedMPS', 'UI_topQrtlFleetSpeedMPS'),
  4: ('UI_meanFleetSplineSpeedMPS', 'UI_medianFleetSpeedMPS', 'UI_meanFleetSplineAccelMPS2', 'UI_rampType'),
  5: ('UI_currSplineIdFull',),
}


class _AdasDecoder:
  """Latest known value of each ADAS/map signal, decoded from raw `can` events as they arrive.

  Not a live view of the bus -- a held-value snapshot, same as the rest of extract()'s `cur`
  dict, so a message sent slower than 20Hz still has a value at every sampled frame.
  """

  def __init__(self):
    try:
      dbc = DBC(ADAS_DBC)
      self.msgs = {addr: dbc.addr_to_msg[addr] for addr in ADAS_ADDRS if addr in dbc.addr_to_msg}
    except Exception:
      cloudlog.exception('shadow: ADAS dbc load failed')
      self.msgs = {}
    self.last: dict[int, dict] = {}

  @staticmethod
  def _value(sig, dat: bytes) -> float:
    raw = get_raw_value(dat, sig)
    if sig.is_signed:
      raw -= ((raw >> (sig.size - 1)) & 0x1) * (1 << sig.size)
    return round(raw * sig.factor + sig.offset, 6)

  def ingest(self, frames) -> None:
    for frame in frames:
      msg = self.msgs.get(frame.address)
      if msg is None:
        continue
      dat = bytes(frame.dat)
      if len(dat) < msg.size:
        continue
      if frame.address == 0x238:
        out = {name: self._value(msg.sigs[name], dat) for name in ROAD_SIGN_COMMON if name in msg.sigs}
        for name in ROAD_SIGN_MUX.get(int(out.get('UI_roadSign', -1)), ()):
          if name in msg.sigs:
            out[name] = self._value(msg.sigs[name], dat)
        self.last[frame.address] = out
      else:
        self.last[frame.address] = {name: self._value(sig, dat) for name, sig in msg.sigs.items()}

  def snapshot(self) -> dict:
    return dict(self.last)

# Sample grid for the returned series. longitudinalPlan runs at 20Hz, so this neither invents
# detail nor throws any away; carState (100Hz) is held between samples.
HZ = 20
DT = 1.0 / HZ

# Only frames with a lead say anything about following. Without one the planner is tracking set
# speed, where a disagreement means nothing.
F_LEAD, F_RADAR, F_ENG, F_BRAKE, F_GAS = 1, 2, 4, 8, 16

# How much more braking openpilot has to ask for, against the plan the drive actually ran with,
# before it counts as a real disagreement rather than the two controllers tracking each other.
DISAGREE_MS2 = 0.5


def _seg_dir(route: str, seg: int) -> str:
  return os.path.join(REALDATA, f'{route}--{seg}')


def _read_events(path: str):
  os.makedirs(SCRATCH, exist_ok=True)
  tmp = os.path.join(SCRATCH, f'sr-{os.getpid()}.rlog')
  try:
    with open(path, 'rb') as src, open(tmp, 'wb') as dst:
      zstandard.ZstdDecompressor().copy_stream(src, dst)
    # Parse from bytes: a segment cut short by the car losing power leaves a truncated final
    # message, and the streaming reader aborts on it inside libkj -- a C++ terminate no Python
    # except can catch. From bytes it raises normally, after yielding every complete event.
    with open(tmp, 'rb') as f:
      data = f.read()
    try:
      yield from capnp_log.Event.read_multiple_bytes(data)
    except capnp.KjException:
      pass
  finally:
    if os.path.exists(tmp):
      os.remove(tmp)


def extract(route: str, seg: int) -> list[dict]:
  """One frame per sample: what the car saw, and what it did about it."""
  path = os.path.join(_seg_dir(route, seg), 'rlog.zst')
  if not os.path.isfile(path):
    raise FileNotFoundError(path)

  cur: dict = {'vEgo': 0.0, 'aEgo': 0.0, 'vCruise': 0.0, 'gap': 0, 'aTarget': 0.0,
               'brake': False, 'gas': False, 'eng': False, 'lead': None,
               'e2eAccel': None, 'e2eStop': False, 'stockMin': None, 'stockMax': None,
               'standstill': False, 'plannerSource': 'stock', 'cruiseEnabled': False}
  t0 = None
  next_t = 0.0
  frames: list[dict] = []

  # t=0 has to be the video's first frame, because the chart is read against the video. That is
  # roadEncodeIdx with segmentId 0 -- the frame the segment's camera file actually starts on.
  # Anchoring on the first carState instead is only right when the two happen to coincide: on
  # the first segment of a route the cameras are recording seconds before the car is fingerprinted
  # and carState appears, which slid the whole chart against the video by that much.
  DATA = ('carState', 'longitudinalPlan', 'radarState', 'selfdriveState', 'modelV2')
  fingerprint = None
  adas = _AdasDecoder()

  for evt in _read_events(path):
    w = evt.which()
    if w == 'carParams' and fingerprint is None:
      fingerprint = str(evt.carParams.carFingerprint)
      continue
    if w == 'roadEncodeIdx' and t0 is None and evt.roadEncodeIdx.segmentId == 0:
      t0 = evt.logMonoTime / 1e9
      continue
    if w == 'can':
      adas.ingest(evt.can)
      continue
    if w not in DATA:
      continue
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if dt < 0:
      continue          # data that predates the segment's first video frame

    if w == 'carState':
      cs = evt.carState
      cur['vEgo'], cur['aEgo'] = cs.vEgo, cs.aEgo
      cur['vCruise'] = cs.cruiseState.speed
      cur['cruiseEnabled'] = bool(cs.cruiseState.enabled)
      cur['gap'] = int(cs.cruiseState.gapAdjust)
      cur['brake'], cur['gas'] = bool(cs.brakePressed), bool(cs.gasPressed)
      cur['standstill'] = bool(cs.standstill)
      # 5.44 m/s^2 (raw 511) is DAS_accelMin/Max's own SNA value, not a real request -- the
      # factory module hasn't reported yet (usually because it isn't the one driving right now).
      cur['stockMin'] = cs.stockAccelMin if cs.stockAccelMin < 5.43 else None
      cur['stockMax'] = cs.stockAccelMax if cs.stockAccelMax < 5.43 else None
    elif w == 'longitudinalPlan':
      cur['aTarget'] = evt.longitudinalPlan.aTarget
      # Which planner recorded that aTarget. Without it the grey line is unlabelled and the
      # comparison silently changes meaning between drives.
      cur['plannerSource'] = str(evt.longitudinalPlan.plannerSource)
    elif w == 'radarState':
      ld = evt.radarState.leadOne
      cur['lead'] = {
        'status': bool(ld.status), 'dRel': ld.dRel, 'vRel': ld.vRel,
        'vLead': ld.vLead, 'vLeadK': ld.vLeadK, 'aLeadK': ld.aLeadK,
        'aLeadTau': ld.aLeadTau, 'modelProb': ld.modelProb, 'radar': bool(ld.radar),
      } if ld.status else None
    elif w == 'selfdriveState':
      cur['eng'] = bool(evt.selfdriveState.enabled)
    elif w == 'modelV2':
      # The driving model's own end-to-end action -- what plannerd blends in only when
      # Experimental Mode is on and openpilot owns longitudinal. Recorded regardless of whether
      # either was true on this drive, since the model computes it unconditionally; a re-solve
      # can turn the blend on for a drive that never had it, using the model's real output.
      act = evt.modelV2.action
      cur['e2eAccel'] = float(act.desiredAcceleration)
      cur['e2eStop'] = bool(act.shouldStop)

    while dt >= next_t:
      ld = cur['lead']
      frames.append({
        't': round(next_t, 2),
        'vEgo': cur['vEgo'], 'aEgo': cur['aEgo'], 'vCruise': cur['vCruise'],
        'gap': cur['gap'], 'aTarget': cur['aTarget'],
        'lead': ld, 'e2eAccel': cur['e2eAccel'], 'e2eStop': cur['e2eStop'],
        'stockMin': cur['stockMin'], 'stockMax': cur['stockMax'],
        'standstill': cur['standstill'],
        'plannerSource': cur['plannerSource'],
        'cruiseEnabled': cur['cruiseEnabled'],
        'flags': ((F_LEAD if ld else 0) | (F_RADAR if ld and ld['radar'] else 0)
                  | (F_ENG if cur['eng'] else 0) | (F_BRAKE if cur['brake'] else 0)
                  | (F_GAS if cur['gas'] else 0)),
        'adas': adas.snapshot(),
      })
      next_t += DT

  for f in frames:
    f['fingerprint'] = fingerprint
  return frames


def _radarstate(ld):
  rs = messaging.new_message('radarState').radarState
  if ld:
    o = rs.leadOne
    o.status = True
    o.dRel, o.vRel = ld['dRel'], ld['vRel']
    o.vLead, o.vLeadK = ld['vLead'], ld['vLeadK']
    o.aLeadK, o.aLeadTau = ld['aLeadK'], ld['aLeadTau']
    o.modelProb = ld['modelProb']
  rs.leadTwo.status = False
  return rs


def summarise(frames: list[dict]) -> dict:
  """How far apart the two controllers were in this segment, using the plan the drive already
  logged -- not a re-solve. This is what makes a scan across a whole route cheap enough to run
  on every segment: no acados solve, just the aTarget that was already computed at drive time.
  """
  worst = 0.0
  worst_t = None
  disagree = 0
  lead_frames = 0
  for f in frames:
    if not f['lead']:
      continue
    lead_frames += 1
    gap = f['aTarget'] - f['aEgo']       # negative = openpilot wanted more braking than the car did
    if gap < -DISAGREE_MS2:
      disagree += 1
    if gap < worst:
      worst, worst_t = gap, f['t']
  return {
    'dur': frames[-1]['t'] if frames else 0.0,
    'leadFrames': lead_frames,
    'disagreeFrames': disagree,
    'disagreePct': round(100.0 * disagree / lead_frames, 1) if lead_frames else 0.0,
    'worst': round(worst, 3), 'worstT': worst_t,
  }


def scan_route(route: str, on_progress=None) -> list[dict]:
  """Every segment of a route, ranked by how far the recorded plan and the recorded car parted.

  Cheap on purpose: this reads what was already logged rather than re-solving anything, so it is
  the triage pass -- picking which segment is worth the cost of a real replay, not a replacement
  for one. The MPC re-solve only ever runs on the one segment picked afterwards.
  """
  out = []
  segs = list_segments(route)
  for i, seg in enumerate(segs):
    if on_progress:
      on_progress(i, len(segs))
    try:
      frames = extract(route, seg)
    except Exception:
      cloudlog.exception(f'shadow: scan failed on {route}--{seg}')
      continue
    if not frames:
      continue
    out.append({'seg': seg, **summarise(frames)})
  out.sort(key=lambda s: s['worst'])
  return out


def car_params(fingerprint: str | None):
  """The car port's CarParams for today's source, not the recording's own.

  The point of re-solving is to see the effect of editing the port -- longitudinalActuatorDelay,
  vEgoStopping, minAccel -- so these come from interfaces[], not from the log's CarParams, which
  would never move no matter what changed.
  """
  if not fingerprint:
    return None
  try:
    from opendbc.car.car_helpers import interfaces
    return interfaces[fingerprint].get_non_essential_params(fingerprint)
  except Exception:
    cloudlog.exception('shadow: car params lookup failed')
    return None


def car_floor(fingerprint: str | None) -> tuple[float, str]:
  """What today's source gives this car, not what the drive ran with."""
  from opendbc.car.interfaces import ACCEL_MIN
  CP = car_params(fingerprint)
  if CP is not None and CP.minAccel < 0:
    return float(CP.minAccel), f'{fingerprint} 포트값'
  return ACCEL_MIN, ('ISO 기본값' if fingerprint else 'ISO 기본값 (차종 미상)')


class _ReplaySM:
  """The parts of SubMaster a planner touches, backed by log messages instead of sockets."""

  def __init__(self):
    self.data: dict = {}
    self.logMonoTime: dict = {}
    self.updated: dict = {}
    self.recv_frame: dict = {}
    self.seen: dict = {}
    self.alive: dict = {}
    self.frame = 0

  def __getitem__(self, key):
    return self.data[key]

  def __contains__(self, key):
    return key in self.data

  def all_checks(self, *a, **k):
    return True

  def valid(self, *a, **k):
    return True


def resolve_carrot(route: str, seg: int) -> list[dict] | None:
  """CarrotPilot's planner over the same segment, on the same time grid.

  Re-reads the log rather than working from extract()'s frames: carrot's planner reads modelV2
  in full -- lane lines, position, the stop-line estimate -- and reducing that to a handful of
  scalars would mean comparing against something carrot does not actually run. Returns None if
  the port is not importable, which is the normal state on a machine without its solver built.
  """
  try:
    from openpilot.selfdrive.controls.lib.longitudinal_planner_carrot import CarrotLongitudinalPlanner
  except Exception:
    cloudlog.exception('shadow: carrot planner unavailable')
    return None

  path = os.path.join(_seg_dir(route, seg), 'rlog.zst')
  if not os.path.isfile(path):
    return None

  NEEDED = ('carControl', 'carState', 'controlsState', 'radarState', 'modelV2',
            'selfdriveState', 'liveParameters')
  CP = None
  planner = None
  sm = _ReplaySM()
  t0 = None
  next_t = 0.0
  out: list[dict] = []
  cur = {'a': 0.0, 'tFollow': 0.0, 'xState': 0, 'stop': False}

  for evt in _read_events(path):
    w = evt.which()
    if w == 'carParams' and CP is None:
      CP = evt.carParams
      continue
    if w == 'roadEncodeIdx' and t0 is None and evt.roadEncodeIdx.segmentId == 0:
      t0 = evt.logMonoTime / 1e9
      continue
    if w in NEEDED:
      sm.data[w] = getattr(evt, w)
      sm.logMonoTime[w] = evt.logMonoTime
      sm.seen[w] = sm.alive[w] = True
      sm.recv_frame[w] = sm.frame
    if w != 'modelV2' or CP is None or len(sm.data) < len(NEEDED):
      continue

    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0
    if dt < 0:
      continue

    if planner is None:
      planner = CarrotLongitudinalPlanner(CP)
    sm.frame += 1
    sm.updated = dict.fromkeys(sm.data, True)
    try:
      planner.update(sm)
      cur = {
        'a': float(planner.planner.output_a_target),
        'tFollow': float(planner.planner.mpc.t_follow),
        'xState': int(planner.carrot.xState.value),
        'stop': bool(planner.planner.output_should_stop),
      }
    except Exception:
      cloudlog.exception('shadow: carrot planner failed mid-segment')
      return None

    while dt >= next_t:
      out.append({'t': round(next_t, 2), 'aCarrot': round(cur['a'], 3),
                  'tFollowCarrot': round(cur['tFollow'], 3),
                  'xState': cur['xState'], 'stopCarrot': cur['stop']})
      next_t += DT
  return out or None


class ShadowReplay:
  """One replay at a time, in the background, never while the car is being driven."""

  def __init__(self, engaged_fn):
    self._engaged = engaged_fn
    self._lock = threading.Lock()
    self._thread: threading.Thread | None = None
    self._state: dict = {'status': 'idle'}

  def state(self) -> dict:
    with self._lock:
      return dict(self._state)

  def start(self, route: str, seg: int, solve: bool = True) -> dict:
    """solve=False stops after extract, which is the cheap half.

    Reading the log costs a few seconds; CarrotPilot's re-solve costs several times that --
    it is the only re-solve this page does now, since the stock planner it used to also replay
    was retired once nothing in the car ran it anymore. Selecting a segment only needs what the
    drive already recorded, so it asks for that and stops.
    """
    with self._lock:
      if self._thread is not None and self._thread.is_alive():
        return {'error': '이미 실행 중입니다', **self._state}
      if self._engaged():
        return {'error': '주행 중에는 실행하지 않습니다'}
      self._state = {'status': 'running', 'route': route, 'seg': seg}
      self._thread = threading.Thread(target=self._run, args=(route, seg, solve), daemon=True)
      self._thread.start()
      return dict(self._state)

  def _run(self, route: str, seg: int, solve: bool = True):
    try:
      frames = extract(route, seg)
      if not frames:
        raise ValueError('세그먼트에 데이터가 없습니다')
      # aEgo/aTarget/vEgo are already in the log -- extract() is a decompress and a capnp parse,
      # not a solve. The video and the recorded lines can be on screen while CarrotPilot's
      # re-solve is still running instead of waiting behind it, so publish them the moment
      # they're ready. Columns 3/4/9 are the retired stock re-solve's (aNew, aExp, tFollow) --
      # left in the row shape rather than renumbered, so they stay None forever now. Column 15
      # is the ADAS/map signal snapshot (see _AdasDecoder), 16 is cruiseState.speed (m/s), 17 is
      # cruiseState.enabled -- all available immediately, no re-solve.
      planner_source = frames[0].get('plannerSource', 'stock') if frames else 'stock'
      floor, floor_src = car_floor(frames[0].get('fingerprint') if frames else None)
      rows = []
      for f in frames:
        ld = f['lead']
        rows.append([
          f['t'], round(f['aEgo'], 3), round(f['aTarget'], 3), None, None,
          round(f['vEgo'], 2), round(ld['dRel'], 1) if ld else None, f['flags'],
          f['gap'], None,
          round(f['stockMin'], 3) if f['stockMin'] is not None else None,
          round(f['stockMax'], 3) if f['stockMax'] is not None else None,
          None, None, None,
          f.get('adas', {}),
          round(f['vCruise'], 2), bool(f.get('cruiseEnabled')),
        ])
      with self._lock:
        if self._state.get('status') == 'running':   # a newer request has not already superseded this one
          self._state = {'status': 'partial', 'route': route, 'seg': seg, 'rows': rows,
                          'recordedPlanner': planner_source, 'accelMin': floor, 'accelMinSrc': floor_src}

      if not solve:
        with self._lock:
          self._state = {'status': 'done', 'route': route, 'seg': seg, 'rows': rows,
                         'recordedPlanner': planner_source, 'accelMin': floor, 'accelMinSrc': floor_src,
                         'solved': False, 'solveSec': 0.0, 'hasCarrot': False}
        return

      t_start = time.monotonic()
      # CarrotPilot's planner over the same segment, if its port is built here. Optional on
      # purpose: the page is still useful without it, and a missing solver must not take the
      # whole replay down.
      carrot = resolve_carrot(route, seg)
      carrot_by_t = {c['t']: c for c in carrot} if carrot else {}
      for row in rows:
        c = carrot_by_t.get(row[0])
        if c:
          row[12], row[13], row[14] = c.get('aCarrot'), c.get('tFollowCarrot'), c.get('xState')
      with self._lock:
        self._state = {
          'status': 'done', 'route': route, 'seg': seg,
          'accelMin': floor, 'accelMinSrc': floor_src,
          'solveSec': round(time.monotonic() - t_start, 1),
          'solved': True, 'hasCarrot': bool(carrot),
          'recordedPlanner': planner_source,
          'rows': rows,
        }
    except Exception as e:                                  # noqa: BLE001 - surfaced to the page
      cloudlog.exception('shadow replay failed')
      with self._lock:
        self._state = {'status': 'error', 'error': f'{type(e).__name__}: {e}'}


def route_floor(route: str) -> dict:
  """The floor today's source gives the car in this recording, for pre-filling the form.

  Reads carParams out of qlog rather than rlog: 400KB against 12MB, and carParams is written
  near the start of both, so paying the rlog cost just to name the car would be silly.
  """
  segs = list_segments(route)
  fingerprint = None
  for seg in segs[:3]:                      # the first segment is occasionally short or truncated
    path = os.path.join(_seg_dir(route, seg), 'qlog.zst')
    if not os.path.isfile(path):
      continue
    try:
      for evt in _read_events(path):
        if evt.which() == 'carParams':
          fingerprint = str(evt.carParams.carFingerprint)
          break
    except Exception:
      cloudlog.exception('shadow: qlog read failed')
    if fingerprint:
      break
  floor, src = car_floor(fingerprint)
  return {'floor': round(floor, 3), 'floorSrc': src, 'fingerprint': fingerprint}


class ShadowScan:
  """One route scan at a time, in the background, never while the car is being driven.

  Separate from ShadowReplay rather than sharing its worker: a scan touches every segment of a
  route in sequence, and letting it share one slot with a single-segment replay would make each
  block the other for no reason -- they answer different questions and are usually run together
  (scan to find the segment, replay to look closely at it).
  """

  def __init__(self, engaged_fn):
    self._engaged = engaged_fn
    self._lock = threading.Lock()
    self._thread: threading.Thread | None = None
    self._state: dict = {'status': 'idle'}

  def state(self) -> dict:
    with self._lock:
      return dict(self._state)

  def start(self, route: str) -> dict:
    with self._lock:
      if self._thread is not None and self._thread.is_alive():
        return {'error': '이미 스캔 중입니다', **self._state}
      if self._engaged():
        return {'error': '주행 중에는 실행하지 않습니다'}
      self._state = {'status': 'running', 'route': route, 'done': 0, 'total': len(list_segments(route))}
      self._thread = threading.Thread(target=self._run, args=(route,), daemon=True)
      self._thread.start()
      return dict(self._state)

  def _run(self, route: str):
    def progress(i, total):
      with self._lock:
        if self._state.get('status') == 'running':
          self._state['done'], self._state['total'] = i, total
    try:
      segments = scan_route(route, on_progress=progress)
      with self._lock:
        self._state = {'status': 'done', 'route': route, 'segments': segments}
    except Exception as e:                                  # noqa: BLE001 - surfaced to the page
      cloudlog.exception('shadow scan failed')
      with self._lock:
        self._state = {'status': 'error', 'error': f'{type(e).__name__}: {e}'}


def list_segments(route: str) -> list[int]:
  if not os.path.isdir(REALDATA):
    return []
  out = []
  for entry in os.listdir(REALDATA):
    name, _, num = entry.rpartition('--')
    if name == route and num.isdigit() and os.path.isfile(os.path.join(REALDATA, entry, 'rlog.zst')):
      out.append(int(num))
  return sorted(out)
