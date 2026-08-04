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

REALDATA = '/data/media/0/realdata'
SCRATCH = '/data/tmp/shadow'

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
               'e2eAccel': None, 'e2eStop': False, 'stockMin': None, 'stockMax': None}
  t0 = None
  next_t = 0.0
  frames: list[dict] = []

  # A segment's rlog opens with initData, whose logMonoTime is the route's start rather than the
  # segment's. Start the clock at the first real data frame instead.
  DATA = ('carState', 'longitudinalPlan', 'radarState', 'selfdriveState', 'modelV2')
  fingerprint = None

  for evt in _read_events(path):
    w = evt.which()
    if w == 'carParams' and fingerprint is None:
      fingerprint = str(evt.carParams.carFingerprint)
      continue
    if w not in DATA:
      continue
    t = evt.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    dt = t - t0

    if w == 'carState':
      cs = evt.carState
      cur['vEgo'], cur['aEgo'] = cs.vEgo, cs.aEgo
      cur['vCruise'] = cs.cruiseState.speed
      cur['gap'] = int(cs.cruiseState.gapAdjust)
      cur['brake'], cur['gas'] = bool(cs.brakePressed), bool(cs.gasPressed)
      # 5.44 m/s^2 (raw 511) is DAS_accelMin/Max's own SNA value, not a real request -- the
      # factory module hasn't reported yet (usually because it isn't the one driving right now).
      cur['stockMin'] = cs.stockAccelMin if cs.stockAccelMin < 5.43 else None
      cur['stockMax'] = cs.stockAccelMax if cs.stockAccelMax < 5.43 else None
    elif w == 'longitudinalPlan':
      cur['aTarget'] = evt.longitudinalPlan.aTarget
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
        'flags': ((F_LEAD if ld else 0) | (F_RADAR if ld and ld['radar'] else 0)
                  | (F_ENG if cur['eng'] else 0) | (F_BRAKE if cur['brake'] else 0)
                  | (F_GAS if cur['gas'] else 0)),
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


def resolve(frames: list[dict], accel_min: float | None = None) -> dict:
  """Solve the MPC again over the same inputs, with the code as it stands now.

  State is re-seeded from the log every frame rather than integrated forward. Letting it run
  open-loop would drift away from the recorded situation within a second or two and the two
  lines would stop being about the same moment.
  """
  # Imported here, not at module scope: this pulls in the compiled acados solver, and the tuning
  # server has to start on machines and states where that is not importable.
  from openpilot.common.realtime import DT_MDL
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    LongitudinalMpc, T_IDXS as T_IDXS_MPC)
  from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
  from openpilot.selfdrive.modeld.constants import ModelConstants

  fingerprint = frames[0].get('fingerprint') if frames else None
  CP = car_params(fingerprint)
  if accel_min is None:
    floor = float(CP.minAccel) if CP is not None and CP.minAccel < 0 else -3.5
    floor_src = f'{fingerprint} 포트값' if CP is not None and CP.minAccel < 0 else 'ISO 기본값'
  else:
    floor, floor_src = float(accel_min), '직접 지정'
  # Matches plannerd's own call: get_accel_from_plan(..., action_t=CP.longitudinalActuatorDelay
  # + DT_MDL, vEgoStopping=CP.vEgoStopping). Falling back to openpilot's stock defaults if the
  # car is unknown, rather than get_accel_from_plan's own (unrelated) 0.05/DT_MDL defaults.
  action_t = (CP.longitudinalActuatorDelay if CP is not None else 0.2) + DT_MDL
  v_ego_stopping = CP.vEgoStopping if CP is not None else 0.5
  ctrl_t = ModelConstants.T_IDXS[:CONTROL_N]

  mpc = LongitudinalMpc(accel_min=floor)
  mpc.set_weights(prev_accel_constraint=True)

  out = []
  t_start = time.monotonic()
  for f in frames:
    mpc.set_cur_state(f['vEgo'], f['aEgo'])
    mpc.update(_radarstate(f['lead']), max(f['vCruise'], 1.0), gap_adjust=f['gap'])
    v_traj = np.interp(ctrl_t, T_IDXS_MPC, mpc.v_solution)
    a_traj = np.interp(ctrl_t, T_IDXS_MPC, mpc.a_solution)
    a_mpc, stop_mpc = get_accel_from_plan(v_traj, a_traj, ctrl_t, action_t=action_t, vEgoStopping=v_ego_stopping)

    # The Experimental Mode blend from longitudinal_planner.py, applied here regardless of
    # whether the drive actually had openpilotLongitudinalControl or the toggle on: the model's
    # own action.desiredAcceleration/shouldStop were computed unconditionally, so this shows
    # what the same driving model wanted to do, mixed in the same way plannerd would mix it.
    a_e2e = f.get('e2eAccel')
    stop_e2e = bool(f.get('e2eStop'))
    if a_e2e is not None:
      a_exp = min(a_e2e, a_mpc)
      stop_exp = stop_e2e or stop_mpc
    else:
      a_exp, stop_exp = a_mpc, stop_mpc

    out.append({
      'aNew': round(float(a_mpc), 3),
      'aExp': round(float(a_exp), 3),
      'aE2e': round(float(a_e2e), 3) if a_e2e is not None else None,
      'stopMpc': bool(stop_mpc), 'stopE2e': stop_e2e, 'stopExp': bool(stop_exp),
      # a_solution[0] is the measured state we just seeded, so the constraint only shows in [1:]
      'aFloorHit': bool(np.min(mpc.a_solution[1:]) <= floor + 1e-3),
      'tFollow': round(float(mpc.t_follow), 3),
    })
  return {'rows': out, 'accelMin': floor, 'accelMinSrc': floor_src,
          'solveSec': round(time.monotonic() - t_start, 1)}


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

  def start(self, route: str, seg: int, accel_min: float | None) -> dict:
    with self._lock:
      if self._thread is not None and self._thread.is_alive():
        return {'error': '이미 실행 중입니다', **self._state}
      if self._engaged():
        return {'error': '주행 중에는 실행하지 않습니다'}
      self._state = {'status': 'running', 'route': route, 'seg': seg, 'accelMin': accel_min}
      self._thread = threading.Thread(target=self._run, args=(route, seg, accel_min), daemon=True)
      self._thread.start()
      return dict(self._state)

  def _run(self, route: str, seg: int, accel_min: float | None):
    try:
      frames = extract(route, seg)
      if not frames:
        raise ValueError('세그먼트에 데이터가 없습니다')
      solved = resolve(frames, accel_min)
      rows = []
      for f, s in zip(frames, solved['rows'], strict=False):
        ld = f['lead']
        flags = (f['flags'] | (32 if s['aFloorHit'] else 0)
                 | (64 if s['stopExp'] else 0) | (128 if s['stopMpc'] else 0))
        rows.append([
          f['t'],
          round(f['aEgo'], 3),          # 순정 ACC 실제
          round(f['aTarget'], 3),       # 주행 당시 계획
          s['aNew'],                    # 지금 코드, MPC 단독
          s['aExp'],                    # 지금 코드 + Experimental Mode 블렌드
          round(f['vEgo'], 2),
          round(ld['dRel'], 1) if ld else None,
          flags,
          f['gap'],
          s['tFollow'],
          round(f['stockMin'], 3) if f['stockMin'] is not None else None,
          round(f['stockMax'], 3) if f['stockMax'] is not None else None,
        ])
      with self._lock:
        self._state = {
          'status': 'done', 'route': route, 'seg': seg,
          'accelMin': solved['accelMin'], 'accelMinSrc': solved['accelMinSrc'],
          'solveSec': solved['solveSec'],
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
