"""Closed-loop lateral replay with the real driving model.

The car in the simulation is allowed to leave the logged trajectory by a lateral offset e (m, + right)
and a heading psi (rad, + right). The camera frames are not re-rendered: the model's own warp matrix
is composed with the ground-plane homography for that pose, which is exact for the road surface.

  latsim.py <segment> [HOLD=0|1] [CLOSED=0|1] [TCLOSE=s] [EINIT=m] [E0=m] [PSI0=rad] [T0=s] [T1=s] [OUT=file.npz]

Needs, outside the repo: op-video/<segment>/{fcamera,ecamera}.hevc from the device, and in LATSIM_DIR
the model compiled for this CPU (compile_modeld.py --model-size 512x256 --camera-resolutions 1344x760
--frame-skip 4 on the device's driving_supercombo.onnx, DEV=CPU:LLVM) plus a clang for tinygrad.

Checked on 0000026b seg 8 (115 km/h, clean lines): open loop matches the log (corr 0.99); the drive
replayed closed-loop stays within 0.12 m of the logged path for 27 s; with the curve hold off the car
sits 0.24 m inside the bend (road data: 0.22); a 0.4 m step on a straight is back to 0.16 m after
1.5 s. It walks away where the lines are untidy (splits, merges), and no desire is fed, so keep lane
changes out of the window and allow 3 s of warm-up.
"""
import os, sys, bisect
S = os.environ.get('LATSIM_DIR', os.path.expanduser('~/.cache/latsim'))   # driving_tinygrad.pkl and clang/ live here
sys.path.insert(0, '/home/compiler/openpilot'); sys.path.insert(1, '/home/compiler/openpilot/tinygrad_repo')
import numpy as np

CC = os.environ.get('CC', f'{S}/clang/usr/lib/llvm-18/bin/clang')


def _heavy():
  """The model runtime, imported on first use so that reading metrics costs nothing."""
  global log, messaging, LogReader, FrameReader, DT_CTRL, DEVICE_CAMERAS, view_frame_from_device_frame, calib_from_medmodel, calib_from_sbigmodel, rot_from_euler, get_nv12_info, clip_curvature, get_lat_smooth_seconds_dynamic, LaneCenteringController, M, fill_model_msg, PublishState, ModelConstants
  if 'M' in globals(): return
  os.environ.setdefault('DEV', 'CPU:LLVM')
  os.environ['CC'] = CC
  from openpilot.cereal import log, messaging
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.tools.lib.framereader import FrameReader
  from openpilot.common.realtime import DT_CTRL
  from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
  from openpilot.common.transformations.model import calib_from_medmodel, calib_from_sbigmodel
  from openpilot.common.transformations.orientation import rot_from_euler
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lat_smooth_seconds_dynamic
  from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
  from openpilot.selfdrive.modeld import modeld as M
  from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, PublishState
  from openpilot.selfdrive.modeld.constants import ModelConstants
  M.modeld_pkl_path = lambda usbgpu: f'{S}/driving_tinygrad.pkl'
  M.open_file_chunked = lambda p: open(p, 'rb')
  M.get_tg_input_devices = lambda name, usbgpu: {'WARP_DEV': 'CPU', 'QUEUE_DEV': 'CPU'}


ACT_DELAY, ACT_TAU = 0.20, 0.08       # measured: actual curvature follows the command 0.2-0.25 s late
FRAME_AGE = 0.05                      # the frame a model output belongs to was taken this long before


class Buf:
  def __init__(self, n): self.data = np.zeros(n, dtype=np.uint8)


def pack_nv12(dst, img, w, h):
  stride, y_height, _, _ = get_nv12_info(w, h)
  uv = stride * y_height
  d = dst.data
  d[:h * stride].reshape(h, stride)[:, :w] = img[:h * w].reshape(h, w)
  d[uv:uv + (h // 2) * stride].reshape(h // 2, stride)[:, :w] = img[h * w:h * w + (h // 2) * w].reshape(h // 2, w)


def warp(rpy, intrinsics, big, e, psi, height):
  """get_warp_matrix for a camera moved e to the right and turned psi to the right."""
  calib_from_model = calib_from_sbigmodel if big else calib_from_medmodel
  camera_from_calib = intrinsics @ view_frame_from_device_frame @ rot_from_euler(rpy)
  R = rot_from_euler([0.0, 0.0, psi])
  t = np.array([0.0, e, 0.0])
  H = (np.eye(3) + np.outer(t, [0.0, 0.0, 1.0]) / height) @ R     # calib frame: x forward, y right, z down
  return (camera_from_calib @ H @ calib_from_model).astype(np.float32)


LOGS = os.environ.get('OP_LOG_ROOT', '/home/compiler/openpilot/op-logs')
VIDEO = os.environ.get('OP_VIDEO_ROOT', '/home/compiler/openpilot/op-video')
COLS = ('t', 'v', 'dc_log', 'dc', 'lane_log', 'lane', 'e', 'psi', 'cmd', 'cmd_log', 'k_log', 'dk', 'hold', 'trim', 'act', 'closed', 'aim')


def available(seg=None):
  """What the replay needs and the repo does not carry."""
  ok = os.path.isfile(f'{S}/driving_tinygrad.pkl') and os.path.isfile(CC)
  if seg is not None:
    ok = ok and all(os.path.isfile(f'{VIDEO}/{seg}/{c}.hevc') for c in ('fcamera', 'ecamera')) and os.path.isfile(f'{LOGS}/{seg}/rlog.zst')
  return ok


def load(seg):
  _heavy()
  rl = f'{LOGS}/{seg}/rlog.zst'
  cs, ctl, mdl = [], [], []
  enc = {'img': {}, 'big_img': {}}
  st = dict(hold_driven=False, act=False, blk=False, v=0.0, roll=0.0, delay=0.3, rpy=None, height=1.37, dev='mici', sensor='os04c10')
  for m in LogReader(rl):
    w = m.which(); t = m.logMonoTime * 1e-9
    if w == 'initData':
      for kv in m.initData.params.entries:
        if kv.key == 'LaneCenteringCurveHold': st['hold_driven'] = bytes(kv.value) == b'1'
    elif w == 'carState':
      c = m.carState; st['v'] = c.vEgo; st['blk'] = bool(c.leftBlinker or c.rightBlinker)
    elif w == 'carControl': st['act'] = bool(m.carControl.latActive)
    elif w == 'lateralDelay': st['delay'] = m.lateralDelay.lateralDelay
    elif w == 'extrinsicsCalibration':
      st['rpy'] = np.array(m.extrinsicsCalibration.rpyCalib, dtype=np.float32)
      if len(m.extrinsicsCalibration.height): st['height'] = float(m.extrinsicsCalibration.height[0])
    elif w in ('vehicleParameters', 'liveParameters'): st['roll'] = getattr(m, w).roll
    elif w == 'roadEncodeIdx': enc['img'][m.roadEncodeIdx.frameId] = m.roadEncodeIdx.segmentId
    elif w == 'wideRoadEncodeIdx': enc['big_img'][m.wideRoadEncodeIdx.frameId] = m.wideRoadEncodeIdx.segmentId
    elif w == 'controlsState':
      c = m.controlsState; ctl.append((t, c.curvature, c.desiredCurvature, st['v'], st['act'], st['blk'], st['roll']))
    elif w == 'modelV2' and st['rpy'] is not None:
      md = m.modelV2
      mdl.append(dict(t=t, fid=md.frameId, fidx=md.frameIdExtra, dc=md.action.desiredCurvature, v=st['v'], act=st['act'],
                      delay=st['delay'], rpy=st['rpy'].copy(), height=st['height'], eof=md.timestampEof,
                      lc=0.5 * (md.laneLines[1].y[0] + md.laneLines[2].y[0]),
                      lcs=str(md.meta.laneChangeState)))
  st['enc'] = enc
  return np.array(ctl), mdl, st


def run(seg, hold=None, closed=True, tclose=0.0, t0=0.0, t1=60.0, einit=0.0, e0=0.0, psi0=0.0, tune=None, aim=False):
  """Replay one window. hold=None uses what the drive had; aim runs lane_aim.LaneAim (the
  drives in the set had none); tune overrides lane_centering / lane_aim constants by name.
  Returns a dict of arrays (COLS), one row per model frame."""
  segn = int(seg.split('--')[-1])
  ctl, mdl, st = load(seg)
  if hold is None: hold = st['hold_driven']
  if tune:
    from openpilot.selfdrive.controls.lib import lane_centering, lane_aim
    for name, value in tune.items():
      mod = lane_aim if name.startswith('AIM_') else lane_centering
      assert hasattr(mod, name), name
      setattr(mod, name, value)
  tb = mdl[0]['t']
  vd = f'{VIDEO}/{seg}'
  fr = {'img': FrameReader(f'{vd}/fcamera.hevc', pix_fmt='nv12'), 'big_img': FrameReader(f'{vd}/ecamera.hevc', pix_fmt='nv12')}
  w, h = fr['img'].w, fr['img'].h
  dc = DEVICE_CAMERAS[(st['dev'], st['sensor'])]
  intr = {'img': dc.narrow_road.intrinsics, 'big_img': dc.wide_road.intrinsics}
  size = get_nv12_info(w, h)[3]
  bufs = {k: Buf(size) for k in fr}
  model = M.ModelState(w, h, False)
  ps = PublishState(); prev_action = log.ModelDataV2.Action()
  lcc = LaneCenteringController()
  lat_smooth_dynamic = 0.0
  # the car, at 100 Hz on the controlsState time base
  e, psi = 0.0, 0.0
  placed = False
  pose_hist = []                                   # (t, e, psi)
  n_delay = int(round(ACT_DELAY / DT_CTRL)); dq = [0.0] * n_delay; lag = 0.0
  desired = None
  ci = bisect.bisect_left(ctl[:, 0], tb + t0)
  from openpilot.selfdrive.controls.lib.lane_aim import LaneAim
  la = LaneAim()                                   # aim: the model is shown `bias` m to the right of where the car is
  bias = 0.0
  rows = []
  for k, mm in enumerate(mdl):
    ts = mm['t'] - tb
    if ts < t0: continue
    if ts > t1: break
    is_closed = closed and ts >= tclose
    if is_closed and not placed:
      e += einit; placed = True
    # the pose the frame was taken from
    if is_closed and pose_hist:
      tq = mm['t'] - FRAME_AGE
      recent = pose_hist[-40:]
      j = max(0, bisect.bisect_right([p[0] for p in recent], tq) - 1)
      _, ef, pf = recent[j]
    else:
      ef, pf = 0.0, 0.0
    ef += e0 + bias; pf += psi0
    for key in fr:
      fid = mm['fid'] if key == 'img' else mm['fidx']
      idx = st['enc'][key].get(fid, fid - 1200 * segn)
      pack_nv12(bufs[key], fr[key].get(int(np.clip(idx, 0, fr[key].frame_count - 1))), w, h)
    tf = {key: warp(mm['rpy'], intr[key], key == 'big_img', ef, pf, mm['height']) for key in fr}
    v = max(mm['v'], 0.0)
    lat_action_t = mm['delay'] + lat_smooth_dynamic + 0.05 + 0.025
    inputs = {'desire_pulse': np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32),
              'traffic_convention': np.array([1.0, 0.0]),
              'action_t': np.array([lat_action_t, 0.3 + 0.3 + 0.075], dtype=np.float32)}
    out = model.run(bufs, tf, inputs)
    lat_smooth_dynamic, _ = get_lat_smooth_seconds_dynamic(out, 0.13)
    if not mm['act']:
      prev_action.desiredCurvature = float(ctl[min(ci, len(ctl) - 1), 1])
    action = M.get_action_from_model(out, prev_action, lat_action_t, 0.675, v, lat_smooth_dynamic)
    prev_action = action
    msg = messaging.new_message('modelV2')
    fill_model_msg(msg, out, action, ps, mm['fid'], mm['fidx'], mm['fid'], 0.0, mm['eof'], 0.0, True)
    mv2 = msg.modelV2
    # controller and car until the next model output
    t_next = mdl[k + 1]['t'] if k + 1 < len(mdl) else mm['t'] + 0.05
    while ci < len(ctl) and ctl[ci, 0] < t_next:
      tc, k_act_log, k_cmd_log, vc, act, blk, roll = ctl[ci]
      new = lcc.update(mv2.action.desiredCurvature, mv2, vc, True, 0.0, 1.0, bool(act), True, True, bool(blk), hold)
      if desired is None: desired = k_cmd_log
      desired, _ = clip_curvature(vc, desired, new, roll)
      if not act: desired = k_cmd_log
      if is_closed:
        dq.append(desired - k_cmd_log); d = dq.pop(0)
        lag += (d - lag) * DT_CTRL / ACT_TAU
        psi += vc * lag * DT_CTRL
        e += vc * psi * DT_CTRL
      pose_hist.append((tc, e, psi))
      ci += 1
    lc_sim = 0.5 * (mv2.laneLines[1].y[0] + mv2.laneLines[2].y[0])
    lc_sim += bias                                 # where the centre really is: what was reported, less what was shown
    if aim and is_closed:
      bias = la.update(mv2.laneLines[1].y[0], mv2.laneLines[2].y[0], mv2.laneLineProbs[1], mv2.laneLineProbs[2], v, True,
                       bool(mm['act']), bool(ctl[min(ci, len(ctl) - 1), 5]), False)
    cj = min(ci, len(ctl) - 1)
    rows.append((ts, v, mm['dc'], action.desiredCurvature, mm['lc'], lc_sim, e, psi, desired if desired is not None else 0.0,
                 float(ctl[cj, 2]), float(ctl[cj, 1]), lag, lcc._hold, lcc._correction, mm['act'], is_closed, bias))
  r = np.array(rows, dtype=float)
  return {c: r[:, i] for i, c in enumerate(COLS)}


def _ma(x, n):
  k = np.ones(n) / n
  return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode='edge'), k, 'valid')


def metrics(r, bend_from=None):
  """What a window looked like once the loop was closed (and, for the bend figures, from `bend_from` s on)."""
  m = r['closed'] > 0
  if bend_from is not None: m &= r['t'] >= bend_from
  v2 = r['v'] ** 2
  lat = (r['k_log'] + r['dk']) * v2                      # the simulated car's lateral acceleration
  lat_log = r['k_log'] * v2
  side = np.sign(np.mean(r['dc_log'][m]))                # + is a bend to the right, and lane y + is right
  swing = lambda a: float(np.sqrt(np.mean((a - _ma(a, 100))[m] ** 2)))
  jerk = lambda a: float(np.sqrt(np.mean(np.gradient(_ma(a, 4), 0.05)[m] ** 2)))
  return dict(
    off_path=float(np.abs(r['e'][m]).max()),             # m from the logged path
    inside=float(-side * np.mean(r['lane'][m])),         # m the car sits toward the inside of the bend
    inside_log=float(-side * np.mean(r['lane_log'][m])),
    swing=swing(lat), swing_log=swing(lat_log), jerk=jerk(lat), jerk_log=jerk(lat_log),
    dc_corr=float(np.corrcoef(r['dc_log'][m], r['dc'][m])[0, 1]),
    lat=float(np.mean(np.abs(lat_log[m]))), v=float(np.mean(r['v'][m])))


def main():
  seg = sys.argv[1]
  kw = dict(a.split('=') for a in sys.argv[2:])
  r = run(seg, hold=(kw['HOLD'] == '1') if 'HOLD' in kw else None, closed=kw.get('CLOSED', '0') == '1',
          tclose=float(kw.get('TCLOSE', 0)), t0=float(kw.get('T0', 0)), t1=float(kw.get('T1', 60)),
          einit=float(kw.get('EINIT', 0)), e0=float(kw.get('E0', 0)), psi0=float(kw.get('PSI0', 0)))
  if 'OUT' in kw: np.savez(kw['OUT'], **r)
  r['closed'][:] = 1
  for k, v in metrics(r).items(): print(f'{k:11s} {v:+.3f}')


if __name__ == '__main__':
  main()
