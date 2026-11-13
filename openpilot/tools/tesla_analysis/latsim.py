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
os.environ.setdefault('DEV', 'CPU:LLVM')
os.environ.setdefault('CC', f'{S}/clang/usr/lib/llvm-18/bin/clang')
sys.path.insert(0, '/home/compiler/openpilot'); sys.path.insert(1, '/home/compiler/openpilot/tinygrad_repo')
import numpy as np
from openpilot.cereal import log, messaging
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.framereader import FrameReader
from openpilot.common.realtime import DT_CTRL
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.model import calib_from_medmodel, calib_from_sbigmodel, get_warp_matrix
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


def load(seg):
  rl = f'/home/compiler/openpilot/op-logs/{seg}/rlog.zst'
  cs, ctl, mdl = [], [], []
  enc = {'img': {}, 'big_img': {}}
  st = dict(act=False, blk=False, v=0.0, roll=0.0, delay=0.3, rpy=None, height=1.37, dev='mici', sensor='os04c10')
  for m in LogReader(rl):
    w = m.which(); t = m.logMonoTime * 1e-9
    if w == 'carState':
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


def main():
  seg = sys.argv[1]
  kw = dict(a.split('=') for a in sys.argv[2:])
  hold, closed = kw.get('HOLD', '1') == '1', kw.get('CLOSED', '0') == '1'
  e0, psi0 = float(kw.get('E0', 0)), float(kw.get('PSI0', 0))
  t0s, t1s = float(kw.get('T0', 0)), float(kw.get('T1', 60))
  tclose = float(kw.get('TCLOSE', 0))            # the loop is closed from here; before it the log is followed
  segn = int(seg.split('--')[-1])
  ctl, mdl, st = load(seg)
  tb = mdl[0]['t']
  vd = f'/home/compiler/openpilot/op-video/{seg}'
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
  # plant, at 100 Hz on the controlsState time base
  e, psi = 0.0, 0.0
  einit = float(kw.get('EINIT', 0)); placed = False
  pose_hist = []                                   # (t, e, psi)
  n_delay = int(round(ACT_DELAY / DT_CTRL)); dq = [0.0] * n_delay; lag = 0.0
  desired = None
  ci = bisect.bisect_left(ctl[:, 0], tb + t0s)
  rows = []
  for k, mm in enumerate(mdl):
    ts = mm['t'] - tb
    if ts < t0s: continue
    if ts > t1s: break
    is_closed = closed and ts >= tclose
    if is_closed and not placed:
      e += einit; placed = True
    # the pose the frame was taken from
    if is_closed and pose_hist:
      tq = mm['t'] - FRAME_AGE
      j = max(0, bisect.bisect_right([p[0] for p in pose_hist[-40:]], tq) - 1)
      _, ef, pf = pose_hist[-40:][j]
    else:
      ef, pf = 0.0, 0.0
    ef += e0; pf += psi0
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
    rows.append((ts, v, mm['dc'], action.desiredCurvature, mm['lc'], lc_sim, e, psi, desired if desired is not None else 0.0,
                 float(ctl[min(ci, len(ctl) - 1), 2]), float(ctl[min(ci, len(ctl) - 1), 1]), lag, lcc._hold, lcc._correction, mm['act']))
  r = np.array(rows)
  np.savez(kw.get('OUT', 'latsim_out.npz'), r=r)
  v2 = r[:, 1] ** 2
  print(f'frames {len(r)}  model dc: logged vs sim  rms diff {np.sqrt(((r[:,2]-r[:,3])**2*v2**2).mean()):.3f} m/s2  corr {np.corrcoef(r[:,2],r[:,3])[0,1]:.3f}')
  print(f'lane centre: logged vs sim rms diff {np.sqrt(((r[:,4]-r[:,5])**2).mean()):.3f} m   mean shift {np.mean(r[:,5]-r[:,4]):+.3f}')
  print(f'controller cmd: logged vs sim rms diff {np.sqrt(((r[:,8]-r[:,9])**2*v2**2).mean()):.3f} m/s2')
  print(f'e: mean {r[:,6].mean():+.3f} min {r[:,6].min():+.3f} max {r[:,6].max():+.3f}')


if __name__ == '__main__':
  main()
