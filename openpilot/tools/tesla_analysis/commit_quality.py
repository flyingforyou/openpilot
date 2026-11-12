"""Ride quality of whole commits, compared on the same morning-commute minutes.

Real drives cannot compare commits: traffic moves the numbers more than the code does (10/26-11/11
read 40 % busier than 10/01-10/25 on the road with identical following code). So each commit is
checked out into its own worktree and replayed closed loop (its own closedloop_sim, planner and
radard) over one fixed set of recorded minutes, quality_windows.txt, with the params the device
had while that commit was driven, at one gap setting.

  .venv/bin/python openpilot/tools/tesla_analysis/commit_quality.py 9e33193df 4f2eae2b8 WORKTREE
  (WORKTREE = this checkout as it is, uncommitted changes included; needs QPARAMS)
  options (env): QWIN=<window file>  QGAP=4 (LongitudinalPersonality for every commit)
                 QWT=<worktree dir>  QPARAMS=<commit whose drive supplies params, for undriven commits>
                 QVARS='[["name", {param or ctf.X overrides}], ...]' (extra variants on every commit)

Columns: lib/ride_quality.py's vector on the plan command, then what only closed loop shows --
jerk of the car's own acceleration, gap error against the MPC's target (s of headway, RMS),
headway median / IQR, hard brakes (< -2 m/s^2) an hour.
"""
import glob, json, os, subprocess, sys
from multiprocessing import Pool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, ROOT)
from openpilot.selfdrive.controls.lib import ride_quality as rq  # noqa: E402

WT = os.environ.get('QWIN_WT', os.environ.get('QWT', os.path.expanduser('~/.cache/op_commit_quality')))
WIN = os.environ.get('QWIN', os.path.join(HERE, 'quality_windows.txt'))
GAP = int(os.environ.get('QGAP', 4))
LIBS = [f"{ROOT}/openpilot/selfdrive/controls/lib/longitudinal_mpc_carrot/c_generated_code",
        f"{ROOT}/openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code"]


def sh(*a, **k):
  return subprocess.run(a, capture_output=True, text=True, **k)


def full_hash(c):
  return sh('git', 'rev-parse', '--short=9', c, cwd=ROOT).stdout.strip()


def worktree(c):
  """The commit's tree, with this checkout's built artifacts (acados, capnp, params) linked in."""
  if c == 'WORKTREE':
    return ROOT
  d = os.path.join(WT, c)
  if not os.path.isdir(d):
    os.makedirs(WT, exist_ok=True)
    sh('git', 'worktree', 'add', '--detach', d, c, cwd=ROOT)
  built = sh('git', 'ls-files', '--others', '--ignored', '--exclude-standard', 'openpilot', cwd=ROOT).stdout.split()
  for f in built:
    if ('c_generated_code/' in f or f.startswith('openpilot/cereal/gen/') or '.so' in os.path.basename(f)) and '__pycache__' not in f:
      dst = os.path.join(d, f)
      if not os.path.lexists(dst):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        os.symlink(os.path.join(ROOT, f), dst)
  return d


def params_of(c):
  """initData params of the first local drive recorded on commit c (numbers only)."""
  from openpilot.tools.lib.logreader import LogReader
  for p in sorted(glob.glob(f"{ROOT}/op-logs/*--0/rlog.zst")):
    for m in LogReader(p):
      if m.which() != 'initData':
        continue
      if m.initData.gitCommit.startswith(c):
        out = {}
        for kv in m.initData.params.entries:
          v = bytes(kv.value).decode(errors='ignore').strip()
          v = {'true': '1', 'True': '1', 'false': '0', 'False': '0'}.get(v, v)
          try:
            out[kv.key] = int(v)
          except ValueError:
            try:
              out[kv.key] = float(v)
            except ValueError:
              pass
        return out
      break
  return None


def windows():
  for line in open(WIN):
    if line.strip() and not line.startswith('#'):
      r, s = line.split()[:2]
      yield r, int(s)


def run(job):
  tree, r, s, variants, tag = job
  dump = os.path.join(WT, 'out', f"{tag}_{r[:8]}_{s}.json")
  env = dict(os.environ, SIMRADARD='1', SIMLEAD2='1', SIMLOC='1', SIMDUMP=dump, SIMROOT=f"{ROOT}/op-logs", SIMDELAY='0.15',
             SIMTAU='0.10', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', PYTHONPATH=f"{tree}:{ROOT}/opendbc_repo", VARIANTS=json.dumps(variants),
             LD_LIBRARY_PATH=':'.join(LIBS + [os.environ.get('LD_LIBRARY_PATH', '')]))
  if os.environ.get('SIMCP'):
    env['SIMCP'] = os.environ['SIMCP']
  p = subprocess.run([sys.executable, f"{tree}/openpilot/tools/tesla_analysis/closedloop_sim.py", r, str(s), '3', '58'],
                     env=env, capture_output=True, text=True, cwd=tree)
  return json.load(open(dump)) if p.returncode == 0 else None


def score(dumps):
  parts, extra = [], dict(jsq=0.0, nj=0, gsq=0.0, ng=0, hw=[], hard=0, sec=0.0)
  for d in dumps:
    a = np.array(d)
    a = a[a[:, 0] >= a[0, 0] + 6]              # the replay's first seconds settle from the recorded state
    v, gap, vl, al, cmd, des, acc = a[:, 1] / 3.6, a[:, 2], a[:, 3] / 3.6, a[:, 4], a[:, 5], a[:, 6], a[:, 9]
    idx = np.where((v > 11) & (gap > 0) & (gap < 3 * np.maximum(v, 1)))[0]
    for R in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
      if len(R) < 200:
        continue
      parts.append(rq.components(cmd[R], v[R], gap[R], vl[R], al[R], 0.05))
      extra['jsq'] += float(np.sum((np.diff(acc[R]) / 0.05) ** 2)); extra['nj'] += len(R) - 1
      ok = des[R] > 1
      extra['gsq'] += float(np.sum(((gap[R][ok] - des[R][ok]) / v[R][ok]) ** 2)); extra['ng'] += int(ok.sum())
      extra['hw'] += list(gap[R] / v[R]); extra['hard'] += rq.crossings_down(acc[R], -2.0); extra['sec'] += len(R) * 0.05
  q = rq.vector(parts)
  q.update(jerk=np.sqrt(extra['jsq'] / extra['nj']), gap_err=np.sqrt(extra['gsq'] / max(extra['ng'], 1)),
           hw=float(np.median(extra['hw'])), hw_iqr=float(np.subtract(*np.percentile(extra['hw'], [75, 25]))),
           hard_h=extra['hard'] / (extra['sec'] / 3600))
  return q


COLS = [('minutes', 'min', '%5.0f'), ('brake_on', 'brake', '%5.2f'), ('ubon', 'unprv', '%5.2f'), ('accel_on', 'thrtl', '%5.2f'),
        ('rev', 'rev', '%5.2f'), ('cmd_jerk', 'cmdJ', '%5.3f'), ('calm_brake', 'calmB', '%5.1f'), ('abs_cmd', '|cmd|', '%5.3f'),
        ('jerk', 'jerk', '%5.3f'), ('gap_err', 'gapE', '%5.3f'), ('hw', 'hw', '%5.2f'), ('hw_iqr', 'hwIQR', '%5.2f'), ('hard_h', 'hard/h', '%6.1f')]


def main():
  commits = [c if c == 'WORKTREE' else full_hash(c) for c in (sys.argv[1:] or ['WORKTREE'])]
  extra = json.loads(os.environ.get('QVARS', '[]'))
  fallback = params_of(full_hash(os.environ['QPARAMS'])) if os.environ.get('QPARAMS') else None
  os.makedirs(os.path.join(WT, 'out'), exist_ok=True)
  W = list(windows())
  print('%-24s ' % 'commit' + ' '.join('%5s' % h for _, h, _ in COLS))
  for c in commits:
    P = (None if c == 'WORKTREE' else params_of(c)) or fallback
    if P is None:
      print(f"{c}: no local drive on this commit -- set QPARAMS=<a driven commit>")
      continue
    P['LongitudinalPersonality'] = GAP
    variants = [['c', P]] + [[n, {**P, **o}] for n, o in extra]
    tree = worktree(c)
    with Pool(int(os.environ.get('QJOBS', os.cpu_count()))) as pool:
      res = pool.map(run, [(tree, r, s, variants, c) for r, s in W])
    for name, _ in variants:
      dumps = [d[name] for d in res if d and name in d]
      q = score(dumps)
      label = c if name == 'c' else f"{c} {name}"
      print('%-24s ' % label + ' '.join(f % q[k] for k, _, f in COLS) + f"   ({len(dumps)}/{len(W)})")


if __name__ == '__main__':
  main()
