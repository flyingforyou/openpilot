#!/usr/bin/env python3
"""Catalog, storage and download of the optional external driving models.

The built-in model is not managed here at all. It stays where the build puts it
(selfdrive/modeld/models/driving_tinygrad.pkl, written by SConscript) and is loaded by the
same code as before -- this module only knows that it exists so the selector can list it and
so `select("stock")` has something to write. Everything below the stock entry is an artifact
downloaded at the user's request into a directory outside the git tree, so a pull or a reset
does not take a model away.

Nothing in here runs on the boot path. Downloads happen only when the WebUI asks for one;
modeld's side of this is a path lookup and a pickle load.
"""
import hashlib
import json
import os
import pickle
import shutil
import subprocess
import sys
import threading
import urllib.error
import urllib.request
from pathlib import Path

from openpilot.common.swaglog import cloudlog

# Same test common.hardware makes, inlined so this module stays importable without dragging in
# cereal -- the WebUI, modeld and a bare `python3 model_manager.py` all load it.
AGNOS = os.path.isfile('/AGNOS')

# The one place a supported model is described. Runtime decisions key off the id, never the
# display name -- the name is UI text and may change, the id is what lands in the parameter.
STOCK_MODEL_ID = "stock"

# Display order is this dict's order: stock first as the always-available fallback, then the
# external models in the order the selector should show them.
DRIVING_MODELS: dict[str, dict] = {
  "stock": {
    # The built-in artifact is comma's CD210 (#37050), repacked into one graph by #38173: the
    # supercombo's model_checkpoint is CD210's vision and policy checkpoints concatenated, and
    # its output slices are the two CD210 graphs' laid end to end. Two later model swaps
    # (#38164, #38475) were both reverted, so the file is byte-identical to the repack.
    # Only the display name says CD210 -- everything at runtime keys off the "stock" id.
    "name": "Stock (CD210)",
    "version": "stock",
    "builtin": True,
  },
  # Both of these predict the action directly rather than a plan, and each is built on a
  # different vision backbone -- which is the only difference between models that changes what
  # modeld does. The rest of the published RDF line (V1, V2, V3, V5, V6) is the same backbone
  # as V4 with a different policy head, so listing it would be five ways to say one thing.
  "rdf43": {
    "name": "RDF V4",
    "version": "v15",
    # vision 1acf0a93-3b20-4808-beb4-739aca6bb852/100, policy 12d7394b-6ad6-49b9-9775-cc4f7f9828b0/400
  },
  "deeprl333": {
    "name": "Deep RL 3 V3",
    "version": "v15",
    # vision 1c8e05fa-bb24-42ad-af22-c0e6d59a5df5/100, policy 6d9d6f8a-5c82-41f6-92aa-4c1a11eb5645/400
  },
}

# The same two sources StarPilot downloads from, in the same order: the GitHub raw host, then
# the GitLab mirror of the same tree. Both serve identical bytes -- checked against the
# published sha256 -- so the second is purely there for when the first is unreachable.
BASE_URLS = (
  "https://raw.githubusercontent.com/firestar5683/StarPilot-Resources/Models",
  "https://gitlab.com/firestar5683/FrogPilot-Resources/-/raw/Models",
)
BASE_URL = BASE_URLS[0]

# The artifacts are published split into ~95MB parts because of GitHub's blob limit. How many
# parts there are is not part of the contract -- .p00 upwards are probed until one is missing,
# and an artifact published as a single file (much of that repository is) works too.
MAX_PARTS = 100
_NET_TIMEOUT = 30
_CHUNK = 1024 * 1024

# Downloads are only ever visible under the final name once they are complete and verified.
DOWNLOAD_SUFFIX = ".download"

# A downloaded artifact is a pickled tinygrad JIT, which only deserializes under the tinygrad
# it was compiled against. Checksums say the bytes arrived intact; they say nothing about that.
# So each install is loaded once, in a throwaway process, and the answer is remembered here --
# otherwise a mismatch only shows up as modeld quietly running stock on the next drive.
STATUS_FILE = ".artifact_status.json"
VERIFY_TIMEOUT = 300


def artifact_name(model_id: str) -> str:
  return f"{model_id}_driving_tinygrad.pkl"


def models_dir() -> Path:
  """Outside the git tree so a pull, reset or branch change keeps the downloaded models.

  /data is the device's persistent partition -- the same one holding /data/params and
  /data/media -- and survives everything short of a reflash.
  """
  if not AGNOS:
    return Path.home() / ".comma" / "models" / "driving"
  return Path("/data/models/driving")


def artifact_path(model_id: str) -> Path:
  return models_dir() / artifact_name(model_id)


def is_builtin(model_id: str) -> bool:
  return bool(DRIVING_MODELS.get(model_id, {}).get("builtin"))


def is_installed(model_id: str) -> bool:
  if model_id not in DRIVING_MODELS:
    return False
  if is_builtin(model_id):
    return True   # the build guarantees it; modeld falls back to it either way
  return artifact_path(model_id).is_file()


def display_name(model_id: str) -> str:
  return DRIVING_MODELS.get(model_id, {}).get("name", model_id)


def resolve_selected(params) -> str:
  """What modeld should try to load. An absent, unknown or uninstalled selection is stock --
  the parameter is a request, not a promise, and stock is always there to fall back on."""
  try:
    raw = params.get("DrivingModel")
  except Exception:
    return STOCK_MODEL_ID
  model_id = (raw or "").strip() if isinstance(raw, str) else STOCK_MODEL_ID
  if model_id not in DRIVING_MODELS:
    return STOCK_MODEL_ID
  return model_id


# ---------------------------------------------------------------------------- downloading

def _open(url: str, method: str = "GET"):
  req = urllib.request.Request(url, method=method, headers={"User-Agent": "openpilot"})
  return urllib.request.urlopen(req, timeout=_NET_TIMEOUT)


def _fetch_text(url: str) -> str:
  with _open(url) as resp:
    return resp.read(4096).decode().strip()


def _head_size(url: str) -> int | None:
  """Size of a part, or None if it is not published. Also how the part count is discovered."""
  try:
    with _open(url, method="HEAD") as resp:
      return int(resp.headers.get("Content-Length") or 0)
  except urllib.error.HTTPError as e:
    if e.code == 404:
      return None
    raise


def _part_urls(base: str, model_id: str) -> tuple[list[str], int]:
  """Every published part of an artifact, in order, with the total byte count."""
  root = f"{base}/{artifact_name(model_id)}"
  urls, total = [], 0
  for i in range(MAX_PARTS):
    size = _head_size(f"{root}.p{i:02d}")
    if size is None:
      break
    urls.append(f"{root}.p{i:02d}")
    total += size
  if not urls:
    # published whole rather than split
    size = _head_size(root)
    if size is None:
      raise FileNotFoundError(f"no artifact published for {model_id}")
    urls, total = [root], size
  return urls, total


def _expected_sha256(base: str, model_id: str) -> str:
  """The published checksum, rejected before a download starts if it is not one.

  A 404 page or an HTML error would otherwise be read as a checksum and turn into a mismatch
  after 128MB of transfer instead of immediately.
  """
  text = _fetch_text(f"{base}/{artifact_name(model_id)}.sha256").split()
  digest = text[0].lower() if text else ""
  if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
    raise ValueError(f"{base} did not serve a sha256 for {model_id}")
  return digest


# ------------------------------------------------------------------ runtime verification

def _status_path() -> Path:
  return models_dir() / STATUS_FILE


def _read_status() -> dict:
  try:
    with open(_status_path()) as f:
      data = json.load(f)
    return data if isinstance(data, dict) else {}
  except (OSError, ValueError):
    return {}


def _write_status(model_id: str, entry: dict | None) -> None:
  data = _read_status()
  if entry is None:
    data.pop(model_id, None)
  else:
    data[model_id] = entry
  try:
    models_dir().mkdir(parents=True, exist_ok=True)
    tmp = _status_path().with_suffix(".tmp")
    with open(tmp, "w") as f:
      json.dump(data, f, indent=1)
    os.replace(tmp, _status_path())
  except OSError:
    cloudlog.exception("could not record artifact status")


def _file_sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with open(path, "rb") as f:
    while buf := f.read(_CHUNK):
      digest.update(buf)
  return digest.hexdigest()


# Failures that are about the artifact: a pickle written by a different tinygrad reaches for
# constructors and attributes that have since changed shape. Anything outside this set -- a
# missing device node, no memory, no disk -- says nothing about the artifact and must not
# condemn it, or a verification run on the wrong machine would permanently blacklist a model
# that works.
_ARTIFACT_FAULTS = ("TypeError", "AttributeError", "ImportError", "ModuleNotFoundError",
                    "UnpicklingError", "EOFError", "IndexError", "KeyError",
                    "AssertionError", "ValueError")

_VERIFY_CODE = """
import json, pickle, sys, traceback
try:
  with open(sys.argv[1], 'rb') as f:
    art = pickle.load(f)
  assert isinstance(art, dict) and 'run_policy' in art, 'not a driving artifact'
  out = {'state': 'ok', 'error': ''}
except BaseException as e:
  out = {'state': 'fail', 'kind': type(e).__name__,
         'error': ''.join(traceback.format_exception_only(type(e), e)).strip()}
sys.stdout.write('RESULT ' + json.dumps(out))
"""


def verify_artifact(model_id: str) -> tuple[str, str]:
  """Can this tinygrad actually deserialize the artifact?

  Returns one of 'ok', 'invalid' or 'unknown' with a reason. Run in a throwaway interpreter: a
  pickle built against a different tinygrad can fail in ways an except clause does not catch,
  and none of them should be able to reach the web server.
  """
  path = artifact_path(model_id)
  if not path.is_file():
    return "unknown", "artifact missing"
  try:
    p = subprocess.run([sys.executable, "-c", _VERIFY_CODE, str(path)], capture_output=True,
                       text=True, timeout=VERIFY_TIMEOUT)
  except subprocess.TimeoutExpired:
    return "unknown", f"load timed out after {VERIFY_TIMEOUT}s"
  except OSError as e:
    return "unknown", f"could not run the check: {e}"

  line = next((ln for ln in reversed((p.stdout or "").splitlines()) if ln.startswith("RESULT ")), None)
  if line is None:
    # died without reporting -- a segfault or the OOM killer, neither of which is a verdict
    tail = (p.stderr or "").strip().splitlines()
    return "unknown", tail[-1] if tail else f"check did not report (exit {p.returncode})"
  try:
    out = json.loads(line[len("RESULT "):])
  except ValueError:
    return "unknown", "check reported nothing usable"

  if out.get("state") == "ok":
    return "ok", ""
  reason = str(out.get("error") or "load failed")
  return ("invalid" if out.get("kind") in _ARTIFACT_FAULTS else "unknown"), reason


def artifact_status(model_id: str) -> dict:
  """The remembered verification result, ignored if the file has changed underneath it."""
  entry = _read_status().get(model_id)
  if not isinstance(entry, dict):
    return {}
  return entry


class _Download:
  """One in-flight download. Runs on its own thread so the HTTP handler returns immediately."""

  def __init__(self, model_id: str):
    self.model_id = model_id
    self.cancelled = threading.Event()
    self.lock = threading.Lock()
    self.done = False
    self.error: str | None = None
    self.received = 0
    self.total = 0
    self.stage = "download"
    self.thread = threading.Thread(target=self._run, name=f"dl-{model_id}", daemon=True)

  def start(self):
    self.thread.start()

  def state(self) -> dict:
    with self.lock:
      pct = int(100 * self.received / self.total) if self.total else 0
      return {"downloading": not self.done, "progress": min(pct, 100), "stage": self.stage,
              "received": self.received, "total": self.total, "error": self.error}

  def _fetch(self, base: str, tmp: Path) -> tuple[int, str]:
    """One full attempt against one mirror. Returns (bytes, sha256) or raises."""
    expected = _expected_sha256(base, self.model_id)
    urls, total = _part_urls(base, self.model_id)
    with self.lock:
      self.received, self.total = 0, total

    digest = hashlib.sha256()
    with open(tmp, "wb") as out:
      for url in urls:
        with _open(url) as resp:
          while True:
            if self.cancelled.is_set():
              raise InterruptedError("cancelled")
            buf = resp.read(_CHUNK)
            if not buf:
              break
            out.write(buf)
            digest.update(buf)
            with self.lock:
              self.received += len(buf)
      out.flush()
      os.fsync(out.fileno())

    got = digest.hexdigest()
    if got != expected:
      raise ValueError(f"checksum mismatch: got {got}, expected {expected}")
    return total, got

  def _run(self):
    tmp = artifact_path(self.model_id).with_name(artifact_name(self.model_id) + DOWNLOAD_SUFFIX)
    try:
      models_dir().mkdir(parents=True, exist_ok=True)
      last: Exception | None = None
      for base in BASE_URLS:
        try:
          total, got = self._fetch(base, tmp)
          break
        except InterruptedError:
          raise
        except Exception as e:
          last = e
          cloudlog.warning(f"driving model {self.model_id} from {base} failed: {e}")
      else:
        raise last or RuntimeError("no source available")

      # Only now does the artifact get the name modeld looks for.
      os.replace(tmp, artifact_path(self.model_id))
      cloudlog.warning(f"driving model {self.model_id} installed ({total} bytes, sha256 {got})")

      # Intact bytes are not the same as a loadable model. Ask now, while the answer can be
      # shown, rather than letting modeld discover it and fall back to stock mid-drive.
      with self.lock:
        self.stage = "verify"
      state, reason = verify_artifact(self.model_id)
      _write_status(self.model_id, {"state": state, "error": reason, "sha256": got})
      if state != "ok":
        with self.lock:
          self.error = reason
        cloudlog.error(f"driving model {self.model_id} verification {state}: {reason}")
    except Exception as e:
      # A partial, unverified or cancelled download must never be left looking installed.
      try:
        tmp.unlink(missing_ok=True)
      except OSError:
        pass
      with self.lock:
        self.error = str(e) or type(e).__name__
      cloudlog.exception(f"driving model {self.model_id} download failed")
    finally:
      with self.lock:
        self.done = True


class ModelManager:
  """Owns the downloads in flight. One per server process."""

  def __init__(self, params):
    self.params = params
    self.lock = threading.Lock()
    self.downloads: dict[str, _Download] = {}

  # -- queries ---------------------------------------------------------------

  def selected(self) -> str:
    return resolve_selected(self.params)

  def running(self) -> str | None:
    """What modeld actually loaded, which is only the same as the selection after a restart.
    None when modeld has not reported yet -- reporting stock for "unknown" would be the same
    misleading answer the selection parameter already gives."""
    try:
      raw = self.params.get("RunningDrivingModel")
    except Exception:
      return None
    model_id = raw.strip() if isinstance(raw, str) else None
    return model_id if model_id in DRIVING_MODELS else None

  def snapshot(self, onroad: bool) -> dict:
    selected = self.selected()
    with self.lock:
      progress = {mid: dl.state() for mid, dl in self.downloads.items()}
    models = []
    for model_id, cfg in DRIVING_MODELS.items():
      entry = {
        "id": model_id,
        "name": cfg["name"],
        "version": cfg.get("version"),
        "builtin": bool(cfg.get("builtin")),
        "installed": is_installed(model_id),
      }
      if model_id == selected:
        entry["selected"] = True
      if entry["installed"] and not entry["builtin"]:
        status = artifact_status(model_id)
        state = status.get("state")
        if state == "invalid":
          entry["invalid"] = True
          entry["error"] = status.get("error") or "이 빌드에서 로드되지 않습니다"
        elif state == "unknown":
          # Could not be answered here rather than answered badly. Say so, but let it be
          # selected: modeld's own fallback still covers it.
          entry["warn"] = status.get("error") or "로드 확인 실패"
      st = progress.get(model_id)
      if st is not None:
        if st["downloading"]:
          entry.update({"downloading": True, "progress": st["progress"],
                        "stage": st.get("stage", "download")})
        elif st["error"]:
          entry["error"] = st["error"]   # stays visible until the next attempt replaces it
      models.append(entry)
    return {"running": self.running(), "selected": selected, "onroad": bool(onroad),
            "models": models}

  # -- mutations -------------------------------------------------------------

  def download(self, model_id: str) -> tuple[int, dict]:
    if model_id not in DRIVING_MODELS:
      return 400, {"error": f"알 수 없는 모델: {model_id}"}
    if is_builtin(model_id):
      return 400, {"error": "기본 모델은 내려받지 않습니다"}
    with self.lock:
      existing = self.downloads.get(model_id)
      if existing is not None and not existing.done:
        return 409, {"error": "이미 내려받는 중입니다"}
      dl = _Download(model_id)
      self.downloads[model_id] = dl
    dl.start()
    return 200, {"ok": True, "downloading": True, "progress": 0}

  def select(self, model_id: str) -> tuple[int, dict]:
    if model_id not in DRIVING_MODELS:
      return 400, {"error": f"알 수 없는 모델: {model_id}"}
    if not is_installed(model_id):
      return 409, {"error": "설치되지 않은 모델입니다"}
    # Selecting one that is known not to load would just make modeld fall back to stock on the
    # next drive, with nothing on screen to say why. Only a verdict about the artifact blocks;
    # a check that could not run does not.
    status = artifact_status(model_id)
    if status.get("state") == "invalid":
      return 409, {"error": f"이 빌드에서 로드되지 않는 artifact입니다: {status.get('error', '')}"}
    try:
      self.params.put("DrivingModel", model_id)
    except Exception as e:
      # The parameter is declared in params_keys.h, so this means the build is older than the
      # declaration. Say that rather than returning a bare 500.
      return 500, {"error": f"DrivingModel 파라미터를 쓸 수 없습니다 (빌드 필요?): {e}"}
    return 200, {"ok": True, "selected": model_id}

  def delete(self, model_id: str, onroad: bool) -> tuple[int, dict]:
    if model_id not in DRIVING_MODELS:
      return 400, {"error": f"알 수 없는 모델: {model_id}"}
    if is_builtin(model_id):
      return 400, {"error": "기본 모델은 삭제할 수 없습니다"}
    # Redundant while offroad-only mutation holds, but the rule is about the running model
    # rather than about being offroad, so state it directly.
    if onroad and self.running() == model_id:
      return 409, {"error": "실행 중인 모델은 삭제할 수 없습니다"}
    with self.lock:
      dl = self.downloads.get(model_id)
      if dl is not None and not dl.done:
        dl.cancelled.set()
    # Deleting what is selected would leave the selection pointing at nothing. modeld would
    # fall back on its own, but leaving the parameter lying is worse than resetting it here.
    if self.selected() == model_id:
      try:
        self.params.put("DrivingModel", STOCK_MODEL_ID)
      except Exception as e:
        return 500, {"error": f"선택을 stock으로 되돌리지 못했습니다: {e}"}
    try:
      artifact_path(model_id).unlink(missing_ok=True)
    except OSError as e:
      return 500, {"error": f"삭제 실패: {e}"}
    _write_status(model_id, None)
    return 200, {"ok": True, "selected": self.selected()}


# ------------------------------------------------------------------- modeld's side

def load_external_artifact(model_id: str) -> dict:
  """Deserialize a downloaded artifact.

  These are plain protocol-5 pickles, unlike the built-in model, which the build writes in the
  out-of-band chunked format and which is loaded by the untouched path in modeld.
  """
  path = artifact_path(model_id)
  if not path.is_file():
    raise FileNotFoundError(path)
  with open(path, "rb") as f:
    artifact = pickle.load(f)
  if not isinstance(artifact, dict):
    raise ValueError(f"artifact is {type(artifact).__name__}, expected dict")
  return artifact


def free_space_bytes() -> int:
  try:
    models_dir().mkdir(parents=True, exist_ok=True)
    return shutil.disk_usage(models_dir()).free
  except OSError:
    return 0


if __name__ == "__main__":
  print(json.dumps({"dir": str(models_dir()),
                    "installed": {m: is_installed(m) for m in DRIVING_MODELS}}, indent=2))
