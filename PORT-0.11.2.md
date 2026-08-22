# Tesla HW1 + CarrotPilot on openpilot 0.11.2

Branch: `tesla-hw1-carrot-0.11.2` · base: `44be03578` (xnor-dev) · 35 commits on top.

This is the forward port of the `tesla-hw1-carrot` branch (openpilot 0.11.1, tip `6e70c485f`) onto
0.11.2. It runs on the car and all daemons stay up. It has not been driven far.

The single most useful thing in this file is [what went wrong and how it was
found](#the-failure-mode-that-cost-the-most), because the same class of bug is still latent in
three files listed under [Known gaps](#known-gaps).

---

## Getting a machine ready

The port was developed from Windows driving WSL, which cost real time (see
[Environment notes](#environment-notes)). On Ubuntu it is just:

```bash
git clone git@github.com:flyingforyou/openpilot.git
cd openpilot
git checkout tesla-hw1-carrot-0.11.2
git submodule update --init --recursive
uv sync --all-extras
uv run scons -j$(nproc)
```

`--all-extras` is not optional: `comma-deps-ncurses` lives in the `tools` extra and SConstruct
imports ncurses while reading, so a plain `uv sync` leaves the build unable to start.

`pytest` is **not installed** — upstream 0.11.2 dropped it from the `testing` extra. Run tests as:

```bash
uv run --with pytest pytest openpilot/selfdrive/controls/tests/ -q
```

### On the device

```bash
ssh comma@<ip>
cd /data/openpilot
git fetch origin tesla-hw1-carrot-0.11.2 && git reset --hard origin/tesla-hw1-carrot-0.11.2
export PATH=/usr/comma/shims:$PATH          # matters, see below
source scripts/setup-device.sh              # not `uv sync`/`uv run` by hand -- see below
uv run scons -j4
sudo systemctl restart comma
```

Three device facts that are not guessable:

- **`/usr/comma/shims/uv` is a wrapper.** It remounts `/` read-write and runs the real uv under
  `sudo -E`, then remounts read-only. Boot puts it first on `PATH`, so everything at boot runs as
  root by design — a root-owned `.venv` is correct, not a bug. An ssh session resolves `uv` to
  `/usr/bin/uv` instead, which is why building over ssh hits permission errors on `.venv/.lock`
  and why `sudo uv run scons` then fails with `Read-only file system: '/root/.cache'`. Put the
  shims directory on `PATH` and the problem disappears.
- **`scripts/setup-device.sh` runs itself on first boot.** `launch_chffrplus.sh` calls it when
  `.venv/bin/python3` is missing, so a fresh checkout builds its own venv without anyone
  intervening. It must never reach for the network — see
  [the eigen mistake](#the-eigen-mistake).
- **Always `source scripts/setup-device.sh` before touching `uv` by hand — never bare `uv sync`
  or `uv run` over ssh.** uv's python install and cache default to `$HOME`, a 100MB overlay on
  AGNOS that does not survive a reboot; the script points `UV_PYTHON_INSTALL_DIR` and
  `UV_CACHE_DIR` at `/data` instead. A bare `uv sync`/`uv run` skips that, and the device comes
  back from a reboot unable to import `capnp` (or `zmq`, or missing `comma-deps-ncurses` if
  `--all-extras` was also skipped) with no sign anything is wrong until the next restart. Hit
  three times in one session before it was worth writing down: `source`, not `bash` or a fresh
  `export` block copied from memory — sourcing is what keeps the script's exports alive for the
  `uv run scons` that follows, on the *same* line, in the *same* shell.

---

## What this branch adds to stock 0.11.2

Five features, each one commit, matching the sections of the `/live` tuning page:

| Feature | Commit | Toggle |
| --- | --- | --- |
| Tesla HW1 Model X port | `950fdf008` | — |
| CarrotPilot longitudinal planner | `b31052a59` | `CarrotLongEnabled` (default on) |
| Map-based cruise target | `45d60a812` | `TeslaMapAutoSpeed` |
| Cluster MAX sync, lead hold, cars-as-trucks | `68e8717e8` | `TeslaSyncClusterSpeed`, `TeslaCarsAsTrucks` |
| Tuning server and its three pages | `084a79a99` | always on, port 8088 |

Other toggles: `TeslaStockLong`, `TeslaCoopSteer`, `TeslaStockAutopark`, `TeslaMapAutoSpeedCurve`,
`TeslaMapCurveLatAccel`, `TeslaMapAutoSpeedMax`, `StopDistanceCarrot`.

Pages: `http://<device>:8088/live`, `/shadow`, `/can`.

---

## The failure mode that cost the most

**Whole files were copied from 0.11.1 instead of having our changes re-applied to 0.11.2's
version.** That silently reverts upstream work, and it does not show up in any diff between the
two branches — only against the 0.11.2 base.

It cost a whole debugging session. The device booted, sat on the comma logo, and did not
recognise the car. The chain was:

1. `controlsd`, `plannerd` and `radard` subscribed to services 0.11.2 renamed. `SubMaster` raises
   on an unknown name, so all three died in their constructor and never ran at all.
2. `ui` reached the onroad render path and raised `KeyError: 'liveCalibration'` on the first frame
   — the logo screen.
3. `card` called `Params.put_nonblocking`, removed in 0.11.2, so the car was never fingerprinted.
4. With those fixed, `controlsd` died on `CP.vEgoStarting` and `radard` on
   `radarState.leadOne.status` — schema members that no longer exist.
5. With *those* fixed, `ui` died the instant the car engaged: `draw_circle_gradient` takes a
   `Vector2` on 0.11.2's raylib, not separate x and y.

Every one of these was a line carried over verbatim from 0.11.1.

### The two checks that find this class

Keep both. They are cheap and they are what turned a guessing game into a list.

**1. Which files revert upstream.** A pure addition removes zero lines. Anything with a high
removal count against the base is a wholesale copy:

```bash
BASE=ee6ba4f30
git diff --name-only $BASE..HEAD | while read -r f; do
  [ -e "$f" ] || continue
  del=$(git diff $BASE..HEAD -- "$f" | grep -c '^-[^-]')
  [ "$del" -gt 5 ] && printf '%4s removed  %s\n' "$del" "$f"
done | sort -rn
```

**2. Every service name resolves.** Walk each `SubMaster(...)`/`PubMaster(...)` call, pull the
quoted strings out, and check them against `openpilot/cereal/services.py`. A name that is not
there is a daemon that cannot start. After the fixes this reports clean.

A third check, for raylib specifically, compares each `rl.*` call site's argument count against
the installed `pyray` signature. 191 call sites, one mismatch — the engage crash.

---

## Renames and removals 0.11.2 introduced

The complete list this port had to absorb. Anything still spelled the left-hand way is a bug.

**Services** (`cereal/services.py`)

| 0.11.1 | 0.11.2 |
| --- | --- |
| `liveCalibration` | `extrinsicsCalibration` |
| `livePose` | `deviceMotion` |
| `liveParameters` | `vehicleParameters` |
| `liveDelay` | `lateralDelay` |
| `liveTorqueParameters` | `lateralTorqueParameters` |
| `liveTracks` | `radarTracks` |

**Schema**

- `RadarState.LeadData.status` → `.present`
- `RadarState.carStateMonoTime` — gone
- `LongitudinalPIDTuning.kpBP` / `kpV` — retired to the `deprecated` group ("deprecate long kp")
- `CarParams.vEgoStarting`, `startingState`, `startAccel`, `stoppingDecelRate` — retired to
  `deprecated`. `LongCtrlState.starting` is therefore unreachable; the branches leading to it are
  gone and the stopping ramp uses upstream's fixed 1.0 m/s²/s.
- `LeadData.fcw`, `dPath` — still deprecated, no longer published. `aLead` was moved *out* of the
  group at its original ordinal `@5` because `carrot_functions.py` reads it.
- `LeadData.aRel`, `vLat` — still deprecated, unused.

**APIs**

| 0.11.1 | 0.11.2 |
| --- | --- |
| `Params.put_nonblocking(k, v)` | `Params.put(k, v)` — already non-blocking; pass `block=True` where the write must land first |
| `Params.put_bool_nonblocking(k, v)` | `Params.put_bool(k, v)` |
| `LatControlTorque.update_live_torque_params` | `update_torque_parameters` |
| `PoseCalibrator.feed_live_calib` | `feed_extrinsics_calibration` |
| `Pose.from_live_pose` | `Pose.from_device_motion` |
| `rl.draw_circle_gradient(x, y, r, c1, c2)` | `rl.draw_circle_gradient(Vector2, r, c1, c2)` |
| `PythonProcess(..., restart_if_crash=True)` | no such argument — a crash is just a crash |
| `driverMonitoringState.awarenessStatus < 0` | `.noResponseForceDecel` |

**Structure**

- tree moved under `openpilot/`; `cereal` → `openpilot.cereal`, `system.hardware` →
  `common.hardware`
- car schema: `from cereal import car` → `from opendbc.car.structs import car`
- `selfdrive/monitoring/helpers.py` → `policy.py`
- `legacy.capnp` → `deprecated.capnp`
- vendored acados → the `acados` PyPI package; eigen → `comma-deps-eigen`
- `AngleSteeringLimits` → `AngleSteeringLimitsVM`
- `TICI` removed; only `AGNOS` and `PC`

**Upstream additions that the copies had deleted, now restored**

`LatControlCurvature` and its branches · `lateralManeuverPlan` as the desired-curvature source ·
the experimental-mode confirmation page · `CC.driverMonitoringEscalation` · the eGPU indicator
(**still missing**, see below).

---

## Known gaps

Ranked by risk. The first is a real feature loss; the rest are unverified rather than known-broken.

1. **`openpilot/selfdrive/ui/mici/onroad/hud_renderer.py` — 50 upstream lines removed.** The
   wholesale copy dropped upstream's eGPU/usbgpu indicator entirely: `_txt_egpu*`,
   `_egpu_fade_time`, `_egpu_alpha_filter`, `_small_model_engaged`, `_draw_model_source`. It does
   not crash; the indicator simply never draws. The fix is to restore the 0.11.2 file and re-apply
   our ~210 lines (cluster MAX display, lead distance, traffic light) on top.
2. **`radard.py` (43), `mici/model_renderer.py` (24), `onroad/model_renderer.py` (8)** — removal
   counts consistent with our own rewrites rather than reverts, but not line-by-line audited.
3. **`tesla_legacy.h` (22), `tesla/carstate.py` (19), `carcontroller.py` (12)** — opendbc side,
   same status.
4. **A dead test on the 0.11.1 branch.** `selfdrive/controls/lib/test_map_cruise_override.py`
   tests `_update_override`, removed by `329d806aa` when the stalk became a trigger. All 9 tests
   fail there. Deliberately not ported; delete it on `tesla-hw1-carrot`.
5. **`CarParamsPersistent` on a device upgraded from 0.11.1 is stale.** Safety model 36 meant
   `teslaLegacy` then and `volvo` now. Only the UI and locationd read it, so it is harmless, but
   clear it to remove the confusion.

---

## Testing still needed

Nothing below has been done. Ordered so a failure stops you before it matters.

### Bench, no car

- [ ] `uv run --with pytest pytest openpilot/selfdrive/controls/ opendbc_repo/opendbc/car/tesla/ -q`
- [ ] Re-run both audit checks above; the service sweep must stay clean
- [ ] Cold boot from a wiped `.venv`, confirm `/live` answers unattended
- [ ] `/live`, `/shadow`, `/can` all render with the car connected

### Stationary, car connected, engine on

- [ ] Fingerprints as `TESLA_MODEL_X_HW1`, `card` publishes `carParams`
- [ ] No `canError` alert. It renders as **"Unknown Vehicle Variant"** — that string is upstream's
      wording for `canError`, not a fingerprinting problem. It means a message in the RX check
      list is missing or off-frequency. **One was seen during the first engage and has not been
      chased down.**
- [ ] Engage and disengage several times; `ui` must survive engage (this is what `9c10a50a5` fixed)
- [ ] Cluster MAX tracks the map target with `TeslaSyncClusterSpeed` on
- [ ] `TeslaCarsAsTrucks` on: cars appear on the cluster wearing a truck icon

### Moving, somewhere safe, hands ready

The longitudinal path changed most, so watch stopping above all.

- [ ] **Stopping behaviour.** The retired `vEgoStopping`/`vEgoStarting`/`stoppingDecelRate` were
      set to 0.1/0.1/0.3 by the Tesla interface on 0.11.1 and are unset on 0.11.2. Stops will feel
      different. Check the last metre and whether it creeps.
- [ ] **Starting from a stop.** `LongCtrlState.starting` no longer exists.
- [ ] Lead following: chevron holds through radar flicker, distance is not over-read at <30 m
- [ ] Curve slowing, on the highway and on ramps
- [ ] Map cruise: posted-limit target, the +10 mph cap, the road-class ceiling
- [ ] Stalk override still takes control away from the map, and openpilot's own stalk presses are
      not read as the driver disagreeing

### Not yet verified at all

- Model changed to supercombo upstream — lateral behaviour is untested on this branch
- The panda firmware here is built from a newer xnor base than 0.11.1 used

---

## Environment notes

Moving to Ubuntu removes all of the following. They are listed only so nobody rediscovers them.

- **`core.autocrlf=true` on Windows.** Committed blobs are LF but the working tree is CRLF, so
  `scp`-ing a working-tree file to the device produces `bash\r` in the shebang. Pull on the device
  with git instead of copying, or strip CR in transit.
- **PowerShell eats backslashes in nested quotes.** `tr -d "\r"` inside a PowerShell-invoked bash
  command became `tr -d "r"` and deleted every letter `r` in a source file (`from` → `fom`). Put
  anything non-trivial in a script file and run the file.
- **git-bash's `python` is a Microsoft Store stub** that silently prints `Python` and exits. Use
  WSL's python.
- **`op-0.11.2` is a git worktree of the `openpilot` checkout**, so its `.git` points at a
  Windows-style path that WSL cannot follow. Run git for it from git-bash, not WSL.

---

## The eigen mistake

Worth its own note, because the fix was to delete code rather than add it.

`setup-device.sh` used to download eigen from gitlab and symlink it at the repo root. That was
never necessary: `rednose` declares `comma-deps-eigen`, `uv.lock` pins it, and
`rednose_repo/site_scons/site_tools/rednose_filter.py` appends `eigen.INCLUDE_DIR` to `CPPPATH`
itself. `uv sync` had been putting the headers in the venv the whole time.

The download also made booting depend on a network. `launch_chffrplus.sh` runs the setup script
before wifi associates, so on a device with no `/data/eigen` yet the first curl failed, `set -e`
killed the script, and the device came up with no venv and nothing running. It passed every
hands-on test because an ssh session by definition already has a network.

**A comma device has to come up offline. Nothing in the boot path may reach for the network.**

---

## Devices

| IP | Role |
| --- | --- |
| `10.0.0.138` | the car. Currently on this branch |
| `10.0.0.56` | bench |

Both are at `/data/openpilot`. Rollback is `git reset --hard origin/tesla-hw1-carrot` plus a
rebuild — the 0.11.1 branch is untouched and still the known-good one.
