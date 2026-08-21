# Replaying a recorded drive onto this device's screen

How to watch a recorded segment play back on the comma 4 with the real UI drawing over it, and
with a live planner deciding rather than the recorded plan being replayed.

Everything here was learned the hard way on this device. Each step exists because skipping it
produces a specific, non-obvious failure, noted alongside.

## The short version

```bash
ssh comma@<device>

sudo systemctl stop comma                     # replay claims services the manager already owns

cat > /tmp/run_plannerd.py <<'EOF'
import openpilot.common.realtime as rt
rt.config_realtime_process = lambda *a, **k: None
from openpilot.selfdrive.controls.plannerd import main
main()
EOF

cd /data/openpilot
TERM=xterm tmux new-session -d -s rp -x 200 -y 50 \
  'cd /data/openpilot && TERM=xterm ./tools/replay/replay -d /data/media/0/realdata \
     --no-loop --no-hw-decoder --start <seconds> <route>'

tmux new-session -d -s opui \
  'cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -m openpilot.selfdrive.ui.ui'

# only if you want the live planner instead of the recorded plan
tmux new-session -d -s plan \
  'cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /tmp/run_plannerd.py'
```

Cleaning up:

```bash
for s in rp opui plan; do tmux kill-session -t $s; done
sudo systemctl start comma
```

## Why each part is the way it is

**`--no-hw-decoder` is not optional.** Without it replay parses the route fine, prints
`STATUS: playing` and the car fingerprint, and then dies:

```
terminate called after throwing an instance of 'std::runtime_error'
  what(): VIDIOC_STREAMON CAPTURE failed
```

That is the V4L2 hardware decoder. camerad does not reliably release it when the manager stops,
so replay cannot bring it up. Software decode costs some frame rate and nothing else.

**It must run under tmux, not `nohup` or `setsid`.** replay's console UI calls `initscr()`, which
needs a real pty. Without one it exits immediately with `Error opening terminal: unknown.` and
leaves an almost-empty log. tmux provides the pty *and* survives the ssh connection dropping,
which matters here — see below.

**`sudo systemctl stop comma` first.** replay publishes `can`, `pandaStates`, `managerState` and
more. With the manager running that is two publishers on one service, and msgq raises
`MultiplePublishersError`, which cascades into the whole stack going down.

**Blocking services.** `-b <service>` stops replay publishing it, so a live process can own it
instead. Block only what you are actually replacing:

- `-b longitudinalPlan` — replay the drive but let a live `plannerd` decide. This is how you see
  what the current planner *would* do against recorded inputs.
- Do **not** also block `selfdriveState`, `controlsState` or `carControl` unless you are running
  the processes that produce them. Nothing else here publishes those, and without them the UI
  has no engagement state and draws **"openpilot unavailable"** over a black screen.

**plannerd standalone needs `config_realtime_process` stubbed.** Outside the manager's cgroup the
core-pinning call is rejected with `OSError: [Errno 22] Invalid argument` and plannerd exits
before it ever starts. The stub above skips only the pinning.

## Choosing a segment

**The overlay is hidden while disengaged.** mici's renderer draws no lane lines, no path and no
lead chevron unless openpilot was engaged in the recording, so a disengaged window replays as
video with nothing on it. Pick a segment that is engaged for its whole minute.

To find one:

```python
# engaged fraction per segment
for evt in read_events(rlog):
    if evt.which() == 'selfdriveState' and evt.selfdriveState.enabled:
        ...
```

`--start` counts seconds into the **route**, not the segment: segment N begins at `N * 60`.

## Checking whether it is actually working

**`pgrep -f 'replay/replay'` lies.** The pattern appears in your own ssh command line, so it
matches the shell running the check and reports a process that is not there. This cost two wrong
conclusions in one session. Use the bracket trick, which cannot match itself:

```bash
pgrep -f '[r]eplay/replay' | wc -l
```

The real test is whether anything is on the bus, not whether a process exists:

```bash
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "
import cereal.messaging as messaging, time
sm = messaging.SubMaster(['carState', 'deviceState', 'longitudinalPlan'])
t0 = time.monotonic()
while time.monotonic() - t0 < 10: sm.update(200)
print('carState', sm.alive['carState'], 'started', sm['deviceState'].started)
print('trafficState', sm['longitudinalPlan'].trafficState)
"
```

For the UI to leave the "unavailable" screen it needs `deviceState.started` **and** ignition from
`pandaStates`; both come from the recording, so both appear once replay is genuinely playing.

## This device's wifi drops

Roughly ten reconnects in an hour, with ping swinging between 2ms and 170ms. Anything attached to
the ssh session dies with it — which is why the stack kept vanishing mid-setup and looked like
replay crashing. The tmux server is not attached to the connection, so sessions started with
`tmux new-session -d` survive; direct background children of the ssh command do not.

It is not memory: 2.6GB stays available throughout and `dmesg` shows no OOM kills.

## The Python stand-ins

`republish_route.py` and `extract_ui_window.py` replay recorded messages without the compiled
binary. They are useful when replay will not build, and for keeping memory down — reading an rlog
straight into memory alongside the UI was once enough to reboot this 3.5GB device, which is why
extraction and playback are two steps.

**They cannot show video.** They publish `roadCameraState` metadata, but the frames themselves go
over VisionIPC, which only the compiled replay (or camerad) feeds. Expect a black background with
the overlay drawn on it.

```bash
# 25s window starting 20s into the segment
PYTHONPATH=. python3 tools/replay/extract_ui_window.py <rlog.zst> /data/tmp/w.pkl 25 20

# play it back, leaving longitudinalPlan to a live plannerd
PYTHONPATH=. python3 tools/replay/republish_route.py /data/tmp/w.pkl --loop --block longitudinalPlan
```

Without the trailing offset, `extract_ui_window.py` picks the first window where openpilot is
engaged *and* has a lead. That is right for checking the chevron and wrong for a traffic light,
which by definition has no lead — the search runs straight past the moment you wanted.

## Building replay on the device

It is not in the default build here. `SConstruct` puts `tools/replay` behind
`GetOption('extras')`, which defaults off on comma hardware with no flag to turn it back on --
only `--minimal` to turn it further off. So:

```bash
scons --replay -j4 openpilot/tools/replay      # binary lands in tools/replay/replay
```

The binary predates that option in some checkouts, having been built elsewhere and copied. If it
is deleted it does not come back without `--replay`, and `scons tools/replay` will cheerfully say
`is up to date` about a file that is not there — that is the alias for the *output directory*,
not the program.

**Run it with the venv on PATH.** replay shells out to python for its downloader, and the system
python3 has no `zstandard`. Without it the TUI still says `STATUS: playing` and sits at
`SPEED: 0.00 m/s` forever, having failed to read a single log:

```bash
export PATH=/usr/local/venv/bin:$PATH
```

**`-b` takes one comma-separated list.** Repeating the flag keeps only the last one, so
`-b carState -b carParams` blocks carParams alone and leaves replay publishing carState -- which
then fights whatever you started to own it, and that process dies with
`MultiplePublishersError`. Write `-b carState,carParams,...`.

**Start replay last.** Bringing up card, the UI and modeld takes 20-30s, and the recording plays
on while you do it, so the moment you wanted is gone before anything is watching.

## Running a live model over a recording

replay feeds camera frames over VisionIPC, so modeld can run on a recorded drive and produce
`modelV2` from whatever model is selected. That is how to compare driving models against the same
road without driving it again.

```bash
# modeld needs the same realtime stub plannerd does
cat > /tmp/run_modeld.py <<'EOF'
import openpilot.common.realtime as rt
rt.config_realtime_process = lambda *a, **k: None
from openpilot.selfdrive.modeld.modeld import main
main()
EOF

# replay FIRST -- modeld attaches to its VisionIPC server and cannot start before it exists
tmux new-session -d -s rp '... replay ... -b modelV2,drivingModelData,cameraOdometry <route>'
sleep 3
tmux new-session -d -s md '... python3 /tmp/run_modeld.py'
```

Block all three of modeld's outputs, not just `modelV2`.

**Restarting replay silently kills modeld.** The vision stream goes away and modeld stops
producing without saying so -- `modelV2` simply goes not-alive. Restart both, in that order.

**Model loading costs 8-13s** and replay is already playing during it, so anything in the first
~15 seconds of the route cannot be captured this way. Seeking back does not help: `M` sent to the
TUI moves *forward* despite `shift+m` being documented as -60s. Pick a moment far enough in, or
start replay at an offset that puts it there once the model is up.

To switch models, write the `DrivingModel` param and restart the pair.
