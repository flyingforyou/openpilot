# Tesla HW1 log analysis

Scripts written while root-causing the Tesla Model X HW1 work: the gap knob, stock autopark,
cooperative steering, and the launch acceleration ceiling. They read recorded route segments and
print what the car and openpilot were actually doing, which is how every conclusion in those
commits was reached.

Analyse logs on a workstation, not on the device -- the comma has ~3.5GB of RAM and is busy
running the car.

## Start here: the playbook

`playbook.py` is the one-shot longitudinal report. Point it at routes, read the diagnosis -- no
per-question script wrangling:

```bash
export OP_LOG_ROOT=~/op-logs
PYTHONPATH=<repo> tools/tesla_analysis/playbook.py                 # every route in OP_LOG_ROOT
PYTHONPATH=<repo> tools/tesla_analysis/playbook.py 00000013 00000016
PYTHONPATH=<repo> tools/tesla_analysis/playbook.py 00000016 --json # machine-readable
```

Per route it prints six sections, each with the numbers and a one-line 진단 (diagnosis):

| # | section | answers |
|---|---|---|
| 1 | car ID | which physical Model X (0x398/0x359 config broadcast) + wall-clock time |
| 2 | gap usage | engaged time at each steering-wheel gap 1-7, all vs city (<58 km/h) |
| 3 | FCW | "Emergency Braking" episodes, rate per gap, onset context -- flags a gap that trips FCW |
| 4 | stop distance | resting dRel behind stopped vs decelerating leads -- flags stopping too close |
| 5 | headway | measured following headway (s) per gap, median + closest-15% |
| 6 | accel response | on a lead pull-away: ceiling-limited? throttle-gated? or jerk-limited sluggish? |

It reads each segment in its own subprocess (`--worker`), so one truncated `rlog.zst` -- which can
crash capnp with an uncatchable C++ terminate -- is skipped instead of killing the whole run. The
known-car fingerprints in `CARS` and the accel ceiling in `A_CRUISE_MAX_*` are the values that were
in effect for this project; update them if the build has moved on.

The single-purpose scripts below are still there for drilling into one question by hand.

## Setup

Copy the segments over first, keeping the `<route>--<n>/rlog.zst` layout:

```bash
export OP_LOG_ROOT=~/op-logs
mkdir -p $OP_LOG_ROOT
for seg in $(ssh comma@<ip> 'cd /data/media/0/realdata && ls -d <route>--*/ | tr -d /'); do
  mkdir -p $OP_LOG_ROOT/$seg
  scp comma@<ip>:/data/media/0/realdata/$seg/rlog.zst $OP_LOG_ROOT/$seg/
done
```

Then run anything here with `PYTHONPATH` pointing at the repo root. `OP_SCRATCH` (default
`/tmp/op-analysis`) is where segments get decompressed; capnp's streaming reader needs a real fd,
so they cannot be piped.

## What each one answers

| script | question |
|---|---|
| `playbook.py [route...]` | One-shot longitudinal report (car ID, gap usage, FCW, stop distance, headway, accel response) with a 진단 per section. Start here; see above |
| `analyze_fail.py <seg>...` | Broad first look: which feature flags were actually on, EPAS histograms, onroad events, and every state transition in order |
| `window.py <from> <to> <seg>...` | Everything that happened between two timestamps, in order. The workhorse for pinning down a 0.1s cause |
| `who_sends.py <from> <to> <route>` | Which module put which frames on which bus. Finds two masters writing one arbitration id |
| `scan_can.py <seg>...` | Every (bus, address) seen, with rates. Start here on an unfamiliar bus |
| `find_battery.py <seg>...` | Brute-forces every bit field on a bus looking for pack current/voltage (they track motor torque) and SOC/energy (slow one-way drift). For when no DBC names the BMS |
| `torque_measure.py <route>...` | Driver torque distribution against handsOnLevel, and the torque at which the EPS trips |
| `inhibit_episodes.py <seg>...` | How long EPAS stayed inhibited, why, and what openpilot was doing during it |
| `find_handson.py <seg>...` | handsOnLevel / eacStatus / eacErrorCode histograms, to find drives with real takeovers |
| `autopark_offer.py <route>...` | Windows where the car offered autopark, and whether openpilot was engaged through them |
| `gap_sizes.py` | How sparse the stock module's bus requests are -- what sizes a hand-back timeout |
| `replay_coop.py <route> <from> <to>` | Replays a recorded override through the cooperative steering controller |

## A warning about foreign signal databases

`talas9/tesla_can_signals` was used as a cross-reference. Only its `CH` bus matches this car's
generation: `STW_ANGL_STAT` at 0x003 and `DI_torque2` at 0x118 agree, while its `ETH` bus puts
entirely different messages at our addresses and will produce confident nonsense. Check a known
signal before trusting any of it.
