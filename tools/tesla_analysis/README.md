# Tesla HW1 log analysis

Scripts written while root-causing the Tesla Model X HW1 work: the gap knob, stock autopark,
cooperative steering, and the launch acceleration ceiling. They read recorded route segments and
print what the car and openpilot were actually doing, which is how every conclusion in those
commits was reached.

Analyse logs on a workstation, not on the device -- the comma has ~3.5GB of RAM and is busy
running the car.

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
| `analyze_fail.py <seg>...` | Broad first look: which feature flags were actually on, EPAS histograms, onroad events, and every state transition in order |
| `window.py <from> <to> <seg>...` | Everything that happened between two timestamps, in order. The workhorse for pinning down a 0.1s cause |
| `who_sends.py <from> <to> <route>` | Which module put which frames on which bus. Finds two masters writing one arbitration id |
| `scan_can.py <seg>...` | Every (bus, address) seen, with rates. Start here on an unfamiliar bus |
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
