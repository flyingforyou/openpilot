# Cluster fixes of 2026-08-12 — what to verify

Two commits landed today and neither has been on the car. Both are observable in a single drive.
This is written so the verification does not have to re-derive any of the reasoning.

| commit | what it changes |
|---|---|
| `d52cac80c` | forward every `DAS_object` frame, not only the relabelled ones |
| `8a3f7f5ec` | tell openpilot's own stalk presses from the driver by direction, not arrival |

Prerequisite: `scons` on the device, and **the panda must reflash** — `d52cac80c` does not touch
panda, but the `CARS_AS_TRUCKS` TX allowlist entry from the earlier work has to actually be on the
board. Compare the firmware signature before assuming it is.

---

## 1. `d52cac80c` — ghost vehicles

**Symptom it fixes.** Cars stayed drawn on the cluster after they were gone.

**Cause.** Panda blocks the factory's `DAS_object` (0x309) outright while cars-as-trucks is on, and
the car port only re-sent frames it had changed. An emptying slot has no `CAR` left to relabel, so
`substitute_type` returned `None` and the frame was dropped — and a dropped frame does not reach
the cluster as "nothing there", it reaches it as nothing at all. Same for the `ROAD_SIGN` and
`VEHICLE_HEADINGS` groups, which have no vehicle type to substitute and so disappeared entirely.

Reproducing those groups needs a byte-exact repack, which is why the commit also adds
`DAS_objSpareBit6` and `DAS_objSpareBit37` to the DBC: those bits belonged to no signal, and bit 37
is set in ~19% of heading frames, so a rebuilt frame silently dropped it.

**Verify on the road.** Follow a car, then change lanes or let it turn off. The icon should clear
within a second or so rather than persisting.

**Verify in the log.** Every group the factory sends on bus 2 should reappear on bus 0:

```
0x309 received on bus 2 (src == 2)     ~33 Hz, five groups rotating at ~6.7 Hz each
0x309 transmitted onto bus 0 (src 128) same rate, same group rotation
```

If the transmitted rate is materially below the received rate, frames are still being dropped.
Group id is `byte0 & 0x07`: `0 LEAD, 1 LEFT, 2 RIGHT, 3 CUTIN, 4 ROAD_SIGN, 5 HEADINGS`.

---

## 2. `8a3f7f5ec` — MAX speed ignored the road

**Symptom it fixes.** The posted limit changed 11 times across route `00000036` and the cluster's
MAX sat at 40 mph the whole way, while openpilot pressed the stalk 72 times to no effect.

**Cause.** With cluster sync on, openpilot drives the stalk itself and those writes come back
looking exactly like the driver turning it. The old guard asked whether the stalk had *arrived* at
the target, within one detent:

```python
if sync_cluster and abs(stalk - v_output) < SYNC_ECHO_TOLERANCE:
    return  # ours
```

It walks there a detent at a time, so every step along the way sat far from the target and read as
a driver override. An override replaces the map's target outright, so the first press of any
correction pinned the target to wherever the walk had reached — and the target then equalled the
stalk, so there was nothing left to correct. **The sync disabled the feature it existed to
display.**

**The fix** is in `_update_override` in `selfdrive/controls/lib/map_cruise.py`: a stalk moving
*toward* the map's target is not a disagreement with it, whoever turned it; moving away or past it
is. `SYNC_ECHO_TOLERANCE` is gone.

**Tests.** `selfdrive/controls/lib/test_map_cruise_override.py`, nine scenarios. Five fail if the
old logic is restored — worth confirming, since a test that passes either way proves nothing.

**Verify in the log.** These are the numbers that were wrong, measured on `00000036`:

| measurement | before | expected after |
|---|---|---|
| share of engaged time where `cruiseTarget` differs from `carState.vCruise` | 2% | materially higher |
| cluster within 2 kph of target *while the map overrides* | 34% | high |
| stalk presses / median gap | 72 @ 0.35 s | fewer, less continuous |
| `UP_5` / `DOWN_5` presses | 2 / 16 | more, once errors are allowed to grow |

The last row is the tell. `±5` is chosen when the error exceeds 5 mph, and the errors could never
get that large while the target was being pinned to the stalk.

---

## Signals, so none of this has to be rediscovered

```
cluster MAX      DI_state 0x368 bus 0, bits 48-55   DI_digitalSpeed
                 owned by the DI. Not writable — every address on every bus was scanned
                 against it and nothing else carries it. The stalk is the only way in.
                 DAS_setSpeed does NOT move it (observed carrying 66.4 with the cluster at 64.4).

stalk            STW_ACTN_RQ 0x45 bus 0, SpdCtrlLvr_Stat bits 0-5
                 0 IDLE, 16 UP_1, 32 DOWN_1, 4 UP_5, 8 DOWN_5

object list      DAS_object 0x309 bus 2, ~33 Hz, multiplexed on byte0 bits 0-2
                 byte0 bits 3-5 = DAS_objVehType, bit 7 = RelevantForControl
                 type: 0 UNKNOWN, 1 TRUCK, 2 CAR, 3 MOTORCYCLE, 4 BICYCLE, 5 PEDESTRIAN, 6 IPSO
                 an empty slot saturates: Dx = 127.5 m

speed limits     DAS_status 0x399 bus 2
                 DAS_fusedSpeedLimit bits 8-12, DAS_visionOnlySpeedLimit bits 16-20, scale 5, mph
```

The car is in **MPH**. Cluster readings of 64.4 / 72.4 / 80.5 / 120.7 kph are 40 / 45 / 50 / 75 mph.

### Stalk response, measured on 189 driver presses made while parked

The DI is reliable — an earlier claim that it drops presses was wrong.

| detent | presses | answered | lag | move |
|---|---|---|---|---|
| `UP_1` | 80 | 95% | 0.24 s | +1 |
| `DOWN_1` | 97 | 99% | 0.16 s | −1 |
| `UP_5` | 11 | 91% | 0.24 s | +5 |

Pressing faster does not lose any: 108 presses at gaps under 0.4 s were answered 100%.

---

## Settled, do not reopen

The cluster not drawing ordinary cars is **Tesla's 2026.26.1 firmware regression**, not ours. The
gate is `DAS_objVehType`: `TRUCK` and `MOTORCYCLE` draw, `CAR` (78.3% of all objects) does not.
Other AP1 owners report the same on Tesla Motors Club, thread 357723.

Eliminated by measurement, each one checked: radar health, camera fault flag, calibration, panda
forwarding, darkness, distance, lane position, `RelevantForControl`, openpilot itself (it
reproduces with the comma disconnected), windshield, car configuration.

Darkness and distance point the wrong way and are worth stating so they are not tried again: a car
and a motorcycle appeared in the same half-second window 101 times with only the motorcycle drawn,
the cars sat nearer (median 19 m) than the drawn trucks (41.5 m), and the darkest segment had the
highest draw rate.

## Claims made during the investigation that were wrong

Listed so they are not repeated:

- *"The DI drops stalk presses."* It answers 95-99%. That came from reading a filtered timeline by
  eye; the parked measurement contradicts it.
- *"The stalk cannot keep up with the target."* The arithmetic assumed single detents. With ±5 it
  tracks 23 kph/s against a 6.7 kph/s ramp.
- *"There is no curve speed, it has to be built."* It exists, derived from fleet spline speed in
  `map_cruise.py`.
- *"The cluster tracks the target 87% of the time."* Flattered by the 98% of samples where the
  target simply *was* the stalk value. Where the map actually overrode, it was 34%.
- *"`TeslaMapAutoSpeed` is off."* It was on.

## Still unverified

- Whether relabelling `CAR` as `TRUCK` restores the display in traffic. It draws in the centre lane
  — the one truck that was drawn sat at a median lateral offset of 0.0 m — but that is one truck,
  not a road full of relabelled cars.
- Whether the map target survives now that it is no longer pinned.
- Whether `±5` starts being used once errors are allowed to grow.
