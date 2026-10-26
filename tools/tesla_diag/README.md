# tesla-diag

Read-first CAN/UDS diagnostics for a Tesla Model S/X (2016 Model X in mind) with an ELM327-class
Bluetooth dongle (vLinker etc.) and the Tesla-diagnostic-connector-to-OBD adapter cable.

**Status: not verified on any car.** Everything here is tested against fakes and synthetic data. No
claim of "works on a 2016 Model X" exists until you send a real capture (see the bottom).

What it does today
- `probe`: checks the dongle's AT layer. Sends no OBD/UDS request.
- `sniff`: passive capture to JSONL. Refuses unless the dongle confirms `ATCSM1` (silent monitoring).
- `replay` / `decode`: offline ID statistics, per-second counts, CSV/candump export, signal decoding
  from a profile (with each signal labelled by source and verification status).
- `uds-read`: builds 0x22 / 0x19 requests and parses answers. `--dry-run` is offline. A live read
  needs a verified profile **and** `--allow-uds-read` **and** `--confirm-parked` (see docs/SAFETY.md).

What it does not do: write, clear, reset, unlock, flash, routine control, address scanning, brute
force. 0x10/0x11/0x14/0x27/0x2E/0x2F/0x31 and transfer services are blocked at two independent layers.

## Install (Ubuntu)
```bash
cd tools/tesla_diag
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[test]'
pytest -q                      # no hardware needed
```
Bluetooth pairing: `tesla_diag/transport/bluetooth_setup.md`. Then `/dev/rfcomm0` exists.

## Use
```bash
# 1. dongle AT layer only
tesla-diag probe --port /dev/rfcomm0
# 2. listen. Protocol is mandatory: 6 = CAN 11-bit/500k, 8 = 11-bit/250k, 7/9 = 29-bit. The bus the
#    adapter cable exposes is NOT known here -- choose it from your cable/wiring info, not from this example.
tesla-diag sniff --port /dev/rfcomm0 --protocol 6 --seconds 60 --out logs/capture.jsonl
# 3. look at it offline
tesla-diag replay logs/capture.jsonl --summary --bucket 1
tesla-diag decode logs/capture.jsonl --profile tesla_diag/profiles/example_unverified.yaml
# no hardware at all:
tesla-diag replay fixtures/synthetic_can.jsonl --summary
tesla-diag uds-read --dry-run --profile tesla_diag/profiles/example_unverified.yaml --did F190
```

Expected `probe` output (a healthy clone):
```
AT-layer probe (no OBD/UDS request was sent)
  version      : ELM327 v1.5
  battery volt : 12.4V
  ATCSM1   : ok
  silent monitoring (ATCSM1): CONFIRMED
```

Troubleshooting
| Message | Meaning / fix |
|---|---|
| `cannot open /dev/rfcomm0` | not bound, or you are not in `dialout`; redo bluetooth_setup.md |
| `no prompt from dongle` | wrong port, LE-only dongle, dongle asleep, or it needs `ATZ` time; retry once |
| `ATCSM1 -> unsupported` | dongle can't do silent monitoring: sniff refuses (by design) |
| `0 frames` | wrong `--protocol` for the bus, adapter pins wrong, car asleep |
| `BUFFER FULL` | Bluetooth/ELM can't keep up with the bus; frames lost -- filter or accept gaps |
| `?` on a command | that firmware doesn't implement it; `probe` lists which |

Limits you should know: docs/LIMITATIONS.md. Research basis: docs/RESEARCH.md.
