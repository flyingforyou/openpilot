# Research log (checked 2026-10-26)

Opened and read vs. only listed is stated for each item. License matters: nothing was copied from any repo;
only facts (ids, layouts) with attribution.

| Source | Commit / version | License | What was actually checked |
|---|---|---|---|
| jetpax/PiCanScanMyTesla | 4e7f0e79fc (2026-07-17) | MIT | **Read** `teslamon.py`: header says it reads the *battery-internal* CAN via socketcan `can0` (l.4,25); BMS UDS `UDS_REQ_ID=0x602`, `UDS_RSP_ID=0x612` (l.41-42); `_isotp_send` hand-rolls ISO-TP on that id (l.~250); the file contains a security key constant and fault routines via `31 01/03`. Model S 85/90 pack; **no Model X claim**. |
| dalathegreat/Battery-Emulator | 24ed864420 (2026-10-25) | GPL-3.0 | **Read** `Software/src/battery/TESLA-LEGACY-BATTERY.cpp`: decodes 0x102 (l.158), 0x202 (l.226), 0x212 (l.136); BMS UDS response 0x612 handled at l.241 (`7F` rejects logged); sends via `TESLA_602` the sequence `02 10 03`, `02 27 05`, `30 ..`, `10 12 27 06 <key>`, `04 31 01 04 01` (l.296-345) -- session, security access, routine start. This is a **bench battery emulator** flow; not run or ported here (rule: no live 0x10/0x27/0x31). |
| NetherlandsForensicInstitute/teslalogs | 150e1906a7 (2023-09-27) | Apache-2.0 | **Read** `writeup/Gateway_mcu1_mcu2.md` headings/grep: gateway firmware/logs on a reference Model S; no diagnostic ECU addresses for Model X found. Not used for ids. |
| outlandnish/tm3diag | b22d8a613a (2026-10-12) | GPL-3.0 | **Tree listed only** (`uds_local/client.py`, `scripts/<ecu>/`, `tm3uds.py`); contents not read. Model 3/Y focused; no Model X claim. |
| sampepin86/UDS-TESLA | f872d73a65 (2026-06-04) | **none declared** | **Tree listed only** (`uds/client.py`, `uds/nodes/nodes.json`, ESP32 bridge, PCAN transports, `tesla_v3_download.py`). No license -> not to be copied. Contains download/bootloader sequences: out of scope. |
| ELM327 datasheet (ELM327DS.pdf) | Elm Electronics | - | **Read** CSM0/CSM1 text: ELM327 is silent by default; CSM0 makes it ACK and "will disturb the flow of data" on a wrong baud; default via PP 21. ATMA stops on any character. |
| Tesla service site | - | - | **Not fetched** (needs login/JS). No claim derived. |
| wk057 CAN deciphering v0.1 (2016-01-15) | via this repo's `openpilot/system/btobd/tesla_sx.py` | community | Layout of 0x102/0x302/0x382 taken from that file's docstring/tests, not re-fetched. Model S. |

## Findings
- `0x602/0x612`: both repos above use them for the BMS on the **pack-internal bus** (bench use).
  Whether a 2016 Model X exposes them on the diagnostic-connector bus is unproven -> candidate, profile left empty.
- Public flows use `10 03` / `27` / `31`. Seen, documented, deliberately not implemented.
- CAN bitrate/id length: only the dongle protocols 6-9 are offered; none is claimed correct for the car.
- Compatibility conclusion: no source confirms Model X 2016 on the OBD adapter-cable bus. The safe, working
  deliverable is passive capture + offline decode; UDS is gated until a capture/probe from the car exists.
