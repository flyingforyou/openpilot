# CAN vs ISO-TP vs UDS

- **CAN**: frames of id + up to 8 bytes. Broadcast signals (BMS voltage, SOC...) are plain CAN frames
  decoded by bit layout (`can/decoder.py`). No request needed; this is what `sniff` records.
- **ISO-TP (15765-2)**: transport on top of CAN for payloads > 7 bytes. First byte high nibble:
  0 single, 1 first, 2 consecutive (seq 1..15,0..), 3 flow control. `uds/isotp.py` parses/simulates
  this offline. Live, the ELM327 does it (`ATCAF1`).
- **UDS (14229)**: request/response services carried in ISO-TP. Positive reply = SID+0x40; negative =
  `7F SID NRC`; NRC 0x78 means "keep waiting" (the ELM handles it; offline parser flags it as pending).

Status of examples:
| Item | Status |
|---|---|
| `22 F1 90` (VIN) | standard-shape example. No evidence any Tesla ECU on your bus answers it. Dry-run + fake only. |
| `0x602`/`0x612` | seen in Battery-Emulator (`TESLA-LEGACY-BATTERY.cpp:241`) and PiCanScanMyTesla (`teslamon.py:41-42`) as BMS request/response on the pack-internal CAN. **Candidate only**; not shown to be reachable from the car's diagnostic connector. |
| `0x19 02 FF` | standard; subfunction meaning checked against ISO 14229-1 (02 = reportDTCByStatusMask). Not tried on a car. |
| CAN ids 0x102/0x302/0x382 | community layouts (wk057); only used as `community-unverified` in the example profile. |
