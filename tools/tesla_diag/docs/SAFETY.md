# Safety design

1. **Offline by default.** `replay`, `decode`, `uds-read --dry-run` never open a port.
2. **Listen-only live.** `sniff` sends only AT commands and one bare CR to stop `ATMA`. It first needs
   `ATCSM1` to answer `OK`; otherwise it exits before `ATMA`. Datasheet (ELM327DS, "CSM0 and CSM1"):
   silent monitoring means the chip generates no ACK bits; default comes from PP 21. **Limit:** clones
   may answer `OK` without being silent, and nothing on the serial side can prove the dongle does not
   ACK. Treat a clone as "probably silent".
3. **Live UDS needs every one of:** a profile with `verified_on_vehicle`, `allow_live`, `evidence_url`,
   `hardware_or_firmware_evidence`, `tx_id`/`rx_id`, `bitrate`; the exact request hex in that profile's
   `supported_service_did`; `--allow-uds-read` and `--confirm-parked`. Checked in `UdsClient.preflight`
   before any port is opened.
4. **Service allowlist in two places** (`uds/client.py`, `transport/elm327.py`): only 0x22 and 0x19
   (subfunctions 0x01/0x02/0x0A). 0x10, 0x11, 0x14, 0x27, 0x2E, 0x2F, 0x31 (including `31 03`), 0x34-0x38, 0x3D,
   0x85 are refused even if a profile lists them, and even if code calls `Elm327.uds_request` directly.
5. **One request at a time**, budget `--max-requests` (default 1), zero retries, `--p2` timeout, an
   audit line for every send/refusal in `logs/uds_audit.jsonl`.
6. **Framing:** live reads use `ATCAF1`/`ATCFC1` so the dongle frames and sends flow control itself;
   the tool never adds PCI bytes in that path (no double framing). Sniffing uses `ATCAF0`.
7. **Not implemented on purpose:** address scanning, brute force, seed/key handling, any control.
   The public BMS flows (`10 03`, `27 05/06`, `31 01 ...`) exist in Battery-Emulator; none are reproduced.
8. Use only parked (P), in a safe place. VINs are masked in `uds-read` output unless `--show-vin`.
9. Mutation check performed: removing the profile-membership gate, the two-flag gate, or the silent-monitoring
   gate each makes tests fail. (Removing only the dongle-layer allowlist does not, because the
   forbidden-service list independently blocks the same services -- two layers by design.)
