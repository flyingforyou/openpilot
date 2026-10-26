# Limitations

- **Nothing has been run on a real car or a real dongle.** Tests use FakeSerial and synthetic data.
- Unknown: which bus/bitrate the Tesla adapter cable exposes on a 2016 Model X; whether any ECU
  there answers UDS to a non-Tesla tool; whether your dongle is truly silent.
- Clone ELM327 firmware varies (`?` for unsupported AT commands, wrong `OK`s). `probe` reports what it
  sees, not what is true.
- Bluetooth + ELM327 drops frames on busy buses (`BUFFER FULL`); drops are visible but not recoverable.
  Timestamps are host arrival time, not bus time.
- Only 11-bit/29-bit plain CAN protocols 6-9; no user-defined (`ATPB`) bitrates (some Tesla buses use
  non-standard speeds -> need CANable/SocketCAN).
- 0x78 pending is handled by the dongle on the live path; there is no multi-request session logic.
- MCU2 retrofit does not tell you anything about BMS/gateway versions; profiles are per evidence.
- LE-only dongles are unsupported (see bluetooth_setup.md).
- Future: a `socketcan.py` transport (python-can + CANable) would remove the dongle limits; the
  `Elm327` class is the only dongle-specific piece.
