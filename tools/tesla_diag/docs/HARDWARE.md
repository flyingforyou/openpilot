# Hardware and wiring assumptions

- Dongle: ELM327-compatible Bluetooth **Classic** (vLinker etc.). `probe` shows version and which
  commands this firmware accepts. Bring that output when reporting.
- Cable: the Tesla diagnostic connector -> OBD-II adapter cable you already use with Scan My Tesla.
  Which CAN bus its pins expose on a 2016 Model X, and at what speed, is **not established** in
  this project. The dongle's CAN pins are (normally) OBD pins 6/14; whether the cable routes the
  powertrain/chassis bus there needs the cable's pinout. Do not assume 500 kbps/11-bit: pick
  `--protocol` from the pinout or try listen-only protocols one at a time (wrong bitrate while
  silent is harmless; with `CSM0` it would not be, which is why `CSM0` is never set).
- The openpilot device in this repo has its own diagnostic page (`openpilot/system/btobd`,
  `/diag`) doing a similar listen-only job with the same ELM327 approach; that code is the closest thing
  to prior in-car use here, and it is also unverified for the diagnostic connector bus.
- 12 V: a dongle left connected drains the 12 V battery; unplug after use.
- Throughput: ELM327 over Bluetooth handles roughly a few hundred frames/s; a busy bus overruns it
  (`BUFFER FULL`). A CANable + SocketCAN is the upgrade path (see LIMITATIONS).
