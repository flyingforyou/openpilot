# Bluetooth setup (Ubuntu)

vLinker/ELM327 dongles come in two kinds, and only one works with `/dev/rfcomm0`:

| Kind | How it shows up | Works with this tool |
|---|---|---|
| Bluetooth **Classic** (SPP/RFCOMM), e.g. vLinker MC/FS/BM | `bluetoothctl` lists it; `sdptool browse` shows "Serial Port" | yes |
| Bluetooth **LE** only (e.g. vLinker MC+, BM+, FD+ when used from iOS) | GATT services, no Serial Port profile | no -- needs a different transport |

Check which one you have: `bluetoothctl scan on`, note the MAC, then `sdptool browse <MAC>` (package
`bluez`). A "Serial Port" record with a `Channel:` means Classic SPP. If there is none, you have an
LE-only unit; `bluetoothctl` -> `connect <MAC>` -> `menu gatt` -> `list-attributes` shows its
characteristics, which is the information a future LE transport would need.

```bash
sudo apt install bluez python3-venv
bluetoothctl
  power on
  agent on
  scan on                 # find the dongle, e.g. 00:1D:A5:68:98:8B
  pair 00:1D:A5:68:98:8B  # PIN is usually 1234 or 0000
  trust 00:1D:A5:68:98:8B
  scan off
  quit
sdptool browse 00:1D:A5:68:98:8B | grep -A6 "Serial Port"     # note "Channel: N"
sudo rfcomm bind /dev/rfcomm0 00:1D:A5:68:98:8B 1             # use the channel from SDP, not a guess
ls -l /dev/rfcomm0
sudo usermod -aG dialout $USER                                # then log out/in
```

Release with `sudo rfcomm release /dev/rfcomm0`. `rfcomm` is deprecated in newer BlueZ and may need
`sudo apt install bluez-deprecated`.

Baud rate is ignored over Bluetooth SPP; 115200 is passed only because pyserial wants a number.
