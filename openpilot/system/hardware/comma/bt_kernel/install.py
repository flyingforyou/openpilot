#!/usr/bin/env python3
"""Install the Bluetooth kernel (boot-bt.img.xz) on the inactive A/B slot and make it the next boot.

Written the way an AGNOS update is: the other slot gets stock 19.6 for every partition (downloaded
from commadist, skipped where it already matches) except boot, which gets this image; then that
slot is set active but not yet successful. If the new kernel does not come up, the bootloader runs
out of retries and goes back to the current slot, which is never touched. launch_chffrplus.sh marks
the slot successful once openpilot starts.

  sudo PYTHONPATH=/data/openpilot /data/openpilot/.venv/bin/python \
    /data/openpilot/openpilot/system/hardware/comma/bt_kernel/install.py

Back to stock afterwards: sudo abctl --set_active <old slot> && sudo reboot
"""
import hashlib
import json
import logging
import lzma
import os
import subprocess
import sys

from openpilot.common.basedir import BASEDIR
from openpilot.common.hardware.comma import agnos

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE = os.path.join(HERE, 'boot-bt.img.xz')
META = os.path.join(HERE, 'boot-bt.json')
MANIFEST = os.path.join(BASEDIR, 'openpilot/system/hardware/comma/agnos.json')


def fail(msg: str) -> None:
  print(f"not installing: {msg}")
  sys.exit(1)


def main() -> None:
  if os.geteuid() != 0:
    fail("needs root (sudo)")
  meta = json.load(open(META))
  manifest = json.load(open(MANIFEST))
  boot = next(p for p in manifest if p['name'] == 'boot')

  version = open('/VERSION').read().strip()
  if version != meta['agnos']:
    fail(f"built for AGNOS {meta['agnos']}, this device runs {version}")
  # the rest of the target slot comes from this manifest, so it has to be the AGNOS the kernel was built for
  if boot['hash_raw'] != meta['stock_boot_sha256']:
    fail("agnos.json is not the AGNOS this kernel was built from; rebuild it (build.sh)")
  if os.path.exists('/dev/btpower'):
    print("this kernel already has Bluetooth (/dev/btpower)")
    return

  image = lzma.decompress(open(IMAGE, 'rb').read())
  if hashlib.sha256(image).hexdigest() != meta['sha256']:
    fail("boot-bt.img.xz does not match boot-bt.json")

  current = subprocess.check_output(['abctl', '--boot_slot'], encoding='utf-8').strip()
  target = agnos.get_target_slot_number()
  print(f"booted from {current}; writing slot {agnos.slot_number_to_suffix(target)}")

  logging.basicConfig(level=logging.INFO)
  subprocess.run(['abctl', '--set_unbootable', str(target)], check=True)
  for p in manifest:
    if p['name'] != 'boot':
      agnos.flash_partition(target, p, logging)

  path = agnos.get_partition_path(target, boot)
  with open(path, 'r+b') as f:
    f.write(image)
    f.flush()
    os.fsync(f.fileno())
  with open(path, 'rb') as f:
    if hashlib.sha256(f.read(len(image))).hexdigest() != meta['sha256']:
      fail(f"{path} read back differently; slot {target} left unbootable")
  logging.info(f"wrote {len(image)} bytes to {path}")

  agnos.swap(MANIFEST, target, logging)
  print(f"\nready: reboot to start the Bluetooth kernel on slot {agnos.slot_number_to_suffix(target)}.")
  print(f"to go back: sudo abctl --set_active {1 - target} && sudo reboot")


if __name__ == "__main__":
  main()
