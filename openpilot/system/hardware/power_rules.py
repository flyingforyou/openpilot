"""Shutdown rules that need no Params, so they can be tested off-device.

power_monitoring imports Params, which needs libparams_c.so and therefore only exists on the
device. The decision itself is pure, so it lives here.
"""

# Minutes offroad on internal battery before shutting down. 0 disables the rule.
DEFAULT_OFFROAD_SHUTDOWN_MIN = 30


def battery_only_shutdown(started_seen: bool, in_car: bool, ignition: bool,
                          offroad_time_s: float, timeout_min: float) -> bool:
  """True when the car has gone to sleep and taken the harness with it.

  Upstream's whole shutdown path is gated on `in_car`, which is
  `harnessStatus != notConnected` -- i.e. it needs the panda to still be powered. That holds on a
  car whose harness sits on a permanent 12V feed: the panda keeps reporting, the offroad power
  budget drains, and the device shuts down to protect the car's battery.

  This car cuts the harness when it sleeps. The panda dies with it, so harnessStatus reads
  notConnected and pandaType unknown; `in_car` goes false and every shutdown condition above it is
  masked out. peripheralState.voltage is None too, so the power budget never integrates and
  CarBatteryCapacity sits pinned at full. The device then runs on its internal battery until it is
  flat -- observed at 64% with dc/usb/pc_port all offline while parked.

  Requiring `started_seen` is what makes this safe: the device only shuts itself down this way if
  it actually went onroad during this power cycle, so a bench unit that never drove is untouched,
  as is a car whose harness stays powered (there `in_car` remains true and upstream's own rules,
  which protect the car battery rather than this one, keep handling it).
  """
  if timeout_min <= 0:
    return False
  if ignition or in_car or not started_seen:
    return False
  return offroad_time_s > timeout_min * 60.0
