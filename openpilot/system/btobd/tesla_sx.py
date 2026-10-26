"""Decoders for the 2012-2021 Model S/X powertrain broadcasts a diagnostic-port dongle can hear.

This is what Scan My Tesla shows on these cars: it does not query anything, it puts an ELM327 into
CAN monitor mode and reads frames the BMS and drive units broadcast anyway. None of these IDs are
on the buses the comma harness taps (checked 2026-10-26: 0x302/0x382 there are radar tracks), so
the dongle is the only way to get them.

Sources, both community reverse engineering, neither official:
  - wk057 (Jason Hughes), "Tesla Model S CAN Bus Deciphering" v0.1, 2016-01-15. Every message
    below that has an example frame in that document is pinned by it in tests/test_tesla_sx.py.
  - OVMS vehicle_teslamodels.cpp for 0x6F2 (brick voltages / module temperatures), 0x306 and 0x338.
Firmware changes can move fields, so the page shows the raw frames next to the decoded values.
"""
from __future__ import annotations


def _s(value: int, bits: int) -> int:
  """Two's complement of an unsigned `bits`-wide field."""
  return value - (1 << bits) if value & (1 << (bits - 1)) else value


def bms_hv(d: bytes) -> dict:
  """0x102 BMS current and voltage, ~100 Hz.

  Current is 15-bit signed in bytes 3/2 (bit 6 of byte 3 the sign, bit 7 unknown), 0.1 A, positive
  into the pack (regen/charging). wk057: 8-byte frames (dual motor) carry a +10000 offset, 6-byte
  frames (single motor) none. Bytes 5/4 are the unfiltered current at double resolution.
  """
  out = {'packVoltage': (d[0] | d[1] << 8) * 0.01}
  if len(d) >= 4:
    raw = (d[2] | d[3] << 8) & 0x7FFF
    current = _s(raw, 15) - (10000 if len(d) >= 8 else 0)
    out['packCurrent'] = current * 0.1
    out['packPower'] = out['packVoltage'] * out['packCurrent'] / 1000.0
  if len(d) >= 8:
    out['negTerminalTemp'] = (d[6] | (d[7] & 0x07) << 8) * 0.1
  return out


def bms_soc(d: bytes) -> dict:
  """0x302 state of charge, ~1 Hz."""
  return {'socMin': (d[0] | (d[1] & 0x03) << 8) / 10.0,
          'socUI': ((d[1] >> 2) | (d[2] & 0x0F) << 6) / 10.0}


def bms_energy(d: bytes) -> dict:
  """0x382 energy status, ~1 Hz. kWh, 0.1 resolution. energyBuffer is the anti-brick reserve."""
  return {
    'nominalFullPackEnergy': (d[0] | (d[1] & 0x03) << 8) * 0.1,
    'nominalEnergyRemaining': ((d[1] >> 2) | (d[2] & 0x0F) << 6) * 0.1,
    'expectedEnergyRemaining': ((d[2] >> 4) | (d[3] & 0x3F) << 4) * 0.1,
    'idealEnergyRemaining': ((d[3] >> 6) | d[4] << 2) * 0.1,
    'energyToChargeComplete': (d[5] | (d[6] & 0x03) << 8) * 0.1,
    'energyBuffer': ((d[6] >> 2) | (d[7] & 0x03) << 6) * 0.1,
  }


def bms_power_limits(d: bytes) -> dict:
  """0x232 battery power limits, ~10 Hz. kW."""
  return {'maxRegenPower': (d[0] | d[1] << 8) / 100.0,
          'maxDischargePower': (d[2] | d[3] << 8) / 100.0}


def _drive_power(d: bytes, prefix: str, has_regen: bool) -> dict:
  out = {
    f'{prefix}Inverter12V': d[0] / 10.0,
    f'{prefix}Dissipation': d[1] * 125.0,                       # W
    f'{prefix}MechPower': _s(d[2] | (d[3] & 0x07) << 8, 11) / 2.0,  # kW
    f'{prefix}StatorCurrent': d[4] | (d[5] & 0x07) << 8,        # A
    f'{prefix}DrivePowerMax': (((d[6] & 0x3F) << 5) + ((d[5] & 0xF0) >> 3)) + 1,  # kW
  }
  if has_regen:
    out[f'{prefix}RegenPowerMax'] = d[7] * 4 - 200            # kW
  return out


def rear_power(d: bytes) -> dict:
  """0x266 rear drive unit power, ~10 Hz."""
  return _drive_power(d, 'rear', True)


def front_power(d: bytes) -> dict:
  """0x2E5 front drive unit power, ~10 Hz. Same layout as 0x266 without regenPowerMax."""
  return _drive_power(d, 'front', False)


GEARS = {1: 'P', 2: 'R', 3: 'N', 4: 'D'}


def rear_torque_status(d: bytes) -> dict:
  """0x116 rear drive torque estimate, gear and speed, ~100 Hz."""
  return {'rearTorqueEstimate': _s(d[0] | (d[1] & 0x0F) << 8, 12) / 2.0,   # Nm
          'gear': GEARS.get((d[1] & 0x70) >> 4, '?'),
          'vehicleSpeedMph': ((d[2] | (d[3] & 0x0F) << 8) - 500) / 20.0}


def rear_drive_info(d: bytes) -> dict:
  """0x106 rear motor RPM and pedal, ~100 Hz."""
  return {'rearMotorRpm': _s(d[4] | d[5] << 8, 16), 'pedalPos': d[6] * 0.4}


def rear_torque_measured(d: bytes) -> dict:
  """0x154 rear drive measured torque and the two pedal sensors, ~100 Hz."""
  return {'rearTorqueMeasured': _s(d[5] | (d[6] & 0x1F) << 8, 13) * 0.25,
          'pedalPosA': d[2] * 0.4, 'pedalPosB': d[3] * 0.4}


def front_torque_measured(d: bytes) -> dict:
  """0x1D4 front drive measured torque, ~100 Hz."""
  return {'frontTorqueMeasured': _s(d[5] | (d[6] & 0x1F) << 8, 13) * 0.25}


def dcdc(d: bytes) -> dict:
  """0x210 DC-DC converter, ~10 Hz."""
  return {'dcdcInputPower': d[3] * 16.0, 'dcdcOutputCurrent': float(d[4]),
          'dcdcOutputVoltage': d[5] / 10.0, 'dcdcOutputPower': d[4] * d[5] / 10.0,
          'dcdcInletTemp': _s(d[2], 8) * 0.5 + 40}


def lifetime_energy(d: bytes) -> dict:
  """0x3D2 lifetime charge/discharge energy, ~1 Hz. kWh."""
  return {'kwhDischargeTotal': int.from_bytes(d[0:4], 'little') / 1000.0,
          'kwhChargeTotal': int.from_bytes(d[4:8], 'little') / 1000.0}


def battery_odometer(d: bytes) -> dict:
  """0x562 battery odometer, ~0.1 Hz. Miles of this pack, not necessarily the car."""
  return {'batteryOdometerMi': int.from_bytes(d[0:4], 'little') / 1000.0}


def drive_temps(d: bytes) -> dict:
  """0x306 inverter and motor temperature (OVMS). degC."""
  return {'inverterTemp': d[1] - 40, 'motorTemp': d[2] - 40}


def range_status(d: bytes) -> dict:
  """0x338 MCU range (OVMS). Miles and Wh/mi."""
  return {'rangeIdealMi': d[0] | d[1] << 8, 'rangeEstMi': d[2] | d[3] << 8,
          'consumptionWhMi': d[4] | d[5] << 8}


def bricks(d: bytes) -> tuple[int, list[int]] | None:
  """0x6F2 one of 32 multiplexed frames: index in byte 0, four 14-bit values in bytes 1-7.

  Indexes 0-23 are brick voltages (96 = 24x4, 0.000305 V), 24-31 module temperatures
  (14-bit signed, 0.0122 degC). Frames with a 0 or saturated value are not yet valid.
  """
  if len(d) < 8:
    return None
  v1 = (d[2] & 0x3F) << 8 | d[1]
  v2 = (d[4] & 0x0F) << 10 | d[3] << 2 | d[2] >> 6
  v3 = (d[6] & 0x03) << 12 | d[5] << 4 | d[4] >> 4
  v4 = d[7] << 6 | d[6] >> 2
  vals = [v1, v2, v3, v4]
  if any(v in (0, 0x3FFF) for v in vals):
    return None
  return d[0], vals


DECODERS = {
  0x102: bms_hv, 0x302: bms_soc, 0x382: bms_energy, 0x232: bms_power_limits,
  0x266: rear_power, 0x2E5: front_power, 0x116: rear_torque_status, 0x106: rear_drive_info,
  0x154: rear_torque_measured, 0x1D4: front_torque_measured, 0x210: dcdc, 0x3D2: lifetime_energy,
  0x562: battery_odometer, 0x306: drive_temps, 0x338: range_status,
}
# Minimum frame length each decoder reads; shorter frames are kept raw but not decoded.
MIN_LEN = {0x102: 2, 0x302: 3, 0x382: 8, 0x232: 4, 0x266: 8, 0x2E5: 7, 0x116: 4, 0x106: 7, 0x154: 7,
           0x1D4: 7, 0x210: 6, 0x3D2: 8, 0x562: 4, 0x306: 3, 0x338: 6}
CELL_ID = 0x6F2
WATCH_IDS = sorted(DECODERS) + [CELL_ID]


class TeslaSXState:
  """Folds decoded frames into one snapshot, the way the page shows it."""

  def __init__(self):
    self.values: dict = {}
    self.stamp: dict = {}          # signal -> monotonic time it was last set
    self.cells_v: list = [None] * 96
    self.cells_t: list = [None] * 32

  def feed(self, msg_id: int, data: bytes, t: float) -> None:
    if msg_id == CELL_ID:
      r = bricks(data)
      if r is None:
        return
      idx, vals = r
      if idx < 24:
        for k, v in enumerate(vals):
          self.cells_v[idx * 4 + k] = round(v * 0.000305, 4)
      elif idx < 32:
        for k, v in enumerate(vals):
          self.cells_t[(idx - 24) * 4 + k] = round(_s(v, 14) * 0.0122, 1)
      self._summarise_cells(t)
      return
    dec = DECODERS.get(msg_id)
    if dec is None or len(data) < MIN_LEN.get(msg_id, 8):
      return
    for k, v in dec(data).items():
      self.values[k] = round(v, 3) if isinstance(v, float) else v
      self.stamp[k] = t

  def _summarise_cells(self, t: float) -> None:
    v = [x for x in self.cells_v if x is not None]
    if v:
      self.values.update({'cellMin': min(v), 'cellMax': max(v), 'cellAvg': round(sum(v) / len(v), 4),
                          'cellDelta': round(max(v) - min(v), 4), 'cellCount': len(v)})
      for k in ('cellMin', 'cellMax', 'cellAvg', 'cellDelta', 'cellCount'):
        self.stamp[k] = t
    tt = [x for x in self.cells_t if x is not None]
    if tt:
      self.values.update({'moduleTempMin': min(tt), 'moduleTempMax': max(tt)})
      self.stamp['moduleTempMin'] = self.stamp['moduleTempMax'] = t

  def snapshot(self, now: float) -> dict:
    return {'values': self.values, 'age': {k: round(now - t, 1) for k, t in self.stamp.items()},
            'cellsV': self.cells_v, 'cellsT': self.cells_t}
