"""Each decoder against the example frames and values printed in wk057's "Tesla Model S CAN Bus
Deciphering" v0.1 (2016-01-15). If one of these fails, the decode no longer matches the reference."""
import pytest

from openpilot.system.btobd import tesla_sx as t


def b(s):
  return bytes(int(x, 16) for x in s.split())


def test_0x102_voltage_and_current_single_motor():
  # 6-byte frame, no offset: "0x9F 0x83 = 92.7A (regen)"
  r = t.bms_hv(b('5C 99 9F 83 3C 07'))
  assert r['packCurrent'] == pytest.approx(92.7)
  r = t.bms_hv(b('66 97 D1 D2 00 00'))
  assert r['packVoltage'] == pytest.approx(387.58)
  assert r['packCurrent'] == pytest.approx(-1156.7)


def test_0x102_dual_motor_offset():
  # wk057's offset cars carry +10000: "0x85 0xA8 = 37.3 A (regen)", "0x27 0xF4 = -1,303.3 A"
  assert t.bms_hv(b('66 97 85 A8 00 00 00 00'))['packCurrent'] == pytest.approx(37.3)   # proves itself
  assert t.bms_hv(b('66 97 27 F4 00 00 00 00'), offset=True)['packCurrent'] == pytest.approx(-1303.3)


def test_0x102_this_model_x_has_no_offset():
  # parked HW1 Model X, 8-byte frame: field 158 -> 15.8 A, not the -984.2 A the length rule gave
  r = t.bms_hv(b('24 8F 9E 00 98 00 FC 1F'))
  assert r['packVoltage'] == pytest.approx(366.44)
  assert r['packCurrent'] == pytest.approx(15.8)
  assert 'negTerminalTemp' not in r                        # FC 1F = 0x7FC, "no reading", not 204.4 degC
  assert t.bms_hv(b('24 8F 9E 00 98 00 E1 00'))['negTerminalTemp'] == pytest.approx(22.5)


def test_state_latches_current_offset():
  s = t.TeslaSXState()
  s.feed(0x102, b('66 97 27 F4 00 00 00 00'), 0.0)        # ambiguous on its own: read without offset
  assert s.values['packCurrent'] == pytest.approx(-303.3)
  s.feed(0x102, b('66 97 85 A8 00 00 00 00'), 0.1)        # 10373 > 6000: offset car
  s.feed(0x102, b('66 97 27 F4 00 00 00 00'), 0.2)
  assert s.values['packCurrent'] == pytest.approx(-1303.3)


def test_0x302_soc():
  r = t.bms_soc(b('82 DF 0D'))
  assert r['socMin'] == pytest.approx(89.8)
  assert r['socUI'] == pytest.approx(88.7)


def test_0x382_energy():
  r = t.bms_energy(b('FC 9A EA 69 A9 00 A0 00'))
  assert r['nominalFullPackEnergy'] == pytest.approx(76.4)
  assert r['nominalEnergyRemaining'] == pytest.approx(67.8)
  assert r['expectedEnergyRemaining'] == pytest.approx(67.0)
  assert r['idealEnergyRemaining'] == pytest.approx(67.7)
  assert r['energyBuffer'] == pytest.approx(4.0)
  assert r['energyToChargeComplete'] == pytest.approx(0.0)
  assert r['socDisplay'] == pytest.approx((67.8 - 4.0) / (76.4 - 4.0) * 100)


def test_0x382_eleven_bit_layout_on_this_model_x():
  # 10-bit decode reads remaining 75.6 of a 72.9 pack; 11-bit gives 37.8 = socUI 51.9% on the same drive
  r = t.bms_energy(b('D9 D2 0B 60 F6 02 00 1E'))
  assert r['nominalFullPackEnergy'] == pytest.approx(72.9)
  assert r['nominalEnergyRemaining'] == pytest.approx(37.8)
  assert r['expectedEnergyRemaining'] == pytest.approx(38.4)
  assert r['idealEnergyRemaining'] == pytest.approx(37.9)
  assert r['energyBuffer'] == pytest.approx(6.0)
  assert r['socDisplay'] == pytest.approx((37.8 - 6.0) / (72.9 - 6.0) * 100)


@pytest.mark.parametrize('soc', [0.05, 0.1, 0.3, 0.9, 1.0])
def test_0x382_layout_choice_holds_at_any_charge(soc):
  # build an 11-bit frame at this pack's size; low charge is where both layouts fit inside the pack
  full, rem = 729, round(729 * soc)
  v = full | rem << 11 | (rem + 6) << 22 | rem << 33 | 60 << 55
  r = t.bms_energy(v.to_bytes(8, 'little'))
  assert r['nominalEnergyRemaining'] == pytest.approx(rem / 10)


def test_0x232_power_limits():
  r = t.bms_power_limits(b('80 19 10 89'))
  assert r['maxDischargePower'] == pytest.approx(350.88)
  assert r['maxRegenPower'] == pytest.approx(65.28)


def test_0x266_rear_power():
  r = t.rear_power(b('85 B1 9F 01 3E EB 8A 44'))
  assert r['rearInverter12V'] == pytest.approx(13.3)
  assert r['rearDissipation'] == pytest.approx(22125)
  assert r['rearMechPower'] == pytest.approx(207.5)
  assert r['rearStatorCurrent'] == 830
  assert r['rearRegenPowerMax'] == 72
  assert r['rearDrivePowerMax'] == 349


def test_0x2e5_front_power():
  r = t.front_power(b('88 62 08 01 D9 99 07 00'))
  assert r['frontInverter12V'] == pytest.approx(13.6)
  assert r['frontDissipation'] == pytest.approx(12250)
  assert r['frontMechPower'] == pytest.approx(132)
  assert r['frontStatorCurrent'] == 473
  assert r['frontDrivePowerMax'] == 243


def test_0x116_gear_torque_speed():
  r = t.rear_torque_status(b('5A 44 75 44 8E FC'))
  assert r['gear'] == 'D'
  assert r['rearTorqueEstimate'] == pytest.approx(557, abs=0.5)
  assert r['vehicleSpeedMph'] == pytest.approx(32.05)


def test_0x106_rpm_pedal():
  r = t.rear_drive_info(b('31 FF 2D 7F A3 14 0E A8'))
  assert r['rearMotorRpm'] == 5283
  assert r['pedalPos'] == pytest.approx(5.6)


def test_0x154_and_0x1d4_measured_torque():
  r = t.rear_torque_measured(b('30 63 FA FA 48 CC 09 F9'))
  assert r['rearTorqueMeasured'] == pytest.approx(627)
  assert r['pedalPosA'] == pytest.approx(100) and r['pedalPosB'] == pytest.approx(100)
  assert t.front_torque_measured(b('00 03 00 00 08 62 E1 23'))['frontTorqueMeasured'] == pytest.approx(88.5)


def test_0x210_dcdc():
  r = t.dcdc(b('00 00 D6 1B 1E 8A 00'))
  assert r['dcdcInputPower'] == pytest.approx(432)
  assert r['dcdcOutputCurrent'] == pytest.approx(30)
  assert r['dcdcOutputVoltage'] == pytest.approx(13.8)
  assert r['dcdcOutputPower'] == pytest.approx(414)
  assert r['dcdcInletTemp'] == pytest.approx(19)


def test_0x3d2_lifetime_and_0x562_odometer():
  r = t.lifetime_energy(b('25 AE 86 00 07 DC 8F 00'))
  assert r['kwhChargeTotal'] == pytest.approx(9427.975)
  assert r['kwhDischargeTotal'] == pytest.approx(8826.405)
  assert t.battery_odometer(b('3C 27 2C 01'))['batteryOdometerMi'] == pytest.approx(19670.844)


def test_0x6f2_bricks_round_trip():
  # Pack four known 14-bit values the way OVMS unpacks them, then decode.
  vals = [12000, 12100, 12200, 12300]   # x 0.000305 V ~= 3.66-3.75 V
  v1, v2, v3, v4 = vals
  d = bytes([5, v1 & 0xFF, (v1 >> 8) & 0x3F | (v2 & 0x3) << 6, (v2 >> 2) & 0xFF,
             (v2 >> 10) & 0x0F | (v3 & 0x0F) << 4, (v3 >> 4) & 0xFF, (v3 >> 12) & 0x3 | (v4 & 0x3F) << 2,
             (v4 >> 6) & 0xFF])
  assert t.bricks(d) == (5, vals)
  s = t.TeslaSXState()
  s.feed(0x6F2, d, 0.0)
  assert s.cells_v[20:24] == [round(v * 0.000305, 4) for v in vals]
  assert s.values['cellMin'] == round(12000 * 0.000305, 4)
  assert s.values['cellDelta'] == round(300 * 0.000305, 4)


def test_short_frames_are_ignored():
  s = t.TeslaSXState()
  s.feed(0x382, b('FC 9A'), 0.0)
  assert s.values == {}
