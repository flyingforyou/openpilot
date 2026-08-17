import pytest

from openpilot.selfdrive.debug.can_viewer import CanDecoder, is_noise_signal


class FakeSignal:
  def __init__(self, name, type=0):
    self.name = name
    self.type = type


class FakeFrame:
  def __init__(self, src, address, dat):
    self.src = src
    self.address = address
    self.dat = dat


@pytest.mark.parametrize("name, noise", [
  ("CRC_STW_ACTN_RQ", True),
  ("MC_STW_ACTN_RQ", True),
  ("DI_torque1Checksum", True),
  ("DI_torque1Counter", True),
  ("ESP_B_CRC", True),
  ("some_Cnt", True),
  # the ones you'd actually be hunting for
  ("DTR_Dist_Rq", False),
  ("TurnIndLvr_Stat", False),
  ("DI_vehicleSpeed", False),
  ("StW_Angl", False),
])
def test_noise_signal_by_name(name, noise):
  assert is_noise_signal(FakeSignal(name)) == noise


def test_noise_signal_by_type():
  # DBCs that do type their counters shouldn't need the name heuristic
  assert is_noise_signal(FakeSignal("anything", type=1))


def _decoder():
  return CanDecoder(['tesla_can'], start=False)


def test_decodes_against_dbc_with_enum_labels():
  dec = _decoder()
  # STW_ACTN_RQ with DTR_Dist_Rq raw 100 -> gap 4
  dec.ingest([FakeFrame(0, 69, bytes([0, 100, 0, 0, 0, 0, 0, 0]))], now=0.0)
  msg = dec.snapshot(now=0.0)['messages'][0]

  assert msg['name'] == 'STW_ACTN_RQ'
  sig = next(s for s in msg['signals'] if s['name'] == 'DTR_Dist_Rq')
  assert sig['v'] == 100
  assert sig['enum'] == 'ACC_DIST_4'


def test_unknown_address_still_listed_with_raw_bytes():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 0x7FF, b'\xde\xad')], now=0.0)
  msg = dec.snapshot(now=0.0)['messages'][0]

  assert msg['name'] is None
  assert msg['hex'] == 'dead'
  assert msg['signals'] == []


def test_rate_needs_two_samples():
  dec = _decoder()
  frame = FakeFrame(0, 69, bytes(8))

  dec.ingest([frame], now=0.0)
  assert dec.snapshot(now=0.0)['messages'][0]['hz'] == 0.0, "a single frame says nothing about rate"

  for i in range(1, 11):
    dec.ingest([frame], now=i * 0.1)
  assert dec.snapshot(now=1.0)['messages'][0]['hz'] == pytest.approx(10.0)


def test_counter_churn_is_not_reported_as_change():
  dec = _decoder()
  # MC_STW_ACTN_RQ lives in the high nibble of byte 6; only that moves between these two
  dec.ingest([FakeFrame(0, 69, bytes([0, 100, 0, 0, 0, 0, 0x10, 0]))], now=0.0)
  dec.ingest([FakeFrame(0, 69, bytes([0, 100, 0, 0, 0, 0, 0x20, 0]))], now=0.1)

  assert dec.snapshot(now=0.1)['messages'][0]['anyChanged'] is False
  assert dec.snapshot(changed_only=True, now=0.1)['total'] == 0


def test_real_signal_change_is_reported_and_expires():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 69, bytes([0, 100, 0, 0, 0, 0, 0, 0]))], now=0.0)
  dec.ingest([FakeFrame(0, 69, bytes([0, 133, 0, 0, 0, 0, 0, 0]))], now=0.1)  # gap 4 -> 5

  changed = [s for s in dec.snapshot(now=0.1)['messages'][0]['signals'] if s['changed']]
  assert [s['name'] for s in changed] == ['DTR_Dist_Rq']

  after_hold = 0.1 + CanDecoder.CHANGE_HOLD_S + 0.1
  assert dec.snapshot(now=after_hold)['messages'][0]['anyChanged'] is False


def test_catalog_lists_dbc_messages_never_received():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 69, bytes(8))], now=0.0)
  snap = dec.snapshot(now=0.0)

  received = [m for m in snap['messages'] if m['seen']]
  assert [m['address'] for m in received] == [69]

  # everything else the DBC defines is still listed, with nothing to show
  from opendbc.can.dbc import DBC
  unseen = [m for m in snap['messages'] if not m['seen']]
  assert len(unseen) == len(DBC('tesla_can').addr_to_msg) - 1
  sample = unseen[0]
  assert sample['name'] and sample['hex'] is None and sample['bus'] is None
  assert all(s['v'] is None for s in sample['signals'])
  assert snap['seen'] == 1


def test_received_message_is_not_duplicated_by_catalog():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 69, bytes(8)), FakeFrame(2, 69, bytes(8))], now=0.0)
  snap = dec.snapshot(now=0.0)

  entries = [m for m in snap['messages'] if m['address'] == 69]
  assert sorted(m['bus'] for m in entries) == [0, 2], "seen on two buses, and no catalog stub"


def test_unseen_can_be_excluded():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 69, bytes(8))], now=0.0)

  assert dec.snapshot(include_unseen=False, now=0.0)['total'] == 1
  assert dec.snapshot(changed_only=True, now=0.0)['total'] == 0, "catalog must not leak in here"
