import pytest

from opendbc.can.dbc import DBC

from openpilot.selfdrive.debug import vehicle_state
from openpilot.selfdrive.debug.can_viewer import CanDecoder


class FakeFrame:
  def __init__(self, src, address, dat):
    self.src = src
    self.address = address
    self.dat = dat


def _decoder():
  return CanDecoder(['tesla_can'], start=False)


def _rows(built):
  for section in built['sections']:
    for group in section['groups']:
      yield from group['rows']


def _by_label(built):
  """Keyed by (group, label): '운전석' is both a door and a seatbelt."""
  return {(g['title'], r['label']): r
          for s in built['sections'] for g in s['groups'] for r in g['rows']}


def test_every_row_points_at_a_real_dbc_signal():
  """A typo in the spec would otherwise just render a permanently blank row."""
  dbc = DBC('tesla_can')
  for _, _, groups in vehicle_state.SECTIONS:
    for group_title, rows in groups:
      for label, addr, signal, _ in rows:
        msg = dbc.addr_to_msg.get(addr)
        assert msg is not None, f'{group_title}/{label}: 0x{addr:03X} not in tesla_can'
        assert signal in msg.sigs, f'{group_title}/{label}: {signal} not in {msg.name}'


def test_nothing_received_yields_blank_rows_not_errors():
  built = vehicle_state.build(_decoder())
  rows = list(_rows(built))

  assert len(rows) == built['total'] == built['missing']
  assert all(r['value'] is None and not r['warn'] for r in rows)


def test_no_decoder_reports_the_reason():
  built = vehicle_state.build(None)
  assert built['sections'] == []
  assert '차량 미연결' in built['error']


def test_decodes_and_translates_a_known_frame():
  dec = _decoder()
  # GTW_carState: driver door open, everything else closed
  dat = bytearray(8)
  dat[1] = 0b0001_0000     # DOOR_STATE_FL (bits 12-13) = 1 "open"
  dec.ingest([FakeFrame(0, 0x318, bytes(dat))], now=0.0)

  rows = _by_label(vehicle_state.build(dec))
  assert rows[('문 · 개폐', '운전석')]['value'] == '열림'
  assert rows[('문 · 개폐', '운전석')]['warn'] is True
  assert rows[('문 · 개폐', '조수석')]['value'] == '닫힘'
  assert rows[('문 · 개폐', '조수석')]['warn'] is False


def test_open_door_marks_its_group():
  dec = _decoder()
  dat = bytearray(8)
  dat[1] = 0b0001_0000
  dec.ingest([FakeFrame(0, 0x318, bytes(dat))], now=0.0)

  groups = {g['title']: g for s in vehicle_state.build(dec)['sections'] for g in s['groups']}
  assert groups['문 · 개폐']['warn'] is True
  assert groups['온도']['warn'] is False


def test_air_suspension_height_reaches_the_page():
  dec = _decoder()
  dec.ingest([FakeFrame(0, 0x10B, bytes([74, 80, 91, 71, 0]))], now=0.0)

  rows = _by_label(vehicle_state.build(dec))
  assert [rows[('에어 서스펜션', k)]['value'] for k in ('앞 좌', '앞 우', '뒤 좌', '뒤 우')] == \
         ['-26 mm', '-20 mm', '-9 mm', '-29 mm']


def test_sna_wins_over_a_meaningless_number():
  dec = _decoder()
  # AirTemp_Outsd raw 255 is "SNA"; scaled it would read as a plausible 87.5 C
  dat = bytearray(8)
  dat[7] = 255
  dec.ingest([FakeFrame(0, 0x283, bytes(dat))], now=0.0)

  rows = _by_label(vehicle_state.build(dec))
  assert rows[('온도', '외기')]['value'] == 'SNA'


def test_bus_0_wins_over_the_forwarded_copy():
  dec = _decoder()
  fl_open = bytearray(8)
  fl_open[1] = 0b0001_0000
  dec.ingest([FakeFrame(2, 0x318, bytes(fl_open)),   # forwarded copy says open
              FakeFrame(0, 0x318, bytes(8))],        # the car itself says closed
             now=0.0)

  rows = _by_label(vehicle_state.build(dec))
  assert rows[('문 · 개폐', '운전석')]['value'] == '닫힘'


def test_country_reads_as_text():
  dec = _decoder()
  # GTW_country is big-endian starting at bit 23, which puts it in bytes 2-3
  dec.ingest([FakeFrame(0, 0x398, bytes([0, 0, 0x55, 0x53, 0, 0, 0, 0]))], now=0.0)

  rows = _by_label(vehicle_state.build(dec))
  assert rows[('차량 사양', '국가')]['value'] == 'US'


@pytest.mark.parametrize("section_id, title", [('core', '중요'), ('more', '그 외')])
def test_both_sections_are_present_and_populated(section_id, title):
  built = vehicle_state.build(_decoder())
  section = next(s for s in built['sections'] if s['id'] == section_id)
  assert section['title'] == title
  assert len(section['groups']) > 0
