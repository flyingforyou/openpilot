"""The gap stalk rides on LongitudinalPersonality, and the planner looks it up in a dict.

That lookup is the whole of this file's concern. A personality read off a message is a pycapnp
_DynamicEnum, which compares equal to its int but does not hash equal to it, so it works in
upstream's if/elif chains and silently misses in a dict keyed on ints -- and the miss raises
NotImplementedError out of the planner, killing it the moment anyone engages.
"""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.carrot_functions import (
    GAP_TO_PERSONALITY_INT, PERSONALITY_TO_GAP_POS, _personality_int)

GAPS = range(1, 8)


def _personality_off_a_message(value: int):
  """Exactly how the planner receives it: written by selfdrived, read back off the wire."""
  msg = messaging.new_message('selfdriveState')
  msg.selfdriveState.personality = value
  return msg.as_reader().selfdriveState.personality


@pytest.mark.parametrize("gap", GAPS)
def test_every_gap_resolves_when_read_off_a_message(gap):
  written = GAP_TO_PERSONALITY_INT[gap - 1]
  personality = _personality_off_a_message(written)
  assert PERSONALITY_TO_GAP_POS.get(_personality_int(personality)) == gap


@pytest.mark.parametrize("gap", GAPS)
def test_every_gap_resolves_when_read_off_the_param(gap):
  # the other path: params hand back a plain int, which has no .raw
  written = GAP_TO_PERSONALITY_INT[gap - 1]
  assert PERSONALITY_TO_GAP_POS.get(_personality_int(written)) == gap


def test_the_raw_enum_would_have_missed():
  """Pin the actual defect, so a refactor that drops the normalisation fails here."""
  personality = _personality_off_a_message(log.LongitudinalPersonality.standard)
  assert personality == log.LongitudinalPersonality.standard, "compares equal"
  assert PERSONALITY_TO_GAP_POS.get(personality) is None, "but does not hash equal"
  assert PERSONALITY_TO_GAP_POS.get(_personality_int(personality)) is not None


def test_gap_mapping_is_a_bijection_over_seven_positions():
  assert sorted(GAP_TO_PERSONALITY_INT) == list(range(7)), "every enum value used exactly once"
  assert sorted(PERSONALITY_TO_GAP_POS.values()) == list(GAPS)


def test_gap_order_is_monotonic_in_following_distance():
  """Gap 1 is closest and gap 7 furthest, whatever numbers capnp handed the enum. The append-only
  numbering interleaves the new values with the old, so this is not the identity."""
  names = {v: k for k, v in log.LongitudinalPersonality.schema.enumerants.items()}
  order = [names[GAP_TO_PERSONALITY_INT[g - 1]] for g in GAPS]
  assert order == ['moreAggressive', 'aggressive', 'lessAggressive', 'standard',
                   'lessRelaxed', 'relaxed', 'moreRelaxed']
