"""Every knob the tuning page serves must be packable: /api/settings builds all three tables in one
response, so a single knob with neither 'range' nor 'options' empties the whole settings page."""
import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "tuning_server.py"


def _tables():
  for node in ast.parse(SRC.read_text()).body:
    if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in ("SETTINGS", "MAP_SETTINGS", "CARROT_SETTINGS"):
      yield node.targets[0].id, ast.literal_eval(node.value)


def test_every_knob_has_range_or_options():
  bad = [f"{name}.{k}" for name, table in _tables() for k, cfg in table.items() if "range" not in cfg and "options" not in cfg]
  assert not bad


def test_bool_knobs_offer_both_states():
  for _, table in _tables():
    for k, cfg in table.items():
      if cfg["type"] == "bool":
        assert {v for v, _ in cfg["options"]} == {0, 1}, k
