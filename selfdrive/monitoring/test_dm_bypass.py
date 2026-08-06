import pytest

from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring
from openpilot.selfdrive.monitoring.test_monitoring import msg_NO_FACE_DETECTED, msg_DISTRACTED

EventName = log.OnroadEvent.EventName

# what the bypass exists to suppress
DM_EVENTS = {EventName.preDriverDistracted, EventName.promptDriverDistracted, EventName.driverDistracted,
             EventName.preDriverUnresponsive, EventName.promptDriverUnresponsive, EventName.driverUnresponsive}

SECONDS = 90


@pytest.fixture
def params():
  p = Params()
  yield p
  p.put_bool("DriverMonitorBypass", False)


def _run(msg, seconds=SECONDS):
  DM = DriverMonitoring()
  events = []
  for _ in range(int(seconds / DT_DMON)):
    DM.run_step({'driverStateV2': msg}, demo=True)
    events.append(DM.current_events)
  return DM, events


def test_off_by_default(params):
  params.put_bool("DriverMonitorBypass", False)
  DM, events = _run(msg_NO_FACE_DETECTED)

  assert not DM.bypass
  assert DM.awareness < 1., "baseline: an absent driver still burns awareness down"
  assert any(set(e.names) & DM_EVENTS for e in events)


@pytest.mark.parametrize("msg", [msg_NO_FACE_DETECTED, msg_DISTRACTED])
def test_bypass_reports_attentive_and_raises_nothing(params, msg):
  params.put_bool("DriverMonitorBypass", True)
  DM, events = _run(msg)

  assert DM.bypass
  assert DM.face_detected and not DM.driver_distracted
  assert DM.awareness == DM.awareness_active == DM.awareness_passive == 1.
  assert not any(set(e.names) & DM_EVENTS for e in events), "no distraction alert may survive the bypass"


def test_bypass_clears_the_state_events_are_derived_from(params):
  """face_detected/driver_distracted/awareness alone aren't enough -- _update_events re-derives
  distraction from the filter and hi_stds, so those have to go too."""
  params.put_bool("DriverMonitorBypass", True)
  DM, _ = _run(msg_DISTRACTED, seconds=30)

  assert DM.driver_distraction_filter.x == 0.
  assert DM.hi_stds == 0
  assert DM.pose.low_std
  assert DM.terminal_alert_cnt == 0


def test_picked_up_without_restart(params):
  params.put_bool("DriverMonitorBypass", False)
  DM = DriverMonitoring()
  for _ in range(int(20 / DT_DMON)):
    DM.run_step({'driverStateV2': msg_NO_FACE_DETECTED}, demo=True)
  assert DM.awareness < 1.

  params.put_bool("DriverMonitorBypass", True)
  for _ in range(int(5 / DT_DMON)):
    DM.run_step({'driverStateV2': msg_NO_FACE_DETECTED}, demo=True)

  assert DM.bypass and DM.awareness == 1., "the /live toggle has to take effect on a running dmonitoringd"
