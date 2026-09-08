"""The device-state failures that stopped this car engaging, as tests.

The logic checks run anywhere -- they take collected state, not a device. The last class only
runs on the car, where there is a /data to look at, and is skipped elsewhere.
"""
import json
import os

import pytest

from openpilot.tools.tesla_analysis.device_preflight import (
  OPENPILOT_ROOT,
  check_branch,
  check_offroad_alerts,
  check_ownership,
  check_staging_ownership,
  updates_disabled,
  check_symlinks,
  check_update_health,
  collect_and_run,
)


class TestOwnership:
  """2026-09-08: 10,812 root-owned files under /data/openpilot made the updater's
  `git clean -xdff` fail, which after 22 days of failed checks blocked engagement."""

  def test_clean_tree_passes(self):
    assert check_ownership(0).ok

  def test_one_foreign_file_is_enough_to_fail(self):
    # git clean stops on the first file it cannot remove, so this is not a threshold
    assert not check_ownership(1).ok

  def test_reports_the_count_and_the_fix(self):
    r = check_ownership(10812)
    assert not r.ok and "10812" in r.detail and "chown" in r.fix


class TestStagingOwnership:
  """The overlay's upper layer fills with root-owned .o/.a/__pycache__ every time scons runs as
  root, which `systemctl restart comma` does. Normal, so it must not cry wolf -- 208 of them were
  present on a device whose updater was perfectly healthy."""

  def test_clean_staging_passes(self):
    assert check_staging_ownership(0, update_ok=True).ok

  def test_artifacts_alone_are_only_a_warning(self):
    r = check_staging_ownership(208, update_ok=True)
    assert r.ok and r.warn

  def test_artifacts_fail_only_while_the_update_is_broken(self):
    r = check_staging_ownership(208, update_ok=False)
    assert not r.ok and "chown" in r.fix


class TestUpdateHealth:
  def test_healthy(self):
    assert check_update_health({"UpdateFailedCount": "0", "LastUpdateException": ""}).ok

  def test_failed_count_fails(self):
    r = check_update_health({"UpdateFailedCount": "10"})
    assert not r.ok and "10" in r.detail

  def test_exception_alone_fails(self):
    # the count can reset while the underlying cause is still there
    r = check_update_health({"UpdateFailedCount": "0",
                             "LastUpdateException": "command failed: ['git', 'clean', '-xdff']"})
    assert not r.ok and "git" in r.detail

  def test_missing_params_are_not_a_failure(self):
    assert check_update_health({}).ok


class TestOffroadAlerts:
  def _alert(self, severity):
    return json.dumps({"text": "...", "severity": severity})

  def test_no_alerts(self):
    assert check_offroad_alerts({"UpdateFailedCount": "0"}).ok

  def test_blocking_alert_fails(self):
    r = check_offroad_alerts({"Offroad_ConnectivityNeeded": self._alert(1)})
    assert not r.ok and "ConnectivityNeeded" in r.detail

  def test_empty_alert_param_is_clear(self):
    # cleared alerts are written as an empty value rather than removed
    assert check_offroad_alerts({"Offroad_ConnectivityNeeded": ""}).ok

  def test_informational_alert_does_not_fail(self):
    assert check_offroad_alerts({"Offroad_NeosUpdate": self._alert(0)}).ok

  def test_unparseable_alert_counts_as_blocking(self):
    assert not check_offroad_alerts({"Offroad_Whatever": "not json"}).ok


class TestSymlinks:
  """`git checkout -- .` rewrites git-lfs symlinks as their pointer text; scons then dies on a
  missing libqpOASES_e.so, often long after the checkout that caused it."""

  def test_intact(self):
    assert check_symlinks([]).ok

  def test_dangling_fails_and_is_named(self):
    r = check_symlinks(["third_party/acados/larch64/lib/libqpOASES_e.so"])
    assert not r.ok and "libqpOASES" in r.detail


class TestBranch:
  def test_expected(self):
    assert check_branch("tesla-hw1-carrot-0.11.2", "tesla-hw1-carrot-0.11.2").ok

  def test_wrong_branch_is_reported_with_both_names(self):
    r = check_branch("master", "tesla-hw1-carrot-0.11.2")
    assert not r.ok and "master" in r.detail and "tesla-hw1-carrot-0.11.2" in r.detail


@pytest.mark.skipif(not os.path.isdir(OPENPILOT_ROOT), reason="차량에서만 실행")
class TestOnDevice:
  def test_preflight_passes(self):
    results = collect_and_run("tesla-hw1-carrot-0.11.2")
    failed = [f"{r.name}: {r.detail}" for r in results if not r.ok]
    assert not failed, "인게이지를 막을 수 있는 상태:\n  " + "\n  ".join(failed)


class TestUpdatesDisabled:
  """DisableUpdates is upstream's own escape hatch -- updated.py exits on it and hardwared.py's
  up_to_date startup condition is satisfied regardless of a stale alert. Once it is set, checking
  update health would report on a process that no longer runs."""

  def test_flag_is_read(self):
    assert updates_disabled({"DisableUpdates": "1"})
    assert not updates_disabled({"DisableUpdates": "0"})
    assert not updates_disabled({})

  def test_update_check_stops_asserting(self):
    # stale failures must not fail the preflight once nothing is running to clear them
    r = check_update_health({"DisableUpdates": "1", "UpdateFailedCount": "10",
                             "LastUpdateException": "command failed"})
    assert r.ok and r.warn and "비활성" in r.detail

  def test_connectivity_alert_is_forgiven(self):
    alert = json.dumps({"text": "connect to internet", "severity": 1})
    assert check_offroad_alerts({"DisableUpdates": "1", "Offroad_ConnectivityNeeded": alert}).ok

  def test_other_alerts_still_block(self):
    # disabling updates forgives the update alerts only, not e.g. a calibration problem
    alert = json.dumps({"text": "recalibrate", "severity": 1})
    r = check_offroad_alerts({"DisableUpdates": "1", "Offroad_Recalibration": alert})
    assert not r.ok and "Recalibration" in r.detail

  def test_still_enforced_when_updates_are_on(self):
    alert = json.dumps({"text": "connect to internet", "severity": 1})
    assert not check_offroad_alerts({"Offroad_ConnectivityNeeded": alert}).ok
