#!/usr/bin/env python3
"""Everything that has silently stopped this car engaging, checked before a drive.

drive_health.py next door reads a finished drive. This reads the device itself, because the
failures it covers do not show up in a log -- they stop the drive from starting. Each one here
actually happened:

  - 2026-09-08: openpilot refused to engage, showing "connect to internet to check for updates",
    with the internet working fine. The updater runs `git clean -xdff`, 10,812 files under
    /data/openpilot and 202 under /data/safe_staging/upper were owned by root from earlier sudo
    runs, the clean failed on them, and 22 days of failed update checks tripped the block.
  - Earlier: `git checkout -- .` re-materialised five git-lfs symlinks as their pointer text, so
    scons died on a missing libqpOASES_e.so long after the checkout that caused it.

Checks are pure functions over collected state so they can be unit tested off-device; the
collection is the only part that needs a real device.

  ./device_preflight.py            # run every check, exit non-zero if any fail
  ./device_preflight.py --json     # same, machine readable
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass, asdict

OPENPILOT_ROOT = "/data/openpilot"
STAGING_ROOT = "/data/safe_staging"
PARAM_DIR = "/data/params/d"

# The updater's own clean step cannot remove what it does not own, and one file is enough.
OWNERSHIP_ROOTS = (OPENPILOT_ROOT, STAGING_ROOT)

# Offroad_* params are the alerts the UI shows; a set one with severity 1 blocks engagement.
BLOCKING_SEVERITY = 1


@dataclass
class Result:
    name: str
    ok: bool
    detail: str
    fix: str = ""
    warn: bool = False      # true = worth knowing, not a reason to refuse the drive


# ---------------------------------------------------------------- pure checks

def check_ownership(repo_count: int) -> Result:
    """Root-owned files in the tracked tree. Always wrong -- source files are the user's, and the
    updater cannot clean what it does not own."""
    if not repo_count:
        return Result("ownership", True, "root 소유 파일 없음")
    return Result("ownership", False, f"{OPENPILOT_ROOT} 에 root 소유 {repo_count}개",
                  f"sudo chown -R comma:comma {OPENPILOT_ROOT}")


def check_staging_ownership(staging_count: int, update_ok: bool) -> Result:
    """The overlay's upper layer collects root-owned build artefacts every time scons runs as root
    (which `systemctl restart comma` does), so their presence is normal, not a defect. They only
    matter once the updater's `git clean -xdff` trips over them -- which the update check sees
    directly. So report them, and only insist while the update is actually broken."""
    if not staging_count:
        return Result("staging_ownership", True, "스테이징 오버레이 깨끗함")
    detail = f"{STAGING_ROOT} 에 root 소유 {staging_count}개 (root 빌드 산출물, 정상)"
    if update_ok:
        return Result("staging_ownership", True, detail, warn=True)
    return Result("staging_ownership", False, detail + " -- 업데이트 실패의 원인일 수 있음",
                  f"sudo chown -R comma:comma {STAGING_ROOT}")


def updates_disabled(params: dict[str, str]) -> bool:
    """DisableUpdates is upstream's own escape hatch: updated.py exits immediately, and
    hardwared.py's `up_to_date` startup condition is satisfied regardless of any stale alert."""
    return (params.get("DisableUpdates") or "").strip() in ("1", "true", "True")


def check_update_health(params: dict[str, str]) -> Result:
    """The updater must be succeeding, or openpilot eventually refuses to start.

    Meaningless once updates are switched off -- nothing runs to succeed or fail, and the gate it
    would trip no longer applies -- so say so rather than passing on stale numbers."""
    if updates_disabled(params):
        return Result("update", True, "업데이트 비활성 (DisableUpdates) -- 검사 안 함", warn=True)
    failed = params.get("UpdateFailedCount", "0").strip() or "0"
    exc = (params.get("LastUpdateException") or "").strip()
    try:
        n = int(failed)
    except ValueError:
        n = 0
    if n == 0 and not exc:
        return Result("update", True, "업데이트 정상 (실패 0회)")
    detail = f"실패 {n}회"
    if exc:
        detail += f" | {exc.splitlines()[0][:100]}"
    return Result("update", False, detail,
                  "원인이 Permission denied 면 ownership 항목을 먼저 고칠 것")


# Alerts hardwared.py's up_to_date condition forgives once updates are disabled.
UPDATE_ALERTS = ("Offroad_ConnectivityNeeded", "Offroad_ConnectivityNeededPrompt")


def check_offroad_alerts(params: dict[str, str]) -> Result:
    """Any Offroad_* alert that is set and severe enough to stop engagement."""
    forgiven = UPDATE_ALERTS if updates_disabled(params) else ()
    blocking = []
    for key, raw in params.items():
        if not key.startswith("Offroad_") or not (raw or "").strip():
            continue
        if key in forgiven:
            continue
        try:
            sev = json.loads(raw).get("severity", 0)
        except (ValueError, AttributeError):
            sev = BLOCKING_SEVERITY      # unparseable but present: treat as blocking
        if sev >= BLOCKING_SEVERITY:
            blocking.append(key)
    if not blocking:
        return Result("offroad_alerts", True, "인게이지를 막는 알림 없음")
    return Result("offroad_alerts", False, "차단 알림: " + ", ".join(sorted(blocking)),
                  "각 알림의 원인을 해소할 것 (업데이트 실패가 가장 흔함)")


def check_symlinks(dangling: list[str]) -> Result:
    """git-lfs pointers re-materialised as broken links kill the next real scons build."""
    if not dangling:
        return Result("lfs_symlinks", True, "끊어진 심볼릭 링크 없음")
    return Result("lfs_symlinks", False, f"끊어진 링크 {len(dangling)}개: " + ", ".join(dangling[:3]),
                  "ln -sfn 으로 수동 복구 (libqpOASES_e.so.3.1, driving_*.onnx)")


def check_branch(current: str, expected: str) -> Result:
    if current == expected:
        return Result("branch", True, f"{current}")
    return Result("branch", False, f"{current} (기대: {expected})",
                  f"git checkout {expected}")


# ------------------------------------------------------------- device reading

def _count_foreign_owner(path: str, uid: int) -> int:
    if not os.path.isdir(path):
        return 0
    n = 0
    for root, dirs, files in os.walk(path, onerror=lambda _e: None):
        if ".git" in dirs:
            dirs.remove(".git")
        for name in dirs + files:
            try:
                if os.lstat(os.path.join(root, name)).st_uid != uid:
                    n += 1
            except OSError:
                pass
    return n


def _read_params() -> dict[str, str]:
    out = {}
    if not os.path.isdir(PARAM_DIR):
        return out
    for name in os.listdir(PARAM_DIR):
        try:
            with open(os.path.join(PARAM_DIR, name), errors="replace") as f:
                out[name] = f.read()
        except OSError:
            pass
    return out


def _dangling_symlinks(root: str) -> list[str]:
    bad = []
    for base, dirs, files in os.walk(root, onerror=lambda _e: None):
        if ".git" in dirs:
            dirs.remove(".git")
        for name in files + dirs:
            p = os.path.join(base, name)
            if os.path.islink(p) and not os.path.exists(p):
                bad.append(os.path.relpath(p, root))
    return bad


def _git_branch(root: str) -> str:
    try:
        return subprocess.run(["git", "-C", root, "rev-parse", "--abbrev-ref", "HEAD"],
                              capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "?"


def collect_and_run(expected_branch: str) -> list[Result]:
    uid = os.stat(OPENPILOT_ROOT).st_uid if os.path.isdir(OPENPILOT_ROOT) else os.getuid()
    params = _read_params()
    update = check_update_health(params)
    return [
        check_ownership(_count_foreign_owner(OPENPILOT_ROOT, uid)),
        check_staging_ownership(_count_foreign_owner(STAGING_ROOT, uid), update.ok),
        update,
        check_offroad_alerts(params),
        check_symlinks(_dangling_symlinks(OPENPILOT_ROOT)),
        check_branch(_git_branch(OPENPILOT_ROOT), expected_branch),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--branch", default="tesla-hw1-carrot-0.11.2")
    args = ap.parse_args()

    results = collect_and_run(args.branch)
    if args.json:
        print(json.dumps([asdict(r) for r in results], ensure_ascii=False, indent=2))
    else:
        for r in results:
            tag = "WARN" if (r.ok and r.warn) else ("OK  " if r.ok else "FAIL")
            print(f"[{tag}] {r.name:<18} {r.detail}")
            if not r.ok and r.fix:
                print(f"{'':>7} 조치: {r.fix}")
    return 0 if all(r.ok for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
