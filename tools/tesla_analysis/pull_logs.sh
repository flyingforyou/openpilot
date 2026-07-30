#!/usr/bin/env bash
# Pull rlog segments off the comma into the local log store.
#
# Only rlog.zst comes across -- the camera files are the bulk of a route and
# nothing here reads them. Already-downloaded segments are skipped, so this is
# safe to re-run after every drive.
#
#   tools/tesla_analysis/pull_logs.sh                  # every route
#   tools/tesla_analysis/pull_logs.sh 00000010         # one route
#   OP_DEVICE=10.32.240.78 tools/tesla_analysis/pull_logs.sh
set -euo pipefail

DEVICE="${OP_DEVICE:-10.32.240.78}"
LOG_ROOT="${OP_LOG_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)/op-logs}"
REMOTE=/data/media/0/realdata
FILTER="${1:-}"

mkdir -p "$LOG_ROOT"
echo "device $DEVICE -> $LOG_ROOT"

segs=$(ssh -o StrictHostKeyChecking=no "comma@$DEVICE" "cd $REMOTE && ls -d *--*/ | tr -d /")
[ -n "$FILTER" ] && segs=$(echo "$segs" | grep "^$FILTER") || true

total=0; got=0
for seg in $segs; do
  total=$((total + 1))
  if [ -s "$LOG_ROOT/$seg/rlog.zst" ]; then
    continue
  fi
  mkdir -p "$LOG_ROOT/$seg"
  # A segment still being written has no rlog yet; skip rather than fail the run.
  if scp -q -o StrictHostKeyChecking=no "comma@$DEVICE:$REMOTE/$seg/rlog.zst" "$LOG_ROOT/$seg/" 2>/dev/null; then
    got=$((got + 1))
    echo "  $seg"
  else
    rmdir "$LOG_ROOT/$seg" 2>/dev/null || true
    echo "  $seg -- no rlog, skipped"
  fi
done

echo "$got new of $total segments; $(du -sh "$LOG_ROOT" | cut -f1) total"
