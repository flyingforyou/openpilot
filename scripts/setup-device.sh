#!/usr/bin/env bash
# Prepare a comma device to build and run this branch.
#
# Everything here was worked out the hard way on 10.0.0.56 and none of it is guessable from the
# tree, so it lives in the repo rather than in someone's shell history. Safe to re-run.
#
#   1. uv's python and cache go on /data. Their defaults are under $HOME, which on AGNOS is a
#      100 MB overlay -- the cache fills it during the first sync, and the interpreter does not
#      survive a reboot. The project venv is only a symlink to that interpreter, so openpilot
#      comes back up unable to import capnp and nothing starts. This is why the tuning page went
#      missing after a restart.
#
#   2. eigen. rednose includes <eigen3/Eigen/Dense> and 0.11.2 provides it nowhere -- not
#      SConstruct, not pyproject, not the lock file. Header-only, so it is unpacked on /data and
#      reached through an eigen3 link at the repo root, which SConstruct's CPPPATH already covers.
#      Nothing is written to the read-only system.
#
#   3. uv sync --all-extras. comma-deps-ncurses sits in the tools extra and SConstruct imports
#      ncurses at read time, so a plain sync leaves the build unable to start.
#
# After this, scons builds and launch_env.sh finds .venv on its own.
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"

if [ ! -d /data ]; then
  echo "not a comma device (/data missing) -- run 'uv sync --all-extras' instead"
  exit 1
fi

export UV_PYTHON_INSTALL_DIR=/data/uv-python
export UV_CACHE_DIR=/data/uv-cache
export UV_PROJECT_ENVIRONMENT="$DIR/.venv"

echo "=== eigen ==="
if [ -f /data/eigen/eigen3/Eigen/Dense ]; then
  echo "  already unpacked"
else
  tmp=$(mktemp -d)
  curl -sL -o "$tmp/eigen.tar.gz" \
    https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  tar xzf "$tmp/eigen.tar.gz" -C "$tmp"
  mkdir -p /data/eigen/eigen3
  cp -r "$tmp"/eigen-3.4.0/Eigen /data/eigen/eigen3/
  cp -r "$tmp"/eigen-3.4.0/unsupported /data/eigen/eigen3/ 2>/dev/null || true
  rm -rf "$tmp"
  echo "  unpacked to /data/eigen"
fi
ln -sfn /data/eigen/eigen3 eigen3
echo "  eigen3 -> $(readlink eigen3)"

echo ""
echo "=== dependencies ==="
uv sync --all-extras
echo "  venv: $(readlink -f .venv/bin/python3)"

echo ""
echo "=== done. build with: uv run scons -j4 ==="
