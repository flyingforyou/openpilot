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
#   2. uv sync --all-extras. comma-deps-ncurses sits in the tools extra and SConstruct imports
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

# AGNOS puts UV_PYTHON_PREFERENCE=only-system in the boot environment so uv uses its own venv.
# That makes uv refuse to fetch or even use a managed interpreter, and 0.11.2 wants 3.12.13, which
# AGNOS does not have -- so at boot the sync failed with "No interpreter found" while the same
# command over ssh worked, because an ssh session does not inherit that variable. Overriding the
# install dir alone was not enough; the preference has to go with it.
export UV_PYTHON_PREFERENCE=managed
# The boot environment also points VIRTUAL_ENV at the AGNOS venv, which uv warns about and
# ignores. Clear it so the intent is unambiguous.
unset VIRTUAL_ENV

echo "=== dependencies ==="
uv sync --all-extras
echo "  venv: $(readlink -f .venv/bin/python3)"

echo ""
echo "=== done. build with: uv run scons -j4 ==="
