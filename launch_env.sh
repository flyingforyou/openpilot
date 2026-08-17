#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.6"
fi

export STAGING_ROOT="/data/safe_staging"

# 0.11.2 takes acados, json11 and the rest from PyPI instead of vendoring them under third_party,
# and the AGNOS system venv predates that -- so on a device the launcher would otherwise start
# manager with an interpreter that cannot import capnp. Prefer the project venv when `uv sync` has
# made one; DIR is set by launch_chffrplus.sh before this file is sourced.
if [ -n "$DIR" ] && [ -x "$DIR/.venv/bin/python3" ]; then
  export PATH="$DIR/.venv/bin:$PATH"
  export VIRTUAL_ENV="$DIR/.venv"
fi
