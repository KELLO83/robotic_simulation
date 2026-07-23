#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/home/rr/isaac-sim-standalone-6.0.0-linux-x86_64}"
ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -x "${ISAAC_SIM_PATH}/python.sh" ]]; then
  echo "Isaac Sim python.sh not found: ${ISAAC_SIM_PATH}/python.sh" >&2
  exit 1
fi

exec "${ISAAC_SIM_PATH}/python.sh" "${ISAAC_ROOT}/apps/run_stage4.py" "$@"
