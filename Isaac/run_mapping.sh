#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${ISAAC_ROOT}/.." && pwd)"

set +u
source "${REPOSITORY_ROOT}/setup_humble.bash"
source "${ISAAC_ROOT}/ros2_ws/install/setup.bash"
set -u

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
exec ros2 launch isaac_stage4_slam isaac_mapping.launch.py "$@"
