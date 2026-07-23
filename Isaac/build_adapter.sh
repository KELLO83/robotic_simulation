#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${ISAAC_ROOT}/.." && pwd)"

set +u
source "${REPOSITORY_ROOT}/setup_humble.bash"
set -u

cd "${ISAAC_ROOT}/ros2_ws"
colcon build \
  --merge-install \
  --symlink-install \
  --packages-select turtlebot3_isaac_adapter isaac_stage4_slam
