#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${ISAAC_ROOT}/.." && pwd)"
ALGORITHM="${1:-ddpg}"

set +u
source "${REPOSITORY_ROOT}/setup_humble.bash"
source "${ISAAC_ROOT}/ros2_ws/install/setup.bash"
set -u

exec ros2 run turtlebot3_drl train_agent "${ALGORITHM}"
