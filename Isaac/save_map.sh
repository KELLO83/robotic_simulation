#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${ISAAC_ROOT}/.." && pwd)"
MAP_PREFIX="${1:-${ISAAC_ROOT}/maps/stage4_waffle_pi}"

set +u
source "${REPOSITORY_ROOT}/setup_humble.bash"
source "${ISAAC_ROOT}/ros2_ws/install/setup.bash"
set -u

mkdir -p "$(dirname -- "${MAP_PREFIX}")"
if ! timeout 30 ros2 topic echo /map --once >/dev/null; then
  echo "ERROR: /map is not available. Start Isaac and run_mapping.sh first." >&2
  exit 1
fi
exec ros2 run nav2_map_server map_saver_cli -f "${MAP_PREFIX}"
