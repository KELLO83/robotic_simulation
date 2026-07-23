#!/usr/bin/env bash

_drlnav_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_drlnav_local_ros="${_drlnav_workspace}/.local_ros_deps/prefix/opt/ros/humble"

source /opt/ros/humble/setup.bash

if [[ -d "${_drlnav_local_ros}" ]]; then
  export AMENT_PREFIX_PATH="${_drlnav_local_ros}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${_drlnav_local_ros}:${CMAKE_PREFIX_PATH:-}"
fi

if [[ -f "${_drlnav_workspace}/install/setup.bash" ]]; then
  source "${_drlnav_workspace}/install/setup.bash"
fi

export DRLNAV_BASE_PATH="${_drlnav_workspace}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:${_drlnav_workspace}/src/turtlebot3_simulations/turtlebot3_gazebo/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:${_drlnav_workspace}/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib"

unset _drlnav_local_ros
unset _drlnav_workspace
