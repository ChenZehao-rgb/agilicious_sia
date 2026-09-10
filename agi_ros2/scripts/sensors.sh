#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash
  else source /opt/ros/humble/setup.bash; fi
fi
source "$repo/install/agi_ros2/local_setup.bash"
exec ros2 run agi_ros2 gazebo_sensors "$@"
