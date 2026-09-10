#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash
  else source /opt/ros/humble/setup.bash; fi
fi
source "$repo/install/agi_ros2/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$repo/build/ros2_split_test_logs}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
exec python3 "$repo/agi_ros2/test/test_node_pipeline.py" "$@"
