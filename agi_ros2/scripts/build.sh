#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash
  else source /opt/ros/humble/setup.bash; fi
fi
export CCACHE_DIR="$repo/build/ccache"
export CCACHE_TEMPDIR="$repo/build/ccache/tmp"
mkdir -p "$CCACHE_TEMPDIR"
# colcon with --merge-install keeps both paths the ROS 2 entry points expect:
# install/agi_ros2/local_setup.bash (launch.sh) and
# install/agi_ros2/lib/agi_ros2/control_node (run.py --no-build check).
colcon build --base-paths "$repo/agi_ros2" \
  --install-base "$repo/install/agi_ros2" --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DACADOS_ROOT="${ACADOS_ROOT:-$repo/agilib/externals/acados-src}" -DFETCH_ACADOS=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
