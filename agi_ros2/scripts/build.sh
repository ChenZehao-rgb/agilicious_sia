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
cmake -S "$repo/agi_ros2" -B "$repo/build/ros2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$repo/install/agi_ros2" \
  -DACADOS_ROOT="${ACADOS_ROOT:-$repo/agilib/externals/acados-src}" -DFETCH_ACADOS=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
cmake --build "$repo/build/ros2" --parallel "${BUILD_JOBS:-4}"
cmake --install "$repo/build/ros2"
