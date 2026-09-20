#!/usr/bin/env bash
set -eo pipefail

# The source bind mount is read-only. Preserve container builds between runs.
mkdir -p /workspace/repo
rsync -a --delete \
  --exclude='.git' --exclude='__pycache__' \
  --exclude='CMakeFiles' --exclude='CMakeCache.txt' \
  --exclude='*.o' --exclude='*.a' --exclude='*.so' --exclude='*.so.*' \
  --exclude='/build/' --exclude='/install/' --exclude='/log/' --exclude='/bags/' \
  --exclude='/agilib/build/' \
  --exclude='/agilib/externals/acados-src/build*/' \
  --exclude='/agilib/externals/acados-src/lib/' \
  --exclude='/agilib/externals/acados-src/bin/' \
  /source/ /workspace/repo/

export ACADOS_ROOT=/workspace/repo/agilib/externals/acados-src
if [[ ! -f "$ACADOS_ROOT/external/blasfeo/CMakeLists.txt" ||
      ! -f "$ACADOS_ROOT/external/hpipm/CMakeLists.txt" ]]; then
  echo 'Missing vendored acados sources/submodules in the host repository.' >&2
  exit 1
fi
cmake -S "$ACADOS_ROOT" -B /workspace/acados-build \
  -DCMAKE_BUILD_TYPE=Release -DACADOS_INSTALL_DIR="$ACADOS_ROOT"
cmake --build /workspace/acados-build --parallel "${BUILD_JOBS:-4}"
cmake --install /workspace/acados-build

# ROS setup scripts are not compatible with nounset.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$ACADOS_ROOT/lib:${LD_LIBRARY_PATH:-}"
cd /workspace/repo
if [[ "${1:-}" == bash ]]; then
  shift
  exec bash "$@"
fi
exec python3 betaflight_sitl/run.py --betaloop-home "$BETALOOP_HOME" "$@"
