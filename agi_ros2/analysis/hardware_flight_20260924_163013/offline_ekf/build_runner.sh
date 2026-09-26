#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/../../../.." && pwd)"
build_dir="${AGILIB_BUILD_DIR:-$repo_dir/build/agi_ros2}"
if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
    echo "Missing configured agi_ros2 build: $build_dir. Set AGILIB_BUILD_DIR to an existing ROS build." >&2
    exit 1
fi
cmake --build "$build_dir" --target agilib --parallel 2
"${CXX:-c++}" -std=c++17 -O3 -DNDEBUG -march=native -fno-finite-math-only \
    -Wall -Wextra -DEIGEN_STACK_ALLOCATION_LIMIT=1048576 \
    -I"$repo_dir/agilib/include" -isystem /usr/include/eigen3 \
    "$script_dir/offline_ekf.cpp" -L"$build_dir/agilib" \
    -Wl,-rpath,"$build_dir/agilib" \
    -Wl,-rpath-link,"$repo_dir/agilib/externals/acados-src/lib" \
    -lagilib -pthread -o "$script_dir/offline_ekf"
echo "Built $script_dir/offline_ekf"
