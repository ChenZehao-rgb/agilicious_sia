#!/usr/bin/env bash
# Local builds, synthetic DDS and PTYs only. Never launches hardware sensors.
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
out="$repo/agi_ros2/analysis/fc_fused_timing_fix_20261027"
mkdir -p "$out/logs"
bash agi_ros2/scripts/build.sh -DAGI_ROS2_TIMING_TESTS=ON --parallel-workers 2 \
  --install-base build/fc_timing_install --cmake-force-configure > "$out/logs/build.log" 2>&1
source build/fc_timing_install/local_setup.bash
export AGI_ROS2_TEST_INSTALL="$repo/build/fc_timing_install"
export ROS_LOG_DIR="$out/logs/ros"
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
export FC_DELIVERY_REPORT="$out/synthetic_delivery.json"
export FC_OUTPUT_REPORT="$out/synthetic_output.json"
export FC_TIMING_REPORT="$out/full_pty_shadow.json"
export PYTHONPATH="$repo/agi_ros2/test:${PYTHONPATH:-}"
ctest --test-dir build/agi_ros2 --output-on-failure > "$out/logs/ctest.log" 2>&1
/usr/bin/python3 agi_ros2/tests/test_fc_control_delivery.py -v > "$out/logs/delivery_test.log" 2>&1
/usr/bin/python3 -m unittest -v test_fc_fused_pipeline test_node_pipeline test_runtime_profile_nodes test_fc_fused_launch \
  > "$out/logs/ros_regression.log" 2>&1
export CCACHE_DIR="$repo/build/ccache" CCACHE_TEMPDIR="$repo/build/ccache/tmp"
/usr/bin/python3 agi_ros2/tests/build_timer_baseline.py build/fc_timer_baseline > "$out/logs/build_baseline.log" 2>&1
export FC_TIMER_BASELINE="$repo/build/fc_timer_baseline/control_node"
export FC_COMPARISON_REPORT="$out/synthetic_comparison.json"
/usr/bin/python3 agi_ros2/tests/compare_fc_delivery.py > "$out/logs/compare_test.log" 2>&1
/usr/bin/python3 agi_ros2/tests/fc_timing_budget.py "$out/phase_budget.json" > "$out/logs/phase_budget.log"
echo "FC timing tests passed: $out"
