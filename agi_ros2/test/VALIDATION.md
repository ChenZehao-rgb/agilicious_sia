# Split-node validation (2026-09-10)

## 2026-09-30 IMU/GNSS/barometer height fusion

Validated on local x86-64 / ROS 2 Humble, with synthetic inputs and PTYs only:

- `./agi_ros2/scripts/build.sh`: passed, including the Barometer message, receiver, fusion and dependent nodes.
- `agilib/build/tests --gtest_filter='EkfImuRtk.*:EkfImuBaro.*'`: 24/24 passed. Covers same-time updates,
  delayed sorted input, NIS rejection and rollback, bias random walk, GNSS-constrained drift, covariance PSD,
  and reference alignment that preserves navigation covariance instead of inventing absolute-height information.
- `test_barometer_reference.cpp`: 3/3 passed with `g++ -std=c++17 -Wall -Wextra -Werror`, covering
  nonzero local height, pressure direction/variance, stationarity, duplicate samples and reference reset.
- `test_mavlink_sensor.py`: 15 serial tests plus 1 launch test passed, including message 29 interval commands,
  pressure/temperature units, timestamp deduplication, stale samples, 32-bit millisecond wrap, source restart and collision.
- `test_baro_fusion.py`: 4/4 passed on the final installed build (12.595 s): reference height, pressure outlier,
  loss/recovery, source invalidation, pressure-first same-time navigation, and observations beyond the reorder window.
- `test_baro_hardware_pipeline.py`: 1/1 passed (2.917 s). Six real nodes with two PTYs fuse MAVLink pressure
  while GNSS flight limits are unconfigured. Readiness remains false, shadow computation runs, ARM/AUTO preserves
  the established reference, and the captured MSP stream contains no code 200.
- Existing delayed RTK without heading, fusion/control/sensor-loss, and simulated clock rewind tests: 3/3 passed.
  Existing GNSS session/unknown accuracy and innovation/readiness tests: 2/2 passed in the final focused run.
  Existing MPC and GEO hardware shadow regressions: 2/2 passed.
- Relevant launch/config checks: 7/7 passed. The full config suite has 21/24 passing; three pre-existing tests
  still assume zero hardware mass/rate/thrust placeholders. Those failures were reproduced before these changes.
- New C++ files and changed C++ ranges passed formatting with clang-format 23.1.1, Tab/140-column checks,
  Python syntax compilation and `git diff --check`.

An earlier concurrent run had two pressure-reference initialization timeouts and one GNSS initialization timeout;
their cause was not established. Focused reruns and the final runs above passed without relaxing production gates.
Reference diagnostics now identify the individual blocking condition. These tests do not establish CM5 timing,
physical UART throughput, sensor installation/airflow behavior or real flight accuracy. Hardware remains in shadow mode.

## 2026-09-30 quadratic thrust mapping

Validated the optional manufacturer-estimated quadratic mapping on local x86-64 / ROS 2 Humble.
The profile remains in shadow mode; the manufacturer's full-input thrust is a model scale, not a controller limit.

- `./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=ON`: passed, including regenerated messages and the Gazebo adapter.
- `test_runtime_config.py`: 21 tests passed, covering legacy table selection, quadratic parameters and selected-model validation.
- `test_runtime_profile_nodes.py RuntimeProfileNodes.test_direct_hardware_profile_rejects_invalid_quadratic_before_uart`:
  passed; 22 invalid configurations failed before UART opening, and table mode ignored an invalid unselected quadratic model.
- New output-node tests passed: RC 1170 at the 734 g hover estimate, identical output at 12/16/24 V,
  invalid battery and stale Health revocation, readiness independent of calibration, out-of-range thrust rejection,
  shadow silence, and read-only model parameters.
- `test_hardware_pipeline.py`: all 9 tests passed using PTY MSP and synthetic MAVLink/GNSS, including
  quadratic readiness propagation, AUTO/KILL, battery-only telemetry loss, table-mode MPC/GEO and shadow behavior.
- CTest `betaflight_hw_test`, `hardware_pilot_test`, `betaflight_rc_mapper_test`, `quadratic_thrust_model_test`:
  4/4 passed after rebuilding. GTest `HardwareGeo.*`: 4/4 passed.
- New C++ files passed full clang-format 18 checks; modified C++ ranges passed formatting, Tab/140-column checks.
  Python syntax and `git diff --check` passed.

The first complete `test_node_pipeline.py` run passed 11/14 tests. The delay-mode acceptance, legacy MSP
pseudo-UART, and simulated clock/sensor-loss cases failed; reported RC timestamp ages in the latter two
exceeded the existing freshness limits. All three passed when rerun as a focused group, along with the
new parameter-immutability test. No production timing threshold was relaxed; a clean complete-suite run
is not claimed. These checks did not open a physical UART, arm hardware, or validate real thrust accuracy.

## SITL timing correction

The earlier wall-timer behavior described below is superseded for
`mode=sitl,use_sim_time=true`: control now ticks at 100 Hz simulation time;
safety evidence uses an explicit `:ros` clock domain. Output retains a separate
250 ms wall command-stream deadline. Hardware timing is unchanged.

Added process-level regression cases for repeated 80 ms clock/sensor pauses,
0.5x clock rate, a 350 ms stall, IMU loss with an advancing clock, control-process
suspension, clock rewind/recovery, and KILL with a frozen clock.
These new cases have **not been executed**: the requested validation is compilation
only, without running simulation or dynamic tests. The historical results below
are not validation of this timing correction.

`./agi_ros2/scripts/build.sh` passed (1 package, 30.1 s); Python syntax compilation
and `git diff --check` passed. No Gazebo, Betaflight or node-pipeline test was run.

## Earlier split-node validation

Validated on the local ROS 2 Humble / x86_64 development machine:

- `./agi_ros2/scripts/build.sh`: passed with the existing C++17/Eigen/acados ABI.
- `./agi_ros2/scripts/test.sh`: all three process-level tests passed (8.7 s).
  - Synthetic IMU/RTK → fused state → MPC warmup/AUTO → loopback UDP.
  - IMU stop revokes control/output; resumed sensors do not reauthorize a held
    AUTO-high switch; a healthy low/high cycle restores operation.
  - Output startup with AUTO high is blocked. Command timeout, NaN thrust,
    stale producer evidence and a different host clock ID do not authorize output.
  - PTY MSP v1 frames have code 200, exactly four AETR channels and correct
    checksums; hardware pitch/yaw use the FRD sign conversion. KILL stops writes.
- `clang-format --dry-run --Werror` with `agi_ros2/.clang-format`: passed for
  all new `.h/.cpp` files using clang-format 23.1.0.
- Python syntax compilation and `git diff --check`: passed.
- `run.py --ros2 --no-build --duration 8`: unarmed Gazebo/Betaflight SITL smoke
  passed; all four nodes launched and the runner exited with status 0.
  Observed 7,735 fused states (7,647 initialized), 1,547 ground-truth messages,
  778 control-cycle messages and 2,333 output-status messages. Acquisition-time
  rates were 1,000 Hz for fusion and 200 Hz for ground truth. Control uses a
  10 ms wall timer; output status also publishes on the 5 ms watchdog.

SITL smoke output is in `build/ros2_split_validation/sitl_smoke.log` and
`sitl_smoke.json`. The summary's first-to-last ROS-time averages for command and
output status include the initial `/clock` startup jump and are not steady-state
rates. No physical UART was opened; no vehicle was armed in the SITL smoke.
These checks establish node wiring, message semantics and basic fault behavior;
they are not a CPC33 trajectory accuracy regression or physical-flight validation.

The fusion algorithm remains constant-velocity delayed-RTK extrapolation plus
AHRS/IMU propagation, now executed at the IMU callback rate. This change does not
claim to eliminate the previously measured RTK correction sawteeth.

## 2026-09-12 MSP ROS 2 integration

- Full ROS 2 package build succeeded with `AGI_ROS2_GAZEBO=OFF` on x86-64/Humble.
- `test_msp_node.py` passed using the installed ROS node, a PTY FC and rosbag2:
  RC 100.02 Hz, ATTITUDE 20.00 Hz, STATUS 5.00 Hz; disabled categories absent;
  missing GPS did not block bench RC; fragmented replies, ACKs and response latency recorded.
  Read back the actual bag: 1204 `/msp/events`, 96 `/msp/attitude`, 24 `/msp/status`,
  5 `/msp/config` messages in that run.
- Updated the existing UART fixture to reply to telemetry and check that KILL stops RC
  while telemetry continues; telemetry loss invalidates flight transport health.
  This hardware test passed in isolation, as did the fusion/control/sensor-loss case.
- The complete control suite is NOT a clean pass: wall-time tests intermittently reject
  authorization. The remaining UDP test failure recorded authority reception age
  0.107–0.146 s and stamp age 0.260–0.298 s, beyond the existing 0.1 s limit.
  Other isolated runs passed that same UDP test. Safety limits were not relaxed;
  treat this as an unresolved real-time/DDS test-environment limitation, not hardware certification.
- No Raspberry Pi UART, physical FC or actual flight was exercised.
