# Split-node validation (2026-09-10)

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
