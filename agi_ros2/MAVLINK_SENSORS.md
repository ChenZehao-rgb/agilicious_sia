# MAVLink sensor input (ROS 2 Humble)

This driver reads a **dedicated** UART. `command_output_node` retains the existing
MSP command/RC/status/battery channel. It does not publish `/authority`, `/health`,
`/sensors/rtk`, control commands, ARM or flight-mode requests. Ordinary GPS topics
alone do not satisfy the existing RTK fusion/heading contract.

## Build and run

Run the existing `./agi_ros2/scripts/build.sh`, then source ROS and
`install/agi_ros2/local_setup.bash`. The pinned generated MAVLink C headers are
vendored under `third_party/mavlink` with provenance and SHA-256 hashes. No network
fetch or absolute path to the Betaflight checkout is needed to build this driver.

Start telemetry only (replace the device with your actual UART):

```sh
ros2 launch agi_ros2 mavlink_sensors.launch.py device:=/dev/serial/by-id/YOUR_MAVLINK_UART
```

Optional settings:

```sh
ros2 launch agi_ros2 mavlink_sensors.launch.py \
  device:=/dev/serial/by-id/YOUR_MAVLINK_UART baud:=921600 \
  gps_mode:=gnss imu_rate_hz:=500 gps_rate_hz:=10 attitude_rate_hz:=100
```

`gps_mode:=rtk` requires a real `fix_type=6` before diagnostics report GPS ready;
float/ordinary fixes remain visible but are not reported as RTK ready. The setting
does not configure a receiver, inject corrections, or manufacture RTK capability.
Current quality decoding covers UBX PVT receiver flags. Other protocols and
independent dual-antenna heading require a receiver-specific extension.

For the existing flight launch, add:

```text
mode:=hardware mavlink_enabled:=true mavlink_device:=/dev/serial/by-id/YOUR_MAVLINK_UART
```

Supply the existing hardware parameters, thrust calibration and **different** MSP
`device` as before. MAVLink is disabled by default; it cannot be enabled in SITL
flight mode. Existing simulator behavior stays unchanged. Starting a flight launch
still requires the existing genuine RTK/authority/health inputs; these are not
supplied by this sensor adapter. Use the standalone launch for ordinary GPS tests.
Do not simultaneously publish simulation and MAVLink IMU on the same topic.
The driver detects another IMU publisher during the 1 Hz diagnostic check and
suppresses sensor output until the conflict clears.

When enabled, flight launch disables duplicate MSP GPS polling. MSP attitude
polling is disabled only if MAVLink attitude is requested. MSP health continues
to represent the command transport. Do not also launch `betaflight_msp_node` on
the command output node's serial device.

## Flight controller setup

Use the updated standard `GEPRC_TAKER_H743` build, not H743V2 or MINI. Preserve the
existing receiver/GPS/MSP configuration. UART2 is the standard board's default
receiver port. If UART4 is genuinely free, an example is:

```text
feature TELEMETRY
serial 3 512 115200 115200 921600 115200
set mavlink_imu_rate = 500
set mavlink_pos_rate = 10
set mavlink_extra1_rate = 0
save
```

FC T4 → host RX, FC R4 ← host TX, common GND, compatible 3.3 V UART logic.
Both directions are required for TIMESYNC and rate acknowledgements. Never use
RS-232 voltage levels. The adapter requests rates on startup/reconnect using
`MAV_CMD_SET_MESSAGE_INTERVAL`; ACKs and measured receive rates are reported.
Rate changes are volatile. Configuring GPS telemetry does not enable/configure
the separate GPS receiver UART or change its native solution frequency.

## Topics and conversions

Names below are relative to the node namespace (root by default).

| Topic | Type | Meaning |
|---|---|---|
| `sensors/imu` | `sensor_msgs/Imu` | `base_link` FLU, m/s² and rad/s |
| `sensors/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link`, degrees, WGS84 ellipsoid metres if known |
| `sensors/gps/velocity` | `geometry_msgs/TwistStamped` | `gps_enu`, East/North/Up m/s |
| `sensors/fc_heading` | `agi_ros2/Heading` | FC estimated yaw, ENU radians, with field-valid flag |
| `sensors/fc_attitude` | `geometry_msgs/QuaternionStamped` | FLU→ENU rotation; parent `gps_enu`, optional |
| `sensors/mavlink/status` | `diagnostic_msgs/DiagnosticArray` | reliable 1 Hz link, sync, fix grade and rate diagnostics |

Sensor topics use SensorDataQoS (best effort, volatile). FRD→FLU negates Y/Z for
both acceleration and angular velocity. Units are already SI. Specific force
retains gravity: a stationary level vehicle reads approximately +9.80665 on ROS Z.
`orientation_covariance[0] = -1` because HIGHRES_IMU has no attitude. Optional
`acceleration_variance` and `angular_velocity_variance` each take three finite,
nonnegative diagonal entries; all zero means unknown. Set measured variances,
not arbitrary high confidence values. The attitude topic is separate and is not
fed back into the IMU fusion stream. The controller's absolute yaw can be
unreferenced without a reliable heading source; COG is not stationary heading.

The FC must be configured with its actual board alignment so the reported frame
really is vehicle `base_link`; antenna offsets/extrinsic transforms are the
integrator's responsibility. The driver does not create an `odom` origin or TF.

GPS velocity is published only for a matching valid GPS_RAW_INT solution (same
boot millisecond). The two MAVLink messages are paired in a bounded queue, so
arrival order does not matter. Missing/invalid NED velocity is not replaced by
zeros. No-fix produces STATUS_NO_FIX and NaN coordinates; no valid velocity is
published. RTK float/fixed grade remains in diagnostics because NavSatFix has no
native RTK-grade enumeration. Reported horizontal/vertical accuracies produce
an *approximated* diagonal covariance; HDOP is not interpreted as metres.

`altitude_source` defaults to `unknown`: NavSatFix altitude is NaN. Explicitly use
`altitude_source:=ellipsoid` only with the corrected FC and a provider that supplies
true ellipsoid height. This FC uses **INT32_MIN** for unavailable `alt_ellipsoid`
(a documented convention of this firmware, not a general MAVLink sentinel).
Unknown accuracy extensions use zero. MSL altitude is separately reported in
diagnostics. MSL and ellipsoid height are never silently interchanged.

## Time and failure behavior

IMU timestamps are captured after successful calibrated/filtered sensor updates,
with the latest filtered gyro snapshot. GPS timestamps are FC complete-solution
update times. Neither is a sensor hardware timestamp; GPS serial/module latency
and sensor filter delays remain. Frames are not a lossless IMU sample archive.
The FC extends its microsecond counter to 64 bits; GPS millisecond message fields
retain their standard 32-bit wrap behavior.

The node sends TIMESYNC every 100 ms, accepts only its own outstanding replies,
rejects RTT >20 ms and averages midpoint offsets. Five valid samples are required
before publishing sensors. The monotonic offset is mapped into ROS system time.
`use_sim_time=true` is rejected. This is approximate clock alignment, not PPS or
GNSS measurement-time synchronization. No synchronized flag is fabricated for RTK.

Backwards device time, large clock discontinuities, or a >50 ms change in the
ROS/monotonic relationship clear the epoch, pairing queues and sample history.
Missing sync for 2 s invalidates it; link timeout is 2 s. UART opens and reconnects
use exclusive access, bounded RX/TX queues and partial-write handling. Reconnect
repeats synchronization and rate requests. Duplicate/out-of-order samples and
stale samples are discarded (IMU older than 50 ms; other streams older than 500 ms).
Future stamps beyond 20 ms are discarded. These adapter limits do not relax the
existing fusion/control freshness thresholds. Physical end-to-end latency must
still fit those stricter thresholds before attempting closed-loop operation.

Inspect `sensors/mavlink/status` for sync, GPS freshness/fix type, field validity,
CRC/source errors, stale/duplicate frames, rate ACK status and host-time publication
rates. Link presence alone is not sensor health; stopped IMU produces a warning.

## Validation

```sh
source /opt/ros/humble/setup.bash
source install/agi_ros2/local_setup.bash
python3 agi_ros2/test/test_mavlink_sensor.py -v
python3 agi_ros2/test/test_node_pipeline.py -v
```

The sensor test uses a pseudo-UART and a real compiled ROS node. It requires
pymavlink and local DDS access, and covers fragmented/coalesced frames, CRC/source
rejection, units/axes, duplicates, invalid GPS/velocity/height, GNSS/RTK selection,
sync establishment and high RTT rejection, restart, reconnect, and extended time.
No physical flight controller is opened. Betaflight's separate SITL harness tests
500 Hz fresh-sample output and verifies that a 50 Hz source remains near 50 Hz.
Physical H743/UART throughput, actual GNSS latency, PPS synchronization and flight
behavior are not established by these tests.


## Heading from GLOBAL_POSITION_INT.hdg

`sensors/fc_heading` directly converts this centidegree compass field:
`heading = remainder(pi/2 - hdg*pi/18000, 2*pi)`.
North → +pi/2, East → 0, South → -pi/2, West → -pi.
`UINT16_MAX` or other values outside 0..35999 produce `valid=false` and NaN.
The message is independent of the optional ATTITUDE stream and follows received
GLOBAL_POSITION_INT messages (normally GPS solution rate). This firmware saves
yaw into the GPS snapshot, so position, velocity and heading share an update time.
When the firmware suppresses GLOBAL_POSITION_INT for unavailable GPS velocity,
this topic also stops; it is not an independent 100 Hz heading source.

The source is FC `attitude.yaw`, not COG or dual-antenna RTK heading. Field validity
only describes representation, not true-north observability or convergence.
Without an absolute heading reference, FC yaw may have an arbitrary offset/drift.
No changes are made to state_fusion_node or its RTK heading validity requirements.


## Validation result (2026-09-16)

Humble build and all eight new sensor/launch pseudo-UART integration tests passed.
Final Betaflight SITL measured approximately 500.1 Hz fresh IMU samples, 100 Hz
ATTITUDE and 9.8 Hz new GPS solutions. A deliberately 50 Hz sensor source produced
48.8 Hz unique IMU output, not repeated 500 Hz data.

The existing control pipeline suite was not consistently green (9/10 then 8/10).
Warm-up/sim-clock cases passed isolated reruns, but the final isolated MSP override
case still failed with RC ROS-stamp ages exceeding the existing 100 ms deadline.
Those control/fusion/MSP sources and safety thresholds were not modified; the
failure's root cause remains unestablished. Sensor-only test success is not a full
flight-pipeline regression pass. No physical flight controller was flashed/tested.
