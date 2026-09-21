# Immutable control-cycle evidence. Times use the enclosing ControlCommand's
# clock_id domain: CLOCK_MONOTONIC, or ROS simulation seconds with ':ros'.
# solve_seconds is always measured with the wall steady clock.
# Consumers must not replace acquisition/command times with receipt times.
float64 now
float64 imu_time
float64 rtk_time
float64 rc_time
float64 command_time
float64 solve_seconds
bool rtk_fixed
bool heading_valid
bool accuracy_ok
bool imu_calibrated
bool synchronized
bool converged
bool imu_ready
bool estimator_ready
bool navigation_ready
bool clock_aligned
bool accuracy_known
bool config_verified
bool thrust_calibrated
bool geofence_ok
bool msp_healthy
bool command_valid
bool controller_warm
bool armed
bool auto_switch
bool kill
bool rc_link
