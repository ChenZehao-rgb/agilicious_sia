# Immutable control-cycle evidence. Times are host CLOCK_MONOTONIC seconds.
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
