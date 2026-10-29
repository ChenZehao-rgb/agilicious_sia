std_msgs/Header header
bool imu_calibrated
bool converged
# GNSS readiness is distinct from legacy RTK calibration/convergence evidence.
# FC sensor/calibration readiness, independent of cached fused-state age.
bool fc_imu_ready
bool imu_ready
bool estimator_ready
bool navigation_ready
bool config_verified
# Mapping readiness is independent of measured thrust calibration.
bool thrust_mapping_ready
bool thrust_calibrated
bool geofence_ok
bool transport_healthy
float64 battery_voltage
string reason

# Independent FC estimator summary; does not imply IMU calibration or ROS EKF convergence.
string state_source
string state_session
uint64 state_reset_counter
bool fc_estimator_healthy
bool msp_healthy
bool control_output_healthy

# Original MSP130 report identity, host monotonic request/receive times (not physical ADC epoch).
string battery_session
uint64 battery_sequence
float64 battery_request_steady_time
float64 battery_receive_steady_time
