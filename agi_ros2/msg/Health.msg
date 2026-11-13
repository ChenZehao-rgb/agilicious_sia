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
# Battery source stamp, unchanged by summary/relay publication.
builtin_interfaces/Time battery_stamp
float64 battery_voltage
float64 battery_current_a
float64 battery_consumed_mah
float64 battery_remaining_percent
float64 battery_temperature_c
string reason

# Independent FC estimator summary; does not imply IMU calibration or ROS EKF convergence.
string state_source
string state_session
uint64 state_reset_counter
bool fc_estimator_healthy
bool msp_healthy
bool control_output_healthy
