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
