std_msgs/Header header
bool imu_calibrated
bool converged
# GNSS readiness is distinct from legacy RTK calibration/convergence evidence.
bool imu_ready
bool estimator_ready
bool navigation_ready
bool config_verified
bool thrust_calibrated
bool geofence_ok
bool transport_healthy
float64 battery_voltage
string reason
