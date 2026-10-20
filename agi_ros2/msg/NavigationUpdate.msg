# Optional exact EKF update diagnostics. All matrix arrays are row-major.
# header is observation update epoch, not the callback or physical sensor epoch.
std_msgs/Header header
string observation_type
string clock_id
string navigation_session
uint64 update_sequence
float64 received_steady_time
float64 processing_steady_time
builtin_interfaces/Time latest_imu_stamp
float64 observation_delay
bool prior_valid
bool committed
uint8 accepted_groups
uint8 rejected_groups
# State: p(3), quaternion(wxyz), v(3), gyro bias(3), accel bias(3), baro bias.
float64[17] prior_state
float64[17] posterior_state
float64[289] prior_covariance
float64[289] posterior_covariance
# Observation rows: px, py, pz, vx, vy, vz, yaw. Residual is h(prior)-z, yaw wrapped.
float64[7] measurement
float64[7] prediction
float64[7] residual
float64[119] observation_matrix
float64[49] measurement_covariance
float64[49] innovation_covariance
# Five groups: XY position, XY velocity, heading, height, vertical velocity.
float64[35] whitened_innovation
float64[5] nis
float64[5] nis_limit
uint32[5] degrees_of_freedom
uint8[5] reason
# Each of seven columns is -K_i * residual_i, before quaternion normalization.
float64[119] correction_contributions
