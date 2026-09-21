# State acquisition time; odom is ENU and base_link is FLU.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Rotation from body FLU to world ENU.
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rates
geometry_msgs/Vector3 acceleration
# All steady times use this Linux host's CLOCK_MONOTONIC domain.
string clock_id
float64 published_steady_time
float64 imu_receive_time
float64 rtk_receive_time
builtin_interfaces/Time rtk_stamp
uint64 reset_counter
bool initialized
bool rtk_fixed
bool heading_valid
bool accuracy_ok
bool synchronized
# GNSS metadata; legacy RTK flags above retain their original meaning.
string navigation_source
string navigation_session
uint8 fix_type
bool clock_aligned
bool accuracy_known
bool navigation_valid
# Readiness derives from measured checks; original RTK/PPS flags remain untouched.
bool imu_ready
bool estimator_ready
bool navigation_ready
bool navigation_accuracy_ok
string readiness_reason
# Posterior covariance at covariance_stamp (not propagated to header.stamp).
builtin_interfaces/Time covariance_stamp
float64[3] position_variance
float64[3] velocity_variance
float64 heading_variance
float64 navigation_innovation_squared
uint64 navigation_rejections
# Consecutive accepted accurate updates, capped at the configured readiness count.
uint32 navigation_accepted_updates
