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
# Mapped FC update epoch converted once at the accepted IMU callback.
# Re-publication and later ROS clock-offset changes must not refresh this value.
float64 imu_sample_steady_time
float64 rtk_receive_time
# Last accepted navigation sample, after bounded observation reordering.
builtin_interfaces/Time rtk_stamp
# Newest original navigation sample; receipt remains rtk_receive_time.
builtin_interfaces/Time navigation_sample_stamp
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
# Current predicted covariance; covariance_stamp equals header.stamp when valid.
builtin_interfaces/Time covariance_stamp
float64[3] position_variance
float64[3] velocity_variance
float64 heading_variance
float64 navigation_innovation_squared
uint64 navigation_rejections
# Consecutive accepted accurate updates, capped at the configured readiness count.
uint32 navigation_accepted_updates
# Delayed posterior quality is retained independently of the current prediction.
builtin_interfaces/Time posterior_covariance_stamp
float64[3] posterior_position_variance
float64[3] posterior_velocity_variance
float64 posterior_heading_variance
builtin_interfaces/Time navigation_attempt_stamp
builtin_interfaces/Time position_accepted_stamp
builtin_interfaces/Time velocity_accepted_stamp
builtin_interfaces/Time heading_accepted_stamp
bool heading_trusted
# 0 initializing, 1 tracking, 2 bounded prediction, 3 recovering; never resets pose.
uint8 navigation_phase
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 acceleration_bias

# FC path: initialized/IMU/EKF fields above are not applicable (false/NaN).
string state_source
bool acceleration_available
bool fc_state_valid
uint64 fc_time_usec
float64 fc_state_sample_steady_time
float64 fc_state_receive_steady_time
float64 fc_health_sample_steady_time
float64 fc_health_receive_steady_time
float64 fc_xy_observation_time
float64 fc_z_observation_time
uint16 fc_flags
uint32 fc_reset_counter
uint32 fc_state_sequence
