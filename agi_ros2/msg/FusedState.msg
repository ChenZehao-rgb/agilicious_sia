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
