# Header is exactly the corresponding sensors/imu sample header.
std_msgs/Header header
string clock_id
string source_session
uint64 fc_time_usec
# Host CLOCK_MONOTONIC: frame dispatch, just before publish, just after publish.
float64 receive_steady_time
float64 published_steady_time
float64 publish_return_steady_time
# ROS mapped sample age; includes clock alignment uncertainty, not UART-only delay.
float64 mapped_sample_age
