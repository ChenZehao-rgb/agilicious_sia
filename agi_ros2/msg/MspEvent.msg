# ROS timestamp for rosbag alignment; steady_time for physical UART timing.
std_msgs/Header header
float64 steady_time
# tx, rx, error, timeout, late, transport_error
string event
uint16 code
uint8[] payload
uint64 errors
# Time from request write to reply arrival; NaN when not applicable.
float64 latency_seconds
