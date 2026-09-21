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
# Matched request evidence, never refreshed by a republisher.
builtin_interfaces/Time request_stamp
float64 request_steady_time
string request_name
string session_id
