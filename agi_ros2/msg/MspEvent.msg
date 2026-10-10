# ROS timestamp for rosbag alignment; steady_time for physical UART timing.
std_msgs/Header header
float64 steady_time
# tx, rx, error, timeout, late, transport_error, write_deferred, write_late
# mode_timeout is nonfatal; mode_rx_unmatched has unknown request times/latency.
# Mode reply correlation remains unknown after a timeout until the session ends.
# write_deferred sends no bytes and has no request identity. write_late marks a
# complete read-only write; its reply still uses the original tx request time.
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
