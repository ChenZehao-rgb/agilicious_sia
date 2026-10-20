# Acquired physical receiver state, or explicit simulated receiver input.
std_msgs/Header header
# Each last successfully matched request; header is their minimum on hardware.
builtin_interfaces/Time rc_stamp
builtin_interfaces/Time status_stamp
# Original matched-request CLOCK_MONOTONIC times on this Linux boot.
# Never reconstruct these from the current ROS clock offset.
string clock_id
float64 rc_request_steady_time
float64 status_request_steady_time
# False until this session has decoded valid physical mode evidence.
bool receiver_valid
string session_id
bool armed
bool auto_switch
bool kill
bool rc_link
uint16[4] manual_aetr
