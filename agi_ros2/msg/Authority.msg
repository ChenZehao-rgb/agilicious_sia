# Acquired physical receiver state, or explicit simulated receiver input.
std_msgs/Header header
# Header is the minimum RC/status source stamp. Hardware status is HEARTBEAT receipt.
builtin_interfaces/Time rc_stamp
builtin_interfaces/Time status_stamp
# Legacy field names: hardware stores mapped RC sample / HEARTBEAT receipt CLOCK_MONOTONIC times.
# MSP diagnostic tools may store matched-request times. No hardware MSP request is made.
# Never reconstruct these from the current ROS clock offset.
string clock_id
float64 rc_request_steady_time
float64 status_request_steady_time
# False until this session has decoded valid physical mode evidence.
bool receiver_valid
string session_id
bool armed
# Physical CH5 request, separate from FC-confirmed armed.
bool arm_switch
bool auto_switch
bool kill
bool rc_link
uint16[4] manual_aetr
