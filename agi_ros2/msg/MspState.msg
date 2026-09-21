# Snapshot of decoded evidence; component timestamps are immutable request times.
std_msgs/Header header
string session_id
builtin_interfaces/Time rc_stamp
builtin_interfaces/Time status_stamp
builtin_interfaces/Time battery_stamp
bool config_verified
bool receiver_valid
bool armed
bool auto_switch
bool kill
bool rc_link
bool transport_healthy
float64 battery_voltage
string reason
