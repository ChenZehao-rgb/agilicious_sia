# Acquired physical receiver state, or explicit simulated receiver input.
std_msgs/Header header
# False until this session has decoded valid physical mode evidence.
bool receiver_valid
string session_id
bool armed
bool auto_switch
bool kill
bool rc_link
uint16[4] manual_aetr
