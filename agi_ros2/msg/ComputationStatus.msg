std_msgs/Header header
bool shadow_only
bool state_valid
# Deprecated compatibility alias of controller_success, including when GEO is selected.
bool mpc_success
uint32 warm_cycles
bool trajectory_active
float64 reference_elapsed
float64 solve_seconds
# Wall duration through decision construction, including time alignment.
float64 cycle_seconds
float64 state_age
float64 navigation_age
float64 imu_age
string reason
# Fixed at process startup; switching requires a restart and fresh AUTO authorization.
string controller_type
bool controller_success
