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

string state_source
uint64 fc_time_usec
float64 fc_sample_age
float64 fc_receive_age
float64 fc_publish_delay
# Explicit identity and timing; success above is command_valid, not acados status.
float64 control_session_start
uint64 command_sequence
uint64 fc_generation
string fc_session
bool computation_requested
float64 fc_sample_age_before
float64 fc_sample_age_after
uint64 ignored_states
uint64 coalesced_states
uint64 warm_resets
