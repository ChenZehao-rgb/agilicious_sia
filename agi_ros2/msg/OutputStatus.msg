std_msgs/Header header
string clock_id
float64 steady_time
bool transport_healthy
# Mapping readiness is independent of measured thrust calibration.
bool thrust_mapping_ready
bool thrust_calibrated
string thrust_model
string thrust_model_source
# Monotonically increasing within this output-node process.
uint64 fault_count
# Process instance identity lets control detect output restarts.
float64 session_start
bool override_active
string reason
string last_fault
float64 write_seconds
float64 command_age
# Last independent gate evaluation, not the status publication time. May be NaN.
float64 output_check_steady_time
float64 control_session_start
uint64 command_sequence
float64 fc_sample_age_at_check
uint64 fc_generation
string fc_session
