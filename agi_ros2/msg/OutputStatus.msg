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

# Reference-table mapping diagnostics; NaN when no successful current mapping exists.
string calibration_id
string reference_mapping_reason
float64 voltage_raw_v
float64 voltage_filtered_v
float64 voltage_sample_age_s
float64 reference_voltage_v
float64 voltage_exponent_p
float64 thrust_voltage_scale
float64 desired_total_thrust_n
float64 reference_lookup_thrust_n
float64 mapped_rc_throttle
float64 model_min_total_thrust_n
float64 model_max_total_thrust_n
uint32 calibration_schema_version
string calibration_provenance
float64 calibration_voltage_min_v
float64 calibration_voltage_max_v
float64 calibration_rc_min
float64 calibration_rc_max

string calibration_status
bool reference_lower_saturated
bool reference_idle_requested
