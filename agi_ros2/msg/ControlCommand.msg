# ROS time: publication on hardware, control-cycle time in SITL.
# Rates are body FLU, not Betaflight hardware FRD.
std_msgs/Header header
# Host boot ID for hardware/real-time tests; boot ID + ':ros' for SITL /clock
# evidence. Consumers reject commands from a different safety clock domain.
string clock_id
uint64 sequence
float64 total_thrust
geometry_msgs/Vector3 body_rates
# total_thrust is total vehicle force in N; body_rates are rad/s.
bool permit_override
uint8 mode
agi_ros2/SafetyEvidence evidence
# Observability only; these fields never refresh safety evidence.
# All steady times use the clock_id host's CLOCK_MONOTONIC, also in SITL.
float64 control_session_start
builtin_interfaces/Time state_stamp
float64 state_published_steady_time
float64 state_received_steady_time
float64 control_start_steady_time
float64 published_steady_time
