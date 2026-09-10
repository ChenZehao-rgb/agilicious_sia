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
