# Publication ROS time. Rates are body FLU, not Betaflight hardware FRD.
std_msgs/Header header
string clock_id
uint64 sequence
float64 total_thrust
geometry_msgs/Vector3 body_rates
# total_thrust is total vehicle force in N; body_rates are rad/s.
bool permit_override
uint8 mode
agi_ros2/SafetyEvidence evidence
