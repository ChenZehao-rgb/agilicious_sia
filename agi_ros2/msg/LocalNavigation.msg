std_msgs/Header header
# False is a revocation event, not a measurement. Its stamp is detection time.
bool observation_valid
string reason
string session_id
string source_session
string altitude_reference
uint8 fix_type
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 heading
bool heading_valid
float64[3] position_variance
float64[3] velocity_variance
float64 heading_variance
bool accuracy_known
bool clock_aligned
# Receiver-reported accuracies, distinct from conservative covariance floors.
float64 horizontal_accuracy
float64 vertical_accuracy
float64 velocity_accuracy
bool accuracy_ok
