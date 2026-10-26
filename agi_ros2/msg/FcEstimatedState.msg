# Converted once from MAVLink2 LOCAL_NED/BODY_FRD to ENU/FLU.
std_msgs/Header header
string clock_id
string source_session
uint64 validity_epoch
uint64 fc_time_usec
float64 sample_steady_time
float64 receive_steady_time
float64 published_steady_time
bool clock_aligned
bool valid
string reason
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 body_rates
# Original MAVLink upper triangles; NaN means unknown, never zero-filled.
float32[21] pose_covariance
float32[21] velocity_covariance
uint8 reset_counter
uint8 estimator_type
int8 quality
