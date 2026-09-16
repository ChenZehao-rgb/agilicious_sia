# Flight-controller estimated heading, radians CCW from East, in [-pi, pi].
# header.frame_id is gps_enu; stamp is the FC solution/snapshot update time.
# valid means the MAVLink hdg field is present and in range, not true-north
# accuracy, RTK heading availability, or estimator convergence.
std_msgs/Header header
float64 heading
bool valid
