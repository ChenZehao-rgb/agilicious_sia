# ROS EKF: stationary OriginBuilder reference; existing heading fields retain their meaning.
# FC: receiver source_session, estimator XY origin (degrees), independent GPS Z
# offset (MSL metres), frame odom. Heading correction is not applied to FC goals.
std_msgs/Header header
string session_id
float64 latitude
float64 longitude
float64 altitude
string altitude_reference
string heading_source
float64 heading_correction_rad
bool fc_declination_applied

# Validity does not depend on nonzero coordinates. Legacy publishers set these
# but leave FC identity/scale fields at defaults; they cannot serve FC goals.
bool xy_valid
bool z_valid
# FC native GPS_cosLat, dimensionless (not recomputed from latitude).
float64 longitude_scale
# Complete FC reset identity, NOT ROS adapter generation or odometry low byte.
uint32 fc_reset_counter
# Completed FC estimator snapshot boot time in microseconds; not receipt time.
uint64 fc_time_usec
