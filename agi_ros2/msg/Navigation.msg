# Atomic MAVLink GPS solution, after timestamp pairing. No PPS claim.
# Transport/clock/fix failure emits a revocation at local detection time.
# Missing velocity/heading components remain NaN/heading_valid=false without
# revoking valid position. device_time_usec is the FC new-solution capture epoch,
# not the receiver PVT epoch or a proven physical sensor sampling time.
std_msgs/Header header
string source_session
uint64 device_time_usec
uint8 fix_type
float64 latitude
float64 longitude
float64 altitude
# unknown, ellipsoid, msl
string altitude_reference
geometry_msgs/Vector3 velocity
float64 heading
bool heading_valid
float64 horizontal_accuracy
float64 vertical_accuracy
float64 velocity_accuracy
bool clock_aligned
