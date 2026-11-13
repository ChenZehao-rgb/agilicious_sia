# MAVLink GLOBAL_POSITION_INT (33), original units and fields; no GPS fix flag.
# header.stamp maps FC time_boot_ms to ROS time, not a GPS PVT sampling epoch.
std_msgs/Header header
string source_session
uint32 time_boot_ms
int32 lat
int32 lon
# Millimetres: MSL altitude and FC-reported relative altitude respectively.
int32 alt
int32 relative_alt
# NED velocity, centimetres/second.
int16 vx
int16 vy
int16 vz
# Centidegrees clockwise from north; 65535 means unknown.
uint16 hdg
