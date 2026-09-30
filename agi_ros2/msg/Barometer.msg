# A valid sample has the pressure acquisition timestamp, mapped from the FC clock.
# An invalid event has the detection timestamp and must not be fused as a sample.
std_msgs/Header header
string source_session
bool clock_aligned
bool valid
# Temperature-compensated absolute pressure in Pa; NaN on invalid events.
float64 pressure_pa
# Pressure variance in Pa^2; zero means unknown, never zero measurement noise.
float64 pressure_variance
string reason
