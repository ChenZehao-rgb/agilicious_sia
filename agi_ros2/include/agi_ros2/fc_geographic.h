#ifndef AGI_ROS2_FC_GEOGRAPHIC_H_
#define AGI_ROS2_FC_GEOGRAPHIC_H_

#include <array>
#include <cmath>

#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/geographic_goal.hpp"
#include "agi_ros2/msg/navigation_origin.hpp"

namespace agi_ros2::fc {
// Match io/gps.h's float EARTH_ANGLE_TO_CM, then convert degE7/cm to degrees/m.
constexpr double kMetersPerDegree = double(111.3195f * 1000 * 100 / 10000000L) * 10000000L * .01;

inline const char* geographicTarget(const msg::NavigationOrigin& origin, const msg::FusedState& state, const msg::GeographicGoal& goal,
                                    std::array<double, 3>& target) {
	if (state.state_source != "fc_fused" || !state.fc_state_valid) return "current FC fused state unavailable";
	if (origin.session_id.empty() || origin.session_id != state.navigation_session ||
	    origin.fc_reset_counter != state.fc_reset_counter || !origin.fc_time_usec || origin.fc_time_usec > state.fc_time_usec)
		return "FC origin session/reset/time mismatch";
	if (origin.header.frame_id != "odom" || state.header.frame_id != "odom" || origin.altitude_reference != "msl")
		return "FC origin must use odom/MSL";
	if (!origin.xy_valid || !origin.z_valid) return "FC GPS XY/MSL Z baseline unavailable";
	if (!std::isfinite(origin.latitude) || std::abs(origin.latitude) > 90 || !std::isfinite(origin.longitude) ||
	    std::abs(origin.longitude) > 180 || !std::isfinite(origin.altitude) || !std::isfinite(origin.longitude_scale) ||
	    origin.longitude_scale < 0 || origin.longitude_scale > 1)
		return "invalid FC origin mapping";
	if (!std::isfinite(goal.latitude) || std::abs(goal.latitude) > 90 || !std::isfinite(goal.longitude) ||
	    std::abs(goal.longitude) > 180 || !std::isfinite(goal.altitude_msl))
		return "invalid WGS84/MSL target";
	double longitude_delta = goal.longitude - origin.longitude;
	if (longitude_delta > 180) longitude_delta -= 360;
	if (longitude_delta < -180) longitude_delta += 360;
	target = {longitude_delta * kMetersPerDegree * origin.longitude_scale, (goal.latitude - origin.latitude) * kMetersPerDegree,
	          goal.altitude_msl - origin.altitude};
	for (double value : target)
		if (!std::isfinite(value)) return "nonfinite ENU target";
	return nullptr;
}
}  // namespace agi_ros2::fc
#endif  // AGI_ROS2_FC_GEOGRAPHIC_H_
