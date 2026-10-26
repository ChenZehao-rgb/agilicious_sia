#ifndef AGI_ROS2_FC_PROTOCOL_H_
#define AGI_ROS2_FC_PROTOCOL_H_
#include <common/mavlink.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace agi_ros2::fc {
struct Health {
	uint16_t flags{};
	uint32_t sequence{}, reset{}, xy_age{}, z_age{}, sources{};
	uint64_t time{};
	uint8_t fix{};
	float trust_xy{}, trust_z{};
	std::array<float, 3> position{}, velocity{};
};
inline uint64_t littleEndian(const uint8_t* p, unsigned count) {
	uint64_t result = 0;
	for (unsigned i = 0; i < count; ++i) result |= uint64_t(p[i]) << (8 * i);
	return result;
}
inline float readFloat(const uint8_t* p) {
	const uint32_t bits = littleEndian(p, 4);
	float value;
	std::memcpy(&value, &bits, sizeof(value));
	return value;
}
inline bool decodeHealth(const mavlink_message_t& message, Health& h) {
	if (message.magic != MAVLINK_STX || message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) return false;
	mavlink_v2_extension_t v{};
	mavlink_msg_v2_extension_decode(&message, &v);
	const auto* p = v.payload;
	if (v.message_type != 0xBF01 || v.target_network || v.target_system || v.target_component || p[0] != 1 || p[1] != 68) return false;
	h.flags = littleEndian(p + 2, 2);
	h.sequence = littleEndian(p + 4, 4);
	h.reset = littleEndian(p + 8, 4);
	h.time = littleEndian(p + 12, 8);
	h.xy_age = littleEndian(p + 20, 4);
	h.z_age = littleEndian(p + 24, 4);
	h.trust_xy = readFloat(p + 28);
	h.trust_z = readFloat(p + 32);
	h.sources = littleEndian(p + 36, 4);
	h.fix = p[40];
	if ((h.flags & ~255u) || (h.sources & ~31u) || p[41] || p[42] || p[43] || (h.fix != 0 && h.fix != 1 && h.fix != 3)) return false;
	for (unsigned i = 68; i < sizeof(v.payload); ++i)
		if (p[i]) return false;
	for (int i = 0; i < 3; ++i) {
		h.position[i] = readFloat(p + 44 + 4 * i);
		h.velocity[i] = readFloat(p + 56 + 4 * i);
		// NaN is a legitimate unknown, but infinity/negative variance is malformed.
		for (float x : {h.position[i], h.velocity[i]})
			if (std::isinf(x) || x < 0) return false;
	}
	for (float x : {h.trust_xy, h.trust_z})
		if (std::isinf(x) || x < 0 || x > 1 || ((h.flags & 128) && !std::isfinite(x))) return false;
	return true;
}
struct State {
	std::array<double, 3> position{}, velocity{}, rates{};
	std::array<double, 4> orientation{};  // w,x,y,z; FLU -> ENU
};
inline bool decodeOdometry(const mavlink_message_t& message, mavlink_odometry_t& v, State& out) {
	if (message.magic != MAVLINK_STX || message.msgid != MAVLINK_MSG_ID_ODOMETRY) return false;
	mavlink_msg_odometry_decode(&message, &v);
	if (v.frame_id != MAV_FRAME_LOCAL_NED || v.child_frame_id != MAV_FRAME_BODY_FRD ||
	    v.estimator_type != MAV_ESTIMATOR_TYPE_AUTOPILOT || v.quality < 0 || v.quality > 100 || !v.time_usec)
		return false;
	for (float x : {v.x, v.y, v.z, v.vx, v.vy, v.vz, v.rollspeed, v.pitchspeed, v.yawspeed})
		if (!std::isfinite(x)) return false;
	for (const float* covariance : {v.pose_covariance, v.velocity_covariance}) {
		if (std::isnan(covariance[0])) continue;  // MAVLink whole-matrix unknown sentinel.
		for (int i = 0; i < 21; ++i)
			if (!std::isfinite(covariance[i])) return false;
		for (int i : {0, 6, 11, 15, 18, 20})
			if (covariance[i] < 0) return false;
	}
	double norm = 0;
	for (float x : v.q) {
		if (!std::isfinite(x)) return false;
		norm += x * x;
	}
	if (std::abs(norm - 1) > 1e-3) return false;
	// Exactly the established ATTITUDE conversion, without a second estimator.
	const double k = std::sqrt(.5 / norm), w = v.q[0], x = v.q[1], y = v.q[2], z = v.q[3];
	out.orientation = {k * (w + z), k * (x + y), k * (x - y), k * (w - z)};
	out.position = {v.y, v.x, -v.z};
	out.rates = {v.rollspeed, -v.pitchspeed, -v.yawspeed};
	const auto& q = out.orientation;
	const std::array<double, 3> body{v.vx, -v.vy, -v.vz};
	const std::array<double, 3> cross{2 * (q[2] * body[2] - q[3] * body[1]), 2 * (q[3] * body[0] - q[1] * body[2]),
	                                  2 * (q[1] * body[1] - q[2] * body[0])};
	out.velocity = {body[0] + q[0] * cross[0] + q[2] * cross[2] - q[3] * cross[1],
	                body[1] + q[0] * cross[1] + q[3] * cross[0] - q[1] * cross[2],
	                body[2] + q[0] * cross[2] + q[1] * cross[1] - q[2] * cross[0]};
	return true;
}
}  // namespace agi_ros2::fc
#endif
