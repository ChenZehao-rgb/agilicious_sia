#include "agi_ros2/fc_geographic.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>

#include "agi_ros2/fc_protocol.h"

namespace {
using agi_ros2::msg::FusedState;
using agi_ros2::msg::GeographicGoal;
using agi_ros2::msg::NavigationOrigin;

TEST(FcGeographic, NativeProjectionAndIdentity) {
	NavigationOrigin origin;
	origin.header.frame_id = "odom";
	origin.session_id = "receiver";
	origin.altitude_reference = "msl";
	origin.xy_valid = origin.z_valid = true;
	origin.longitude_scale = .75;  // Intentionally not cos(latitude).
	origin.fc_reset_counter = 0x12345678;
	origin.fc_time_usec = (uint64_t(1) << 32) + 100;
	FusedState state;
	state.header.frame_id = "odom";
	state.state_source = "fc_fused";
	state.fc_state_valid = true;
	state.navigation_session = origin.session_id;
	state.fc_reset_counter = origin.fc_reset_counter;
	state.reset_counter = 42;  // ROS generation must not be used as FC reset.
	state.fc_time_usec = origin.fc_time_usec + 1;
	GeographicGoal goal;
	std::array<double, 3> target;
	ASSERT_EQ(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
	EXPECT_EQ(target, (std::array<double, 3>{0, 0, 0}));
	origin.altitude = 123;
	goal.latitude = .001;
	goal.longitude = -.002;
	goal.altitude_msl = 127;
	ASSERT_EQ(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
	// Independent numeric anchors: float EARTH_ANGLE_TO_CM = 1.1131949424743652.
	EXPECT_NEAR(target[0], -166.97924137115479, 1e-10);
	EXPECT_NEAR(target[1], 111.31949424743652, 1e-10);
	EXPECT_DOUBLE_EQ(target[2], 4);
	for (auto change : {0, 1, 2, 3, 4, 5, 6}) {
		auto bad = origin;
		if (change == 0) bad.session_id = "previous";
		if (change == 1) bad.fc_reset_counter += 256;  // Same low byte is insufficient.
		if (change == 2) bad.fc_time_usec = state.fc_time_usec + 1;
		if (change == 3) bad.xy_valid = false;
		if (change == 4) bad.z_valid = false;
		if (change == 5) bad.altitude_reference = "ellipsoid";
		if (change == 6) bad.longitude_scale = NAN;
		EXPECT_NE(nullptr, agi_ros2::fc::geographicTarget(bad, state, goal, target));
	}
	origin.longitude = 179.999;
	goal.longitude = -179.999;
	ASSERT_EQ(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
	EXPECT_NEAR(target[0], 166.97924137115479, 1e-8);
	origin.longitude = -179.999;
	goal.longitude = 179.999;
	ASSERT_EQ(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
	EXPECT_NEAR(target[0], -166.97924137115479, 1e-8);
	goal.latitude = 91;
	EXPECT_NE(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
	goal.latitude = 0;
	goal.altitude_msl = INFINITY;
	EXPECT_NE(nullptr, agi_ros2::fc::geographicTarget(origin, state, goal, target));
}

TEST(FcGeographic, OriginWireCrcTailAndMalformedPayload) {
	mavlink_v2_extension_t v{};
	v.message_type = 0xBF02;
	// v1/32 bytes, XY+Z, full reset=0xfedcba98, time=0x123456789,
	// latitude=-123456789, longitude=1800000000, scale=.75, MSL offset=0.
	const uint8_t payload[] = {1,    32,   3,    0,    0x98, 0xba, 0xdc, 0xfe, 0x89, 0x67, 0x45, 0x23, 1, 0, 0, 0,
	                           0xeb, 0x32, 0xa4, 0xf8, 0,    0xd2, 0x49, 0x6b, 0,    0,    0x40, 0x3f, 0, 0, 0, 0};
	std::copy(std::begin(payload), std::end(payload), v.payload);
	mavlink_message_t msg{}, decoded{}, parser{};
	mavlink_status_t status{}, result{};
	mavlink_msg_v2_extension_encode(1, 1, &msg, &v);
	EXPECT_LT(msg.len, 37);
	uint8_t wire[MAVLINK_MAX_PACKET_LEN];
	const auto size = mavlink_msg_to_send_buffer(wire, &msg);
	unsigned count = 0;
	for (unsigned i = 0; i < size; ++i)
		count += mavlink_frame_char_buffer(&parser, &status, wire[i], &decoded, &result) == MAVLINK_FRAMING_OK;
	ASSERT_EQ(1u, count);
	agi_ros2::fc::Origin origin;
	ASSERT_TRUE(agi_ros2::fc::decodeOrigin(decoded, origin));
	EXPECT_EQ(origin.reset, 0xfedcba98u);
	EXPECT_EQ(origin.time, 0x123456789ULL);
	EXPECT_EQ(origin.latitude, -123456789);
	EXPECT_EQ(origin.longitude, 1800000000);
	EXPECT_FLOAT_EQ(origin.longitude_scale, .75f);
	EXPECT_FLOAT_EQ(origin.altitude_offset_cm, 0);
	agi_ros2::fc::Health health;
	EXPECT_FALSE(agi_ros2::fc::decodeHealth(decoded, health));
	for (int field = 0; field < 8; ++field) {
		auto bad = v;
		if (field == 0) bad.message_type = 0xBF01;
		if (field == 1) bad.message_type = 0xBF03;
		if (field == 2) bad.payload[0] = 2;
		if (field == 3) bad.payload[1] = 31;
		if (field == 4) bad.payload[2] = 4;
		if (field == 5) bad.payload[32] = 1;
		if (field == 6) {
			bad.payload[26] = 0x80;
			bad.payload[27] = 0x7f;
		}  // Infinity.
		if (field == 7) {
			bad.payload[30] = 0xc0;
			bad.payload[31] = 0x7f;
		}  // NaN.
		mavlink_msg_v2_extension_encode(1, 1, &msg, &bad);
		EXPECT_FALSE(agi_ros2::fc::decodeOrigin(msg, origin));
	}
}
}  // namespace
