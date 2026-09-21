#include <gtest/gtest.h>

#include "agi_ros2/imu_initialization.h"

namespace agi_ros2 {
namespace {

TEST(ImuInitialization, RecoversStationaryTiltAndGyroBiasAfterFullWindow) {
	ImuInitialization initializer(ImuInitialization::Params{});
	const agi::Vector<3> acceleration(0, agi::G * 0.6, agi::G * 0.8);
	const agi::Vector<3> bias(0.01, -0.02, 0.03);
	for (int i = 0; i < 1500; ++i) EXPECT_FALSE(initializer.addSample({1 + i * 0.002, acceleration, bias}));
	EXPECT_TRUE(initializer.addSample({4, acceleration, bias}));
	EXPECT_TRUE(initializer.gyroBias().isApprox(bias, 1e-10));
	EXPECT_TRUE(initializer.acceleration().isApprox(acceleration, 1e-10));
}

TEST(ImuInitialization, MotionAndInvalidGravityCannotInitialize) {
	ImuInitialization initializer(ImuInitialization::Params{});
	for (int i = 0; i <= 1600; ++i) {
		EXPECT_FALSE(initializer.addSample({1 + i * 0.002, agi::Vector<3>(0, 0, agi::G), agi::Vector<3>(0.2, 0, 0)}));
	}
	initializer.reset();
	for (int i = 0; i <= 1600; ++i) {
		EXPECT_FALSE(initializer.addSample({1 + i * 0.002, agi::Vector<3>(0, 0, 8.0), agi::Vector<3>::Zero()}));
	}
	initializer.reset();
	for (int i = 0; i <= 1600; ++i) {
		const double vibration = i % 2 ? 0.04 : -0.04;
		EXPECT_FALSE(initializer.addSample({1 + i * 0.002, agi::Vector<3>(0, 0, agi::G), agi::Vector<3>(vibration, 0, 0)}));
	}
}

TEST(ImuInitialization, GapRewindAndExplicitResetRequireNewWindow) {
	ImuInitialization initializer(ImuInitialization::Params{});
	for (int i = 0; i <= 1500; ++i) initializer.addSample({1 + i * 0.002, agi::Vector<3>(0, 0, agi::G), agi::Vector<3>::Zero()});
	ASSERT_TRUE(initializer.ready());
	EXPECT_FALSE(initializer.addSample({4.1, agi::Vector<3>(0, 0, agi::G), agi::Vector<3>::Zero()}));
	EXPECT_FALSE(initializer.addSample({1.0, agi::Vector<3>(0, 0, agi::G), agi::Vector<3>::Zero()}));
	initializer.reset();
	EXPECT_FALSE(initializer.ready());
}

}  // namespace
}  // namespace agi_ros2
