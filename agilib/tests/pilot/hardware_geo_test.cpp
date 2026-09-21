#include <gtest/gtest.h>

#include <filesystem>

#include "agilib/math/gravity.hpp"
#include "agilib/pilot/hardware_pilot.hpp"

namespace agi::hardware {
namespace {

class HardwareGeo : public ::testing::Test {
protected:
	void SetUp() override {
		const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
		const Yaml profile(root / "agi_ros2/config/simulation.yaml");
		PilotParams params;
		params.directory_ = root / "agi_ros2/config";
		ASSERT_TRUE(params.load(profile["pilot"], "GEO"));
		ASSERT_TRUE(params.quad_.validRatesThrust());
		ASSERT_FALSE(params.quad_.valid());
		_runtime = std::make_unique<HardwarePilot>(params, [this] { return _now; });
		_state.setZero();
		_state.p = Vector<3>(2, 3, 4);
	}

	ControlDecision step(bool automatic, bool armed = true, bool healthy = true, bool shadow = false) {
		_now += .01;
		_state.t = _now;
		Evidence evidence;
		evidence.now = evidence.imu_time = evidence.rtk_time = evidence.rc_time = _now;
		evidence.rtk_fixed = evidence.heading_valid = evidence.accuracy_ok = evidence.imu_calibrated = true;
		evidence.synchronized = evidence.converged = evidence.config_verified = evidence.thrust_calibrated = true;
		evidence.geofence_ok = evidence.msp_healthy = evidence.rc_link = true;
		evidence.armed = armed;
		evidence.kill = false;
		evidence.auto_switch = automatic;
		if (!healthy) evidence.imu_time -= .020;
		return _runtime->tick(_state, evidence, shadow, healthy);
	}

	std::unique_ptr<HardwarePilot> _runtime;
	QuadState _state;
	double _now = 10;
};

TEST_F(HardwareGeo, CapturesPositionAndYawOnlyAfterHealthyArmedAutoEdge) {
	for (int i = 0; i < 80; ++i) EXPECT_FALSE(step(true).permit_override);
	ASSERT_EQ(_runtime->warmCycles(), 50u);
	step(false, false);
	EXPECT_FALSE(step(true, false).permit_override);
	EXPECT_FALSE(step(true).permit_override);
	_state.p = Vector<3>(6, -2, 3);
	_state.q(Quaternion(Eigen::AngleAxisd(.7, Vector<3>::UnitZ())));
	step(false);
	const auto decision = step(true);
	ASSERT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.reference.p.isApprox(_state.p));
	EXPECT_NEAR(decision.reference.getYaw(), .7, 1e-8);
	EXPECT_NEAR(decision.command.collective_thrust, G, 1e-6);
	EXPECT_LT(decision.command.omega.norm(), 1e-8);
	_state.p.x() += .1;
	EXPECT_GT(step(true).command.omega.norm(), .01);
}

TEST_F(HardwareGeo, FaultAndOutputRestartRequireFreshLowHighAfterWarmup) {
	for (int i = 0; i < 80; ++i) step(false);
	ASSERT_TRUE(step(true).permit_override);
	auto failed = step(true, true, false);
	EXPECT_FALSE(failed.permit_override);
	EXPECT_FALSE(failed.trajectory_active);
	for (int i = 0; i < 80; ++i) EXPECT_FALSE(step(true).permit_override);
	step(false);
	ASSERT_TRUE(step(true).permit_override);
	_runtime->reportOutputFault();
	for (int i = 0; i < 80; ++i) EXPECT_FALSE(step(true).permit_override);
	_state.p.x() += 2;
	step(false);
	const auto recovered = step(true);
	ASSERT_TRUE(recovered.permit_override);
	EXPECT_TRUE(recovered.reference.p.isApprox(_state.p));
}

TEST_F(HardwareGeo, ShadowComputesWithoutOutputAuthorization) {
	for (int i = 0; i < 80; ++i) EXPECT_FALSE(step(false, true, true, true).permit_override);
	const auto decision = step(true, true, true, true);
	EXPECT_TRUE(decision.evidence.command_valid);
	EXPECT_TRUE(decision.trajectory_active);
	EXPECT_FALSE(decision.permit_override);
}

TEST_F(HardwareGeo, ExplicitTrajectoryUsesCapturedFrameAndEndsInHover) {
	SetpointVector points(2);
	for (int i = 0; i < 2; ++i) {
		points[i].state.setZero();
		points[i].state.t = i;
		points[i].state.p.x() = i;
		points[i].state.v.x() = 1;
		points[i].input = Command(i, G, Vector<3>::Zero());
	}
	ASSERT_TRUE(_runtime->setTrajectory(points));
	_state.q(Quaternion(Eigen::AngleAxisd(M_PI / 2, Vector<3>::UnitZ())));
	for (int i = 0; i < 80; ++i) step(false);
	ASSERT_TRUE(step(true).permit_override);
	ControlDecision decision;
	for (int i = 0; i < 50; ++i) decision = step(true);
	ASSERT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.reference.p.isApprox(Vector<3>(2, 3.5, 4), 1e-6));
	EXPECT_TRUE(decision.reference.v.isApprox(Vector<3>(0, 1, 0), 1e-6));
	for (int i = 0; i < 60; ++i) decision = step(true);
	ASSERT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.reference.p.isApprox(Vector<3>(2, 4, 4), 1e-6));
	EXPECT_TRUE(decision.reference.v.isZero());
}

}  // namespace
}  // namespace agi::hardware
