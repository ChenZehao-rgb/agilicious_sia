#include <gtest/gtest.h>

#include <filesystem>

#include "agilib/math/gravity.hpp"
#include "agilib/pilot/hardware_pilot.hpp"
#include "agilib/sampler/time_based/time_sampler.hpp"

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

	ControlDecision step(bool automatic, bool armed = true, bool healthy = true, bool shadow = false,
	                     const Vector<3>* target = nullptr) {
		_now += .01;
		_state.t = _now;
		Evidence evidence;
		evidence.now = evidence.imu_sample_time = evidence.status_time = evidence.imu_time = evidence.rtk_time = evidence.rc_time =
		        _now;
		evidence.rtk_fixed = evidence.heading_valid = evidence.accuracy_ok = evidence.imu_calibrated = true;
		evidence.synchronized = evidence.converged = evidence.config_verified = evidence.thrust_calibrated = true;
		evidence.thrust_mapping_ready = true;
		evidence.geofence_ok = evidence.msp_healthy = evidence.rc_link = true;
		evidence.armed = armed;
		evidence.kill = false;
		evidence.auto_switch = automatic;
		if (!healthy) evidence.imu_time -= .020;
		return _runtime->tick(_state, evidence, shadow, healthy, true, target);
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

TEST(CapturedOnlineReference, AbsoluteEndpointsRelativeClockAndPredictionDoNotChangeExecution) {
	const Vector<3> target(5, -1, 2);
	for (double yaw : {0.0, M_PI / 2, -2.0}) {
		QuadState state;
		state.setZero();
		state.t = 1800000000.0;
		state.p = Vector<3>(2, 3, 4);
		state.q(yaw);
		auto reference = std::make_shared<CapturedReference>();
		reference->capture(state, state.t, false);
		ASSERT_TRUE(reference->goToPosition(target, state.t, 1.0));
		const auto start = reference->getSetpoint(state, state.t);
		EXPECT_LT((start.state.p - state.p).norm(), 1e-8);
		EXPECT_LT(start.state.v.norm(), 1e-8);
		EXPECT_LT(start.state.a.norm(), 1e-8);
		EXPECT_LT(start.state.j.norm(), 1e-8);
		EXPECT_NEAR(start.state.getYaw(), yaw, 1e-8);
		const double duration = (target - state.p).norm();
		const auto middle = reference->getSetpoint(state, state.t + duration / 2);
		EXPECT_LT((middle.state.p - (target + state.p) / 2).norm(), 1e-6);
		EXPECT_GT(middle.state.v.norm(), .1);
		const auto dynamic = reference->getSetpoint(state, state.t + duration / 4);
		EXPECT_NEAR(dynamic.input.collective_thrust, (dynamic.state.a - GVEC).norm(), 1e-8);
		EXPECT_TRUE(dynamic.input.omega.isApprox(dynamic.state.w));
		EXPECT_GT(dynamic.input.omega.norm(), 1e-4);
		EXPECT_NEAR(dynamic.state.q().norm(), 1, 1e-8);
		state.t += duration - .2;
		TimeSampler sampler(20, .1);
		SetpointVector sampled;
		ASSERT_TRUE(sampler.getAt(state, {reference}, &sampled));
		EXPECT_TRUE(reference->movingAt(state.t));
		EXPECT_GT(sampled.front().state.v.norm(), 0);
		for (size_t i = 0; i < sampled.size(); ++i) {
			EXPECT_NEAR(sampled[i].state.t, state.t + .1 * i, 2e-6);
			EXPECT_EQ(sampled[i].input.t, sampled[i].state.t);
		}
		EXPECT_TRUE(sampled.back().state.p.isApprox(target));
		EXPECT_TRUE(sampled.back().state.v.isZero());
		EXPECT_TRUE(sampled.back().state.a.isZero());
		EXPECT_TRUE(sampled.back().state.j.isZero());
		EXPECT_TRUE(sampled.back().input.omega.isZero());
		EXPECT_DOUBLE_EQ(sampled.back().input.collective_thrust, G);
		for (double offset : {1.0, 1000.0}) {
			const auto end = reference->getSetpoint(state, state.t + offset);
			EXPECT_TRUE(end.state.p.isApprox(target));
			EXPECT_NEAR(end.state.getYaw(), yaw, 1e-8);
			EXPECT_EQ(end.state.t, state.t + offset);
		}
		EXPECT_TRUE(reference->movingAt(state.t));
		EXPECT_FALSE(reference->movingAt(state.t + 1));
	}
}

TEST_F(HardwareGeo, OnlineTargetsRequireActualHoverAndCanRepeatWithoutAutoEdge) {
	_now = 1800000000.0;
	const Vector<3> target(3, 3, 4), second(3, 4, 4);
	EXPECT_NE(step(false, true, true, false, &target).goal_event.find("rejected"), std::string::npos);
	for (int i = 0; i < 80; ++i) step(false);
	ASSERT_TRUE(step(true).permit_override);
	_state.v.x() = .21;
	EXPECT_NE(step(true, true, true, false, &target).goal_event.find("speed"), std::string::npos);
	_state.v.setZero();
	_state.p.x() += .21;
	EXPECT_NE(step(true, true, true, false, &target).goal_event.find("position error"), std::string::npos);
	_state.p.x() -= .11;  // A small tracking error must not shift the starting reference.
	auto decision = step(true, true, true, false, &target);
	ASSERT_NE(decision.goal_event.find("accepted"), std::string::npos);
	EXPECT_LT((decision.reference.p - Vector<3>(2, 3, 4)).norm(), 1e-8);
	EXPECT_EQ(_runtime->warmCycles(), 50u);
	EXPECT_NE(step(true, true, true, false, &second).goal_event.find("trajectory moving"), std::string::npos);
	Vector<3> invalid(NAN, 0, 0);
	EXPECT_NE(step(true, true, true, false, &invalid).goal_event.find("nonfinite"), std::string::npos);
	for (int i = 0; i < 170; ++i) decision = step(true);
	EXPECT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.trajectory_active);
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	EXPECT_TRUE(decision.reference.v.isZero());
	EXPECT_NE(decision.reason.find("settling"), std::string::npos);
	EXPECT_NE(step(true, true, true, false, &second).goal_event.find("position error"), std::string::npos);
	_state.p = target;
	decision = step(true);
	EXPECT_TRUE(decision.goal_rejection.empty());
	EXPECT_NE(decision.reason.find("stable hover"), std::string::npos);
	const double auto_elapsed = decision.reference_elapsed;
	decision = step(true, true, true, false, &second);
	ASSERT_NE(decision.goal_event.find("accepted"), std::string::npos);
	EXPECT_GT(decision.reference_elapsed, auto_elapsed);
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	for (int i = 0; i < 170; ++i) decision = step(true);
	EXPECT_TRUE(decision.reference.p.isApprox(second));
	EXPECT_TRUE(decision.permit_override);
	EXPECT_EQ(_runtime->warmCycles(), 50u);
	step(false);
	decision = step(true);
	EXPECT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.reference.p.isApprox(_state.p));
	EXPECT_EQ(decision.reason.find("online target"), std::string::npos);
}

TEST_F(HardwareGeo, OnlineTargetShadowAndFaultResetPreserveConfiguredCsv) {
	SetpointVector points(2);
	for (int i = 0; i < 2; ++i) {
		points[i].state.setZero();
		points[i].state.t = .1 * i;
		points[i].state.p.x() = i;
		points[i].input = Command(.1 * i, G, Vector<3>::Zero());
	}
	ASSERT_TRUE(_runtime->setTrajectory(points));
	for (int i = 0; i < 80; ++i) step(false, true, true, true);
	ASSERT_TRUE(step(true, true, true, true).trajectory_active);
	for (int i = 0; i < 20; ++i) step(true, true, true, true);
	_state.p.x() += 1;
	const Vector<3> target(3, 4, 4);
	auto decision = step(true, true, true, true, &target);
	ASSERT_NE(decision.goal_event.find("accepted"), std::string::npos);
	for (int i = 0; i < 170; ++i) {
		decision = step(true, true, true, true);
		ASSERT_FALSE(decision.permit_override);
		ASSERT_TRUE(decision.trajectory_active);
	}
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	_runtime->reportOutputFault();  // Also used by state/session/coordinate resets in control_node.
	for (int i = 0; i < 80; ++i) EXPECT_FALSE(step(true, true, true, true).trajectory_active);
	step(false, true, true, true);
	decision = step(true, true, true, true, &target);
	EXPECT_NE(decision.goal_event.find("rejected"), std::string::npos);
	EXPECT_TRUE(decision.reference.p.isApprox(_state.p));
	for (int i = 0; i < 20; ++i) decision = step(true, true, true, true);
	EXPECT_TRUE(decision.reference.p.isApprox(_state.p + Vector<3>(1, 0, 0)));
	EXPECT_EQ(decision.reason.find("online target"), std::string::npos);
}

}  // namespace
}  // namespace agi::hardware
