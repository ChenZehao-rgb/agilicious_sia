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

	ControlDecision step(bool automatic, bool armed = true, bool healthy = true, bool shadow = false, const Vector<3>* target = nullptr,
	                     bool pause = false) {
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
		return _runtime->tick(_state, evidence, shadow, healthy, true, target, pause);
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

TEST(CapturedOnlineReference, CruiseProfilesRespectBoundariesLimitsAndQueryOrder) {
	struct Case {
		double distance;
		double velocity;
		double acceleration_time;
		double cruise_time;
	};
	for (const auto& test : {Case{20, 2, 3.75, 6.25}, Case{5, 1.632993161855452, 3.061862178478973, 0}, Case{7.5, 2, 3.75, 0},
	                         Case{.01, .0133333333333333, .75, 0}, Case{0, 0, .75, 0}}) {
		SCOPED_TRACE(test.distance);
		QuadState state;
		state.setZero();
		state.t = 100;
		state.p = Vector<3>(0, 0, 1);
		const Vector<3> target(test.distance, 0, 1);
		auto reference = std::make_shared<CapturedReference>();
		reference->capture(state, state.t, false);
		ASSERT_TRUE(reference->goToPosition(target, state.t, 2, 1, .5));
		ASSERT_TRUE(reference->hasOnlineTarget());
		EXPECT_FALSE(reference->isHover());
		EXPECT_TRUE(reference->onlineEndpoint().isApprox(target));
		const double ta = test.acceleration_time, tc = test.cruise_time;
		const double duration = 2 * ta + tc;
		EXPECT_TRUE(reference->movingAt(state.t + duration - 1e-6));
		EXPECT_FALSE(reference->movingAt(state.t + duration + 1e-6));
		for (int i = 0; i <= 1000; ++i) {
			const double time = duration * i / 1000;
			const auto point = reference->getSetpoint(state, state.t + time);
			ASSERT_TRUE(point.state.valid());
			ASSERT_TRUE(point.input.valid());
			EXPECT_NEAR(point.state.q().norm(), 1, 1e-8);
			EXPECT_NEAR(point.input.collective_thrust, (point.state.a - GVEC).norm(), 1e-8);
			EXPECT_LT((point.input.omega - point.state.w).norm(), 1e-8);
			EXPECT_LE(point.state.v.norm(), 2 + 1e-7);
			EXPECT_LE(point.state.a.norm(), 1 + 1e-7);
			EXPECT_NEAR(point.state.p.y(), 0, 1e-8);
			EXPECT_NEAR(point.state.p.z(), 1, 1e-8);
			const double u = std::clamp(std::min(time, duration - time) / ta, 0.0, 1.0);
			const double expected_speed = test.velocity * u * u * u * (10 - 15 * u + 6 * u * u);
			EXPECT_NEAR(point.state.v.x(), expected_speed, 1e-7);
			if (time > ta && time < ta + tc) {
				EXPECT_NEAR(point.state.p.x(), .5 * test.velocity * ta + test.velocity * (time - ta), 1e-7);
				EXPECT_LT(point.state.a.norm(), 1e-7);
				EXPECT_LT(point.state.j.norm(), 1e-7);
			}
		}
		for (double time : {0.0, ta, ta + tc, duration}) {
			const auto at = reference->getSetpoint(state, state.t + time);
			const auto left = reference->getSetpoint(state, state.t + time - 1e-6);
			const auto right = reference->getSetpoint(state, state.t + time + 1e-6);
			EXPECT_LT((left.state.p - right.state.p).norm(), 1e-5);
			EXPECT_LT((left.state.v - right.state.v).norm(), 1e-5);
			EXPECT_LT((left.state.a - right.state.a).norm(), 1e-5);
			EXPECT_LT((left.state.j - right.state.j).norm(), 1e-5);
			EXPECT_LT(left.state.q().angularDistance(right.state.q()), 1e-5);
			EXPECT_NEAR(left.input.collective_thrust, right.input.collective_thrust, 1e-5);
			EXPECT_LT((left.input.omega - right.input.omega).norm(), 1e-5);
			EXPECT_LT(at.state.a.norm(), 1e-7);
			EXPECT_LT(at.state.j.norm(), 1e-7);
			const double expected_x = time == 0          ? 0
			                          : time == duration ? test.distance
			                          : time == ta       ? .5 * test.velocity * ta
			                                             : test.distance - .5 * test.velocity * ta;
			EXPECT_NEAR(at.state.p.x(), expected_x, 1e-7);
			EXPECT_NEAR(at.state.v.x(), time == 0 || time == duration ? 0 : test.velocity, 1e-7);
		}
		// Each real TimeSampler horizon crosses segments; the next control tick starts earlier again.
		TimeSampler sampler(20, duration / 10);
		for (double time : {ta / 2, ta + tc / 2, ta + tc + ta / 2}) {
			state.t = 100 + time;
			const auto expected = reference->getSetpoint(state, state.t);
			SetpointVector sampled;
			ASSERT_TRUE(sampler.getAt(state, {reference}, &sampled));
			for (size_t i = 0; i < sampled.size(); ++i) {
				const double query = state.t + i * duration / 10;
				const auto actual = reference->getSetpoint(state, query);
				EXPECT_LT((actual.state.p - sampled[i].state.p).norm(), 1e-7);
				EXPECT_LT((actual.state.v - sampled[i].state.v).norm(), 1e-7);
				EXPECT_LT((actual.state.a - sampled[i].state.a).norm(), 1e-7);
				EXPECT_LT((actual.state.j - sampled[i].state.j).norm(), 1e-7);
				EXPECT_LT(actual.state.q().angularDistance(sampled[i].state.q()), 1e-7);
				EXPECT_LT((actual.input.omega - sampled[i].input.omega).norm(), 1e-7);
				EXPECT_NEAR(actual.input.collective_thrust, sampled[i].input.collective_thrust, 1e-7);
			}
			const auto revisited = reference->getSetpoint(state, state.t);
			EXPECT_LT((revisited.state.x - expected.state.x).norm(), 1e-8);
		}
		for (double time : {duration + 1e-6, duration + 100}) {
			const auto end = reference->getSetpoint(state, 100 + time);
			EXPECT_TRUE(end.state.p.isApprox(target));
			EXPECT_TRUE(end.state.v.isZero());
			EXPECT_TRUE(end.state.a.isZero());
			EXPECT_TRUE(end.state.j.isZero());
			EXPECT_TRUE(end.input.omega.isZero());
			EXPECT_DOUBLE_EQ(end.input.collective_thrust, G);
		}
	}
}

TEST(CapturedOnlineReference, InvalidReplacementClearCaptureAndNearZeroHold) {
	QuadState state;
	state.setZero();
	state.p.z() = 1;
	CapturedReference reference;
	reference.capture(state, 10, false);
	ASSERT_TRUE(reference.goToPosition(Vector<3>(20, 0, 1), 10, 2, 1, .5));
	const auto before = reference.getSetpoint(state, 18);
	for (double invalid : {0.0, -1.0, double(NAN), double(INF)}) {
		EXPECT_FALSE(reference.goToPosition(Vector<3>(1, 0, 1), 11, invalid, 1, .5));
		EXPECT_FALSE(reference.goToPosition(Vector<3>(1, 0, 1), 11, 2, invalid, .5));
		EXPECT_LT((reference.getSetpoint(state, 18).state.x - before.state.x).norm(), 1e-8);
	}
	// Failure after computing a candidate turn must also leave all old segments intact.
	EXPECT_FALSE(reference.goToPosition(Vector<3>(-1e308, 1e308, 1), 11, 2, 1, .5));
	EXPECT_LT((reference.getSetpoint(state, 18).state.x - before.state.x).norm(), 1e-8);
	for (double distance : {0.0, 1e-12}) {
		const Vector<3> target(distance, 0, 1);
		ASSERT_TRUE(reference.goToPosition(target, 10, 2, 1, .5));
		EXPECT_TRUE(reference.hasOnlineTarget());
		for (double time : {10.0, 10.75, 11.5, 100.0}) EXPECT_TRUE(reference.getSetpoint(state, time).state.valid());
		EXPECT_LT((reference.getSetpoint(state, 100).state.p - target).norm(), 1e-15);
		EXPECT_FALSE(reference.movingAt(11.5));
	}
	reference.clearOnlineTarget();
	EXPECT_FALSE(reference.hasOnlineTarget());
	EXPECT_TRUE(reference.isHover());
	EXPECT_FALSE(reference.movingAt(10));
	EXPECT_TRUE(reference.getSetpoint(state, 100).state.p.isApprox(state.p));
	ASSERT_TRUE(reference.goToPosition(Vector<3>(20, 0, 1), 10, 2, 1, .5));
	state.p = Vector<3>(3, 2, 1);
	reference.capture(state, 12, false);
	EXPECT_FALSE(reference.hasOnlineTarget());
	EXPECT_FALSE(reference.pauseActive());
	EXPECT_TRUE(reference.getSetpoint(state, 100).state.p.isApprox(state.p));
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
		ASSERT_TRUE(reference->goToPosition(target, state.t, 2.0, 1.0, .5));
		const auto start = reference->getSetpoint(state, state.t);
		EXPECT_LT((start.state.p - state.p).norm(), 1e-8);
		EXPECT_LT(start.state.v.norm(), 1e-8);
		EXPECT_LT(start.state.a.norm(), 1e-8);
		EXPECT_LT(start.state.j.norm(), 1e-8);
		EXPECT_NEAR(start.state.getYaw(), yaw, 1e-8);
		const double heading = std::atan2(target.y() - state.p.y(), target.x() - state.p.x());
		const double turn_duration = std::max(1.875 * std::abs(std::remainder(heading - yaw, 2 * M_PI)) / .5, 1.5);
		const double duration = 2 * std::sqrt(1.875 * (target - state.p).norm());
		const auto middle = reference->getSetpoint(state, state.t + turn_duration + duration / 2);
		EXPECT_LT((middle.state.p - (target + state.p) / 2).norm(), 1e-6);
		EXPECT_GT(middle.state.v.norm(), .1);
		const auto dynamic = reference->getSetpoint(state, state.t + turn_duration + duration / 6);
		EXPECT_NEAR(dynamic.input.collective_thrust, (dynamic.state.a - GVEC).norm(), 1e-8);
		EXPECT_TRUE(dynamic.input.omega.isApprox(dynamic.state.w));
		EXPECT_GT(dynamic.input.omega.norm(), 1e-4);
		EXPECT_NEAR(dynamic.state.q().norm(), 1, 1e-8);
		state.t += turn_duration + duration - .2;
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
			EXPECT_NEAR(end.state.getYaw(), heading, 1e-8);
			EXPECT_EQ(end.state.t, state.t + offset);
		}
		EXPECT_TRUE(reference->movingAt(state.t));
		EXPECT_FALSE(reference->movingAt(state.t + 1));
	}
}

TEST(CapturedOnlineReference, TurnsInPlaceAlongShortestArcThenMovesWithConsistentBodyRates) {
	for (const auto& headings : {Vector<2>(0.7, -1.5), Vector<2>(3.1, -3.1), Vector<2>(-3.1, 3.1)}) {
		QuadState state;
		state.setZero();
		state.t = 1000000;
		state.p = Vector<3>(2, 3, 4);
		state.q(headings.x());
		const Vector<3> target = state.p + Vector<3>(3 * std::cos(headings.y()), 3 * std::sin(headings.y()), 1);
		const double delta = std::remainder(headings.y() - headings.x(), 2 * M_PI);
		const double turn_duration = std::max(1.875 * std::abs(delta) / .5, 1.5);
		const double move_duration = 2 * std::sqrt(1.875 * (target - state.p).norm());
		CapturedReference reference;
		reference.capture(state, state.t, false);
		ASSERT_TRUE(reference.goToPosition(target, state.t, 2, 1, .5));
		for (int i = 0; i <= 100; ++i) {
			const auto point = reference.getSetpoint(state, state.t + turn_duration * i / 100);
			EXPECT_LT((point.state.p - state.p).norm(), 1e-8);
			EXPECT_LT(point.state.v.norm(), 1e-8);
			EXPECT_LE(point.state.w.norm(), .5 + 1e-8);
			EXPECT_GE(point.state.w.z() * delta, -1e-8);
		}
		const auto middle_turn = reference.getSetpoint(state, state.t + turn_duration / 2);
		EXPECT_NEAR(std::remainder(middle_turn.state.getYaw() - headings.x(), 2 * M_PI), delta / 2, 1e-8);
		for (double t : {0.0, turn_duration, turn_duration + move_duration}) {
			const auto boundary = reference.getSetpoint(state, state.t + t);
			EXPECT_LT(boundary.state.v.norm(), 1e-8);
			EXPECT_LT(boundary.state.a.norm(), 1e-8);
			EXPECT_LT(boundary.state.j.norm(), 1e-8);
			EXPECT_LT(boundary.state.w.norm(), 1e-8);
		}
		for (int i = 1; i < 200; ++i) {
			const double t = state.t + (turn_duration + move_duration) * i / 200;
			constexpr double dt = 1e-4;
			const auto before = reference.getSetpoint(state, t - dt);
			const auto after = reference.getSetpoint(state, t + dt);
			const auto point = reference.getSetpoint(state, t);
			const Eigen::AngleAxisd rotation(before.state.q().conjugate() * after.state.q());
			const Vector<3> measured_rate = rotation.axis() * rotation.angle() / (2 * dt);
			EXPECT_LT((point.state.w - measured_rate).norm(), 2e-6);
			EXPECT_LT((point.input.omega - measured_rate).norm(), 2e-6);
			EXPECT_NEAR(point.input.collective_thrust, (point.state.a - GVEC).norm(), 1e-8);
			if (t > state.t + turn_duration) {
				EXPECT_NEAR(std::remainder(point.state.getYaw() - headings.y(), 2 * M_PI), 0, 1e-8);
			}
		}
		// Out-of-order horizon queries must not advance the execution phase.
		reference.getSetpoint(state, state.t + turn_duration + move_duration + 100);
		EXPECT_TRUE(reference.movingAt(state.t + turn_duration / 2));
		EXPECT_LT((reference.getSetpoint(state, state.t).state.p - state.p).norm(), 1e-8);
		const auto end = reference.getSetpoint(state, state.t + turn_duration + move_duration + 1);
		EXPECT_TRUE(end.state.p.isApprox(target));
		EXPECT_NEAR(std::remainder(end.state.getYaw() - headings.y(), 2 * M_PI), 0, 1e-8);
	}
}

TEST(CapturedOnlineReference, VerticalAndZeroDisplacementKeepHeadingWithoutTurnDelay) {
	QuadState state;
	state.setZero();
	state.t = 100;
	state.p = Vector<3>(2, 3, 4);
	state.q(2.4);
	for (double dz : {0.0, 1.0, -1.0}) {
		CapturedReference reference;
		reference.capture(state, state.t, false);
		const Vector<3> target = state.p + Vector<3>(0, 0, dz);
		ASSERT_TRUE(reference.goToPosition(target, state.t, 2, 1, .5));
		const double duration = std::max(1.5, 2 * std::sqrt(1.875 * std::abs(dz)));
		const auto middle = reference.getSetpoint(state, state.t + duration / 2);
		EXPECT_LT((middle.state.p - (state.p + target) / 2).norm(), 1e-8);
		for (double dt : {0.0, duration / 2, duration, 100.0}) {
			const auto point = reference.getSetpoint(state, state.t + dt);
			EXPECT_NEAR(point.state.getYaw(), 2.4, 1e-8);
			EXPECT_LT(point.state.w.norm(), 1e-8);
		}
		EXPECT_FALSE(reference.movingAt(state.t + duration + 1e-6));
	}
}

TEST(CapturedOnlineReference, PausePreservesDerivativesAndNeverRevealsOldTrajectory) {
	for (const Vector<3>& target : {Vector<3>(8, 5, 6), Vector<3>(0, 0, 8)}) {
		for (double deceleration : {3.0, .3}) {
			for (double pause_time : {3.0, 3.9, 6.5}) {
				QuadState state;
				state.setZero();
				state.t = 100;
				state.q(std::atan2(target.y(), target.x()));
				CapturedReference reference;
				reference.capture(state, state.t, false);
				const double heading = reference.getSetpoint(state, state.t).state.getHoverState().getYaw();
				const double delta = target.head<2>().norm() > 1e-6
				                             ? std::remainder(std::atan2(target.y(), target.x()) - heading, 2 * M_PI)
				                             : 0;
				const double turn_duration = delta == 0 ? 0 : std::max(1.5, 1.875 * std::abs(delta) / .5);
				ASSERT_TRUE(reference.goToPosition(target, state.t, 2, 1, .5));
				const double now = state.t + turn_duration + pause_time;
				const auto before = reference.getSetpoint(state, now).state;
				ASSERT_GT(before.v.norm(), .1);
				if (pause_time == 3.9) {
					ASSERT_LT(before.a.norm(), 1e-7);
					ASSERT_LT(before.j.norm(), 1e-7);
				} else {
					ASSERT_GT(before.a.norm(), .01);
					ASSERT_GT(before.j.norm(), .01);
				}
				const double duration = std::max(1.5, 1.875 * before.v.norm() / deceleration);
				const Vector<3> endpoint = before.p + .5 * duration * before.v + .1 * duration * duration * before.a +
				                           duration * duration * duration / 120 * before.j;
				ASSERT_TRUE(reference.pauseHover(now, deceleration, Vector<3>::Constant(-100), Vector<3>::Constant(100))
				                    .empty());
				const auto after = reference.getSetpoint(state, now).state;
				EXPECT_LT((after.p - before.p).norm(), 1e-7);
				EXPECT_LT((after.v - before.v).norm(), 1e-7);
				EXPECT_LT((after.a - before.a).norm(), 1e-7);
				EXPECT_LT((after.j - before.j).norm(), 1e-7);
				EXPECT_LT(after.q().angularDistance(before.q()), 1e-7);
				EXPECT_LT((after.w - before.w).norm(), 1e-7);
				reference.getSetpoint(state, now + 1000);  // MPC horizon is not execution time.
				EXPECT_TRUE(reference.movingAt(now));
				EXPECT_FALSE(reference.movingAt(now + duration + 1e-6));
				ASSERT_TRUE(
				        reference.pauseHover(now + .3, deceleration, Vector<3>::Constant(-100), Vector<3>::Constant(100))
				                .empty());
				EXPECT_FALSE(reference.movingAt(now + duration + 1e-6));
				for (int i = 0; i < 100; ++i)
					EXPECT_TRUE(reference.getSetpoint(state, now + duration * i / 100).state.valid());
				for (double offset : {duration + 1e-6, duration + 10, duration + 1000}) {
					const auto held = reference.getSetpoint(state, now + offset).state;
					EXPECT_LT((held.p - endpoint).norm(), 1e-7);
					EXPECT_TRUE(held.v.isZero());
					EXPECT_TRUE(held.a.isZero());
					EXPECT_TRUE(held.j.isZero());
					EXPECT_TRUE(held.w.isZero());
				}
			}
		}
	}
}

TEST(CapturedOnlineReference, PauseYawCrossesPiAndPreservesUnwrappedStopAngle) {
	for (double initial : {3.1, -3.1}) {
		QuadState state;
		state.setZero();
		state.q(initial);
		CapturedReference reference;
		reference.capture(state, 10, false);
		const double goal_heading = -initial;
		ASSERT_TRUE(reference.goToPosition(Vector<3>(5 * std::cos(goal_heading), 5 * std::sin(goal_heading), 0), 10, 2, 1, .5));
		const auto before = reference.getSetpoint(state, 10.5).state;
		ASSERT_GT(std::abs(before.w.z()), .01);
		ASSERT_TRUE(reference.pauseHover(10.5, 3, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
		const auto after = reference.getSetpoint(state, 10.5).state;
		EXPECT_LT(after.q().angularDistance(before.q()), 1e-8);
		EXPECT_NEAR(after.w.z(), before.w.z(), 1e-8);
		const auto end = reference.getSetpoint(state, 100).state;
		EXPECT_NEAR(std::remainder(end.getYaw() - before.getYaw() - .75 * before.w.z(), 2 * M_PI), 0, 1e-8);
		EXPECT_LT(end.p.norm(), 1e-8);
		EXPECT_TRUE(end.w.isZero());
	}
	// CSV reference with a rate whose unwrapped relative stop angle exceeds pi.
	SetpointVector points(2);
	for (int i = 0; i < 2; ++i) {
		points[i].state.setZero();
		points[i].state.t = 100 * i;
		points[i].state.p.x() = 100 * i;
		points[i].state.v.x() = 1;
		points[i].state.w.z() = 5;
		points[i].input = Command(100 * i, G, points[i].state.w);
	}
	QuadState state;
	state.setZero();
	CapturedReference reference;
	reference.setTrajectory(std::make_shared<SampledTrajectory>(points));
	reference.capture(state, 10, true);
	ASSERT_TRUE(reference.pauseHover(11, 3, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
	double angle = 0, previous = 0;
	for (int i = 0; i <= 150; ++i) {
		const auto sample = reference.getSetpoint(state, 11 + .01 * i).state;
		angle += std::remainder(sample.getYaw() - previous, 2 * M_PI);
		previous = sample.getYaw();
		EXPECT_GE(sample.w.z(), -1e-8);
	}
	EXPECT_NEAR(angle, 3.75, 1e-7);
	EXPECT_TRUE(reference.getSetpoint(state, 20).state.w.isZero());
	const auto stop = reference.getSetpoint(state, 20).state;
	EXPECT_TRUE(reference.getSetpoint(state, 1000).state.p.isApprox(stop.p));
	EXPECT_TRUE(reference.getSetpoint(state, 1000).state.v.isZero());
	EXPECT_LT(stop.p.x(), 10);
	EXPECT_TRUE(reference.pauseActive());
}

TEST(CapturedOnlineReference, PauseUsesFlatHeadingDuringTiltedTranslationAndYaw) {
	SetpointVector points(2);
	for (int i = 0; i < 2; ++i) {
		auto& state = points[i].state;
		state.setZero();
		state.t = i * 100;
		state.p = Vector<3>(i * 100, 0, 1);
		state.v = Vector<3>(1, .3, .2);
		state.a = Vector<3>(2, -3, 1);
		state.j = Vector<3>(.4, .2, .3);
		const Vector<3> thrust = state.a - GVEC;
		const Quaternion tilt = Quaternion::FromTwoVectors(Vector<3>::UnitZ(), thrust);
		state.q(tilt * Quaternion(Eigen::AngleAxisd(3.1, Vector<3>::UnitZ())));
		const Vector<3> body_jerk = state.q().conjugate() * state.j;
		state.w = Vector<3>(-body_jerk.y() / thrust.norm(), body_jerk.x() / thrust.norm(), .7);
		points[i].input = Command(state.t, thrust.norm(), state.w);
	}
	QuadState state;
	state.setZero();
	CapturedReference reference;
	reference.setTrajectory(std::make_shared<SampledTrajectory>(points));
	reference.capture(state, 10, true);
	const auto before = reference.getSetpoint(state, 11).state;
	ASSERT_GT(std::abs(std::remainder(before.getYaw() - 3.1, 2 * M_PI)), .01);
	ASSERT_TRUE(reference.pauseHover(11, 3, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
	const auto after = reference.getSetpoint(state, 11).state;
	EXPECT_LT(after.q().angularDistance(before.q()), 1e-8);
	EXPECT_LT((after.w - before.w).norm(), 1e-8);
	EXPECT_LT((after.a - before.a).norm(), 1e-8);
	EXPECT_LT((after.j - before.j).norm(), 1e-8);
	const auto end = reference.getSetpoint(state, 20).state;
	EXPECT_NEAR(std::remainder(end.getYaw() - 3.1 - .75 * .7, 2 * M_PI), 0, 1e-8);
	EXPECT_TRUE(end.w.isZero());
}

TEST(CapturedOnlineReference, RejectedBrakeIsAtomicAndFixedReferenceDoesNotRecaptureNoise) {
	QuadState state;
	state.setZero();
	state.p = Vector<3>(1, 2, 3);
	CapturedReference reference;
	reference.capture(state, 10, false);
	ASSERT_TRUE(reference.goToPosition(Vector<3>(8, 2, 3), 10, 2, 1, .5));
	const auto before = reference.getSetpoint(state, 12).state;
	EXPECT_FALSE(reference.pauseHover(12, 3, Vector<3>::Constant(-1), Vector<3>::Constant(1)).empty());
	EXPECT_FALSE(reference.pauseActive());
	EXPECT_LT((reference.getSetpoint(state, 12).state.x - before.x).norm(), 1e-8);
	EXPECT_FALSE(reference.pauseHover(12, NAN, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
	const auto held = reference.getSetpoint(state, 100).state;
	state.p += Vector<3>(.1, -.1, .05);
	ASSERT_TRUE(reference.pauseHover(100, 3, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
	EXPECT_FALSE(reference.movingAt(100));
	EXPECT_TRUE(reference.getSetpoint(state, 100).state.p.isApprox(held.p));
	ASSERT_TRUE(reference.pauseHover(200, 3, Vector<3>::Constant(-100), Vector<3>::Constant(100)).empty());
	EXPECT_TRUE(reference.getSetpoint(state, 200).state.p.isApprox(held.p));
}

TEST_F(HardwareGeo, PauseKeepsAutoAndRequiresActualHoverBeforeNewGoal) {
	for (int i = 0; i < 80; ++i) step(false);
	ASSERT_TRUE(step(true).permit_override);
	const Vector<3> target = _state.p + Vector<3>(8, 0, 0);
	ASSERT_TRUE(step(true, true, true, false, &target).goal_accepted);
	for (int i = 0; i < 200; ++i) step(true);
	auto decision = step(true, true, true, false, nullptr, true);
	ASSERT_TRUE(decision.goal_accepted);
	ASSERT_TRUE(decision.pause_active);
	ASSERT_TRUE(decision.permit_override);
	EXPECT_EQ(_runtime->warmCycles(), 50u);
	const auto endpoint = decision.active_endpoint;
	decision = step(true, true, true, false, &target);
	EXPECT_FALSE(decision.goal_accepted);
	EXPECT_TRUE(decision.pause_active);
	for (int i = 0; i < 500; ++i) {
		decision = step(true);
		ASSERT_TRUE(decision.permit_override);
	}
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Settling);
	_state.p = endpoint;
	_state.v.x() = .3;
	EXPECT_EQ(step(true).online_goal_phase, OnlineGoalPhase::Settling);
	_state.v.setZero();
	EXPECT_EQ(step(true).online_goal_phase, OnlineGoalPhase::Stable);
	decision = step(true, true, true, false, nullptr, true);
	EXPECT_TRUE(decision.active_endpoint.isApprox(endpoint));
	decision = step(true, true, true, false, &target);
	EXPECT_TRUE(decision.goal_accepted);
	EXPECT_FALSE(decision.pause_active);
	ASSERT_TRUE(step(true, true, true, false, nullptr, true).pause_active);
	EXPECT_FALSE(step(false).pause_active);
	ASSERT_TRUE(step(true).permit_override);
	ASSERT_TRUE(step(true, true, true, false, nullptr, true).pause_active);
	_runtime->reportOutputFault();
	EXPECT_FALSE(step(true).pause_active);
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
	EXPECT_TRUE(decision.goal_accepted);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Moving);
	EXPECT_LT((decision.reference.p - Vector<3>(2, 3, 4)).norm(), 1e-8);
	EXPECT_EQ(_runtime->warmCycles(), 50u);
	EXPECT_NE(step(true, true, true, false, &second).goal_event.find("trajectory moving"), std::string::npos);
	Vector<3> invalid(NAN, 0, 0);
	EXPECT_NE(step(true, true, true, false, &invalid).goal_event.find("nonfinite"), std::string::npos);
	for (int i = 0; i < 290; ++i) decision = step(true);
	EXPECT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.trajectory_active);
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	EXPECT_TRUE(decision.reference.v.isZero());
	EXPECT_NE(decision.reason.find("settling"), std::string::npos);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Settling);
	EXPECT_NE(step(true, true, true, false, &second).goal_event.find("position error"), std::string::npos);
	_state.p = target;
	decision = step(true);
	EXPECT_TRUE(decision.goal_rejection.empty());
	EXPECT_NE(decision.reason.find("stable hover"), std::string::npos);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Stable);
	const double auto_elapsed = decision.reference_elapsed;
	decision = step(true, true, true, false, &second);
	ASSERT_NE(decision.goal_event.find("accepted"), std::string::npos);
	EXPECT_TRUE(decision.goal_accepted);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Moving);
	EXPECT_GT(decision.reference_elapsed, auto_elapsed);
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	// Still turning at the original position: another command must remain busy.
	for (int i = 0; i < 170; ++i) decision = step(true);
	EXPECT_TRUE(decision.reference.p.isApprox(target));
	EXPECT_NE(step(true, true, true, false, &target).goal_event.find("trajectory moving"), std::string::npos);
	for (int i = 0; i < 720; ++i) decision = step(true);
	EXPECT_TRUE(decision.reference.p.isApprox(second));
	EXPECT_NEAR(decision.reference.getYaw(), M_PI / 2, 1e-8);
	EXPECT_TRUE(decision.permit_override);
	EXPECT_EQ(_runtime->warmCycles(), 50u);
	step(false);
	decision = step(true);
	EXPECT_TRUE(decision.permit_override);
	EXPECT_TRUE(decision.reference.p.isApprox(_state.p));
	EXPECT_EQ(decision.reason.find("online target"), std::string::npos);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::None);
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
	EXPECT_TRUE(decision.goal_accepted);
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::Moving);
	for (int i = 0; i < 900; ++i) {
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
	EXPECT_EQ(decision.online_goal_phase, OnlineGoalPhase::None);
}

}  // namespace
}  // namespace agi::hardware
