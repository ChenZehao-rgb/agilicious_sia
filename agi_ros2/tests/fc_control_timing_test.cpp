#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "agi_ros2/fc_control_schedule.h"
#include "agilib/pilot/hardware_pilot.hpp"

namespace agi_ros2 {
namespace {
using agi::hardware::Evidence;
using agi::hardware::HardwarePilot;
using agi::hardware::NavigationPolicy;
using agi::hardware::ReceiverPolicy;
using agi::hardware::SafetyGate;

TEST(FcSchedule, PublishedBeforeTimerDeliveredAfterTimer) {
	FcControlSchedule schedule;
	EXPECT_EQ(schedule.observe("s", 1, 0, 100, 108.97795736285993, 108.983113185, 108.983389519, true),
	          FcControlSchedule::Update::Sample);
	ASSERT_TRUE(schedule.take(108.983389519));
	// Publishing does not update the controller cache. The old periodic tick
	// would compute/reject that cache; the new watchdog cannot request a solve.
	EXPECT_FALSE(schedule.take(108.994073923));
	EXPECT_GT(108.994089431 - 108.97795736285993, SafetyGate::kStateMaxAge);
	EXPECT_EQ(schedule.observe("s", 1, 0, 101, 108.98795736285993, 108.993806011, 108.994358308, true),
	          FcControlSchedule::Update::Sample);
	EXPECT_TRUE(schedule.take(108.994358308));
	EXPECT_LT(108.994358308 + .00213 - 108.98795736285993, SafetyGate::kStateMaxAge);
}

TEST(FcSchedule, DuplicatesGenerationsReorderingAndRestart) {
	FcControlSchedule s;
	const auto sample = FcControlSchedule::Update::Sample;
	const auto fault = FcControlSchedule::Update::Fault;
	const auto ignore = FcControlSchedule::Update::Ignore;
	EXPECT_EQ(s.observe("a", 1, 0, 100, 1., 1.004, 1.005, true), sample);
	EXPECT_TRUE(s.take(1.005));
	EXPECT_EQ(s.observe("a", 1, 0, 100, 1., 1.006, 1.006, true), ignore);
	EXPECT_FALSE(s.take(1.015));
	EXPECT_EQ(s.observe("a", 1, 0, 99, .999, 1.007, 1.007, true), fault);
	EXPECT_EQ(s.observe("a", 2, 0, 100, 1., 1.008, 1.008, true), fault);
	// Depth-1 loss of an invalid event still exposes its generation on recovery.
	EXPECT_EQ(s.observe("a", 3, 0, 101, 1.01, 1.014, 1.014, true), sample);
	EXPECT_TRUE(s.take(1.014));
	EXPECT_EQ(s.observe("a", 2, 0, 102, 1.02, 1.024, 1.024, true), ignore);
	EXPECT_EQ(s.observe("b", 4, 0, 1, 1.03, 1.034, 1.034, true), sample);
	EXPECT_TRUE(s.take(1.034));
	EXPECT_EQ(s.observe("a", 3, 0, 103, 1.03, 1.033, 1.035, true), ignore);
	EXPECT_EQ(s.observe("b", 5, 1, 1, 1.04, 1.044, 1.044, true), sample);
	EXPECT_TRUE(s.take(1.044));
	EXPECT_EQ(s.observe("b", 6, 1, 2, 1.06, 1.054, 1.054, true), fault);
	EXPECT_EQ(s.observe("b", 6, 1, 2, 1.05, 1.055, 1.055, false), fault);
	EXPECT_FALSE(s.take(1.1));
}

TEST(FcSchedule, CoalescesBurstsWithoutCatchup) {
	FcControlSchedule s;
	s.observe("s", 1, 0, 1, 1., 1.004, 1.005, true);
	ASSERT_TRUE(s.take(1.005));
	s.observe("s", 1, 0, 2, 1.001, 1.006, 1.006, true);
	EXPECT_FALSE(s.take(1.006));
	s.observe("s", 1, 0, 3, 1.002, 1.007, 1.007, true);
	EXPECT_FALSE(s.take(1.007));
	EXPECT_TRUE(s.take(1.01301));
	EXPECT_FALSE(s.take(1.014));
	EXPECT_EQ(s.coalesced, 1u);
}

TEST(FcSchedule, PhaseScan) {
	// Arrival age, compute duration, timer phase and 50 Hz health are independent.
	// This is a logical time model, not an OS latency benchmark.
	int old_skips = 0;
	for (double age : {.00374, .00524, .00734}) {
		for (double solve : {.001, .00213}) {
			for (int phase = 0; phase < 100; ++phase) {
				FcControlSchedule s;
				for (int i = 0; i < 100; ++i) {
					const double sample = 1. + i * .01;
					const double received = sample + age;
					const double callback = received + .0006;
					const double old_start = callback + phase * .0001;
					if (old_start - sample > .015) ++old_skips;
					ASSERT_EQ(s.observe("s", 1, 0, i + 1, sample, received + .0002, callback, true),
					          FcControlSchedule::Update::Sample);
					ASSERT_TRUE(s.take(callback));
					EXPECT_LT(callback + solve - sample, .015);
					EXPECT_FALSE(s.take(callback + .001));
				}
			}
		}
	}
	EXPECT_GT(old_skips, 0);
}

agi::PilotParams configuration() {
	agi::PilotParams p;
	p.quad_ = agi::Quadrotor(1.5, .25);  // Synthetic test fixture, not flight tuning.
	p.pipeline_cfg_.bridge_cfg.type = "External";
	p.pipeline_cfg_.estimator_cfg.type = "External";
	p.pipeline_cfg_.outer_controller_cfg.type = "MPC";
	p.pipeline_cfg_.sampler_cfg.type = "Time";
	p.velocity_in_bodyframe_ = false;
	return p;
}

Evidence health(double now) {
	Evidence e;
	e.now = e.rc_time = e.status_time = now;
	e.fc_sample_time = now - .004;
	e.fc_receive_time = e.fc_publish_time = now;
	e.fc_health_sample_time = e.fc_health_receive_time = now - .020;
	e.fc_xy_observation_time = e.fc_z_observation_time = now - .050;
	e.fc_session = "fc-session";
	e.fc_generation = 1;
	e.fc_flags = 0xc7;
	e.fc_position_variance.fill(.01);
	e.fc_velocity_variance.fill(.01);
	e.fc_valid = e.clock_aligned = e.heading_valid = e.accuracy_ok = true;
	e.config_verified = e.thrust_mapping_ready = e.geofence_ok = e.msp_healthy = true;
	e.receiver_valid = e.armed = e.rc_link = true;
	e.kill = false;
	return e;
}

class PilotFixture : public testing::Test {
protected:
	PilotFixture()
	        : _pilot(
	                  configuration(), [this] { return _time; }, [this] { return _time + (_clock_calls++ >= 2 ? _finish_delay : 0); },
	                  NavigationPolicy::FcFused, 0., ReceiverPolicy::LatchedSwitches) {
		_state.setZero();
	}
	agi::hardware::ControlDecision step(bool automatic = false, bool armed = true, bool kill = false, bool valid = true,
	                                    uint64_t generation = 1, bool compute = true) {
		_time += .01;
		_state.t = _time - .004;
		_clock_calls = 0;
		auto e = health(_time);
		e.auto_switch = automatic;
		e.armed = armed;
		e.kill = kill;
		e.fc_valid = valid;
		e.fc_generation = generation;
		return _pilot.tick(_state, e, false, valid, compute);
	}
	double _time = 10;
	double _finish_delay = 0;
	int _clock_calls = 0;
	agi::QuadState _state;
	HardwarePilot _pilot;
};

TEST_F(PilotFixture, FiftyIndependentComputationsAndRecovery) {
	for (int i = 1; i <= 49; ++i) {
		ASSERT_TRUE(step().evidence.command_valid);
		ASSERT_EQ(_pilot.warmCycles(), static_cast<unsigned>(i));
	}
	EXPECT_FALSE(step().permit_override);
	EXPECT_EQ(_pilot.warmCycles(), 50u);
	EXPECT_TRUE(step(true).permit_override);
	EXPECT_TRUE(step(true).trajectory_active);
	EXPECT_FALSE(step(true, true, false, false).permit_override);
	EXPECT_EQ(_pilot.warmCycles(), 0u);
	for (int i = 0; i < 60; ++i) EXPECT_FALSE(step(true).permit_override);
	EXPECT_FALSE(step(false).permit_override);
	EXPECT_TRUE(step(true).permit_override);
	// A skipped invalid notification still changes generation on the next sample.
	EXPECT_FALSE(step(true, true, false, true, 2).permit_override);
	EXPECT_EQ(_pilot.warmCycles(), 1u);
	for (int i = 0; i < 60; ++i) EXPECT_FALSE(step(true, true, false, true, 2).permit_override);
	step(false, true, false, true, 2);
	EXPECT_TRUE(step(true, true, false, true, 2).permit_override);
}

TEST_F(PilotFixture, ArmKillAndWatchdogDoNotCompute) {
	for (int i = 0; i < 50; ++i) step();
	EXPECT_TRUE(step(true).permit_override);
	auto stopped = step(true, true, false, true, 1, false);
	EXPECT_FALSE(stopped.evidence.command_valid);
	EXPECT_TRUE(std::isnan(stopped.evidence.solve_seconds));
	EXPECT_FALSE(stopped.trajectory_active);
	EXPECT_EQ(_pilot.warmCycles(), 0u);
	EXPECT_FALSE(step(true, false).evidence.command_valid);
	EXPECT_FALSE(step(true, true, true).evidence.command_valid);
	for (int i = 0; i < 55; ++i) EXPECT_FALSE(step(true).permit_override);
	step();
	EXPECT_TRUE(step(true).permit_override);
}

TEST_F(PilotFixture, FiniteResultPastDeadlineCannotAuthorize) {
	for (int i = 0; i < 50; ++i) step();
	ASSERT_TRUE(step(true).permit_override);
	_finish_delay = .012;
	const auto result = step(true);
	EXPECT_TRUE(result.evidence.command_valid);  // Diagnostic result remains visible.
	EXPECT_GT(result.evidence.now - result.evidence.fc_sample_time, .015);
	EXPECT_FALSE(result.permit_override);
	EXPECT_FALSE(result.trajectory_active);
	EXPECT_EQ(_pilot.warmCycles(), 0u);
}

TEST(FcOutputSafety, HoldDoesNotAdmitStaleInputOrFreezeOtherEvidence) {
	using agi::hardware::FcStateCheck;
	auto e = health(10.);
	e.command_time = 10.;
	e.now = 10.012;
	EXPECT_NE(SafetyGate::fcFailure(e), nullptr);
	EXPECT_EQ(SafetyGate::fcFailure(e, FcStateCheck::Hold), nullptr);
	EXPECT_EQ(SafetyGate::fcFailure(e, FcStateCheck::CommandHold), nullptr);

	// A newly generated command must not disguise an already stale input.
	e.command_time = 10.012;
	EXPECT_NE(SafetyGate::fcFailure(e, FcStateCheck::CommandHold), nullptr);
	e.command_time = 10.;
	e.fc_health_sample_time = e.now - .061;
	EXPECT_NE(SafetyGate::fcFailure(e, FcStateCheck::CommandHold), nullptr);

	e = health(10.);
	e.command_time = 10.;
	e.now = 10.012;
	e.rc_time = e.now - .101;
	EXPECT_NE(SafetyGate::inputFailure(e, NavigationPolicy::FcFused, 0., ReceiverPolicy::LatchedSwitches, FcStateCheck::CommandHold),
	          nullptr);
	e = health(10.);
	e.command_time = e.now + .001;
	EXPECT_NE(SafetyGate::fcFailure(e, FcStateCheck::CommandHold), nullptr);
}

TEST(FcOutputSafety, IndependentExpiryAndHiddenInvalidNotification) {
	SafetyGate gate(NavigationPolicy::FcFused, 0., ReceiverPolicy::LatchedSwitches);
	auto e = health(10.);
	e.command_time = 10.;
	e.command_valid = e.controller_warm = true;
	e.solve_seconds = .001;
	gate.update(e, agi::hardware::FcStateCheck::CommandHold);  // Establish session.
	EXPECT_FALSE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	e.auto_switch = true;
	ASSERT_TRUE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	e.now = 10.012;  // Sample age 16 ms: hold a command created from fresh state.
	EXPECT_TRUE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	e.now = 10.022;  // Sample age 26 ms: command age is still only 22 ms.
	EXPECT_FALSE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	EXPECT_EQ(gate.reason(), "FC state sample stale/future");
	e = health(10.03);
	e.command_time = e.now;
	e.command_valid = e.controller_warm = e.auto_switch = true;
	e.solve_seconds = .001;
	EXPECT_FALSE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	e.auto_switch = false;
	gate.update(e, agi::hardware::FcStateCheck::CommandHold);
	e.auto_switch = true;
	EXPECT_TRUE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
	++e.fc_generation;
	EXPECT_FALSE(gate.update(e, agi::hardware::FcStateCheck::CommandHold));
}
}  // namespace
}  // namespace agi_ros2
