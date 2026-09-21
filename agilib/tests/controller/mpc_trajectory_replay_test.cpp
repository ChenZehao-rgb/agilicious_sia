#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <vector>

#include "agilib/controller/mpc/controller_mpc.hpp"
#include "agilib/pilot/pilot_params.hpp"
#include "agilib/reference/trajectory_csv.hpp"
#include "agilib/reference/trajectory_reference/sampled_trajectory.hpp"

namespace agi {
namespace {

// Reference-state replay checks the solver over the complete original time base.
// This is an oracle-input numerical regression, not a closed-loop tracking test.
class MpcTrajectoryReplay : public testing::TestWithParam<const char*> {};

TEST_P(MpcTrajectoryReplay, OriginalTimeBase) {
	const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
	const auto path = root / "miscellaneous/datasets/ref_trajs/open_source" / GetParam();
	const auto rows = trajectory_csv::readTrajectoryRows(path);
	const double source_mass = trajectory_csv::estimateSourceMass(rows);
	const auto points = trajectory_csv::loadTrajectory(rows, 0.0, Vector<3>(0.0, 0.0, 5.0), 0.0, source_mass, 0.0);
	SampledTrajectory trajectory(points);
	const Yaml config(root / "agi_ros2/config/simulation.yaml");
	PilotParams pilot;
	pilot.directory_ = root / "agi_ros2/config";
	ASSERT_TRUE(pilot.load(config["pilot"], "MPC"));
	auto parameters = std::make_shared<MpcParameters>();
	ASSERT_TRUE(parameters->load(pilot.pipeline_cfg_.outer_controller_cfg.parameters));
	MpcController controller(pilot.quad_, parameters);
	ASSERT_TRUE(controller.reset(points.front().state));

	std::vector<double> durations;
	int failures = 0;
	SetpointVector prediction;
	const double duration = points.back().state.t;
	for (double t = 0.0; t <= duration; t += 0.01) {
		QuadState state = trajectory.getSetpoint(points.front().state, t).state;
		state.t = t;
		SetpointVector reference;
		for (int stage = 0; stage < controller.horizonLength(); ++stage) {
			reference.push_back(trajectory.getSetpoint(state, std::min(duration, t + stage * controller.dt())));
		}
		const auto before = std::chrono::steady_clock::now();
		const bool success = controller.getCommand(state, reference, &prediction);
		durations.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count());
		if (!success) {
			if (++failures <= 3) ADD_FAILURE() << path.filename() << " failed at t=" << t;
			continue;
		}
		ASSERT_FALSE(prediction.empty());
		ASSERT_TRUE(prediction.front().input.isRatesThrust());
		ASSERT_FALSE(prediction.front().input.isSingleRotorThrusts());
		ASSERT_GE(prediction.front().input.collective_thrust, 0.0);
	}
	std::sort(durations.begin(), durations.end());
	RecordProperty("trajectory", GetParam());
	RecordProperty("duration_seconds", duration);
	RecordProperty("samples", static_cast<int>(durations.size()));
	RecordProperty("failures", failures);
	RecordProperty("cycle_p99_ms", durations[static_cast<std::size_t>(0.99 * (durations.size() - 1))] * 1000.0);
	RecordProperty("cycle_max_ms", durations.back() * 1000.0);
	EXPECT_EQ(failures, 0);
}

INSTANTIATE_TEST_SUITE_P(OriginalCsv, MpcTrajectoryReplay,
                         testing::Values("aggressive_50mps.csv", "HELIX_FWD20_50mps.csv", "CPC33_Z1.csv"));

}  // namespace
}  // namespace agi
