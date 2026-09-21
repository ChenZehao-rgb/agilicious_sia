#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>

#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/pilot/pilot_params.hpp"
#include "agilib/utils/module_config.hpp"

namespace agi {
namespace {

std::filesystem::path repositoryRoot() { return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path(); }

TEST(RuntimeConfig, InlinePilotPreservesSimulationModelAndMpcParameters) {
	const auto root = repositoryRoot();
	const Yaml profile(root / "agi_ros2/config/simulation.yaml");
	EXPECT_EQ(profile["flight"]["trajectory"].as<std::string>(), "");
	EXPECT_EQ(profile["flight"]["thrust_table"].as<std::string>(), "");
	PilotParams inline_params;
	inline_params.directory_ = root / "agi_ros2/config";
	ASSERT_TRUE(inline_params.load(profile["pilot"]));
	const PilotParams legacy(root / "agilib/params/pilot_ros2.yaml", root / "agilib/params");
	EXPECT_DOUBLE_EQ(inline_params.quad_.m_, legacy.quad_.m_);
	EXPECT_TRUE(inline_params.quad_.J_.isApprox(legacy.quad_.J_));
	EXPECT_TRUE(inline_params.quad_.t_BM_.isApprox(legacy.quad_.t_BM_));
	EXPECT_TRUE(inline_params.pipeline_cfg_.outer_controller_cfg.file.empty());
	MpcParameters inline_mpc, legacy_mpc;
	ASSERT_TRUE(inline_mpc.load(inline_params.pipeline_cfg_.outer_controller_cfg.parameters));
	ASSERT_TRUE(legacy_mpc.load(root / "agilib/params/mpc_betaflight_sitl.yaml"));
	EXPECT_TRUE(inline_mpc.Q_pos_.isApprox(legacy_mpc.Q_pos_));
	EXPECT_TRUE(inline_mpc.Q_att_.isApprox(legacy_mpc.Q_att_));
	EXPECT_TRUE(inline_mpc.Q_vel_.isApprox(legacy_mpc.Q_vel_));
	EXPECT_TRUE(inline_mpc.R_.isApprox(legacy_mpc.R_));
	std::shared_ptr<ControllerBase> controller;
	ASSERT_TRUE(inline_params.createController(controller, inline_params.pipeline_cfg_.outer_controller_cfg));
	EXPECT_NE(controller, nullptr);
}

TEST(RuntimeConfig, RejectsInlineAndFileParameterAmbiguity) {
	ModuleConfig module;
	const Yaml yaml(std::string("type: MPC\nfile: mpc.yaml\nparameters:\n  Q_pos_x: 1.0\n"));
	EXPECT_THROW(module.loadIfUndefined(yaml), std::invalid_argument);
}

TEST(RuntimeConfig, RejectsUnconfiguredHardwareModel) {
	const auto root = repositoryRoot();
	const Yaml profile(root / "agi_ros2/config/hardware.yaml");
	PilotParams parameters;
	parameters.directory_ = root / "agi_ros2/config";
	EXPECT_THROW(parameters.load(profile["pilot"]), ParameterException);
}

}  // namespace
}  // namespace agi
