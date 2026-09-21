#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/pilot/pilot_params.hpp"
#include "agilib/utils/module_config.hpp"

namespace agi {
namespace {

std::filesystem::path repositoryRoot() { return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path(); }

Yaml minimalGeoPilot(bool drag_compensation = false, bool external_estimator = true) {
	return Yaml(std::string("pipeline:\n") + "  estimator:\n    type: " + (external_estimator ? "External" : "EKF") +
	            "\n  bridge:\n    type: External\n  sampler:\n    type: Time\n  controller:\n    type: GEO\n"
	            "    parameter_sets:\n      MPC:\n        timing: false\n"
	            "        Q_pos_x: 200\n        Q_pos_y: 200\n        Q_pos_z: 500\n"
	            "        Q_att_x: 5\n        Q_att_y: 5\n        Q_att_z: 200\n        Q_vel: 1\n"
	            "        R_collective_thrust: 0.1\n        R_body_rates: [1.0, 1.0, 1.0]\n        exp_decay: 1.0\n      GEO:\n"
	            "        kpacc: [2.0, 2.0, 3.0]\n        kdacc: [2.0, 2.0, 2.0]\n        kprate: [1.0, 1.0, 1.0]\n"
	            "        kpatt_xy: 5.0\n        kpatt_z: 2.0\n        filter_sampling_frequency: 100.0\n"
	            "        filter_cutoff_frequency: 10.0\n        drag_compensation: " +
	            (drag_compensation ? "true" : "false") +
	            "\nquadrotor:\n  mass: 1.0\n  thrust_min: 0.0\n  thrust_max: 5.0\n  omega_max: [2.0, 2.0, 1.0]\n"
	            "dt_min: 0.01\ndt_telemetry: 0.1\ntraj_type: poly_min_snap\nguard:\n  type: None\n");
}

TEST(RuntimeConfig, InlinePilotPreservesRequiredLimitsAndMigratedMpcParameters) {
	const auto root = repositoryRoot();
	const Yaml profile(root / "agi_ros2/config/simulation.yaml");
	EXPECT_EQ(profile["flight"]["trajectory"].as<std::string>(), "");
	EXPECT_EQ(profile["flight"]["thrust_table"].as<std::string>(), "");
	PilotParams inline_params;
	inline_params.directory_ = root / "agi_ros2/config";
	ASSERT_TRUE(inline_params.load(profile["pilot"]));
	const PilotParams legacy(root / "agilib/params/pilot_ros2.yaml", root / "agilib/params");
	EXPECT_DOUBLE_EQ(inline_params.quad_.m_, legacy.quad_.m_);
	EXPECT_TRUE(inline_params.usesRatesThrustModel());
	EXPECT_TRUE(inline_params.quad_.omega_max_.isApprox(legacy.quad_.omega_max_));
	EXPECT_DOUBLE_EQ(inline_params.quad_.thrust_min_, legacy.quad_.thrust_min_);
	EXPECT_DOUBLE_EQ(inline_params.quad_.thrust_max_, legacy.quad_.thrust_max_);
	EXPECT_FALSE(inline_params.quad_.J_.allFinite());
	EXPECT_FALSE(inline_params.quad_.t_BM_.allFinite());
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

TEST(RuntimeConfig, SelectsMatchingControllerParametersAndRejectsAmbiguity) {
	const Yaml config(std::string("type: MPC\nparameter_sets:\n  MPC:\n    marker: 1\n  GEO:\n    marker: 2\n"));
	ModuleConfig mpc;
	ASSERT_TRUE(mpc.loadIfUndefined(config));
	EXPECT_EQ(mpc.type, "MPC");
	EXPECT_EQ(mpc.parameters["marker"].as<int>(), 1);
	ModuleConfig geo;
	ASSERT_TRUE(geo.loadIfUndefined(config, "GEO"));
	EXPECT_EQ(geo.type, "GEO");
	EXPECT_EQ(geo.parameters["marker"].as<int>(), 2);
	ModuleConfig missing;
	EXPECT_THROW(missing.loadIfUndefined(config, "PID"), std::invalid_argument);
	for (const auto& legacy : {"parameters:\n  marker: 3\n", "file: old.yaml\n"}) {
		ModuleConfig ambiguous;
		const Yaml value(std::string("type: MPC\nparameter_sets:\n  MPC:\n    marker: 1\n") + legacy);
		EXPECT_THROW(ambiguous.loadIfUndefined(value), std::invalid_argument);
	}
	ModuleConfig legacy;
	const Yaml single(std::string("type: MPC\nparameters:\n  marker: 1\n"));
	EXPECT_THROW(legacy.loadIfUndefined(single, "GEO"), std::invalid_argument);
}

TEST(RuntimeConfig, GeoExternalLoadsOnlyRequiredModelAndHasNoFabricatedDynamics) {
	PilotParams parameters;
	parameters.directory_ = repositoryRoot();
	ASSERT_TRUE(parameters.load(minimalGeoPilot()));
	EXPECT_TRUE(parameters.usesRatesThrustModel());
	EXPECT_TRUE(parameters.valid());
	EXPECT_TRUE(parameters.quad_.validRatesThrust());
	EXPECT_FALSE(parameters.quad_.valid());
	EXPECT_DOUBLE_EQ(parameters.quad_.m_, 1.0);
	EXPECT_FALSE(parameters.quad_.J_.allFinite());
	EXPECT_FALSE(parameters.quad_.t_BM_.allFinite());
	EXPECT_FALSE(parameters.quad_.thrust_map_.allFinite());
	EXPECT_FALSE(std::isfinite(parameters.quad_.motor_tau_inv_));
	EXPECT_FALSE(std::isfinite(parameters.quad_.motor_omega_max_));
	EXPECT_FALSE(std::isfinite(parameters.quad_.kappa_));
	std::shared_ptr<ControllerBase> controller;
	ASSERT_TRUE(parameters.createController(controller, parameters.pipeline_cfg_.outer_controller_cfg));
	EXPECT_NE(controller, nullptr);
	EXPECT_DOUBLE_EQ(controller->dt(), .01);
}

TEST(RuntimeConfig, MinimalModelSupportsRateMpcButRejectsDynamicsEstimator) {
	PilotParams mpc;
	mpc.directory_ = repositoryRoot();
	ASSERT_TRUE(mpc.load(minimalGeoPilot(), "MPC"));
	EXPECT_TRUE(mpc.usesRatesThrustModel());
	EXPECT_TRUE(mpc.quad_.validRatesThrust());
	EXPECT_FALSE(mpc.quad_.valid());
	EXPECT_FALSE(mpc.quad_.J_.allFinite());
	EXPECT_FALSE(mpc.quad_.thrust_map_.allFinite());
	std::shared_ptr<ControllerBase> controller;
	ASSERT_TRUE(mpc.createController(controller, mpc.pipeline_cfg_.outer_controller_cfg));
	for (const auto* type : {"MPC", "GEO"}) {
		PilotParams estimator;
		estimator.directory_ = repositoryRoot();
		EXPECT_THROW(estimator.load(minimalGeoPilot(false, false), type), ParameterException);
	}
}

TEST(RuntimeConfig, RateMpcWeightsHaveExplicitUnitsAndRejectObsoleteRotorWeights) {
	const auto yaml = minimalGeoPilot()["pipeline"]["controller"]["parameter_sets"]["MPC"];
	MpcParameters parameters;
	ASSERT_TRUE(parameters.load(yaml));
	EXPECT_DOUBLE_EQ(parameters.R_(0), 0.1);
	EXPECT_TRUE(parameters.R_.tail<3>().isApprox(Vector<3>::Ones()));
	std::ostringstream document;
	document << yaml;
	for (const auto* obsolete : {"R: 6.0\n", "Q_omega_xy: 1.0\n", "Q_omega_z: 1.0\n", "cog_enable: true\n",
	                             "threaded_preparation: true\n", "Q_cog_omega: 1.0\n"}) {
		const Yaml invalid(std::string(obsolete) + document.str());
		EXPECT_THROW(parameters.load(invalid), ParameterException);
	}
	ASSERT_TRUE(parameters.load(yaml));
	parameters.R_(0) = -0.1;
	EXPECT_FALSE(parameters.valid());
	ASSERT_TRUE(parameters.load(yaml));
	parameters.R_(2) = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(parameters.valid());
}

TEST(RuntimeConfig, StandaloneGeoRetainsFullDynamicsModel) {
	const auto root = repositoryRoot();
	const PilotParams parameters(root / "agilib/params/pilot_betaflight_sitl.yaml", root / "agilib/params");
	EXPECT_EQ(parameters.pipeline_cfg_.outer_controller_cfg.type, "GEO");
	EXPECT_FALSE(parameters.usesRatesThrustModel());
	EXPECT_TRUE(parameters.quad_.valid());
	EXPECT_TRUE(parameters.quad_.J_.allFinite());
	EXPECT_TRUE(parameters.quad_.thrust_map_.allFinite());
}

TEST(RuntimeConfig, GeoExternalRejectsDragCompensationWithoutMotorRpm) {
	PilotParams parameters;
	parameters.directory_ = repositoryRoot();
	ASSERT_TRUE(parameters.load(minimalGeoPilot(true)));
	std::shared_ptr<ControllerBase> controller;
	EXPECT_THROW(parameters.createController(controller, parameters.pipeline_cfg_.outer_controller_cfg), ParameterException);
}

TEST(RuntimeConfig, MinimalGeoBoundsMustBeFiniteAndStrict) {
	Quadrotor quad;
	ASSERT_TRUE(quad.loadRatesThrust(minimalGeoPilot()["quadrotor"]));
	quad.omega_max_(0) = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(quad.validRatesThrust());
	ASSERT_TRUE(quad.loadRatesThrust(minimalGeoPilot()["quadrotor"]));
	quad.thrust_max_ = quad.thrust_min_;
	EXPECT_FALSE(quad.validRatesThrust());
	ASSERT_TRUE(quad.loadRatesThrust(minimalGeoPilot()["quadrotor"]));
	quad.m_ = std::numeric_limits<Scalar>::quiet_NaN();
	EXPECT_FALSE(quad.validRatesThrust());
}

}  // namespace
}  // namespace agi
