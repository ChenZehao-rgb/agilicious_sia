#include "agilib/controller/geometric/controller_geo.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

namespace agi {
namespace {

Quadrotor minimalModel() {
	Quadrotor quad;
	quad.m_ = 2.0;
	quad.thrust_min_ = 0.0;
	quad.thrust_max_ = 15.0;
	quad.omega_max_ = Vector<3>(2.0, 2.0, 1.0);
	quad.J_.setConstant(NAN);
	quad.J_inv_.setConstant(NAN);
	quad.t_BM_.setConstant(NAN);
	quad.kappa_ = NAN;
	quad.motor_omega_min_ = NAN;
	quad.motor_omega_max_ = NAN;
	quad.motor_tau_inv_ = NAN;
	quad.thrust_map_.setConstant(NAN);
	quad.torque_map_.setConstant(NAN);
	quad.aero_coeff_1_.setConstant(NAN);
	quad.aero_coeff_3_.setConstant(NAN);
	quad.aero_coeff_h_ = NAN;
	return quad;
}

std::shared_ptr<GeometricControllerParams> gains() {
	auto params = std::make_shared<GeometricControllerParams>();
	params->kp_acc_ = Vector<3>(2.0, 2.0, 3.0);
	params->kd_acc_ = Vector<3>(2.0, 2.0, 2.0);
	params->kp_att_xy_ = 6.0;
	params->kp_att_z_ = 2.0;
	return params;
}

Setpoint hoverReference() {
	QuadState state;
	state.setZero();
	return Setpoint(state, Command(state.t, G, Vector<3>::Zero()));
}

TEST(GeometricController, HoverWorksWithoutMotorOrRigidBodyModel) {
	const Quadrotor quad = minimalModel();
	ASSERT_TRUE(quad.validRatesThrust());
	ASSERT_FALSE(quad.valid());
	GeometricController controller(quad, gains());
	const Setpoint reference = hoverReference();
	SetpointVector output;
	ASSERT_TRUE(controller.getCommand(reference.state, {reference}, &output));
	ASSERT_EQ(output.size(), 1U);
	EXPECT_TRUE(output.front().input.isRatesThrust());
	EXPECT_NEAR(output.front().input.collective_thrust, G, 1e-9);
	EXPECT_NEAR(output.front().input.omega.norm(), 0.0, 1e-9);
	EXPECT_TRUE(output.front().state.tau.isZero());
	EXPECT_EQ(controller.horizonLength(), 1);
	EXPECT_DOUBLE_EQ(controller.dt(), 0.01);
}

TEST(GeometricController, PositionVelocityAndYawErrorsHaveCorrectDirections) {
	GeometricController controller(minimalModel(), gains());
	Setpoint reference = hoverReference();
	const QuadState state = reference.state;
	SetpointVector output;
	reference.state.p.x() = 0.1;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_GT(output.front().input.omega.y(), 0.0);
	reference = hoverReference();
	reference.state.p.y() = 0.1;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_LT(output.front().input.omega.x(), 0.0);
	reference = hoverReference();
	reference.state.v.x() = 0.1;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_GT(output.front().input.omega.y(), 0.0);
	reference = hoverReference();
	reference.state.p.z() = 0.1;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_GT(output.front().input.collective_thrust, G);
	reference = hoverReference();
	reference.state.q(0.1);
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_GT(output.front().input.omega.z(), 0.0);
}

TEST(GeometricController, FullModelPreservesLegacyIndiAngularAccelerationReference) {
	const Quadrotor full_model(2.0, 0.25);
	ASSERT_TRUE(full_model.valid());
	const auto params = gains();
	GeometricController controller(full_model, params);
	const Setpoint reference = hoverReference();
	QuadState moving = reference.state;
	moving.w = Vector<3>(0.1, -0.2, 0.3);
	SetpointVector output;
	ASSERT_TRUE(controller.getCommand(moving, {reference}, &output));
	const Vector<3> expected_acceleration = -params->kp_rate_.cwiseProduct(moving.w);
	EXPECT_TRUE(output.front().state.tau.isApprox(expected_acceleration, 1e-9));
	EXPECT_TRUE(output.front().input.omega.isZero(1e-9));
	GeometricController rates_only(minimalModel(), params);
	ASSERT_TRUE(rates_only.getCommand(moving, {reference}, &output));
	EXPECT_TRUE(output.front().state.tau.isZero());
	EXPECT_TRUE(output.front().input.omega.isZero(1e-9));
}

TEST(GeometricController, LimitsBodyratesAndCollectiveThrust) {
	Quadrotor quad = minimalModel();
	quad.omega_max_.setConstant(0.1);
	quad.thrust_min_ = 1.0;
	quad.thrust_max_ = 6.0;
	GeometricController controller(quad, gains());
	Setpoint reference = hoverReference();
	const QuadState state = reference.state;
	reference.state.p = Vector<3>(100.0, 100.0, 100.0);
	reference.state.q(1.0);
	SetpointVector output;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_TRUE((output.front().input.omega.cwiseAbs().array() <= quad.omega_max_.array()).all());
	EXPECT_NEAR(output.front().input.collective_thrust, quad.collective_thrust_max(), 1e-9);
	reference = hoverReference();
	reference.state.a.z() = -G + 0.5;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_NEAR(output.front().input.collective_thrust, quad.collective_thrust_min(), 1e-9);
}

TEST(GeometricController, LimitsDesiredTiltBeforeAttitudeControl) {
	Quadrotor quad = minimalModel();
	quad.omega_max_.setConstant(100.0);
	auto params = gains();
	params->max_tilt_rad_ = 0.3;
	GeometricController controller(quad, params);
	Setpoint reference = hoverReference();
	const QuadState state = reference.state;
	reference.state.p.x() = 100.0;
	SetpointVector output;
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	const Scalar expected_pitch_rate = 2.0 * params->kp_att_xy_ * std::sin(params->max_tilt_rad_ / 2.0);
	EXPECT_NEAR(output.front().input.omega.y(), expected_pitch_rate, 1e-9);
	EXPECT_NEAR(output.front().input.collective_thrust, G / std::cos(params->max_tilt_rad_), 1e-9);
}

TEST(GeometricController, InvalidInputClearsOutputAndEmptyReferenceIsSafe) {
	GeometricController controller(minimalModel(), gains());
	const Setpoint reference = hoverReference();
	SetpointVector output{reference};
	EXPECT_FALSE(controller.getCommand(reference.state, {}, &output));
	EXPECT_TRUE(output.empty());
	EXPECT_FALSE(controller.getCommand(reference.state, {reference}, nullptr));
	QuadState invalid = reference.state;
	invalid.p.x() = NAN;
	EXPECT_FALSE(controller.getCommand(invalid, {reference}, &output));
	invalid = reference.state;
	invalid.qx.setZero();
	EXPECT_FALSE(controller.getCommand(invalid, {reference}, &output));
	Setpoint invalid_reference = reference;
	invalid_reference.state.a.z() = NAN;
	EXPECT_FALSE(controller.getCommand(reference.state, {invalid_reference}, &output));
	invalid_reference = reference;
	invalid_reference.state.qx.setZero();
	EXPECT_FALSE(controller.getCommand(reference.state, {invalid_reference}, &output));
	invalid_reference = reference;
	invalid_reference.input.t = NAN;
	EXPECT_FALSE(controller.getCommand(reference.state, {invalid_reference}, &output));
	EXPECT_TRUE(output.empty());
}

TEST(GeometricController, DegenerateAccelerationAndInvertedAttitudeReject) {
	GeometricController controller(minimalModel(), gains());
	Setpoint reference = hoverReference();
	QuadState state = reference.state;
	SetpointVector output;
	reference.state.a = GVEC;
	EXPECT_FALSE(controller.getCommand(state, {reference}, &output));
	reference.state.a = 2.0 * GVEC;
	EXPECT_FALSE(controller.getCommand(state, {reference}, &output));
	reference = hoverReference();
	state.q(std::acos(-1.0), Vector<3>::UnitX());
	EXPECT_FALSE(controller.getCommand(state, {reference}, &output));
	EXPECT_TRUE(output.empty());
	state.q(std::acos(-1.0) - 1e-4, Vector<3>::UnitX());
	ASSERT_TRUE(controller.getCommand(state, {reference}, &output));
	EXPECT_TRUE(output.front().input.omega.allFinite());
	EXPECT_LE(output.front().input.omega.cwiseAbs().maxCoeff(), 2.0);
}

TEST(GeometricController, ParametersAndSamplePeriodMustBeFinite) {
	auto params = gains();
	params->max_tilt_rad_ = std::acos(-1.0) / 2.0;
	EXPECT_FALSE(params->valid());
	params->max_tilt_rad_ = 0.3;
	params->kp_att_xy_ = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(params->valid());
	params = gains();
	params->filter_sampling_frequency_ = std::numeric_limits<Scalar>::infinity();
	EXPECT_FALSE(params->valid());
	GeometricController custom_period(minimalModel(), gains(), 0.02);
	EXPECT_DOUBLE_EQ(custom_period.dt(), 0.02);
	const Setpoint reference = hoverReference();
	SetpointVector output;
	GeometricController invalid_period(minimalModel(), gains(), NAN);
	EXPECT_FALSE(invalid_period.getCommand(reference.state, {reference}, &output));
	GeometricController missing_params(minimalModel(), nullptr);
	EXPECT_FALSE(missing_params.getCommand(reference.state, {reference}, &output));
}

TEST(GeometricController, DragCompensationRequiresMeasuredMotorSpeedsAndModel) {
	auto params = gains();
	params->drag_compensation_ = true;
	const Setpoint reference = hoverReference();
	SetpointVector output;
	GeometricController missing_model(minimalModel(), params);
	EXPECT_FALSE(missing_model.getCommand(reference.state, {reference}, &output));
	Quadrotor quad = minimalModel();
	quad.thrust_map_ = Vector<3>(1e-6, 0.0, 0.0);
	GeometricController controller(quad, params);
	EXPECT_FALSE(controller.getCommand(reference.state, {reference}, &output));
	QuadState measured = reference.state;
	measured.mot.setConstant(1000.0);
	ASSERT_TRUE(controller.getCommand(measured, {reference}, &output));
	EXPECT_TRUE(output.front().input.isRatesThrust());
}

TEST(GeometricController, PipelineImuInterfaceFeedsDragObserver) {
	Quadrotor quad = minimalModel();
	quad.thrust_map_ = Vector<3>(G / 2e6, 0.0, 0.0);
	auto params = gains();
	params->drag_compensation_ = true;
	GeometricController controller(quad, params);
	ControllerBase* interface = &controller;
	interface->addImuSample(ImuSample(0.0, Vector<3>(1.0, 0.0, G), Vector<3>::Zero()));
	EXPECT_FALSE(controller.addImu(ImuSample()));
	Setpoint reference = hoverReference();
	reference.state.v.x() = 4.0;
	QuadState measured = reference.state;
	measured.mot.setConstant(1000.0);
	SetpointVector output;
	for (int i = 0; i < 100; ++i) ASSERT_TRUE(controller.getCommand(measured, {reference}, &output));
	EXPECT_LT(output.front().input.omega.y(), -0.1);
}

}  // namespace
}  // namespace agi
