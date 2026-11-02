#include "agilib/controller/mpc/controller_mpc.hpp"

#include <gtest/gtest.h>

#include <limits>

#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/simulator/quadrotor_simulator.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/types/setpoint.hpp"

using namespace agi;

static constexpr Scalar M = 1.5;
static constexpr Scalar L = 0.25;

TEST(MPC, ConstructorTest) {
	Quadrotor quad(M, L);
	std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
	MpcController mpc(quad, params);
}

TEST(MPC, StaticTest) {
	Quadrotor quad(M, L);
	std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
	MpcController mpc(quad, params);

	QuadState hover_state;
	hover_state.setZero();
	Command hover_command;
	hover_command.t = 0.0;
	hover_command.thrusts.setConstant(M * G / 4.0);

	SetpointVector references(20, Setpoint(hover_state, hover_command));
	SetpointVector setpoints;

	for (int i = 0; i < 10; ++i) {
		ASSERT_TRUE(mpc.getCommand(hover_state, references, &setpoints));
		Command command = setpoints.front().input;
		EXPECT_TRUE(command.isRatesThrust());
		EXPECT_FALSE(command.isSingleRotorThrusts());
		EXPECT_NEAR(command.collective_thrust, G, 1e-3);
		EXPECT_LT(command.omega.norm(), 1e-6);
	}
	mpc.printTiming();
}

TEST(MPC, TakeOffTest) {
	Quadrotor quad(M, L);
	std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
	MpcController mpc(quad, params);

	QuadState initial_state;
	initial_state.setZero();
	QuadState start_state = initial_state;
	start_state.p.z() = 1.0;
	Command hover_command;
	hover_command.t = 0.0;
	hover_command.thrusts.setConstant(M * G / 4.0);

	SetpointVector references(20, Setpoint(start_state, hover_command));
	SetpointVector setpoints;

	QuadState state = initial_state;
	for (int i = 0; i < 20; ++i) {
		state.p.z() = i * 0.1;
		ASSERT_TRUE(mpc.getCommand(state, references, &setpoints));
		Command command = setpoints.front().input;
		if (i < 10)
			EXPECT_GT(command.collective_thrust, G);
		else if (i == 10)
			EXPECT_NEAR(command.collective_thrust, G, 1e-3);
		else
			EXPECT_LT(command.collective_thrust, G);
	}
	mpc.printTiming();
}

TEST(MPC, TakeOffTestCollectiveThrustRates) {
	Quadrotor quad(M, L);
	std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
	MpcController mpc(quad, params);

	QuadState initial_state;
	initial_state.setZero();
	QuadState start_state = initial_state;
	start_state.p.z() = 1.0;
	Command hover_command;
	hover_command.t = 0.0;
	hover_command.omega.setZero();
	hover_command.collective_thrust = G;

	SetpointVector references(20, Setpoint(start_state, hover_command));
	SetpointVector setpoints;

	QuadState state = initial_state;
	for (int i = 0; i < 20; ++i) {
		state.p.z() = i * 0.1;
		ASSERT_TRUE(mpc.getCommand(state, references, &setpoints));
		Command command = setpoints.front().input;
		if (i < 10)
			EXPECT_GT(command.collective_thrust, G);
		else if (i == 10)
			EXPECT_NEAR(command.collective_thrust, G, 1e-3);
		else
			EXPECT_LT(command.collective_thrust, G);
	}
	mpc.printTiming();
}

TEST(MPC, StepReferenceJump) {
	static constexpr Scalar dt = 0.01;
	static constexpr Scalar T = 8.0;
	Quadrotor quad(M, L);
	std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
	MpcController mpc(quad, params);

	// The constructor already installs motor, thrust/torque and rigid-body models.
	QuadrotorSimulator sim(quad);

	QuadState initial_state;
	initial_state.setZero();

	sim.reset(initial_state);

	QuadState reference_state;
	reference_state.t = 0.0;
	reference_state.setZero();
	reference_state.p = Vector<3>(1.0, 1.0, 1.0);
	reference_state.q(0.5 * M_PI);

	Command reference_command;
	reference_command.t = 0.0;
	reference_command.thrusts.setConstant(9.8066 * M / 4.0);

	Setpoint reference;
	reference.state = reference_state;
	reference.input = reference_command;
	SetpointVector references({reference});
	SetpointVector setpoints;
	QuadState state;
	Command command;
	Timer timer("MpcSim");
	for (Scalar t = 0.0; t <= T; t += dt) {
		timer.tic();
		EXPECT_TRUE(sim.getState(&state));
		ASSERT_TRUE(mpc.getCommand(state, references, &setpoints));
		Command command = setpoints.front().input;
		sim.setCommand(command);
		sim.run(dt);
		timer.toc();
	}

	std::cout << timer;

	// copy motors to ignore them
	state.mot = reference_state.mot;
	state.motdes = reference_state.motdes;

	EXPECT_LT((state.p - reference_state.p).norm(), 0.15);
	EXPECT_LT(state.v.norm(), 0.15);
	EXPECT_LT(state.q().angularDistance(reference_state.q()), 0.1);
	EXPECT_LT(state.w.norm(), 0.15);
}

TEST(MPC, MinimalModelAndIndependentMechanicalParameters) {
	Quadrotor full(M, L);
	Quadrotor minimal = full;
	minimal.J_.setConstant(NAN);
	minimal.J_inv_.setConstant(NAN);
	minimal.t_BM_.setConstant(NAN);
	minimal.kappa_ = NAN;
	minimal.motor_tau_inv_ = NAN;
	minimal.thrust_map_.setConstant(NAN);
	ASSERT_TRUE(minimal.validRatesThrust());
	ASSERT_FALSE(minimal.valid());
	auto params = std::make_shared<MpcParameters>();
	MpcController full_controller(full, params);
	MpcController minimal_controller(minimal, params);
	QuadState state;
	state.setZero();
	QuadState target = state;
	target.p << 1.0, -0.5, 1.0;
	SetpointVector references{Setpoint(target, Command(0.0, G, Vector<3>::Zero()))};
	SetpointVector full_prediction;
	SetpointVector minimal_prediction;
	ASSERT_TRUE(full_controller.getCommand(state, references, &full_prediction));
	ASSERT_TRUE(minimal_controller.getCommand(state, references, &minimal_prediction));
	EXPECT_NEAR(full_prediction.front().input.collective_thrust, minimal_prediction.front().input.collective_thrust, 1e-10);
	EXPECT_TRUE(full_prediction.front().input.omega.isApprox(minimal_prediction.front().input.omega, 1e-10));
}

TEST(MPC, EnforcesRateAndCollectiveBounds) {
	Quadrotor quad(M, L);
	quad.omega_max_ << 0.2, 0.3, 0.4;
	quad.thrust_min_ = M * 2.0 / 4.0;
	quad.thrust_max_ = M * 12.0 / 4.0;
	MpcController controller(quad, std::make_shared<MpcParameters>());
	QuadState state;
	state.setZero();
	QuadState target = state;
	target.p << 10, -10, 10;
	SetpointVector references{Setpoint(target, Command(0.0, 30.0, Vector<3>(4, -4, 4)))};
	SetpointVector prediction;
	for (int i = 0; i < 10; ++i) {
		ASSERT_TRUE(controller.getCommand(state, references, &prediction));
		for (const auto& point : prediction) {
			EXPECT_TRUE(point.state.valid());
			EXPECT_TRUE(point.input.isRatesThrust());
			EXPECT_FALSE(point.input.isSingleRotorThrusts());
			EXPECT_GE(point.input.collective_thrust, 2.0);
			EXPECT_LE(point.input.collective_thrust, 12.0);
			EXPECT_TRUE((point.input.omega.array().abs() <= quad.omega_max_.array()).all());
		}
	}
}

TEST(MPC, InvalidReferenceClearsPreviousOutput) {
	Quadrotor quad(M, L);
	MpcController controller(quad, std::make_shared<MpcParameters>());
	QuadState state;
	state.setZero();
	SetpointVector references{Setpoint(state, Command(0.0, G, Vector<3>::Zero()))};
	SetpointVector prediction;
	ASSERT_TRUE(controller.getCommand(state, references, &prediction));
	ASSERT_FALSE(prediction.empty());
	EXPECT_FALSE(controller.getCommand(state, {}, &prediction));
	EXPECT_TRUE(prediction.empty());
	references.front().state.qx.setZero();
	EXPECT_FALSE(controller.getCommand(state, references, &prediction));
	references.front().state = state;
	references.front().input.collective_thrust = -1.0;
	EXPECT_FALSE(controller.getCommand(state, references, &prediction));
	EXPECT_FALSE(controller.getCommand(state, references, nullptr));
}

namespace agi::acados {
#include "acados/ocp_nlp/ocp_nlp_cost_nls.h"
#include "blasfeo_d_aux.h"
}  // namespace agi::acados

namespace agi {
// Read the installed acados matrices, not a duplicate parameter-side cache.
class MpcRuntimeTestAccess {
public:
	static Matrix<acados::MpcWrapper::NY, acados::MpcWrapper::NY> stage(MpcController& controller, int index) {
		auto& wrapper = controller._wrapper;
		Matrix<acados::MpcWrapper::NY, acados::MpcWrapper::NY> result;
		auto* model = static_cast<acados::ocp_nlp_cost_nls_model*>(wrapper._nlp_in->cost[index]);
		acados::blasfeo_unpack_dmat(result.rows(), result.cols(), &model->W, 0, 0, result.data(), result.rows());
		return result;
	}
	static Matrix<acados::MpcWrapper::NYN, acados::MpcWrapper::NYN> terminal(MpcController& controller) {
		auto& wrapper = controller._wrapper;
		Matrix<acados::MpcWrapper::NYN, acados::MpcWrapper::NYN> result;
		auto* model = static_cast<acados::ocp_nlp_cost_nls_model*>(wrapper._nlp_in->cost[acados::MpcWrapper::N]);
		acados::blasfeo_unpack_dmat(result.rows(), result.cols(), &model->W, 0, 0, result.data(), result.rows());
		return result;
	}
};
}  // namespace agi

TEST(MPC, RuntimeWeightsAreInstalledAtomicallyAndUsedByNextSolve) {
	using W = acados::MpcWrapper;
	MpcController controller(Quadrotor(M, L), std::make_shared<MpcParameters>());
	QuadState state;
	state.setZero();
	QuadState target = state;
	target.p << .2, -.1, .3;
	SetpointVector reference{Setpoint(target, Command(0.0, G, Vector<3>::Zero()))}, prediction;
	for (int i = 0; i < 5; ++i) ASSERT_TRUE(controller.getCommand(state, reference, &prediction));
	const auto before = prediction.front().input;
	auto candidate = controller.getParameters();
	candidate->Q_pos_ *= 4;
	candidate->Q_vel_.setConstant(5);
	candidate->R_ << 1, 100, 110, 120;
	candidate->exp_decay_ = 1.01;
	EXPECT_NE(controller.getParameters()->Q_pos_.x(), candidate->Q_pos_.x());
	ASSERT_TRUE(controller.updateParameters(candidate));
	Vector<W::NY> diagonal;
	diagonal << candidate->Q_pos_, candidate->Q_att_, candidate->Q_vel_, candidate->R_;
	const Matrix<W::NY, W::NY> weights = diagonal.asDiagonal();
	for (int i = 0; i < W::N; ++i) EXPECT_TRUE(MpcRuntimeTestAccess::stage(controller, i).isApprox(std::pow(1.01, i) * weights, 1e-12));
	const Matrix<W::NYN, W::NYN> terminal = std::pow(1.01, W::N) * weights.topLeftCorner<W::NYN, W::NYN>();
	EXPECT_TRUE(MpcRuntimeTestAccess::terminal(controller).isApprox(terminal, 1e-12));
	for (int i = 0; i < 5; ++i) ASSERT_TRUE(controller.getCommand(state, reference, &prediction));
	EXPECT_GT(std::abs(before.collective_thrust - prediction.front().input.collective_thrust) +
	                  (before.omega - prediction.front().input.omega).norm(),
	          .01);
	for (const double invalid : {-1., double(NAN), double(INFINITY), 1e100}) {
		auto rejected = controller.getParameters();
		rejected->exp_decay_ = invalid;
		EXPECT_FALSE(controller.updateParameters(rejected));
		EXPECT_DOUBLE_EQ(controller.getParameters()->exp_decay_, 1.01);
		for (int i = 0; i < W::N; ++i)
			EXPECT_TRUE(MpcRuntimeTestAccess::stage(controller, i).isApprox(std::pow(1.01, i) * weights, 1e-12));
		EXPECT_TRUE(MpcRuntimeTestAccess::terminal(controller).isApprox(terminal, 1e-12));
	}
	// Finite gamma and finite stage costs can still overflow only at the terminal.
	auto terminal_overflow = controller.getParameters();
	terminal_overflow->Q_pos_.x() = std::numeric_limits<double>::max() / std::pow(1.01, W::N - .5);
	ASSERT_TRUE(std::isfinite(terminal_overflow->Q_pos_.x() * std::pow(1.01, W::N - 1)));
	EXPECT_FALSE(controller.updateParameters(terminal_overflow));
	for (int i = 0; i < W::N; ++i) EXPECT_TRUE(MpcRuntimeTestAccess::stage(controller, i).isApprox(std::pow(1.01, i) * weights, 1e-12));
	EXPECT_TRUE(MpcRuntimeTestAccess::terminal(controller).isApprox(terminal, 1e-12));
	auto invalid_weight = controller.getParameters();
	invalid_weight->R_[2] = -1;
	EXPECT_FALSE(controller.updateParameters(invalid_weight));
	candidate->Q_pos_.setZero();
	candidate->Q_att_.setZero();
	candidate->Q_vel_.setZero();
	candidate->R_.setZero();
	candidate->exp_decay_ = .95;
	ASSERT_TRUE(controller.updateParameters(candidate));
	for (int i = 0; i < W::N; ++i) EXPECT_TRUE(MpcRuntimeTestAccess::stage(controller, i).isZero());
	EXPECT_TRUE(MpcRuntimeTestAccess::terminal(controller).isZero());
}
