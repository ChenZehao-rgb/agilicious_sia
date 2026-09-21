#include "agilib/controller/mpc/controller_mpc.hpp"

#include <algorithm>
#include <stdexcept>

namespace agi {

MpcController::MpcController(const Quadrotor& quad, const std::shared_ptr<MpcParameters>& params, const Scalar exec_dt)
        : ControllerBase("MPC", exec_dt), _quad(quad), _params(params) {
	if (!updateParameters(quad, params)) throw std::invalid_argument("Invalid rates/thrust MPC configuration");
	pred_dt_ = _wrapper.getDt();
	horizon_length_ = Wrapper::N + 1;
	logger_.advertisePublishingVariable("timing_update_ms");
	if (!reset()) throw std::runtime_error("Rates/thrust MPC initial solve failed");
}

bool MpcController::getCommand(const QuadState& state, const SetpointVector& reference, SetpointVector* const setpoints) {
	if (setpoints == nullptr) return false;
	setpoints->clear();
	if (!state.valid() || reference.empty() || std::abs(state.qx.norm() - 1.0) > 0.01) return false;

	std::lock_guard<std::mutex> guard(_acados_mutex);
	_timing_update.tic();
	Matrix<Wrapper::NX, Wrapper::N + 1> reference_states;
	Matrix<Wrapper::NU, Wrapper::N> reference_inputs;
	for (int i = 0; i <= Wrapper::N; ++i) {
		const Setpoint& setpoint = reference[std::min<std::size_t>(i, reference.size() - 1)];
		if (!setpoint.state.valid() || std::abs(setpoint.state.qx.norm() - 1.0) > 0.01 || !setpoint.input.valid()) {
			_timing_update.toc();
			return false;
		}
		reference_states.col(i) = setpoint.state.x.head<Wrapper::NX>();
		reference_states.col(i).segment<4>(Wrapper::STATEATT).normalize();
		if (i == Wrapper::N) continue;
		if (setpoint.input.isRatesThrust()) {
			reference_inputs(0, i) = setpoint.input.collective_thrust;
			reference_inputs.col(i).tail<3>() = setpoint.input.omega;
		} else {
			// Legacy rotor-force references belong to this quadrotor. CSV loading
			// already converts source-vehicle forces to mass-normalized commands.
			reference_inputs(0, i) = setpoint.input.thrusts.sum() / _quad.m_;
			reference_inputs.col(i).tail<3>() = setpoint.state.w;
		}
		if (reference_inputs(0, i) < 0.0 || !reference_inputs.col(i).allFinite()) {
			_timing_update.toc();
			return false;
		}
	}

	_wrapper.setReferences(reference_states.leftCols<Wrapper::N>(), reference_inputs);
	_wrapper.setReferenceN(reference_states.col(Wrapper::N));
	Vector<Wrapper::NX> initial_state = state.x.head<Wrapper::NX>();
	initial_state.segment<4>(Wrapper::STATEATT).normalize();
	if (_reset_pending) {
		// A failed iterate must not poison subsequent warmup. Recovery still has
		// to pass the normal health, 50-cycle warmup and fresh AUTO-edge checks.
		_wrapper.setStatesPred(initial_state.replicate(1, Wrapper::N + 1));
		_wrapper.setInputsPred(_hover_input.replicate(1, Wrapper::N));
		_reset_pending = false;
	}
	_timing_solver.tic();
	const int status = _wrapper.update(initial_state);
	_timing_solver.toc();
	if (status != acados::ACADOS_SUCCESS || !_wrapper.getStates().allFinite() || !_wrapper.getInputs().allFinite()) {
		_reset_pending = true;
		logger_.error("Rates/thrust MPC solve failed (acados status %d)", status);
		_timing_update.toc();
		return false;
	}

	SetpointVector prediction;
	prediction.reserve(Wrapper::N + 1);
	for (int i = 0; i <= Wrapper::N; ++i) {
		const Vector<Wrapper::NU> input = _wrapper.getInput(std::min(i, Wrapper::N - 1));
		// Reject material constraint violations; clamp only numerical solver tolerance.
		const Scalar minimum = 4.0 * _quad.thrust_min_ / _quad.m_;
		const Scalar maximum = 4.0 * _quad.thrust_max_ / _quad.m_;
		if (input(0) < minimum - 1e-5 || input(0) > maximum + 1e-5 ||
		    (input.tail<3>().array().abs() > _quad.omega_max_.array() + 1e-5).any()) {
			_reset_pending = true;
			_timing_update.toc();
			return false;
		}
		Setpoint setpoint;
		setpoint.state.setZero();
		setpoint.state.t = state.t + i * pred_dt_;
		setpoint.state.x.head<Wrapper::NX>() = _wrapper.getState(i);
		if (setpoint.state.qx.norm() < 1e-6) {
			_reset_pending = true;
			_timing_update.toc();
			return false;
		}
		setpoint.state.qx.normalize();
		setpoint.input = Command(setpoint.state.t, std::clamp(input(0), minimum, maximum), _quad.clampBodyrates(input.tail<3>()));
		// No single-rotor command is fabricated: the flight controller owns allocation.
		setpoint.state.w = setpoint.input.omega;
		setpoint.state.a = setpoint.state.R().col(2) * setpoint.input.collective_thrust + GVEC;
		if (i + 1 < Wrapper::N) {
			setpoint.state.tau = (_wrapper.getInput(i + 1).tail<3>() - input.tail<3>()) / pred_dt_;
		}
		prediction.push_back(setpoint);
	}
	setpoints->swap(prediction);
	_timing_update.toc();
	return true;
}

bool MpcController::reset() { return reset(_hover_state); }

bool MpcController::reset(const QuadState& state) {
	if (!state.valid() || std::abs(state.qx.norm() - 1.0) > 0.01) return false;
	Vector<Wrapper::NX> initial_state = state.x.head<Wrapper::NX>();
	initial_state.segment<4>(Wrapper::STATEATT).normalize();
	return reset(initial_state);
}

bool MpcController::reset(const Vector<Wrapper::NX>& state) {
	std::lock_guard<std::mutex> guard(_acados_mutex);
	_wrapper.setInitialState(state);
	_wrapper.setReferences(state.replicate(1, Wrapper::N), _hover_input.replicate(1, Wrapper::N));
	_wrapper.setReferenceN(state);
	_wrapper.setStatesPred(state.replicate(1, Wrapper::N + 1));
	_wrapper.setInputsPred(_hover_input.replicate(1, Wrapper::N));
	_reset_pending = _wrapper.update(state) != acados::ACADOS_SUCCESS;
	return !_reset_pending;
}

void MpcController::logTiming() const {
	logger_.addPublishingVariable("timing_update_ms",
	                              1000.0 * Vector<3>(_timing_update.mean(), _timing_update.min(), _timing_update.max()));
}

void MpcController::printTiming() const {
	logger_ << _timing_update;
	logger_ << _timing_solver;
}

bool MpcController::updateParameters(const Quadrotor& quad, const std::shared_ptr<MpcParameters>& params) {
	if (!quad.validRatesThrust() || !params || !params->valid()) return false;
	return updateParameters(quad) && updateParameters(params);
}

bool MpcController::updateParameters(const Quadrotor& quad) {
	if (!quad.validRatesThrust()) return false;
	std::lock_guard<std::mutex> guard(_acados_mutex);
	_quad = quad;
	_wrapper.setBodyRateConstraints(-quad.omega_max_, quad.omega_max_);
	_wrapper.setThrustConstraints(4.0 * quad.thrust_min_ / quad.m_, 4.0 * quad.thrust_max_ / quad.m_);
	return true;
}

bool MpcController::updateParameters(const std::shared_ptr<MpcParameters>& params) {
	if (!params || !params->valid()) return false;
	std::lock_guard<std::mutex> guard(_acados_mutex);
	_params = params;
	const Matrix<Wrapper::NQ, Wrapper::NQ> Q =
	        (Vector<Wrapper::NQ>() << params->Q_pos_, params->Q_att_, params->Q_vel_).finished().asDiagonal();
	Matrix<Wrapper::NY, Wrapper::NY> W = Matrix<Wrapper::NY, Wrapper::NY>::Zero();
	W.topLeftCorner<Wrapper::NQ, Wrapper::NQ>() = Q;
	W.bottomRightCorner<Wrapper::NU, Wrapper::NU>() = params->R_.asDiagonal();
	_wrapper.setCosts(W.replicate(1, Wrapper::N), params->exp_decay_);
	_wrapper.setCostN(Q, params->exp_decay_);
	return true;
}

std::shared_ptr<MpcParameters> MpcController::getParameters() {
	std::lock_guard<std::mutex> guard(_acados_mutex);
	return _params;
}

}  // namespace agi
