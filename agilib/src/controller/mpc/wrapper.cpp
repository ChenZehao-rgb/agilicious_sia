#include "agilib/controller/mpc/wrapper.hpp"

#include <cmath>
#include <stdexcept>

#include "agilib/math/gravity.hpp"

namespace agi::acados {

MpcWrapper::MpcWrapper()
        : _online_params(Vector<4>(1.0, 0.0, 0.0, 0.0).replicate(1, N + 1)),
          _lower_input_bounds(0.0, -10.0, -10.0, -4.0),
          _upper_input_bounds(30.0, 10.0, 10.0, 4.0),
          _logger("MPC wrapper") {
	_capsule = drone_model_acados_create_capsule();
	if (!_capsule) throw std::runtime_error("Could not allocate acados capsule");
	const int status = drone_model_acados_create(_capsule);
	if (status != ACADOS_SUCCESS) {
		drone_model_acados_free(_capsule);
		drone_model_acados_free_capsule(_capsule);
		_capsule = nullptr;
		throw std::runtime_error("Could not create collective-thrust/body-rate acados solver");
	}
	_nlp_config = drone_model_acados_get_nlp_config(_capsule);
	_nlp_dims = drone_model_acados_get_nlp_dims(_capsule);
	_nlp_in = drone_model_acados_get_nlp_in(_capsule);
	_nlp_out = drone_model_acados_get_nlp_out(_capsule);
	_nlp_opts = drone_model_acados_get_nlp_opts(_capsule);
	_dt = _nlp_in->Ts[0];
	Vector<NX> initial_state = Vector<NX>::Zero();
	initial_state(STATEATT) = 1.0;
	setStatesPred(initial_state.replicate(1, N + 1));
	setInputsPred(Vector<NU>(G, 0.0, 0.0, 0.0).replicate(1, N));
}

MpcWrapper::~MpcWrapper() {
	if (!_capsule) return;
	const int free_status = drone_model_acados_free(_capsule);
	if (free_status) _logger.info("acados free returned %d", free_status);
	const int capsule_status = drone_model_acados_free_capsule(_capsule);
	if (capsule_status) _logger.info("acados capsule free returned %d", capsule_status);
}

void MpcWrapper::setInitialState(const Vector<NX>& state) {
	if (!state.allFinite()) throw std::invalid_argument("MPC initial state must be finite");
	Vector<NX> initial_state = state;
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "lbx", initial_state.data());
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "ubx", initial_state.data());
}

void MpcWrapper::setBodyRateConstraints(const Vector<3>& lower_bound, const Vector<3>& upper_bound) {
	if (!lower_bound.allFinite() || !upper_bound.allFinite() || (lower_bound.array() > upper_bound.array()).any()) {
		throw std::invalid_argument("MPC body-rate bounds must be finite and ordered");
	}
	_lower_input_bounds.tail<3>() = lower_bound;
	_upper_input_bounds.tail<3>() = upper_bound;
	_constraints_dirty = true;
}

void MpcWrapper::setThrustConstraints(const Scalar lower_bound, const Scalar upper_bound) {
	if (!std::isfinite(lower_bound) || !std::isfinite(upper_bound) || lower_bound < 0.0 || upper_bound <= lower_bound) {
		throw std::invalid_argument("MPC collective-acceleration bounds must be finite, nonnegative and ordered");
	}
	_lower_input_bounds(COLLECTIVE) = lower_bound;
	_upper_input_bounds(COLLECTIVE) = upper_bound;
	_constraints_dirty = true;
}

void MpcWrapper::updateInputConstraints() {
	if (!_constraints_dirty) return;
	for (int i = 0; i < N; ++i) {
		ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, i, "lbu", _lower_input_bounds.data());
		ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, i, "ubu", _upper_input_bounds.data());
	}
	_constraints_dirty = false;
}

int MpcWrapper::update(const Vector<NX>& state) {
	if (!state.allFinite()) return ACADOS_FAILURE;
	updateOnlineParams();
	updateInputConstraints();
	int rti_phase = 0;
	ocp_nlp_solver_opts_set(_nlp_config, _nlp_opts, "rti_phase", &rti_phase);

	// Align equivalent q/-q measurements with the actual solver iterate before RTI linearization.
	Vector<NX> warm_start;
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 0, "x", warm_start.data());
	Vector<NX> aligned_state = state;
	if (warm_start.segment<4>(STATEATT).allFinite() && warm_start.segment<4>(STATEATT).dot(state.segment<4>(STATEATT)) < 0.0) {
		aligned_state.segment<4>(STATEATT) *= -1.0;
	}
	setInitialState(aligned_state);
	const int status = drone_model_acados_solve(_capsule);
	for (int i = 0; i <= N; ++i) {
		ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, i, "x", _states_pred.col(i).data());
		if (i < N) ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, i, "u", _inputs_pred.col(i).data());
	}
	if (!_states_pred.allFinite() || !_inputs_pred.allFinite()) return ACADOS_FAILURE;
	return status;
}

void MpcWrapper::setReferences(const Matrix<NX, N>& state_refs, const Matrix<NU, N>& input_refs) {
	for (int i = 0; i < N; ++i) setReference(state_refs.col(i), input_refs.col(i), i);
}

void MpcWrapper::setReference(const Vector<NX>& state_ref, const Vector<NU>& input_ref, const int i) {
	if (i < 0 || i >= N || !state_ref.allFinite() || !input_ref.allFinite()) {
		throw std::invalid_argument("Invalid MPC stage reference");
	}
	Vector<NY> reference = Vector<NY>::Zero();
	reference.segment<3>(REFPOS) = state_ref.segment<3>(STATEPOS);
	reference.segment<3>(REFVEL) = state_ref.segment<3>(STATEVEL);
	reference.tail<NU>() = input_ref;
	setQuaternionReference(state_ref.segment<4>(STATEATT), i);
	ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, i, "yref", reference.data());
}

void MpcWrapper::setReferenceN(const Vector<NX>& state_ref) {
	if (!state_ref.allFinite()) throw std::invalid_argument("Invalid MPC terminal reference");
	Vector<NYN> reference = Vector<NYN>::Zero();
	reference.segment<3>(REFPOS) = state_ref.segment<3>(STATEPOS);
	reference.segment<3>(REFVEL) = state_ref.segment<3>(STATEVEL);
	setQuaternionReference(state_ref.segment<4>(STATEATT), N);
	ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "yref", reference.data());
}

void MpcWrapper::setCosts(const Matrix<NY, NY * N>& weights, const Scalar gamma) {
	for (int i = 0; i < N; ++i) setCost(std::pow(gamma, i) * weights.block<NY, NY>(0, i * NY), i);
}

void MpcWrapper::setCost(const Matrix<NY, NY>& weight, const int i) {
	if (i < 0 || i >= N || !weight.allFinite()) throw std::invalid_argument("Invalid MPC stage cost");
	Matrix<NY, NY> mutable_weight = weight;
	ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, i, "W", mutable_weight.data());
}

void MpcWrapper::setCostN(const Matrix<NYN, NYN>& weight, const Scalar gamma) {
	Matrix<NYN, NYN> mutable_weight = weight * std::pow(gamma, N);
	if (!mutable_weight.allFinite()) throw std::invalid_argument("Invalid MPC terminal cost");
	ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "W", mutable_weight.data());
}

void MpcWrapper::setQuaternionReference(const Vector<4>& attitude, const int i) { _online_params.col(i) = attitude; }

void MpcWrapper::updateOnlineParams() {
	for (int i = 0; i <= N; ++i) drone_model_acados_update_params(_capsule, i, _online_params.col(i).data(), NP);
}

void MpcWrapper::setStatesPred(const Matrix<NX, N + 1>& states) {
	if (!states.allFinite()) throw std::invalid_argument("Invalid MPC state warm start");
	_states_pred = states;
	for (int i = 0; i <= N; ++i) ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, i, "x", _states_pred.col(i).data());
}

void MpcWrapper::setInputsPred(const Matrix<NU, N>& inputs) {
	if (!inputs.allFinite()) throw std::invalid_argument("Invalid MPC input warm start");
	_inputs_pred = inputs;
	for (int i = 0; i < N; ++i) ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, i, "u", _inputs_pred.col(i).data());
}

}  // namespace agi::acados
