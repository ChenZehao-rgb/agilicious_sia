#pragma once

#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi::acados {

#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "agilib/controller/mpc/acados/acados_solver_drone_model.h"

// Ideal-inner-loop MPC: x=[p, q_wxyz, v], u=[T/m, omega_FLU].
// Inner-loop tracking delay and motor dynamics are deliberately not modeled.
class MpcWrapper {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MpcWrapper();
	~MpcWrapper();
	MpcWrapper(const MpcWrapper&) = delete;
	MpcWrapper& operator=(const MpcWrapper&) = delete;

	static constexpr int NX = DRONE_MODEL_NX;
	static constexpr int NZ = DRONE_MODEL_NZ;
	static constexpr int NU = DRONE_MODEL_NU;
	static constexpr int NP = DRONE_MODEL_NP;
	static constexpr int NBX = DRONE_MODEL_NBX;
	static constexpr int NBX0 = DRONE_MODEL_NBX0;
	static constexpr int NBU = DRONE_MODEL_NBU;
	static constexpr int NY0 = DRONE_MODEL_NY0;
	static constexpr int NY = DRONE_MODEL_NY;
	static constexpr int NYN = DRONE_MODEL_NYN;
	static constexpr int N = DRONE_MODEL_N;
	static constexpr int NQ = NY - NU;
	static_assert(NX == 10 && NU == 4 && NP == 4 && NY == 13 && NYN == 9 && NBX == 0 && NBU == 4,
	              "Regenerate the collective-thrust/body-rate acados solver before building");

	enum INPUTCONSTR : int { COLLECTIVE = 0, OMEGA_X = 1, OMEGA_Y = 2, OMEGA_Z = 3 };
	enum STATEIDX : int { STATEPOS = 0, STATEATT = 3, STATEVEL = 7 };
	enum REFIDX : int { REFPOS = 0, REFATT = 3, REFVEL = 6, REFU = 9 };

	void setInitialState(const Vector<NX>& state);
	int update(const Vector<NX>& state);
	void setBodyRateConstraints(const Vector<3>& lower_bound, const Vector<3>& upper_bound);
	void setThrustConstraints(Scalar lower_bound, Scalar upper_bound);

	Vector<NX> getState(int i) const { return _states_pred.col(i); }
	Matrix<NX, N + 1> getStates() const { return _states_pred; }
	Vector<NU> getInput(int i) const { return _inputs_pred.col(i); }
	Matrix<NU, N> getInputs() const { return _inputs_pred; }

	void setReference(const Vector<NX>& state_ref, const Vector<NU>& input_ref, int i);
	void setReferences(const Matrix<NX, N>& state_refs, const Matrix<NU, N>& input_refs);
	void setReferenceN(const Vector<NX>& state_ref);
	void setCosts(const Matrix<NY, NY * N>& weights, Scalar gamma = 1.0);
	void setCost(const Matrix<NY, NY>& weight, int i);
	void setCostN(const Matrix<NYN, NYN>& weight, Scalar gamma = 1.0);

	// Reset the actual solver iterate as well as the exposed prediction cache.
	void setStatesPred(const Matrix<NX, N + 1>& states);
	void setInputsPred(const Matrix<NU, N>& inputs);
	Scalar getDt() const { return _dt; }

private:
	void updateInputConstraints();
	void updateOnlineParams();
	void setQuaternionReference(const Vector<4>& attitude, int i);

	drone_model_solver_capsule* _capsule = nullptr;
	ocp_nlp_config* _nlp_config = nullptr;
	ocp_nlp_dims* _nlp_dims = nullptr;
	ocp_nlp_in* _nlp_in = nullptr;
	ocp_nlp_out* _nlp_out = nullptr;
	void* _nlp_opts = nullptr;
	Matrix<NX, N + 1> _states_pred;
	Matrix<NU, N> _inputs_pred;
	Matrix<NP, N + 1> _online_params;
	Vector<NBU> _lower_input_bounds;
	Vector<NBU> _upper_input_bounds;
	bool _constraints_dirty = true;
	Scalar _dt = 0.05;
	Logger _logger;
};

}  // namespace agi::acados
