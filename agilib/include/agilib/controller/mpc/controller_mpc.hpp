#pragma once

#include <mutex>

#include "agilib/controller/controller_base.hpp"
#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/controller/mpc/wrapper.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

// Kinematic outer-loop MPC: the flight controller closes the body-rate loop.
// Inputs are [collective acceleration (m/s^2), body rates (rad/s)], not rotor forces.
class MpcController : public ControllerBase {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	MpcController(const Quadrotor& quad, const std::shared_ptr<MpcParameters>& params, const Scalar exec_dt = 0.01);

	bool getCommand(const QuadState& state, const SetpointVector& reference, SetpointVector* const setpoints) override;
	bool reset();
	bool reset(const QuadState& state);
	bool updateParameters(const Quadrotor& quad, const std::shared_ptr<MpcParameters>& params);
	bool updateParameters(const Quadrotor& quad);
	bool updateParameters(const std::shared_ptr<MpcParameters>& params);
	std::shared_ptr<MpcParameters> getParameters();
	void printTiming() const;
	void logTiming() const override;

private:
	using Wrapper = acados::MpcWrapper;
	bool reset(const Vector<Wrapper::NX>& state);

	Quadrotor _quad;
	std::shared_ptr<MpcParameters> _params;
	Wrapper _wrapper;
	std::mutex _acados_mutex;
	bool _reset_pending = false;
	Timer _timing_solver{"Solver"};
	Timer _timing_update{"Update"};
	const Vector<Wrapper::NX> _hover_state =
	        (Vector<Wrapper::NX>() << Vector<3>::Zero(), Vector<4>(1, 0, 0, 0), Vector<3>::Zero()).finished();
	const Vector<Wrapper::NU> _hover_input = (Vector<Wrapper::NU>() << G, 0, 0, 0).finished();
};

}  // namespace agi
