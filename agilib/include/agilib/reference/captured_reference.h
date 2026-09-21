#ifndef AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
#define AGILIB_REFERENCE_CAPTURED_REFERENCE_H_

#include "agilib/math/gravity.hpp"
#include "agilib/reference/trajectory_reference/sampled_trajectory.hpp"

namespace agi {
// Single control-thread owner. A relative trajectory is loaded once; only the
// sampled prediction window is transformed when the vehicle takes control.
class CapturedReference final : public ReferenceBase {
public:
	CapturedReference() : ReferenceBase("Captured hover/trajectory") {}
	void setTrajectory(std::shared_ptr<SampledTrajectory> trajectory) { _trajectory = std::move(trajectory); }
	void capture(const QuadState& state, double now, bool execute_trajectory) {
		start_state_ = state.getHoverState();
		start_state_.t = now;
		duration_ = INF;
		_alignment = Quaternion(Eigen::AngleAxisd(state.getYaw(), Vector<3>::UnitZ()));
		_execute_trajectory = execute_trajectory && static_cast<bool>(_trajectory);
	}
	Setpoint getSetpoint(const QuadState& state, const Scalar t) override {
		if (!_execute_trajectory) return hover(start_state_, t);
		const double relative_time = t - start_state_.t;
		Setpoint result = _trajectory->getSetpoint(state, relative_time);
		result.state.p = start_state_.p + _alignment * result.state.p;
		result.state.v = _alignment * result.state.v;
		result.state.a = _alignment * result.state.a;
		result.state.j = _alignment * result.state.j;
		result.state.s = _alignment * result.state.s;
		result.state.q(_alignment * result.state.q());
		if (relative_time > _trajectory->getEndTime()) return hover(result.state, t);
		result.state.t = t;
		result.input.t = t;
		return result;
	}
	bool isHover() const override { return !_execute_trajectory; }

private:
	static Setpoint hover(const QuadState& state, double t) {
		Setpoint result;
		result.state = state.getHoverState();
		result.state.t = t;
		result.input = Command(t, G, Vector<3>::Zero());
		return result;
	}
	std::shared_ptr<SampledTrajectory> _trajectory;
	Quaternion _alignment{Quaternion::Identity()};
	bool _execute_trajectory{false};
};
}  // namespace agi
#endif  // AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
