#ifndef AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
#define AGILIB_REFERENCE_CAPTURED_REFERENCE_H_

#include <algorithm>

#include "agilib/math/gravity.hpp"
#include "agilib/reference/trajectory_reference/polynomial_trajectory.hpp"
#include "agilib/reference/trajectory_reference/sampled_trajectory.hpp"

namespace agi {
// Single control-thread owner. A relative trajectory is loaded once; only the
// sampled prediction window is transformed when the vehicle takes control.
class CapturedReference final : public ReferenceBase {
public:
	CapturedReference() : ReferenceBase("Captured hover/trajectory") {}
	void setTrajectory(std::shared_ptr<SampledTrajectory> trajectory) { _trajectory = std::move(trajectory); }
	// Online references are already in odom; the configured CSV keeps its capture transform.
	bool goToPosition(const Vector<3>& position, double now, double mean_velocity) {
		if (!position.allFinite() || !std::isfinite(now) || !std::isfinite(mean_velocity) || mean_velocity <= 0) return false;
		QuadState start = getSetpoint(start_state_, now).state.getHoverState();
		start.t = 0;
		QuadState end = start;
		end.p = position;
		end.t = std::max((end.p - start.p).norm() / mean_velocity, 1.5);
		if (!start.valid() || !end.valid()) return false;
		auto trajectory = std::make_shared<MinSnapTrajectory>(start, end);
		if (!trajectory->valid()) return false;
		_online_trajectory = std::move(trajectory);
		_online_start = now;
		_online_end = end;
		return true;
	}
	void clearOnlineTarget() { _online_trajectory.reset(); }
	bool hasOnlineTarget() const { return static_cast<bool>(_online_trajectory); }
	bool movingAt(double now) const {
		if (_online_trajectory) return now - _online_start < _online_end.t;
		return _execute_trajectory && now - start_state_.t < _trajectory->getEndTime();
	}
	void capture(const QuadState& state, double now, bool execute_trajectory) {
		clearOnlineTarget();
		start_state_ = state.getHoverState();
		start_state_.t = now;
		duration_ = INF;
		_alignment = Quaternion(Eigen::AngleAxisd(state.getYaw(), Vector<3>::UnitZ()));
		_execute_trajectory = execute_trajectory && static_cast<bool>(_trajectory);
	}
	Setpoint getSetpoint(const QuadState& state, const Scalar t) override {
		if (_online_trajectory) {
			const double relative_time = std::max(0.0, t - _online_start);
			if (relative_time >= _online_end.t) return hover(_online_end, t);
			// PolynomialTrajectory uses max(state.t, t): both must share the relative clock.
			QuadState query = state;
			query.t = relative_time;
			Setpoint result = _online_trajectory->getSetpoint(query, relative_time);
			result.state.t = result.input.t = t;
			return result;
		}
		if (!_execute_trajectory) return hover(start_state_, t);
		const double relative_time = t - start_state_.t;
		Setpoint result = _trajectory->getSetpoint(state, relative_time);
		result.state.p = start_state_.p + _alignment * result.state.p;
		result.state.v = _alignment * result.state.v;
		result.state.a = _alignment * result.state.a;
		result.state.j = _alignment * result.state.j;
		result.state.s = _alignment * result.state.s;
		result.state.q(_alignment * result.state.q());
		if (relative_time >= _trajectory->getEndTime()) return hover(result.state, t);
		result.state.t = t;
		result.input.t = t;
		return result;
	}
	bool isHover() const override { return !_execute_trajectory && !_online_trajectory; }

private:
	static Setpoint hover(const QuadState& state, double t) {
		Setpoint result;
		result.state = state.getHoverState();
		result.state.t = t;
		result.input = Command(t, G, Vector<3>::Zero());
		return result;
	}
	std::shared_ptr<SampledTrajectory> _trajectory;
	std::shared_ptr<MinSnapTrajectory> _online_trajectory;
	QuadState _online_end;
	double _online_start{NAN};
	Quaternion _alignment{Quaternion::Identity()};
	bool _execute_trajectory{false};
};
}  // namespace agi
#endif  // AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
