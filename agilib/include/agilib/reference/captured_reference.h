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
	bool goToPosition(const Vector<3>& position, double now, double mean_velocity, double yaw_rate_max) {
		if (!position.allFinite() || !std::isfinite(now) || !std::isfinite(mean_velocity) || mean_velocity <= 0 ||
		    !std::isfinite(yaw_rate_max) || yaw_rate_max <= 0)
			return false;
		QuadState start = getSetpoint(start_state_, now).state.getHoverState();
		start.t = 0;
		const Vector<3> displacement = position - start.p;
		const double heading = start.getYaw();
		const double heading_delta = displacement.head<2>().norm() > 1e-6
		                                     ? std::remainder(std::atan2(displacement.y(), displacement.x()) - heading, 2 * M_PI)
		                                     : 0.0;
		// Solve yaw relative to the held heading to cross +/-pi along the shortest arc.
		// Right-compose this constant heading (and transform body rates) when sampling.
		const Quaternion alignment(Eigen::AngleAxisd(heading, Vector<3>::UnitZ()));
		start.q(0.0);
		QuadState turn_end = start;
		turn_end.q(heading_delta);
		std::shared_ptr<MinSnapTrajectory> turn;
		if (heading_delta != 0.0) {
			// Quintic rest-to-rest yaw has peak rate 1.875 * abs(delta) / duration.
			turn_end.t = std::max(1.875 * std::abs(heading_delta) / yaw_rate_max, 1.5);
			if (!start.valid() || !turn_end.valid()) return false;
			turn = std::make_shared<MinSnapTrajectory>(start, turn_end);
			if (!turn->valid()) return false;
		}
		QuadState end = turn_end;
		end.p = position;
		end.t += std::max(displacement.norm() / mean_velocity, 1.5);
		if (!start.valid() || !end.valid()) return false;
		auto trajectory = std::make_shared<MinSnapTrajectory>(turn_end, end);
		if (!trajectory->valid()) return false;
		_online_turn = std::move(turn);
		_online_turn_end = turn_end.t;
		_online_heading = alignment;
		_online_trajectory = std::move(trajectory);
		_online_start = now;
		_online_end = end;
		_online_end.q(end.q() * alignment);
		return true;
	}
	void clearOnlineTarget() {
		_online_turn.reset();
		_online_trajectory.reset();
	}
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
			const auto& trajectory = relative_time < _online_turn_end ? _online_turn : _online_trajectory;
			Setpoint result = trajectory->getSetpoint(query, relative_time);
			result.state.q(result.state.q() * _online_heading);
			result.state.w = _online_heading.conjugate() * result.state.w;
			result.input.omega = result.state.w;
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
	std::shared_ptr<MinSnapTrajectory> _online_turn;
	std::shared_ptr<MinSnapTrajectory> _online_trajectory;
	double _online_turn_end{0};
	Quaternion _online_heading{Quaternion::Identity()};
	QuadState _online_end;
	double _online_start{NAN};
	Quaternion _alignment{Quaternion::Identity()};
	bool _execute_trajectory{false};
};
}  // namespace agi
#endif  // AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
