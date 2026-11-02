#ifndef AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
#define AGILIB_REFERENCE_CAPTURED_REFERENCE_H_

#include <algorithm>
#include <limits>
#include <vector>

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
	bool goToPosition(const Vector<3>& position, double now, double max_velocity, double max_acceleration, double yaw_rate_max) {
		if (!position.allFinite() || !std::isfinite(now) || !std::isfinite(max_velocity) || max_velocity <= 0 ||
		    !std::isfinite(max_acceleration) || max_acceleration <= 0 || !std::isfinite(yaw_rate_max) || yaw_rate_max <= 0)
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
		const double distance = displacement.norm();
		if (!std::isfinite(distance)) return false;
		QuadState end = turn_end;
		end.p = position;
		std::vector<QuadState> boundaries{turn_end};
		if (distance <= 1e-9) {
			// Keep a valid rest-to-rest hold, including the requested endpoint.
			end.t += 1.5;
		} else {
			double velocity = std::min(max_velocity, std::sqrt(max_acceleration * distance / 1.875));
			double acceleration_time = 1.875 * velocity / max_acceleration;
			double cruise_time = std::max(0.0, distance / velocity - acceleration_time);
			// Suppress only roundoff at the triangular/trapezoidal boundary.
			if (cruise_time <= 32 * std::numeric_limits<double>::epsilon() * std::max(1.0, acceleration_time)) cruise_time = 0;
			const double scale = std::max(1.0, 1.5 / (2 * acceleration_time + cruise_time));
			acceleration_time *= scale;
			cruise_time *= scale;
			velocity /= scale;
			const Vector<3> direction = displacement / distance;
			QuadState acceleration_end = turn_end;
			acceleration_end.t += acceleration_time;
			acceleration_end.p += .5 * velocity * acceleration_time * direction;
			acceleration_end.v = velocity * direction;
			boundaries.push_back(acceleration_end);
			if (cruise_time > 0) {
				QuadState cruise_end = acceleration_end;
				cruise_end.t += cruise_time;
				cruise_end.p = position - .5 * velocity * acceleration_time * direction;
				boundaries.push_back(cruise_end);
			}
			end.t = boundaries.back().t + acceleration_time;
		}
		boundaries.push_back(end);
		std::vector<std::shared_ptr<MinSnapTrajectory>> trajectories;
		trajectories.reserve(boundaries.size() - 1);
		for (size_t i = 1; i < boundaries.size(); ++i) {
			if (!boundaries[i - 1].valid() || !boundaries[i].valid() || boundaries[i].t <= boundaries[i - 1].t) return false;
			auto trajectory = std::make_shared<MinSnapTrajectory>(boundaries[i - 1], boundaries[i]);
			if (!trajectory->valid()) return false;
			trajectories.push_back(std::move(trajectory));
		}
		_pause_yaw.reset();
		_online_turn = std::move(turn);
		_online_turn_end = turn_end.t;
		_online_heading = alignment;
		_online_trajectories = std::move(trajectories);
		_online_start = now;
		_online_end = end;
		_online_end.q(end.q() * alignment);
		return true;
	}
	// Candidate first: a failed brake must leave the currently valid reference intact.
	std::string pauseHover(double now, double deceleration, const Vector<3>& fence_min, const Vector<3>& fence_max) {
		if (pauseActive()) return {};
		if (!std::isfinite(now) || !std::isfinite(deceleration) || deceleration <= 0)
			return "brake_deceleration/time must be finite and positive";
		QuadState start = getSetpoint(start_state_, now).state;
		if (!start.valid()) return "current reference invalid";
		const bool moving = movingAt(now);
		// Undo the existing tilt * heading composition; projected attitude yaw is
		// not the flat heading when the reference is tilted.
		const Vector<3> thrust = start.a - GVEC;
		if (thrust.norm() <= 1e-3) return "reference thrust direction undefined";
		const Quaternion tilt = Quaternion::FromTwoVectors(Vector<3>::UnitZ(), thrust);
		const Quaternion heading = (tilt.conjugate() * start.q()).normalized();
		const double rate = start.w.z();
		// This designs duration, not a strict acceleration bound for nonzero a/j.
		const double duration = std::max(1.5, 1.875 * start.v.norm() / deceleration);
		start.t = 0;
		start.q(0.0);
		start.w.setZero();
		QuadState end = start.getHoverState();
		end.t = duration;
		end.p = start.p + .5 * duration * start.v + .1 * duration * duration * start.a +
		        (duration * duration * duration / 120) * start.j;
		if (!end.valid()) return "nonfinite brake endpoint";
		// Same endpoint fence as online goals; this is not a swept-path/obstacle check.
		if (!(end.p.array() >= fence_min.array()).all() || !(end.p.array() <= fence_max.array()).all())
			return "brake endpoint outside configured geofence";
		auto trajectory = std::make_shared<MinSnapTrajectory>(start, end);
		// Scalar relative yaw retains turns larger than pi without quaternion wrapping.
		auto yaw = std::make_shared<Polynomial<>>(5, Vector<3>(0, 0, 1));
		yaw->scale(0, duration);
		yaw->addConstraint(0, Vector<3>(0, rate, 0));
		yaw->addConstraint(duration, Vector<3>(.5 * rate * duration, 0, 0));
		if (!trajectory->valid() || !yaw->solve() || !yaw->solved()) return "brake polynomial solve failed";
		if (std::abs((*yaw)(0)) > 1e-6 || std::abs((*yaw)(0, 1) - rate) > 1e-6 ||
		    std::abs((*yaw)(duration)-.5 * rate * duration) > 1e-6 || std::abs((*yaw)(duration, 1)) > 1e-6)
			return "brake heading boundary mismatch";
		for (int i = 0; i <= 100; ++i) {
			const double time = duration * i / 100;
			const QuadState sample = trajectory->getState(time);
			if (!sample.valid() || !std::isfinite((*yaw)(time)) || !std::isfinite((*yaw)(time, 1)))
				return "brake polynomial sample invalid";
		}
		for (const auto& boundary : {start, end}) {
			const QuadState sample = trajectory->getState(boundary.t);
			if ((sample.p - boundary.p).norm() > 1e-6 || (sample.v - boundary.v).norm() > 1e-6 ||
			    (sample.a - boundary.a).norm() > 1e-6 || (sample.j - boundary.j).norm() > 1e-6)
				return "brake polynomial boundary mismatch";
		}
		_online_turn.reset();
		_online_turn_end = 0;
		_online_heading = heading;
		_online_trajectories = {std::move(trajectory)};
		_pause_yaw = std::move(yaw);
		_online_start = now;
		_online_end = end;
		_online_end.q(Quaternion(Eigen::AngleAxisd(.5 * rate * duration, Vector<3>::UnitZ())) * heading);
		// A fixed hover/settling reference keeps its exact position and heading.
		if (!moving) _online_end.t = 0;
		return {};
	}
	bool pauseActive() const { return static_cast<bool>(_pause_yaw); }
	Vector<3> onlineEndpoint() const { return _online_end.p; }
	void clearOnlineTarget() {
		_pause_yaw.reset();
		_online_turn.reset();
		_online_trajectories.clear();
	}
	bool hasOnlineTarget() const { return !_online_trajectories.empty(); }
	bool movingAt(double now) const {
		if (hasOnlineTarget()) return now - _online_start < _online_end.t;
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
		if (hasOnlineTarget()) {
			const double relative_time = std::max(0.0, t - _online_start);
			if (relative_time >= _online_end.t) return hover(_online_end, t);
			// PolynomialTrajectory uses max(state.t, t): both must share the relative clock.
			QuadState query = state;
			query.t = relative_time;
			auto trajectory = _online_turn;
			if (relative_time >= _online_turn_end) {
				trajectory = _online_trajectories.back();
				// Select anew for each query: an MPC horizon may precede the next current-time query.
				for (const auto& segment : _online_trajectories) {
					if (relative_time < segment->getEndTime()) {
						trajectory = segment;
						break;
					}
				}
			}
			Setpoint result = trajectory->getSetpoint(query, relative_time);
			if (_pause_yaw) {
				const Quaternion yaw(Eigen::AngleAxisd((*_pause_yaw)(relative_time), Vector<3>::UnitZ()));
				result.state.q(result.state.q() * yaw);
				result.state.w = yaw.conjugate() * result.state.w;
				result.state.w.z() = (*_pause_yaw)(relative_time, 1);
			}
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
	bool isHover() const override { return !_execute_trajectory && !hasOnlineTarget(); }

private:
	static Setpoint hover(const QuadState& state, double t) {
		Setpoint result;
		result.state = state.getHoverState();
		result.state.t = t;
		result.input = Command(t, G, Vector<3>::Zero());
		return result;
	}
	std::shared_ptr<Polynomial<>> _pause_yaw;
	std::shared_ptr<SampledTrajectory> _trajectory;
	std::shared_ptr<MinSnapTrajectory> _online_turn;
	std::vector<std::shared_ptr<MinSnapTrajectory>> _online_trajectories;
	double _online_turn_end{0};
	Quaternion _online_heading{Quaternion::Identity()};
	QuadState _online_end;
	double _online_start{NAN};
	Quaternion _alignment{Quaternion::Identity()};
	bool _execute_trajectory{false};
};
}  // namespace agi
#endif  // AGILIB_REFERENCE_CAPTURED_REFERENCE_H_
