#include "agilib/reference/captured_reference.h"

namespace agi {

bool CapturedReference::startSampledTrajectory(std::shared_ptr<SampledTrajectory> trajectory, double now, const Vector<3>& fence_min,
                                               const Vector<3>& fence_max) {
	if (!trajectory || !std::isfinite(now) || trajectory->getStartTime() != 0 || trajectory->getEndTime() <= 0) return false;
	const QuadState start = getSetpoint(start_state_, now).state.getHoverState();
	const Quaternion heading(Eigen::AngleAxisd(start.getYaw(), Vector<3>::UnitZ()));
	QuadState end = trajectory->getEndSetpoint().state;
	end.p = start.p + heading * end.p;
	end.q(heading * end.q());
	if (!start.valid() || !end.valid() || !(end.p.array() >= fence_min.array()).all() || !(end.p.array() <= fence_max.array()).all())
		return false;
	clearOnlineTarget();
	_online_origin = start.p;
	_online_heading = heading;
	_online_start = now;
	_online_end = end;
	_online_sampled = std::move(trajectory);
	return true;
}

std::pair<double, double> CapturedReference::tangentHeading(const MinSnapTrajectory& trajectory, double time, double fallback) {
	const double begin_time = trajectory.getStartTime(), end_time = trajectory.getEndTime();
	const Vector<> velocity = trajectory.evalTranslation(time, 1);
	const bool near_boundary = std::min(time - begin_time, end_time - time) < .001 * (end_time - begin_time);
	const bool resting_boundary = velocity.head<2>().squaredNorm() < 1e-14 && near_boundary;
	if (resting_boundary) time = time - begin_time < end_time - time ? begin_time : end_time;
	const bool end = time >= end_time;
	// At rest, use the first nonzero Taylor coefficient of horizontal velocity.
	// This also makes endpoint heading independent of MPC query order.
	for (int order = resting_boundary ? 4 : 1; order <= 11; ++order) {
		const Vector<> direction = trajectory.evalTranslation(time, order);
		const double square = direction.head<2>().squaredNorm();
		if (square < 1e-20) continue;
		const double sign = end && order % 2 == 0 ? -1 : 1;
		double rate = 0;
		if (order < 11) {
			const Vector<> next = trajectory.evalTranslation(time, order + 1);
			rate = (direction.x() * next.y() - direction.y() * next.x()) / (order * square);
		}
		return {std::atan2(sign * direction.y(), sign * direction.x()), rate};
	}
	return {fallback, 0};
}

double CapturedReference::derivativeBound(const MinSnapTrajectory& trajectory, int derivative) {
	// A polynomial stays in the convex hull of its Bernstein control points.
	// Bound vector norms, rather than relying on a sampling grid for v/a limits.
	const int degree = 11 - derivative;
	const double duration = trajectory.getEndTime() - trajectory.getStartTime();
	std::vector<Vector<3>> powers;
	double factor = 1;
	for (int k = 0; k <= degree; ++k) {
		powers.push_back(factor * trajectory.evalTranslation(trajectory.getStartTime(), derivative + k));
		factor *= duration / (k + 1);
	}
	double bound = 0;
	for (int i = 0; i <= degree; ++i) {
		Vector<3> point = Vector<3>::Zero();
		double ratio = 1;
		for (int k = 0; k <= i; ++k) {
			point += ratio * powers[k];
			if (k < degree) ratio *= static_cast<double>(i - k) / (degree - k);
		}
		bound = std::max(bound, point.norm());
	}
	return bound;
}

std::string CapturedReference::beginSmoothRoute(const std::vector<Vector<3>>& points, double now, double max_velocity,
                                                double max_acceleration, double yaw_rate_max) {
	if (points.empty() || !std::isfinite(now) || !std::isfinite(max_velocity) || max_velocity <= 0 ||
	    !std::isfinite(max_acceleration) || max_acceleration <= 0 || !std::isfinite(yaw_rate_max) || yaw_rate_max <= 0)
		return "invalid route/time/limits";
	auto plan = std::make_shared<SmoothPlan>();
	QuadState start = getSetpoint(start_state_, now).state.getHoverState();
	if (!start.valid()) return "current reference invalid";
	plan->heading = start.getYaw();
	start.q(0.0);
	start.t = 0;
	plan->boundaries.push_back(start);
	plan->max_velocity = max_velocity;
	plan->max_acceleration = max_acceleration;
	plan->yaw_rate_max = yaw_rate_max;
	for (const auto& point : points) {
		if (!point.allFinite()) return "nonfinite waypoint";
		const double distance = (point - plan->boundaries.back().p).norm();
		if (!std::isfinite(distance)) return "nonfinite waypoint distance";
		if (distance > 1e-9) {
			QuadState end = start;
			end.p = point;
			end.t = plan->boundaries.back().t +
			        std::max({1.5, 2 * distance / max_velocity, std::sqrt(8 * distance / max_acceleration)});
			plan->boundaries.push_back(end);
		}
		plan->waypoint_ends.push_back(plan->boundaries.size() - 1);
	}
	// An all-coincident route is a valid hold, not a zero-duration polynomial.
	if (plan->boundaries.size() == 1) {
		start.t = 1.5;
		plan->boundaries.push_back(start);
		for (auto& index : plan->waypoint_ends) index = 1;
	}
	for (size_t i = 1; i + 1 < plan->boundaries.size(); ++i) {
		auto& at = plan->boundaries[i];
		const auto& before = plan->boundaries[i - 1];
		const auto& after = plan->boundaries[i + 1];
		const Vector<3> incoming = at.p - before.p, outgoing = after.p - at.p;
		const Vector<3> direction = incoming.normalized() + outgoing.normalized();
		// Vertical transitions and exact reversals have no continuous horizontal heading.
		if (direction.norm() < 1e-6 || incoming.head<2>().norm() < 1e-6 || outgoing.head<2>().norm() < 1e-6) continue;
		const double speed = .5 * std::min(incoming.norm() / (at.t - before.t), outgoing.norm() / (after.t - at.t));
		at.v = speed * direction.normalized();
	}
	plan->arrivals.resize(plan->boundaries.size(), 0);
	_smooth_plan = std::move(plan);
	return {};
}

std::string CapturedReference::prepareSmoothRoute(double now, bool stable_hover) {
	if (!_smooth_plan) return "no route preparation";
	auto& plan = *_smooth_plan;
	if (!plan.assembling) {
		const size_t index = plan.motions.size();
		auto motion = std::make_shared<MinSnapTrajectory>(plan.boundaries[index], plan.boundaries[index + 1]);
		if (!motion->valid()) return "smooth polynomial solve failed";
		for (const auto& boundary : {plan.boundaries[index], plan.boundaries[index + 1]}) {
			const QuadState sample = motion->getState(boundary.t);
			if (!sample.valid() || (sample.p - boundary.p).norm() > 1e-6 || (sample.v - boundary.v).norm() > 1e-6 ||
			    (sample.a - boundary.a).norm() > 1e-6 || (sample.j - boundary.j).norm() > 1e-6)
				return "smooth polynomial boundary mismatch";
		}
		plan.scale = std::max({plan.scale, derivativeBound(*motion, 1) / plan.max_velocity,
		                       std::sqrt(derivativeBound(*motion, 2) / plan.max_acceleration)});
		for (int i = 0; i <= 256; ++i) {
			const double t = motion->getStartTime() + (motion->getEndTime() - motion->getStartTime()) * i / 256;
			const auto heading = tangentHeading(*motion, t, plan.heading);
			if (!std::isfinite(heading.first) || !std::isfinite(heading.second) || !motion->getState(t).valid())
				return "invalid smooth polynomial sample";
			// Reserve yaw-rate margin for rest-point turns and numerical extrema between samples.
			plan.scale = std::max(plan.scale, 4 * std::abs(heading.second) / plan.yaw_rate_max);
		}
		if (!std::isfinite(plan.scale)) return "invalid smooth time scale";
		plan.motions.push_back({motion, true, 0});
		if (plan.motions.size() + 1 == plan.boundaries.size()) {
			plan.scale *= 1.01;
			plan.assembling = true;
		}
		return {};
	}
	const bool finished = plan.next == plan.motions.size();
	double next_heading = plan.heading, next_rate = 0;
	if (!finished) {
		const auto& motion = *plan.motions[plan.next].trajectory;
		const auto heading = tangentHeading(motion, motion.getStartTime(), plan.heading);
		next_heading = heading.first;
		next_rate = heading.second / plan.scale;
	}
	const double delta = std::remainder(next_heading - plan.heading, 2 * M_PI);
	const bool at_rest = finished || plan.boundaries[plan.next].v.norm() < 1e-9;
	if (at_rest && (std::abs(delta) > 1e-8 || std::abs(next_rate - plan.rate) > 1e-8)) {
		QuadState start = plan.boundaries[finished ? plan.boundaries.size() - 1 : plan.next].getHoverState();
		start.t = plan.time;
		start.q(0.0);
		start.w.z() = plan.rate;
		QuadState end = start;
		end.t += std::max(1.5, 4 * std::abs(delta) / plan.yaw_rate_max);
		end.q(delta);
		end.w.z() = next_rate;
		auto turn = std::make_shared<MinSnapTrajectory>(start, end);
		if (!turn->valid()) return "smooth turn solve failed";
		plan.ready.push_back({turn, false, plan.heading});
		plan.heading = next_heading;
		plan.rate = next_rate;
		plan.time = end.t;
		return {};
	}
	if (!finished) {
		auto motion = plan.motions[plan.next];
		const double duration = (motion.trajectory->getEndTime() - motion.trajectory->getStartTime()) * plan.scale;
		motion.heading = next_heading;
		motion.trajectory->scale(plan.time, duration);
		plan.ready.push_back(motion);
		plan.time += duration;
		const auto end_heading = tangentHeading(*motion.trajectory, plan.time, next_heading);
		plan.heading = end_heading.first;
		plan.rate = end_heading.second;
		plan.arrivals[++plan.next] = plan.time;
		return {};
	}
	// Preparation spans control ticks: repeat the existing admission check at installation.
	if (!stable_hover) return "not stable at route installation";
	QuadState end = plan.boundaries.back().getHoverState();
	end.t = plan.time;
	end.q(plan.heading);
	if (!end.valid() || !std::isfinite(now)) return "invalid smooth endpoint/time";
	std::vector<double> arrivals;
	for (const size_t index : plan.waypoint_ends) arrivals.push_back(plan.arrivals[index]);
	_smooth_segments = std::move(plan.ready);
	_smooth_waypoint_times = std::move(arrivals);
	_online_start = now;
	_online_end = end;
	_pause_yaw.reset();
	_online_sampled.reset();
	_smooth_active = true;
	_smooth_plan.reset();
	return "ready";
}

size_t CapturedReference::smoothWaypointIndex(double now) const {
	if (_smooth_waypoint_times.empty()) return 0;
	const auto found = std::upper_bound(_smooth_waypoint_times.begin(), _smooth_waypoint_times.end(), now - _online_start);
	return std::min(static_cast<size_t>(found - _smooth_waypoint_times.begin()), _smooth_waypoint_times.size() - 1);
}

Setpoint CapturedReference::sampleSmooth(double time) const {
	const double relative = std::max(0.0, time - _online_start);
	if (relative >= _online_end.t) return hover(_online_end, time);
	const SmoothSegment* chosen = &_smooth_segments.back();
	for (const auto& segment : _smooth_segments) {
		if (relative < segment.trajectory->getEndTime()) {
			chosen = &segment;
			break;
		}
	}
	Setpoint result;
	result.state = chosen->trajectory->getState(relative);
	auto& state = result.state;
	if (chosen->tangent) {
		const auto heading = tangentHeading(*chosen->trajectory, relative, chosen->heading);
		const Quaternion yaw(Eigen::AngleAxisd(heading.first, Vector<3>::UnitZ()));
		const double thrust = (state.a - GVEC).norm();
		const Quaternion tilt = Quaternion::FromTwoVectors(Vector<3>::UnitZ(), state.a - GVEC);
		state.q(tilt * yaw);
		const Vector<3> jerk = state.q().conjugate() * state.j;
		state.w = Vector<3>(-jerk.y() / thrust, jerk.x() / thrust, heading.second);
	} else {
		const Quaternion heading(Eigen::AngleAxisd(chosen->heading, Vector<3>::UnitZ()));
		state.q(state.q() * heading);
		state.w = heading.conjugate() * state.w;
	}
	state.t = time;
	result.input = Command(time, (state.a - GVEC).norm(), state.w);
	return result;
}

}  // namespace agi
