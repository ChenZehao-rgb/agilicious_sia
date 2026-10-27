#include "agi_ros2/fc_state_adapter_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "agi_ros2/fc_state_evidence.h"
#include "agi_ros2/node_common.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/math/types.hpp"
namespace agi_ros2 {
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
FcStateAdapterNode::FcStateAdapterNode() : Node("fc_state_adapter"), _clock_id(readClockId()) {
	if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("FC adapter requires host time");
	_fused_pub = create_publisher<msg::FusedState>("fused_state", 1);
	_odom_pub = create_publisher<nav_msgs::msg::Odometry>("state", 1);
	_state_sub = create_subscription<msg::FcEstimatedState>("sensors/fc/estimated_state", 1,
	                                                        [this](msg::FcEstimatedState::ConstSharedPtr s) { onState(s); });
	_health_sub = create_subscription<msg::FcEstimatorStatus>("sensors/fc/estimator_status", 1,
	                                                          [this](msg::FcEstimatorStatus::ConstSharedPtr h) { onHealth(h); });
	_timer = create_wall_timer(std::chrono::milliseconds(2), [this] { watchdog(); });
}
void FcStateAdapterNode::invalidate(const std::string& reason) {
	// Failure event preserves the original state epoch. It cannot refresh a state lease.
	if (_output.fc_state_valid) ++_generation;
	_output.fc_state_valid = false;
	_output.estimator_ready = _output.navigation_ready = false;
	_output.reset_counter = _generation;
	_output.state_source = "fc_fused";
	_output.clock_id = _clock_id;
	_output.readiness_reason = reason;
	_output.published_steady_time = monotonicSeconds();
	_fused_pub->publish(_output);
}
void FcStateAdapterNode::onHealth(msg::FcEstimatorStatus::ConstSharedPtr h) {
	if (h->clock_id != _clock_id || h->source_session.empty()) {
		invalidate("FC health clock/session invalid");
		return;
	}
	if (_session != h->source_session) {
		invalidate("FC session changed");
		++_generation;
		_session = h->source_session;
		_last_time = 0;
		_have_reset = false;
	}
	if (!_have_reset || _reset != h->reset_counter) {
		invalidate("FC estimator reset");
		++_generation;
		_reset = h->reset_counter;
		_have_reset = true;
	}
	_health = *h;
	// Heading flags remain observable, but do not revoke FC control state.
	if (!h->valid || !h->clock_aligned || (h->flags & 0xc3) != 0xc3) invalidate("FC estimator health invalid");
	// A healthy status never republishes an old odometry sample as new state.
}
void FcStateAdapterNode::onState(msg::FcEstimatedState::ConstSharedPtr s) {
	if (s->clock_id == _clock_id && _receiver_epoch != s->validity_epoch) {
		invalidate("FC receiver validity epoch changed");
		++_generation;
		_receiver_epoch = s->validity_epoch;
		_health.valid = false;
	}
	if (!s->valid || s->clock_id != _clock_id || s->source_session.empty()) {
		invalidate(s->reason);
		return;
	}
	if (_session != s->source_session) {
		invalidate("FC state session changed");
		++_generation;
		_session = s->source_session;
		_last_time = 0;
		_have_reset = false;
		_health = msg::FcEstimatorStatus();
	}
	if (s->fc_time_usec <= _last_time) {
		if (s->fc_time_usec < _last_time) invalidate("FC state time regressed");
		return;
	}
	_last_time = s->fc_time_usec;
	msg::FusedState out;
	out.header = s->header;
	out.clock_id = _clock_id;
	out.state_source = "fc_fused";
	out.navigation_source = "fc_fused";
	out.navigation_session = s->source_session;
	out.reset_counter = _generation;
	out.fc_reset_counter = _health.reset_counter;
	out.fc_state_sequence = _health.state_sequence;
	out.fc_time_usec = s->fc_time_usec;
	out.position = s->position;
	out.orientation = s->orientation;
	out.velocity = s->velocity;
	out.body_rates = s->body_rates;
	// Finite structural placeholder ONLY. Runtime GEO rejects drag compensation.
	out.acceleration_available = false;
	out.imu_receive_time = out.imu_sample_steady_time = out.rtk_receive_time = NAN;
	out.heading_variance = out.posterior_heading_variance = out.navigation_innovation_squared = NAN;
	out.posterior_position_variance.fill(NAN);
	out.posterior_velocity_variance.fill(NAN);
	out.gyro_bias.x = out.gyro_bias.y = out.gyro_bias.z = NAN;
	out.acceleration_bias.x = out.acceleration_bias.y = out.acceleration_bias.z = NAN;
	out.fc_state_sample_steady_time = s->sample_steady_time;
	out.fc_state_receive_steady_time = s->receive_steady_time;
	out.fc_health_sample_steady_time = _health.sample_steady_time;
	out.fc_health_receive_steady_time = _health.receive_steady_time;
	out.fc_xy_observation_time = _health.sample_steady_time - _health.xy_observation_age_us * 1e-6;
	out.fc_z_observation_time = _health.sample_steady_time - _health.z_observation_age_us * 1e-6;
	out.fc_flags = _health.flags;
	std::copy(_health.position_variance.begin(), _health.position_variance.end(), out.position_variance.begin());
	std::copy(_health.velocity_variance.begin(), _health.velocity_variance.end(), out.velocity_variance.begin());
	out.covariance_stamp = _health.header.stamp;  // Reported FC diagonals, never a ROS posterior.
	out.clock_aligned = s->clock_aligned && _health.clock_aligned;
	out.heading_valid = (_health.flags & 4) != 0;
	out.published_steady_time = monotonicSeconds();
	const agi::Quaternion q(s->orientation.w, s->orientation.x, s->orientation.y, s->orientation.z);
	bool finite = q.coeffs().allFinite() && std::abs(q.norm() - 1) < 1e-3;
	for (double v : {s->position.x, s->position.y, s->position.z, s->velocity.x, s->velocity.y, s->velocity.z, s->body_rates.x,
	                 s->body_rates.y, s->body_rates.z})
		finite = finite && std::isfinite(v);
	const bool exclusive = count_publishers("fused_state") == 1 && count_publishers("state") == 1 &&
	                       count_publishers("sensors/fc/estimated_state") == 1 && count_publishers("sensors/fc/estimator_status") == 1;
	out.fc_state_valid = exclusive && finite && s->header.frame_id == "odom" && s->quality >= -1 && s->quality <= 100 &&
	                     s->estimator_type == 8 && _have_reset && _health.valid && _health.source_session == s->source_session &&
	                     uint8_t(_health.reset_counter) == s->reset_counter && _health.version == 1 && _health.payload_length == 68 &&
	                     SafetyGate::fresh(now().seconds(), stampSeconds(s->header.stamp), .015);
	agi::hardware::Evidence e;
	e.now = monotonicSeconds();
	setFcEvidence(out, e);
	const char* failure = SafetyGate::fcFailure(e);
	out.fc_state_valid = !failure;
	out.estimator_ready = out.navigation_ready = out.fc_state_valid;
	out.accuracy_known =
	        std::all_of(out.position_variance.begin(), out.position_variance.end(), [](double v) { return std::isfinite(v); }) &&
	        std::all_of(out.velocity_variance.begin(), out.velocity_variance.end(), [](double v) { return std::isfinite(v); });
	out.accuracy_ok = out.navigation_accuracy_ok = !failure;
	out.readiness_reason = failure ? failure : "FC fused state ready; acceleration/attitude covariance unavailable";
	if (!out.fc_state_valid && _output.fc_state_valid) out.reset_counter = ++_generation;
	_output = out;
	_fused_pub->publish(out);
	if (!out.fc_state_valid) return;
	nav_msgs::msg::Odometry odom;
	odom.header = out.header;
	odom.child_frame_id = "base_link";
	odom.pose.pose.position = out.position;
	odom.pose.pose.orientation = out.orientation;
	const agi::Vector<3> body = q.conjugate() * agi::Vector<3>(out.velocity.x, out.velocity.y, out.velocity.z);
	// nav_msgs twist belongs to child_frame_id; FusedState velocity remains world ENU.
	odom.twist.twist.linear.x = body.x();
	odom.twist.twist.linear.y = body.y();
	odom.twist.twist.linear.z = body.z();
	odom.twist.twist.angular = out.body_rates;
	odom.pose.covariance.fill(NAN);
	odom.twist.covariance.fill(NAN);
	// Diagonals alone cannot reconstruct cross covariance or rotate it into body.
	for (int i = 0; i < 3; ++i) odom.pose.covariance[i * 7] = out.position_variance[i];
	_odom_pub->publish(odom);
}
void FcStateAdapterNode::watchdog() {
	const double t = monotonicSeconds(), offset = now().seconds() - t;
	if (std::isfinite(_ros_offset) && std::abs(offset - _ros_offset) > .05) {
		invalidate("ROS clock jumped");
		_health = msg::FcEstimatorStatus();
		_have_reset = false;
	}
	_ros_offset = offset;
	if (!_output.fc_state_valid) return;
	agi::hardware::Evidence e;
	e.now = t;
	setFcEvidence(_output, e);
	if (const char* failure = SafetyGate::fcFailure(e, agi::hardware::FcStateCheck::Hold))
		invalidate(failure);
	else if (count_publishers("fused_state") > 1 || count_publishers("state") > 1 ||
	         count_publishers("sensors/fc/estimated_state") > 1 || count_publishers("sensors/fc/estimator_status") > 1)
		invalidate("Conflicting state publishers");
}
}  // namespace agi_ros2
