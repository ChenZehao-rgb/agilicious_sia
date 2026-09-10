#include "agi_ros2/state_fusion_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include "agi_ros2/node_common.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

StateFusionNode::StateFusionNode()
        : Node("state_fusion"), _clock_id(readClockId()), _rtk_receive_time(kUnknownTime), _last_rtk_time(kUnknownTime) {
	reset();
	_fused_pub = create_publisher<msg::FusedState>("fused_state", 1);
	_state_pub = create_publisher<nav_msgs::msg::Odometry>("state", 1);
	_rtk_sub = create_subscription<msg::Rtk>("sensors/rtk", 10, std::bind(&StateFusionNode::onRtk, this, std::placeholders::_1));
	_imu_sub = create_subscription<sensor_msgs::msg::Imu>("sensors/imu", rclcpp::SensorDataQoS().keep_last(256),
	                                                      std::bind(&StateFusionNode::onImu, this, std::placeholders::_1));
}

void StateFusionNode::reset() {
	_ahrs = std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
	_state.setZero();
	_state.t = kUnknownTime;
	_last_rtk_time = kUnknownTime;
	++_reset_counter;
}

void StateFusionNode::onRtk(msg::Rtk::ConstSharedPtr message) {
	_rtk = *message;
	_rtk_receive_time = monotonicSeconds();
}

void StateFusionNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
	const double received = monotonicSeconds();
	const double time = stampSeconds(message->header.stamp);
	if (message->header.frame_id != "base_link") {
		return;
	}
	const agi::ImuSample imu(time, {message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z},
	                         {message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z});
	if (!imu.valid()) {
		return;
	}
	if (std::isfinite(_state.t)) {
		if (time == _state.t) {
			return;
		}
		if (time < _state.t || time - _state.t > 0.025) {
			if (time < _state.t) {
				_rtk = msg::Rtk();
				_rtk_receive_time = kUnknownTime;
			}
			reset();
		}
	}

	const double fix_time = stampSeconds(_rtk.header.stamp);
	if ((get_parameter("use_sim_time").as_bool() || SafetyGate::fresh(received, _rtk_receive_time, 0.3)) &&
	    SafetyGate::fresh(time, fix_time, 0.3) && _rtk.header.frame_id == "odom" && _rtk.fixed && _rtk.heading_valid &&
	    _rtk.accuracy_ok && std::isfinite(_rtk.heading) && (!std::isfinite(_last_rtk_time) || fix_time > _last_rtk_time)) {
		const agi::Vector<3> position(_rtk.position.x, _rtk.position.y, _rtk.position.z);
		const agi::Vector<3> velocity(_rtk.velocity.x, _rtk.velocity.y, _rtk.velocity.z);
		if (position.allFinite() && velocity.allFinite()) {
			// Preserve the existing algorithm for this architectural split. This
			// constant-velocity extrapolation is NOT delayed-measurement IMU replay.
			const double state_time = std::isfinite(_state.t) ? _state.t : fix_time;
			_state.p = position + velocity * std::max(0.0, state_time - fix_time);
			_state.v = velocity;
			_ahrs->setHeading(_rtk.heading);
			_ahrs->setVelocity(velocity, fix_time);
			_last_rtk_time = fix_time;
		}
	}

	_ahrs->addImu(imu);
	if (!_ahrs->initialized()) {
		return;
	}
	const double dt = std::isfinite(_state.t) ? time - _state.t : 0.0;
	_state.q(_ahrs->attitude());
	_state.w = imu.omega - _ahrs->gyroBias();
	_state.a = _state.q() * imu.acc + agi::Vector<3>(0, 0, -9.8066);
	if (dt > 0 && dt <= 0.025 && std::isfinite(_last_rtk_time)) {
		_state.p += _state.v * dt + 0.5 * _state.a * dt * dt;
		_state.v += _state.a * dt;
	}
	_state.t = time;
	publishState(received);
}

void StateFusionNode::publishState(double imu_receive_time) {
	msg::FusedState out;
	out.header.stamp = rosStamp(_state.t);
	out.header.frame_id = "odom";
	out.position.x = _state.p.x();
	out.position.y = _state.p.y();
	out.position.z = _state.p.z();
	out.velocity.x = _state.v.x();
	out.velocity.y = _state.v.y();
	out.velocity.z = _state.v.z();
	out.orientation.w = _state.q().w();
	out.orientation.x = _state.q().x();
	out.orientation.y = _state.q().y();
	out.orientation.z = _state.q().z();
	out.body_rates.x = _state.w.x();
	out.body_rates.y = _state.w.y();
	out.body_rates.z = _state.w.z();
	out.acceleration.x = _state.a.x();
	out.acceleration.y = _state.a.y();
	out.acceleration.z = _state.a.z();
	out.clock_id = _clock_id;
	out.published_steady_time = monotonicSeconds();
	out.imu_receive_time = imu_receive_time;
	out.rtk_receive_time = _rtk_receive_time;
	if (std::isfinite(_last_rtk_time)) {
		out.rtk_stamp = rosStamp(_last_rtk_time);
	}
	out.reset_counter = _reset_counter;
	out.initialized = _state.valid() && std::isfinite(_last_rtk_time);
	out.rtk_fixed = _rtk.fixed;
	out.heading_valid = _rtk.heading_valid;
	out.accuracy_ok = _rtk.accuracy_ok;
	out.synchronized = _rtk.synchronized;
	_fused_pub->publish(out);

	// Keep the established evaluation topic, with standard body-frame twist.
	nav_msgs::msg::Odometry odometry;
	odometry.header = out.header;
	odometry.child_frame_id = "base_link";
	odometry.pose.pose.position = out.position;
	odometry.pose.pose.orientation = out.orientation;
	const auto velocity_body = _state.q().conjugate() * _state.v;
	odometry.twist.twist.linear.x = velocity_body.x();
	odometry.twist.twist.linear.y = velocity_body.y();
	odometry.twist.twist.linear.z = velocity_body.z();
	odometry.twist.twist.angular = out.body_rates;
	_state_pub->publish(odometry);
}

}  // namespace agi_ros2
