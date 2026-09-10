#include "agi_ros2/control_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>

#include "agi_ros2/node_common.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/reference/trajectory_csv.hpp"
#include "rclcpp/create_timer.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

ControlNode::ControlNode()
        : Node("flight_control"),
          _clock_id(readClockId()),
          _previous_clock(kUnknownTime),
          _authority_receive_time(kUnknownTime),
          _health_receive_time(kUnknownTime),
          _output_receive_time(kUnknownTime) {
	_mode = declare_parameter<std::string>("mode", "sitl");
	if (_mode != "sitl" && _mode != "hardware") {
		throw std::invalid_argument("mode must be sitl or hardware");
	}
	if (_mode == "hardware" && get_parameter("use_sim_time").as_bool()) {
		throw std::invalid_argument("Hardware requires use_sim_time=false");
	}
	_simulation_time = _mode == "sitl" && get_parameter("use_sim_time").as_bool();
	const auto params_dir = declare_parameter<std::string>("params_dir", "");
	const auto pilot_file = declare_parameter<std::string>("pilot_config", "pilot_ros2.yaml");
	const auto trajectory = declare_parameter<std::string>("trajectory", "");
	_params = std::make_unique<agi::PilotParams>(std::filesystem::path(params_dir) / pilot_file, params_dir);
	_pilot = std::make_unique<agi::hardware::HardwarePilot>(
	        *_params, [this] { return _control_time; }, [this] { return _simulation_time ? _control_time : monotonicSeconds(); });
	if (!trajectory.empty()) {
		const auto rows = agi::trajectory_csv::readTrajectoryRows(trajectory);
		const auto points = agi::trajectory_csv::loadTrajectory(rows, 0, agi::Vector<3>::Zero(), 0,
		                                                        agi::trajectory_csv::estimateSourceMass(rows),
		                                                        -std::numeric_limits<double>::infinity());
		if (!_pilot->setTrajectory(points)) {
			throw std::invalid_argument("Invalid trajectory timestamps or states");
		}
		RCLCPP_INFO(get_logger(), "Loaded trajectory: %s (%zu samples, %.3f s)",
		            trajectory.c_str(), points.size(), points.back().state.t);
	}
	_state_sub = create_subscription<msg::FusedState>("fused_state", 1, std::bind(&ControlNode::onState, this, std::placeholders::_1));
	_authority_sub = create_subscription<msg::Authority>("authority", 1, [this](msg::Authority::ConstSharedPtr message) {
		_authority = *message;
		_authority_receive_time = monotonicSeconds();
	});
	_health_sub = create_subscription<msg::Health>("health", 1, [this](msg::Health::ConstSharedPtr message) {
		_health = *message;
		_health_receive_time = monotonicSeconds();
	});
	_output_sub = create_subscription<msg::OutputStatus>("output_status", 1,
	                                                     std::bind(&ControlNode::onOutputStatus, this, std::placeholders::_1));
	_command_pub = create_publisher<msg::ControlCommand>("control_command", 1);
	_reference_pub = create_publisher<nav_msgs::msg::Odometry>("reference", 1);
	_diagnostic_pub = create_publisher<std_msgs::msg::Float64MultiArray>("control_diagnostics", 10);
	_status_pub = create_publisher<std_msgs::msg::String>("status", 1);
	if (_simulation_time) {
		// A paused /clock must not run duplicate MPC ticks or consume warmup.
		_timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), std::bind(&ControlNode::tick, this));
	} else {
		_timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControlNode::tick, this));
	}
}

void ControlNode::onState(msg::FusedState::ConstSharedPtr message) {
	if (message->clock_id != _clock_id) {
		return;
	}
	if (_state.reset_counter != 0 && message->reset_counter != _state.reset_counter) {
		_output_fault = true;
	}
	_state = *message;
}

void ControlNode::onOutputStatus(msg::OutputStatus::ConstSharedPtr message) {
	if (message->clock_id != _clock_id) {
		return;
	}
	if (!_output.clock_id.empty() && (_output.session_start != message->session_start || _output.fault_count != message->fault_count)) {
		_output_fault = true;
	}
	_output = *message;
	_output_receive_time = monotonicSeconds();
}

void ControlNode::tick() {
	_control_time = alignedRosTime(*this, std::max({stampSeconds(_state.header.stamp), stampSeconds(_authority.header.stamp),
	                                                stampSeconds(_health.header.stamp)}));
	const double wall = monotonicSeconds();
	const double safety_now = _simulation_time ? _control_time : wall;
	if (_output_fault ||
	    (std::isfinite(_previous_clock) && (_control_time < _previous_clock || _control_time - _previous_clock > 0.25))) {
		_pilot->reportOutputFault();
		_output_fault = false;
	}
	_previous_clock = _control_time;

	agi::QuadState state;
	state.setZero();
	state.t = kUnknownTime;
	if (_state.initialized && _state.clock_id == _clock_id && _state.header.frame_id == "odom" &&
	    SafetyGate::fresh(wall, _state.published_steady_time, _simulation_time ? kSitlWallTimeout : 0.010)) {
		state.t = stampSeconds(_state.header.stamp);
		state.p = agi::Vector<3>{_state.position.x, _state.position.y, _state.position.z};
		state.v = agi::Vector<3>{_state.velocity.x, _state.velocity.y, _state.velocity.z};
		state.q(agi::Quaternion(_state.orientation.w, _state.orientation.x, _state.orientation.y, _state.orientation.z));
		state.w = agi::Vector<3>{_state.body_rates.x, _state.body_rates.y, _state.body_rates.z};
		state.a = agi::Vector<3>{_state.acceleration.x, _state.acceleration.y, _state.acceleration.z};
	}
	agi::hardware::Evidence evidence;
	evidence.now = safety_now;
	evidence.imu_time = _state.clock_id == _clock_id ? (_simulation_time ? state.t : _state.imu_receive_time) : kUnknownTime;
	evidence.rtk_time = _state.initialized ? safety_now - (_control_time - stampSeconds(_state.rtk_stamp)) : kUnknownTime;
	evidence.rc_time = safety_now - (_control_time - stampSeconds(_authority.header.stamp));
	evidence.rc_link =
	        _authority.rc_link && SafetyGate::fresh(wall, _authority_receive_time, _simulation_time ? kSitlWallTimeout : 0.1);
	evidence.armed = _authority.armed;
	evidence.auto_switch = _authority.auto_switch;
	evidence.kill = _authority.kill;
	evidence.rtk_fixed = _state.rtk_fixed && (_simulation_time || SafetyGate::fresh(wall, _state.rtk_receive_time, 0.3));
	evidence.heading_valid = _state.heading_valid;
	evidence.accuracy_ok = _state.accuracy_ok;
	evidence.synchronized = _state.synchronized;
	const bool health_fresh = SafetyGate::fresh(wall, _health_receive_time, _simulation_time ? kSitlWallTimeout : 0.2) &&
	                          SafetyGate::fresh(_control_time, stampSeconds(_health.header.stamp), 0.2);
	const bool output_fresh = SafetyGate::fresh(wall, _output_receive_time, _simulation_time ? kSitlWallTimeout : 0.05) &&
	                          SafetyGate::fresh(wall, _output.steady_time, _simulation_time ? kSitlWallTimeout : 0.05);
	evidence.imu_calibrated = health_fresh && _health.imu_calibrated;
	evidence.converged = health_fresh && _health.converged && _state.initialized;
	evidence.config_verified = health_fresh && _health.config_verified;
	evidence.thrust_calibrated = health_fresh && _health.thrust_calibrated && output_fresh && _output.thrust_calibrated;
	evidence.geofence_ok = health_fresh && _health.geofence_ok;
	evidence.msp_healthy = health_fresh && _health.transport_healthy && output_fresh && _output.transport_healthy;
	publishDecision(_pilot->tick(state, evidence), state);
}

void ControlNode::publishDecision(const agi::hardware::ControlDecision& decision, const agi::QuadState& state) {
	msg::ControlCommand command;
	if (_simulation_time) {
		command.header.stamp = rosStamp(_control_time);
	} else {
		command.header.stamp = now();
	}
	command.header.frame_id = "base_link";
	command.clock_id = evidenceClockId(_clock_id, _simulation_time);
	command.sequence = ++_sequence;
	// Agilib stores collective thrust as acceleration; expose physical N on ROS.
	command.total_thrust = decision.command.collective_thrust * _params->quad_.m_;
	command.body_rates.x = decision.command.omega.x();
	command.body_rates.y = decision.command.omega.y();
	command.body_rates.z = decision.command.omega.z();
	command.permit_override = decision.permit_override;
	command.mode = static_cast<uint8_t>(decision.mode);
	command.evidence = encodeEvidence(decision.evidence);
	_command_pub->publish(command);

	std_msgs::msg::Float64MultiArray diagnostic;
	const auto& evidence = decision.evidence;
	diagnostic.data = {_control_time,
	                   evidence.now,
	                   evidence.now - evidence.imu_time,
	                   _control_time - state.t,
	                   _control_time - stampSeconds(_state.rtk_stamp),
	                   evidence.now - evidence.rc_time,
	                   evidence.solve_seconds,
	                   static_cast<double>(_pilot->warmCycles()),
	                   decision.permit_override ? 1.0 : 0.0,
	                   static_cast<double>(decision.mode)};
	_diagnostic_pub->publish(diagnostic);
	std_msgs::msg::String status;
	status.data = std::string(SafetyGate::name(decision.mode)) + ": " + decision.reason;
	_status_pub->publish(status);
	if (decision.reference.valid()) {
		const auto& reference = decision.reference;
		nav_msgs::msg::Odometry message;
		message.header.stamp = rosStamp(reference.t);
		message.header.frame_id = "odom";
		message.pose.pose.position.x = reference.p.x();
		message.pose.pose.position.y = reference.p.y();
		message.pose.pose.position.z = reference.p.z();
		message.pose.pose.orientation.w = reference.q().w();
		message.pose.pose.orientation.x = reference.q().x();
		message.pose.pose.orientation.y = reference.q().y();
		message.pose.pose.orientation.z = reference.q().z();
		_reference_pub->publish(message);
	}
}

}  // namespace agi_ros2
