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
#include "agi_ros2/runtime_config.h"
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
	rcl_interfaces::msg::ParameterDescriptor shadow_descriptor;
	shadow_descriptor.read_only = true;
	_mode = declare_parameter<std::string>("mode", "sitl", shadow_descriptor);
	const auto profile = loadRuntimeConfig(*this, _mode);
	_shadow_only = declare_parameter<bool>("shadow_only", profile ? profile->section("flight")["shadow_only"].as<bool>() : false,
	                                       shadow_descriptor);
	const auto navigation =
	        declare_parameter<std::string>("navigation_source", _mode == "hardware" ? "gnss" : "rtk", shadow_descriptor);
	if (navigation != "gnss" && navigation != "rtk") throw std::invalid_argument("navigation_source must be gnss or rtk");
	_navigation_policy = navigation == "gnss" ? agi::hardware::NavigationPolicy::Gnss : agi::hardware::NavigationPolicy::Rtk;
	if (_shadow_only && _mode != "hardware") throw std::invalid_argument("shadow_only requires hardware mode");
	if (_mode != "sitl" && _mode != "hardware") {
		throw std::invalid_argument("mode must be sitl or hardware");
	}
	if (_mode == "hardware" && get_parameter("use_sim_time").as_bool()) {
		throw std::invalid_argument("Hardware requires use_sim_time=false");
	}
	_simulation_time = _mode == "sitl" && get_parameter("use_sim_time").as_bool();
	const bool delay_test = declare_parameter<bool>(
	        "sitl_delay_test", profile ? profile->section("flight")["sitl_delay_test"].as<bool>() : false, shadow_descriptor);
	if (delay_test && !_simulation_time) {
		throw std::invalid_argument("sitl_delay_test requires mode=sitl and use_sim_time=true");
	}
	_timing_checks = !delay_test;
	if (delay_test) RCLCPP_INFO(get_logger(), "SITL delay test: timing limits disabled");

	const auto params_dir = declare_parameter<std::string>("params_dir", "");
	const auto pilot_file = declare_parameter<std::string>("pilot_config", "pilot_ros2.yaml");
	const auto trajectory = declare_parameter<std::string>(
	        "trajectory", profile ? profile->resolvePath(profile->section("flight")["trajectory"].as<std::string>()) : "",
	        shadow_descriptor);
	_params = profile ? profile->createPilotParams()
	                  : std::make_unique<agi::PilotParams>(std::filesystem::path(params_dir) / pilot_file, params_dir);
	_pilot = std::make_unique<agi::hardware::HardwarePilot>(
	        *_params, [this] { return _control_time; }, [this] { return _simulation_time ? _control_time : monotonicSeconds(); },
	        _navigation_policy);
	if (!trajectory.empty()) {
		const auto rows = agi::trajectory_csv::readTrajectoryRows(trajectory);
		const auto points = agi::trajectory_csv::loadTrajectory(rows, 0, agi::Vector<3>::Zero(), 0,
		                                                        agi::trajectory_csv::estimateSourceMass(rows),
		                                                        -std::numeric_limits<double>::infinity());
		if (!_pilot->setTrajectory(points)) {
			throw std::invalid_argument("Invalid trajectory timestamps or states");
		}
		RCLCPP_INFO(get_logger(), "Loaded trajectory: %s (%zu samples, %.3f s)", trajectory.c_str(), points.size(),
		            points.back().state.t);
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
	_computation_pub = create_publisher<msg::ComputationStatus>("computation_status", 10);
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
	const auto cycle_start = std::chrono::steady_clock::now();
	const auto timely = [this](double now, double sample, double limit) {
		return SafetyGate::fresh(now, sample, limit, _timing_checks);
	};
	_control_time = alignedRosTime(*this, std::max({stampSeconds(_state.header.stamp), stampSeconds(_authority.header.stamp),
	                                                stampSeconds(_health.header.stamp)}));
	const double wall = monotonicSeconds();
	const double safety_now = _simulation_time ? _control_time : wall;
	if (_output_fault || (std::isfinite(_previous_clock) &&
	                      (_control_time < _previous_clock || (_timing_checks && _control_time - _previous_clock > 0.25)))) {
		_pilot->reportOutputFault();
		_output_fault = false;
	}
	_previous_clock = _control_time;

	agi::QuadState state;
	state.setZero();
	state.t = kUnknownTime;
	if (_state.initialized && _state.clock_id == _clock_id && _state.header.frame_id == "odom" &&
	    timely(wall, _state.published_steady_time, _simulation_time ? kSitlWallTimeout : 0.010)) {
		state.t = stampSeconds(_state.header.stamp);
		state.p = agi::Vector<3>{_state.position.x, _state.position.y, _state.position.z};
		state.v = agi::Vector<3>{_state.velocity.x, _state.velocity.y, _state.velocity.z};
		state.q(agi::Quaternion(_state.orientation.w, _state.orientation.x, _state.orientation.y, _state.orientation.z));
		state.w = agi::Vector<3>{_state.body_rates.x, _state.body_rates.y, _state.body_rates.z};
		state.a = agi::Vector<3>{_state.acceleration.x, _state.acceleration.y, _state.acceleration.z};
	}
	agi::hardware::Evidence evidence;
	evidence.timing_checks = _timing_checks;
	evidence.now = safety_now;
	evidence.imu_time = _state.clock_id == _clock_id ? (_simulation_time ? state.t : _state.imu_receive_time) : kUnknownTime;
	evidence.rtk_time = _state.initialized ? safety_now - (_control_time - stampSeconds(_state.rtk_stamp)) : kUnknownTime;
	evidence.rc_time = safety_now - (_control_time - stampSeconds(_authority.header.stamp));
	evidence.rc_link = _authority.rc_link && timely(wall, _authority_receive_time, _simulation_time ? kSitlWallTimeout : 0.1);
	evidence.armed = _authority.armed;
	evidence.auto_switch = _authority.auto_switch;
	evidence.kill = _authority.kill;
	evidence.rtk_fixed = _state.rtk_fixed && (_simulation_time || timely(wall, _state.rtk_receive_time, 0.3));
	evidence.heading_valid = _state.heading_valid;
	evidence.accuracy_ok = _state.accuracy_ok;
	evidence.synchronized = _state.synchronized;
	const bool health_fresh = timely(wall, _health_receive_time, _simulation_time ? kSitlWallTimeout : 0.2) &&
	                          timely(_control_time, stampSeconds(_health.header.stamp), 0.2);
	const bool output_fresh = timely(wall, _output_receive_time, _simulation_time ? kSitlWallTimeout : 0.05) &&
	                          timely(wall, _output.steady_time, _simulation_time ? kSitlWallTimeout : 0.05);
	evidence.imu_calibrated = health_fresh && _health.imu_calibrated;
	evidence.converged = health_fresh && _health.converged && _state.initialized;
	evidence.imu_ready = health_fresh && _health.imu_ready && _state.imu_ready;
	evidence.estimator_ready = health_fresh && _health.estimator_ready && _state.estimator_ready;
	evidence.clock_aligned = _state.clock_aligned;
	evidence.accuracy_known = _state.accuracy_known;
	evidence.navigation_ready = health_fresh && _health.navigation_ready && _state.navigation_ready &&
	                            _state.navigation_source == "gnss" && _state.fix_type >= 3 && _state.fix_type <= 6 &&
	                            timely(wall, _state.rtk_receive_time, .3);
	if (_navigation_policy == agi::hardware::NavigationPolicy::Gnss) evidence.accuracy_ok = _state.navigation_accuracy_ok;
	evidence.config_verified = health_fresh && _health.config_verified;
	evidence.thrust_calibrated = health_fresh && _health.thrust_calibrated && output_fresh && _output.thrust_calibrated;
	evidence.geofence_ok = health_fresh && _health.geofence_ok;
	evidence.msp_healthy = health_fresh && _health.transport_healthy && output_fresh && _output.transport_healthy;
	const bool navigation_valid = _state.navigation_source == "gnss" && _state.navigation_valid && _state.clock_aligned &&
	                              _state.heading_valid && timely(_control_time, stampSeconds(_state.rtk_stamp), .3) &&
	                              timely(wall, _state.rtk_receive_time, .3) && timely(wall, _state.imu_receive_time, .010);
	const auto decision = _pilot->tick(state, evidence, _shadow_only, navigation_valid);
	_cycle_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count();
	publishDecision(decision, state);
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
	command.permit_override = !_shadow_only && decision.permit_override;
	command.mode = static_cast<uint8_t>(decision.mode);
	command.evidence = encodeEvidence(decision.evidence);
	_command_pub->publish(command);
	msg::ComputationStatus computation;
	computation.header = command.header;
	computation.shadow_only = _shadow_only;
	computation.state_valid = decision.state_valid;
	computation.mpc_success = decision.evidence.command_valid;
	computation.warm_cycles = _pilot->warmCycles();
	computation.trajectory_active = decision.trajectory_active;
	computation.reference_elapsed = decision.reference_elapsed;
	computation.solve_seconds = decision.evidence.solve_seconds;
	computation.cycle_seconds = _cycle_seconds;
	computation.state_age = _control_time - state.t;
	computation.navigation_age = _control_time - stampSeconds(_state.rtk_stamp);
	computation.imu_age = decision.evidence.now - decision.evidence.imu_time;
	computation.reason = decision.reason;
	if (!_state.readiness_reason.empty()) computation.reason += "; fusion: " + _state.readiness_reason;
	if (!_health.reason.empty()) computation.reason += "; health: " + _health.reason;
	_computation_pub->publish(computation);

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
		message.twist.twist.linear.x = reference.v.x();
		message.twist.twist.linear.y = reference.v.y();
		message.twist.twist.linear.z = reference.v.z();
		message.twist.twist.angular.x = reference.w.x();
		message.twist.twist.angular.y = reference.w.y();
		message.twist.twist.angular.z = reference.w.z();
		_reference_pub->publish(message);
	}
}

}  // namespace agi_ros2
