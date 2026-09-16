#include "agi_ros2/command_output_node.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "agi_ros2/node_common.h"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kGravity = 9.8066;
constexpr std::array<uint16_t, 4> kIdleChannels = {1500, 1500, 1000, 1500};
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

CommandOutputNode::CommandOutputNode()
        : Node("command_output"),
          _clock_id(readClockId()),
          _session_start(monotonicSeconds()),
          _command_receive_time(kUnknownTime),
          _authority_receive_time(kUnknownTime),
          _health_receive_time(kUnknownTime),
          _previous_command_time(kUnknownTime),
          _previous_ros_time(kUnknownTime) {
	_mode = declare_parameter<std::string>("mode", "sitl");
	if (_mode != "sitl" && _mode != "hardware") {
		throw std::invalid_argument("mode must be sitl or hardware");
	}
	if (_mode == "hardware" && get_parameter("use_sim_time").as_bool()) {
		throw std::invalid_argument("Hardware requires use_sim_time=false");
	}
	_simulation_time = _mode == "sitl" && get_parameter("use_sim_time").as_bool();
	const bool delay_test = declare_parameter<bool>("sitl_delay_test", false);
	if (delay_test && !_simulation_time) {
		throw std::invalid_argument("sitl_delay_test requires mode=sitl and use_sim_time=true");
	}
	_timing_checks = !delay_test;
	if (delay_test) RCLCPP_INFO(get_logger(), "SITL delay test: timing limits disabled");

	const auto params_dir = declare_parameter<std::string>("params_dir", "");
	const auto pilot_file = declare_parameter<std::string>("pilot_config", "pilot_ros2.yaml");
	const auto bridge_file = declare_parameter<std::string>("bridge_config", "betaflight_udp.yaml");
	const auto device = declare_parameter<std::string>("device", "/dev/ttyAMA0");
	const int baud = declare_parameter<int>("baud", 921600);
	const auto thrust_file = declare_parameter<std::string>("thrust_table", "");
	if (!_bridge_params.load(std::filesystem::path(params_dir) / bridge_file)) {
		throw std::invalid_argument("Invalid Betaflight channel mapping");
	}
	const agi::Yaml pilot_config(std::filesystem::path(params_dir) / pilot_file);
	const auto quad_file = pilot_config["quadrotor"].as<std::string>();
	agi::Quadrotor quad;
	if (!quad.load(std::filesystem::path(params_dir) / "quads" / quad_file) || !quad.valid()) {
		throw std::invalid_argument("Invalid vehicle mass configuration");
	}
	_mass = quad.m_;
	_mapper = std::make_unique<agi::BetaflightRcMapper>(_bridge_params);
	if (!thrust_file.empty()) {
		loadThrustTable(thrust_file);
	}
	if (_mode == "hardware") {
		if (!_thrust) {
			throw std::invalid_argument("Hardware output requires a thrust_table");
		}
		_msp = std::make_unique<agi::hardware::BetaflightMspBridge>(device, baud);
	} else {
		_destination.sin_family = AF_INET;
		_destination.sin_port = htons(_bridge_params.port);
		if (inet_pton(AF_INET, _bridge_params.host.c_str(), &_destination.sin_addr) != 1) {
			throw std::invalid_argument("Invalid UDP destination address");
		}
		_socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
		if (_socket_fd < 0) {
			throw std::runtime_error("Cannot open SITL UDP socket");
		}
	}
	_status_pub = create_publisher<msg::OutputStatus>("output_status", 1);
	_command_sub = create_subscription<msg::ControlCommand>("control_command", 1,
	                                                        std::bind(&CommandOutputNode::onCommand, this, std::placeholders::_1));
	_authority_sub = create_subscription<msg::Authority>("authority", 1, [this](msg::Authority::ConstSharedPtr message) {
		const bool revoke = message->kill || !message->armed || message->auto_switch != _authority.auto_switch || !message->rc_link;
		_authority = *message;
		_authority_receive_time = monotonicSeconds();
		if (revoke) {
			processOutput();
		}
	});
	_health_sub = create_subscription<msg::Health>("health", 1, [this](msg::Health::ConstSharedPtr message) {
		_health = *message;
		_health_receive_time = monotonicSeconds();
	});
	if (_msp) {
		_telemetry = std::make_unique<MspTelemetry>(*this);
		_msp_timer = create_wall_timer(std::chrono::milliseconds(1), [this] {
			// RC stays on the 100 Hz control-command callback: adding another
			// independent 10 ms wait would age the command's IMU evidence.
			_telemetry->tick(*_msp, monotonicSeconds() + 0.0025);
			if (!_telemetry->healthy()) _transport_healthy = false;
		});
	}
	_watchdog = create_wall_timer(std::chrono::milliseconds(5), std::bind(&CommandOutputNode::watchdog, this));
}

CommandOutputNode::~CommandOutputNode() {
	if (_socket_fd >= 0) {
		sendUdp(kIdleChannels, false);
		close(_socket_fd);
	}
	// MSP destruction only closes the serial port. Never write ARM/AUX or an
	// invented replacement throttle to physical hardware on shutdown.
}

void CommandOutputNode::loadThrustTable(const std::string& filename) {
	std::ifstream input(filename);
	if (!input) {
		throw std::invalid_argument("Cannot open thrust calibration table");
	}
	std::vector<double> voltages;
	std::vector<double> pwm;
	std::vector<std::vector<double>> forces;
	std::string line;
	bool first = true;
	while (std::getline(input, line)) {
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream row_stream(line);
		double value;
		std::vector<double> row;
		while (row_stream >> value) {
			row.push_back(value);
		}
		if (row.size() < 3) {
			throw std::invalid_argument("Invalid thrust calibration row");
		}
		if (first) {
			pwm.assign(row.begin() + 1, row.end());
			first = false;
		} else {
			voltages.push_back(row.front());
			forces.emplace_back(row.begin() + 1, row.end());
		}
	}
	_thrust = std::make_unique<agi::hardware::ThrustTable>(voltages, pwm, forces);
}

void CommandOutputNode::onCommand(msg::ControlCommand::ConstSharedPtr message) {
	if (message->clock_id != evidenceClockId(_clock_id, _simulation_time) || !std::isfinite(message->evidence.now) ||
	    (_timing_checks && _simulation_time && message->evidence.now > now().seconds() + 0.010) ||
	    (std::isfinite(_previous_command_time) && message->evidence.now <= _previous_command_time)) {
		return;
	}
	_previous_command_time = message->evidence.now;
	_command = *message;
	_command_receive_time = monotonicSeconds();
	processOutput();
}

std::array<uint16_t, 4> CommandOutputNode::mapCommand() const {
	if (!std::isfinite(_command.total_thrust) || _command.total_thrust < 0 || !std::isfinite(_command.body_rates.x) ||
	    !std::isfinite(_command.body_rates.y) || !std::isfinite(_command.body_rates.z)) {
		throw std::invalid_argument("Nonfinite or negative thrust/rates command");
	}
	std::array<uint16_t, 4> channels = kIdleChannels;
	const double sign = _mode == "hardware" ? -1.0 : 1.0;
	channels[0] = _mapper->rateToPwm(_mapper->inverseActualRate(_command.body_rates.x * kRadiansToDegrees, 0), _bridge_params.deadband);
	// Hardware is FRD. The SITL model already flips its sensor axes.
	channels[1] = _mapper->rateToPwm(_mapper->inverseActualRate(sign * _command.body_rates.y * kRadiansToDegrees, 1),
	                                 _bridge_params.deadband);
	channels[3] = _mapper->rateToPwm(_mapper->inverseActualRate(sign * _command.body_rates.z * kRadiansToDegrees, 2),
	                                 _bridge_params.yaw_deadband);
	const double acceleration = _command.total_thrust / _mass;
	if (_msp) {
		channels[2] = _thrust->collectiveThrustToRc(acceleration, _mass, _health.battery_voltage);
	} else {
		const double motor = (_bridge_params.motor_idle + (1 - _bridge_params.motor_idle) * _bridge_params.hover_throttle) *
		                     std::sqrt(acceleration / kGravity);
		channels[2] = static_cast<uint16_t>(
		        std::lround(_bridge_params.min_check +
		                    (2000 - _bridge_params.min_check) *
		                            std::clamp((motor - _bridge_params.motor_idle) / (1 - _bridge_params.motor_idle), 0.0, 1.0)));
	}
	return channels;
}

void CommandOutputNode::reportFault(const std::string& reason) {
	++_fault_count;
	_reason = reason;
}

void CommandOutputNode::processOutput() {
	const auto timely = [this](double now, double sample, double limit) {
		return SafetyGate::fresh(now, sample, limit, _timing_checks);
	};
	const double ros_time = alignedRosTime(*this, std::max({stampSeconds(_authority.header.stamp), stampSeconds(_health.header.stamp),
	                                                        stampSeconds(_command.header.stamp)}));
	auto evidence = decodeEvidence(_command.evidence);
	evidence.timing_checks = _timing_checks;
	const double wall = monotonicSeconds();
	evidence.now = _simulation_time ? ros_time : wall;
	const bool command_fresh =
	        _command.clock_id == evidenceClockId(_clock_id, _simulation_time) && _command.header.frame_id == "base_link" &&
	        timely(wall, _command_receive_time, _simulation_time ? kSitlWallTimeout : 0.025) &&
	        timely(evidence.now, _command.evidence.now, 0.025) && timely(ros_time, stampSeconds(_command.header.stamp), 0.025);
	const bool rc_fresh = timely(wall, _authority_receive_time, _simulation_time ? kSitlWallTimeout : 0.1) &&
	                      timely(ros_time, stampSeconds(_authority.header.stamp), 0.1) && _authority.rc_link;
	const bool health_fresh = timely(wall, _health_receive_time, _simulation_time ? kSitlWallTimeout : 0.2) &&
	                          timely(ros_time, stampSeconds(_health.header.stamp), 0.2);
	// New authority can revoke old commands immediately, but never authorize a
	// command computed under a different ARM/AUTO state.
	const bool authority_matches = evidence.armed == _authority.armed && evidence.auto_switch == _authority.auto_switch;
	// Authority and command travel on different DDS topics. A new AUTO edge
	// may arrive before its matching control decision (or vice versa). Wait for
	// that pair without consuming the healthy low edge. Never defer a revocation
	// while output is active, and never extend the command's original deadline.
	if (command_fresh && !authority_matches && !_override_active && rc_fresh && _authority.armed && !_authority.kill) {
		publishStatus();
		return;
	}
	evidence.rc_link = evidence.rc_link && rc_fresh;
	evidence.kill = evidence.kill || _authority.kill;
	evidence.armed = evidence.armed && _authority.armed;
	evidence.auto_switch = _authority.auto_switch;
	evidence.command_valid =
	        evidence.command_valid && command_fresh && authority_matches && (!_authority.auto_switch || _command.permit_override);
	evidence.msp_healthy = evidence.msp_healthy && _transport_healthy && health_fresh && _health.transport_healthy;
	evidence.config_verified = evidence.config_verified && health_fresh && _health.config_verified;
	evidence.geofence_ok = evidence.geofence_ok && health_fresh && _health.geofence_ok;
	evidence.thrust_calibrated = evidence.thrust_calibrated && health_fresh && _health.thrust_calibrated;
	bool active = _gate.update(evidence) && _command.permit_override;
	auto channels = kIdleChannels;
	if (active) {
		try {
			channels = mapCommand();
		} catch (const std::exception& error) {
			reportFault(error.what());
			evidence.command_valid = false;
			_gate.update(evidence);
			active = false;
		}
	}
	if (_override_active && !active && _authority.auto_switch && !_authority.kill) {
		reportFault("Output authorization or command expired");
	}
	if (_authority.auto_switch && !active) {
		evidence.command_valid = false;
	}
	if (_msp) {
		// The MSP bridge has its own gate, which must also observe healthy AUTO
		// low.
		const uint64_t previous_errors = _msp->errors();
		const bool sent = _msp->sendOverride(channels, evidence, monotonicSeconds() + 0.002);
		if (sent && _telemetry) _telemetry->sentRc(channels, _msp->errors());
		if (active && !sent) {
			reportFault("MSP output rejected or write failed");
			if (_msp->errors() != previous_errors) {
				_transport_healthy = false;
			}
			active = false;
		}
	} else {
		if (!active && rc_fresh && !_authority.kill && !_authority.auto_switch) {
			channels = _authority.manual_aetr;
		}
		const bool armed = rc_fresh && !_authority.kill && _authority.armed && (!_authority.auto_switch || active);
		if (!sendUdp(channels, armed)) {
			if (_transport_healthy) {
				reportFault("UDP send failed");
			}
			_transport_healthy = false;
			active = false;
		}
	}
	_override_active = active;
	if (active) {
		_reason = "Authorized output";
	} else if (!rc_fresh) {
		_reason = "RC timeout/link unavailable: SITL disarms; hardware releases override" + std::string(" ages=") +
		          std::to_string(wall - _authority_receive_time) + "," +
		          std::to_string(ros_time - stampSeconds(_authority.header.stamp));
	} else if (_authority.kill || !_authority.armed) {
		_reason = "Receiver KILL or ARM low";
	} else if (_authority.auto_switch) {
		_reason = "AUTO rejected: " + _gate.reason();
	} else {
		_reason = "Manual receiver passthrough";
	}
	publishStatus();
}

void CommandOutputNode::watchdog() {
	const auto timely = [this](double now, double sample, double limit) {
		return SafetyGate::fresh(now, sample, limit, _timing_checks);
	};
	// Short SITL stalls freeze acquisition/command age, not the independent
	// wall liveness deadline. Hardware keeps its original 25 ms deadline.
	const double wall = monotonicSeconds();
	const double ros_time = now().seconds();
	if (_simulation_time && std::isfinite(_previous_ros_time) && ros_time < _previous_ros_time) {
		// Invalidate the old epoch, but allow new lower timestamps after the
		// required healthy AUTO-low/high recovery cycle.
		_command = msg::ControlCommand();
		_previous_command_time = kUnknownTime;
		processOutput();
	}
	_previous_ros_time = ros_time;
	const double safety_now = _simulation_time ? ros_time : wall;
	if (!_timing_checks || !timely(wall, _command_receive_time, _simulation_time ? kSitlWallTimeout : 0.025) ||
	    !timely(safety_now, _command.evidence.now, 0.025)) {
		processOutput();
	} else {
		publishStatus();
	}
}

void CommandOutputNode::publishStatus() {
	msg::OutputStatus status;
	status.header.stamp = now();
	status.clock_id = _clock_id;
	status.steady_time = monotonicSeconds();
	status.transport_healthy = _transport_healthy;
	status.thrust_calibrated = _mode == "sitl" || static_cast<bool>(_thrust);
	status.fault_count = _fault_count;
	status.session_start = _session_start;
	status.override_active = _override_active;
	status.reason = _reason;
	_status_pub->publish(status);
}

bool CommandOutputNode::sendUdp(const std::array<uint16_t, 4>& channels, bool armed) {
	std::array<uint8_t, 40> bytes{};
	const double time = monotonicSeconds();
	uint64_t bits;
	static_assert(sizeof(bits) == sizeof(time));
	std::memcpy(&bits, &time, sizeof(bits));
	for (int i = 0; i < 8; ++i) {
		bytes[i] = (bits >> (8 * i)) & 255;
	}
	for (int i = 0; i < 16; ++i) {
		uint16_t value = i < 4 ? channels[i] : 1000;
		if (i == 4 && armed) {
			value = 2000;
		}
		if (value < 1000 || value > 2000) {
			value = 1000;
		}
		bytes[8 + 2 * i] = value & 255;
		bytes[9 + 2 * i] = value >> 8;
	}
	return sendto(_socket_fd, bytes.data(), bytes.size(), 0, reinterpret_cast<const sockaddr*>(&_destination), sizeof(_destination)) ==
	       static_cast<ssize_t>(bytes.size());
}

}  // namespace agi_ros2
