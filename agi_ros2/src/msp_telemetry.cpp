#include "agi_ros2/msp_telemetry.h"

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace agi_ros2 {
using agi::hardware::monotonicSeconds;
MspTelemetry::MspTelemetry(rclcpp::Node& node) : _node(node) {
	_config = node.create_publisher<std_msgs::msg::String>("msp/config", rclcpp::QoS(1).reliable().transient_local());
	_events = node.create_publisher<msg::MspEvent>("msp/events", rclcpp::QoS(1000).reliable());
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	descriptor.read_only = true;
	_timeout = node.declare_parameter<double>("msp.response_timeout_ms", 100.0, descriptor) / 1000;
	if (!std::isfinite(_timeout) || _timeout < .001 || _timeout > 5)
		throw std::invalid_argument("msp.response_timeout_ms must be 1..5000");
	std::ifstream boot_file("/proc/sys/kernel/random/boot_id");
	std::string boot_id;
	if (!std::getline(boot_file, boot_id) || boot_id.empty()) throw std::runtime_error("Cannot identify MSP clock host");
	_session_id = boot_id + ":" + std::to_string(monotonicSeconds());
	const std::array<std::string, 6> names{"attitude", "rc", "status", "analog", "battery", "gps"};
	const std::array<uint8_t, 6> codes{108, 105, 150, 110, 130, 106};
	const std::array<double, 6> rates{10, 10, 5, 2, 2, 2};
	for (size_t i = 0; i < names.size(); ++i) {
		const auto prefix = "msp." + names[i];
		const bool enabled = node.declare_parameter<bool>(prefix + ".enabled", true, descriptor);
		const double hz = node.declare_parameter<double>(prefix + ".rate_hz", rates[i], descriptor);
		if (!std::isfinite(hz) || hz < 0 || hz > 100) throw std::invalid_argument(prefix + ".rate_hz must be 0..100");
		_polls.push_back({codes[i], enabled ? hz : 0, monotonicSeconds() + .001 * i, 0, false,
		                  node.create_publisher<msg::MspEvent>("msp/" + names[i], rclcpp::QoS(100).reliable()), "",
		                  builtin_interfaces::msg::Time{}, codes[i] == 105 || codes[i] == 150 || codes[i] == 130});
	}
	if (node.declare_parameter<bool>("msp.read_configuration", false, descriptor)) {
		for (uint16_t code : {1, 2, 3, 34, 238, 64, 44, 119, 111, 125}) {
			_polls.push_back({code, 1., monotonicSeconds(), 0, false, nullptr, "", builtin_interfaces::msg::Time{}, true});
		}
		for (const std::string setting : {"msp_override_channels_mask", "msp_override_failsafe", "msp_override_timeout_ms"}) {
			_polls.push_back(
			        {0x3010, 1., monotonicSeconds(), 0, false, nullptr, setting, builtin_interfaces::msg::Time{}, true});
		}
	}
}
void MspTelemetry::emit(const std::string& event, const agi::hardware::MspFrame& frame, uint64_t errors, double latency,
                        const Poll* request) {
	msg::MspEvent message;
	message.header.stamp = _node.now();
	message.header.frame_id = "fc";
	message.steady_time = monotonicSeconds();
	message.event = event;
	message.code = frame.code;
	message.payload = frame.payload;
	message.errors = errors + _timeouts;
	message.latency_seconds = latency;
	message.session_id = _session_id;
	if (request) {
		message.request_stamp = request->stamp;
		message.request_steady_time = request->sent;
		message.request_name = request->setting;
	}
	_events->publish(message);
	if (event == "rx")
		for (auto& p : _polls)
			if (p.code == frame.code && p.publisher) p.publisher->publish(message);
}
void MspTelemetry::sentRc(const std::array<uint16_t, 4>& channels, uint64_t errors) {
	agi::hardware::MspFrame frame{200, false, {}};
	for (auto c : channels) {
		frame.payload.push_back(c & 255);
		frame.payload.push_back(c >> 8);
	}
	emit("tx", frame, errors);
}
void MspTelemetry::tick(agi::hardware::BetaflightMspBridge& bridge, double write_deadline) {
	try {
		if (monotonicSeconds() >= _next_config) {
			std_msgs::msg::String config;
			for (const auto& name : _node.list_parameters({}, 10).names) {
				if (name.rfind("msp.", 0) == 0 || name == "mode" || name == "device" || name == "baud" ||
				    name == "bench_aetr" || name == "props_removed")
					config.data += name + ": " + _node.get_parameter(name).value_to_string() + "\n";
			}
			_config->publish(config);
			_next_config = monotonicSeconds() + 1;
		}
		for (int i = 0; i < 32; ++i) {
			agi::hardware::MspFrame frame;
			if (!bridge.receive(&frame)) break;
			std::string event = frame.error ? "error" : "rx";
			double latency = NAN;
			Poll* matched = nullptr;
			for (auto& p : _polls) {
				if (p.code != frame.code || !p.pending) continue;
				matched = &p;
				latency = monotonicSeconds() - p.sent;
				p.pending = false;
				if (frame.error) {
					p.hz = 0;
					if (p.critical)
						_healthy = false;
					else
						event = "optional_error";
				}
				break;
			}
			if (!matched) {
				event = frame.code == 200 ? (frame.error ? "error" : "ack") : "late";
				if (frame.code == 200 && frame.error) _healthy = false;
			}
			emit(event, frame, bridge.errors(), latency, matched);
		}
		for (auto& p : _polls) {
			const double now = monotonicSeconds();
			if (p.pending && now - p.sent >= _timeout) {
				p.pending = false;
				p.hz = 0;  // Never match a delayed reply to a newer request of the same code.
				++_timeouts;
				if (p.critical) _healthy = false;
				emit(p.critical ? "timeout" : "optional_timeout", {p.code, false, {}}, bridge.errors(), NAN, &p);
				for (auto& other : _polls)
					if (other.code == p.code) other.hz = 0;
			}
			if (p.hz == 0 || p.pending || now < p.next || write_deadline - now < .002) continue;
			bool occupied = false;
			for (const auto& other : _polls)
				if (other.code == p.code && other.pending) occupied = true;
			if (occupied) continue;
			p.stamp = _node.now();
			p.sent = monotonicSeconds();
			const bool sent = p.setting.empty()
			                          ? bridge.sendRequest(static_cast<uint8_t>(p.code), std::min(write_deadline, now + .002))
			                          : bridge.readOverrideSetting(p.setting, std::min(write_deadline, now + .002));
			if (!sent) throw std::runtime_error("MSP telemetry write failed");
			p.pending = true;
			p.next += (std::floor((p.sent - p.next) * p.hz) + 1) / p.hz;
			emit("tx", {p.code, false, {}}, bridge.errors(), NAN, &p);
		}
	} catch (...) {
		_healthy = false;
		emit("transport_error", {}, bridge.errors());
		throw;
	}
}
}  // namespace agi_ros2
