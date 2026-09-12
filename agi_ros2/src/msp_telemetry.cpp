#include "agi_ros2/msp_telemetry.h"
#include <cmath>
#include <stdexcept>

namespace agi_ros2 {
using agi::hardware::monotonicSeconds;
MspTelemetry::MspTelemetry(rclcpp::Node& node) : node_(node) {
  config_ = node.create_publisher<std_msgs::msg::String>("msp/config", rclcpp::QoS(1).reliable().transient_local());
  events_ = node.create_publisher<msg::MspEvent>("msp/events", rclcpp::QoS(1000).reliable());
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  timeout_ = node.declare_parameter<double>("msp.response_timeout_ms", 100.0, descriptor) / 1000;
  if (!std::isfinite(timeout_) || timeout_ < .001 || timeout_ > 5)
    throw std::invalid_argument("msp.response_timeout_ms must be 1..5000");
  const std::array<std::string,6> names{"attitude","rc","status","analog","battery","gps"};
  const std::array<uint8_t,6> codes{108,105,101,110,130,106};
  const std::array<double,6> rates{10,10,5,2,2,2};
  for (size_t i=0; i<names.size(); ++i) {
    const auto prefix = "msp." + names[i];
    const bool enabled = node.declare_parameter<bool>(prefix + ".enabled", true, descriptor);
    const double hz = node.declare_parameter<double>(prefix + ".rate_hz", rates[i], descriptor);
    if (!std::isfinite(hz) || hz < 0 || hz > 100)
      throw std::invalid_argument(prefix + ".rate_hz must be 0..100");
    polls_.push_back({codes[i], enabled ? hz : 0, monotonicSeconds() + .001*i, 0, false,
      node.create_publisher<msg::MspEvent>("msp/" + names[i], rclcpp::QoS(100).reliable())});
  }
}
void MspTelemetry::emit(const std::string& event, const agi::hardware::MspFrame& frame,
                        uint64_t errors, double latency) {
  msg::MspEvent message;
  message.header.stamp = node_.now();
  message.header.frame_id = "fc";
  message.steady_time = monotonicSeconds();
  message.event = event;
  message.code = frame.code;
  message.payload = frame.payload;
  message.errors = errors + timeouts_;
  message.latency_seconds = latency;
  events_->publish(message);
  if (event == "rx") for (auto& p : polls_) if (p.code == frame.code) p.publisher->publish(message);
}
void MspTelemetry::sentRc(const std::array<uint16_t,4>& channels, uint64_t errors) {
  agi::hardware::MspFrame frame{200,false,{}};
  for (auto c : channels) { frame.payload.push_back(c & 255); frame.payload.push_back(c >> 8); }
  emit("tx", frame, errors);
}
void MspTelemetry::tick(agi::hardware::BetaflightMspBridge& bridge, double write_deadline) {
  try {
    if (monotonicSeconds() >= next_config_) {
      std_msgs::msg::String config;
      for (const auto& name : node_.list_parameters({}, 10).names) {
        if (name.rfind("msp.", 0) == 0 || name == "mode" || name == "device" ||
            name == "baud" || name == "bench_aetr" || name == "props_removed")
          config.data += name + ": " + node_.get_parameter(name).value_to_string() + "\n";
      }
      config_->publish(config);
      next_config_ = monotonicSeconds() + 1;
    }
    for (int i=0; i<32; ++i) {
      agi::hardware::MspFrame frame;
      if (!bridge.receive(&frame)) break;
      std::string event = frame.error ? "error" : "rx";
      double latency = NAN;
      for (auto& p : polls_) if (p.code == frame.code) {
        if (p.pending) latency = monotonicSeconds() - p.sent;
        else event = "late";
        p.pending = false;
        if (frame.error) { p.hz = 0; healthy_ = false; }
      }
      emit(event, frame, bridge.errors(), latency);
    }
    for (auto& p : polls_) {
      const double now = monotonicSeconds();
      if (p.pending && now - p.sent >= timeout_) {
        p.pending = false;
        p.hz = 0; // Never match a delayed reply to a newer request of the same code.
        ++timeouts_;
        healthy_ = false;
        emit("timeout", {p.code,false,{}}, bridge.errors());
      }
      if (p.hz == 0 || p.pending || now < p.next || write_deadline - now < .002) continue;
      if (!bridge.sendRequest(p.code, std::min(write_deadline, now + .002)))
        throw std::runtime_error("MSP telemetry write failed");
      p.sent = monotonicSeconds();
      p.pending = true;
      p.next += (std::floor((p.sent - p.next)*p.hz)+1)/p.hz;
      emit("tx", {p.code,false,{}}, bridge.errors());
    }
  } catch (...) {
    healthy_ = false;
    emit("transport_error", {}, bridge.errors());
    throw;
  }
}
}
