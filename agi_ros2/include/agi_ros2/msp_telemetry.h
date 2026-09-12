#pragma once
#include <array>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "agi_ros2/msg/msp_event.hpp"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"

namespace agi_ros2 {
// Shares the output node's single-threaded serial owner. Never opens a second fd.
class MspTelemetry {
 public:
  explicit MspTelemetry(rclcpp::Node& node);
  void tick(agi::hardware::BetaflightMspBridge& bridge, double write_deadline);
  void sentRc(const std::array<uint16_t,4>& channels, uint64_t errors);
  bool healthy() const { return healthy_; }
 private:
  struct Poll {
    uint8_t code;
    double hz, next, sent{0};
    bool pending{false};
    rclcpp::Publisher<msg::MspEvent>::SharedPtr publisher;
  };
  void emit(const std::string& event, const agi::hardware::MspFrame& frame,
            uint64_t errors, double latency = NAN);
  rclcpp::Node& node_;
  std::vector<Poll> polls_;
  rclcpp::Publisher<msg::MspEvent>::SharedPtr events_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_;
  double next_config_{0};
  double timeout_;
  uint64_t timeouts_{0};
  bool healthy_{true};
};
}
