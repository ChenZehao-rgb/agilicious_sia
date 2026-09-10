#ifndef AGI_ROS2_COMMAND_OUTPUT_NODE_H_
#define AGI_ROS2_COMMAND_OUTPUT_NODE_H_

#include <netinet/in.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/control_command.hpp"
#include "agi_ros2/msg/health.hpp"
#include "agi_ros2/msg/output_status.hpp"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/betaflight_rc_mapper.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/bridge/betaflight/thrust_table.hpp"
#include "agilib/bridge/betaflight_udp/betaflight_udp_bridge_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agi_ros2 {

// A single-threaded executor owns transport construction, I/O and destruction.
class CommandOutputNode final : public rclcpp::Node {
 public:
  CommandOutputNode();
  ~CommandOutputNode() override;

 private:
  void LoadThrustTable(const std::string& filename);
  void OnCommand(msg::ControlCommand::ConstSharedPtr message);
  void ProcessOutput();
  void Watchdog();
  void PublishStatus();
  void ReportFault(const std::string& reason);
  std::array<uint16_t, 4> MapCommand() const;
  bool SendUdp(const std::array<uint16_t, 4>& channels, bool armed);

  const std::string clock_id_;
  const double session_start_;
  std::string mode_;
  double mass_ = 0.0;
  int socket_fd_ = -1;
  sockaddr_in destination_{};
  agi::BetaflightUdpBridgeParams bridge_params_;
  std::unique_ptr<agi::BetaflightRcMapper> mapper_;
  std::unique_ptr<agi::hardware::BetaflightMspBridge> msp_;
  std::unique_ptr<agi::hardware::ThrustTable> thrust_;
  agi::hardware::SafetyGate gate_;
  msg::ControlCommand command_;
  msg::Authority authority_;
  msg::Health health_;
  double command_receive_time_;
  double authority_receive_time_;
  double health_receive_time_;
  double previous_command_time_;
  bool transport_healthy_ = true;
  bool override_active_ = false;
  uint64_t fault_count_ = 0;
  std::string reason_ = "Waiting for control and authority";
  rclcpp::Subscription<msg::ControlCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<msg::Authority>::SharedPtr authority_sub_;
  rclcpp::Subscription<msg::Health>::SharedPtr health_sub_;
  rclcpp::Publisher<msg::OutputStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_COMMAND_OUTPUT_NODE_H_
