#ifndef AGI_ROS2_CONTROL_NODE_H_
#define AGI_ROS2_CONTROL_NODE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/control_command.hpp"
#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/health.hpp"
#include "agi_ros2/msg/output_status.hpp"
#include "agilib/pilot/hardware_pilot.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace agi_ros2 {

// Owns trajectory tracking and the authority FSM; never opens a transport.
class ControlNode final : public rclcpp::Node {
 public:
  ControlNode();

 private:
  void OnState(msg::FusedState::ConstSharedPtr message);
  void OnOutputStatus(msg::OutputStatus::ConstSharedPtr message);
  void Tick();
  void PublishDecision(const agi::hardware::ControlDecision& decision,
                       const agi::QuadState& state);

  const std::string clock_id_;
  std::string mode_;
  std::unique_ptr<agi::PilotParams> params_;
  std::unique_ptr<agi::hardware::HardwarePilot> pilot_;
  double control_time_ = 0.0;
  double previous_clock_;
  double authority_receive_time_;
  double health_receive_time_;
  double output_receive_time_;
  bool output_fault_ = false;
  uint64_t sequence_ = 0;
  msg::FusedState state_;
  msg::Authority authority_;
  msg::Health health_;
  msg::OutputStatus output_;
  rclcpp::Subscription<msg::FusedState>::SharedPtr state_sub_;
  rclcpp::Subscription<msg::Authority>::SharedPtr authority_sub_;
  rclcpp::Subscription<msg::Health>::SharedPtr health_sub_;
  rclcpp::Subscription<msg::OutputStatus>::SharedPtr output_sub_;
  rclcpp::Publisher<msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      diagnostic_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_CONTROL_NODE_H_
