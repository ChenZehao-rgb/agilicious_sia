#ifndef AGI_ROS2_NODE_COMMON_H_
#define AGI_ROS2_NODE_COMMON_H_

#include <string>

#include "agi_ros2/msg/safety_evidence.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agi_ros2 {

// Cross-process steady timestamps are only valid on the same Linux boot.
std::string ReadClockId();
double StampSeconds(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time RosStamp(double seconds);
// /clock and data use independent DDS streams. Bound the alignment wait.
double AlignedRosTime(rclcpp::Node& node, double newest);
msg::SafetyEvidence EncodeEvidence(const agi::hardware::Evidence& evidence);
agi::hardware::Evidence DecodeEvidence(const msg::SafetyEvidence& message);

}  // namespace agi_ros2

#endif  // AGI_ROS2_NODE_COMMON_H_
