#ifndef AGI_ROS2_NODE_COMMON_H_
#define AGI_ROS2_NODE_COMMON_H_

#include <string>

#include "agi_ros2/msg/safety_evidence.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agi_ros2 {

// SITL tolerates brief simulator/scheduler stalls, but never an unbounded
// command-stream outage. This wall deadline is independent of /clock.
constexpr double kSitlWallTimeout = 0.250;

// Cross-process steady timestamps are only valid on the same Linux boot.
std::string readClockId();
// Explicitly distinguish simulated safety evidence from hardware steady time.
std::string evidenceClockId(const std::string& host_id, bool simulation);
double stampSeconds(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time rosStamp(double seconds);
// /clock and data use independent DDS streams. Bound the alignment wait.
double alignedRosTime(rclcpp::Node& node, double newest);
msg::SafetyEvidence encodeEvidence(const agi::hardware::Evidence& evidence);
agi::hardware::Evidence decodeEvidence(const msg::SafetyEvidence& message);

}  // namespace agi_ros2

#endif  // AGI_ROS2_NODE_COMMON_H_
