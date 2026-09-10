#ifndef AGI_ROS2_STATE_FUSION_NODE_H_
#define AGI_ROS2_STATE_FUSION_NODE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/rtk.hpp"
#include "agilib/types/quad_state.hpp"
#include "companion_ahrs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace agi_ros2 {

// A single-threaded executor owns the AHRS and the propagated state.
class StateFusionNode final : public rclcpp::Node {
 public:
  StateFusionNode();

 private:
  void OnRtk(msg::Rtk::ConstSharedPtr message);
  void OnImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void Reset();
  void PublishState(double imu_receive_time);

  const std::string clock_id_;
  agi::QuadState state_;
  std::unique_ptr<agi::CompanionAhrs> ahrs_;
  msg::Rtk rtk_;
  double rtk_receive_time_;
  double last_rtk_time_;
  uint64_t reset_counter_ = 0;
  rclcpp::Subscription<msg::Rtk>::SharedPtr rtk_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<msg::FusedState>::SharedPtr fused_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_STATE_FUSION_NODE_H_
