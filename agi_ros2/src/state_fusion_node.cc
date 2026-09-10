#include "agi_ros2/state_fusion_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include "agi_ros2/node_common.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

StateFusionNode::StateFusionNode()
    : Node("state_fusion"),
      clock_id_(ReadClockId()),
      rtk_receive_time_(kUnknownTime),
      last_rtk_time_(kUnknownTime) {
  Reset();
  fused_pub_ = create_publisher<msg::FusedState>("fused_state", 1);
  state_pub_ = create_publisher<nav_msgs::msg::Odometry>("state", 1);
  rtk_sub_ = create_subscription<msg::Rtk>(
      "sensors/rtk", 10,
      std::bind(&StateFusionNode::OnRtk, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "sensors/imu", rclcpp::SensorDataQoS().keep_last(256),
      std::bind(&StateFusionNode::OnImu, this, std::placeholders::_1));
}

void StateFusionNode::Reset() {
  ahrs_ = std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
  state_.setZero();
  state_.t = kUnknownTime;
  last_rtk_time_ = kUnknownTime;
  ++reset_counter_;
}

void StateFusionNode::OnRtk(msg::Rtk::ConstSharedPtr message) {
  rtk_ = *message;
  rtk_receive_time_ = monotonicSeconds();
}

void StateFusionNode::OnImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
  const double received = monotonicSeconds();
  const double time = StampSeconds(message->header.stamp);
  if (message->header.frame_id != "base_link") {
    return;
  }
  const agi::ImuSample imu(
      time,
      {message->linear_acceleration.x, message->linear_acceleration.y,
       message->linear_acceleration.z},
      {message->angular_velocity.x, message->angular_velocity.y,
       message->angular_velocity.z});
  if (!imu.valid()) {
    return;
  }
  if (std::isfinite(state_.t)) {
    if (time == state_.t) {
      return;
    }
    if (time < state_.t || time - state_.t > 0.025) {
      if (time < state_.t) {
        rtk_ = msg::Rtk();
        rtk_receive_time_ = kUnknownTime;
      }
      Reset();
    }
  }

  const double fix_time = StampSeconds(rtk_.header.stamp);
  if (SafetyGate::fresh(received, rtk_receive_time_, 0.3) &&
      SafetyGate::fresh(time, fix_time, 0.3) &&
      rtk_.header.frame_id == "odom" && rtk_.fixed && rtk_.heading_valid &&
      rtk_.accuracy_ok && std::isfinite(rtk_.heading) &&
      (!std::isfinite(last_rtk_time_) || fix_time > last_rtk_time_)) {
    const agi::Vector<3> position(rtk_.position.x, rtk_.position.y,
                                  rtk_.position.z);
    const agi::Vector<3> velocity(rtk_.velocity.x, rtk_.velocity.y,
                                  rtk_.velocity.z);
    if (position.allFinite() && velocity.allFinite()) {
      // Preserve the existing algorithm for this architectural split. This
      // constant-velocity extrapolation is NOT delayed-measurement IMU replay.
      const double state_time = std::isfinite(state_.t) ? state_.t : fix_time;
      state_.p = position + velocity * std::max(0.0, state_time - fix_time);
      state_.v = velocity;
      ahrs_->setHeading(rtk_.heading);
      ahrs_->setVelocity(velocity, fix_time);
      last_rtk_time_ = fix_time;
    }
  }

  ahrs_->addImu(imu);
  if (!ahrs_->initialized()) {
    return;
  }
  const double dt = std::isfinite(state_.t) ? time - state_.t : 0.0;
  state_.q(ahrs_->attitude());
  state_.w = imu.omega - ahrs_->gyroBias();
  state_.a = state_.q() * imu.acc + agi::Vector<3>(0, 0, -9.8066);
  if (dt > 0 && dt <= 0.025 && std::isfinite(last_rtk_time_)) {
    state_.p += state_.v * dt + 0.5 * state_.a * dt * dt;
    state_.v += state_.a * dt;
  }
  state_.t = time;
  PublishState(received);
}

void StateFusionNode::PublishState(double imu_receive_time) {
  msg::FusedState out;
  out.header.stamp = RosStamp(state_.t);
  out.header.frame_id = "odom";
  out.position.x = state_.p.x();
  out.position.y = state_.p.y();
  out.position.z = state_.p.z();
  out.velocity.x = state_.v.x();
  out.velocity.y = state_.v.y();
  out.velocity.z = state_.v.z();
  out.orientation.w = state_.q().w();
  out.orientation.x = state_.q().x();
  out.orientation.y = state_.q().y();
  out.orientation.z = state_.q().z();
  out.body_rates.x = state_.w.x();
  out.body_rates.y = state_.w.y();
  out.body_rates.z = state_.w.z();
  out.acceleration.x = state_.a.x();
  out.acceleration.y = state_.a.y();
  out.acceleration.z = state_.a.z();
  out.clock_id = clock_id_;
  out.published_steady_time = monotonicSeconds();
  out.imu_receive_time = imu_receive_time;
  out.rtk_receive_time = rtk_receive_time_;
  if (std::isfinite(last_rtk_time_)) {
    out.rtk_stamp = RosStamp(last_rtk_time_);
  }
  out.reset_counter = reset_counter_;
  out.initialized = state_.valid() && std::isfinite(last_rtk_time_);
  out.rtk_fixed = rtk_.fixed;
  out.heading_valid = rtk_.heading_valid;
  out.accuracy_ok = rtk_.accuracy_ok;
  out.synchronized = rtk_.synchronized;
  fused_pub_->publish(out);

  // Keep the established evaluation topic, with standard body-frame twist.
  nav_msgs::msg::Odometry odometry;
  odometry.header = out.header;
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position = out.position;
  odometry.pose.pose.orientation = out.orientation;
  const auto velocity_body = state_.q().conjugate() * state_.v;
  odometry.twist.twist.linear.x = velocity_body.x();
  odometry.twist.twist.linear.y = velocity_body.y();
  odometry.twist.twist.linear.z = velocity_body.z();
  odometry.twist.twist.angular = out.body_rates;
  state_pub_->publish(odometry);
}

}  // namespace agi_ros2
