#include "manual_flight_interface/manual_flight_interface.hpp"

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "agiros/ros_eigen.hpp"
#include "manual_flight_interface/joypad_axes_buttons.hpp"
#include "manual_flight_interface/rc_channels_switches.hpp"

namespace manual_flight_interface {

ManualFlightInterface::ManualFlightInterface(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    time_last_joypad_msg_(),
    velocity_command_is_zero_(true),
    joypad_timeout_(0.5),
    joypad_axes_zero_tolerance_(0.05),
    vmax_xy_(1.5),
    vmax_z_(0.7),
    rmax_yaw_(1.5) {
  loadParameters();

  joypad_command_ = sensor_msgs::Joy();
  joypad_command_.axes = std::vector<float>(8, 0);
  joypad_command_.buttons = std::vector<int32_t>(8, 0);
  previous_joypad_command_ = joypad_command_;

  manual_desired_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
    "agiros_pilot/velocity_command", 1);
  start_pub_ = nh_.advertise<std_msgs::Empty>("agiros_pilot/start", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("agiros_pilot/land", 1);

  joypad_sub_ =
    nh_.subscribe("joy", 1, &ManualFlightInterface::joyCallback, this);

  main_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / kLoopFrequency_),
                                     &ManualFlightInterface::mainLoop, this);
}

ManualFlightInterface::~ManualFlightInterface() {}


geometry_msgs::TwistStamped ManualFlightInterface::getVelCommandJoypad() {
  geometry_msgs::TwistStamped velocity_command;
  velocity_command.header.stamp = ros::Time::now();

  if (fabs(joypad_command_.axes[joypad::axes::kX]) >
      joypad_axes_zero_tolerance_) {
    velocity_command.twist.linear.x =
      vmax_xy_ * joypad_command_.axes[joypad::axes::kX];
  }
  if (fabs(joypad_command_.axes[joypad::axes::kY]) >
      joypad_axes_zero_tolerance_) {
    velocity_command.twist.linear.y =
      vmax_xy_ * joypad_command_.axes[joypad::axes::kY];
  }
  if (fabs(joypad_command_.axes[joypad::axes::kZ]) >
      joypad_axes_zero_tolerance_) {
    velocity_command.twist.linear.z =
      vmax_z_ * joypad_command_.axes[joypad::axes::kZ];
  }
  if (fabs(joypad_command_.axes[joypad::axes::kYaw]) >
      joypad_axes_zero_tolerance_) {
    velocity_command.twist.angular.z =
      rmax_yaw_ * joypad_command_.axes[joypad::axes::kYaw];
  }
  return velocity_command;
}

void ManualFlightInterface::mainLoop(const ros::TimerEvent& time) {
  geometry_msgs::TwistStamped velocity_command;
  if (joypadAvailable()) {
    velocity_command = getVelCommandJoypad();

    // Start and Land Buttons
    if (!previous_joypad_command_.buttons.empty()) {
      if (joypad_command_.buttons[joypad::buttons::kGreen] &&
          !previous_joypad_command_.buttons[joypad::buttons::kGreen]) {
        start_pub_.publish(std_msgs::Empty());
      }
      if (joypad_command_.buttons[joypad::buttons::kBlue] &&
          !previous_joypad_command_.buttons[joypad::buttons::kBlue]) {
        land_pub_.publish(std_msgs::Empty());
      }
    }
  } else {
    // Publish zero velocity command
    velocity_command.header.stamp = ros::Time::now();
  }

  publishVelocityCommand(velocity_command);
}

void ManualFlightInterface::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  previous_joypad_command_ = joypad_command_;
  joypad_command_ = *msg;
  time_last_joypad_msg_ = ros::Time::now();
}


bool ManualFlightInterface::joypadAvailable() {
  if ((ros::Time::now() - time_last_joypad_msg_) >
      ros::Duration(joypad_timeout_)) {
    return false;
  }

  return true;
}

void ManualFlightInterface::publishVelocityCommand(
  const geometry_msgs::TwistStamped& velocity_command) {
  if (agi::fromRosVec3(velocity_command.twist.linear).norm() <=
        kVelocityCommandZeroThreshold_ &&
      fabs(velocity_command.twist.angular.z) <=
        kVelocityCommandZeroThreshold_) {
    if (!velocity_command_is_zero_) {
      velocity_command_is_zero_ = true;
      // Publish one zero velocity command afterwards stop publishing
      const geometry_msgs::TwistStamped zero_command;
      manual_desired_velocity_pub_.publish(zero_command);
    }
  } else if (velocity_command_is_zero_) {
    velocity_command_is_zero_ = false;
  }

  if (!velocity_command_is_zero_) {
    manual_desired_velocity_pub_.publish(velocity_command);
  }
}

bool ManualFlightInterface::loadParameters() {
  if (!pnh_.getParam("joypad_timeout", joypad_timeout_)) {
    return false;
  }

  if (!pnh_.getParam("joypad_axes_zero_tolerance",
                     joypad_axes_zero_tolerance_)) {
    return false;
  }

  if (!pnh_.getParam("vmax_xy", vmax_xy_)) {
    return false;
  }

  if (!pnh_.getParam("vmax_z", vmax_z_)) {
    return false;
  }

  if (!pnh_.getParam("rmax_yaw", rmax_yaw_)) {
    return false;
  }

  return true;
}

}  // namespace manual_flight_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_flight_interface");
  manual_flight_interface::ManualFlightInterface manual_flight_interface;

  ros::spin();
  return 0;
}
