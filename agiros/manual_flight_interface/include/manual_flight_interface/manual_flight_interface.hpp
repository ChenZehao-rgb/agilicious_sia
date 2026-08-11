#pragma once

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace manual_flight_interface {

class ManualFlightInterface {
 public:
  ManualFlightInterface(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ManualFlightInterface()
    : ManualFlightInterface(ros::NodeHandle(), ros::NodeHandle("~")) {}
  virtual ~ManualFlightInterface();

 private:
  void mainLoop(const ros::TimerEvent& time);

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  bool joypadAvailable();
  geometry_msgs::TwistStamped getVelCommandJoypad();

  void publishVelocityCommand(
    const geometry_msgs::TwistStamped& velocity_command);

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher manual_desired_velocity_pub_;
  ros::Publisher start_pub_;
  ros::Publisher land_pub_;

  ros::Subscriber joypad_sub_;

  ros::Timer main_loop_timer_;

  sensor_msgs::Joy joypad_command_;
  sensor_msgs::Joy previous_joypad_command_;
  ros::Time time_last_joypad_msg_;

  bool velocity_command_is_zero_;

  // Parameters
  double joypad_timeout_;
  double joypad_axes_zero_tolerance_;
  double vmax_xy_;
  double vmax_z_;
  double rmax_yaw_;

  // Constants
  static constexpr double kLoopFrequency_ = 50.0;
  static constexpr double kVelocityCommandZeroThreshold_ = 0.03;
};

}  // namespace manual_flight_interface
