#include <ros/ros.h>

#include "agilib/bridge/msg_encoding.hpp"
#include "agilib/bridge/sbus_bridge.hpp"
#include "agilib/serial/serial.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"

using namespace agi;
using namespace srl;

int main(int argc, char **argv) {
  ros::init(argc, argv, "Sbus");
  ros::start();

  SbusParams settings("/dev/ttyUSB0");

  Quadrotor quad(2.0, 0.15);
  quad.thrust_max_ = 8;
  quad.thrust_min_ = 0.2;

  SbusBridge bridge(quad, settings);
  if (!bridge.isOpen()) ROS_ERROR("Could not open settings!");

  const Scalar min_thrust = 0.2;
  const Scalar max_rampup_thrust = 3.0;
  const Command zero_command(0.0, 0.0, Vector<3>::Zero());
  const Command min_command(0.0, min_thrust, Vector<3>::Zero());

  const Scalar dt = 0.02;
  const ros::Duration ros_dt(dt);
  const Scalar test_period = 5.0;

  ros::Duration(2.0).sleep();

  ROS_INFO("Activate!");
  bridge.reset();
  bridge.activate();

  ROS_INFO("Zero Command!");
  for (Scalar t = 0.0; t < test_period && ros::ok(); t += dt) {
    if (!bridge.send(zero_command)) ROS_WARN("Could not send!");
    ros::spinOnce();
    ros_dt.sleep();
  }

  ROS_INFO("Min Command!");
  for (Scalar t = 0.0; t < test_period && ros::ok(); t += dt) {
    if (!bridge.send(min_command)) ROS_WARN("Could not send!");
    ros::spinOnce();
    ros_dt.sleep();
  }

  ROS_INFO("RampUp Command!");
  for (Scalar t = 0.0; t < test_period && ros::ok(); t += dt) {
    const Scalar thrust =
      min_thrust + t / test_period * (max_rampup_thrust - min_thrust);
    if (!bridge.send(Command(0.0, thrust, Vector<3>::Zero())))
      ROS_WARN("Could not send!");
    ros::spinOnce();
    ros_dt.sleep();
  }

  ROS_INFO("RampDown Command!");
  for (Scalar t = 0.0; t < test_period && ros::ok(); t += dt) {
    const Scalar thrust =
      max_rampup_thrust - t / test_period * (max_rampup_thrust - min_thrust);
    if (!bridge.send(Command(0.0, thrust, Vector<3>::Zero())))
      ROS_WARN("Could not send!");
    ros::spinOnce();
    ros_dt.sleep();
  }

  ROS_INFO("Trigger Timeout by waiting 3 seconds!");
  ros::spinOnce();
  ros::Duration(3.0).sleep();

  if (bridge.active()) {
    ROS_WARN("Bridge still active!");
    ros::spinOnce();
    ros::shutdown();
    return 0;
  } else {
    ROS_INFO("Bridge deactivated!");
    ros::spinOnce();
  }

  if (!bridge.locked()) {
    ROS_WARN("Bridge did not lock!");
    ros::spinOnce();
    ros::shutdown();
    return 0;
  } else {
    ROS_INFO("Bridge locked!");
    ros::spinOnce();
  }

  ROS_INFO("Resetting Bridge");
  ros::spinOnce();
  bridge.reset();

  if (bridge.locked()) {
    ROS_WARN("Could not reset bridge!");
    ros::spinOnce();
    ros::shutdown();
    return 0;
  } else {
    ROS_INFO("Bridge Reset!");
    ros::spinOnce();
  }

  ROS_INFO("Arm again and spinn 5 sec.");
  ros::spinOnce();
  bridge.activate();

  for (Scalar t = 0.0; t < test_period && ros::ok(); t += dt) {
    if (!bridge.send(min_command)) ROS_WARN("Could not send!");
    ros::spinOnce();
    ros_dt.sleep();
  }

  ROS_INFO("Deactivate!");
  ros::spinOnce();
  bridge.deactivate();

  if (bridge.active()) {
    ROS_WARN("Could not deactivate bridge!");
    ros::spinOnce();
    ros::shutdown();
    return 0;
  } else {
    ROS_INFO("Bridge deactivated!");
    ros::spinOnce();
  }

  ROS_INFO("Done!");
  ros::shutdown();

  return 0;
}
