#include <ros/ros.h>

#include <memory>

#include "agilib/bridge/laird/laird_bridge.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros/time.hpp"
#include "agiros_msgs/Command.h"

using namespace agi;

class RosToLairdBridge {
 public:
  RosToLairdBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                   const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh), pnh_(pnh) {
    LairdParams params;


    bool check = true;
    check &= pnh_.getParam("port", params.serial_settings.port);
    check &= pnh_.getParam("baudrate", params.serial_settings.baudrate);

    if (!check) {
      logger_.info("Ground Control to Major Tom:");
      logger_.info("Your circuit’s dead, there’s something wrong!");
      logger_.error("Could not load Laird bridge parameters!");
      ros::shutdown();
      return;
    }

    params.serial_settings.serial_mode = SerialMode::Write;

    logger_ << params;

    laird_bridge_ = std::make_shared<LairdBridge>(params, RosTime);
    if (!laird_bridge_->isOpen()) {
      logger_.info("Ground Control to Major Tom:");
      logger_.info("Your circuit’s dead, there’s something wrong!");
      logger_.error("Could not start Laird bridge!");
      ros::shutdown();
      return;
    }

    command_sub_ =
      nh_.subscribe("command", 1, &RosToLairdBridge::commandCallback, this);
    arm_sub_ = nh_.subscribe("arm", 1, &RosToLairdBridge::armCallback, this);

    logger_.info("Ground Control to Major Tom:");
    logger_.info("Commencing countdown, Laird is on!");
    logger_.info("Forwarding ROS commands via Laird!");
  }


 private:
  void commandCallback(const agiros_msgs::Command& msg) {
    if (!laird_bridge_) return;
    laird_bridge_->send(fromRosCommand(msg));
  }

  void armCallback(const std_msgs::BoolConstPtr& msg) {
    if (!laird_bridge_) return;
    if (msg->data) {
      laird_bridge_->activate();
    } else {
      laird_bridge_->deactivate();
    }
  }

  Logger logger_{"RosToLaird"};

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::Subscriber arm_sub_;

  std::shared_ptr<LairdBridge> laird_bridge_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_to_laird_bridge");
  RosToLairdBridge bridge;

  ros::spin();
  return 0;
}
