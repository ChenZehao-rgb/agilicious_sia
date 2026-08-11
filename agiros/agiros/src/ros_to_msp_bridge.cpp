#include <ros/ros.h>

#include <memory>

#include "agilib/bridge/msp/msp_bridge.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros/time.hpp"
#include "agiros_msgs/Command.h"

using namespace agi;

class RosToMspBridge {
 public:
  RosToMspBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                 const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh), pnh_(pnh) {
    MspParams params;
    Quadrotor quad;


    bool check = true;
    check &= pnh_.getParam("port", params.serial_settings.port);
    check &= pnh_.getParam("baudrate", params.serial_settings.baudrate);

    if (!check) {
      logger_.info("Ground Control to Major Tom:");
      logger_.info("Your circuit’s dead, there’s something wrong!");
      logger_.error("Could not load Msp bridge parameters!");
      ros::shutdown();
      return;
    }

    params.serial_settings.serial_mode = SerialMode::ReadWrite;

    logger_ << params;

    msp_bridge_ = std::make_shared<MspBridge>(quad, params, RosTime);
    if (!msp_bridge_->isOpen()) {
      logger_.info("Ground Control to Major Tom:");
      logger_.info("Your circuit’s dead, there’s something wrong!");
      logger_.error("Could not start Msp bridge!");
      ros::shutdown();
      return;
    }

    command_sub_ =
      nh_.subscribe("command", 1, &RosToMspBridge::commandCallback, this);
    arm_sub_ = nh_.subscribe("arm", 1, &RosToMspBridge::armCallback, this);

    logger_.info("Ground Control to Major Tom:");
    logger_.info("Commencing countdown, Msp is on!");
    logger_.info("Forwarding ROS commands via Msp!");
  }


 private:
  void commandCallback(const agiros_msgs::Command& msg) {
    if (!msp_bridge_) return;
    msp_bridge_->send(fromRosCommand(msg));
  }

  void armCallback(const std_msgs::BoolConstPtr& msg) {
    if (!msp_bridge_) return;
    if (msg->data) {
      msp_bridge_->activate();
    } else {
      msp_bridge_->deactivate();
    }
  }

  Logger logger_{"RosToMsp"};

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::Subscriber arm_sub_;

  std::shared_ptr<MspBridge> msp_bridge_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_to_msp_bridge");
  RosToMspBridge bridge;

  ros::spin();
  return 0;
}
