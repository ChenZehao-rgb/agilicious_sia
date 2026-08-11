#include <ros/ros.h>

#include <memory>

#include "agilib/bridge/ctrl/ctrl_bridge.hpp"
#include "agiros/ctrl_feedback_publisher.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros/time.hpp"
#include "agiros_msgs/Command.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

using namespace agi;

class RosToCtrlBridge {
 public:
  RosToCtrlBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                  const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh),
      pnh_(pnh),
      ctrl_bridge_(loadQuadrotorFile(pnh), loadCtrlParamsFile(pnh), RosTime) {
    CtrlParams params;

    if (!ctrl_bridge_.isOpen()) {
      logger_.info("Ground Control to Major Tom:");
      logger_.info("Your circuit’s dead, there’s something wrong!");
      logger_.error("Could not start Ctrl bridge!");
      ros::shutdown();
      return;
    }

    command_sub_ =
      nh_.subscribe("command", 1, &RosToCtrlBridge::commandCallback, this);
    arm_sub_ = nh_.subscribe("arm", 1, &RosToCtrlBridge::armCallback, this);
    ctrl_feedback_publisher =
      std::make_unique<CtrlFeedbackPublisher>(nh_, pnh_);
    ctrl_bridge_.registerFeedbackCallback(
      std::bind(&CtrlFeedbackPublisher::addFeedback,
                ctrl_feedback_publisher.get(), std::placeholders::_1));


    logger_.info("Ground Control to Major Tom:");
    logger_.info("Commencing countdown, Ctrl is on!");
    logger_.info("Forwarding ROS commands via Ctrl!");
  }

 private:
  void commandCallback(const agiros_msgs::Command& msg) {
    // if (!ctrl_bridge_) return;
    agi::Command command = fromRosCommand(msg);
    command.t = ros::Time::now().toSec();
    ctrl_bridge_.send(command);
  }

  void armCallback(const std_msgs::BoolConstPtr& msg) {
    // if (!ctrl_bridge_) return;
    if (msg->data) {
      ctrl_bridge_.activate();
    } else {
      ctrl_bridge_.deactivate();
      ctrl_bridge_.reset();
    }
  }

  static Quadrotor loadQuadrotorFile(const ros::NodeHandle& nh) {
    Logger logger{"RosToCtrl"};
    std::string filename;
    Quadrotor quad;
    if (!nh.getParam("quadrotor", filename)) {
      logger.error("Could not load quadrotor parameters from %s!",
                   filename.c_str());
    } else {
      logger.info("Loading quadrotor from %s", filename.c_str());
      quad.load(filename);
    }
    if (!quad.valid()) {
      logger.error("Quadrotor parameters not valid!");
    }

    return quad;
  }

  static CtrlParams loadCtrlParamsFile(const ros::NodeHandle& nh) {
    Logger logger{"RosToCtrl"};
    std::string filename;
    CtrlParams params;
    params.serial_settings.serial_mode = SerialMode::ReadWrite;
    if (!nh.getParam("ctrl_config", filename)) {
      logger.error("Could not load Ctrl parameters from %s!", filename.c_str());
    } else {
      logger.info("Loading Ctrl parameters from %s", filename.c_str());
      params.load(filename);
      // params.serial_settings.serial_mode = SerialMode::Read;
    }
    logger << params;
    return params;
  }

  Logger logger_{"RosToCtrl"};

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::Subscriber arm_sub_;

  CtrlBridge ctrl_bridge_;

  // CTRL feedback
  std::unique_ptr<CtrlFeedbackPublisher> ctrl_feedback_publisher;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_to_ctrl_bridge");
  RosToCtrlBridge bridge;

  ros::spin();

  return 0;
}
