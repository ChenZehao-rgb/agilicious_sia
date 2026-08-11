#include <ros/ros.h>

#include <memory>
#include <thread>

#include "agilib/bridge/ctrl/ctrl_bridge.hpp"
#include "agilib/bridge/laird/laird_bridge.hpp"
#include "agiros/bridge/ros_bridge.hpp"
#include "agiros/ctrl_feedback_publisher.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros/time.hpp"
#include "agiros_msgs/Command.h"

using namespace agi;

class LairdToCtrlBridge {
 public:
  LairdToCtrlBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                    const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh),
      pnh_(pnh),
      laird_bridge_(loadLairdParamsFile(pnh), RosTime),
      ctrl_bridge_(loadQuadrotorFile(pnh), loadCtrlParamsFile(pnh), RosTime),
      ros_bridge_(nh_, pnh_, RosTime, "command", "is_armed", 0.0, 0) {
    logger_.info("This is your captain speaking!");
    logger_.info("Checking serial bridges...");
    if (!laird_bridge_.isOpen()) {
      logger_.error("Could not start Laird bridge!");
      ros::shutdown();
      return;
    }
    if (!ctrl_bridge_.isOpen()) {
      logger_.error("Could not start CTRL bridge!");
      ros::shutdown();
      return;
    }
    logger_.info("Serial bridges ok!");

    ctrl_feedback_publisher =
      std::make_unique<CtrlFeedbackPublisher>(nh_, pnh_);

    logger_.info("Starting laird process thread...");
    laird_processing_thread_ =
      std::thread(&LairdToCtrlBridge::lairdProcessThread, this);
    logger_.info("Starting feedback process thread...");
    feedback_processing_thread_ =
      std::thread(&LairdToCtrlBridge::feedbackProcessThread, this);
    logger_.info("Process thread started! Have a nice flight!");
  }

  ~LairdToCtrlBridge() {
    should_exit_ = true;
    logger_.info(
      "This is your captain speaking again,\n"
      "thank you for flying with agilicious airlines!");
  }

 private:
  static CtrlParams loadCtrlParamsFile(const ros::NodeHandle& nh) {
    Logger logger{"LairdToCTRL"};
    std::string filename;
    CtrlParams params;
    if (!nh.getParam("ctrl_config", filename)) {
      logger.error("Could not load CTRL parameters from %s!", filename.c_str());
    } else {
      logger.info("Loading CTRL parameters from %s", filename.c_str());
      params.load(filename);
    }
    logger << params;
    return params;
  }

  static LairdParams loadLairdParamsFile(const ros::NodeHandle& nh) {
    Logger logger{"LairdToCTRL"};
    std::string filename;
    LairdParams params;
    params.serial_settings.serial_mode = SerialMode::Read;
    if (!nh.getParam("laird_config", filename)) {
      logger.error("Could not load Laird parameters from %s!",
                   filename.c_str());
    } else {
      logger.info("Loading Laird parameters from %s", filename.c_str());
      params.load(filename);
      params.serial_settings.serial_mode = SerialMode::Read;
    }
    logger << params;
    return params;
  }

  static Quadrotor loadQuadrotorFile(const ros::NodeHandle& nh) {
    Logger logger{"LairdToCTRL"};
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

  void feedbackProcessThread() {
    static Feedback feedback;
    static ros::Rate loop_rate(1000);

    while (ros::ok() && !should_exit_) {
      if (ctrl_bridge_.getFeedback(&feedback)) {
        ctrl_feedback_publisher->addFeedback(feedback);
        loop_rate.sleep();
      }
    }
  }

  void lairdProcessThread() {
    while (ros::ok() && !should_exit_) {
      static Command command;
      static bool armed{false};

      if (laird_bridge_.waitForData() &&
          laird_bridge_.receiveCommand(&command, &armed)) {
        if (armed_last_ != armed) {
          if (armed) {
            ctrl_bridge_.activate();
            ros_bridge_.activate();
          } else {
            ctrl_bridge_.deactivate();
            ros_bridge_.deactivate();
            ctrl_bridge_.reset();
          }
        }
        ctrl_bridge_.send(command);
        ros_bridge_.send(command);
        armed_last_ = armed;
      }
    }
  }

  Logger logger_{"LairdToCTRL"};

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher command_pub_;
  ros::Publisher armed_pub_;

  // CTRL feedback
  std::unique_ptr<CtrlFeedbackPublisher> ctrl_feedback_publisher;

  bool should_exit_{false};
  std::thread laird_processing_thread_;
  std::thread feedback_processing_thread_;

  bool armed_last_{false};


  LairdBridge laird_bridge_;
  CtrlBridge ctrl_bridge_;
  RosBridge ros_bridge_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laird_to_ctrl_bridge");
  LairdToCtrlBridge bridge;

  ros::spin();
  return 0;
}
