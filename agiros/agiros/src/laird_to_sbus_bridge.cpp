#include <ros/ros.h>

#include <thread>

#include "agilib/bridge/laird/laird_bridge.hpp"
#include "agilib/bridge/sbus/sbus_bridge.hpp"
#include "agiros/bridge/ros_bridge.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros/time.hpp"
#include "agiros_msgs/Command.h"

using namespace agi;

class LairdToSbusBridge {
 public:
  LairdToSbusBridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                    const ros::NodeHandle& pnh = ros::NodeHandle("~"))
    : nh_(nh),
      pnh_(pnh),
      laird_bridge_(loadLairdParamsFile(pnh), RosTime),
      sbus_bridge_(loadQuadrotorFile(pnh), loadSbusParamsFile(pnh), RosTime),
      ros_bridge_(nh_, pnh_, RosTime, "command", "is_armed", 0.0, 0) {
    logger_.info("This is your captain speaking!");
    logger_.info("Checking serial bridges...");
    if (!laird_bridge_.isOpen()) {
      logger_.error("Could not start Laird bridge!");
      ros::shutdown();
      return;
    }
    if (!sbus_bridge_.isOpen()) {
      logger_.error("Could not start SBUS bridge!");
      ros::shutdown();
      return;
    }
    logger_.info("Serial bridges ok!");

    logger_.info("Starting process thread...");
    processing_thread_ = std::thread(&LairdToSbusBridge::processThread, this);
    logger_.info("Process thread started! Have a nice flight!");
  }

  ~LairdToSbusBridge() {
    should_exit_ = true;
    logger_.info(
      "This is your captain speaking again,\n"
      "thank you for flying with agilicious airlines!");
  }

 private:
  static SbusParams loadSbusParamsFile(const ros::NodeHandle& nh) {
    Logger logger{"LairdToSBUS"};
    std::string filename;
    SbusParams params;
    if (!nh.getParam("sbus_config", filename)) {
      logger.error("Could not load SBUS parameters from %s!", filename.c_str());
    } else {
      logger.info("Loading SBUS parameters from %s", filename.c_str());
      params.load(filename);
    }
    logger << params;
    return params;
  }

  static LairdParams loadLairdParamsFile(const ros::NodeHandle& nh) {
    Logger logger{"LairdToSBUS"};
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
    Logger logger{"LairdToSBUS"};
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

  void processThread() {
    while (ros::ok() && !should_exit_) {
      static Command command;
      static bool armed{false};
      if (laird_bridge_.waitForData() &&
          laird_bridge_.receiveCommand(&command, &armed)) {
        if (armed_last_ != armed) {
          if (armed) {
            sbus_bridge_.activate();
            ros_bridge_.activate();
          } else {
            sbus_bridge_.deactivate();
            ros_bridge_.deactivate();
            sbus_bridge_.reset();
          }
        }
        sbus_bridge_.send(command);
        ros_bridge_.send(command);
        armed_last_ = armed;
      }
    }
  }

  Logger logger_{"LairdToSBUS"};

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher command_pub_;
  ros::Publisher armed_pub_;

  bool should_exit_{false};
  std::thread processing_thread_;

  bool armed_last_{false};


  LairdBridge laird_bridge_;
  SbusBridge sbus_bridge_;
  RosBridge ros_bridge_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laird_to_sbus_bridge");
  LairdToSbusBridge bridge;

  ros::spin();
  return 0;
}
