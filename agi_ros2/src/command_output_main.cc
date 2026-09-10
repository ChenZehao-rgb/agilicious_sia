#include <exception>
#include <memory>

#include "agi_ros2/command_output_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<agi_ros2::CommandOutputNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("command_output"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
