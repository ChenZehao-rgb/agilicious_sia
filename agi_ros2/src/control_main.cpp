#include <exception>
#include <memory>

#include "agi_ros2/control_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<agi_ros2::ControlNode>());
	} catch (const std::exception& error) {
		RCLCPP_FATAL(rclcpp::get_logger("flight_control"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
