#include <exception>
#include <memory>

#include "agi_ros2/fc_state_adapter_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<agi_ros2::FcStateAdapterNode>());
	} catch (const std::exception& error) {
		RCLCPP_FATAL(rclcpp::get_logger("fc_state_adapter"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
