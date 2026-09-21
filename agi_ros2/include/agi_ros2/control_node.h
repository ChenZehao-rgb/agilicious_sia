#ifndef AGI_ROS2_CONTROL_NODE_H_
#define AGI_ROS2_CONTROL_NODE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/computation_status.hpp"
#include "agi_ros2/msg/control_command.hpp"
#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/health.hpp"
#include "agi_ros2/msg/output_status.hpp"
#include "agilib/pilot/hardware_pilot.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace agi_ros2 {

// Owns trajectory tracking and the authority FSM; never opens a transport.
class ControlNode final : public rclcpp::Node {
public:
	ControlNode();

private:
	void onState(msg::FusedState::ConstSharedPtr message);
	void onOutputStatus(msg::OutputStatus::ConstSharedPtr message);
	void tick();
	void publishDecision(const agi::hardware::ControlDecision& decision, const agi::QuadState& state);

	const std::string _clock_id;
	std::string _mode;
	bool _shadow_only = false;
	bool _simulation_time = false;
	bool _timing_checks = true;
	agi::hardware::NavigationPolicy _navigation_policy = agi::hardware::NavigationPolicy::Rtk;
	double _cycle_seconds = 0.0;
	std::unique_ptr<agi::PilotParams> _params;
	std::unique_ptr<agi::hardware::HardwarePilot> _pilot;
	double _control_time = 0.0;
	double _previous_clock;
	double _authority_receive_time;
	double _health_receive_time;
	double _output_receive_time;
	bool _output_fault = false;
	uint64_t _sequence = 0;
	msg::FusedState _state;
	msg::Authority _authority;
	msg::Health _health;
	msg::OutputStatus _output;
	rclcpp::Subscription<msg::FusedState>::SharedPtr _state_sub;
	rclcpp::Subscription<msg::Authority>::SharedPtr _authority_sub;
	rclcpp::Subscription<msg::Health>::SharedPtr _health_sub;
	rclcpp::Subscription<msg::OutputStatus>::SharedPtr _output_sub;
	rclcpp::Publisher<msg::ControlCommand>::SharedPtr _command_pub;
	rclcpp::Publisher<msg::ComputationStatus>::SharedPtr _computation_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _reference_pub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _diagnostic_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _status_pub;
	rclcpp::TimerBase::SharedPtr _timer;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_CONTROL_NODE_H_
