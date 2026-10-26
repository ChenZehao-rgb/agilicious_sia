#ifndef AGI_ROS2_FC_STATE_ADAPTER_NODE_H_
#define AGI_ROS2_FC_STATE_ADAPTER_NODE_H_
#include <cmath>
#include <cstdint>
#include <string>

#include "agi_ros2/msg/fc_estimated_state.hpp"
#include "agi_ros2/msg/fc_estimator_status.hpp"
#include "agi_ros2/msg/fused_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
namespace agi_ros2 {
// Read-only format/validity adapter: no integration, prediction or fusion.
class FcStateAdapterNode final : public rclcpp::Node {
public:
	FcStateAdapterNode();

private:
	void onState(msg::FcEstimatedState::ConstSharedPtr state);
	void onHealth(msg::FcEstimatorStatus::ConstSharedPtr status);
	void invalidate(const std::string& reason);
	void watchdog();
	const std::string _clock_id;
	std::string _session;
	uint64_t _generation{1}, _last_time{0};
	uint32_t _reset{0};
	uint64_t _receiver_epoch{0};
	bool _have_reset{false};
	double _ros_offset{NAN};
	msg::FcEstimatorStatus _health;
	msg::FusedState _output;
	rclcpp::Publisher<msg::FusedState>::SharedPtr _fused_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
	rclcpp::Subscription<msg::FcEstimatedState>::SharedPtr _state_sub;
	rclcpp::Subscription<msg::FcEstimatorStatus>::SharedPtr _health_sub;
	rclcpp::TimerBase::SharedPtr _timer;
};
}  // namespace agi_ros2
#endif
