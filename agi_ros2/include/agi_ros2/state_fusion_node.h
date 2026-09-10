#ifndef AGI_ROS2_STATE_FUSION_NODE_H_
#define AGI_ROS2_STATE_FUSION_NODE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/rtk.hpp"
#include "agilib/types/quad_state.hpp"
#include "companion_ahrs.hpp"
#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace agi_ros2 {

// A single-threaded executor owns the initialization AHRS and EKF.
class StateFusionNode final : public rclcpp::Node {
public:
	StateFusionNode();

private:
	void onRtk(msg::Rtk::ConstSharedPtr message);
	void onImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
	void reset();
	void publishState(double imu_receive_time);

	const std::string _clock_id;
	bool _timing_checks = true;
	agi::QuadState _state;
	std::unique_ptr<agi::EkfImu> _ekf;
	agi::Vector<3> _rtk_position_variance;
	agi::Vector<3> _rtk_velocity_variance;
	double _rtk_heading_variance;
	std::unique_ptr<agi::CompanionAhrs> _ahrs;
	msg::Rtk _rtk;
	double _rtk_receive_time;
	double _last_rtk_time;
	uint64_t _reset_counter = 0;
	rclcpp::Subscription<msg::Rtk>::SharedPtr _rtk_sub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
	rclcpp::Publisher<msg::FusedState>::SharedPtr _fused_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _state_pub;
};

}  // namespace agi_ros2

#endif  // AGI_ROS2_STATE_FUSION_NODE_H_
