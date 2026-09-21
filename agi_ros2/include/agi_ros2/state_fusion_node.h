#ifndef AGI_ROS2_STATE_FUSION_NODE_H_
#define AGI_ROS2_STATE_FUSION_NODE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "agi_ros2/imu_initialization.h"
#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/local_navigation.hpp"
#include "agi_ros2/msg/rtk.hpp"
#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/types/quad_state.hpp"
#include "companion_ahrs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace agi_ros2 {

// A single-threaded executor owns the initialization AHRS and EKF.
class StateFusionNode final : public rclcpp::Node {
public:
	StateFusionNode();

private:
	void onNavigation(msg::LocalNavigation::ConstSharedPtr message);
	void onRtk(msg::Rtk::ConstSharedPtr message);
	void onImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
	void reset();
	bool navigationFresh(double time, double received) const;
	bool covarianceReady(const agi::EkfImu::NavigationQuality& quality) const;
	void publishState(double imu_receive_time);

	const std::string _clock_id;
	bool _timing_checks = true;
	agi::QuadState _state;
	std::unique_ptr<agi::EkfImu> _ekf;
	std::shared_ptr<agi::EkfImuParameters> _ekf_parameters;
	agi::Vector<3> _rtk_position_variance;
	agi::Vector<3> _rtk_velocity_variance;
	double _rtk_heading_variance;
	std::unique_ptr<agi::CompanionAhrs> _ahrs;
	std::unique_ptr<ImuInitialization> _imu_initialization;
	bool _imu_ready = false;
	double _initialization_max_speed = 0.3;
	int _navigation_ready_updates = 30;
	int _accepted_navigation_updates = 0;
	double _navigation_nis_threshold = 24.322;
	double _max_horizontal_position_stddev = 0.0;
	double _max_vertical_position_stddev = 0.0;
	double _max_velocity_stddev = 0.0;
	double _max_heading_stddev = 0.0;
	double _last_imu_time = NAN;
	double _last_imu_receive_time = NAN;
	double _last_navigation_attempt = NAN;
	std::string _readiness_reason = "Waiting for navigation and IMU";
	msg::Authority _authority;
	double _authority_receive_time = NAN;
	rclcpp::Subscription<msg::Authority>::SharedPtr _authority_sub;
	msg::Rtk _rtk;
	msg::LocalNavigation _navigation;
	std::string _navigation_source;
	rclcpp::Subscription<msg::LocalNavigation>::SharedPtr _navigation_sub;
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
