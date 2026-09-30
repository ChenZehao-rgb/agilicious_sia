#ifndef AGI_ROS2_STATE_FUSION_NODE_H_
#define AGI_ROS2_STATE_FUSION_NODE_H_

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "agi_ros2/barometer_reference.h"
#include "agi_ros2/imu_initialization.h"
#include "agi_ros2/msg/authority.hpp"
#include "agi_ros2/msg/barometer.hpp"
#include "agi_ros2/msg/fused_state.hpp"
#include "agi_ros2/msg/local_navigation.hpp"
#include "agi_ros2/msg/rtk.hpp"
#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/types/quad_state.hpp"
#include "companion_ahrs.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace agi_ros2 {

// A single-threaded executor owns the initialization AHRS and EKF.
class StateFusionNode final : public rclcpp::Node {
public:
	StateFusionNode();

private:
	struct Observation {
		double time = NAN;
		bool barometer = false;
		msg::Rtk navigation;
		agi::Vector<3> position_variance = agi::Vector<3>::Zero();
		agi::Vector<3> velocity_variance = agi::Vector<3>::Zero();
		double heading_variance = NAN;
		double pressure = NAN;
		double pressure_variance = NAN;
	};

	void onNavigation(msg::LocalNavigation::ConstSharedPtr message);
	void onRtk(msg::Rtk::ConstSharedPtr message);
	void onBarometer(msg::Barometer::ConstSharedPtr message);
	void onImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
	void enqueueObservation(const Observation& observation);
	void enqueueNavigation(const msg::Rtk& navigation);
	void processObservations(double time, double received);
	void processBarometer(const Observation& observation, double received);
	void resetBarometer(const std::string& reason);
	void publishBarometerStatus();
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
	bool _baro_enabled = false;
	double _observation_delay = 0.0;
	double _observation_watermark = NAN;
	std::deque<Observation> _observations;
	std::deque<agi::ImuSample> _reference_imus;
	double _reference_gravity_tolerance = 0.5;
	double _reference_max_angular_speed = 0.15;
	uint64_t _late_navigation = 0;
	uint64_t _late_barometer = 0;
	uint64_t _observation_overflows = 0;
	double _observation_processing_seconds = 0.0;
	double _baro_max_age = 0.25;
	double _baro_pressure_variance = 4.0;
	double _baro_min_pressure = 30000.0;
	double _baro_max_pressure = 120000.0;
	double _baro_model_variance = 0.25;
	double _baro_nis_threshold = 10.828;
	double _baro_last_time = NAN;
	double _baro_receive_time = NAN;
	double _baro_last_height = NAN;
	double _baro_last_height_variance = NAN;
	double _baro_reference_bias_variance = NAN;
	std::string _baro_session;
	std::string _baro_reason = "Barometer disabled";
	uint64_t _baro_reference_resets = 0;
	uint64_t _baro_invalid_samples = 0;
	std::unique_ptr<BarometerReference> _baro_reference;
	rclcpp::Subscription<msg::Barometer>::SharedPtr _baro_sub;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _baro_status_pub;
	rclcpp::TimerBase::SharedPtr _baro_status_timer;
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
