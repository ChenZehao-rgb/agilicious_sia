#include "agi_ros2/state_fusion_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "agi_ros2/node_common.h"
#include "agi_ros2/runtime_config.h"
#include "agilib/bridge/betaflight/betaflight_msp_bridge.hpp"
#include "agilib/bridge/betaflight/hardware_safety.hpp"
#include "agilib/types/imu_sample.hpp"

namespace agi_ros2 {
namespace {
constexpr double kUnknownTime = std::numeric_limits<double>::quiet_NaN();
using agi::hardware::monotonicSeconds;
using agi::hardware::SafetyGate;
}  // namespace

StateFusionNode::StateFusionNode()
        : Node("state_fusion"), _clock_id(readClockId()), _rtk_receive_time(kUnknownTime), _last_rtk_time(kUnknownTime) {
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	descriptor.read_only = true;
	const auto mode = declare_parameter<std::string>("mode", "sitl", descriptor);
	if (mode != "sitl" && mode != "hardware") throw std::invalid_argument("mode must be sitl or hardware");
	if (mode == "hardware" && get_parameter("use_sim_time").as_bool())
		throw std::invalid_argument("Hardware fusion requires use_sim_time=false");
	const auto profile = loadRuntimeConfig(*this, mode);
	const auto configured = [this, &descriptor, &profile](const char* name, auto fallback) {
		using Value = decltype(fallback);
		if (profile) {
			const auto value = profile->section("fusion")[name];
			if (!value.isNull()) {
				if constexpr (std::is_same_v<Value, std::vector<double>>) {
					if (!value.isSequence()) throw std::invalid_argument(name);
					fallback.clear();
					for (size_t i = 0; i < value.size(); ++i)
						fallback.push_back(value[static_cast<int>(i)].as<double>());
				} else {
					fallback = value.as<Value>();
				}
			}
		}
		return declare_parameter<Value>(name, fallback, descriptor);
	};
	const bool delay_test = declare_parameter<bool>(
	        "sitl_delay_test", profile ? profile->section("flight")["sitl_delay_test"].as<bool>() : false, descriptor);
	if (delay_test && (mode != "sitl" || !get_parameter("use_sim_time").as_bool())) {
		throw std::invalid_argument("sitl_delay_test requires mode=sitl and use_sim_time=true");
	}
	_timing_checks = !delay_test;
	const auto positive = [&configured](const char* name, double value) {
		const double result = configured(name, value);
		if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(name);
		return result;
	};
	_rtk_position_variance.setConstant(positive("rtk_position_variance", 0.0004));
	_rtk_velocity_variance.setConstant(positive("rtk_velocity_variance", 0.0025));
	_rtk_heading_variance = positive("rtk_heading_variance", 0.0001);
	_ekf_parameters = std::make_shared<agi::EkfImuParameters>();
	_ekf_parameters->R_acc.setConstant(positive("imu_acceleration_variance", 0.1));
	_ekf_parameters->R_omega.setConstant(positive("imu_angular_velocity_variance", 0.0001));
	const auto variances = [&configured](const char* name, auto& destination, double value) {
		const auto values = configured(name, std::vector<double>(destination.size(), value));
		if (values.size() != static_cast<size_t>(destination.size())) throw std::invalid_argument(name);
		for (size_t i = 0; i < values.size(); ++i) {
			if (!std::isfinite(values[i]) || values[i] < 0) throw std::invalid_argument(name);
			destination(i) = values[i];
		}
	};
	variances("ekf_process_position_variance", _ekf_parameters->Q_pos, 1e-6);
	variances("ekf_process_attitude_variance", _ekf_parameters->Q_att, 1e-6);
	variances("ekf_process_velocity_variance", _ekf_parameters->Q_vel, 1e-3);
	variances("ekf_process_gyro_bias_variance", _ekf_parameters->Q_bome, 1e-9);
	variances("ekf_process_acceleration_bias_variance", _ekf_parameters->Q_bacc, 1e-9);
	variances("ekf_initial_attitude_variance", _ekf_parameters->Q_init_att, 0.01);
	variances("ekf_initial_gyro_bias_variance", _ekf_parameters->Q_init_bome, 0.001);
	variances("ekf_initial_acceleration_bias_variance", _ekf_parameters->Q_init_bacc, 0.01);
	_navigation_source = declare_parameter<std::string>("navigation_source", mode == "hardware" ? "gnss" : "rtk", descriptor);
	if (_navigation_source != "rtk" && _navigation_source != "gnss")
		throw std::invalid_argument("navigation_source must be rtk or gnss");
	ImuInitialization::Params initialization;
	initialization.duration = positive("imu_initialization_duration", 3.0);
	initialization.minimum_samples = configured("imu_initialization_samples", 1000);
	if (initialization.minimum_samples < 2 || initialization.minimum_samples > 10000)
		throw std::invalid_argument("imu_initialization_samples must be between 2 and 10000");
	initialization.max_gyro_bias = positive("imu_max_gyro_bias", 0.15);
	initialization.max_gyro_stddev = positive("imu_max_gyro_stddev", 0.02);
	initialization.max_acceleration_stddev = positive("imu_max_acceleration_stddev", 0.2);
	initialization.gravity_tolerance = positive("imu_gravity_tolerance", 0.5);
	_imu_initialization = std::make_unique<ImuInitialization>(initialization);
	_initialization_max_speed = positive("imu_initialization_max_speed", 0.3);
	_navigation_ready_updates = configured("navigation_ready_updates", 30);
	if (_navigation_ready_updates < 1) throw std::invalid_argument("navigation_ready_updates must be positive");
	_navigation_nis_threshold = positive("navigation_nis_threshold", 24.322);
	const auto limit = [&configured](const char* name) {
		const double value = configured(name, 0.0);
		if (!std::isfinite(value) || value < 0) throw std::invalid_argument(name);
		return value;
	};
	_max_horizontal_position_stddev = limit("max_horizontal_position_stddev");
	_max_vertical_position_stddev = limit("max_vertical_position_stddev");
	_max_velocity_stddev = limit("max_velocity_stddev");
	_max_heading_stddev = limit("max_heading_stddev");
	reset();
	_fused_pub = create_publisher<msg::FusedState>("fused_state", 1);
	_state_pub = create_publisher<nav_msgs::msg::Odometry>("state", 1);
	if (_navigation_source == "rtk") {
		_rtk_sub =
		        create_subscription<msg::Rtk>("sensors/rtk", 10, std::bind(&StateFusionNode::onRtk, this, std::placeholders::_1));
	} else {
		_authority_sub = create_subscription<msg::Authority>("authority", 1, [this](msg::Authority::ConstSharedPtr message) {
			_authority = *message;
			_authority_receive_time = monotonicSeconds();
		});
		_navigation_sub =
		        create_subscription<msg::LocalNavigation>("sensors/local_navigation", rclcpp::SensorDataQoS(),
			                                          std::bind(&StateFusionNode::onNavigation, this, std::placeholders::_1));
	}
	_imu_sub = create_subscription<sensor_msgs::msg::Imu>("sensors/imu", rclcpp::SensorDataQoS().keep_last(256),
	                                                      std::bind(&StateFusionNode::onImu, this, std::placeholders::_1));
}

void StateFusionNode::reset() {
	_ahrs = std::make_unique<agi::CompanionAhrs>(agi::CompanionAhrs::Params{});
	auto params = std::make_shared<agi::EkfImuParameters>(*_ekf_parameters);
	params->Q_init_pos = _rtk_position_variance;
	params->Q_init_vel = _rtk_velocity_variance;
	_ekf = std::make_unique<agi::EkfImu>(params);
	_state.setZero();
	_state.t = kUnknownTime;
	_last_rtk_time = kUnknownTime;
	_last_navigation_attempt = kUnknownTime;
	_last_imu_time = kUnknownTime;
	_imu_ready = false;
	_accepted_navigation_updates = 0;
	_imu_initialization->reset();
	_readiness_reason = "Waiting for stationary IMU initialization";
	++_reset_counter;
}

void StateFusionNode::onRtk(msg::Rtk::ConstSharedPtr message) {
	_rtk = *message;
	_rtk_receive_time = monotonicSeconds();
}

void StateFusionNode::onNavigation(msg::LocalNavigation::ConstSharedPtr message) {
	if (message->header.frame_id != "odom" || message->session_id.empty() || message->source_session.empty()) return;
	const bool changed = _navigation.session_id != message->session_id || _navigation.source_session != message->source_session;
	if (!changed && stampSeconds(message->header.stamp) <= stampSeconds(_navigation.header.stamp)) return;
	if (!message->observation_valid || !message->clock_aligned || message->fix_type < 3 || message->fix_type > 6 ||
	    !message->heading_valid || !std::isfinite(message->heading)) {
		const double previous_imu_time = _state.t;
		if (changed) {
			reset();
			_state.t = previous_imu_time;
		}
		_navigation = *message;
		_rtk.heading_valid = false;
		_rtk.accuracy_ok = false;
		_rtk_receive_time = kUnknownTime;
		_accepted_navigation_updates = 0;
		_readiness_reason = message->reason.empty() ? "Navigation source explicitly revoked its solution" : message->reason;
		// Do not refresh IMU or last accepted navigation acquisition times.
		publishState(_last_imu_receive_time);
		return;
	}
	if (message->altitude_reference != "ellipsoid" && message->altitude_reference != "msl") return;
	for (double variance : message->position_variance)
		if (!std::isfinite(variance) || variance <= 0) return;
	for (double variance : message->velocity_variance)
		if (!std::isfinite(variance) || variance <= 0) return;
	if (!std::isfinite(message->heading_variance) || message->heading_variance <= 0) return;
	if (!std::isfinite(message->position.x) || !std::isfinite(message->position.y) || !std::isfinite(message->position.z) ||
	    !std::isfinite(message->velocity.x) || !std::isfinite(message->velocity.y) || !std::isfinite(message->velocity.z) ||
	    !message->heading_valid || !std::isfinite(message->heading))
		return;
	const bool initialize_parameters = !_navigation.observation_valid && !_ekf->healthy();
	_navigation = *message;
	for (int i = 0; i < 3; ++i) {
		_rtk_position_variance(i) = message->position_variance[i];
		_rtk_velocity_variance(i) = message->velocity_variance[i];
	}
	_rtk_heading_variance = message->heading_variance;
	if (changed || initialize_parameters) {
		_rtk = msg::Rtk();
		reset();
	}
	// Internal observation storage only: never manufacture fixed/PPS evidence.
	_rtk.header = message->header;
	_rtk.position = message->position;
	_rtk.velocity = message->velocity;
	_rtk.heading = message->heading;
	_rtk.heading_valid = message->heading_valid;
	_rtk.accuracy_ok = message->accuracy_known && message->accuracy_ok && std::isfinite(message->horizontal_accuracy) &&
	                   message->horizontal_accuracy > 0 && std::isfinite(message->vertical_accuracy) &&
	                   message->vertical_accuracy > 0 && std::isfinite(message->velocity_accuracy) && message->velocity_accuracy > 0;
	_rtk_receive_time = monotonicSeconds();
}

bool StateFusionNode::navigationFresh(double time, double received) const {
	const double fix_time = stampSeconds(_rtk.header.stamp);
	// IMU and navigation travel on different DDS streams. Hold a slightly newer
	// navigation sample until IMU coverage reaches it, without revoking the last update.
	const bool aligned = SafetyGate::fresh(time, fix_time, 0.3, _timing_checks) ||
	                     (std::isfinite(time) && fix_time > time && fix_time - time <= 0.010);
	return (get_parameter("use_sim_time").as_bool() || SafetyGate::fresh(received, _rtk_receive_time, 0.3)) && aligned &&
	       _rtk.header.frame_id == "odom" &&
	       (_navigation_source == "gnss" ? _navigation.clock_aligned : (_rtk.fixed && _rtk.accuracy_ok));
}

bool StateFusionNode::covarianceReady(const agi::EkfImu::NavigationQuality& quality) const {
	return quality.valid && _max_horizontal_position_stddev > 0 && _max_vertical_position_stddev > 0 && _max_velocity_stddev > 0 &&
	       _max_heading_stddev > 0 && quality.position_variance.head<2>().maxCoeff() <= std::pow(_max_horizontal_position_stddev, 2) &&
	       quality.position_variance.z() <= std::pow(_max_vertical_position_stddev, 2) &&
	       quality.velocity_variance.maxCoeff() <= std::pow(_max_velocity_stddev, 2) &&
	       quality.heading_variance <= std::pow(_max_heading_stddev, 2);
}

void StateFusionNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
	const double received = monotonicSeconds();
	const double time = stampSeconds(message->header.stamp);
	if (message->header.frame_id != "base_link") return;
	const agi::ImuSample imu(time, {message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z},
	                         {message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z});
	if (!imu.valid()) return;
	if (std::isfinite(_last_imu_time)) {
		if (time == _last_imu_time) return;
		if (time < _last_imu_time || (_timing_checks && time - _last_imu_time > 0.025)) {
			if (time < _last_imu_time) {
				_rtk = msg::Rtk();
				_rtk_receive_time = kUnknownTime;
			}
			reset();
		}
	}
	_last_imu_time = time;
	_last_imu_receive_time = received;
	const double fix_time = stampSeconds(_rtk.header.stamp);
	const bool navigation_fresh = navigationFresh(time, received);
	const bool valid_fix =
	        navigation_fresh && fix_time <= time && (!std::isfinite(_last_navigation_attempt) || fix_time > _last_navigation_attempt);
	const agi::Vector<3> position(_rtk.position.x, _rtk.position.y, _rtk.position.z);
	const agi::Vector<3> velocity(_rtk.velocity.x, _rtk.velocity.y, _rtk.velocity.z);
	const bool heading_valid = _rtk.heading_valid && std::isfinite(_rtk.heading);
	if (!navigation_fresh || (_navigation_source == "gnss" && !_rtk.accuracy_ok)) _accepted_navigation_updates = 0;

	if (!_ekf->healthy()) {
		_state.t = time;
		if (_navigation_source == "gnss") {
			const bool disarmed = SafetyGate::fresh(received, _authority_receive_time, 0.1) &&
			                      SafetyGate::fresh(now().seconds(), stampSeconds(_authority.header.stamp), 0.1) &&
			                      _authority.rc_link && !_authority.armed;
			if (!disarmed || !navigation_fresh || !heading_valid || !position.allFinite() || !velocity.allFinite() ||
			    velocity.norm() > _initialization_max_speed) {
				_imu_initialization->reset();
				_readiness_reason = "Initialization requires disarmed authority and stationary, fresh navigation";
				publishState(received);
				return;
			}
			if (!_imu_initialization->addSample(imu) || !valid_fix) {
				_readiness_reason = "Collecting stationary IMU samples; checking gravity, gyro bias and noise";
				publishState(received);
				return;
			}
			_ahrs->setHeading(_rtk.heading);
			_ahrs->addImu({time, _imu_initialization->acceleration(), _imu_initialization->gyroBias()});
		} else {
			// Preserve the existing SITL/RTK initialization contract.
			if (valid_fix && heading_valid) _ahrs->setHeading(_rtk.heading);
			_ahrs->addImu(imu);
		}
		if (!valid_fix || !heading_valid || !position.allFinite() || !velocity.allFinite() || !_ahrs->initialized()) {
			_readiness_reason = "Waiting for valid navigation and initial attitude";
			publishState(received);
			return;
		}
		_state.p = position + velocity * (time - fix_time);
		_state.v = velocity;
		_state.q(_ahrs->attitude());
		_state.q(agi::Quaternion(Eigen::AngleAxis<agi::Scalar>(_rtk.heading - _state.getYaw(), agi::Vector<3>::UnitZ())) *
		         _state.q());
		_state.bw = _navigation_source == "gnss" ? _imu_initialization->gyroBias() : _ahrs->gyroBias();
		if (!_ekf->initialize(_state) || !_ekf->addImu(imu)) {
			reset();
			_state.t = time;
			_readiness_reason = "EKF initialization failed";
			publishState(received);
			return;
		}
		_imu_ready = true;
		_last_rtk_time = fix_time;
		_last_navigation_attempt = fix_time;
	} else {
		if (!_ekf->addImu(imu)) {
			reset();
			_state.t = time;
			_readiness_reason = "EKF rejected IMU; reinitialization required";
			publishState(received);
			return;
		}
		if (valid_fix) {
			_last_navigation_attempt = fix_time;
			const double innovation_limit =
			        _navigation_source == "gnss" ? _navigation_nis_threshold : std::numeric_limits<double>::infinity();
			if (_ekf->addRtk(fix_time, position, velocity, _rtk.heading, heading_valid, _rtk_position_variance,
			                 _rtk_velocity_variance, _rtk_heading_variance, innovation_limit)) {
				_last_rtk_time = fix_time;
				if (_rtk.accuracy_ok)
					_accepted_navigation_updates =
					        std::min(_accepted_navigation_updates + 1, _navigation_ready_updates);
				_readiness_reason = "Waiting for consecutive accepted navigation updates and covariance limits";
			} else {
				_accepted_navigation_updates = 0;
				_readiness_reason = "Navigation update rejected by EKF timing, innovation or numerical checks";
			}
		}
	}
	if (!_ekf->getAt(time, &_state) || !_state.valid()) {
		reset();
		_state.t = time;
		_readiness_reason = "EKF prediction failed; reinitialization required";
	}
	publishState(received);
}

void StateFusionNode::publishState(double imu_receive_time) {
	msg::FusedState out;
	out.header.stamp = rosStamp(std::isfinite(_state.t) ? _state.t : now().seconds());
	out.header.frame_id = "odom";
	out.position.x = _state.p.x();
	out.position.y = _state.p.y();
	out.position.z = _state.p.z();
	out.velocity.x = _state.v.x();
	out.velocity.y = _state.v.y();
	out.velocity.z = _state.v.z();
	out.orientation.w = _state.q().w();
	out.orientation.x = _state.q().x();
	out.orientation.y = _state.q().y();
	out.orientation.z = _state.q().z();
	out.body_rates.x = _state.w.x();
	out.body_rates.y = _state.w.y();
	out.body_rates.z = _state.w.z();
	out.acceleration.x = _state.a.x();
	out.acceleration.y = _state.a.y();
	out.acceleration.z = _state.a.z();
	out.clock_id = _clock_id;
	out.published_steady_time = monotonicSeconds();
	out.imu_receive_time = imu_receive_time;
	out.rtk_receive_time = _rtk_receive_time;
	if (std::isfinite(_last_rtk_time)) {
		out.rtk_stamp = rosStamp(_last_rtk_time);
	}
	out.navigation_source = _navigation_source;
	out.navigation_session = _navigation.session_id + ":" + _navigation.source_session;
	out.fix_type = _navigation_source == "gnss" ? _navigation.fix_type : (_rtk.fixed ? 6 : 0);
	out.clock_aligned = _navigation_source == "gnss" ? _navigation.clock_aligned : _rtk.synchronized;
	out.accuracy_known = _navigation_source == "gnss" ? _navigation.accuracy_known : _rtk.accuracy_ok;
	out.navigation_valid = std::isfinite(_last_rtk_time) && SafetyGate::fresh(_state.t, _last_rtk_time, .3, _timing_checks) &&
	                       navigationFresh(_state.t, imu_receive_time);
	out.reset_counter = _reset_counter;
	out.initialized = _ekf->healthy() && _state.valid() && std::isfinite(_last_rtk_time);
	out.rtk_fixed = _rtk.fixed;
	out.heading_valid = _rtk.heading_valid;
	out.accuracy_ok = _rtk.accuracy_ok;
	out.synchronized = _rtk.synchronized;
	out.imu_ready = _imu_ready;
	out.navigation_accuracy_ok = _rtk.accuracy_ok;
	out.navigation_ready = out.navigation_valid && out.heading_valid && out.navigation_accuracy_ok && out.clock_aligned &&
	                       _last_navigation_attempt == _last_rtk_time;
	const auto quality = _ekf->navigationQuality();
	if (std::isfinite(quality.stamp)) out.covariance_stamp = rosStamp(quality.stamp);
	for (int i = 0; i < 3; ++i) {
		out.position_variance[i] = quality.position_variance(i);
		out.velocity_variance[i] = quality.velocity_variance(i);
	}
	out.heading_variance = quality.heading_variance;
	out.navigation_innovation_squared = quality.innovation_squared;
	out.navigation_rejections = quality.rejected_updates;
	out.navigation_accepted_updates = _accepted_navigation_updates;
	out.estimator_ready =
	        out.initialized && out.imu_ready && out.navigation_ready &&
	        (_navigation_source == "rtk" || (_accepted_navigation_updates >= _navigation_ready_updates && covarianceReady(quality)));
	out.readiness_reason = _readiness_reason;
	if (out.initialized) {
		if (_navigation_source == "gnss" && !_navigation.observation_valid) {
			out.readiness_reason = _readiness_reason;
		} else if (!out.navigation_valid) {
			out.readiness_reason = "Navigation unavailable, stale or clock alignment lost";
		} else if (!out.navigation_accuracy_ok) {
			out.readiness_reason = "GNSS accuracy unknown, above limits, or flight accuracy limits unconfigured";
		} else if (out.navigation_ready && _navigation_source == "gnss" && !covarianceReady(quality)) {
			out.readiness_reason = "Estimator covariance invalid, above limits, or flight covariance limits unconfigured";
		} else if (out.estimator_ready) {
			out.readiness_reason = "Navigation and estimator satisfy configured readiness checks";
		}
	}
	_fused_pub->publish(out);
	if (!out.initialized) return;

	// Keep the established evaluation topic, with standard body-frame twist.
	nav_msgs::msg::Odometry odometry;
	odometry.header = out.header;
	odometry.child_frame_id = "base_link";
	odometry.pose.pose.position = out.position;
	odometry.pose.pose.orientation = out.orientation;
	const auto velocity_body = _state.q().conjugate() * _state.v;
	odometry.twist.twist.linear.x = velocity_body.x();
	odometry.twist.twist.linear.y = velocity_body.y();
	odometry.twist.twist.linear.z = velocity_body.z();
	odometry.twist.twist.angular = out.body_rates;
	_state_pub->publish(odometry);
}

}  // namespace agi_ros2
